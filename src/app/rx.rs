//! Video Input receiver state machine.
//!
//! The receiver advances once per main-loop tick (see [`Rx::tick`]). One tick
//! runs two steps, in order:
//!
//! * [`Rx::state_handler`] — forward progress: the work done *in* the current
//!   state, and the transition to the next state once that work completes.
//! * [`Rx::state_judge`]   — regressions: the 5V source-detect edge plus the
//!   hardware events (clock loss, PLL unlock, colour-space change…) that
//!   demote the FSM back to an earlier state so it re-runs the affected
//!   bring-up steps. The source-detect edge is applied first, so a
//!   plug/unplug always wins over the other rewind checks.
//!
//! HPD is owned entirely by this state machine: it is raised only on the
//! [`RxState::WaitSource`] → [`RxState::ClkStableCheck`] transition and
//! dropped — always through [`Rx::disconnect`], which also disables the DDC
//! slave and arms the HPD-hold timer — on every path that returns the FSM to
//! an idle state. Because the state is always known, each HPD change is made
//! exactly where it belongs — there is no separate HPD tracking or invariant
//! pass.
//!
//! `tick` reports the receiver's situation to the rest of the daemon as an
//! [`RxStatus`] — a snapshot of *levels* (is a source plugged? is video live?
//! is audio live?), not a stream of edge events. The transmitter and the sink
//! reporter each read those levels and react; no edge bookkeeping crosses the
//! module boundary.
//!
//! All receiver bookkeeping (sticky interrupt flags, debounce counters, the
//! shared per-state timer, the audio acquisition) lives on [`Rx`]; the driver
//! layer stays stateless and the transmitter is never referenced from here.

use std::time::Duration;

use crate::app::audio::Audio;
use crate::app::health::Health;
use crate::config::*;
use crate::drv::audio::Rate;
use crate::drv::{
    audio as drv_audio, auto_eq, edid, hddvi_rx as drv_rx, system as drv_sys,
    video_scrambler_rx as drv_vscr,
};
use crate::format::{MipiFormat, PixelEncoding};
use crate::reg;
use crate::stub::Failure;
use crate::time;
use crate::time::Timer;
use crate::timing::Timing;

/// Receiver state. Every variant either does one-shot work and moves on, or
/// waits for a specific condition — there are no pure pass-through states.
///
/// Variants are in bring-up order so the derived `Ord` lets [`Rx::state_judge`]
/// rank how far a regression must rewind.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, Default)]
pub enum RxState {
    /// One-shot driver/scrambler/interrupt bring-up.
    #[default]
    PowerOn,
    /// Idle until 5V is present and a kernel EDID is loaded; then raise HPD.
    WaitSource,
    /// Wait for the upstream TMDS clock to debounce stable.
    ClkStableCheck,
    /// One-shot PHY reset, RxPLL calibration and equaliser training.
    PllConfig,
    /// Wait for a stable, repeating AVI InfoFrame.
    PktRdy,
    /// Wait for the video-check timing measurement to settle.
    VideoCheck,
    /// Video (and audio) live; the audio acquisition runs here.
    PlayBack,
}

/// Inputs the receiver reads from the rest of the daemon each tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct RxEnv {
    /// The current kernel EDID, or `None` when the kernel has not supplied a
    /// usable one. The receiver programs it into the chip itself when it
    /// brings the link up — see [`RxState::WaitSource`].
    pub kernel_edid: Option<edid::Edid>,
    /// Newest measurement of the 5V source-detect input.
    pub source_5v: Level,
}

/// The receiver's situation after a tick, expressed as continuously-valid
/// *levels* rather than edge events. `App` feeds these to the transmitter and
/// the sink reporter, both of which react to the level — not to a transition.
///
/// There is no `source_plugged` level: the whole receiver pipeline requires
/// 5V, so a live `video` already implies a plugged source, and nothing
/// downstream acts on "plugged but no video" (it is the same idle wait as
/// "unplugged" — see the `Tx` state machine).
#[derive(Clone, Copy, Debug, Default)]
pub struct RxStatus {
    /// The receiver is delivering live video — in [`RxState::PlayBack`].
    pub video: bool,
    /// The receiver is delivering live audio at this sample rate: the I2S
    /// clock runs and has been verified against the rate.
    pub audio: Option<Rate>,
    /// The video-check timing latched when the receiver last passed
    /// [`RxState::VideoCheck`]. Measured solely within Rx; the transmitter
    /// consumes it through [`crate::app::tx::TxEnv`]. Stays at its default
    /// until the first successful video check.
    pub timing: Timing,
}

/// Three-value debounce counter: consecutive good/bad readings only flip the
/// verdict once one side crosses its threshold.
#[derive(Clone, Copy, Debug, Default)]
pub struct Debounce {
    good: u8,
    bad: u8,
}

impl Debounce {
    pub fn bump_good(&mut self, stable_threshold: u8) -> Stability {
        self.bad = 0;
        self.good = self.good.saturating_add(1);
        if self.good >= stable_threshold {
            self.good = 0;
            Stability::Stable
        } else {
            Stability::Pending
        }
    }

    pub fn bump_bad(&mut self, unstable_threshold: u8) -> Stability {
        self.good = 0;
        self.bad = self.bad.saturating_add(1);
        if self.bad >= unstable_threshold {
            self.bad = 0;
            Stability::Unstable
        } else {
            Stability::Pending
        }
    }
}

// ---- Rx ----

/// Video Input receiver context. Holds everything the receiver FSM needs.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rx {
    pub state: RxState,
    /// One-shot "the FSM just entered this state" flag; consumed by [`Rx::entered`].
    pub state_changed: bool,

    /// 5V source-detect line as last observed by the FSM.
    pub source_det: Level,
    /// The source sends no AVI InfoFrame (DVI signalling); latched in
    /// [`RxState::PktRdy`].
    pub dvi: bool,
    /// Failure streak, lock count and measurements; see [`Health`].
    pub health: Health,

    pub vid_chk_scan: bool,

    pub interlace: bool,
    pub pixel_encoding: PixelEncoding,
    pub format: MipiFormat,

    /// Latched one-shot: set when the video timing settles, consumed (and
    /// cleared) by [`crate::app::App::video_format_handler`] to resolve the native format once.
    pub format_pending: bool,

    /// Scrambler 1.4 capability (probed at boot by
    /// [`crate::app::video_scrambler_chip`]).
    pub scrambler14_ver: bool,

    pub clk_stb_dbnc: Debounce,
    pub vid_chk_dbnc: Debounce,
    pub avi_rcv_dbnc: Debounce,

    /// Last scrambler receiver status word (used to de-duplicate log lines).
    pub scrambler_status_last: u8,

    /// Shared per-state timer; restarted on each transition that needs it.
    pub timer: Timer,
    /// HPD-hold timer. Started by [`Rx::disconnect`] on every path that drops
    /// HPD, so that [`RxState::WaitSource`] keeps HPD low (with the DDC slave
    /// off) for at least [`HPD_HOLD`] before re-raising it. That is what
    /// guarantees the source has abandoned any in-flight EDID read before the
    /// EDID SRAM is rewritten on the next connect.
    pub hpd_hold: Timer,
    /// Consecutive mismatching AVI packets seen in [`RxState::PktRdy`].
    pub avi_packet_cnt: u8,

    /// Video-check timing, latched on each successful pass through
    /// [`RxState::VideoCheck`]. The receiver is the only owner of this
    /// measurement; it is surfaced to the transmitter via [`RxStatus::timing`].
    pub timing: Timing,

    /// Audio acquisition, ticked only in [`RxState::PlayBack`].
    audio: Audio,
    /// The live audio rate from the last tick, surfaced as [`RxStatus::audio`].
    audio_live: Option<Rate>,
}

/// Minimum time HPD stays low, with the DDC slave off, before
/// [`RxState::WaitSource`] reconnects. Video Input sources must treat an HPD low of
/// 100 ms or more as a disconnect; 500 ms leaves margin for the source to
/// abort an EDID read in progress and to notice the replug afterwards.
const HPD_HOLD: Duration = Duration::from_millis(500);

impl Rx {
    /// One-shot bring-up.
    pub fn power_on_init(&mut self) {
        *self = Rx {
            scrambler14_ver: self.scrambler14_ver,
            ..Default::default()
        };
        self.disconnect();
        drv_rx::phy_shut();
    }

    /// Present a disconnect to the upstream source: drop HPD, disable the DDC
    /// slave, and arm the HPD-hold timer so [`RxState::WaitSource`] does not
    /// reconnect for at least [`HPD_HOLD`].
    ///
    /// This is the *only* way HPD goes low. Every path that later reaches the
    /// WaitSource connect — power-on, 5V unplug, kernel EDID change — goes
    /// through here, so the EDID SRAM is never rewritten while a source may
    /// still be inside a DDC transaction. Without the hold, a 5V flap or a
    /// daemon restart drops HPD for only a few milliseconds, which sources
    /// don't register as an unplug.
    pub fn disconnect(&mut self) {
        drv_rx::hpd_set(Level::Low);
        edid::ddc_slave_set(false);
        self.hpd_hold.start();
    }

    /// Handle a kernel-side EDID change.
    ///
    /// Disconnects and rewinds the FSM to [`RxState::WaitSource`]; the hold
    /// gives the upstream source a clean unplug/replug so it re-reads the new
    /// EDID, which reaches the receiver through the next tick's [`RxEnv`].
    pub fn reload_edid(&mut self) {
        self.disconnect();
        self.set_state(RxState::WaitSource);
    }

    /// Advance the state machine by one tick and report the resulting levels.
    pub fn tick(&mut self, env: RxEnv) -> RxStatus {
        self.state_handler(env);
        self.state_judge(env);
        self.status()
    }

    /// Snapshot the receiver's current output levels.
    fn status(&self) -> RxStatus {
        let in_playback = self.state == RxState::PlayBack;
        RxStatus {
            video: in_playback,
            audio: self.audio_live,
            timing: self.timing,
        }
    }

    // ---- transitions ----

    fn set_state(&mut self, new: RxState) {
        if self.state == new {
            return;
        }
        let last = std::mem::replace(&mut self.state, new);
        self.state_changed = true;
        log::log!(
            self.health.progress_level(),
            "Receiver state: {:?} -> {:?}",
            last,
            new
        );

        if new == RxState::PlayBack {
            self.health.locked();
        }
        if new <= RxState::WaitSource {
            // A new connection, or a new EDID: the next attempt starts over.
            self.health.reconnected();
        }
        if last == RxState::PlayBack {
            self.audio.stop();
            self.audio_live = None;
        }
    }

    /// Consume the one-shot "just entered this state" flag.
    fn entered(&mut self) -> bool {
        std::mem::take(&mut self.state_changed)
    }

    // ---- forward progress ----

    fn state_handler(&mut self, env: RxEnv) {
        match self.state {
            RxState::PowerOn => {
                drv_rx::phy_shut();
                self.disconnect();
                drv_rx::setting_init();
                drv_audio::init();
                self.scrambler_init();
                self.timer.start();
                self.set_state(RxState::WaitSource);
            }
            RxState::WaitSource => {
                if let Some(kernel_edid) = env.kernel_edid {
                    // `hpd_hold` was armed by `disconnect()` on whichever
                    // path brought us here; wait it out before rewriting the
                    // EDID SRAM and re-raising HPD.
                    if self.source_det.is_high() && self.hpd_hold.due(HPD_HOLD) {
                        log::debug!("Connecting receiver to source");
                        edid::edid_set(&kernel_edid);
                        edid::ddc_slave_set(true);
                        drv_rx::hpd_set(Level::High);
                        self.set_state(RxState::ClkStableCheck);
                    }
                }
            }
            RxState::ClkStableCheck => {
                if self.entered() {
                    self.timer.start();
                    drv_vscr::dec_sw_rst();
                }
                if self.timer.due(Duration::from_millis(10)) {
                    if drv_rx::chk_clk_stb(self) == Stability::Stable {
                        self.set_state(RxState::PllConfig);
                        return;
                    }
                    self.timer.start();
                }
            }
            RxState::PllConfig => {
                auto_eq::phug_set();
                drv_rx::phy_soft_reset();
                if SCRAMBLER_RX_MODE != SCRAMBLER_NOT {
                    drv_vscr::re_auth_clear();
                }
                if let Err(e) = drv_rx::pll_cali() {
                    self.health.fail(Failure::LinkSetup, e);
                    return;
                }
                log::debug!("Receiver equalization started");
                if auto_eq::eq_set() {
                    log::debug!("Receiver equalization completed");
                    self.set_state(RxState::PktRdy);
                } else {
                    self.health
                        .fail(Failure::LinkSetup, "receiver channel equalization failed");
                }
            }
            RxState::PktRdy => {
                if self.entered() {
                    drv_rx::avi_rcv_set();
                    drv_rx::depacket_rst();
                    self.timer.start();
                }
                if self.timer.due(Duration::from_millis(10)) {
                    match drv_rx::chk_avi_rcv(self) {
                        Stability::Stable => {
                            drv_rx::avi_gcp_chg_int_en();
                            self.dvi = drv_sys::pkt_header_empty();
                            self.avi_packet_cnt = 0;
                            self.set_state(RxState::VideoCheck);
                            return;
                        }
                        Stability::Unstable => {
                            log::debug!("AVI packet mismatch");
                            self.avi_packet_cnt += 1;
                            if self.avi_packet_cnt > 2 {
                                self.health.fail(
                                    Failure::Infoframe,
                                    "AVI InfoFrame changed 3 times; resetting the receiver",
                                );
                                self.avi_packet_cnt = 0;
                                self.set_state(RxState::PowerOn);
                                return;
                            }
                        }
                        Stability::Pending => {}
                    }
                    self.timer.start();
                }
            }
            RxState::VideoCheck => {
                if self.entered() {
                    let (pe, _, fmt, _) = drv_rx::format_update(self.format);
                    self.pixel_encoding = pe;
                    self.format = fmt;
                    log::debug!("Receiver input format: {}", fmt);
                    drv_rx::half_pix_pll_set();
                    self.vid_chk_scan = false;
                    self.timer.start();
                }
                if self.timer.due(Duration::from_millis(10)) {
                    match self.vid_chk_deal() {
                        Stability::Stable => {
                            // Latch the timing — the receiver is its sole
                            // owner — and log it.
                            self.timing = drv_sys::video_check_all_get();
                            let t = self.timing;
                            let frame_rate_mhz = drv_sys::frame_rate_mhz();
                            log::info!(
                                "Receiver locked: {}x{} at {:.2} Hz, pixel clock {:.2} MHz, total {}x{}, {}, {}",
                                t.h_active,
                                t.v_active,
                                frame_rate_mhz as f64 / 1000.0,
                                t.pixel_clock as f64 / 1000.0,
                                t.h_total,
                                t.v_total,
                                self.format,
                                if self.dvi { "DVI" } else { "HDMI" }
                            );
                            log::debug!("Receiver timing: {:?}", t);
                            self.format_pending = true;
                            self.vid_chk_scan = true;
                            self.set_state(RxState::PlayBack);
                            return;
                        }
                        Stability::Unstable => {
                            self.fail_video_check();
                            self.set_state(RxState::ClkStableCheck);
                            return;
                        }
                        Stability::Pending => {}
                    }
                    self.timer.start();
                }
            }
            RxState::PlayBack => {
                self.audio_live = self.audio.tick();
            }
        }
    }

    // ---- regressions ----

    /// Apply the 5V source-detect edge, then watch for hardware events that
    /// invalidate the current state and rewind the FSM to the state whose
    /// bring-up must be redone.
    ///
    /// The source-detect edge is handled first: an unplug demotes straight to
    /// [`RxState::WaitSource`] and drops HPD, so it always wins — the rewind
    /// checks below then early-return on the now-low `source_det`.
    ///
    /// Each rewind check guards on the *live* `self.state`, and the checks run
    /// in ascending target order. So the first rewind that fires drops the
    /// state low enough that every later (higher-target) check's guard stops
    /// matching — co-occurring faults resolve to the earliest affected state
    /// on their own, with no minimum to accumulate.
    fn state_judge(&mut self, env: RxEnv) {
        match (self.source_det.is_high(), env.source_5v.is_high()) {
            (false, true) => {
                reg::set_bit(0xE82C, 0);
                log::info!("Source connected");
                self.source_det = Level::High;
            }
            (true, false) => {
                reg::clear_bit(0xE105, 2);
                reg::set_bit(0xE105, 2);
                reg::clear_bit(0xE82C, 0);
                log::info!("Source disconnected");
                self.source_det = Level::Low;
                // Present the disconnect now no source is present, and demote
                // to WaitSource — which drops the `video` level if we were
                // playing.
                self.disconnect();
                self.set_state(RxState::WaitSource);
            }
            _ => {}
        }

        if !self.source_det.is_high() {
            return;
        }

        // ---- rewinds to ClkStableCheck ----
        if self.state > RxState::ClkStableCheck && !drv_rx::clk_stable() {
            reg::clear_bit(0xE10A, 7);
            self.health.fail(
                Failure::ClockLost,
                format_args!("TMDS clock unstable in {:?}", self.state),
            );
            self.set_state(RxState::ClkStableCheck);
        }
        if self.state > RxState::VideoCheck && !drv_rx::video_stable() && !self.vid_chk_scan {
            reg::clear_bit(0xE10A, 7);
            self.health.fail(
                Failure::SyncLost,
                "video-check stable flag dropped during playback",
            );
            self.set_state(RxState::ClkStableCheck);
        }
        // ---- rewind to PllConfig ----
        if self.state > RxState::PllConfig && !drv_rx::pll_locked() {
            reg::clear_bit(0xE10A, 7);
            self.health.fail(
                Failure::ClockLost,
                format_args!(
                    "receiver PLL lost lock in {:?} (Register E5F1=0x{:02X}); recalibrating",
                    self.state,
                    reg::read(0xE5F1)
                ),
            );
            self.set_state(RxState::PllConfig);
        }

        // ---- rewind to PktRdy ----
        if self.state > RxState::PktRdy
            && drv_sys::pkt_acphdr_get(drv_sys::PKT_AVI_INFOFRAME)
            && (drv_rx::avi_chg_int_get() || drv_rx::gcp_chg_int_get())
        {
            log::info!("Receiver color signaling changed");
            log::debug!(
                "Receiver diagnostics: AVI bytes=[0x{:02X}, 0x{:02X}, 0x{:02X}], channel errors=[0x{:04X}, 0x{:04X}, 0x{:04X}]",
                reg::read(0xE875),
                reg::read(0xE876),
                reg::read(0xE877),
                pack_16(reg::read(0xE8D8), reg::read(0xE8D9)),
                pack_16(reg::read(0xE8DA), reg::read(0xE8DB)),
                pack_16(reg::read(0xE8DC), reg::read(0xE8DD)),
            );
            self.set_state(RxState::PktRdy);
        }

        if self.state == RxState::PlayBack && SCRAMBLER_RX_MODE != SCRAMBLER_NOT {
            drv_vscr::ver_get(&mut self.scrambler_status_last);
        }
    }

    // ---- helpers ----

    fn scrambler_init(&self) {
        if SCRAMBLER_RX_MODE == SCRAMBLER_NOT {
            drv_vscr::disable();
        } else {
            drv_vscr::init(SCRAMBLER_RX_MODE);
        }
    }

    /// Record a video check that never settled, logging what the chip
    /// measured.
    fn fail_video_check(&mut self) {
        let raw = drv_sys::video_check_all_get();
        let frame_rate_mhz = drv_sys::frame_rate_mhz();
        let frame_rate = if frame_rate_mhz == 0 {
            "no vertical sync".to_string()
        } else {
            format!("{:.2} Hz", frame_rate_mhz as f64 / 1000.0)
        };
        self.health.fail(
            Failure::NoSync,
            format_args!(
                "pixel clock {:.2} MHz, {}, stable flag {} (Register E580=0x{:02X}), raw {}x{} of {}x{}",
                raw.pixel_clock as f64 / 1000.0,
                frame_rate,
                if drv_rx::video_stable() { "set" } else { "clear" },
                reg::read(0xE580),
                raw.h_active,
                raw.v_active,
                raw.h_total,
                raw.v_total
            ),
        );
    }

    /// Resolve the video-check result: progressive formats gate on the chip's
    /// stable bit; interlaced formats fall back to a timing scan.
    fn vid_chk_deal(&mut self) -> Stability {
        match drv_rx::video_chk_stable(self) {
            Stability::Stable => {
                time::delay_ms(5);
                let h = drv_sys::video_check_single_get(drv_sys::HACTIVE_POS) as u16;
                let v = drv_sys::video_check_single_get(drv_sys::VACTIVE_POS) as u16;
                if drv_rx::resolution_sane(h, v) {
                    Stability::Stable
                } else {
                    Stability::Unstable
                }
            }
            Stability::Unstable => {
                if drv_rx::vid_chk_scan(self) {
                    log::debug!("Receiver video timing scan completed");
                    Stability::Stable
                } else {
                    Stability::Unstable
                }
            }
            Stability::Pending => Stability::Pending,
        }
    }
}
