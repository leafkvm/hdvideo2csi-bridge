//! MIPI transmitter state machine.
//!
//! The transmitter advances once per main-loop tick (see [`Tx::tick`]). A tick
//! runs two steps:
//!
//! * [`Tx::state_judge`]   — regressions: rewind to an earlier state when an
//!   upstream condition (receiver video, kernel stream) is no longer met.
//! * [`Tx::state_handler`] — forward progress: the work done in the current
//!   state and the transition out of it.
//!
//! The transmitter is decoupled from the receiver: everything it needs from
//! Rx — including the `video` level that gates bring-up — arrives through
//! [`TxEnv`] as plain data, re-read fresh every tick.

use std::time::Duration;

use crate::config::*;
use crate::drv::mipi_tx as drv_tx;
use crate::format::{BitDepth, MipiFormat, PixelEncoding};
use crate::reg;
use crate::time::Timer;
use crate::timing::Timing;

/// MIPI transmitter state. Variants are in bring-up order so the derived `Ord`
/// lets [`Tx::state_judge`] rank regressions.
///
/// There is no separate "device ready" state: for the transmitter, "source
/// plugged but no video" is indistinguishable from "no source" — both are an
/// idle wait with the output off. The only milestone it acts on is video-ready.
///
/// The kernel-stream gate lives in [`TxState::AwaitStream`]: it is only
/// reachable with [`crate::config::MIPITX_OUT_CTRL`] enabled. Without it,
/// [`TxState::ConfigVideoAudio`] brings the output up itself and jumps
/// straight to [`TxState::PlayBack`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, Default)]
pub enum TxState {
    /// Idle until the upstream receiver has video ready; then latch timing.
    #[default]
    UpstreamVideoReady,
    /// One-shot: compute MIPI parameters and program PLL / PHY / digital config.
    ConfigVideoAudio,
    /// Kernel-stream-control gate; idle until `env.stream_on` is true.
    AwaitStream,
    /// MIPI output live.
    PlayBack,
}

pub const NONE_3D: u8 = 0;
pub const LR_MODE: u8 = 1;

/// The receiver-derived per-frame inputs the transmitter needs. App reads
/// these out of `Rx` and forwards them as plain data — Tx never holds a
/// reference to Rx.
///
/// Note the Tx *output* format is not here: it is owned by `Tx::format` and
/// written solely by `App::video_format_handler`, which runs before `tx.tick` each tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct TxFrame {
    pub interlace: bool,
    pub pixel_encoding: PixelEncoding,
}

/// All external inputs the transmitter needs per tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct TxEnv {
    pub frame: TxFrame,
    /// The receiver's `video` level: `true` while it is delivering live video.
    /// Gates [`TxState::UpstreamVideoReady`] — when it drops, [`Tx::state_judge`]
    /// rewinds the FSM to wait again.
    pub video: bool,
    /// Whether the kernel-side V4L2 pipeline is currently streaming.
    pub stream_on: bool,
    /// The receiver's latched video-check timing. Measured solely within Rx
    /// (see [`crate::app::rx::RxStatus::timing`]); the transmitter only reads
    /// it. Meaningful once `video` is `true`.
    pub timing: Timing,
}

// ---- Tx ----

/// MIPI transmitter context — recomputed once per video format change.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tx {
    pub state: TxState,
    /// One-shot "the FSM just entered this state" flag; consumed by [`Tx::entered`].
    pub state_changed: bool,

    pub clock_burst: bool,
    pub csi_8lane: bool,

    pub format: MipiFormat,
    pub pixel_encoding: PixelEncoding,
    pub bit_depth: BitDepth,
    pub bpp: u8,
    pub port_num: u8,
    pub lane_num: u8,
    pub three_d_mode: u8,
    pub data_rate: u32,
    pub mipi_in_clk: u32,
    pub data_rate_incre: u32,
    /// Per-lane MIPI data rate before LP/HS headroom is added (kbps). Latched
    /// by [`Tx::data_rate_adj`] so the D-PHY timing patches in `mipi_tx` gate
    /// on the same value. See [`LANE_RATE_HIGH`] / [`LANE_RATE_MID`].
    pub base_data_rate: u32,

    pub check_reg_timer: Timer,
}

impl Tx {
    /// One-shot bring-up. The FSM starts in
    /// [`TxState::UpstreamVideoReady`] — i.e. already waiting for video.
    pub fn power_on_init(&mut self) {
        *self = Tx::default();
        // Hardware CSI frame-end insertion when the output is disabled.
        reg::set_bit(0xEAB9, 6);
    }

    /// Drive a transition. Public so App can rewind the FSM when a receiver
    /// event demands a fresh video session (re-running resolution config).
    pub fn set_state(&mut self, new: TxState) {
        if self.state == new {
            return;
        }
        let last = std::mem::replace(&mut self.state, new);
        self.state_changed = true;
        log::debug!("Transmitter state: {:?} -> {:?}", last, new);
    }

    /// Consume the one-shot "just entered this state" flag.
    fn entered(&mut self) -> bool {
        std::mem::take(&mut self.state_changed)
    }

    /// Turn the MIPI output off. Idempotent.
    pub fn video_off(&self) {
        drv_tx::video_set(false);
    }

    /// Advance the state machine by one tick.
    pub fn tick(&mut self, env: TxEnv) {
        self.state_judge(env);
        self.state_handler(env);
    }

    // ---- regressions ----

    /// Rewind to an earlier state when an upstream condition the current state
    /// relies on stops holding.
    fn state_judge(&mut self, env: TxEnv) {
        // Upstream receiver lost video — rewind to the wait state.
        if self.state > TxState::UpstreamVideoReady && !env.video {
            self.set_state(TxState::UpstreamVideoReady);
        }
        if self.state == TxState::PlayBack {
            // Kernel stopped the stream — drop the live output back to the gate.
            if MIPITX_OUT_CTRL && !env.stream_on {
                self.video_off();
                self.set_state(TxState::AwaitStream);
            }
        }
    }

    // ---- forward progress ----

    fn state_handler(&mut self, env: TxEnv) {
        match self.state {
            TxState::UpstreamVideoReady => {
                if self.entered() {
                    self.video_off();
                }
                if env.video {
                    log::info!("Transmitter input video is ready");
                    if self.resolution_config(env.frame, &env.timing) {
                        self.set_state(TxState::ConfigVideoAudio);
                    }
                }
            }
            TxState::ConfigVideoAudio => {
                // `self.format` is the native input format, already
                // resolved by App::video_format_handler before this tick's tx.tick()
                // (see App::tick) — no copy from the receiver needed here.
                self.para_set();
                self.data_rate_adj();
                drv_tx::pll_set(self);
                drv_tx::phy_timing_para_set(self, env.frame.pixel_encoding);
                self.digital_config();
                self.analog_config();
                self.hss_set();
                self.fifo_rddly_config(&env.timing);
                if self.clock_burst {
                    drv_tx::dphy_clk_hs_trig();
                    drv_tx::dphy_skew_cali();
                }
                self.set_state(TxState::AwaitStream);
            }
            TxState::AwaitStream => {
                if env.stream_on || !MIPITX_OUT_CTRL {
                    self.bring_up_video();
                    self.set_state(TxState::PlayBack);
                }
            }
            TxState::PlayBack => {
                self.check_register_value();
            }
        }
    }

    /// Bring the MIPI output live: trigger the HS clock + skew calibration
    /// (unless in clock-burst mode, where that already happened during config),
    /// then enable the data lanes.
    fn bring_up_video(&self) {
        if !self.clock_burst {
            drv_tx::dphy_clk_hs_trig();
            drv_tx::dphy_skew_cali();
        }
        drv_tx::video_set(true);
    }

    // ---- parameter computation ----

    fn data_rate_adj(&mut self) {
        let base =
            self.mipi_in_clk * self.bpp as u32 * 2 / (self.port_num as u32 * self.lane_num as u32);
        self.base_data_rate = base;
        let mut rate = base;
        // Headroom for the D-PHY LP/HS switch, sized by the per-lane data rate
        // (i.e. proximity to the MIPITX_PLL_HIGH ceiling) rather than aggregate
        // bandwidth, so the tiers hold as the port/lane count changes. Burst
        // mode needs more than the fixed non-burst margin; dual-port adds more.
        if self.clock_burst {
            rate += if base >= LANE_RATE_HIGH {
                180_000
            } else if base >= LANE_RATE_MID {
                150_000
            } else {
                120_000
            };
        } else {
            rate += 80_000;
        }
        if self.port_num == 2 {
            rate += 40_000;
        }
        rate = rate.clamp(MIPITX_PLL_LOW, MIPITX_PLL_HIGH);
        self.data_rate = rate;
        self.data_rate_incre = rate - base;
        log::debug!(
            "Transmitter data-rate headroom: {} kbit/s per lane",
            self.data_rate_incre
        );
    }

    fn para_set(&mut self) {
        self.lane_num = MIPI_LANE_NUM;
        self.port_num = MIPI_PORT_NUM;
        self.clock_burst = MIPI_CLOCK_BURST;
        log::debug!("Transmitter clock burst: enabled={}", self.clock_burst);

        self.bpp = match self.format {
            MipiFormat::Rgb8Bit | MipiFormat::Yuv444_8Bit | MipiFormat::Yuv420_8Bit => 24,
            MipiFormat::Rgb10Bit | MipiFormat::Yuv444_10Bit | MipiFormat::Yuv420_10Bit => 30,
            MipiFormat::Yuv422_8Bit => 16,
            MipiFormat::Yuv422_10Bit => 20,
            MipiFormat::Yuv422_12Bit => 24,
            _ => 24,
        };

        drv_tx::refresh_mipi_in_clk(self);

        // If a single port can't carry the rate, split across two ports (L/R).
        let single_port_clk = self.mipi_in_clk * self.bpp as u32 * 2 / self.lane_num as u32;
        if single_port_clk > MIPITX_PLL_HIGH {
            self.port_num = 2;
        }
        if self.lane_num == 8 {
            self.port_num = 1;
            self.csi_8lane = true;
            self.three_d_mode = NONE_3D;
        } else {
            self.three_d_mode = if self.port_num == 2 { LR_MODE } else { NONE_3D };
        }
    }

    fn resolution_config(&self, frame: TxFrame, timing: &Timing) -> bool {
        if !frame.interlace && reg::read(0xE580) & 0x04 == 0 {
            log::warn!("Transmitter input timing is unstable");
            return false;
        }

        let h_ok = timing.h_total == timing.h_active + timing.h_sw + timing.h_fp + timing.h_bp;
        let v_ok = timing.v_total as u32
            == timing.v_active as u32
                + timing.v_sw as u32
                + timing.v_fp as u32
                + timing.v_bp as u32;
        if h_ok && v_ok && timing.v_active & 1 == 0 && timing.h_active & 1 == 0 {
            log::info!("Transmitter timing: {:?}", timing);
            drv_tx::timing_set(timing);
            drv_tx::in_hsync_pol_sel(timing.h_spol);
            drv_tx::in_vsync_pol_sel(timing.v_spol);
            true
        } else {
            log::warn!("Transmitter timing parameters are invalid");
            false
        }
    }

    // ---- chip configuration ----

    fn hss_set(&self) {
        let hs_rqst_pre = reg::read(0xEA8A) as u16;
        drv_tx::hss_set(hs_rqst_pre + 3);
    }

    fn fifo_rddly_config(&self, timing: &Timing) {
        let rdpixclk = (self.mipi_in_clk / 1000).max(1);
        let mut rdbyteclk = self.data_rate / 2000 / 8 * self.port_num as u32;
        if self.lane_num == 8 && self.port_num == 1 {
            rdbyteclk <<= 1;
        }
        let hss = drv_tx::hss_get() as u32;
        let rgod_hact = drv_tx::fsm_hact_get() as u32;
        let h_half_sum = (timing.h_sw as u32 >> 1) + (timing.h_bp as u32 >> 1);

        let rddly_min1 = (rdbyteclk * h_half_sum / rdpixclk).saturating_sub(hss);
        let rddly_min2 = if self.port_num == 1 {
            let q = rdbyteclk * (h_half_sum + (timing.h_active as u32 >> 1)) / rdpixclk;
            q.saturating_sub(hss).saturating_sub(rgod_hact)
        } else {
            rddly_min1
        }
        .max(rddly_min1);

        let rddly_max = {
            let cap = 0x8000u32 / ((self.bpp as u32) << 1);
            rdbyteclk * (cap + h_half_sum) / rdpixclk - hss
        };
        log::debug!(
            "Transmitter FIFO delay bounds: minimum 1={}, minimum 2={}, maximum={}",
            rddly_min1,
            rddly_min2,
            rddly_max
        );

        let rddly = if rddly_max > rddly_min2 {
            // Bias inside the valid window; a wider window with high data-rate
            // headroom is biased closer to the minimum.
            let div = if self.data_rate_incre >= 200_000 {
                20
            } else {
                10
            };
            (rddly_max - rddly_min2) / div + rddly_min2
        } else {
            rddly_min2
        };
        drv_tx::fifo_delay_set(rddly as u16);
        log::debug!("Transmitter FIFO delay: {}", rddly);
    }

    fn digital_config(&self) {
        drv_tx::lane_set(self.lane_num);
        drv_tx::swap_set(&MIPI_SWAP_CONFIG);
        drv_tx::dphy_clk_data_set();
        drv_tx::dphy_clk_mode_sel(self.clock_burst);
        drv_tx::half_wr_clk_src_sel(drv_tx::WRITE_CLK);
        if self.csi_8lane {
            drv_tx::half_wr_clk_src_sel(drv_tx::HALF_WRITE_CLK);
            drv_tx::dphy_csi_8lane_en(true);
        }
        drv_tx::port_set(self.port_num);
        drv_tx::three_d_mode_set(self.three_d_mode);
        drv_tx::csi_data_type_set(self.format);
        log::info!("Transmitter output format: {}", self.format);
    }

    fn analog_config(&self) {
        drv_tx::dphy_set(self.data_rate);
        drv_tx::np_swap_set(&MIPI_SWAP_CONFIG);
    }

    /// Periodically scrub a MIPI register the chip is known to corrupt.
    fn check_register_value(&mut self) {
        if !self.check_reg_timer.due(Duration::from_millis(50)) {
            return;
        }
        self.check_reg_timer.start();
        let v = reg::read(0xEA7A);
        if v & 0xF0 != 0 {
            reg::write(0xEA7A, v & 0x0F);
            log::debug!("Register EA7A contained invalid bits; cleared upper nibble");
        }
    }
}

const MIPITX_PLL_LOW: u32 = 400_000;
const MIPITX_PLL_HIGH: u32 = 2_500_000;

/// Per-lane data-rate tiers (kbps) that size the clock-burst D-PHY headroom,
/// read relative to the 2.5 Gbps `MIPITX_PLL_HIGH` ceiling. Because they gate
/// on the per-lane rate — which already divides out the port/lane count — the
/// tiers hold across lane configurations, unlike the old aggregate-bandwidth
/// match:
///   * `>= LANE_RATE_HIGH` — ~2.2 Gbps/lane (4096x2160@60 YUV422 class)
///   * `>= LANE_RATE_MID`  — ~1.15 Gbps/lane (2560x1440@60 class)
///
// TODO: these boundaries are derived from the nominal pixel clocks of the
// resolutions the old exact-match checks keyed on; confirm them against the
// measured per-lane rate (Tx::base_data_rate) for the modes actually in use.
pub const LANE_RATE_HIGH: u32 = 2_200_000;
pub const LANE_RATE_MID: u32 = 1_150_000;
