//! Application layer: all daemon-wide state and per-tick state machines.

pub mod audio;
pub mod rx;
pub mod tx;
pub mod video_scrambler_chip;

use std::sync::LazyLock;

use crate::app::rx::{Rx, RxEnv, RxStatus};
use crate::app::tx::{Tx, TxEnv, TxFrame, TxState};
use crate::config::Level;
use crate::drv::sink::{self, SinkEvent};
use crate::drv::{self, mipi_tx, video_color};
use crate::iio::VoltageChannel;
use crate::stub;
use crate::stub::{Color, StubEvent};
use crate::time;

/// Daemon-wide state. Owned by `main`. The Rx and Tx state machines each own
/// their own bookkeeping; the only cross-domain glue kept here is the
/// per-channel sink-dedup slots. The video-check timing is Rx state — it
/// reaches the transmitter through [`TxEnv`].
#[derive(Default)]
pub struct App {
    pub rx: Rx,
    pub tx: Tx,

    /// The latest EDID fetched from the kernel stub, or `None` when the kernel
    /// has no usable EDID. Refreshed at boot and on every `EdidUpdate` event;
    /// handed to the receiver each tick through [`RxEnv`].
    pub kernel_edid: Option<drv::edid::Edid>,

    /// Colour signalling (quant range + colorimetry) resolved by `App::video_format_handler`
    /// from Video Input input signalling. Reported with the unmodified payload and timings.
    /// Defaults to DEFAULT/unknown until an input is resolved.
    pub color: Color,

    /// The stub supports the current CSI wire format. A failed atomic video
    /// report also clears this gate until the next receiver configuration.
    pub output_format_ready: bool,

    /// Dedup slots for the kernel-stub reporter — one per channel so video and
    /// audio levels can be pushed independently each tick without clobbering.
    last_video_event: Option<SinkEvent>,
    last_audio_event: Option<SinkEvent>,
}

static FIVE_V: LazyLock<VoltageChannel> = LazyLock::new(|| {
    let sensor = VoltageChannel::new("hd_5v", 0).expect("Failed to open Video Input 5 V sensor");
    log::info!(
        "Video Input 5 V sensor found: name=hd_5v, voltage={:.0} mV",
        sensor
            .read()
            .expect("Failed to read Video Input 5 V sensor")
    );
    sensor
});

pub fn five_v_get() -> Level {
    let voltage_mv = FIVE_V
        .read()
        .expect("Failed to read Video Input 5 V sensor");
    if voltage_mv > 4000.0 {
        Level::High
    } else {
        Level::Low
    }
}

impl App {
    /// Crystal/clock + video-scrambler key probe + Rx/Tx power-on init.
    pub fn boot(&mut self) {
        // Seed the dedup slots to the boot reality (no video, no audio) so the
        // first ticks don't push a spurious "disappear" to the kernel.
        self.last_video_event = Some(SinkEvent::VideoDisappear);
        self.last_audio_event = Some(SinkEvent::AudioDisappear);

        drv::system::xtal_set();
        self.rx.scrambler14_ver = video_scrambler_chip::check_keys();
        self.rx.power_on_init();
        self.tx.power_on_init();

        // Fetch the current kernel EDID up front so the receiver can connect
        // without first waiting for a `StubEvent::EdidUpdate`.
        self.reload_kernel_edid();
    }

    /// Fetch the current EDID from the kernel stub, validate it, and store the
    /// result in `self.kernel_edid` (`None` = no usable EDID). This is the only
    /// place the daemon reads EDID from the stub.
    fn reload_kernel_edid(&mut self) {
        self.kernel_edid = match stub::get_edid() {
            Ok(e) => {
                let blocks = e.blocks as usize;
                match drv::edid::Edid::from_kernel(&e.data, blocks) {
                    Some(edid) => {
                        log::info!("Kernel EDID loaded: blocks={}", blocks);
                        Some(edid)
                    }
                    None => {
                        if blocks == 0 {
                            log::info!("Kernel EDID is empty; keeping receiver disconnected");
                        } else {
                            log::warn!(
                                "Kernel EDID exceeds receiver capacity: blocks={}, maximum={}; keeping receiver disconnected",
                                blocks,
                                drv::edid::EDID_MAX_SUPPORTED_BLOCKS
                            );
                        }
                        None
                    }
                }
            }
            Err(e) => {
                log::warn!(
                    "Failed to read kernel EDID: {}; keeping receiver disconnected",
                    e
                );
                None
            }
        };
    }

    fn video_format_handler(&mut self) {
        if !self.rx.format_pending {
            return;
        }

        self.tx.pixel_encoding = self.rx.pixel_encoding;
        self.tx.format = self.rx.format;
        self.tx.bit_depth = self.rx.format.bit_depth();
        self.color = video_color::configure_bypass(self.rx.pixel_encoding, self.rx.timing.v_active);

        // Never replace a deep-colour or unsupported input with a lower-depth
        // format. Keep video off unless the native packing and sink are supported.
        self.output_format_ready = mipi_tx::csi_packet_format(self.tx.format).is_some()
            && crate::stub::supports_output_format(self.tx.format);
        if !self.output_format_ready {
            log::error!(
                "Unsupported native output format: {}; keeping video disabled",
                self.tx.format
            );
        }
        self.rx.format_pending = false;
    }

    /// One iteration of the main loop.
    pub fn tick(&mut self) {
        while let Ok(Some(ev)) = stub::poll_event() {
            self.handle_kernel_event(ev);
        }

        // Advance the receiver and read its output levels.
        let rx_env = RxEnv {
            kernel_edid: self.kernel_edid,
            source_5v: five_v_get(),
        };
        let rx = self.rx.tick(rx_env);

        // Resolve the native Tx output format before the Tx FSM consumes
        // it. video_format_handler only does work on the tick the receiver latched
        // `format_pending`; otherwise it early-returns and the previous
        // `tx.format` persists. Running it here — rather than after
        // `tx.tick` — closes the IMP-17 race where `ConfigVideoAudio` would
        // program the MIPI data type from a stale format.
        self.video_format_handler();

        // Advance the transmitter; it consumes the receiver's `video` level
        // and latched timing directly through `TxEnv` and gates its own
        // bring-up / rewind on them.
        let tx_env = TxEnv {
            frame: TxFrame {
                interlace: self.rx.interlace,
                pixel_encoding: self.rx.pixel_encoding,
            },
            video: rx.video && self.output_format_ready,
            stream_on: stub::stream_on().unwrap_or(false),
            timing: rx.timing,
        };
        self.tx.tick(tx_env);

        self.report_to_sink(rx);

        time::delay_ms(2);
    }

    /// Push the current video / audio levels to the kernel stub. The stub
    /// protocol is edge-based, but each `sink::report_*` call dedups against
    /// its channel's slot — so pushing the level every tick yields exactly one
    /// report per real transition.
    fn report_to_sink(&mut self, rx: RxStatus) {
        // Video is "ready" once the transmitter has computed its MIPI
        // parameters — i.e. it has reached AwaitStream or beyond.
        if self.output_format_ready && self.tx.state >= TxState::AwaitStream {
            let mipi_byte_clock = self.tx.data_rate / 8;
            if let Err(e) = sink::report_video_ready(
                &mut self.last_video_event,
                &rx.timing,
                mipi_byte_clock,
                self.tx.format,
                self.color,
            ) {
                log::error!("Failed to publish CSI video state: {}", e);
                self.output_format_ready = false;
                self.tx.video_off();
                if let Err(e) = sink::report_video_disappear(&mut self.last_video_event) {
                    log::error!("Failed to clear CSI video state: {}", e);
                }
            }
        } else if let Err(e) = sink::report_video_disappear(&mut self.last_video_event) {
            log::error!("Failed to clear CSI video state: {}", e);
        }

        if let Some(rate) = rx.audio {
            sink::report_audio_ready(&mut self.last_audio_event, rate);
        } else {
            sink::report_audio_disappear(&mut self.last_audio_event);
        }
    }

    fn handle_kernel_event(&mut self, ev: StubEvent) {
        match ev {
            StubEvent::EdidUpdate => {
                log::info!("Kernel EDID changed; reloading");
                self.reload_kernel_edid();
                self.rx.reload_edid();
            }
            StubEvent::StreamChange(on) => log::info!(
                "Kernel stream state changed: {}",
                if on { "on" } else { "off" }
            ),
            StubEvent::Unknown(n) => log::warn!("Unknown kernel stub event: {}", n),
        }
    }
}
