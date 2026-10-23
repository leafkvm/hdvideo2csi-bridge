//! Audio acquisition state machine.
//!
//! Ticked by the receiver while video is in playback. It turns the chip's
//! audio status into one level: the sample rate the I2S output is delivering
//! with a verified clock, or `None`.
//!
//! ```text
//!          packets present                 locked on a known rate for 30 ms
//!   Idle ─────────────────────► Acquiring ─────────────────────────────────► Live
//!    ▲    (enable output)          │  ▲                                        │
//!    │                             │  └──── clock off the rate or PLL off ─────┘
//!    └──────── packets gone ───────┴───────────────────────────────────────────┘
//!              (disable output)
//! ```
//!
//! The rate is taken from the measured PLL clock, not from the chip's rate
//! meter, which ramps through other rates' values for the first 100 ms or
//! so. The PLL locks on its own within 0.4–1.2 s; if it
//! has not after [`LOCK_TIMEOUT`], it is re-seeded ([`drv::pll_reset`]).
//!
//! With [`AUDIO_TEST_PATTERN`] the source is ignored and the output carries
//! the chip's internal test tone instead (see [`Audio::tick_test_pattern`]).

use std::time::{Duration, Instant};

use crate::config::AUDIO_TEST_PATTERN;
use crate::drv::audio as drv;
use crate::drv::audio::Rate;

/// How long the PLL must stay locked on the rate before audio is reported
/// live.
const LOCK_STABLE: Duration = Duration::from_millis(30);
/// Time without lock after which a running PLL is re-seeded. A normal
/// acquisition locks after about 0.5 s, up to about 1.25 s.
const LOCK_TIMEOUT: Duration = Duration::from_millis(2000);
/// Sample rate of the test pattern.
const TEST_PATTERN_RATE: Rate = Rate::HZ_48000;

#[derive(Clone, Copy, Debug, Default)]
enum State {
    /// No audio packets; output disabled.
    #[default]
    Idle,
    /// Packets are arriving and the output is enabled; waiting for the PLL
    /// to lock on a known rate.
    Acquiring {
        since: Instant,
        /// Start of the current run of good readings.
        good_since: Option<Instant>,
        /// Last re-seed, so re-seeds are spaced by [`LOCK_TIMEOUT`].
        last_reset: Option<Instant>,
        /// The lock-off-rate debug line has been logged for this acquisition.
        unknown_logged: bool,
    },
    /// The I2S output runs with a verified clock.
    Live { rate: Rate },
}

impl State {
    fn acquiring() -> State {
        State::Acquiring {
            since: Instant::now(),
            good_since: None,
            last_reset: None,
            unknown_logged: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Audio {
    state: State,
}

impl Audio {
    /// Advance by one tick. Returns the live sample rate, or `None` while the
    /// output clock is absent or not yet verified.
    pub fn tick(&mut self) -> Option<Rate> {
        if AUDIO_TEST_PATTERN {
            return self.tick_test_pattern();
        }

        let packets = drv::packets_present();
        match (self.state, packets) {
            (State::Idle, false) => return None,
            (State::Idle, true) => {
                drv::output_enable(true);
                log::info!("Receiver audio detected");
                self.state = State::acquiring();
            }
            (_, false) => {
                drv::output_enable(false);
                log::info!("Receiver audio is unavailable");
                self.state = State::Idle;
                return None;
            }
            (_, true) => {}
        }

        let pll = drv::read_pll();
        match &mut self.state {
            State::Idle => None,
            State::Acquiring {
                since,
                good_since,
                last_reset,
                unknown_logged,
            } => {
                let rate = if pll.locked { pll.rate() } else { None };
                if let Some(rate) = rate {
                    let good_since = *good_since.get_or_insert_with(Instant::now);
                    if good_since.elapsed() >= LOCK_STABLE {
                        log::info!(
                            "Receiver audio locked: {rate} after {} ms ({pll})",
                            since.elapsed().as_millis()
                        );
                        self.state = State::Live { rate };
                    }
                    return None;
                }
                *good_since = None;
                // The lock flag sometimes asserts while the clock is still
                // slewing; the clock check above covers that.
                if pll.locked && !*unknown_logged {
                    log::debug!("Receiver audio PLL reports lock off any known rate ({pll})");
                    *unknown_logged = true;
                }
                // Re-seed a running PLL that has not locked in time. A PLL
                // that is not running has nothing to re-seed: the packets
                // have only just started, or this source never starts it.
                let waited = last_reset.unwrap_or(*since).elapsed();
                if pll.running && !pll.locked && waited >= LOCK_TIMEOUT {
                    log::warn!(
                        "Receiver audio PLL not locked after {} ms ({pll}); re-seeding it",
                        since.elapsed().as_millis()
                    );
                    drv::pll_reset();
                    *last_reset = Some(Instant::now());
                }
                None
            }
            State::Live { rate } => {
                let rate = *rate;
                // Live means the clock measures right. The lock flag only
                // qualifies the entry: it can flicker for a moment after
                // lock while the clock stays on frequency.
                if pll.running && pll.clock_matches(rate) {
                    return Some(rate);
                }
                // The PLL stops in the same instant the packets do, so a stop
                // seen right after a positive packet reading is usually the
                // source stopping; recheck before calling it a clock loss.
                if !drv::packets_present() {
                    drv::output_enable(false);
                    log::info!("Receiver audio is unavailable");
                    self.state = State::Idle;
                } else {
                    log::info!("Receiver audio clock lost ({pll})");
                    self.state = State::acquiring();
                }
                None
            }
        }
    }

    /// Test mode: run the internal pattern generator for the whole video
    /// session and report it live while its clock measures right. The
    /// pattern PLL has no lock indication, so only the clock is checked.
    fn tick_test_pattern(&mut self) -> Option<Rate> {
        if let State::Idle = self.state {
            drv::pattern_start(TEST_PATTERN_RATE);
            drv::output_enable(true);
            log::info!("Receiver audio test pattern started: {TEST_PATTERN_RATE}");
            self.state = State::Live {
                rate: TEST_PATTERN_RATE,
            };
        }
        drv::read_pll()
            .clock_matches(TEST_PATTERN_RATE)
            .then_some(TEST_PATTERN_RATE)
    }

    /// End of the video session: drop the output and reset the audio domain
    /// so the next session starts clean.
    pub fn stop(&mut self) {
        if !matches!(self.state, State::Idle) {
            log::info!("Receiver audio is unavailable");
        }
        if AUDIO_TEST_PATTERN {
            drv::pattern_stop();
        }
        drv::restart();
        drv::output_enable(false);
        self.state = State::Idle;
    }
}
