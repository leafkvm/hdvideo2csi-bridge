//! Time utilities.
//!
//! Wraps [`std::time::Instant`] for use by the state machine timers.

use std::time::{Duration, Instant};

#[inline]
pub fn delay_ms(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

#[inline]
pub fn delay_us(us: u64) {
    std::thread::sleep(Duration::from_micros(us));
}

/// One-shot timer used by state machines to gate "wait N ms before checking
/// again". An unset timer is always considered due.
#[derive(Clone, Copy, Debug, Default)]
pub struct Timer {
    start: Option<Instant>,
}

impl Timer {
    pub fn start(&mut self) {
        self.start = Some(Instant::now());
    }

    /// `true` if `dur` has elapsed since [`start`](Self::start), or the timer
    /// has never been started.
    pub fn due(&self, dur: Duration) -> bool {
        self.start.is_none_or(|s| s.elapsed() >= dur)
    }
}
