//! Receiver health: why the input bring-up last failed, and how many times
//! in a row.
//!
//! It feeds two consumers. The kernel status report (see
//! [`crate::app::App`]) publishes it in chip-independent terms; the details
//! of each failure only go to the log. The log uses
//! it to keep a retry loop readable: a source the receiver cannot lock makes
//! the FSM cycle every few seconds, so each attempt's lines go out at INFO
//! (failures at WARN) only on the first attempts and then on every
//! power-of-two repeat, and at DEBUG otherwise.

use std::fmt::Display;

use crate::stub::Failure;

#[derive(Clone, Copy, Debug, Default)]
pub struct Health {
    /// Cause of the most recent failure; kept after a later lock.
    pub last_error: Option<Failure>,
    /// Consecutive failures with `last_error`; reset by [`Health::locked`].
    pub error_count: u32,
}

/// Attempt `n` of a retry loop is logged loudly: the first two, then every
/// power of two.
fn loud(n: u32) -> bool {
    n <= 2 || n.is_power_of_two()
}

impl Health {
    /// Record a failed bring-up step, logging `detail` with it.
    pub fn fail(&mut self, failure: Failure, detail: impl Display) {
        if self.last_error == Some(failure) && self.error_count > 0 {
            self.error_count = self.error_count.saturating_add(1);
        } else {
            self.last_error = Some(failure);
            self.error_count = 1;
        }
        let n = self.error_count;
        let level = if loud(n) {
            log::Level::Warn
        } else {
            log::Level::Debug
        };
        if n == 1 {
            log::log!(level, "{failure}: {detail}");
        } else {
            log::log!(level, "{failure} (attempt {n}): {detail}");
        }
    }

    /// The input locked: the failure streak, if any, is over.
    pub fn locked(&mut self) {
        if self.error_count > 1 {
            log::info!("Recovered after {} failed attempts", self.error_count);
        }
        self.error_count = 0;
    }

    /// The source reconnected or the EDID changed: the streak is over, but
    /// `last_error` still shows what happened last.
    pub fn reconnected(&mut self) {
        self.error_count = 0;
    }

    /// Level for the progress lines of the attempt in flight, which ends with
    /// failure number `error_count + 1` if it fails too.
    pub fn progress_level(&self) -> log::Level {
        if loud(self.error_count.saturating_add(1)) {
            log::Level::Info
        } else {
            log::Level::Debug
        }
    }
}
