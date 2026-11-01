//! The bridge's link to the capture side, chosen at compile time: the kernel
//! CSI stub (`stub` feature, the vendor V4L2 path) or the D-Bus service
//! (`dbus` feature, for the MIPI-only receiver driven from userspace).

#[cfg(feature = "dbus")]
pub use crate::bus::{
    get_edid, report_audio, report_status, report_video, stream_on, supports_output_format,
};
#[cfg(feature = "stub")]
pub use crate::stub::{
    get_edid, report_audio, report_status, report_video, stream_on, supports_output_format,
};

/// Something the capture side changed.
#[derive(Debug)]
pub enum Event {
    EdidUpdate,
    StreamChange(bool),
    /// Lanes or maximum lane rate changed (D-Bus only).
    #[cfg_attr(feature = "stub", allow(dead_code))]
    ConfigChange,
    #[cfg_attr(feature = "dbus", allow(dead_code))]
    Unknown(u32),
}

/// What the receiver takes: the transmitter's lane count and its lane-rate
/// ceiling (kbit/s per lane).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkConfig {
    pub lanes: u8,
    pub max_lane_rate_kbps: u32,
}

#[cfg(feature = "stub")]
pub fn config() -> LinkConfig {
    // the stub's receiver takes what the transmitter computes
    LinkConfig {
        lanes: crate::config::MIPI_LANE_NUM,
        max_lane_rate_kbps: u32::MAX,
    }
}

#[cfg(feature = "dbus")]
pub use crate::bus::config;

/// Every pending event, without blocking.
#[cfg(feature = "stub")]
pub fn poll_events() -> std::io::Result<Vec<Event>> {
    use crate::stub::StubEvent;
    let mut v = Vec::new();
    // as before the split: a failed read ends the batch
    while let Ok(Some(ev)) = crate::stub::poll_event() {
        v.push(match ev {
            StubEvent::EdidUpdate => Event::EdidUpdate,
            StubEvent::StreamChange(on) => Event::StreamChange(on),
            StubEvent::Unknown(n) => Event::Unknown(n),
        });
    }
    Ok(v)
}

#[cfg(feature = "dbus")]
pub use crate::bus::poll_events;
