//! D-Bus service (`dbus` feature): the bridge's link to the server, in place
//! of the kernel CSI stub.
//!
//! The daemon owns `org.leafkvm.Bridge` on the system bus and exports one
//! object, `/org/leafkvm/Bridge`, with the interface `org.leafkvm.Bridge1`:
//!
//! * properties (read-only, `PropertiesChanged` on every change), each a
//!   `a{sv}` dictionary:
//!   * `Video`: `sequence` (u, +1 on every change of the report), `present`
//!     (b), `width`, `height` (u), `pixel_clock` (t, Hz), `h_total`,
//!     `v_total` (u; frame rate = pixel_clock / (h_total * v_total)),
//!     `h_front_porch`, `h_sync`, `h_back_porch`, `v_front_porch`,
//!     `v_sync`, `v_back_porch` (u), `h_sync_positive`, `v_sync_positive` (b),
//!     `interlaced` (b), `encoding` (s: rgb, yuv444, yuv422, yuv420),
//!     `data_type` (u, CSI-2), `virtual_channel` (u), `lanes` (u),
//!     `lane_rate` (t, bit/s per lane), `quantization`, `colorspace` (u,
//!     V4L2 enum values);
//!   * `Audio`: `present` (b), `sample_rate` (u, Hz);
//!   * `Status`: `stage`, `last_failure`, `failure_count` (u; the values of
//!     the stub's `HDMI_CSI_RX_*` / `HDMI_CSI_ERR_*`).
//! * methods:
//!   * `SetEdid(ay)`: the EDID to present, whole 128-byte blocks; empty =
//!     none (hot-plug low).
//!   * `Configure(u lanes, t max_lane_rate)`: the receiver's lanes and its
//!     highest lane rate in bit/s.
//!   * `SetStreaming(b on, u sequence)`: send (or stop) the stream described
//!     by the `Video` report `sequence`.  A different report (any change)
//!     stops the output until it is acknowledged the same way, so the
//!     receiver can always be set up before the transmitter starts.
//!     Fails with `org.leafkvm.Bridge1.Error.Stale` if `sequence` is not the
//!     current report (a newer one is on its way).
//!
//! The daemon keeps no state across restarts: a new instance starts without
//! an EDID and with streaming off; the server re-sends everything when the
//! name appears again.  Everything runs in the main loop: [`poll_event`]
//! handles the pending calls once per tick, without blocking.

use std::io;
use std::sync::{Arc, LazyLock, Mutex};
use std::time::Duration;

use dbus::arg::{PropMap, RefArg, Variant};
use dbus::blocking::Connection;
use dbus::blocking::stdintf::org_freedesktop_dbus::PropertiesPropertiesChanged;
use dbus::channel::Sender;
use dbus::message::SignalArgs;
use dbus_crossroads::{Crossroads, MethodErr};

use crate::config::MIPI_LANE_NUM;
use crate::format::{MipiFormat, PixelEncoding};
use crate::link::{Event, LinkConfig};
use crate::stub::{Status, VideoInputCsiEdid, VideoState};

pub const BUS_NAME: &str = "org.leafkvm.Bridge";
pub const OBJECT_PATH: &str = "/org/leafkvm/Bridge";
pub const INTERFACE: &str = "org.leafkvm.Bridge1";
const ERR_STALE: &str = "org.leafkvm.Bridge1.Error.Stale";

const EDID_BLOCK: usize = 128;
const EDID_MAX: usize = 8 * EDID_BLOCK;
/// Until `Configure`: all four lanes, the D-PHY ceiling of the receiver.
const DEFAULT_MAX_LANE_RATE: u64 = 2_500_000_000;

/// CSI-2 data type of a transmitter format the receiver takes.
fn data_type(format: MipiFormat) -> Option<u8> {
    match format {
        MipiFormat::Rgb8Bit | MipiFormat::Yuv444_8Bit => Some(0x24),
        MipiFormat::Yuv422_8Bit => Some(0x1e),
        MipiFormat::Yuv420_8Bit => Some(0x1a), // legacy YUV420 8-bit
        _ => None,
    }
}

#[derive(Default)]
struct Video {
    sequence: u32,
    state: Option<VideoState>,
    lanes: u8,
}

/// Everything the method and property callbacks share with the main loop.
struct Shared {
    edid: Vec<u8>,
    config: LinkConfig,
    /// `SetStreaming(true, n)`: the report the server set the receiver up for.
    stream: Option<u32>,
    video: Video,
    audio: (bool, u32),
    status: Status,
    events: Vec<Event>,
}

type State = Arc<Mutex<Shared>>;

fn video_map(v: &Video) -> PropMap {
    let mut m = PropMap::new();
    let mut put = |k: &str, val: Box<dyn RefArg>| {
        m.insert(k.into(), Variant(val));
    };
    put("sequence", Box::new(v.sequence));
    put("present", Box::new(v.state.is_some()));
    if let Some(s) = v.state {
        let t = s.timing;
        // YUV420 is measured at half the horizontal size and clock
        let mul = if s.format.is_yuv420() { 2 } else { 1 };
        put("width", Box::new(t.h_active as u32 * mul));
        put("height", Box::new(t.v_active as u32));
        put(
            "pixel_clock",
            Box::new(t.pixel_clock as u64 * 1000 * mul as u64),
        );
        put("h_total", Box::new(t.h_total as u32 * mul));
        put("v_total", Box::new(t.v_total as u32));
        put("h_front_porch", Box::new(t.h_fp as u32 * mul));
        put("h_sync", Box::new(t.h_sw as u32 * mul));
        put("h_back_porch", Box::new(t.h_bp as u32 * mul));
        put("v_front_porch", Box::new(t.v_fp as u32));
        put("v_sync", Box::new(t.v_sw as u32));
        put("v_back_porch", Box::new(t.v_bp as u32));
        put("h_sync_positive", Box::new(t.h_spol));
        put("v_sync_positive", Box::new(t.v_spol));
        put("interlaced", Box::new(false));
        let enc = match s.format.pixel_encoding() {
            PixelEncoding::Rgb => "rgb",
            PixelEncoding::Yuv444 => "yuv444",
            PixelEncoding::Yuv422 => "yuv422",
            PixelEncoding::Yuv420 => "yuv420",
        };
        put("encoding", Box::new(enc.to_string()));
        put(
            "data_type",
            Box::new(data_type(s.format).unwrap_or(0) as u32),
        );
        put("virtual_channel", Box::new(0u32));
        put("lanes", Box::new(v.lanes as u32));
        // the byte clock is in kHz: lane rate = 8 bits per byte clock
        put("lane_rate", Box::new(s.mipi_byte_clock as u64 * 8000));
        put("quantization", Box::new(s.color.quantization as u32));
        put("colorspace", Box::new(s.color.colorspace as u32));
    }
    m
}

fn audio_map(present: bool, rate: u32) -> PropMap {
    let mut m = PropMap::new();
    m.insert("present".into(), Variant(Box::new(present)));
    m.insert("sample_rate".into(), Variant(Box::new(rate)));
    m
}

fn status_map(s: &Status) -> PropMap {
    let mut m = PropMap::new();
    m.insert("stage".into(), Variant(Box::new(s.rx_stage as u32)));
    m.insert(
        "last_failure".into(),
        Variant(Box::new(s.last_error as u32)),
    );
    m.insert("failure_count".into(), Variant(Box::new(s.error_count)));
    m
}

pub struct Bus {
    conn: Connection,
    cr: Crossroads,
    state: State,
}

impl Bus {
    fn new() -> Result<Self, dbus::Error> {
        let conn = Connection::new_system()?;
        let reply = conn.request_name(BUS_NAME, false, true, true)?;
        if reply != dbus::blocking::stdintf::org_freedesktop_dbus::RequestNameReply::PrimaryOwner {
            return Err(dbus::Error::new_failed(&format!(
                "{BUS_NAME} is owned by another process"
            )));
        }
        let state: State = Arc::new(Mutex::new(Shared {
            edid: Vec::new(),
            config: LinkConfig {
                lanes: MIPI_LANE_NUM,
                max_lane_rate_kbps: (DEFAULT_MAX_LANE_RATE / 1000) as u32,
            },
            stream: None,
            video: Video::default(),
            audio: (false, 0),
            status: Status::default(),
            events: Vec::new(),
        }));

        let mut cr = Crossroads::new();
        let iface = cr.register(INTERFACE, |b| {
            b.property("Video")
                .get(|_, s: &mut State| Ok(video_map(&s.lock().unwrap().video)));
            b.property("Audio").get(|_, s: &mut State| {
                let a = s.lock().unwrap().audio;
                Ok(audio_map(a.0, a.1))
            });
            b.property("Status")
                .get(|_, s: &mut State| Ok(status_map(&s.lock().unwrap().status)));
            b.method(
                "SetEdid",
                ("edid",),
                (),
                |_, s: &mut State, (edid,): (Vec<u8>,)| {
                    if edid.len() % EDID_BLOCK != 0 || edid.len() > EDID_MAX {
                        return Err(MethodErr::invalid_arg(&format!(
                            "EDID of {} bytes: whole 128-byte blocks, at most 8",
                            edid.len()
                        )));
                    }
                    let mut s = s.lock().unwrap();
                    if s.edid != edid {
                        log::info!("Server EDID set: blocks={}", edid.len() / EDID_BLOCK);
                        s.edid = edid;
                        s.events.push(Event::EdidUpdate);
                    }
                    Ok(())
                },
            );
            b.method(
                "Configure",
                ("lanes", "max_lane_rate"),
                (),
                |_, s: &mut State, (lanes, max_lane_rate): (u32, u64)| {
                    if !matches!(lanes, 1 | 2 | 4) || max_lane_rate < 80_000_000 {
                        return Err(MethodErr::invalid_arg(&format!(
                            "lanes {lanes} (1, 2 or 4), maximum lane rate {max_lane_rate} bit/s"
                        )));
                    }
                    let config = LinkConfig {
                        lanes: lanes as u8,
                        max_lane_rate_kbps: (max_lane_rate / 1000).min(u32::MAX as u64) as u32,
                    };
                    let mut s = s.lock().unwrap();
                    if s.config != config {
                        log::info!(
                            "Receiver configuration: lanes={}, maximum lane rate={} Mbit/s",
                            lanes,
                            max_lane_rate / 1_000_000
                        );
                        s.config = config;
                        s.events.push(Event::ConfigChange);
                    }
                    Ok(())
                },
            );
            b.method(
                "SetStreaming",
                ("on", "sequence"),
                (),
                |_, s: &mut State, (on, sequence): (bool, u32)| {
                    let mut s = s.lock().unwrap();
                    if on && sequence != s.video.sequence {
                        return Err(MethodErr::from((
                            ERR_STALE,
                            format!(
                                "video report {sequence} is stale, the current one is {}",
                                s.video.sequence
                            ),
                        )));
                    }
                    let stream = on.then_some(sequence);
                    if s.stream != stream {
                        match stream {
                            Some(n) => log::info!("Server streams video report {}", n),
                            None => log::info!("Server stopped streaming"),
                        }
                        s.stream = stream;
                        s.events.push(Event::StreamChange(on));
                    }
                    Ok(())
                },
            );
        });
        cr.insert(OBJECT_PATH, &[iface], state.clone());
        Ok(Self { conn, cr, state })
    }

    fn emit(&self, name: &str, value: PropMap) {
        let mut changed = PropMap::new();
        changed.insert(name.into(), Variant(Box::new(value)));
        let sig = PropertiesPropertiesChanged {
            interface_name: INTERFACE.into(),
            changed_properties: changed,
            invalidated_properties: Vec::new(),
        };
        let path = dbus::Path::from(OBJECT_PATH);
        if self.conn.send(sig.to_emit_message(&path)).is_err() {
            log::error!("Failed to queue the {name} property change");
        }
        self.conn.channel().flush();
    }

    /// Handle every pending call; returns the events they caused.
    fn poll(&mut self) -> io::Result<Vec<Event>> {
        let ch = self.conn.channel();
        ch.read_write(Some(Duration::ZERO))
            .map_err(|()| io::Error::other("D-Bus connection lost"))?;
        while let Some(msg) = ch.pop_message() {
            // signals and replies are not for us (Err)
            let _ = self.cr.handle_message(msg, &self.conn);
        }
        Ok(std::mem::take(&mut self.state.lock().unwrap().events))
    }
}

// ---------- process-wide singleton, same shape as `stub` ----------

static BUS: LazyLock<Mutex<Option<Bus>>> = LazyLock::new(|| Mutex::new(None));

pub fn init() -> Result<(), dbus::Error> {
    let bus = Bus::new()?;
    log::info!("D-Bus service {} ready at {}", BUS_NAME, OBJECT_PATH);
    *BUS.lock().unwrap() = Some(bus);
    Ok(())
}

fn with<R>(f: impl FnOnce(&mut Bus) -> R) -> R {
    let mut guard = BUS.lock().unwrap();
    f(guard.as_mut().expect("bus::init not called"))
}

pub fn supports_output_format(format: MipiFormat) -> bool {
    data_type(format).is_some()
}

pub fn report_video(video: Option<VideoState>) -> io::Result<()> {
    if let Some(v) = video {
        if data_type(v.format).is_none() {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!("the receiver cannot take {}", v.format),
            ));
        }
    }
    with(|b| {
        let map = {
            let mut s = b.state.lock().unwrap();
            let lanes = s.config.lanes;
            let v = &mut s.video;
            v.sequence = v.sequence.wrapping_add(1);
            v.state = video;
            v.lanes = lanes;
            log::info!(
                "Video report {}: {}",
                v.sequence,
                match video {
                    Some(st) => format!(
                        "{}x{} {} at {} kbit/s per lane",
                        st.timing.h_active,
                        st.timing.v_active,
                        st.format,
                        st.mipi_byte_clock * 8
                    ),
                    None => "no video".into(),
                }
            );
            video_map(v)
        };
        b.emit("Video", map);
    });
    Ok(())
}

pub fn report_audio(sample_rate: u32, present: bool) -> io::Result<()> {
    with(|b| {
        {
            let mut s = b.state.lock().unwrap();
            if s.audio == (present, sample_rate) {
                return;
            }
            s.audio = (present, sample_rate);
        }
        b.emit("Audio", audio_map(present, sample_rate));
    });
    Ok(())
}

pub fn report_status(status: &Status) -> io::Result<()> {
    with(|b| {
        b.state.lock().unwrap().status = *status;
        b.emit("Status", status_map(status));
    });
    Ok(())
}

/// The EDID the server set, in the stub's layout.
pub fn get_edid() -> io::Result<VideoInputCsiEdid> {
    with(|b| {
        let s = b.state.lock().unwrap();
        let mut e = VideoInputCsiEdid::default();
        e.blocks = (s.edid.len() / EDID_BLOCK) as u16;
        e.data[..s.edid.len()].copy_from_slice(&s.edid);
        Ok(e)
    })
}

/// Streaming is on while the server acknowledged the current video report.
pub fn stream_on() -> io::Result<bool> {
    with(|b| {
        let s = b.state.lock().unwrap();
        Ok(s.video.state.is_some() && s.stream == Some(s.video.sequence))
    })
}

pub fn config() -> LinkConfig {
    with(|b| b.state.lock().unwrap().config)
}

pub fn poll_events() -> io::Result<Vec<Event>> {
    with(|b| b.poll())
}
