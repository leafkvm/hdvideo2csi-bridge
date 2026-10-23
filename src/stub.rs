//! Video Input CSI stub access and the process-wide device handle.
//!
//! [`StubDevice`] owns the supplied device. Video format, timings and colour
//! are published together through its private control ABI. The free functions
//! below forward through a [`LazyLock`] singleton so callers need not thread a
//! device handle through the daemon.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::io::AsRawFd;
use std::sync::{LazyLock, Mutex};

use crate::format::{MipiFormat, PixelEncoding};
use crate::timing::Timing;

// ---------- UAPI constants (must match the kernel CSI stub ABI) ----------

const IOC_MAGIC: u8 = b'H';
const ABI_VERSION: u32 = 2;
const MAX_MBUS_CODES: usize = 8;

// Event types
pub const EVT_EDID_UPDATE: u32 = 1;
pub const EVT_STREAM_CHANGE: u32 = 3;

// ---------- UAPI structures ----------

/// V4L2 quantization (sample-value range) values, mirrored from
/// `enum v4l2_quantization` for the kernel ABI.
pub mod quantization {
    pub const DEFAULT: u8 = 0;
    pub const FULL_RANGE: u8 = 1;
    pub const LIM_RANGE: u8 = 2;
}

/// V4L2 colorimetry values, mirrored from `enum v4l2_colorspace`. Only the
/// subset the daemon can detect from AVI signalling is listed.
pub mod colorspace {
    pub const DEFAULT: u8 = 0;
    pub const SMPTE170M: u8 = 1; // BT.601 525-line
    pub const SYSTEM_BG: u8 = 6; // BT.601 625-line
    pub const REC709: u8 = 3; // BT.709
    pub const BT2020: u8 = 10;
}

/// Colour signalling reported alongside the timings. Values use the V4L2 enum
/// conventions above so the kernel re-exports them verbatim.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Color {
    pub quantization: u8,
    pub colorspace: u8,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VideoInputCsiTimings {
    pub mipi_byte_clock: u32,
    pub pixel_clock: u32,
    pub h_active: u16,
    pub h_total: u16,
    pub h_fp: u16,
    pub h_sync: u16,
    pub h_bp: u16,
    pub v_active: u16,
    pub v_total: u16,
    pub v_fp: u8,
    pub v_sync: u8,
    pub v_bp: u8,
    pub h_sync_pol: bool,
    pub v_sync_pol: bool,
    pub interlaced: u8,
    // Colour signalling, folded in so it is reported together with timings.
    // Field layout must match the kernel CSI stub timing structure.
    pub quantization: u8,
    pub colorspace: u8,
    pub reserved: [u8; 10],
}

/// Complete source snapshot carried by REPORT_VIDEO (not a DDR layout).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct VideoInputCsiVideo {
    timings: VideoInputCsiTimings,
    mbus_fmt_code: u32,
    payload_encoding: u32,
    reserved: [u32; 2],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct VideoInputCsiCaps {
    abi_version: u32,
    num_mbus_codes: u32,
    mbus_codes: [u32; MAX_MBUS_CODES],
    reserved: [u32; 6],
}

impl VideoInputCsiCaps {
    fn validate(&self) -> io::Result<()> {
        if self.abi_version != ABI_VERSION
            || self.num_mbus_codes == 0
            || self.num_mbus_codes as usize > MAX_MBUS_CODES
        {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "Unsupported Video Input CSI stub capabilities; requires ABI version 2",
            ));
        }
        Ok(())
    }

    fn supports(&self, format: MipiFormat) -> bool {
        mbus_code(format)
            .is_some_and(|code| self.mbus_codes[..self.num_mbus_codes as usize].contains(&code))
    }
}

fn mbus_code(format: MipiFormat) -> Option<u32> {
    match format {
        MipiFormat::Rgb8Bit | MipiFormat::Yuv444_8Bit => Some(0x100a), // MEDIA_BUS_FMT_RGB888_1X24
        MipiFormat::Yuv422_8Bit => Some(0x2006),                       // MEDIA_BUS_FMT_UYVY8_2X8
        MipiFormat::Yuv420_8Bit => Some(0x2015), // MEDIA_BUS_FMT_UV8_1X8, legacy CSI 0x1a
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VideoState {
    pub timing: Timing,
    pub mipi_byte_clock: u32,
    pub format: MipiFormat,
    pub color: Color,
}

impl From<VideoState> for VideoInputCsiVideo {
    fn from(video: VideoState) -> Self {
        let t = video.timing;
        Self {
            timings: VideoInputCsiTimings {
                pixel_clock: t.pixel_clock,
                h_active: t.h_active,
                h_total: t.h_total,
                h_fp: t.h_fp,
                h_sync: t.h_sw,
                h_bp: t.h_bp,
                v_active: t.v_active,
                v_total: t.v_total,
                v_fp: t.v_fp as u8,
                v_sync: t.v_sw as u8,
                v_bp: t.v_bp as u8,
                h_sync_pol: t.h_spol,
                v_sync_pol: t.v_spol,
                interlaced: 0,
                mipi_byte_clock: video.mipi_byte_clock,
                quantization: video.color.quantization,
                colorspace: video.color.colorspace,
                reserved: [0; 10],
            },
            // report_video checks capabilities before constructing the ABI payload.
            mbus_fmt_code: mbus_code(video.format).unwrap(),
            payload_encoding: match video.format.pixel_encoding() {
                PixelEncoding::Rgb => 1,
                PixelEncoding::Yuv444 => 2,
                PixelEncoding::Yuv422 => 3,
                PixelEncoding::Yuv420 => 4,
            },
            reserved: [0; 2],
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VideoInputCsiAudioInfo {
    /// Sample rate in Hz, or 0 when audio is absent.
    pub sample_rate: u32,
    pub present: u8,
    pub reserved: [u8; 3],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VideoInputCsiConfig {
    pub csi_lanes: u8,
    pub reserved1: [u8; 3],
    pub mbus_fmt_code: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VideoInputCsiStreamStatus {
    pub stream_on: u8,
    pub reserved: [u8; 3],
    pub sequence: u32,
}

const EDID_MAX_SIZE: usize = 8 * 128;

#[repr(C)]
#[derive(Clone)]
pub struct VideoInputCsiEdid {
    pub blocks: u16,
    pub reserved: u16,
    pub data: [u8; EDID_MAX_SIZE],
}

impl Default for VideoInputCsiEdid {
    fn default() -> Self {
        Self {
            blocks: 0,
            reserved: 0,
            data: [0u8; EDID_MAX_SIZE],
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VideoInputCsiEvent {
    pub r#type: u32,
    pub value: u32,
}

// ---------- ioctl definitions ----------

nix::ioctl_read!(ioc_get_caps, IOC_MAGIC, 5, VideoInputCsiCaps);
nix::ioctl_write_ptr!(ioc_report_video, IOC_MAGIC, 0, VideoInputCsiVideo);
nix::ioctl_write_ptr!(ioc_report_audio, IOC_MAGIC, 1, VideoInputCsiAudioInfo);
nix::ioctl_read!(ioc_get_config, IOC_MAGIC, 2, VideoInputCsiConfig);
nix::ioctl_read!(ioc_get_edid, IOC_MAGIC, 3, VideoInputCsiEdid);
nix::ioctl_read!(
    ioc_get_stream_status,
    IOC_MAGIC,
    4,
    VideoInputCsiStreamStatus
);

// ---------- High-level wrapper ----------

pub struct StubDevice {
    file: File,
    caps: VideoInputCsiCaps,
}

/// An event received from the kernel.
#[derive(Debug)]
pub enum StubEvent {
    EdidUpdate,
    StreamChange(bool),
    Unknown(u32),
}

impl StubDevice {
    pub fn open(path: &str) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        let mut caps = VideoInputCsiCaps::default();
        unsafe { ioc_get_caps(file.as_raw_fd(), &mut caps) }.map_err(|e| {
            io::Error::new(
                io::ErrorKind::Unsupported,
                format!("Failed to read Video Input CSI stub capabilities: {e}; kernel requires native-payload REPORT_VIDEO ABI"),
            )
        })?;
        caps.validate()?;
        log::info!(
            "Kernel stub capabilities: ABI version={}, media-bus codes={:04X?}",
            caps.abi_version,
            &caps.mbus_codes[..caps.num_mbus_codes as usize]
        );
        Ok(Self { file, caps })
    }

    pub fn supports_output_format(&self, format: MipiFormat) -> bool {
        self.caps.supports(format)
    }

    /// Atomically report the actual CSI wire format, timing and colour, or
    /// no signal. No V4L2 subdevice handle is needed by the receiver daemon.
    pub fn report_video(&self, video: Option<VideoState>) -> io::Result<()> {
        if let Some(video) = video {
            if !self.supports_output_format(video.format) {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    format!("CSI stub cannot describe {}", video.format),
                ));
            }
        }
        let report = video.map_or_else(VideoInputCsiVideo::default, VideoInputCsiVideo::from);
        unsafe { ioc_report_video(self.file.as_raw_fd(), &report) }.map_err(io::Error::from)?;
        Ok(())
    }

    /// Report audio status to the kernel; `sample_rate` is in Hz.
    pub fn report_audio(&self, sample_rate: u32, present: bool) -> io::Result<()> {
        let ai = VideoInputCsiAudioInfo {
            sample_rate,
            present: if present { 1 } else { 0 },
            reserved: [0; 3],
        };
        unsafe { ioc_report_audio(self.file.as_raw_fd(), &ai) }.map_err(io::Error::from)?;
        Ok(())
    }

    /// Read the current configuration from the kernel.
    pub fn get_config(&self) -> io::Result<VideoInputCsiConfig> {
        let mut cfg: VideoInputCsiConfig = unsafe { std::mem::zeroed() };
        unsafe { ioc_get_config(self.file.as_raw_fd(), &mut cfg) }.map_err(io::Error::from)?;
        Ok(cfg)
    }

    /// Read the current EDID from the kernel.
    pub fn get_edid(&self) -> io::Result<VideoInputCsiEdid> {
        let mut edid: VideoInputCsiEdid = VideoInputCsiEdid::default();
        unsafe { ioc_get_edid(self.file.as_raw_fd(), &mut edid) }.map_err(io::Error::from)?;
        Ok(edid)
    }

    /// Read whether the kernel-side V4L2 pipeline is currently streaming.
    pub fn stream_on(&self) -> io::Result<bool> {
        let mut status: VideoInputCsiStreamStatus = unsafe { std::mem::zeroed() };
        unsafe { ioc_get_stream_status(self.file.as_raw_fd(), &mut status) }
            .map_err(io::Error::from)?;
        Ok(status.stream_on != 0)
    }

    /// Non-blocking poll for an event. Returns None if no event available.
    pub fn poll_event(&self) -> io::Result<Option<StubEvent>> {
        use nix::poll::{PollFd, PollFlags, poll};

        let mut pfd = [PollFd::new(
            unsafe { std::os::unix::io::BorrowedFd::borrow_raw(self.file.as_raw_fd()) },
            PollFlags::POLLIN,
        )];

        let n = poll(&mut pfd, nix::poll::PollTimeout::from(0u8)).map_err(io::Error::from)?;
        if n == 0 {
            return Ok(None);
        }

        let mut ev = VideoInputCsiEvent::default();
        let buf = unsafe {
            std::slice::from_raw_parts_mut(
                &mut ev as *mut _ as *mut u8,
                std::mem::size_of::<VideoInputCsiEvent>(),
            )
        };

        use std::io::Read;
        match (&self.file).read(buf) {
            Ok(n) if n == std::mem::size_of::<VideoInputCsiEvent>() => {}
            Ok(_) => return Ok(None),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) => return Err(e),
        }

        Ok(Some(match ev.r#type {
            EVT_EDID_UPDATE => StubEvent::EdidUpdate,
            EVT_STREAM_CHANGE => StubEvent::StreamChange(ev.value != 0),
            other => StubEvent::Unknown(other),
        }))
    }
}

// ---------- Process-wide singleton ----------

static STUB: LazyLock<Mutex<Option<StubDevice>>> = LazyLock::new(|| Mutex::new(None));

pub fn init(path: &str) -> io::Result<()> {
    let dev = StubDevice::open(path)?;
    *STUB.lock().unwrap() = Some(dev);
    Ok(())
}

fn with<R>(f: impl FnOnce(&StubDevice) -> R) -> R {
    let guard = STUB.lock().unwrap();
    let dev = guard.as_ref().expect("stub::init not called");
    f(dev)
}

pub fn supports_output_format(format: MipiFormat) -> bool {
    with(|d| d.supports_output_format(format))
}

pub fn report_video(video: Option<VideoState>) -> io::Result<()> {
    with(|d| d.report_video(video))
}

pub fn report_audio(sample_rate: u32, present: bool) -> io::Result<()> {
    with(|d| d.report_audio(sample_rate, present))
}

pub fn get_config() -> io::Result<VideoInputCsiConfig> {
    with(|d| d.get_config())
}

pub fn get_edid() -> io::Result<VideoInputCsiEdid> {
    with(|d| d.get_edid())
}

pub fn stream_on() -> io::Result<bool> {
    with(|d| d.stream_on())
}

pub fn poll_event() -> io::Result<Option<StubEvent>> {
    with(|d| d.poll_event())
}
