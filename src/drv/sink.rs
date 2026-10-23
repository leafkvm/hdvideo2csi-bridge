//! Pushes detected video / audio state to the kernel stub via
//! the Video Input CSI stub device.
//!
//! Each `report_*` function dedups against the last reported event via the
//! shared `last_event` slot the caller owns; a call that matches the last
//! reported state is a no-op.

use crate::drv::audio::Rate;
use crate::format::MipiFormat;
use crate::stub;
use crate::stub::{Color, VideoState};
use crate::timing::Timing;
use std::io;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SinkEvent {
    VideoReady(VideoState),
    VideoDisappear,
    /// Live audio, with its sample rate in Hz for the kernel ABI.
    AudioReady(u32),
    AudioDisappear,
}

/// Dedup against `last`: if `event` matches what was last reported, return
/// `false` (the caller should no-op). Otherwise latch `event`, log it, and
/// return `true` so the caller proceeds to push the report.
fn arm(last: &mut Option<SinkEvent>, event: SinkEvent, desc: &str) -> bool {
    if *last == Some(event) {
        return false;
    }
    *last = Some(event);
    log::debug!("Kernel sink event: {}", desc);
    true
}

pub fn report_video_ready(
    last: &mut Option<SinkEvent>,
    timing: &Timing,
    mipi_byte_clock: u32,
    format: MipiFormat,
    color: Color,
) -> io::Result<()> {
    let video = VideoState {
        timing: output_timing(timing, format),
        mipi_byte_clock,
        format,
        color,
    };
    let event = SinkEvent::VideoReady(video);
    if *last != Some(event) {
        // Latch only successful reports. Format-only and colour-only changes
        // are real source changes even if video never disappeared in between.
        stub::report_video(Some(video))?;
        *last = Some(event);
        log::debug!("Kernel sink event: video ready, state={:?}", video);
    }
    Ok(())
}

fn output_timing(timing: &Timing, format: MipiFormat) -> Timing {
    // Video Input YUV420 is measured at half horizontal resolution/clock. Scale
    // both together so the reported full-resolution timing retains its fps.
    let mul = if format.is_yuv420() { 2 } else { 1 };
    Timing {
        pixel_clock: timing.pixel_clock * mul as u32,
        h_total: timing.h_total * mul,
        h_active: timing.h_active * mul,
        h_fp: timing.h_fp * mul,
        h_sw: timing.h_sw * mul,
        h_bp: timing.h_bp * mul,
        ..*timing
    }
}

pub fn report_video_disappear(last: &mut Option<SinkEvent>) -> io::Result<()> {
    if *last == Some(SinkEvent::VideoDisappear) {
        return Ok(());
    }
    stub::report_video(None)?;
    *last = Some(SinkEvent::VideoDisappear);
    log::debug!("Kernel sink event: video unavailable");
    if let Err(e) = stub::report_audio(0, false) {
        log::error!("Failed to report inactive audio to kernel: {}", e);
    }
    Ok(())
}

pub fn report_audio_ready(last: &mut Option<SinkEvent>, rate: Rate) {
    let sample_rate_hz = rate.hz();
    if !arm(last, SinkEvent::AudioReady(sample_rate_hz), "audio ready") {
        return;
    }
    if let Err(e) = stub::report_audio(sample_rate_hz, true) {
        log::error!("Failed to report active audio to kernel: {}", e);
    }
}

pub fn report_audio_disappear(last: &mut Option<SinkEvent>) {
    if !arm(last, SinkEvent::AudioDisappear, "audio unavailable") {
        return;
    }
    if let Err(e) = stub::report_audio(0, false) {
        log::error!("Failed to report inactive audio to kernel: {}", e);
    }
}
