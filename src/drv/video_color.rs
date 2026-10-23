//! Native-video bypass and input colour signalling.

use crate::avi::{self, Avi, Range};
use crate::format::PixelEncoding;
use crate::reg;
use crate::stub::{Color, colorspace};
use crate::time;

fn disable() {
    reg::write(0xE565, 0x00);
    reg::write(0xE566, 0x00);
    reg::write(0xE567, 0x00);
    reg::write(0xE568, 0x00);
}

fn detect_rgb_range() -> Option<Range> {
    // The detector sees the CSC datapath. Its old configuration must already
    // be bypassed; otherwise a full->limited switch can still detect "full".
    reg::clear_bit(0xE560, 0); // active pixels, not blanking
    reg::clear_bit(0xE561, 7); // finite detection window
    reg::write(0xE562, 0x03);
    reg::let_bits(0xE561, 6, 0, 0x7F);
    reg::clear_bit(0xE560, 1);
    time::delay_ms(5);
    reg::set_bit(0xE560, 1);
    for _ in 0..20 {
        time::delay_ms(10);
        // Read valid and result together; an unfinished result is not full.
        match reg::read(0xE55F) & 3 {
            2 => return Some(Range::Limited),
            3 => return Some(Range::Full),
            _ => {}
        }
    }
    log::warn!("RGB range detection timed out; using AVI timing default");
    None
}

fn read_avi(rx_enc: PixelEncoding) -> Option<Avi> {
    let packet = std::array::from_fn(|i| reg::read(0xE875 + i as u16));
    log::debug!("AVI packet: {:02X?}", packet);
    let avi = Avi::parse(packet, rx_enc);
    if avi.is_none() {
        log::warn!("AVI color signaling is invalid: encoding={}", rx_enc);
    }
    avi
}

/// Bypass all matrix, range and chroma-resampling stages, then describe the
/// unmodified input samples. Range detection only affects metadata.
pub fn configure_bypass(rx_enc: PixelEncoding, v_active: u16) -> Color {
    disable();
    let avi = read_avi(rx_enc);
    // Explicit AVI Q takes precedence over sample extrema. With Q=Default
    // retain pixel detection for PC sources that send full RGB with a CE VIC.
    // This heuristic cannot distinguish full-range content entirely inside
    // 16..235 from limited RGB. Never apply it to YCbCr (which uses AVI YQ).
    let detected = if rx_enc == PixelEncoding::Rgb && avi.is_some_and(Avi::rgb_range_is_default) {
        detect_rgb_range()
    } else {
        None
    };
    let input = avi::input_color(avi, rx_enc, v_active, detected);
    log::info!(
        "Native video output: encoding={}, input color={:?}",
        rx_enc,
        input
    );
    if input.colorspace == colorspace::DEFAULT {
        log::warn!(
            "AVI colorimetry is unsupported or missing; reporting unknown output colorimetry"
        );
    }
    log::debug!(
        "Color conversion register values: Register E565 through Register E568={:02X?}",
        std::array::from_fn::<_, 4, _>(|i| reg::read(0xE565 + i as u16))
    );
    input
}
