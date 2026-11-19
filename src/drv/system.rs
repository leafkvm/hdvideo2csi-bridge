//! Frequency meter, video-check, format mapping, crystal/xtal config, packet
//! header probe.

use crate::config::*;
use crate::reg;
use crate::time;
use crate::timing::Timing;

// ---- Packet types ----
pub const PKT_CUSTOM_INFOFRAME: u8 = 0x77;
pub const PKT_VENDOR_SPECIFIC_INFOFRAME: u8 = 0x81;
pub const PKT_AVI_INFOFRAME: u8 = 0x82;
pub const PKT_GENERAL_CONTROL: u8 = 0x03;
pub const PKT_AUDIO_INFOFRAME: u8 = 0x84;

// ---- Frequency meter clock source IDs ----
pub const AD_RXPLL_HALF_PIX_CLK: u8 = 0x0F;
/// Audio PLL output, 256 × fs.
pub const AD_AUDPLL_256FS_CLK: u8 = 0x1D;

pub const FREQ_METER_1: u8 = 1;
pub const FREQ_METER_2: u8 = 2;

/// Measured frame rate in millihertz, from the frame-period counter of
/// the video check (40 MHz ticks per frame). Zero when the counter is empty or
/// saturated, i.e. no vertical sync arrived.
pub fn frame_rate_mhz() -> u32 {
    let sum = ((reg::read(0xE583) as u64) << 16)
        | ((reg::read(0xE584) as u64) << 8)
        | reg::read(0xE585) as u64;
    if sum == 0 || sum == 0xFF_FFFF {
        return 0;
    }
    (40_000_000_000 / sum) as u32
}

/// Pixel clock (kHz) measured with frequency meter 1 on the RxPLL half-pixel
/// clock. The meter reports the *half*-pixel rate, so the full pixel clock is
/// twice the reading.
pub fn pixel_clock_get() -> u32 {
    fm_clk_set(FREQ_METER_1, AD_RXPLL_HALF_PIX_CLK);
    time::delay_ms(5);
    fm_clk_get(FREQ_METER_1) * 2
}

/// Read the full latched video-check measurement off the chip as a fresh
/// [`Timing`], including the frequency-meter `pixel_clock` (see
/// [`pixel_clock_get`]).
pub fn video_check_all_get() -> Timing {
    let pixel_clock = pixel_clock_get();
    Timing {
        pixel_clock,
        ..video_check_geometry_get()
    }
}

/// Registers can change at a field boundary; acquisition brackets this with
/// field-indicator reads and rejects snapshots whose porches do not add up.
pub fn video_check_geometry_get() -> Timing {
    video_check_get().1
}

/// The video-check status (E580) and geometry, read as one 26-byte block
/// E580..E599 in a single I2C transaction: about 2.6 ms at 100 kHz, against
/// 11-12 ms for one register at a time. Horizontal values count 2 pixels.
pub fn video_check_get() -> (u8, Timing) {
    let mut r = [0u8; 26];
    reg::read_block(0xE580, &mut r);
    let at = |addr: u16| {
        let i = usize::from(addr - 0xE580);
        pack_16(r[i], r[i + 1])
    };
    let status = r[0];
    let timing = Timing {
        pixel_clock: 0,
        h_total: at(0xE592) << 1,
        h_active: at(0xE596) << 1,
        h_fp: at(0xE58C) << 1,
        h_sw: at(0xE586) << 1,
        h_bp: at(0xE58A) << 1,
        v_total: at(0xE594),
        v_active: at(0xE598),
        v_fp: at(0xE590),
        v_sw: at(0xE588),
        v_bp: at(0xE58E),
        h_spol: status & 0x10 != 0,
        v_spol: status & 0x20 != 0,
        second_field: None,
    };
    (status, timing)
}

pub fn pkt_hd_header_set(opt: u8) {
    reg::write(0xE80A, opt);
    time::delay_ms(5);
}

/// The selected packet header reads all zero: nothing of that type arrives.
/// With the AVI header selected, that is DVI signalling.
pub fn pkt_header_empty() -> bool {
    (0xE875..=0xE877).all(|r| reg::read(r) == 0)
}

pub fn pkt_acphdr_get(ty: u8) -> bool {
    let b75 = reg::read(0xE875);
    let b76 = reg::read(0xE876);
    let b77 = reg::read(0xE877);
    match ty {
        PKT_CUSTOM_INFOFRAME => b75 == 0x77,
        PKT_VENDOR_SPECIFIC_INFOFRAME => b75 == 0x81 && b76 == 0x01 && b77 == 0x1b,
        PKT_AVI_INFOFRAME => {
            (b75 == 0x82 && (b76 == 0x02 || b76 == 0x03) && b77 == 0x0d)
                || (b75 == 0 && b76 == 0 && b77 == 0)
        }
        PKT_GENERAL_CONTROL => b75 == 0x03 && b76 == 0 && b77 == 0,
        PKT_AUDIO_INFOFRAME => b75 == 0x84 && b76 == 0x01 && b77 == 0x0a,
        _ => false,
    }
}

pub fn xtal_set() {
    reg::clear_bit(0xE201, 7);
    reg::write(0xE5FE, 0x02);

    reg::write(0xE725, 0x00);
    reg::write(0xE726, 0x9c);
    reg::write(0xE727, 0x40);
    reg::write(0xE722, 0xc7);

    reg::write(0xE725, 0x00);
    reg::write(0xE726, 0x9c);
    reg::write(0xE727, 0x40);
    reg::write(0xE722, 0xc7);

    reg::write(0xE545, 0x00);
    reg::write(0xE546, 0x9c);
    reg::write(0xE547, 0x40);
    reg::write(0xE542, 0xc7);

    reg::write(0xE555, 0x00);
    reg::write(0xE556, 0x9c);
    reg::write(0xE557, 0x40);
    reg::write(0xE552, 0xc7);
}

pub fn fm_clk_set(fm: u8, src: u8) {
    match fm {
        FREQ_METER_1 => {
            reg::and(0xE540, 0x80);
            reg::or(0xE540, src);
        }
        FREQ_METER_2 => {
            reg::and(0xE550, 0x80);
            reg::or(0xE550, src);
        }
        _ => {}
    }
}

pub fn fm_clk_get(fm: u8) -> u32 {
    match fm {
        FREQ_METER_1 => {
            reg::or(0xE540, 0x80);
            let v = pack_24(
                reg::read(0xE548) & 0x0f,
                reg::read(0xE549),
                reg::read(0xE54A),
            );
            reg::and(0xE540, 0x7f);
            v
        }
        FREQ_METER_2 => {
            reg::or(0xE550, 0x80);
            let v = pack_24(
                reg::read(0xE558) & 0x0f,
                reg::read(0xE559),
                reg::read(0xE55A),
            );
            reg::and(0xE550, 0x7f);
            v
        }
        _ => 0,
    }
}
