//! Frequency meter, video-check, format mapping, crystal/xtal config, packet
//! header probe.

use crate::config::*;
use crate::reg;
use crate::time;
use crate::timing::Timing;

// ---- POS_INDEX ----
pub const HTOTAL_POS: u8 = 0;
pub const HACTIVE_POS: u8 = 1;
pub const HFP_POS: u8 = 2;
pub const HSW_POS: u8 = 3;
pub const HBP_POS: u8 = 4;
pub const VTOTAL_POS: u8 = 5;
pub const VACTIVE_POS: u8 = 6;
pub const VFP_POS: u8 = 7;
pub const VSW_POS: u8 = 8;
pub const VBP_POS: u8 = 9;
pub const HSPOL_POS: u8 = 10;
pub const VSPOL_POS: u8 = 11;

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

pub fn video_check_single_get(para: u8) -> u32 {
    match para {
        HTOTAL_POS => (pack_16(reg::read(0xE592), reg::read(0xE593)) as u32) << 1,
        HACTIVE_POS => (pack_16(reg::read(0xE596), reg::read(0xE597)) as u32) << 1,
        HFP_POS => (pack_16(reg::read(0xE58C), reg::read(0xE58D)) as u32) << 1,
        HSW_POS => (pack_16(reg::read(0xE586), reg::read(0xE587)) as u32) << 1,
        HBP_POS => (pack_16(reg::read(0xE58A), reg::read(0xE58B)) as u32) << 1,
        VTOTAL_POS => pack_16(reg::read(0xE594), reg::read(0xE595)) as u32,
        VACTIVE_POS => pack_16(reg::read(0xE598), reg::read(0xE599)) as u32,
        VFP_POS => pack_16(reg::read(0xE590), reg::read(0xE591)) as u32,
        VSW_POS => pack_16(reg::read(0xE588), reg::read(0xE589)) as u32,
        VBP_POS => pack_16(reg::read(0xE58E), reg::read(0xE58F)) as u32,
        HSPOL_POS => reg::get_bit(0xE580, 4) as u32,
        VSPOL_POS => reg::get_bit(0xE580, 5) as u32,
        _ => 0,
    }
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
    Timing {
        pixel_clock: pixel_clock_get(),
        h_total: video_check_single_get(HTOTAL_POS) as u16,
        h_active: video_check_single_get(HACTIVE_POS) as u16,
        h_fp: video_check_single_get(HFP_POS) as u16,
        h_sw: video_check_single_get(HSW_POS) as u16,
        h_bp: video_check_single_get(HBP_POS) as u16,
        v_total: video_check_single_get(VTOTAL_POS) as u16,
        v_active: video_check_single_get(VACTIVE_POS) as u16,
        v_fp: video_check_single_get(VFP_POS) as u16,
        v_sw: video_check_single_get(VSW_POS) as u16,
        v_bp: video_check_single_get(VBP_POS) as u16,
        h_spol: video_check_single_get(HSPOL_POS) != 0,
        v_spol: video_check_single_get(VSPOL_POS) != 0,
    }
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
