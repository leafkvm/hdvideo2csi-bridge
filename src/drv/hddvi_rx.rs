//! Video Input receiver low-level: PHY, PLL cal, video-check debouncer,
//! AVI/GCP-change interrupts, EDID glue.

use crate::app::rx::Rx;
use crate::config::*;
use crate::drv::system;
use crate::format::{BitDepth, MipiFormat, PixelEncoding};
use crate::reg;
use crate::time;

/// Returns `true` when something has gone wrong on the receiver and the caller
/// should abandon the in-progress operation (e.g. EQ scan).
pub fn abnormal_deal() -> bool {
    !clk_stable()
}

/// Busy-wait while polling for abnormal conditions.
pub fn wait_loop(delay_cnt: u8) {
    for _ in 0..=delay_cnt {
        time::delay_ms(1);
        if abnormal_deal() {
            break;
        }
    }
}

fn pll_cali_set() {
    reg::and(0xE220, 0xDF);
    reg::and(0xE220, 0xBF);
    reg::or(0xE220, 0x02);
    reg::or(0xE220, 0x04);
    reg::or(0xE220, 0x08);
    reg::and(0xE10C, 0xFC);
    reg::or(0xE10C, 0x03);
    reg::and(0xE10F, 0xDF);
    reg::or(0xE10F, 0x20);
    reg::and(0xE734, 0x7F);
    reg::or(0xE734, 0x80);
}

/// Calibrate the receiver PLL; the error says which step failed.
pub fn pll_cali() -> Result<(), &'static str> {
    log::debug!("Receiver PLL calibration started");
    pll_cali_set();
    let mut cnt: u8 = 0;
    loop {
        cnt += 1;
        wait_loop(5);
        if cnt >= 20 || reg::read(0xE735) & 0x02 != 0 {
            break;
        }
    }
    if (reg::read(0xE735) & 0x03) != 0x02 {
        return Err("receiver PLL calibration did not finish");
    }
    if reg::get_bit(0xE735, 3) == 0 {
        return Err("receiver PLL did not lock after calibration");
    }
    log::debug!(
        "Receiver PLL calibration: Register E736=0x{:02X}, Register E737=0x{:02X}",
        reg::read(0xE736),
        reg::read(0xE737)
    );
    vco_div_reset();
    Ok(())
}

pub fn csc_src_sel() {
    reg::write(0xE5D7, 0x01);
}

pub fn half_pix_pll_set() {
    reg::or(0xE227, 0x08);
    reg::or(0xE227, 0x20);
    reg::or(0xE227, 0x10);
    reg::and(0xE227, 0xFB);
    reg::and(0xE227, 0xFC);
}

pub fn phy_cal() {
    let mut cnt: u8 = 0;
    loop {
        reg::and(0xE714, 0xFE);
        reg::or(0xE714, 0x01);
        time::delay_ms(5);
        cnt += 1;
        if cnt >= 3 || reg::read(0xE71A) & 0x08 != 0 {
            break;
        }
    }
    log::debug!(
        "Receiver PHY termination: Register E71B=0x{:02X}",
        reg::read(0xE71B)
    );
}

pub fn phy_power_set(on: bool) {
    if on {
        reg::or(0xE207, 0x80);
        reg::and(0xE207, 0xF0);
    } else {
        reg::and(0xE207, 0x7F);
        reg::or(0xE207, 0x0F);
    }
}

pub fn phy_soft_reset() {
    reg::and(0xE10F, 0xBF);
    reg::and(0xE10F, 0xF7);
    reg::and(0xE10D, 0xF7);
    reg::or(0xE10F, 0x40);
    reg::or(0xE10F, 0x08);
    reg::or(0xE10D, 0x08);
    time::delay_ms(1);
}

pub fn pi_enable(on: bool) {
    if on {
        reg::write(0xE21B, 0x00);
    } else {
        reg::write(0xE21B, 0x0F);
    }
}

pub fn hpd_set(level: Level) {
    match level {
        Level::High => {
            reg::or(0xE808, 0x40);
            reg::write(0xE29D, 0x04);
        }
        Level::Low => {
            reg::and(0xE808, 0xBF);
            reg::write(0xE29D, 0xA4);
        }
    }
}

pub fn depacket_rst() {
    reg::and(0xE105, 0xFD);
    reg::or(0xE105, 0x02);
    reg::and(0xE105, 0xFE);
    reg::or(0xE105, 0x01);
    wait_loop(10);
}

pub fn vco_div_reset() {
    reg::or(0xE221, 0x02);
    reg::and(0xE221, 0xFD);
}

pub fn rxpll_clk_set() {
    reg::write(0xE225, 0x50);
}

pub fn phy_shut() {
    phy_power_set(false);
    pi_enable(false);
}

/// Live RxPLL FMCLK clock-stability level: bit 0 of the 0xE86E status register.
pub fn clk_stable() -> bool {
    (reg::read(0xE86E) & 0x01) != 0
}

/// Update the clock-stability debouncer from the live status register.
pub fn chk_clk_stb(rx: &mut Rx) -> Stability {
    if clk_stable() {
        rx.clk_stb_dbnc.bump_good(20)
    } else {
        rx.clk_stb_dbnc.bump_bad(200)
    }
}

/// Live video-check timing-stable level: bit 2 of the 0xE580 status register.
pub fn video_stable() -> bool {
    (reg::read(0xE580) & 0x04) != 0
}

/// Plausibility bounds for a measured active resolution (pixels / lines). Used
/// to reject the transient garbage the video-check block latches on hot-plug.
/// Both the settle check and the interlace scan gate on this so they agree on
/// what counts as a sane measurement.
pub fn resolution_sane(h: u16, v: u16) -> bool {
    const MIN: u16 = 80;
    const MAX: u16 = 5220;
    (MIN < h && h < MAX) && (MIN < v && v < MAX)
}

pub fn pll_locked() -> bool {
    reg::read(0xE5F1) & 0xF >= 0x09
}

pub fn video_chk_stable(rx: &mut Rx) -> Stability {
    if video_stable() {
        rx.vid_chk_dbnc.bump_good(20)
    } else {
        rx.vid_chk_dbnc.bump_bad(200)
    }
}

/// Returns `(encoding, depth, format, changed)`.
pub fn format_update(prev_rx_format: MipiFormat) -> (PixelEncoding, BitDepth, MipiFormat, bool) {
    let avi_yy = reg::read(0xE8D0);
    let gcp_cd_pp = reg::read(0xE8D1);
    log::debug!(
        "Receiver format signaling: AVI=0x{:02X}, GCP=0x{:02X}",
        avi_yy,
        gcp_cd_pp
    );
    let pe_code = (avi_yy & 0x70) >> 4;
    let encoding = PixelEncoding::from_chip(pe_code).unwrap_or_else(|| {
        log::error!(
            "Unsupported receiver pixel encoding: 0x{:02X}; using RGB",
            pe_code
        );
        PixelEncoding::Rgb
    });
    let depth = BitDepth::from_chip((gcp_cd_pp & 0xF0) >> 4);
    let rx_format = MipiFormat::from_encoding_depth(encoding, depth);
    let changed = rx_format != prev_rx_format;
    (encoding, depth, rx_format, changed)
}

pub fn vid_stb_frm_set() {
    reg::write(0xE5A9, 0x03);
}
pub fn sync_mode_sel() {
    reg::and(0xE828, 0xFE);
}
pub fn deskew_set() {
    reg::or(0xE811, 0x01);
}
pub fn error_detect_set() {
    reg::write(0xE82A, 0x9C);
    reg::write(0xE82B, 0x40);
    reg::or(0xE815, 0x80);
    reg::and(0xE819, 0x0F);
    reg::or(0xE819, 0x60);
}

pub fn setting_init() {
    reg::and(0xE81B, 0x7F);
    reg::or(0xE81B, 0x40);
    reg::and(0xE115, 0xF0);
    phy_power_set(true);
    pi_enable(true);
    csc_src_sel();
    phy_cal();
    error_detect_set();
    vid_stb_frm_set();
    sync_mode_sel();
    deskew_set();
    rxpll_clk_set();
}

pub fn avi_chg_int_clr() {
    reg::or(0xE858, 0x10);
    reg::and(0xE858, 0xEF);
}
pub fn avi_chg_int_en() {
    reg::or(0xE858, 0x20);
    avi_chg_int_clr();
}
pub fn avi_chg_int_get() -> bool {
    if reg::get_bit(0xE8F3, 5) != 0 {
        avi_chg_int_clr();
        true
    } else {
        false
    }
}
pub fn gcp_chg_int_clr() {
    reg::or(0xE858, 0x04);
    reg::and(0xE858, 0xFB);
}
pub fn gcp_chg_int_en() {
    reg::or(0xE858, 0x08);
    gcp_chg_int_clr();
}
pub fn gcp_chg_int_get() -> bool {
    if reg::get_bit(0xE8F3, 4) != 0 {
        gcp_chg_int_clr();
        true
    } else {
        false
    }
}

pub fn avi_gcp_chg_int_en() {
    avi_chg_int_en();
    gcp_chg_int_en();
}

pub fn avi_rcv_set() {
    system::pkt_hd_header_set(0x82);
}

pub fn chk_avi_rcv(rx: &mut Rx) -> Stability {
    if system::pkt_acphdr_get(system::PKT_AVI_INFOFRAME) {
        rx.avi_rcv_dbnc.bump_good(20)
    } else {
        rx.avi_rcv_dbnc.bump_bad(200)
    }
}

/// Scan video-check registers until the active resolution settles. Updates
/// `rx.interlace` and `rx.vid_chk_scan` on success. Returns `false` if the
/// resolution kept changing or fell outside sane bounds.
pub fn vid_chk_scan(rx: &mut Rx) -> bool {
    let mut prev_h = system::video_check_single_get(system::HACTIVE_POS) as u16;
    let mut prev_v = system::video_check_single_get(system::VACTIVE_POS) as u16;
    let (mut cur_h, mut cur_v) = (prev_h, prev_v);
    for _ in 0..50 {
        time::delay_ms(5);
        cur_h = system::video_check_single_get(system::HACTIVE_POS) as u16;
        cur_v = system::video_check_single_get(system::VACTIVE_POS) as u16;
        if prev_h != cur_h || prev_v != cur_v {
            return false;
        }
        if !resolution_sane(cur_h, cur_v) {
            return false;
        }
        prev_h = cur_h;
        prev_v = cur_v;
    }

    let interlace = matches!((cur_h, cur_v), (1920, 540) | (1440, 288) | (1440, 240));
    rx.interlace = interlace;
    rx.vid_chk_scan = true;
    log::debug!("Receiver timing scan: interlaced={}", interlace);
    true
}
