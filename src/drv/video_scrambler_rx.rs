//! Video scrambler 1.4 / 2.2 receiver control.

use crate::config::*;
use crate::reg;
use crate::time;

pub fn dec_sw_rst() {
    reg::clear_bit(0xE107, 4);
    reg::clear_bit(0xE107, 3);
    reg::clear_bit(0xE107, 1);
    reg::clear_bit(0xE107, 0);
    time::delay_us(20);
    reg::set_bit(0xE107, 4);
    reg::set_bit(0xE107, 3);
    reg::set_bit(0xE107, 1);
    reg::set_bit(0xE107, 0);
}

pub fn re_auth_clear() {
    reg::set_bit(0xE911, 6);
    time::delay_us(10);
    reg::clear_bit(0xE911, 6);
    reg::clear_bit(0xE900, 0);
    reg::set_bit(0xE901, 6);
    time::delay_us(10);
    reg::clear_bit(0xE901, 6);
    reg::set_bit(0xE900, 0);
}

pub fn is_key_valid() -> bool {
    reg::get_bit(0xE980, 1) != 0
}

/// Reads the current video-scrambler receiver status. Callers can pass `last` to
/// suppress repeated log lines for unchanged status.
pub fn ver_get(last: &mut u8) {
    let part14 = reg::read(0xE984) & 0x03;
    let part22 = reg::read(0xE9B3) & 0x03;

    let auth14 = part14 & 0x02 != 0;
    let dec14 = part14 & 0x01 != 0;
    let edkey22 = part22 & 0x02 != 0;
    let dec22 = part22 & 0x01 != 0;

    let state = (part14 << 2) | part22;
    if *last == state {
        return;
    }
    *last = state;

    log::info!(
        "Video scrambler status: 1.4 authentication={}, 1.4 decryption={}, 2.2 key={}, 2.2 decryption={}",
        if auth14 { "success" } else { "failed" },
        if dec14 { "success" } else { "failed" },
        if edkey22 { "ready" } else { "failed" },
        if dec22 { "success" } else { "failed" },
    );
}

pub fn init(opt: u8) {
    reg::clear_bit(0xE800, 1);
    reg::clear_bit(0xE931, 0);

    if is_key_valid() {
        log::info!("Video scrambler key is valid");
    } else {
        log::error!("Video scrambler key is invalid");
    }

    if opt & SCRAMBLER1X != 0 {
        reg::or(0xE114, 0xc0);
        reg::or(0xE901, 0x80);
    }
    if opt & SCRAMBLER2X != 0 {
        reg::set_bit(0xE900, 1);
        reg::set_bit(0xE900, 2);
        reg::set_bit(0xE911, 0);
        reg::set_bit(0xE912, 2);
        reg::clear_bit(0xE912, 3);
        reg::write(0xE916, 0x02);
        reg::write(0xE915, 0);
        reg::write(0xE914, 0);
    }
}

pub fn disable() {
    reg::clear_bit(0xE906, 6);
    reg::set_bit(0xE906, 5);
    log::info!("Video scrambler versions 1.4 and 2.2 disabled");
}
