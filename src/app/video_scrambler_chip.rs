//! Video scrambler key + chip-version probe.
//!
//! Video scrambler keys are expected to be provisioned by the bootloader; this
//! module brings up the register sequence to probe the scrambler key-valid bit.

use crate::reg;
use crate::time;

fn scrambler_key_chk() -> bool {
    reg::clear_bit(0xE931, 1);
    let a = (reg::read(0xE980) & 0x02) != 0;
    time::delay_ms(1);
    reg::set_bit(0xE931, 1);
    let b = (reg::read(0xE980) & 0x02) != 0;
    if !a && b {
        log::info!("Video scrambler key supports version 1.4 only");
        true
    } else {
        false
    }
}

fn scrambler_load_key() -> bool {
    reg::write(0xE92D, 0x85);
    reg::or(0xE92B, 0x40);
    reg::and(0xE92B, !0x40);
    reg::write(0xE068, 0x05);
    reg::write(0xE069, 0x00);
    reg::write(0xE06A, 0x00);
    reg::write(0xE065, 0x00);
    reg::write(0xE066, 0x08);
    reg::write(0xE067, 0x00);
    reg::or(0xE060, 0x02);
    time::delay_ms(10);
    reg::and(0xE060, !0x02);

    for n in 1u8..=0xFF {
        time::delay_ms(1);
        if n >= 0x14 && (reg::read(0xE980) & 0x01) != 0 {
            break;
        }
    }
    if reg::read(0xE980) & 0x01 != 0 {
        log::info!("Video scrambler key loaded");
        true
    } else {
        log::error!("Failed to load video scrambler key");
        false
    }
}

/// Probe video-scrambler keys at boot. Returns `true` if the link should be
/// treated as scrambler 1.4-only — callers use this to gate 2.2 attempts.
pub fn check_keys() -> bool {
    if !scrambler_load_key() {
        // Key loading failed — the scrambler version is indeterminate.
        // Assume a scrambler 1.4-only link so the additional clock-stability
        // settle pass still runs.
        log::warn!("Video scrambler key loading failed; assuming version 1.4 only");
        return true;
    }
    let key_chk = scrambler_key_chk();
    if key_chk {
        reg::clear_bit(0xE81B, 7);
        reg::set_bit(0xE81B, 6);
        reg::clear_bits(0xE115, 3, 0);
    }
    key_chk
}
