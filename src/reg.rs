//! Register-access primitives.
//!
//! Free functions that go through a singleton ioctl device wrapped in a
//! Mutex. Pass the raw u16 address.

use std::sync::{LazyLock, Mutex};

use crate::kernel_proxy_reg::RegisterAccess;

static REG: LazyLock<Mutex<Option<RegisterAccess>>> = LazyLock::new(|| Mutex::new(None));

pub fn init(path: &str) -> std::io::Result<()> {
    let reg = RegisterAccess::open(path)?;
    *REG.lock().unwrap() = Some(reg);
    Ok(())
}

#[inline]
pub fn read(addr: u16) -> u8 {
    let mut guard = REG.lock().unwrap();
    let reg = guard.as_mut().expect("reg::init not called");
    reg.read(addr).unwrap_or_else(|e| {
        log::error!("Failed to read Register {:04X}: {}", addr, e);
        0
    })
}

#[inline]
pub fn write(addr: u16, val: u8) {
    let mut guard = REG.lock().unwrap();
    let reg = guard.as_mut().expect("reg::init not called");
    if let Err(e) = reg.write(addr, val) {
        log::error!(
            "Failed to write Register {:04X} with value 0x{:02X}: {}",
            addr,
            val,
            e
        );
    }
}

/// REG |= mask
#[inline]
pub fn or(addr: u16, mask: u8) {
    let v = read(addr);
    write(addr, v | mask);
}

/// REG &= mask
#[inline]
pub fn and(addr: u16, mask: u8) {
    let v = read(addr);
    write(addr, v & mask);
}

/// SET_BIT(REG, bit)
#[inline]
pub fn set_bit(addr: u16, bit: u8) {
    or(addr, 1 << bit);
}

/// CLEAR_BIT(REG, bit)
#[inline]
pub fn clear_bit(addr: u16, bit: u8) {
    and(addr, !(1u8 << bit));
}

/// GET_BIT(REG, bit) — returns 0 or 1.
#[inline]
pub fn get_bit(addr: u16, bit: u8) -> u8 {
    (read(addr) >> bit) & 1
}

/// Mask for the bit field [m:n] (inclusive, m >= n).
#[inline]
fn field_mask(m: u8, n: u8) -> u8 {
    (0xFFu8 >> (7 - m + n)) << n
}

/// CLEAR_BITS(REG, m, n) — clear bits [m:n].
#[inline]
pub fn clear_bits(addr: u16, m: u8, n: u8) {
    and(addr, !field_mask(m, n));
}

/// LET_BITS(REG, m, n, new) — write `new` into bit field [m:n].
#[inline]
pub fn let_bits(addr: u16, m: u8, n: u8, new: u8) {
    let mask = field_mask(m, n);
    let v = read(addr);
    write(addr, (v & !mask) | ((new << n) & mask));
}
