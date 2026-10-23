//! EDID slave control. The EDID payload is sourced from the kernel by the
//! application layer (see [`crate::app`]); this module never touches the stub.
//! It only programs an already-validated [`Edid`] into the receiver and toggles
//! the DDC slave. Without a kernel EDID, HPD/DDC must stay inactive so the
//! upstream source sees no sink.

use crate::reg;

pub const EDID_BLOCK_SIZE: usize = 128;
pub const EDID_SIZE_4BLOCK: usize = 512;
pub const EDID_MAX_SUPPORTED_BLOCKS: usize = EDID_SIZE_4BLOCK / EDID_BLOCK_SIZE;

/// A validated EDID payload ready to be programmed into the receiver.
///
/// Built by the application layer from the raw kernel EDID; the block count is
/// already known to be in `1..=EDID_MAX_SUPPORTED_BLOCKS`, so [`edid_set`]
/// needs no further checks.
#[derive(Clone, Copy)]
pub struct Edid {
    blocks: usize,
    data: [u8; EDID_SIZE_4BLOCK],
}

impl Edid {
    /// Build a programmable EDID from a raw kernel buffer. Returns `None` when
    /// the buffer is empty or has more blocks than the receiver supports —
    /// in both cases the receiver must stay unconnected.
    pub fn from_kernel(data: &[u8], blocks: usize) -> Option<Self> {
        if blocks == 0 || blocks > EDID_MAX_SUPPORTED_BLOCKS {
            return None;
        }
        let bytes = blocks * EDID_BLOCK_SIZE;
        let mut buf = [0u8; EDID_SIZE_4BLOCK];
        buf[..bytes].copy_from_slice(&data[..bytes]);
        Some(Self { blocks, data: buf })
    }
}

impl std::fmt::Debug for Edid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Edid")
            .field("blocks", &self.blocks)
            .finish()
    }
}

/// Program a validated kernel EDID into the receiver's EDID slave.
///
/// Only the receiver state machine calls this — see [`crate::app::rx`]. The
/// caller must have held HPD low with the DDC slave disabled (see
/// [`crate::app::rx::Rx::disconnect`]).
///
/// The EDID SRAM is streamed through an auto-increment pointer that wraps at
/// the end of the selected window (256 bytes for up to two blocks, 512 for
/// more). Writing 0xE807 does *not* reset that pointer: the only thing that
/// brings it back to 0 is wrapping. So this always streams the whole window,
/// zero-padded, exactly as the vendor firmware does. Streaming only
/// `blocks * 128` bytes for a one-block EDID left the pointer resting at 128,
/// and the next two-block EDID then landed rotated by a block — the source
/// read the CEA extension at offset 0 and rejected the EDID.
pub fn edid_set(edid: &Edid) {
    log::info!(
        "Receiver EDID programmed from kernel: blocks={}",
        edid.blocks
    );
    reg::write(0xE807, 0x00);
    reg::write(0xE808, 0x81); // plus_en | hpd | src_rd_en | access edid
    reg::write(0xE808, 0x01); // EDID_ADDR_PLUS_EN off

    let window = if edid.blocks > 2 {
        reg::or(0xE806, 0x40);
        EDID_SIZE_4BLOCK
    } else {
        reg::and(0xE806, !0x40);
        2 * EDID_BLOCK_SIZE
    };

    for &b in &edid.data[..window] {
        reg::write(0xE894, b);
    }
}

/// Enable or disable the DDC slave. Only the receiver state machine calls
/// this — see [`crate::app::rx`].
pub fn ddc_slave_set(enable: bool) {
    if enable {
        reg::set_bit(0xE808, 1);
        reg::clear_bit(0xE808, 0);
    } else {
        reg::clear_bit(0xE808, 1);
    }
}
