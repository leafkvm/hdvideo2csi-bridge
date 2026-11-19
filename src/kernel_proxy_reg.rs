use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::io::AsRawFd;

const VIDEO_CHIP_IOC_MAGIC: u8 = b'L';

#[repr(C)]
struct VideoChipRegOp {
    reg: u16,
    val: u8,
    _pad: u8,
}

/// Longest block read the kernel driver takes; a block also may not cross a
/// register page.
pub const BLOCK_MAX: usize = 64;

#[repr(C)]
struct VideoChipBlockOp {
    reg: u16,
    len: u16,
    val: [u8; BLOCK_MAX],
}

nix::ioctl_readwrite!(
    video_chip_ioc_read_block,
    VIDEO_CHIP_IOC_MAGIC,
    2,
    VideoChipBlockOp
);
nix::ioctl_readwrite!(
    video_chip_ioc_read_reg,
    VIDEO_CHIP_IOC_MAGIC,
    0,
    VideoChipRegOp
);
nix::ioctl_write_ptr!(
    video_chip_ioc_write_reg,
    VIDEO_CHIP_IOC_MAGIC,
    1,
    VideoChipRegOp
);

pub struct RegisterAccess {
    file: File,
}

impl RegisterAccess {
    pub fn open(path: &str) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        Ok(Self { file })
    }

    pub fn write(&self, reg: u16, value: u8) -> io::Result<()> {
        log::trace!("Register {:04X} write: value=0x{:02X}", reg, value);
        let op = VideoChipRegOp {
            reg,
            val: value,
            _pad: 0,
        };
        // SAFETY: op is a valid VideoChipRegOp and the kernel driver validates the ioctl.
        unsafe { video_chip_ioc_write_reg(self.file.as_raw_fd(), &op) }.map_err(io::Error::from)?;
        Ok(())
    }

    pub fn read(&self, reg: u16) -> io::Result<u8> {
        let mut op = VideoChipRegOp {
            reg,
            val: 0,
            _pad: 0,
        };
        // SAFETY: op is a valid VideoChipRegOp and the kernel driver validates the ioctl.
        unsafe { video_chip_ioc_read_reg(self.file.as_raw_fd(), &mut op) }
            .map_err(io::Error::from)?;
        log::trace!("Register {:04X} read: value=0x{:02X}", reg, op.val);
        Ok(op.val)
    }

    /// Read `buf.len()` consecutive registers from `reg` in one I2C
    /// transaction (the chip auto-increments the register address).
    pub fn read_block(&self, reg: u16, buf: &mut [u8]) -> io::Result<()> {
        let mut op = VideoChipBlockOp {
            reg,
            len: buf.len() as u16,
            val: [0; BLOCK_MAX],
        };
        // SAFETY: op is a valid VideoChipBlockOp and the kernel driver validates
        // the register range and length.
        unsafe { video_chip_ioc_read_block(self.file.as_raw_fd(), &mut op) }
            .map_err(io::Error::from)?;
        buf.copy_from_slice(&op.val[..buf.len()]);
        log::trace!("Register {:04X} block read: {:02X?}", reg, buf);
        Ok(())
    }
}
