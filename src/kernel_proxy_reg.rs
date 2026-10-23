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
}
