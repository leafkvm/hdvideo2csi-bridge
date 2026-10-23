//! Read a voltage channel from a named Linux IIO sysfs device.

use std::io;
use std::path::{Path, PathBuf};

pub struct VoltageChannel {
    raw_path: PathBuf,
    scale: f64,
}

impl VoltageChannel {
    pub fn new(device_name: &str, channel: u32) -> io::Result<Self> {
        let device_dir = find_device(Path::new("/sys/bus/iio/devices"), device_name)?;
        Self::from_device_dir(&device_dir, channel)
    }

    fn from_device_dir(device_dir: &Path, channel: u32) -> io::Result<Self> {
        // The scale is fixed. Prefer the channel's scale when the driver
        // provides one, otherwise use the scale shared by all channels.
        let scale_str =
            std::fs::read_to_string(device_dir.join(format!("in_voltage{channel}_scale")))
                .or_else(|_| std::fs::read_to_string(device_dir.join("in_voltage_scale")))?;
        let scale: f64 = scale_str.trim().parse().map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid IIO scale: {e}"),
            )
        })?;
        if !scale.is_finite() || scale <= 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "IIO voltage scale must be finite and positive",
            ));
        }
        Ok(Self {
            raw_path: device_dir.join(format!("in_voltage{channel}_raw")),
            scale,
        })
    }

    /// Return the voltage in millivolts using the cached divider scale.
    pub fn read(&self) -> io::Result<f64> {
        let raw: f64 = std::fs::read_to_string(&self.raw_path)?
            .trim()
            .parse()
            .map_err(|e| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Invalid IIO sample: {e}"),
                )
            })?;
        if !raw.is_finite() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "IIO voltage sample must be finite",
            ));
        }
        Ok(raw * self.scale)
    }
}

fn find_device(root: &Path, device_name: &str) -> io::Result<PathBuf> {
    for entry in std::fs::read_dir(root)? {
        let path = entry?.path();
        if let Ok(name) = std::fs::read_to_string(path.join("name")) {
            if name.trim() == device_name {
                return Ok(path);
            }
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!("IIO device {device_name:?} was not found"),
    ))
}
