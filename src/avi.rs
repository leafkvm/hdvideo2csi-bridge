//! Colour signalling from the AVI InfoFrame (header, checksum, 13 data bytes).

use crate::format::PixelEncoding;
use crate::stub::{Color, colorspace, quantization};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Range {
    Limited,
    Full,
}

impl Range {
    pub fn quantization(self) -> u8 {
        match self {
            Self::Limited => quantization::LIM_RANGE,
            Self::Full => quantization::FULL_RANGE,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Avi {
    db: [u8; 13],
}

impl Avi {
    pub fn parse(packet: [u8; 17], encoding: PixelEncoding) -> Option<Self> {
        // Reject DVI's zero header, incomplete/torn packets and stale packets
        // from the preceding encoding. Do not interpret them as valid AVI.
        if packet[0] != 0x82
            || !matches!(packet[1], 2 | 3)
            || packet[2] != 13
            || packet.iter().fold(0u8, |sum, v| sum.wrapping_add(*v)) != 0
            || PixelEncoding::from_chip(packet[4] >> 5) != Some(encoding)
        {
            return None;
        }
        Some(Self {
            db: packet[4..].try_into().unwrap(),
        })
    }

    pub fn explicit_rgb_range(self) -> Option<Range> {
        match (self.db[2] >> 2) & 3 {
            1 => Some(Range::Limited),
            2 => Some(Range::Full),
            _ => None,
        }
    }

    pub fn rgb_range_is_default(self) -> bool {
        (self.db[2] >> 2) & 3 == 0
    }

    /// Used only when the RGB detector has not completed. AVI Q=Default
    /// means limited for CE timings other than VGA, full for IT/VGA timings.
    pub fn default_rgb_range(self) -> Range {
        if self.db[3] & 0x7f <= 1 {
            Range::Full
        } else {
            Range::Limited
        }
    }

    pub fn ycc_range(self) -> Option<Range> {
        match self.db[4] >> 6 {
            0 => Some(Range::Limited),
            1 => Some(Range::Full),
            _ => None, // reserved YQ; do not advertise a known range
        }
    }

    pub fn colorspace(self, encoding: PixelEncoding, v_active: u16) -> u8 {
        let bt601 = if matches!(v_active, 288 | 576) {
            colorspace::SYSTEM_BG
        } else {
            colorspace::SMPTE170M
        };
        match self.db[1] >> 6 {
            0 if encoding == PixelEncoding::Rgb => colorspace::REC709,
            0 if v_active <= 576 => bt601,
            0 => colorspace::REC709,
            1 => bt601,
            2 => colorspace::REC709,
            3 => match (self.db[2] >> 4) & 7 {
                6 => colorspace::BT2020, // EC=6 covers both RGB and YCbCr
                // xvYCC/sYCC/opYCC, opRGB, BT.2020 constant-luminance and
                // EC=7 need encoding/gamut signalling absent from Color.
                _ => colorspace::DEFAULT,
            },
            _ => unreachable!(),
        }
    }
}

pub fn input_color(
    avi: Option<Avi>,
    encoding: PixelEncoding,
    v_active: u16,
    detected_rgb_range: Option<Range>,
) -> Color {
    let range = if encoding == PixelEncoding::Rgb {
        match avi {
            None => Some(Range::Full), // DVI uses full RGB
            Some(a) if a.rgb_range_is_default() => {
                Some(detected_rgb_range.unwrap_or_else(|| a.default_rgb_range()))
            }
            Some(a) => a.explicit_rgb_range(), // reserved Q remains unknown
        }
    } else {
        avi.and_then(Avi::ycc_range)
    };
    Color {
        quantization: range.map_or(quantization::DEFAULT, Range::quantization),
        colorspace: avi.map_or(
            if encoding == PixelEncoding::Rgb {
                colorspace::REC709
            } else {
                colorspace::DEFAULT
            },
            |a| a.colorspace(encoding, v_active),
        ),
    }
}
