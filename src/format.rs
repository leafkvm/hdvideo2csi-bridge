//! Typed colour format primitives.
//!
//! The receiver reports three distinct format concepts — AVI pixel encoding,
//! chip-side bit-depth, and internal MIPI-format index. These types make the
//! distinction explicit.

use std::fmt;

/// AVI pixel encoding from Register E8D0[6:4].
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum PixelEncoding {
    #[default]
    Rgb,
    Yuv422,
    Yuv444,
    Yuv420,
}

impl PixelEncoding {
    /// Decode the 3-bit field as written by the source. Returns `None` for
    /// values outside the AVI spec (the chip can sometimes report transient
    /// garbage on hot-plug).
    pub fn from_chip(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Rgb),
            1 => Some(Self::Yuv422),
            2 => Some(Self::Yuv444),
            3 => Some(Self::Yuv420),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Rgb => "RGB",
            Self::Yuv422 => "YUV422",
            Self::Yuv444 => "YUV444",
            Self::Yuv420 => "YUV420",
        }
    }
}

impl fmt::Display for PixelEncoding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Bit depth per component, as encoded in the GCP CD field
/// (Register E8D1[7:4] — 0=8bit, 5=10bit, 6=12bit).
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum BitDepth {
    #[default]
    Bpp8,
    Bpp10,
    Bpp12,
}

impl BitDepth {
    /// Map a chip-side code to a [`BitDepth`]. Unknown codes are treated as
    /// 8bpp.
    pub fn from_chip(code: u8) -> Self {
        match code {
            5 => Self::Bpp10,
            6 => Self::Bpp12,
            _ => Self::Bpp8,
        }
    }
}

/// Internal MIPI input/output format. Combines [`PixelEncoding`] with
/// [`BitDepth`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
#[allow(non_camel_case_types, clippy::enum_variant_names)]
pub enum MipiFormat {
    #[default]
    Rgb8Bit,
    Rgb10Bit,
    Rgb12Bit,
    Yuv444_8Bit,
    Yuv444_10Bit,
    Yuv444_12Bit,
    Yuv422_8Bit,
    Yuv422_10Bit,
    Yuv422_12Bit,
    Yuv420_8Bit,
    Yuv420_10Bit,
    Yuv420_12Bit,
}

impl MipiFormat {
    pub fn from_encoding_depth(enc: PixelEncoding, depth: BitDepth) -> Self {
        match (enc, depth) {
            (PixelEncoding::Rgb, BitDepth::Bpp8) => Self::Rgb8Bit,
            (PixelEncoding::Rgb, BitDepth::Bpp10) => Self::Rgb10Bit,
            (PixelEncoding::Rgb, BitDepth::Bpp12) => Self::Rgb12Bit,
            (PixelEncoding::Yuv422, BitDepth::Bpp8) => Self::Yuv422_8Bit,
            (PixelEncoding::Yuv422, BitDepth::Bpp10) => Self::Yuv422_10Bit,
            (PixelEncoding::Yuv422, BitDepth::Bpp12) => Self::Yuv422_12Bit,
            (PixelEncoding::Yuv444, BitDepth::Bpp8) => Self::Yuv444_8Bit,
            (PixelEncoding::Yuv444, BitDepth::Bpp10) => Self::Yuv444_10Bit,
            (PixelEncoding::Yuv444, BitDepth::Bpp12) => Self::Yuv444_12Bit,
            (PixelEncoding::Yuv420, BitDepth::Bpp8) => Self::Yuv420_8Bit,
            (PixelEncoding::Yuv420, BitDepth::Bpp10) => Self::Yuv420_10Bit,
            (PixelEncoding::Yuv420, BitDepth::Bpp12) => Self::Yuv420_12Bit,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Rgb8Bit => "RGB 8bit",
            Self::Rgb10Bit => "RGB 10bit",
            Self::Rgb12Bit => "RGB 12bit",
            Self::Yuv444_8Bit => "YUV444 8bit",
            Self::Yuv444_10Bit => "YUV444 10bit",
            Self::Yuv444_12Bit => "YUV444 12bit",
            Self::Yuv422_8Bit => "YUV422 8bit",
            Self::Yuv422_10Bit => "YUV422 10bit",
            Self::Yuv422_12Bit => "YUV422 12bit",
            Self::Yuv420_8Bit => "YUV420 8bit",
            Self::Yuv420_10Bit => "YUV420 10bit",
            Self::Yuv420_12Bit => "YUV420 12bit",
        }
    }

    pub fn pixel_encoding(self) -> PixelEncoding {
        use MipiFormat::*;
        match self {
            Rgb8Bit | Rgb10Bit | Rgb12Bit => PixelEncoding::Rgb,
            Yuv444_8Bit | Yuv444_10Bit | Yuv444_12Bit => PixelEncoding::Yuv444,
            Yuv422_8Bit | Yuv422_10Bit | Yuv422_12Bit => PixelEncoding::Yuv422,
            Yuv420_8Bit | Yuv420_10Bit | Yuv420_12Bit => PixelEncoding::Yuv420,
        }
    }

    pub fn bit_depth(self) -> BitDepth {
        use MipiFormat::*;
        match self {
            Rgb8Bit | Yuv444_8Bit | Yuv422_8Bit | Yuv420_8Bit => BitDepth::Bpp8,
            Rgb10Bit | Yuv444_10Bit | Yuv422_10Bit | Yuv420_10Bit => BitDepth::Bpp10,
            Rgb12Bit | Yuv444_12Bit | Yuv422_12Bit | Yuv420_12Bit => BitDepth::Bpp12,
        }
    }

    pub fn is_yuv420(self) -> bool {
        matches!(
            self,
            Self::Yuv420_8Bit | Self::Yuv420_10Bit | Self::Yuv420_12Bit
        )
    }
}

impl fmt::Display for MipiFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}
