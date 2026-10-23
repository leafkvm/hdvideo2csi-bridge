//! Compile-time configuration constants.

// ---- top-level toggles ----
pub const MIPI_CLOCK_BURST: bool = false;
pub const MIPI_LANE_NUM: u8 = 4;
pub const MIPI_PORT_NUM: u8 = 1;

pub const MIPI_PHYSICAL_PORT_COUNT: usize = 2;
pub const MIPI_PORT_LANE_COUNT: usize = 5;
pub const MIPI_TX0: usize = 0;
pub const MIPI_TX1: usize = 1;
pub const MIPI_TX2: usize = 2;
pub const MIPI_TX3: usize = 3;
pub const MIPI_TXC: usize = 4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiLaneConfig {
    pub logic_channel: usize,
    pub swap: bool,
}

impl MipiLaneConfig {
    pub const fn new(logic_channel: usize, swap: bool) -> Self {
        Self {
            logic_channel,
            swap,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiPortConfig {
    pub lanes: [Option<MipiLaneConfig>; MIPI_PORT_LANE_COUNT],
    pub logic_port: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiSwapConfig {
    pub ports: [MipiPortConfig; MIPI_PHYSICAL_PORT_COUNT],
}

const MIPI_SWAP_PORT: MipiPortConfig = MipiPortConfig {
    logic_port: 1,
    lanes: [
        Some(MipiLaneConfig::new(MIPI_TX0, true)),
        Some(MipiLaneConfig::new(MIPI_TX1, true)),
        Some(MipiLaneConfig::new(MIPI_TX3, true)),
        Some(MipiLaneConfig::new(MIPI_TX2, true)),
        Some(MipiLaneConfig::new(MIPI_TXC, true)),
    ],
};

pub const MIPI_SWAP_CONFIG: MipiSwapConfig = MipiSwapConfig {
    ports: [MIPI_SWAP_PORT, MIPI_SWAP_PORT],
};

pub const MIPITX_OUT_CTRL: bool = true;

/// Test aid: drive the I2S output from the chip's internal pattern generator
/// (a 1 kHz tone at 48 kHz) instead of the source's audio, and report it as
/// live audio. Exercises the I2S/SAI path without an audio source.
pub const AUDIO_TEST_PATTERN: bool = false;

pub const SCRAMBLER_NOT: u8 = 0x00;
pub const SCRAMBLER1X: u8 = 0x10;
pub const SCRAMBLER2X: u8 = 0x20;
pub const SCRAMBLER_RX_MODE: u8 = SCRAMBLER1X | SCRAMBLER2X;

// Pack helpers
#[inline]
pub fn pack_16(hi: u8, lo: u8) -> u16 {
    ((hi as u16) << 8) | lo as u16
}

#[inline]
pub fn pack_24(hi: u8, mid: u8, lo: u8) -> u32 {
    ((hi as u32) << 16) | ((mid as u32) << 8) | lo as u32
}

// ---- Typed enums used by the state machines ----

/// Logic level.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum Level {
    #[default]
    Low,
    High,
}

impl Level {
    pub fn is_high(self) -> bool {
        matches!(self, Level::High)
    }
}

/// Debounced stability state.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum Stability {
    #[default]
    Pending,
    Stable,
    Unstable,
}
