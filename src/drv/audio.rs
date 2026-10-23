//! Audio clock recovery and I2S output of the receiver chip.
//!
//! Hardware model:
//!
//! ```text
//!  HDMI audio sample packets ──► clock-recovery PLL ──► I2S formatter ──► pads (MCLK, SCLK, WS, SD0)
//!   PKT_AUDIO_PRESENT bit 1        PLL_RUNNING bit 0        OUT_EN bit 0
//!                                  PLL_STATUS (lock)        OUT_RESET
//!                                  SOFT_RESET bit 3         ▲
//!                                  256 × fs ──► frequency meter 2 (system::fm_clk_*)
//! ```
//!
//! * [`PKT_AUDIO_PRESENT`] bit 1 is a live level: audio sample packets are
//!   arriving. It is the earliest start indication and the first to drop
//!   when the source stops.
//! * The clock-recovery PLL starts about 50–150 ms after the packets
//!   ([`PLL_RUNNING`]), seeds itself from the incoming rate and slews onto
//!   it; it reports lock ([`PLL_STATUS`] = 0x10 in the low five bits) about
//!   500 ms after the packets, up to about 1.2 s. The seed can be wrong,
//!   which only lengthens the slew; the clock still converges. The lock
//!   status usually means the clock is within 0.05 %, but it can be asserted
//!   with the clock 0.15 % off, and briefly while still slewing, so the
//!   clock is checked independently.
//! * The chip's second frequency meter can measure the PLL's 256 × fs output
//!   directly, in kHz with a 1 ms gate. That is what [`read_pll`] reports as
//!   the clock, and what the sample rate is derived from.
//! * The I2S clocks (BCLK/LRCK) come from the PLL, so they stop when the
//!   source stops sending audio packets.
//!
//! This module is stateless: it reads and writes registers. The acquisition
//! state machine lives in `app::audio`.

use std::fmt;

use crate::drv::system;
use crate::reg;
use crate::time;

// ---- registers ----

/// Audio-domain soft resets (active low; the whole byte is written 0x00 then
/// 0xFF to restart the domain). Bit 3 re-seeds and restarts the
/// clock-recovery PLL; it locks again about 300 ms later.
const SOFT_RESET: u16 = 0xE108;
const SOFT_RESET_PLL: u8 = 3;
/// I2S formatter reset, bit 0 (pulsed low at init).
const OUT_RESET: u16 = 0xE109;
/// Audio PLL analogue control: bits 7:6 power the PLL down when set.
const PLL_POWER_DOWN: u16 = 0xE203;
/// Audio PLL configuration written once at init: MCLK ratio, hardware
/// (clock-recovery driven) frequency mode, divider.
const PLL_MCLK_CFG: u16 = 0xE204;
const PLL_FREQ_MODE: u16 = 0xE205;
const PLL_DIV_MODE: u16 = 0xE206;
/// Bit 0: the audio PLL is powered and running (follows the packets).
const PLL_RUNNING: u16 = 0xE763;
/// Bit 1: audio sample packets are arriving.
const PKT_AUDIO_PRESENT: u16 = 0xE8D6;
/// I2S output control: bit 0 enables the I2S output, bits 3:0 are the data
/// line enables, bit 6 is the (unused) SPDIF output.
const OUT_EN: u16 = 0xEB0A;
/// Output mode: bit 0 selects the internal pattern generator instead of the
/// HDMI input, bits 3 and 7 select 8-channel TDM. All kept clear.
const OUT_MODE: u16 = 0xEB0B;
/// Input configuration written 0xFF at init (meaning unknown).
const IN_CFG0: u16 = 0xEB22;
const IN_CFG1: u16 = 0xEB23;
/// Bit 0 enables the HDMI audio input.
const IN_EN: u16 = 0xEB2F;
/// PLL status: bit 4 = running, bit 0 = frequency error, bits 2:1 = lock
/// qualifiers (cleared last). Running and locked reads 0x10 in these bits.
const PLL_STATUS: u16 = 0xEB8D;
const PLL_STATUS_MASK: u8 = 0x17;
const PLL_STATUS_LOCKED: u8 = 0x10;

// Pattern generator: an internal audio source (1 kHz tone) that runs the PLL
// on a programmed word instead of the recovered clock. Test aid only (see
// `config::AUDIO_TEST_PATTERN`).
/// Bit 4 of [`SOFT_RESET`]: 0 = the pattern PLL runs on the programmed
/// word; a falling edge loads the word. 1 in normal (HDMI) operation.
const SOFT_RESET_PATTERN_PLL: u8 = 4;
/// Pattern generator enables.
const PATTERN_EN: u16 = 0xEB08; // bit 1; must be clear for lock after a handover
const PATTERN_CFG: u16 = 0xEB09; // written 0x30
/// Programmed PLL word: M, then K[23:16], K[15:8], K[7:0]; M.K = 512 × fs in
/// MHz, the same scale as the recovered word.
const PATTERN_WORD: u16 = 0xEB13;
const PATTERN_LOAD: u16 = 0xEB17; // bit 2
/// Bit 2 = PLL on the programmed word (blocks clock recovery), bit 0 set
/// with it, bits 7:4 = output divider: 0xB for rates up to 48 kHz, 0xA for
/// twice that, 0x9 for four times.
const PATTERN_PLL_CTRL: u16 = 0xEB24;
const PATTERN_IN_EN_BIT: u8 = 3; // in IN_EN
const PATTERN_TONE: u16 = 0xEB37; // bit 0 = single tone

// Pad routing (GPIO out-signal select, pad driver mode, input threshold).
const SIG_ACR_I2S: u8 = 0x04;
const MODE_OUT_DISABLE: u8 = 0x00;
const MODE_OUT_PP_1X: u8 = 0x02;

// ---- sample rates ----

/// An audio sample rate in Hz.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Rate(pub u32);

impl Rate {
    pub const HZ_32000: Rate = Rate(32_000);
    pub const HZ_44100: Rate = Rate(44_100);
    pub const HZ_48000: Rate = Rate(48_000);
    pub const HZ_88200: Rate = Rate(88_200);
    pub const HZ_96000: Rate = Rate(96_000);
    pub const HZ_176400: Rate = Rate(176_400);
    pub const HZ_192000: Rate = Rate(192_000);

    /// The rates an HDMI source can send.
    const KNOWN: [Rate; 7] = [
        Rate::HZ_32000,
        Rate::HZ_44100,
        Rate::HZ_48000,
        Rate::HZ_88200,
        Rate::HZ_96000,
        Rate::HZ_176400,
        Rate::HZ_192000,
    ];

    pub fn hz(self) -> u32 {
        self.0
    }

    /// The PLL output clock for this rate, in Hz (256 × fs).
    fn clock_hz(self) -> u64 {
        u64::from(self.0) * 256
    }

    /// The PLL word (M.K = 512 × fs in MHz, 24-bit fraction) and output
    /// divider code for this rate. Rates above 48 kHz run the PLL at the
    /// base rate's word and halve the divider per doubling.
    fn pll_word(self) -> (u8, u32, u8) {
        let mut hz = u64::from(self.0);
        let mut divider = 0xB;
        while hz > 48_000 {
            hz /= 2;
            divider -= 1;
        }
        let hz512 = hz * 512;
        let m = (hz512 / 1_000_000) as u8;
        let k = ((hz512 % 1_000_000) << 24) / 1_000_000;
        (m, k as u32, divider)
    }
}

impl fmt::Display for Rate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let hz = self.0;
        if hz % 1000 == 0 {
            write!(f, "{} kHz", hz / 1000)
        } else {
            write!(f, "{}.{} kHz", hz / 1000, (hz % 1000) / 100)
        }
    }
}

// ---- clock-recovery PLL ----

/// Audio sample packets are arriving from the source.
pub fn packets_present() -> bool {
    reg::get_bit(PKT_AUDIO_PRESENT, 1) != 0
}

/// A snapshot of the clock-recovery PLL.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Pll {
    /// The PLL is powered and running (not necessarily locked).
    pub running: bool,
    /// The PLL reports lock. Usually the clock is then within 0.05 % of the
    /// rate, but not always: see [`Pll::clock_matches`].
    pub locked: bool,
    /// The measured PLL output clock (256 × fs) in kHz; 0 while the PLL is
    /// off, about 10000 before the first seed is loaded.
    pub clock_khz: u32,
}

/// Tolerance on the measured clock, in basis points: ±0.25 %. A locked clock
/// measures within ±0.02 %, but the lock flag can be set with the clock
/// 0.15 % off; a slewing clock is off by 1–4 %.
const CLOCK_TOLERANCE_BP: u64 = 25;

impl Pll {
    /// The measured clock is the one for `rate`, within tolerance.
    pub fn clock_matches(&self, rate: Rate) -> bool {
        let expected = rate.clock_hz();
        let measured = u64::from(self.clock_khz) * 1000;
        measured.abs_diff(expected) <= expected * CLOCK_TOLERANCE_BP / 10_000
    }

    /// The sample rate the measured clock corresponds to, if it is a known
    /// one.
    pub fn rate(&self) -> Option<Rate> {
        Rate::KNOWN.into_iter().find(|&r| self.clock_matches(r))
    }
}

impl fmt::Display for Pll {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "clock={} kHz", self.clock_khz)?;
        f.write_str(match (self.running, self.locked) {
            (_, true) => " locked",
            (true, false) => " unlocked",
            (false, false) => " off",
        })
    }
}

pub fn read_pll() -> Pll {
    let running = reg::get_bit(PLL_RUNNING, 0) != 0;
    let locked = running && reg::read(PLL_STATUS) & PLL_STATUS_MASK == PLL_STATUS_LOCKED;
    let clock_khz = system::fm_clk_get(system::FREQ_METER_2);
    Pll {
        running,
        locked,
        clock_khz,
    }
}

/// Re-seed and restart the clock-recovery PLL.
pub fn pll_reset() {
    reg::clear_bit(SOFT_RESET, SOFT_RESET_PLL);
    reg::set_bit(SOFT_RESET, SOFT_RESET_PLL);
}

/// Reset the whole audio domain. Used at the end of a video session so the
/// next one starts from a clean state; the configuration from [`init`] is
/// not affected.
pub fn restart() {
    reg::write(SOFT_RESET, 0x00);
    time::delay_ms(1);
    reg::write(SOFT_RESET, 0xFF);
}

// ---- output ----

/// Enable or disable the I2S output. The clocks only run while the PLL is
/// fed by an audio source, whatever this says.
pub fn output_enable(on: bool) {
    if on {
        reg::set_bit(OUT_EN, 0);
    } else {
        reg::clear_bit(OUT_EN, 0);
        // SPDIF output enable: never set by this driver, kept cleared.
        reg::clear_bit(OUT_EN, 6);
    }
}

// ---- pattern generator (test aid) ----

/// Drive the I2S output from the internal pattern generator: a 1 kHz tone at
/// `rate`, with the PLL on the programmed word. The output must be enabled
/// separately. The PLL reports no lock in this mode; [`Pll::clock_khz`] still
/// measures the clock.
pub fn pattern_start(rate: Rate) {
    let (m, k, divider) = rate.pll_word();
    reg::set_bit(PATTERN_PLL_CTRL, 2);
    reg::set_bit(PATTERN_PLL_CTRL, 0);
    reg::set_bit(IN_EN, PATTERN_IN_EN_BIT);
    reg::set_bit(PATTERN_TONE, 0);
    reg::set_bit(PATTERN_EN, 1);
    reg::write(PATTERN_CFG, 0x30);
    reg::write(PATTERN_WORD, m);
    reg::write(PATTERN_WORD + 1, (k >> 16) as u8);
    reg::write(PATTERN_WORD + 2, (k >> 8) as u8);
    reg::write(PATTERN_WORD + 3, k as u8);
    reg::let_bits(PATTERN_PLL_CTRL, 7, 4, divider);
    reg::set_bit(PATTERN_LOAD, 2);
    reg::set_bit(OUT_MODE, 0);
    // Load the word and start the pattern PLL: the word is latched on the
    // falling edge, so the edge must follow the write.
    reg::set_bit(SOFT_RESET, SOFT_RESET_PATTERN_PLL);
    reg::clear_bit(SOFT_RESET, SOFT_RESET_PATTERN_PLL);
}

/// Return the output to the source's audio: data from the source, pattern
/// PLL off, clock recovery tracking again.
pub fn pattern_stop() {
    reg::clear_bit(OUT_MODE, 0);
    reg::set_bit(SOFT_RESET, SOFT_RESET_PATTERN_PLL);
    reg::clear_bit(PATTERN_PLL_CTRL, 2);
    reg::clear_bit(PATTERN_PLL_CTRL, 0);
    reg::clear_bit(PATTERN_EN, 1);
    reg::clear_bit(PATTERN_LOAD, 2);
    reg::clear_bit(IN_EN, PATTERN_IN_EN_BIT);
    reg::clear_bit(PATTERN_TONE, 0);
}

/// Route I2S WS/SCK/MCLK/SD0 to their pads and configure the pad drivers.
fn pads_init() {
    // Out-signal select = ACR I2S function.
    reg::let_bits(0xE5E6, 3, 0, SIG_ACR_I2S); // GPIO7  I2S WS
    reg::let_bits(0xE5E7, 7, 4, SIG_ACR_I2S); // GPIO8  I2S SCK
    reg::let_bits(0xE5E7, 3, 0, SIG_ACR_I2S); // GPIO9  I2S MCLK
    reg::let_bits(0xE5E8, 7, 4, SIG_ACR_I2S); // SD0    I2S D0
    // Pad mode = push-pull x1, voltage threshold = 3.3V (clear bit).
    reg::let_bits(0xE283, 7, 4, MODE_OUT_PP_1X); // SD0  mode
    reg::clear_bit(0xE289, 6); //                   SD0  Vth 3.3V
    reg::let_bits(0xE281, 3, 0, MODE_OUT_PP_1X); // WS   mode
    reg::clear_bit(0xE289, 3); //                   WS   Vth 3.3V
    reg::let_bits(0xE282, 7, 4, MODE_OUT_PP_1X); // SCK  mode
    reg::clear_bit(0xE289, 4); //                   SCK  Vth 3.3V
    reg::let_bits(0xE282, 3, 0, MODE_OUT_PP_1X); // MCLK mode
    reg::clear_bit(0xE289, 5); //                   MCLK Vth 3.3V
    reg::let_bits(0xE280, 7, 4, MODE_OUT_DISABLE); // SD1 unused, disabled
    reg::clear_bit(0xE289, 2); //                   SD1  Vth 3.3V
}

/// One-time configuration of the audio path: HDMI input, 2-channel I2S
/// output, clock recovery driven by the incoming audio, and frequency meter
/// 2 pointed at the PLL output.
pub fn init() {
    output_enable(false);

    // Input: HDMI audio.
    reg::set_bit(IN_EN, 0);
    reg::clear_bit(OUT_MODE, 0);
    reg::write(IN_CFG0, 0xFF);
    reg::write(IN_CFG1, 0xFF);

    // Output: 2-channel standard I2S on the WS/SCK/MCLK/SD0 pads.
    reg::clear_bits(OUT_EN, 3, 0);
    reg::clear_bit(OUT_MODE, 3);
    reg::clear_bit(OUT_MODE, 7);
    pads_init();
    reg::clear_bit(OUT_RESET, 0);
    reg::set_bit(OUT_RESET, 0);

    // Clock-recovery PLL powered, in hardware mode.
    reg::clear_bit(PLL_POWER_DOWN, 6);
    reg::clear_bit(PLL_POWER_DOWN, 7);
    reg::write(PLL_MCLK_CFG, 0x20);
    reg::write(PLL_FREQ_MODE, 0x43);
    reg::write(PLL_DIV_MODE, 0x16);

    system::fm_clk_set(system::FREQ_METER_2, system::AD_AUDPLL_256FS_CLK);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn locked(clock_khz: u32) -> Pll {
        Pll {
            running: true,
            locked: true,
            clock_khz,
        }
    }

    #[test]
    fn measured_clocks_map_to_rates() {
        // Meter readings of locked clocks.
        assert_eq!(locked(12288).rate(), Some(Rate::HZ_48000));
        assert_eq!(locked(12286).rate(), Some(Rate::HZ_48000));
        assert_eq!(locked(12290).rate(), Some(Rate::HZ_48000));
        assert_eq!(locked(11290).rate(), Some(Rate::HZ_44100));
        assert_eq!(locked(8192).rate(), Some(Rate::HZ_32000));
        assert_eq!(locked(8204).rate(), Some(Rate::HZ_32000)); // 0.15 % off, lock flag set
        assert_eq!(locked(24576).rate(), Some(Rate::HZ_96000));
    }

    #[test]
    fn slewing_and_idle_clocks_match_nothing() {
        assert_eq!(locked(12193).rate(), None); // 0.8 % slow, slewing
        assert_eq!(locked(12240).rate(), None); // 0.4 % slow
        assert_eq!(locked(12068).rate(), None); // 1.8 % slow
        assert_eq!(locked(10000).rate(), None); // default word before the seed
        assert_eq!(locked(0).rate(), None);
        assert!(!locked(12193).clock_matches(Rate::HZ_48000));
    }

    #[test]
    fn pll_words_for_the_standard_rates() {
        assert_eq!(Rate::HZ_32000.pll_word(), (0x10, 0x624DD2, 0xB));
        assert_eq!(Rate::HZ_44100.pll_word(), (0x16, 0x944673, 0xB));
        assert_eq!(Rate::HZ_48000.pll_word(), (0x18, 0x9374BC, 0xB));
        assert_eq!(Rate::HZ_96000.pll_word(), (0x18, 0x9374BC, 0xA));
        assert_eq!(Rate::HZ_176400.pll_word(), (0x16, 0x944673, 0x9));
    }

    #[test]
    fn rate_display() {
        assert_eq!(Rate::HZ_48000.to_string(), "48 kHz");
        assert_eq!(Rate::HZ_44100.to_string(), "44.1 kHz");
    }
}
