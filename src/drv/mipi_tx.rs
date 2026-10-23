//! MIPI transmitter PHY / digital control.

use crate::app::tx::Tx;
use crate::config::*;
use crate::drv::system;
use crate::format::{MipiFormat, PixelEncoding};
use crate::reg;
use crate::time;
use crate::timing::Timing;

const TXPLL_2_5G: u32 = 2_500_000;
const TXPLL_640M: u32 = 640_000;
const TXPLL_320M: u32 = 320_000;
const TXPLL_160M: u32 = 160_000;
const TXPLL_80M: u32 = 80_000;
const CTS_DATARATE: u32 = 1_900_000;

pub const HALF_WRITE_CLK: u8 = 0;
pub const WRITE_CLK: u8 = 1;

const TAP0: [u8; 2] = [0x14, 0x14];
const TAP1: [u8; 2] = [0x08, 0x08];
const MIPI_SWING: [u8; 2] = [0x14, 0x14];

fn set_bit_value(addr: u16, bit: u8, on: bool) {
    if on {
        reg::set_bit(addr, bit);
    } else {
        reg::clear_bit(addr, bit);
    }
}

/// Enable or disable the CSI video output. Bits 4/5 of EA89 select the DSI /
/// CSI output; only CSI is ever driven, so bit 4 (DSI) is always cleared.
pub fn video_set(on: bool) {
    if on {
        reg::set_bit(0xE10A, 7);
        reg::set_bit(0xEA89, 5);
        reg::and(0xEA89, !0x10);
        log::info!("CSI video output enabled");
    } else {
        reg::clear_bit(0xE10A, 7);
        reg::and(0xEA89, !(0x10 | 0x20));
        log::info!("CSI video output disabled");
    }
}

pub fn refresh_mipi_in_clk(tx: &mut Tx) -> u32 {
    system::fm_clk_set(system::FREQ_METER_1, system::AD_RXPLL_HALF_PIX_CLK);
    time::delay_ms(5);
    let v = system::fm_clk_get(system::FREQ_METER_1);
    if v != 0 && v != tx.mipi_in_clk {
        tx.mipi_in_clk = v;
    }
    v
}

fn dphy_cali() {
    let (mut tx0_rd, mut tx0_rp) = (0u8, 0u8);
    let (mut tx1_rd, mut tx1_rp) = (0u8, 0u8);

    reg::set_bit(0xE234, 5);
    reg::set_bit(0xE244, 5);
    reg::set_bit(0xE760, 0);
    reg::set_bit(0xE760, 1);

    for _ in 0..10 {
        if reg::get_bit(0xE75C, 7) != 0 {
            tx0_rd = reg::read(0xE75C) & 0x1F;
            tx0_rp = reg::read(0xE75D) & 0x1F;
            log::debug!(
                "Transmitter D-PHY calibration: port=0, RD=0x{:02X}, RP=0x{:02X}",
                tx0_rd,
                tx0_rp
            );
            break;
        }
        time::delay_ms(1);
    }
    for _ in 0..10 {
        if reg::get_bit(0xE75E, 7) != 0 {
            tx1_rd = reg::read(0xE75E) & 0x1F;
            tx1_rp = reg::read(0xE75F) & 0x1F;
            log::debug!(
                "Transmitter D-PHY calibration: port=1, RD=0x{:02X}, RP=0x{:02X}",
                tx1_rd,
                tx1_rp
            );
            break;
        }
        time::delay_ms(1);
    }

    if tx0_rd != 0 && tx0_rp != 0 {
        reg::and(0xE236, 0xE0);
        reg::or(0xE236, tx0_rp);
        reg::and(0xE237, 0xE0);
        reg::or(0xE237, tx0_rd);
    }
    if tx1_rd != 0 && tx1_rp != 0 {
        reg::and(0xE246, 0xE0);
        reg::or(0xE246, tx1_rp);
        reg::and(0xE247, 0xE0);
        reg::or(0xE247, tx1_rd);
    }

    reg::clear_bit(0xE234, 5);
    reg::clear_bit(0xE244, 5);
}

pub fn dphy_set(rate: u32) {
    reg::set_bit(0xE10A, 7);
    reg::clear_bit(0xE230, 4);
    reg::write(0xE238, 0);
    reg::write(0xE239, 0);
    reg::write(0xE248, 0);
    reg::write(0xE249, 0);

    reg::write(0xE233, MIPI_SWING[0]);
    reg::write(0xE243, MIPI_SWING[1]);

    reg::set_bit(0xE288, 5);

    if rate >= CTS_DATARATE {
        reg::set_bit(0xE230, 4);
    }
    reg::set_bit(0xE231, 7);
    reg::set_bit(0xE234, 4);

    reg::write(0xE236, TAP0[0]);
    reg::write(0xE237, TAP0[1]);
    reg::set_bit(0xE236, 5);
    reg::set_bit(0xE23A, 5);
    reg::set_bit(0xE23B, 5);
    reg::set_bit(0xE23C, 5);
    reg::set_bit(0xE23D, 5);
    reg::set_bit(0xE23E, 5);

    reg::and(0xE234, 0xFC);
    reg::or(0xE234, 0x01);

    if rate >= CTS_DATARATE {
        reg::write(0xE238, TAP1[0]);
        reg::write(0xE239, TAP1[1]);
        reg::set_bit(0xE238, 5);
    }

    reg::set_bit(0xE241, 7);
    reg::set_bit(0xE244, 4);

    reg::write(0xE246, TAP0[0]);
    reg::write(0xE247, TAP0[1]);
    reg::set_bit(0xE246, 5);
    reg::set_bit(0xE24A, 5);
    reg::set_bit(0xE24B, 5);
    reg::set_bit(0xE24C, 5);
    reg::set_bit(0xE24D, 5);
    reg::set_bit(0xE24E, 5);

    reg::and(0xE244, 0xFC);
    reg::or(0xE244, 0x01);

    if rate >= CTS_DATARATE {
        reg::write(0xE248, TAP1[0]);
        reg::write(0xE249, TAP1[1]);
        reg::set_bit(0xE248, 5);
    }

    dphy_cali();
}

fn pll_cali() -> bool {
    reg::set_bit(0xE271, 4);
    reg::set_bit(0xE271, 5);
    reg::set_bit(0xE271, 3);

    reg::clear_bit(0xE270, 6);
    reg::clear_bit(0xE270, 7);

    reg::clear_bit(0xE10B, 1);
    reg::set_bit(0xE10B, 1);

    reg::clear_bit(0xE10E, 6);
    reg::set_bit(0xE10E, 6);

    reg::clear_bit(0xE74B, 7);
    reg::set_bit(0xE74B, 7);

    reg::clear_bit(0xE10B, 0);
    reg::set_bit(0xE10B, 0);

    for _ in 0..10 {
        time::delay_ms(5);
        if reg::get_bit(0xE753, 1) != 0 {
            break;
        }
    }
    if (reg::read(0xE753) & 0x03) != 0x02 {
        log::error!("Failed to calibrate transmitter PLL");
        return false;
    }
    if reg::get_bit(0xE753, 3) == 0 {
        log::debug!("Transmitter PLL is unlocked");
        return false;
    }
    log::debug!(
        "Transmitter PLL calibration: Register E754=0x{:02X}, Register E755=0x{:02X}",
        reg::read(0xE754),
        reg::read(0xE755)
    );
    true
}

pub fn pll_set(tx: &Tx) {
    let ul_half_pix = tx.data_rate;
    let ul_byteclk = ul_half_pix >> 3;

    log::debug!(
        "Transmitter PLL configuration: input clock={} kHz, byte clock={} kHz, lane rate={} kbit/s, bits per pixel={}, ports={}, lanes={}",
        tx.mipi_in_clk,
        ul_byteclk,
        ul_half_pix,
        tx.bpp,
        tx.port_num,
        tx.lane_num
    );

    reg::clear_bit(0xE270, 5);
    let predivset = 1u32;
    reg::and(0xE270, 0xF0);
    reg::or(0xE270, 0);
    reg::set_bit(0xE27E, 7);

    let mut freqset: u32 = 1;
    let mut tbcrset: u32 = 1;

    if (TXPLL_640M..TXPLL_2_5G).contains(&ul_half_pix) {
        reg::and(0xE27E, 0xCF);
        reg::or(0xE27E, 0);
        freqset = 1;
    } else if (TXPLL_320M..TXPLL_640M).contains(&ul_half_pix) {
        reg::and(0xE27E, 0xCF);
        reg::or(0xE27E, 0x10);
        freqset = 2;
    } else if (TXPLL_160M..TXPLL_320M).contains(&ul_half_pix) {
        reg::and(0xE27E, 0xCF);
        reg::or(0xE27E, 0x20);
        freqset = 4;
    } else if (TXPLL_80M..TXPLL_160M).contains(&ul_half_pix) {
        reg::and(0xE27E, 0xCF);
        reg::or(0xE27E, 0x30);
        freqset = 8;
    } else {
        log::error!(
            "Transmitter PLL data rate is out of range: {} kbit/s per lane",
            ul_half_pix
        );
    }

    if ul_half_pix > 1_000_000 {
        reg::set_bit(0xE271, 6);
        tbcrset = 2;
    } else {
        reg::clear_bit(0xE271, 6);
    }

    let m2 = ((freqset * ul_half_pix * predivset) / (tbcrset * 0x5DC0)) as u8;
    reg::write(0xE277, m2);

    log::debug!(
        "Transmitter PLL dividers: N1=0x{:02X}, N2=0x{:02X}, M1=0x{:02X}, M2=0x{:02X}",
        predivset,
        freqset,
        tbcrset,
        m2
    );
    pll_cali();
    reg::clear_bit(0xE230, 7);
    reg::set_bit(0xE230, 7);
}

pub fn port_data_enable(config: &MipiSwapConfig) {
    reg::let_bits(0xE5DE, 4, 0, command_enable_mask(&config.ports[0]));
    reg::let_bits(0xE5DF, 4, 0, command_enable_mask(&config.ports[1]));
}

pub fn phy_timing_para_set(tx: &Tx, pixel_encoding: PixelEncoding) {
    let rdbyteclk = (tx.data_rate / 1000) >> 3;

    let a4 = (rdbyteclk * 6 / 100) as u8 + 1;
    let a6 = (rdbyteclk * 7 / 100) as u8 + 4;
    let a9 = (rdbyteclk * 7 / 100) as u8 + 7;
    reg::write(0xEAA4, a4);
    reg::write(0xEAA6, a6);
    reg::write(0xEAA9, a9);

    let (mut a5, mut a7);
    let a8a: u8;

    // High-bandwidth video (4K60 YUV422 class and above) needs the D-PHY
    // timing patches below; gate on the per-lane data rate (pre-headroom) so
    // the threshold holds across lane/port configurations.
    let high_bw = tx.base_data_rate >= crate::app::tx::LANE_RATE_HIGH;
    if tx.clock_burst {
        a5 = (rdbyteclk * 5 / 100) as u8;
        if high_bw && pixel_encoding == PixelEncoding::Rgb {
            a5 = a5.saturating_sub(1);
        }
        if tx.data_rate < CTS_DATARATE {
            a7 = (rdbyteclk * 6 / 25) as u8 + 5;
        } else {
            a7 = (rdbyteclk * 6 / 25) as u8 + 2;
        }
        if high_bw {
            a7 = 5;
            if pixel_encoding == PixelEncoding::Rgb {
                a5 = a5.saturating_sub(1);
            }
        }
        reg::write(0xEAA5, a5);
        reg::write(0xEAA7, a7);
        a8a = a4.wrapping_add(a5).wrapping_add(a7).wrapping_add(2);
        reg::write(0xEA8A, a8a);
    } else {
        a5 = (rdbyteclk * 6 / 100) as u8;
        a7 = (rdbyteclk * 6 / 25) as u8;
        a8a = (rdbyteclk / 10) as u8;
        reg::write(0xEAA5, a5);
        reg::write(0xEAA7, a7);
        reg::write(0xEA8A, a8a);
    }

    if tx.csi_8lane {
        let a4 = reg::read(0xEAA4);
        let a5 = reg::read(0xEAA5);
        reg::write(0xEA8A, a4.wrapping_add(a5).wrapping_add(0x0A));
    }
}

pub fn timing_set(t: &Timing) {
    reg::write(0xEA7E, ((t.h_active >> 8) & 0xff) as u8);
    reg::write(0xEA7F, (t.h_active & 0xff) as u8);

    reg::write(0xEA7C, ((t.v_active >> 8) & 0xff) as u8);
    reg::write(0xEA7D, (t.v_active & 0xff) as u8);

    let sum = (t.v_sw as u16) + (t.v_bp as u16);
    reg::write(0xEA80, ((sum >> 8) & 0xff) as u8);
    reg::write(0xEA81, (sum & 0xff) as u8);

    reg::write(0xEA7A, 0);
    let hss = t.h_active >> 2;
    reg::write(0xEA7A, ((hss >> 8) & 0xff) as u8);
    reg::write(0xEA7B, (hss & 0xff) as u8);
}

pub fn in_hsync_pol_sel(pol: bool) {
    if pol {
        reg::and(0xEA70, 0xFD);
    } else {
        reg::or(0xEA70, 0x02);
    }
}
pub fn in_vsync_pol_sel(pol: bool) {
    if pol {
        reg::and(0xEA70, 0xFE);
    } else {
        reg::or(0xEA70, 0x01);
    }
}

pub fn hss_set(mut v: u16) {
    if v > 0x03FF {
        v = 0x03FF;
    }
    reg::and(0xEA84, 0xFC);
    reg::or(0xEA84, ((v >> 8) & 0x03) as u8);
    reg::write(0xEA85, (v & 0xff) as u8);
}

pub fn hss_get() -> u16 {
    (((reg::read(0xEA84) & 0x03) as u16) << 8) | reg::read(0xEA85) as u16
}

pub fn fsm_hact_get() -> u16 {
    (((reg::read(0xEAC1) & 0x3F) as u16) << 8) | reg::read(0xEAC2) as u16
}

pub fn fifo_delay_set(rddly: u16) {
    reg::write(0xEA82, ((rddly >> 8) & 0xff) as u8);
    reg::write(0xEA83, (rddly & 0xff) as u8);
}

pub fn dphy_csi_8lane_en(on: bool) {
    if on {
        reg::or(0xEAA0, 0x40);
    } else {
        reg::and(0xEAA0, 0xBF);
    }
}

pub fn lane_set(lane_num: u8) {
    reg::and(0xEA89, 0xFC);
    if lane_num == 8 {
        reg::and(0xEA89, 0xFC);
    } else if lane_num < 4 {
        reg::and(0xEA89, 0xFC);
        reg::or(0xEA89, lane_num);
    }
}

fn lane_logic_channel(config: &MipiPortConfig, lane: usize, fallback: usize) -> u8 {
    config.lanes[lane]
        .map(|lc| lc.logic_channel)
        .unwrap_or(fallback)
        .min(MIPI_TXC) as u8
}

fn lane_swap(config: &MipiPortConfig, lane: usize) -> bool {
    config.lanes[lane].map(|lc| lc.swap).unwrap_or(false)
}

fn command_enable_mask(config: &MipiPortConfig) -> u8 {
    config
        .lanes
        .iter()
        .enumerate()
        .fold(0, |mask, (idx, lane)| {
            mask | ((lane.is_some() as u8) << idx)
        })
}

fn physical_port_swap_set(config: &MipiPortConfig, src_addr: u16) {
    reg::let_bits(src_addr, 4, 4, config.logic_port & 0x01);
    reg::let_bits(
        src_addr,
        2,
        0,
        lane_logic_channel(config, MIPI_TXC, MIPI_TXC),
    );
    reg::let_bits(
        src_addr + 1,
        6,
        4,
        lane_logic_channel(config, MIPI_TX3, MIPI_TX3),
    );
    reg::let_bits(
        src_addr + 1,
        2,
        0,
        lane_logic_channel(config, MIPI_TX2, MIPI_TX2),
    );
    reg::let_bits(
        src_addr + 2,
        6,
        4,
        lane_logic_channel(config, MIPI_TX1, MIPI_TX1),
    );
    reg::let_bits(
        src_addr + 2,
        2,
        0,
        lane_logic_channel(config, MIPI_TX0, MIPI_TX0),
    );
}

pub fn swap_set(config: &MipiSwapConfig) {
    physical_port_swap_set(&config.ports[0], 0xE5D8);
    physical_port_swap_set(&config.ports[1], 0xE5DB);
    port_data_enable(config);
}

fn physical_port_np_swap_set(config: &MipiPortConfig, base_addr: u16) {
    set_bit_value(base_addr, 3, lane_swap(config, MIPI_TX0));
    set_bit_value(base_addr + 1, 3, lane_swap(config, MIPI_TX1));
    set_bit_value(base_addr + 2, 3, lane_swap(config, MIPI_TXC));
    set_bit_value(base_addr + 3, 3, lane_swap(config, MIPI_TX2));
    set_bit_value(base_addr + 4, 3, lane_swap(config, MIPI_TX3));
}

pub fn np_swap_set(config: &MipiSwapConfig) {
    physical_port_np_swap_set(&config.ports[0], 0xE23A);
    physical_port_np_swap_set(&config.ports[1], 0xE24A);
}

pub fn port_set(port_num: u8) {
    reg::and(0xEA87, 0xFC);
    reg::or(0xEA87, port_num);
}

pub fn three_d_mode_set(mode: u8) {
    let tmp = reg::read(0xEA87) & 0x8F;
    reg::write(0xEA87, tmp);
    reg::write(0xEA87, tmp | ((mode << 4) & 0xF0));
}

pub fn dphy_clk_mode_sel(burst: bool) {
    if burst {
        reg::or(0xEA89, 0x80);
    } else {
        reg::and(0xEA89, 0x7F);
    }
}

pub fn dphy_clk_data_set() {
    reg::write(0xEAAA, 0xAA);
}

pub fn half_wr_clk_src_sel(clock: u8) {
    if clock == WRITE_CLK {
        reg::or(0xE230, 0x40);
    } else {
        reg::and(0xE230, 0xBF);
    }
}

/// Native packet packing. YUV444 uses the RGB carrier without matrix or
/// subsampling; its payload encoding is reported separately in the stub ABI.
pub fn csi_packet_format(tx_format: MipiFormat) -> Option<(u8, u8)> {
    Some(match tx_format {
        MipiFormat::Rgb8Bit | MipiFormat::Yuv444_8Bit => (0x25, 0x24),
        MipiFormat::Yuv422_8Bit => (0x01, 0x1E),
        MipiFormat::Yuv422_10Bit => (0x31, 0x1F),
        MipiFormat::Yuv420_8Bit => (0x20, 0x1A),
        MipiFormat::Yuv420_10Bit => (0x41, 0x19),
        _ => return None,
    })
}

pub fn csi_data_type_set(tx_format: MipiFormat) {
    if let Some((a, b)) = csi_packet_format(tx_format) {
        reg::write(0xEA88, a);
        reg::write(0xEA86, b);
    }
}

pub fn dphy_skew_cali() {
    reg::or(0xEAAB, 0x0F);
    reg::write(0xEAAC, 0x18);
    reg::write(0xEAAD, 0);
    reg::write(0xEAAE, 0x10);
    reg::or(0xEAAB, 0x10);
    time::delay_ms(5);
    reg::clear_bit(0xEAAB, 4);
    reg::write(0xEAAE, 0);
}

pub fn dphy_clk_hs_trig() {
    reg::set_bit(0xE10A, 7);
    reg::clear_bit(0xEAAB, 5);
    time::delay_ms(1);
    reg::set_bit(0xEAAB, 5);
    time::delay_ms(1);
    reg::clear_bit(0xEAAB, 5);
}
