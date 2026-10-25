//! Adaptive equalisation tuner for the 3 Video Input channels.

use crate::config::pack_16;
use crate::drv::hddvi_rx;
use crate::reg;
use crate::time;

pub const EQ_LEN: usize = 16;

const EQ: [u8; EQ_LEN] = [
    0x7f, 0x65, 0x53, 0x45, 0x35, 0x2d, 0x27, 0x23, 0x1f, 0x1b, 0x19, 0x17, 0x15, 0x13, 0x11, 0x0f,
];

const EQ_ERR_UPLIMIT: u16 = 0x8010;
const EQ_GOOD_MAX: u16 = 0x8001;
const EQ_GOOD_MIN: u16 = 0x8000;

#[derive(Clone, Copy, Debug)]
enum Channel {
    Ch0,
    Ch1,
    Ch2,
    All,
}

pub fn phug_set() {
    let buf = (reg::read(0xE73F) & 0x30) >> 4;
    if buf == 0 {
        reg::write(0xE70A, 0x03);
        reg::write(0xE228, 0x77);
    } else {
        reg::write(0xE70A, 0x04);
        reg::write(0xE228, 0x00);
    }
    reg::write(0xE709, 0x02);
    reg::write(0xE209, 0x10);
    reg::write(0xE20A, 0x12);
    reg::write(0xE20B, 0x10);
    reg::write(0xE20C, 0x12);
    reg::write(0xE81A, 0x0c);
    log::debug!("Register E70A: value=0x{:02X}", reg::read(0xE70A));
    log::debug!("Register E228: value=0x{:02X}", reg::read(0xE228));
}

fn value_set(channel: Channel, value: u8) {
    reg::write(0xE20D, 0x3f);
    match channel {
        Channel::Ch0 => {
            reg::write(0xE20E, value);
            reg::write(0xE20F, 0x7f);
        }
        Channel::Ch1 => {
            reg::write(0xE210, value);
            reg::write(0xE211, 0x7f);
        }
        Channel::Ch2 => {
            reg::write(0xE212, value);
            reg::write(0xE213, 0x7f);
        }
        Channel::All => {
            reg::write(0xE20E, value);
            reg::write(0xE20F, 0x7f);
            reg::write(0xE210, value);
            reg::write(0xE211, 0x7f);
            reg::write(0xE212, value);
            reg::write(0xE213, 0x7f);
        }
    }
}

pub fn clr_err_cnt() {
    reg::set_bit(0xE828, 2);
    reg::clear_bit(0xE828, 2);
}

fn err_cnt(channel: Channel) -> u16 {
    match channel {
        Channel::Ch0 => pack_16(reg::read(0xE8D8), reg::read(0xE8D9)),
        Channel::Ch1 => pack_16(reg::read(0xE8DA), reg::read(0xE8DB)),
        Channel::Ch2 => pack_16(reg::read(0xE8DC), reg::read(0xE8DD)),
        Channel::All => 0,
    }
}

fn exceeds_uplimit(channel: Channel) -> bool {
    err_cnt(channel) >= EQ_ERR_UPLIMIT
}

fn in_good_band(channel: Channel) -> bool {
    let n = err_cnt(channel);
    (EQ_GOOD_MIN..=EQ_GOOD_MAX).contains(&n)
}

fn set_mid_eq(data: &[u8; 2]) -> u8 {
    match (data[0], data[1]) {
        (0xff, _) => 0x08,
        (a, 0xff) => a >> 1,
        (a, b) if b >= a => (a + b) >> 1,
        (a, _) => a >> 1,
    }
}

fn scan_lr() -> bool {
    let mut auc_ch = [[0xffu8; 2]; 3];
    let mut count = 0u8;
    while count < 2 {
        for i in 0..16u8 {
            let idx = if count == 0 { i } else { 16 - i - 1 };
            value_set(Channel::All, EQ[idx as usize]);
            hddvi_rx::phy_soft_reset();
            clr_err_cnt();
            let mut delay = 0u8;
            while delay <= 5 {
                time::delay_ms(1);
                delay += 1;
                if exceeds_uplimit(Channel::Ch0)
                    && exceeds_uplimit(Channel::Ch1)
                    && exceeds_uplimit(Channel::Ch2)
                {
                    break;
                }
                if hddvi_rx::abnormal_deal() {
                    return false;
                }
            }
            if auc_ch[0][count as usize] == 0xff && in_good_band(Channel::Ch0) {
                auc_ch[0][count as usize] = idx;
            }
            if auc_ch[1][count as usize] == 0xff && in_good_band(Channel::Ch1) {
                auc_ch[1][count as usize] = idx;
            }
            if auc_ch[2][count as usize] == 0xff && in_good_band(Channel::Ch2) {
                auc_ch[2][count as usize] = idx;
            }
            if auc_ch[0][count as usize] != 0xff
                && auc_ch[1][count as usize] != 0xff
                && auc_ch[2][count as usize] != 0xff
            {
                break;
            }
        }
        // C `check_dir`: the backward scan only runs if the forward scan
        // found a good band for all 3 channels. If the forward pass ran
        // through all 16 EQ entries without completing, skip the backward
        // pass and compute midpoints from the forward-only results.
        if count == 0 && (auc_ch[0][0] == 0xff || auc_ch[1][0] == 0xff || auc_ch[2][0] == 0xff) {
            break;
        }
        count += 1;
    }

    let n0 = set_mid_eq(&auc_ch[0]);
    value_set(Channel::Ch0, EQ[n0 as usize]);
    let n1 = set_mid_eq(&auc_ch[1]);
    value_set(Channel::Ch1, EQ[n1 as usize]);
    let n2 = set_mid_eq(&auc_ch[2]);
    value_set(Channel::Ch2, EQ[n2 as usize]);

    log::debug!(
        "Receiver equalization: channel 0 range={}..{}, selected={}; channel 1 range={}..{}, selected={}; channel 2 range={}..{}, selected={}",
        auc_ch[0][0],
        auc_ch[0][1],
        n0,
        auc_ch[1][0],
        auc_ch[1][1],
        n1,
        auc_ch[2][0],
        auc_ch[2][1],
        n2
    );
    clr_err_cnt();
    true
}

pub fn eq_set() -> bool {
    reg::set_bit(0xE82C, 2);
    reg::set_bit(0xE828, 1);
    scan_lr()
}
