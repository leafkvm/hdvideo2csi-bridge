#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub struct Timing {
    pub pixel_clock: u32,
    pub h_total: u16,
    pub h_active: u16,
    pub h_fp: u16,
    pub h_sw: u16,
    pub h_bp: u16,
    pub v_total: u16,
    pub v_active: u16,
    pub v_fp: u16,
    pub v_sw: u16,
    pub v_bp: u16,
    pub h_spol: bool,
    pub v_spol: bool,
}
