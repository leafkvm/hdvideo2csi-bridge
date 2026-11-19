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
    /// The other field's measured vertical timing, only after alternating
    /// hardware field IDs have been observed. Base geometry is field ID 0.
    pub second_field: Option<FieldTiming>,
}

/// One CSI field's line counts. The transmitter does not weave two fields.
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub struct FieldTiming {
    pub total: u16,
    pub active: u16,
    pub front_porch: u16,
    pub sync: u16,
    pub back_porch: u16,
}

impl Timing {
    pub fn field(self) -> FieldTiming {
        FieldTiming {
            total: self.v_total,
            active: self.v_active,
            front_porch: self.v_fp,
            sync: self.v_sw,
            back_porch: self.v_bp,
        }
    }

    pub fn interlaced(self) -> bool {
        self.second_field.is_some()
    }

    pub fn frame_height(self) -> u32 {
        u32::from(self.v_active) + self.second_field.map_or(0, |f| u32::from(f.active))
    }

    pub fn frame_total(self) -> u32 {
        u32::from(self.v_total) + self.second_field.map_or(0, |f| u32::from(f.total))
    }

    /// Pixel clock is kHz, hence the factor of one million for millihertz.
    pub fn frame_rate_millihz(self) -> u64 {
        let total = u64::from(self.h_total) * u64::from(self.frame_total());
        (u64::from(self.pixel_clock) * 1_000_000)
            .checked_div(total)
            .unwrap_or(0)
    }

    /// Sane sizes whose porches add up to the totals. Odd sizes are fine:
    /// the transmitter is programmed with the measured size and the capture
    /// side takes it as is.
    pub fn coherent(self) -> bool {
        self.h_active > 80
            && self.h_active < 5220
            && self.v_active > 80
            && self.v_active < 5220
            && u32::from(self.h_total)
                == u32::from(self.h_active)
                    + u32::from(self.h_fp)
                    + u32::from(self.h_sw)
                    + u32::from(self.h_bp)
            && u32::from(self.v_total)
                == u32::from(self.v_active)
                    + u32::from(self.v_fp)
                    + u32::from(self.v_sw)
                    + u32::from(self.v_bp)
    }

    fn same_horizontal(self, other: Self) -> bool {
        (
            self.h_total,
            self.h_active,
            self.h_fp,
            self.h_sw,
            self.h_bp,
            self.h_spol,
            self.v_spol,
        ) == (
            other.h_total,
            other.h_active,
            other.h_fp,
            other.h_sw,
            other.h_bp,
            other.h_spol,
            other.v_spol,
        )
    }
}

/// Interpret repeated coherent snapshots, never infer scan mode from dimensions.
/// E580[7:6] is the vendor field indicator: values 0/1 alternate on interlace.
/// A constant indicator alone is insufficient: progressive also requires bit 2
/// (video-check stable). Values 2/3 are unqualified and yield no observation.
/// Callers must sample across several fields with nonuniform delays.
#[derive(Default)]
pub struct ScanDetector {
    fields: [Option<Timing>; 2],
    counts: [u8; 2],
    last_field: Option<usize>,
    transitions: u8,
    progressive_good: u8,
    // Observation age of each field; stale samples cannot qualify a new lock.
    ages: [u8; 2],
}

impl ScanDetector {
    pub fn observe(&mut self, before: u8, timing: Timing, after: u8) {
        // A snapshot is about 24 register reads, 11-12 ms: most of a 60 Hz
        // field, so most snapshots of interlaced input straddle a field
        // change. They carry no evidence and do not age the evidence so far.
        if before >> 6 != after >> 6 {
            self.progressive_good = 0;
            return;
        }
        for age in &mut self.ages {
            *age = age.saturating_add(1);
        }
        if self.ages.iter().all(|&age| age >= 8) {
            *self = Self::default();
        }
        let field = usize::from(before >> 6);
        if field > 1 || !timing.coherent() {
            self.progressive_good = 0;
            return;
        }
        // A source change invalidates the accumulated evidence. Pixel clock is
        // measured separately after acquisition, and is zero in these samples.
        if self
            .fields
            .iter()
            .flatten()
            .any(|old| !old.same_horizontal(timing))
            || self.fields[field].is_some_and(|old| old.field() != timing.field())
        {
            *self = Self::default();
        }
        if self.last_field.is_some_and(|last| last != field) {
            self.transitions = self.transitions.saturating_add(1);
        }
        self.last_field = Some(field);
        self.fields[field] = Some(timing);
        self.ages[field] = 0;
        self.counts[field] = self.counts[field].saturating_add(1);
        if before & after & 4 != 0 {
            self.progressive_good = self.progressive_good.saturating_add(1);
        } else {
            self.progressive_good = 0;
        }
    }

    /// Unknown/unstable measurements are rejected instead of labelled progressive.
    pub fn finish(&self) -> Option<Timing> {
        let last = self.last_field?;
        if self.ages[last] > 2 {
            return None;
        }
        if self.transitions >= 4
            && self.counts.iter().all(|&n| n >= 3)
            && self.ages.iter().all(|&age| age <= 16)
        {
            let mut first = self.fields[0]?;
            let second = self.fields[1]?;
            // Native HDMI interlace starts one field half a line later, which
            // rounds sync and back porch either way: back porch 15 and 16 for
            // 1080i and 480i, sync 2 and 3 for 1024x768i at 87 Hz (Radeon HDMI).
            if first.v_active != second.v_active
                || first.v_sw.abs_diff(second.v_sw) > 1
                || first.v_bp.abs_diff(second.v_bp) > 1
                || first.v_total.abs_diff(second.v_total) > 1
            {
                return None;
            }
            first.second_field = Some(second.field());
            Some(first)
        } else if self.transitions == 0 && self.progressive_good >= 8 {
            self.fields[self.last_field?]
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 480i field timing as measured through the VGA adapter: field ID 0
    /// has 263 lines, field ID 1 262 (one more front porch line).
    fn field_480i(field: u8) -> Timing {
        Timing {
            h_total: 1716,
            h_active: 1440,
            h_fp: 38,
            h_sw: 124,
            h_bp: 114,
            v_total: 263 - u16::from(field),
            v_active: 240,
            v_fp: 5 - u16::from(field),
            v_sw: 3,
            v_bp: 15,
            ..Timing::default()
        }
    }

    /// The tail of a scan from the rig: a run of straddling snapshots
    /// (indicator changed during the reads, geometry of either field) ended
    /// a scan, so the evidence aged out and the lock failed.
    #[test]
    fn straddling_snapshots_do_not_age_the_evidence() {
        let mut scan = ScanDetector::default();
        for _ in 0..4 {
            scan.observe(0x00, field_480i(0), 0x00);
            scan.observe(0x40, field_480i(1), 0x40);
        }
        for i in 0..12u8 {
            let (before, after) = if i % 2 == 0 {
                (0x40, 0x00)
            } else {
                (0x00, 0x40)
            };
            scan.observe(before, field_480i(i % 2), after);
        }
        scan.observe(0x00, field_480i(0), 0x00);
        let timing = scan.finish().expect("interlaced lock");
        assert!(timing.interlaced());
        assert_eq!(timing.frame_total(), 525);
        assert_eq!(timing.frame_height(), 480);
    }

    /// 1080i60 from a Radeon HDMI output: the fields differ in back porch
    /// as well as in front porch and total.
    #[test]
    fn fields_may_differ_by_one_back_porch_line() {
        let field = |id: u8| Timing {
            h_total: 2200,
            h_active: 1920,
            h_fp: 88,
            h_sw: 44,
            h_bp: 148,
            v_total: 563 - u16::from(id),
            v_active: 540,
            v_fp: 3 - 2 * u16::from(id),
            v_sw: 5,
            v_bp: 15 + u16::from(id),
            ..Timing::default()
        };
        let mut scan = ScanDetector::default();
        for _ in 0..4 {
            scan.observe(0x30, field(0), 0x30);
            scan.observe(0x70, field(1), 0x70);
        }
        let timing = scan.finish().expect("interlaced lock");
        assert!(timing.interlaced());
        assert_eq!(timing.frame_total(), 1125);
        assert_eq!(timing.frame_height(), 1080);
    }

    #[test]
    fn fields_may_differ_by_one_sync_line() {
        // 1024x768i at 87 Hz from Radeon HDMI, as measured.
        let field = |v_total, v_fp, v_sw, v_bp| Timing {
            h_total: 1264,
            h_active: 1024,
            h_fp: 8,
            h_sw: 176,
            h_bp: 56,
            v_total,
            v_active: 384,
            v_fp,
            v_sw,
            v_bp,
            ..Timing::default()
        };
        let mut scan = ScanDetector::default();
        for _ in 0..4 {
            scan.observe(0x30, field(409, 2, 3, 20), 0x30);
            scan.observe(0x70, field(408, 1, 2, 21), 0x70);
        }
        let timing = scan.finish().expect("interlaced lock");
        assert!(timing.interlaced());
        assert_eq!(timing.frame_total(), 817);
        assert_eq!(timing.frame_height(), 768);
    }

    #[test]
    fn steady_stable_snapshots_are_progressive() {
        let mut scan = ScanDetector::default();
        let mut t = field_480i(1);
        t.v_active = 480;
        t.v_total = 502;
        for _ in 0..10 {
            scan.observe(0x4c, t, 0x4c);
        }
        let timing = scan.finish().expect("progressive lock");
        assert!(!timing.interlaced());
    }
}
