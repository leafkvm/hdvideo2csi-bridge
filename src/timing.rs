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

    pub fn coherent(self) -> bool {
        self.h_active > 80
            && self.h_active < 5220
            && self.v_active > 80
            && self.v_active < 5220
            && self.h_active & 1 == 0
            && self.v_active & 1 == 0
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
        for age in &mut self.ages {
            *age = age.saturating_add(1);
        }
        if self.ages.iter().all(|&age| age >= 8) {
            *self = Self::default();
        }
        let field = usize::from(before >> 6);
        if field > 1 || before >> 6 != after >> 6 || !timing.coherent() {
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
    pub fn finish(self) -> Option<Timing> {
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
            if first.v_active != second.v_active
                || first.v_sw != second.v_sw
                || first.v_bp != second.v_bp
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
