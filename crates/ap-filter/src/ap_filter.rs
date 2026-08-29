//! Configurable notch lookup, upstream `Filter/AP_Filter`.
//!
//! `AC_PID::set_notch_sample_rate` (COP-008) looks up a filter by the PID's
//! NTF/NEF index and asks it to configure a `NotchFilterFloat`. The parameter
//! table and the 1 Hz `AP_Filters::update` allocator stay on FW-003; this
//! module is the hook that leftover needs.
//!
//! ADR-0004 rules out `AP::filters()`, so the table is passed in.

use ap_math::scalar::{is_equal, is_zero};

use crate::notch::NotchFilter;

/// Upstream `AP_FILTER_NUM_FILTERS` on boards with `HAL_PROGRAM_SIZE_LIMIT_KB > 1024`.
pub const AP_FILTER_NUM_FILTERS: u8 = 8;

/// What `AP_NotchFilter_params` holds for `setup_notch_filter`.
///
/// Gains are plain fields, not `AP_Float`. Same approach as `AcPid`: the
/// arithmetic is unaffected; only the binding to storage is deferred.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NotchFilterParams {
    /// Centre frequency, Hz. Upstream `_center_freq_hz`.
    pub center_freq_hz: f32,
    /// Quality factor (centre / bandwidth). Upstream `_quality`.
    pub quality: f32,
    /// Attenuation, dB. Upstream `_attenuation_dB`.
    pub attenuation_db: f32,
}

impl NotchFilterParams {
    /// Configure `filter` for `sample_rate`, upstream `setup_notch_filter`.
    ///
    /// Returns `false` when any of the three parameters is zero, which is
    /// what makes `set_notch_sample_rate` drop the notch and clear the
    /// index. A rejected `init` still returns `true` � upstream does not
    /// inspect `initialised` after `init`.
    pub fn setup_notch_filter(&self, filter: &mut NotchFilter<f32>, sample_rate: f32) -> bool {
        if is_zero(self.quality) || is_zero(self.center_freq_hz) || is_zero(self.attenuation_db) {
            return false;
        }

        if !is_equal(sample_rate, filter.sample_freq())
            || !is_equal(self.center_freq_hz, filter.center_freq())
        {
            filter.init(
                sample_rate,
                self.center_freq_hz,
                self.center_freq_hz / self.quality,
                self.attenuation_db,
            );
        }
        true
    }
}

/// Lookup for `AP::filters().get_filter(id)`.
///
/// Index is 1-based, matching the FILT1..FILTn parameter groups.
pub trait NotchFilterSource {
    /// The notch at `index`, or `None` for a null pointer.
    fn get_filter(&self, index: u8) -> Option<NotchFilterParams>;
}

/// No `AP_Filters` singleton is present. Every lookup is a null pointer.
impl NotchFilterSource for () {
    fn get_filter(&self, _index: u8) -> Option<NotchFilterParams> {
        None
    }
}

/// 1-based table, upstream `AP_Filters`.
///
/// Slots stay empty until a caller installs params � the 1 Hz allocator
/// that new's `AP_NotchFilter_params` from a type parameter is FW-003.
#[derive(Debug, Clone, Copy)]
pub struct Filters {
    slots: [Option<NotchFilterParams>; AP_FILTER_NUM_FILTERS as usize],
}

impl Default for Filters {
    fn default() -> Self {
        Self {
            slots: [None; AP_FILTER_NUM_FILTERS as usize],
        }
    }
}

impl Filters {
    /// An empty table. Every lookup is a null pointer until [`Filters::set`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Install a notch at the 1-based index `set_notch_sample_rate` looks up.
    ///
    /// Index 8 writes the last slot, but [`Filters::get_filter`] rejects it
    /// � upstream's `index >= AP_FILTER_NUM_FILTERS` guard, reproduced.
    pub fn set(&mut self, index: u8, params: NotchFilterParams) {
        if index == 0 {
            return;
        }
        if let Some(slot) = self.slots.get_mut(usize::from(index) - 1) {
            *slot = Some(params);
        }
    }
}

impl NotchFilterSource for Filters {
    /// Upstream `AP_Filters::get_filter`.
    ///
    /// `index >= AP_FILTER_NUM_FILTERS` is a null pointer, so FILT8 is
    /// unreachable. Index 0 is never passed by `set_notch_sample_rate`
    /// (it returns early); if it were, `filters[index-1]` would wrap,
    /// which is also out of range here.
    fn get_filter(&self, index: u8) -> Option<NotchFilterParams> {
        if index >= AP_FILTER_NUM_FILTERS {
            return None;
        }
        let slot = usize::from(index.wrapping_sub(1));
        self.slots.get(slot).copied().flatten()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    fn valid() -> NotchFilterParams {
        NotchFilterParams {
            center_freq_hz: 100.0,
            quality: 2.0,
            attenuation_db: 40.0,
        }
    }

    #[test]
    fn zero_params_refuse_setup() {
        let mut f = NotchFilter::<f32>::new();
        for broken in [
            NotchFilterParams {
                center_freq_hz: 0.0,
                ..valid()
            },
            NotchFilterParams {
                quality: 0.0,
                ..valid()
            },
            NotchFilterParams {
                attenuation_db: 0.0,
                ..valid()
            },
        ] {
            assert!(!broken.setup_notch_filter(&mut f, 400.0));
            assert!(!f.is_initialised());
        }
    }

    #[test]
    fn setup_inits_from_centre_over_quality() {
        let mut f = NotchFilter::<f32>::new();
        assert!(valid().setup_notch_filter(&mut f, 400.0));
        assert!(f.is_initialised());
        assert_eq!(f.sample_freq(), 400.0);
        assert_eq!(f.center_freq(), 100.0);
    }

    #[test]
    fn setup_skips_an_unchanged_rate_and_centre() {
        let mut f = NotchFilter::<f32>::new();
        assert!(valid().setup_notch_filter(&mut f, 400.0));
        let before = f.coefficients();
        assert!(valid().setup_notch_filter(&mut f, 400.0));
        assert_eq!(f.coefficients(), before);
    }

    #[test]
    fn get_filter_is_one_based_and_rejects_eight() {
        let mut filters = Filters::new();
        filters.set(1, valid());
        filters.set(8, valid());
        assert_eq!(filters.get_filter(1), Some(valid()));
        assert_eq!(filters.get_filter(8), None, "upstream index >= NUM is null");
        assert_eq!(filters.get_filter(0), None);
        assert_eq!(filters.get_filter(9), None);
        assert_eq!(().get_filter(1), None);
    }
}
