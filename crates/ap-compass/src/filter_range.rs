//! Compass sample filter-range stub, upstream `COMPASS_FLTR_RNG`. FW-014.
//!
//! `_filter_range` is a percentage around the running mean field length.
//! `AP_Compass_Backend::field_ok` rejects a sample when
//! `|mean-len|/(mean+len)*200 > COMPASS_FLTR_RNG`. Zero disables the filter
//! (`HAL_COMPASS_FILTER_DEFAULT`).

use ap_math::scalar::is_zero;
use ap_math::vector3::Vector3f;

/// Upstream `COMPASS_FLTR_RNG` / `HAL_COMPASS_FILTER_DEFAULT` (`0` = off).
pub const COMPASS_FLTR_RNG_DEFAULT: u8 = 0;

/// Upstream `FILTER_KOEF` complementary-filter coefficient.
pub const FILTER_KOEF: f32 = 0.1;

/// Per-backend mean-length state, upstream `_mean_field_length` / `_error_count`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FilterRangeState {
    /// Running mean of accepted (and slowly of rejected) field lengths.
    pub mean_field_length: f32,
    /// Count of samples rejected by the range check.
    pub error_count: u32,
}

/// True when `COMPASS_FLTR_RNG` is non-zero so the mean filter is active.
#[must_use]
pub const fn filter_enabled(filter_range: u8) -> bool {
    filter_range > 0
}

impl FilterRangeState {
    /// Upstream `AP_Compass_Backend::field_ok`: accept or reject one sample.
    #[must_use]
    pub fn sample_ok(&mut self, field: Vector3f, filter_range: u8) -> bool {
        if field.is_inf() || field.is_nan() {
            return false;
        }

        let range = f32::from(filter_range);
        if range <= 0.0 {
            return true;
        }

        let length = field.length();
        if is_zero(self.mean_field_length) {
            self.mean_field_length = length;
            return true;
        }

        let mut ret = true;
        let d = (self.mean_field_length - length).abs() / (self.mean_field_length + length);
        let mut koeff = FILTER_KOEF;

        if d * 200.0 > range {
            ret = false;
            koeff /= d * 10.0;
            self.error_count = self.error_count.saturating_add(1);
        }
        self.mean_field_length = self.mean_field_length * (1.0 - koeff) + length * koeff;
        ret
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_range_disables_filter() {
        assert_eq!(COMPASS_FLTR_RNG_DEFAULT, 0);
        assert!(!filter_enabled(COMPASS_FLTR_RNG_DEFAULT));
        let mut state = FilterRangeState::default();
        let field = Vector3f::new(0.3, 0.1, 0.4);
        assert!(state.sample_ok(field, COMPASS_FLTR_RNG_DEFAULT));
        assert!(state.sample_ok(field * 10.0, COMPASS_FLTR_RNG_DEFAULT));
        assert_eq!(state.error_count, 0);
        assert!(is_zero(state.mean_field_length));
    }

    #[test]
    fn rejects_nan_and_inf_before_range() {
        let mut state = FilterRangeState::default();
        assert!(!state.sample_ok(Vector3f::new(f32::NAN, 0.0, 0.0), 0));
        assert!(!state.sample_ok(Vector3f::new(f32::INFINITY, 0.0, 0.0), 10));
    }

    #[test]
    fn first_sample_seeds_mean() {
        let mut state = FilterRangeState::default();
        let field = Vector3f::new(500.0, 0.0, 0.0);
        assert!(state.sample_ok(field, 10));
        assert!((state.mean_field_length - 500.0).abs() < 1e-6);
        assert_eq!(state.error_count, 0);
    }

    #[test]
    fn spike_outside_percent_range_is_rejected() {
        let mut state = FilterRangeState {
            mean_field_length: 500.0,
            error_count: 0,
        };
        // |500-1500|/(500+1500)*200 = 100, so range 10 rejects.
        let spike = Vector3f::new(1500.0, 0.0, 0.0);
        assert!(!state.sample_ok(spike, 10));
        assert_eq!(state.error_count, 1);
        assert!(state.mean_field_length > 500.0);
        assert!(state.mean_field_length < 550.0);
    }

    #[test]
    fn nearby_sample_is_accepted() {
        let mut state = FilterRangeState {
            mean_field_length: 500.0,
            error_count: 0,
        };
        // |500-550|/(500+550)*200 ≈ 9.52, so range 10 accepts.
        let near = Vector3f::new(550.0, 0.0, 0.0);
        assert!(state.sample_ok(near, 10));
        assert_eq!(state.error_count, 0);
    }
}
