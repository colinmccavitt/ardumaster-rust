//! Compass filter-range stub: `COMPASS_FLTR_RNG`.

use ap_compass::filter_range::{filter_enabled, FilterRangeState, COMPASS_FLTR_RNG_DEFAULT};
use ap_compass::params::CompassParams;
use ap_math::vector3::Vector3f;

#[test]
fn default_params_leave_filter_disabled() {
    let params = CompassParams::default();
    assert_eq!(params.filter_range, COMPASS_FLTR_RNG_DEFAULT);
    assert!(!filter_enabled(params.filter_range));
}

#[test]
fn enabled_range_rejects_length_spike() {
    let mut params = CompassParams::default();
    params.filter_range = 10;
    assert!(filter_enabled(params.filter_range));
    let mut state = FilterRangeState::default();
    assert!(state.sample_ok(Vector3f::new(500.0, 0.0, 0.0), params.filter_range));
    assert!(!state.sample_ok(Vector3f::new(1500.0, 0.0, 0.0), params.filter_range));
    assert_eq!(state.error_count, 1);
}
