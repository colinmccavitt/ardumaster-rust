//! Compass primary instance selection: `Compass::get_first_usable`.

use ap_compass::params::CompassParams;
use ap_compass::primary::first_usable;

#[test]
fn params_default_first_usable_is_zero() {
    let params = CompassParams::default();
    assert!(params.compass1.use_for_yaw);
    assert!(params.compass2.use_for_yaw);
    assert_eq!(params.first_usable(), 0);
    assert_eq!(
        first_usable(&[params.compass1.use_for_yaw, params.compass2.use_for_yaw]),
        0
    );
}

#[test]
fn use2_only_selects_secondary() {
    let mut params = CompassParams::default();
    params.compass1.use_for_yaw = false;
    params.compass2.use_for_yaw = true;
    assert_eq!(params.first_usable(), 1);
}

#[test]
fn all_use_disabled_stays_zero() {
    let mut params = CompassParams::default();
    params.compass1.use_for_yaw = false;
    params.compass2.use_for_yaw = false;
    assert_eq!(params.first_usable(), 0);
}
