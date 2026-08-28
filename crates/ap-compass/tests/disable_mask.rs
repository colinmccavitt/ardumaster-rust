//! Compass disable driver-type mask stub, upstream `COMPASS_DISBLMSK`.

use ap_compass::disable_mask::{
    driver_enabled, instance_disabled, sitl_enabled, DriverType, COMPASS_DISBLMSK_DEFAULT,
};
use ap_compass::params::CompassParams;

#[test]
fn default_params_leave_sitl_enabled() {
    let params = CompassParams::default();
    assert_eq!(params.disable_mask, COMPASS_DISBLMSK_DEFAULT);
    assert!(sitl_enabled(params.disable_mask));
    assert!(!instance_disabled(
        params.disable_mask,
        params.compass1.disabled
    ));
}

#[test]
fn masking_sitl_disables_both_instances() {
    let mut params = CompassParams::default();
    params.disable_mask = DriverType::Sitl.mask_bit();
    assert!(!driver_enabled(params.disable_mask, DriverType::Sitl));
    assert!(instance_disabled(
        params.disable_mask,
        params.compass1.disabled
    ));
    assert!(instance_disabled(
        params.disable_mask,
        params.compass2.disabled
    ));
}
