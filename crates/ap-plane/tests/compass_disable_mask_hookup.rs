//! Compass disable-mask: `COMPASS_DISBLMSK` / `_driver_enabled`.

use ap_compass::disable_mask::{sitl_enabled, DriverType, COMPASS_DISBLMSK_DEFAULT};
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_disable_mask_hookup::{
    apply_disable_mask, compass_disable_mask_tick, disable_sitl_driver,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::SitlCompassHookup;

#[test]
fn hookup_default_mask_leaves_sitl_healthy_path_open() {
    let mut hookup = SitlCompassHookup::default();
    let out = compass_disable_mask_tick(&hookup);
    assert_eq!(out.disable_mask, COMPASS_DISBLMSK_DEFAULT);
    assert!(out.sitl_enabled);
    assert!(!out.primary_disabled);
    hookup.truth.now_ms = 10;
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(published.healthy);
}

#[test]
fn hookup_sitl_mask_disables_primary_instance() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    disable_sitl_driver(&mut hookup);
    let out = compass_disable_mask_tick(&hookup);
    assert!(!sitl_enabled(out.disable_mask));
    assert!(out.primary_disabled);
    assert!(hookup.cluster().backend(0).expect("0").config().disabled);
    assert!(hookup.cluster().backend(1).expect("1").config().disabled);
    hookup.truth.now_ms = 10;
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(!published.healthy);
}

#[test]
fn main_loop_disblmsk_disables_sitl() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    apply_disable_mask(&mut hookup, DriverType::Sitl.mask_bit());
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let hookup = vehicle.sitl_compass.as_ref().expect("sitl compass");
    let out = compass_disable_mask_tick(hookup);
    assert!(!out.sitl_enabled);
    assert!(out.primary_disabled);
    assert!(!hookup.backend().expect("backend").healthy());
}
