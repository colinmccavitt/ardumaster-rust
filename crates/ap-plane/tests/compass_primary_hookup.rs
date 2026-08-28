//! Compass primary instance selection: `Compass::get_first_usable`.

use ap_compass::params::CompassParams;
use ap_ins::LoopTiming;
use ap_plane::compass_primary_hookup::compass_primary_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::SitlCompassHookup;

#[test]
fn hookup_selects_secondary_when_use1_disabled() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass1.use_for_yaw = false;
    params.compass2.use_for_yaw = true;
    hookup.apply_compass_params(params);

    let out = compass_primary_tick(&mut hookup);
    assert_eq!(out.first_usable, 1);
    assert_eq!(out.primary, 1);
    assert_eq!(hookup.cluster().primary(), 1);
}

#[test]
fn main_loop_first_usable_follows_compass_use() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass1.use_for_yaw = false;
    hookup.apply_compass_params(params);
    vehicle.sitl_compass = Some(hookup);

    let out = compass_primary_tick(vehicle.sitl_compass.as_mut().expect("sitl compass"));
    assert_eq!(out.first_usable, 1);
    assert_eq!(out.primary, 1);
}
