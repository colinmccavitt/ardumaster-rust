//! Compass auto-orientation stub: `COMPASS_AUTO_ROT`.

use ap_compass::auto_rot::{
    AutoRot, COMPASS_AUTO_ROT_CHECK_AND_FIX, COMPASS_AUTO_ROT_CHECK_ONLY, COMPASS_AUTO_ROT_FIX_45,
};
use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
use ap_compass::params::CompassParams;
use ap_ins::LoopTiming;
use ap_plane::compass_auto_rot_hookup::{
    apply_auto_orientation, compass_auto_rot_start_settings, compass_auto_rot_tick,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::SitlCompassHookup;

#[test]
fn hookup_check_and_fix_saves_external_orient() {
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.rotate_auto = COMPASS_AUTO_ROT_CHECK_AND_FIX;
    params.compass1.external = true;
    hookup.apply_compass_params(params);

    let out = compass_auto_rot_tick(&hookup);
    assert_eq!(out.mode, Some(AutoRot::CheckAndFix));
    let settings = compass_auto_rot_start_settings(&hookup, 0).expect("settings");
    assert!(settings.check_orientation);
    assert!(settings.fix_orientation);
    assert!(apply_auto_orientation(
        &mut hookup,
        0,
        COMPASS_ORIENT_YAW_90
    ));
    assert_eq!(
        hookup.compass_params().compass1.orientation,
        COMPASS_ORIENT_YAW_90
    );
}

#[test]
fn main_loop_internal_does_not_save_orient() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.rotate_auto = COMPASS_AUTO_ROT_FIX_45;
    params.compass1.external = false;
    hookup.apply_compass_params(params);
    vehicle.sitl_compass = Some(hookup);

    let hookup = vehicle.sitl_compass.as_mut().expect("sitl compass");
    let out = compass_auto_rot_tick(hookup);
    assert_eq!(out.mode, Some(AutoRot::CheckAndFix45));
    assert!(out.always_45_deg);
    let settings = compass_auto_rot_start_settings(hookup, 0).expect("internal start");
    assert_eq!(settings.orientation, 0);
    assert!(!settings.is_external);
    assert!(!apply_auto_orientation(hookup, 0, COMPASS_ORIENT_YAW_90));
    assert_eq!(hookup.compass_params().compass1.orientation, 0);

    // CheckOnly still starts a check but does not rewrite.
    let mut params = *hookup.compass_params();
    params.rotate_auto = COMPASS_AUTO_ROT_CHECK_ONLY;
    params.compass1.external = true;
    hookup.apply_compass_params(params);
    assert!(!apply_auto_orientation(hookup, 0, COMPASS_ORIENT_YAW_90));
}
