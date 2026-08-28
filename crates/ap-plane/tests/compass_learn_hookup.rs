//! COMPASS_LEARN mode enum stub: `Compass::LearnType`.

use ap_compass::learn::LearnType;
use ap_compass::offset::{COMPASS_LEARN_EKF, COMPASS_LEARN_INFLIGHT};
use ap_compass::params::CompassParams;
use ap_ins::LoopTiming;
use ap_plane::compass_learn_hookup::compass_learn_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::SitlCompassHookup;

#[test]
fn hookup_reports_learn_mode() {
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.learn = COMPASS_LEARN_EKF;
    hookup.apply_compass_params(params);

    let out = compass_learn_tick(&hookup);
    assert_eq!(out.learn, COMPASS_LEARN_EKF);
    assert_eq!(out.mode, Some(LearnType::CopyFromEkf));
    assert!(!out.inflight_learn);
    assert!(out.offsets_learn);
}

#[test]
fn main_loop_learn_mode_is_readable() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.learn = COMPASS_LEARN_INFLIGHT;
    hookup.apply_compass_params(params);
    vehicle.sitl_compass = Some(hookup);

    let out = compass_learn_tick(vehicle.sitl_compass.as_ref().expect("sitl compass"));
    assert_eq!(out.mode, Some(LearnType::Inflight));
    assert!(out.inflight_learn);
}
