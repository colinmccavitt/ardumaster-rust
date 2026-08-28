//! Compass consistency stub: `Compass::consistent()`.

use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
use ap_compass::params::CompassParams;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_consistent_hookup::compass_consistent_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn dual_matching_instances_are_consistent() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
    let out = compass_consistent_tick(&hookup);
    assert_eq!(out.instance_count, 2);
    assert_eq!(out.checked, 2);
    assert!(out.consistent);
}

#[test]
fn unused_yawed_secondary_is_skipped() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass2.orientation = COMPASS_ORIENT_YAW_90;
    params.compass2.use_for_yaw = false;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
    let out = compass_consistent_tick(&hookup);
    assert_eq!(out.checked, 1);
    assert!(out.consistent);
}

#[test]
fn main_loop_reports_inconsistent_yawed_pair() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass2.orientation = COMPASS_ORIENT_YAW_90;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
    vehicle.sitl_compass = Some(hookup);

    let out = compass_consistent_tick(vehicle.sitl_compass.as_ref().expect("sitl compass"));
    assert!(!out.consistent);
}
