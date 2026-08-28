//! Disable use-for-yaw when compasses fail `Compass::consistent()`.

use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
use ap_compass::params::CompassParams;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_use_for_yaw_hookup::compass_use_for_yaw_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn matching_dual_instances_keep_use_for_yaw() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(published.yaw_compass.is_some());
    let out = compass_use_for_yaw_tick(&mut hookup);
    assert!(out.consistent);
    assert!(out.use_for_yaw);
    let again = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(again.yaw_compass.is_some());
}

#[test]
fn unused_yawed_secondary_keeps_use_for_yaw() {
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
    let out = compass_use_for_yaw_tick(&mut hookup);
    assert!(out.consistent);
    assert!(out.use_for_yaw);
}

#[test]
fn main_loop_inconsistent_pair_drops_yaw_compass() {
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

    let hookup = vehicle.sitl_compass.as_mut().expect("sitl compass");
    let out = compass_use_for_yaw_tick(hookup);
    assert!(!out.consistent);
    assert!(!out.use_for_yaw);
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(published.yaw_compass.is_none());
}
