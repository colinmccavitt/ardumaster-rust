//! Compass external / orientation stub: COMPASS_ORIENT / COMPASS_EXTERNAL.

use ap_compass::orientation::{apply_orientation, rotate_field, COMPASS_ORIENT_YAW_90};
use ap_compass::params::CompassParams;
use ap_compass::sitl::mag_field_body_ned;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_orientation_hookup::compass_orientation_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn hookup_yaw90_rotates_published_field() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass1.orientation = COMPASS_ORIENT_YAW_90;
    params.compass2.orientation = COMPASS_ORIENT_YAW_90;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    let out = compass_orientation_tick(&hookup);
    assert_eq!(out.orientation, COMPASS_ORIENT_YAW_90);
    assert!(!out.external);

    let attitude = Matrix3f::identity();
    let published = hookup.publish(attitude, 0.0025, None);
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, attitude);
    let expected = apply_orientation(wmm, COMPASS_ORIENT_YAW_90);
    assert!((published.sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((published.sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((published.sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn main_loop_external_skips_board_orientation() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.compass1.external = true;
    params.board_orientation = COMPASS_ORIENT_YAW_90;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = vehicle.mag_sample.expect("mag sample");
    let expected = rotate_field(wmm, 0, true, COMPASS_ORIENT_YAW_90);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - wmm.z).abs() < 1e-5);
}
