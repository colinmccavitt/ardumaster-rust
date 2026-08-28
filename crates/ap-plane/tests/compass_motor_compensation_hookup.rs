//! Compass motor compensation stub: COMPASS_MOT current-based hard-iron.

use ap_compass::motor_comp::COMPASS_MOT_COMP_CURRENT;
use ap_compass::params::CompassParams;
use ap_compass::sitl::mag_field_body_ned;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::compass_motor_compensation_hookup::{
    compass_motor_compensation_tick, CompassMotorCompensationInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn hookup_current_shifts_published_field() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.motor_comp_type = COMPASS_MOT_COMP_CURRENT;
    let mot = Vector3f::new(0.01, -0.02, 0.005);
    params.compass1.motor_compensation = mot;
    params.compass2.motor_compensation = mot;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    let current = 10.0;
    let mot_out = compass_motor_compensation_tick(
        &mut hookup,
        CompassMotorCompensationInputs {
            thr_or_curr: current,
        },
    );
    assert!(mot_out.enabled);
    assert!((mot_out.motor_offset.x - 0.1).abs() < 1e-6);

    let attitude = Matrix3f::identity();
    let published = hookup.publish(attitude, 0.0025, None);
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, attitude);
    assert!((published.sample.mag_body.x - (wmm.x + 0.1)).abs() < 1e-5);
    assert!((published.sample.mag_body.y - (wmm.y - 0.2)).abs() < 1e-5);
    assert!((published.sample.mag_body.z - (wmm.z + 0.05)).abs() < 1e-5);
}

#[test]
fn main_loop_current_applies_motor_offset() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.motor_comp_type = COMPASS_MOT_COMP_CURRENT;
    params.compass1.motor_compensation = Vector3f::new(0.01, 0.0, 0.0);
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    vehicle.sitl_compass = Some(hookup);
    vehicle.compass_battery_current_amps = 10.0;

    vehicle.ahrs_update();
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = vehicle.mag_sample.expect("mag sample");
    assert!((sample.mag_body.x - (wmm.x + 0.1)).abs() < 1e-5);
    assert!((sample.mag_body.y - wmm.y).abs() < 1e-5);
}
