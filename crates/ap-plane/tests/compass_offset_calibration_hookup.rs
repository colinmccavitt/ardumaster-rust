//! Mag offset / learn-offsets stub: apply COMPASS_OFS, then cancel hard-iron.

use ap_compass::offset::COMPASS_LEARN_INFLIGHT;
use ap_compass::params::CompassParams;
use ap_compass::sitl::mag_field_body_ned;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::compass_offset_calibration_hookup::{
    compass_offset_calibration_tick, CompassOffsetCalibrationInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn hookup_learn_cancels_hardiron_bias() {
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.learn = COMPASS_LEARN_INFLIGHT;
    hookup.apply_compass_params(params);
    let bias = Vector3f::new(0.05, -0.02, 0.01);
    hookup.set_hardiron_bias(bias);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    let attitude = Matrix3f::identity();
    let first = hookup.publish(attitude, 0.0025, None);
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, attitude);
    assert!((first.sample.mag_body.x - (wmm.x + bias.x)).abs() < 1e-5);

    let cal = compass_offset_calibration_tick(
        &mut hookup,
        CompassOffsetCalibrationInputs {
            request_learn: true,
        },
    );
    assert!(cal.learned);
    assert!((cal.primary_offset.x + bias.x).abs() < 1e-5);
    assert_eq!(hookup.cluster().instance_count(), 2);
    assert!((hookup.cluster().backend(1).unwrap().config().offset.x + bias.x).abs() < 1e-5);

    hookup.truth.now_ms = 20;
    let rest = hookup.publish(attitude, 0.0025, None);
    assert!((rest.sample.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((rest.sample.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((rest.sample.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn main_loop_learn_request_latches_offset() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.learn = COMPASS_LEARN_INFLIGHT;
    hookup.apply_compass_params(params);
    hookup.set_hardiron_bias(Vector3f::new(0.05, 0.0, 0.0));
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let first = vehicle.mag_sample.expect("mag sample");
    assert!((first.mag_body.x - (wmm.x + 0.05)).abs() < 1e-5);
    assert!(!vehicle.compass_offsets_learned);

    vehicle.compass_learn_requested = true;
    vehicle.sitl_compass.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();
    assert!(vehicle.compass_offsets_learned);
    assert!(!vehicle.compass_learn_requested);

    vehicle.sitl_compass.as_mut().unwrap().truth.now_ms = 30;
    vehicle.ahrs_update();
    let after = vehicle.mag_sample.expect("learned mag sample");
    assert!((after.mag_body.x - wmm.x).abs() < 1e-5);
}
