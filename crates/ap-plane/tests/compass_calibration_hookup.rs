//! Compass MAG_CAL start/cancel: `Compass::start_calibration_all`.

use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_calibration_hookup::{
    compass_calibration_tick, CompassCalibrationInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

fn healthy_hookup() -> SitlCompassHookup {
    let mut hookup = SitlCompassHookup::default();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
    hookup
}

#[test]
fn hookup_start_marks_yaw_sample_calibrating() {
    let mut hookup = healthy_hookup();
    let out = compass_calibration_tick(
        &mut hookup,
        CompassCalibrationInputs {
            request_start: true,
            request_cancel: false,
        },
    );
    assert!(out.started);
    assert!(out.calibrating);
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    let yaw = published.yaw_compass.expect("yaw sample");
    assert!(yaw.calibrating);
}

#[test]
fn main_loop_cancel_clears_calibrating() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_compass = Some(healthy_hookup());

    let hookup = vehicle.sitl_compass.as_mut().expect("sitl compass");
    let started = compass_calibration_tick(
        hookup,
        CompassCalibrationInputs {
            request_start: true,
            request_cancel: false,
        },
    );
    assert!(started.calibrating);
    let cancelled = compass_calibration_tick(
        hookup,
        CompassCalibrationInputs {
            request_start: false,
            request_cancel: true,
        },
    );
    assert!(cancelled.cancelled);
    assert!(!cancelled.calibrating);
    let published = hookup.publish(Matrix3f::identity(), 0.0025, None);
    let yaw = published.yaw_compass.expect("yaw sample");
    assert!(!yaw.calibrating);
}
