//! Pitot offset calibration stub: latch raw TAS, then zero a biased tube.

use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_offset_calibration_hookup::{
    airspeed_offset_calibration_tick, AirspeedOffsetCalibrationInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_calibrate_zeros_biased_pitot() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
        now_ms: 10,
    };
    let first = hookup.publish(1.0);
    assert!((first.sample.tas_mps - 3.0).abs() < 1e-6);

    let cal = airspeed_offset_calibration_tick(
        &mut hookup,
        AirspeedOffsetCalibrationInputs {
            request_calibrate: true,
        },
    );
    assert!(cal.calibrated);
    assert!((cal.primary_offset_mps - 3.0).abs() < 1e-6);
    assert_eq!(hookup.cluster().instance_count(), 2);
    assert!((hookup.cluster().backend(1).unwrap().config().offset_mps - 3.0).abs() < 1e-6);

    hookup.truth.now_ms = 20;
    let rest = hookup.publish(1.0);
    assert!(rest.sample.tas_mps.abs() < 1e-6);

    hookup.truth.airspeed_bf = Vector3f::new(23.0, 0.0, 0.0);
    hookup.truth.now_ms = 30;
    let flying = hookup.publish(1.0);
    assert!((flying.sample.tas_mps - 20.0).abs() < 1e-6);
}

#[test]
fn main_loop_calibrate_request_latches_offset() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::default());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert!((vehicle.airspeed_tas - 3.0).abs() < 1e-6);
    assert!(!vehicle.airspeed_offset_calibrated);

    vehicle.airspeed_calibrate_requested = true;
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();
    assert!(vehicle.airspeed_offset_calibrated);
    assert!(!vehicle.airspeed_calibrate_requested);

    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 30;
    vehicle.ahrs_update();
    assert!(vehicle.airspeed_tas.abs() < 1e-6);
}
