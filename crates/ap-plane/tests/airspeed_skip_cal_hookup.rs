//! ARSPD_SKIP_CAL stub: skip requested pitot offset calibration.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::ARSPD_SKIP_CAL_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_offset_calibration_hookup::{
    airspeed_offset_calibration_tick, AirspeedOffsetCalibrationInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_skip_cal_allows_offset_latch() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!(!hookup.airspeed_params().primary_skip_cal());
    assert_eq!(hookup.airspeed_params().primary_skip_cal(), ARSPD_SKIP_CAL_DEFAULT);
    assert!(!published.skip_cal);

    let cal = airspeed_offset_calibration_tick(
        &mut hookup,
        AirspeedOffsetCalibrationInputs {
            request_calibrate: true,
        },
    );
    assert!(cal.calibrated);
    assert!((cal.primary_offset_mps - 3.0).abs() < 1e-6);
}

#[test]
fn hookup_skip_cal_blocks_primary_and_secondary_offset() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    let mut params = AirspeedParams::default();
    params.airspeed1.skip_cal = true;
    params.airspeed2.skip_cal = true;
    hookup.apply_airspeed_params(params);
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(4.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!(published.skip_cal);
    assert!(hookup.airspeed_params().primary_skip_cal());

    let cal = airspeed_offset_calibration_tick(
        &mut hookup,
        AirspeedOffsetCalibrationInputs {
            request_calibrate: true,
        },
    );
    assert!(!cal.calibrated);
    assert_eq!(cal.primary_offset_mps, 0.0);
    assert_eq!(hookup.cluster().backend(0).unwrap().config().offset_mps, 0.0);
    assert_eq!(hookup.cluster().backend(1).unwrap().config().offset_mps, 0.0);
}

#[test]
fn main_loop_ahrs_update_honors_skip_cal() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .set_skip_cal(true);
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert!(vehicle.airspeed_skip_cal);
    assert!(!vehicle.airspeed_offset_calibrated);

    vehicle.airspeed_calibrate_requested = true;
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert!(vehicle.airspeed_skip_cal);
    assert!(!vehicle.airspeed_offset_calibrated);
    assert!(!vehicle.airspeed_calibrate_requested);
    assert!((vehicle.airspeed_tas - 3.0).abs() < 1e-6);
}
