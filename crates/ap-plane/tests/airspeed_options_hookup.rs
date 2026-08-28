//! ARSPD_OPTIONS stub: vehicle-level bitmask for wind-max / EKF / offset report.

use ap_airspeed::options::{
    ARSPD_OPTIONS_DEFAULT, ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION, ARSPD_OPTION_REPORT_OFFSET,
};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_options_hookup::AirspeedOptionsHookup;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_options_match_upstream() {
    let hookup = AirspeedOptionsHookup::default();
    let published = hookup.publish();
    assert_eq!(published.options, ARSPD_OPTIONS_DEFAULT);
    assert!(published.disable_on_wind_max_failure);
    assert!(published.reenable_on_wind_max_recovery);
    assert!(!published.disable_voltage_correction);
    assert!(published.use_ekf_consistency);
    assert!(!published.report_offset);
}

#[test]
fn hookup_options_decode_report_offset_and_clear() {
    let mut hookup = AirspeedOptionsHookup::default();
    hookup.set_options(ARSPD_OPTIONS_DEFAULT | ARSPD_OPTION_REPORT_OFFSET);
    let reported = hookup.publish();
    assert!(reported.report_offset);
    assert!(reported.use_ekf_consistency);

    hookup.set_options(ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION);
    let voltage = hookup.publish();
    assert!(voltage.disable_voltage_correction);
    assert!(!voltage.disable_on_wind_max_failure);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_options() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert_eq!(vehicle.airspeed_options, ARSPD_OPTIONS_DEFAULT);

    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .set_options(ARSPD_OPTIONS_DEFAULT | ARSPD_OPTION_REPORT_OFFSET);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert_eq!(
        vehicle.airspeed_options,
        ARSPD_OPTIONS_DEFAULT | ARSPD_OPTION_REPORT_OFFSET
    );
}
