//! ARSPD_WIND_MAX stub: max |airspeed - groundspeed| check.

use ap_airspeed::wind_max::ARSPD_WIND_MAX_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_wind_max_hookup::{check_airspeed_wind_max, AirspeedWindMaxHookup};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_wind_max_disables_check() {
    let hookup = AirspeedWindMaxHookup::default();
    let published = hookup.publish(40.0, 5.0);
    assert!((published.wind_max - ARSPD_WIND_MAX_DEFAULT).abs() < 1e-6);
    assert!(!published.enabled);
    assert!(!published.exceeded);
}

#[test]
fn hookup_wind_max_flags_airspeed_groundspeed_mismatch() {
    let mut hookup = AirspeedWindMaxHookup::default();
    hookup.set_wind_max(8.0);
    let fail = hookup.publish(20.0, 5.0);
    assert!(fail.enabled);
    assert!(fail.exceeded);
    let ok = hookup.publish(20.0, 14.0);
    assert!(!ok.exceeded);
    let gated = check_airspeed_wind_max(20.0, 5.0, 0.0);
    assert!(!gated.exceeded);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_wind_max() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    vehicle.sitl_airspeed.as_mut().unwrap().gps_groundspeed_mps = 5.0;

    vehicle.ahrs_update();
    assert!((vehicle.airspeed_wind_max - ARSPD_WIND_MAX_DEFAULT).abs() < 1e-6);
    assert!(!vehicle.airspeed_wind_max_exceeded);

    vehicle.sitl_airspeed.as_mut().unwrap().set_wind_max(8.0);
    vehicle.sitl_airspeed.as_mut().unwrap().gps_groundspeed_mps = 5.0;
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert!((vehicle.airspeed_wind_max - 8.0).abs() < 1e-6);
    assert!(vehicle.airspeed_wind_max_exceeded);
}
