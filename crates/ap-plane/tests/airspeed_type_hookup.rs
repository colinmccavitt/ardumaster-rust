//! ARSPD_TYPE backend selection stub: type param picks SITL, analog, or none.

use ap_airspeed::backend::{
    AirspeedBackendKind, ARSPD_TYPE_ANALOG, ARSPD_TYPE_MS4525, ARSPD_TYPE_NONE, ARSPD_TYPE_SITL,
};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_type_hookup::AirspeedTypeHookup;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_type_selects_sitl() {
    let hookup = AirspeedTypeHookup::default();
    let published = hookup.publish();
    assert_eq!(published.sensor_type, ARSPD_TYPE_SITL);
    assert_eq!(published.configured, AirspeedBackendKind::Sitl);
    assert_eq!(published.active, AirspeedBackendKind::Sitl);
    assert!(published.enabled);
}

#[test]
fn hookup_type_selects_analog_and_none() {
    let mut hookup = AirspeedTypeHookup::default();
    hookup.set_sensor_type(ARSPD_TYPE_ANALOG);
    let analog = hookup.publish();
    assert_eq!(analog.configured, AirspeedBackendKind::Analog);
    assert_eq!(analog.active, AirspeedBackendKind::Analog);
    assert!(analog.enabled);

    hookup.set_sensor_type(ARSPD_TYPE_NONE);
    let none = hookup.publish();
    assert_eq!(none.active, AirspeedBackendKind::None);
    assert!(!none.enabled);
}

#[test]
fn hookup_unported_type_falls_back_to_none() {
    let mut hookup = AirspeedTypeHookup::default();
    hookup.set_sensor_type(ARSPD_TYPE_MS4525);
    let published = hookup.publish();
    assert_eq!(
        published.configured,
        AirspeedBackendKind::Other(ARSPD_TYPE_MS4525)
    );
    assert_eq!(published.active, AirspeedBackendKind::None);
    assert!(!published.enabled);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_type() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert_eq!(vehicle.airspeed_type, ARSPD_TYPE_SITL);
    assert_eq!(vehicle.active_airspeed_backend, AirspeedBackendKind::Sitl);
    assert!(vehicle.airspeed_use_for_control);
    assert!((vehicle.airspeed_tas - 20.0).abs() < 1e-6);

    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .set_sensor_type(ARSPD_TYPE_NONE);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_type, ARSPD_TYPE_NONE);
    assert_eq!(vehicle.active_airspeed_backend, AirspeedBackendKind::None);
    assert!(!vehicle.airspeed_use_for_control);
}
