//! ARSPD_DEVID stub: probe-assigned sensor ID (type + bus + instance).

use ap_airspeed::backend::{ARSPD_TYPE_MS4525, ARSPD_TYPE_SITL};
use ap_airspeed::bus::ARSPD_BUS_EXTERNAL2;
use ap_airspeed::devid::{
    devid_bus, devid_bus_type, ARSPD_DEVID_DEFAULT, BUS_TYPE_I2C, BUS_TYPE_SITL,
};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_devid_hookup::AirspeedDevidHookup;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_devid_is_unset() {
    let hookup = AirspeedDevidHookup::default();
    let published = hookup.publish();
    assert_eq!(published.devid, ARSPD_DEVID_DEFAULT);
    assert!(!published.is_set);
}

#[test]
fn hookup_probe_assigns_and_clears_devid() {
    let mut hookup = AirspeedDevidHookup::default();
    hookup.set_sensor_type(ARSPD_TYPE_SITL);
    hookup.assign_from_probe(true);
    let sitl = hookup.publish();
    assert!(sitl.is_set);
    assert_eq!(devid_bus_type(sitl.devid as u32), BUS_TYPE_SITL);

    hookup.set_sensor_type(ARSPD_TYPE_MS4525);
    hookup.set_bus(ARSPD_BUS_EXTERNAL2);
    hookup.assign_from_probe(true);
    let digital = hookup.publish();
    assert!(digital.is_set);
    assert_eq!(devid_bus_type(digital.devid as u32), BUS_TYPE_I2C);
    assert_eq!(devid_bus(digital.devid as u32), ARSPD_BUS_EXTERNAL2);

    hookup.assign_from_probe(false);
    let missing = hookup.publish();
    assert_eq!(missing.devid, ARSPD_DEVID_DEFAULT);
    assert!(!missing.is_set);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_devid() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert_eq!(vehicle.airspeed_devid, ARSPD_DEVID_DEFAULT);

    let assigned = {
        let airspeed = vehicle.sitl_airspeed.as_mut().unwrap();
        airspeed.set_sensor_type(ARSPD_TYPE_SITL);
        airspeed.assign_devid_from_probe(true);
        airspeed.airspeed_params().primary_devid()
    };
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert!(assigned != ARSPD_DEVID_DEFAULT);
    assert_eq!(vehicle.airspeed_devid, assigned);
    assert_eq!(devid_bus_type(vehicle.airspeed_devid as u32), BUS_TYPE_SITL);
}
