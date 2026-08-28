//! ARSPD_BUS stub: I2C bus number for digital pitot backends.

use ap_airspeed::backend::{ARSPD_TYPE_MS4525, ARSPD_TYPE_SITL};
use ap_airspeed::bus::{ARSPD_BUS_DEFAULT, ARSPD_BUS_EXTERNAL2, ARSPD_BUS_INTERNAL};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_bus_hookup::AirspeedBusHookup;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_bus_is_external() {
    let hookup = AirspeedBusHookup::default();
    let published = hookup.publish();
    assert_eq!(published.bus, ARSPD_BUS_DEFAULT);
    assert_eq!(published.probe_bus, 1);
    assert!(!published.uses_i2c);
}

#[test]
fn hookup_bus_selects_internal_and_i2c_type() {
    let mut hookup = AirspeedBusHookup::default();
    hookup.set_bus(ARSPD_BUS_INTERNAL);
    let internal = hookup.publish();
    assert_eq!(internal.bus, ARSPD_BUS_INTERNAL);
    assert_eq!(internal.probe_bus, 0);
    assert!(!internal.uses_i2c);

    hookup.set_sensor_type(ARSPD_TYPE_MS4525);
    hookup.set_bus(ARSPD_BUS_EXTERNAL2);
    let digital = hookup.publish();
    assert_eq!(digital.bus, 2);
    assert_eq!(digital.probe_bus, 2);
    assert!(digital.uses_i2c);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_bus() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert_eq!(vehicle.airspeed_bus, ARSPD_BUS_DEFAULT);
    assert_eq!(vehicle.airspeed_type, ARSPD_TYPE_SITL);

    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .set_bus(ARSPD_BUS_INTERNAL);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_bus, ARSPD_BUS_INTERNAL);
}
