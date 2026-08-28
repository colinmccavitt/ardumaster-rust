//! ARSPD_PRIMARY stub: primary airspeed instance select.

use ap_airspeed::primary::ARSPD_PRIMARY_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_primary_hookup::{
    airspeed_primary_tick, check_airspeed_primary, AirspeedPrimaryHookup,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_primary_is_first_instance() {
    let hookup = AirspeedPrimaryHookup::default();
    let published = hookup.publish(2);
    assert_eq!(published.configured, ARSPD_PRIMARY_DEFAULT);
    assert_eq!(published.clamped, 0);
}

#[test]
fn hookup_primary_selects_secondary_when_configured() {
    let mut hookup = AirspeedPrimaryHookup::default();
    hookup.set_primary(1);
    let selected = hookup.publish(2);
    assert_eq!(selected.configured, 1);
    assert_eq!(selected.clamped, 1);
    let gated = check_airspeed_primary(5, 2);
    assert_eq!(gated.clamped, 0);
}

#[test]
fn sitl_cluster_honors_arspd_primary_when_both_healthy() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    hookup.set_primary(1);
    let published = hookup.publish(1.0);
    assert_eq!(published.health.primary, 1);
    assert!(published.health.primary_healthy());
    let tick = airspeed_primary_tick(&mut hookup);
    assert_eq!(tick.configured, 1);
    assert_eq!(tick.clamped, 1);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_primary() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert_eq!(vehicle.airspeed_primary, ARSPD_PRIMARY_DEFAULT);

    vehicle.sitl_airspeed.as_mut().unwrap().set_primary(1);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_primary, 1);
    assert_eq!(vehicle.airspeed_health.primary, 1);
}
