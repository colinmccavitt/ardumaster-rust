//! Airspeed healthy-for-TECS publish gate: unhealthy TAS is not used for TECS.

use ap_airspeed::sitl::{tas_for_tecs, use_airspeed_for_tecs};
use ap_baro::eas2tas_for_alt_amsl;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_tecs_health_hookup::publish_airspeed_for_tecs;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};

#[test]
fn hookup_unhealthy_before_first_sample_gates_tecs() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 0,
    };
    let published = hookup.publish(1.0);
    assert!(published.use_airspeed);
    assert!(!published.healthy);
    assert!(!published.use_for_tecs);
    assert!(!use_airspeed_for_tecs(published.healthy, published.use_airspeed));
    assert_eq!(
        tas_for_tecs(published.sample.tas_mps, published.healthy, published.use_airspeed),
        0.0
    );
    let gated = publish_airspeed_for_tecs(
        published.sample.tas_mps,
        published.healthy,
        published.use_airspeed,
    );
    assert!(!gated.use_for_tecs);
    assert_eq!(gated.tas_for_tecs, 0.0);
}

#[test]
fn hookup_healthy_sample_publishes_tas_to_tecs() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!(published.use_airspeed);
    assert!(published.healthy);
    assert!(published.use_for_tecs);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
    let gated = publish_airspeed_for_tecs(
        published.sample.tas_mps,
        published.healthy,
        published.use_airspeed,
    );
    assert!(gated.use_for_tecs);
    assert!((gated.tas_for_tecs - 20.0).abs() < 1e-6);
}

#[test]
fn main_loop_unhealthy_airspeed_does_not_feed_tecs() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 0.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(15.0, 0.0, 0.0),
        now_ms: 0,
    };
    let _ = eas2tas_for_alt_amsl(0.0);

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.airspeed_use_for_control);
    assert!(!vehicle.airspeed_healthy);
    assert!(vehicle.last_altitude_tecs_ran);
    assert!(!vehicle.last_tecs_use_airspeed);
}

#[test]
fn main_loop_healthy_airspeed_feeds_tecs() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 0.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(15.0, 0.0, 0.0),
        now_ms: 10,
    };
    let _ = eas2tas_for_alt_amsl(0.0);

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.airspeed_use_for_control);
    assert!(vehicle.airspeed_healthy);
    assert!((vehicle.airspeed_tas - 15.0).abs() < 1e-6);
    assert!(vehicle.last_altitude_tecs_ran);
    assert!(vehicle.last_tecs_use_airspeed);
}
