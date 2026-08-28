//! ARSPD_USE gate: when disabled, TAS is not used for TECS/nav.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::{tas_for_nav, ARSPD_USE_DEFAULT};
use ap_baro::eas2tas_for_alt_amsl;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};

#[test]
fn hookup_default_use_airspeed_keeps_tas_for_control() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert_eq!(hookup.airspeed_params().primary_use_airspeed(), ARSPD_USE_DEFAULT);
    assert!(published.use_airspeed);
    assert!(published.healthy);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
}

#[test]
fn hookup_arspd_use_zero_still_samples_but_gates_control() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    let mut params = AirspeedParams::default();
    params.airspeed1.use_airspeed = 0;
    params.airspeed2.use_airspeed = 0;
    hookup.apply_airspeed_params(params);
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!(!published.use_airspeed);
    assert!(published.healthy);
    assert!(published.sample.have_sample);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
    assert_eq!(tas_for_nav(published.sample.tas_mps, published.use_airspeed), 0.0);
}

#[test]
fn main_loop_arspd_use_zero_does_not_feed_tecs_or_nav() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().set_use_airspeed(0);
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

    assert_eq!(vehicle.airspeed_use, 0);
    assert!(!vehicle.airspeed_use_for_control);
    assert!(vehicle.airspeed_healthy);
    assert!((vehicle.airspeed_tas - 15.0).abs() < 1e-6);
    assert!(vehicle.last_altitude_tecs_ran);
    assert!(!vehicle.last_tecs_use_airspeed);
}
