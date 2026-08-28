//! Airspeed temperature compensation stub: TAS scales with sensor temperature.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::{sitl_airspeed_temperature_c, ARSPD_TEMP_REF_C};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_temp_comp_leaves_pitot_tas_unchanged() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!((hookup.airspeed_params().primary_temperature_c() - ARSPD_TEMP_REF_C).abs() < 1e-6);
    assert!((published.temperature_c - 15.0).abs() < 1e-6);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
    assert!((sitl_airspeed_temperature_c(0.0) - 15.0).abs() < 1e-6);
}

#[test]
fn hookup_temp_coeff_scales_primary_and_secondary_tas() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    let mut params = AirspeedParams::default();
    params.airspeed1.temperature_c = 25.0;
    params.airspeed1.temp_coeff = 0.01;
    params.airspeed2.temperature_c = 25.0;
    params.airspeed2.temp_coeff = 0.01;
    hookup.apply_airspeed_params(params);
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!((published.temperature_c - 25.0).abs() < 1e-6);
    assert!((published.sample.tas_mps - 22.0).abs() < 1e-6);
    assert!((published.sample.eas_mps - 22.0).abs() < 1e-6);
    assert!((hookup.cluster().backend(1).unwrap().state().tas_mps - 22.0).abs() < 1e-6);
}

#[test]
fn main_loop_ahrs_update_applies_temp_compensation() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    let mut params = AirspeedParams::default();
    params.airspeed1.temperature_c = 5.0;
    params.airspeed1.temp_coeff = 0.01;
    params.airspeed2.temperature_c = 5.0;
    params.airspeed2.temp_coeff = 0.01;
    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .apply_airspeed_params(params);
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert!((vehicle.airspeed_temperature_c - 5.0).abs() < 1e-6);
    // 20 * (1 + 0.01 * (5 - 15)) = 18
    assert!((vehicle.airspeed_tas - 18.0).abs() < 1e-6);
    assert!(vehicle.airspeed_healthy);
    assert_eq!(vehicle.airspeed_health.instance_count, 2);
}
