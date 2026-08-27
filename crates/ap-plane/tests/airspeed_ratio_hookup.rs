//! Pitot tube ratio stub: ARSPD_RATIO scales TAS on the SITL cluster.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::ARSPD_RATIO_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_ratio_leaves_pitot_tas_unchanged() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!((hookup.airspeed_params().primary_ratio() - ARSPD_RATIO_DEFAULT).abs() < 1e-6);
    assert!((published.ratio - 2.0).abs() < 1e-6);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
}

#[test]
fn hookup_arspd_ratio_half_scales_primary_and_secondary_tas() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    let mut params = AirspeedParams::default();
    params.airspeed1.ratio = 1.0;
    params.airspeed2.ratio = 1.0;
    hookup.apply_airspeed_params(params);
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!((published.ratio - 1.0).abs() < 1e-6);
    assert!((published.sample.tas_mps - 10.0).abs() < 1e-6);
    assert!((published.sample.eas_mps - 10.0).abs() < 1e-6);
    assert!((hookup.cluster().backend(1).unwrap().state().tas_mps - 10.0).abs() < 1e-6);
}

#[test]
fn main_loop_ahrs_update_applies_arspd_ratio() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    let mut params = AirspeedParams::default();
    params.airspeed1.ratio = 4.0;
    params.airspeed2.ratio = 4.0;
    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .apply_airspeed_params(params);
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(15.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert!((vehicle.airspeed_ratio - 4.0).abs() < 1e-6);
    assert!((vehicle.airspeed_tas - 30.0).abs() < 1e-6);
    assert!(vehicle.airspeed_healthy);
    assert_eq!(vehicle.airspeed_health.instance_count, 2);
}
