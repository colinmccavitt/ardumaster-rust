//! ARSPD_AUTOCAL stub: GPS groundspeed vs TAS learns the pitot ratio.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::ARSPD_AUTOCAL_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_autocal_leaves_ratio_and_tas_unchanged() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    hookup.gps_groundspeed_mps = 25.0;
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert_eq!(hookup.airspeed_params().primary_autocal(), ARSPD_AUTOCAL_DEFAULT);
    assert_eq!(published.autocal, 0);
    assert!((published.ratio - 2.0).abs() < 1e-6);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
}

#[test]
fn hookup_autocal_scales_primary_and_secondary_ratio() {
    let mut hookup = SitlAirspeedHookup::with_dual_backends();
    let mut params = AirspeedParams::default();
    params.airspeed1.autocal = 1;
    params.airspeed2.autocal = 1;
    hookup.apply_airspeed_params(params);
    hookup.gps_groundspeed_mps = 25.0;
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert_eq!(published.autocal, 1);
    assert!((published.ratio - 2.5).abs() < 1e-6);
    assert!((published.sample.tas_mps - 25.0).abs() < 1e-6);
    assert!((published.sample.eas_mps - 25.0).abs() < 1e-6);
    assert!((hookup.cluster().backend(1).unwrap().config().ratio - 2.5).abs() < 1e-6);
}

#[test]
fn main_loop_ahrs_update_applies_autocal() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    let mut params = AirspeedParams::default();
    params.airspeed1.autocal = 1;
    params.airspeed2.autocal = 1;
    vehicle
        .sitl_airspeed
        .as_mut()
        .unwrap()
        .apply_airspeed_params(params);
    vehicle.sitl_airspeed.as_mut().unwrap().gps_groundspeed_mps = 25.0;
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_autocal, 1);
    assert!((vehicle.airspeed_ratio - 2.5).abs() < 1e-6);
    assert!((vehicle.airspeed_tas - 25.0).abs() < 1e-6);
    assert!(vehicle.airspeed_healthy);
    assert_eq!(vehicle.airspeed_health.instance_count, 2);
}
