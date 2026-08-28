//! ARSPD_PSI_RANGE stub: clamp / validate sensor pressure range.

use ap_airspeed::psi_range::ARSPD_PSI_RANGE_DEFAULT;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_psi_range_hookup::{validate_airspeed_psi_range, AirspeedPsiRangeHookup};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_psi_range_matches_upstream() {
    let hookup = AirspeedPsiRangeHookup::default();
    let published = hookup.publish();
    assert!((published.psi_range - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
    assert!(published.valid);
    assert!((published.configured - 1.0).abs() < 1e-6);
}

#[test]
fn hookup_psi_range_clamps_invalid_sensor_range() {
    let mut hookup = AirspeedPsiRangeHookup::default();
    hookup.set_psi_range(2.0);
    let ok = hookup.publish();
    assert!(ok.valid);
    assert!((ok.psi_range - 2.0).abs() < 1e-6);
    hookup.set_psi_range(-1.0);
    let bad = hookup.publish();
    assert!(!bad.valid);
    assert!((bad.psi_range - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
    let gated = validate_airspeed_psi_range(0.0);
    assert!(!gated.valid);
    assert!((gated.psi_range - 1.0).abs() < 1e-6);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_psi_range() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert!((vehicle.airspeed_psi_range - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);

    vehicle.sitl_airspeed.as_mut().unwrap().set_psi_range(2.5);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();
    assert!((vehicle.airspeed_psi_range - 2.5).abs() < 1e-6);

    vehicle.sitl_airspeed.as_mut().unwrap().set_psi_range(0.0);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 30;
    vehicle.ahrs_update();
    assert!((vehicle.airspeed_psi_range - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
}
