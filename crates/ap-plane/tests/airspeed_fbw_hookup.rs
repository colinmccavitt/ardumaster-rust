//! ARSPD_FBW_MIN / ARSPD_FBW_MAX stub: fly-by-wire airspeed limits.

use ap_airspeed::fbw::{ARSPD_FBW_MAX_DEFAULT, ARSPD_FBW_MIN_DEFAULT};
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::airspeed_fbw_hookup::{limit_airspeed_fbw, AirspeedFbwHookup};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

#[test]
fn hookup_default_fbw_limits_match_upstream() {
    let hookup = AirspeedFbwHookup::default();
    let published = hookup.publish(15.0);
    assert!((published.fbw_min - ARSPD_FBW_MIN_DEFAULT).abs() < 1e-6);
    assert!((published.fbw_max - ARSPD_FBW_MAX_DEFAULT).abs() < 1e-6);
    assert!((published.limited_mps - 15.0).abs() < 1e-6);
    assert!((hookup.publish(4.0).limited_mps - 9.0).abs() < 1e-6);
    assert!((hookup.publish(40.0).limited_mps - 22.0).abs() < 1e-6);
}

#[test]
fn hookup_fbw_limits_clamp_demanded_airspeed() {
    let mut hookup = AirspeedFbwHookup::default();
    hookup.set_fbw_min(12.0);
    hookup.set_fbw_max(20.0);
    let low = hookup.publish(8.0);
    assert!((low.limited_mps - 12.0).abs() < 1e-6);
    let high = hookup.publish(28.0);
    assert!((high.limited_mps - 20.0).abs() < 1e-6);
    let inverted = limit_airspeed_fbw(5.0, 22.0, 9.0);
    assert!((inverted.limited_mps - 9.0).abs() < 1e-6);
}

#[test]
fn main_loop_ahrs_update_honors_arspd_fbw_limits() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    assert!((vehicle.airspeed_fbw_min - ARSPD_FBW_MIN_DEFAULT).abs() < 1e-6);
    assert!((vehicle.airspeed_fbw_max - ARSPD_FBW_MAX_DEFAULT).abs() < 1e-6);

    vehicle.sitl_airspeed.as_mut().unwrap().set_fbw_min(11.0);
    vehicle.sitl_airspeed.as_mut().unwrap().set_fbw_max(19.0);
    vehicle.sitl_airspeed.as_mut().unwrap().truth.now_ms = 20;
    vehicle.ahrs_update();

    assert!((vehicle.airspeed_fbw_min - 11.0).abs() < 1e-6);
    assert!((vehicle.airspeed_fbw_max - 19.0).abs() < 1e-6);
}
