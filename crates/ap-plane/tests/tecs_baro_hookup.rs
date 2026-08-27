use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};

#[test]
fn main_loop_publishes_tecs_baro_feed_from_baro_cluster() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 200.0,
        now_ms: 1000,
        ..SitlBaroTruth::default()
    };
    vehicle.ahrs_update();
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 250.0,
        now_ms: 2000,
        ..SitlBaroTruth::default()
    };
    vehicle.ahrs_update();
    assert!(vehicle.tecs_baro_feed.height_m > 40.0);
    assert!(vehicle.tecs_baro_feed.hgt_afe_m > 40.0);
}
