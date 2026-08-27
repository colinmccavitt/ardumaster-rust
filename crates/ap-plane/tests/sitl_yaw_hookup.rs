//! SITL yaw sample publish into the main loop.

use ap_ahrs::GPS_SPEED_MIN;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{cd_to_rad, radians};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish};

#[test]
fn sitl_publish_fills_compass_and_context_at_level_north() {
    let source = SitlYawPublish {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        compass_use_for_yaw: true,
        have_gps: false,
        now_ms: 1000,
        ..SitlYawPublish::default()
    };
    let attitude = Matrix3f::from_euler(0.0, 0.0, 0.0);
    let samples = publish_sitl_yaw_samples(&source, attitude, 1.0 / 400.0);

    let compass = samples.compass.expect("compass sample");
    assert!(compass.mag_body.length() > 0.1);
    assert_eq!(compass.update_interval_s, Some(1.0 / 400.0));
    assert!(!compass.calibrating);
    assert!(samples.gps_yaw.is_none());
    assert!(samples.yaw_ctx.compass_use_for_yaw);
    assert_eq!(samples.yaw_ctx.now_ms, 1000);
}

#[test]
fn sitl_publish_fills_gps_yaw_when_fix_available() {
    let source = SitlYawPublish {
        have_gps: true,
        ground_speed_mps: GPS_SPEED_MIN + 2.0,
        ground_course_deg: 270.0,
        last_fix_time_ms: 5000,
        compass_use_for_yaw: false,
        fly_forward: true,
        now_ms: 5000,
        ..SitlYawPublish::default()
    };
    let attitude = Matrix3f::from_euler(0.0, 0.0, cd_to_rad(9000.0));
    let samples = publish_sitl_yaw_samples(&source, attitude, 0.0025);

    assert!(samples.compass.is_none());
    let gps = samples.gps_yaw.expect("gps sample");
    assert_eq!(gps.ground_course_deg, 270.0);
    assert_eq!(gps.ground_speed, GPS_SPEED_MIN + 2.0);
    assert!(samples.yaw_ctx.have_gps);
    assert!(samples.yaw_ctx.fly_forward);
}

#[test]
fn ahrs_update_uses_sitl_yaw_publish_before_dcm() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_yaw = Some(SitlYawPublish {
        have_gps: true,
        ground_speed_mps: 10.0,
        ground_course_deg: 90.0,
        last_fix_time_ms: 100,
        compass_use_for_yaw: true,
        now_ms: 100,
        ..SitlYawPublish::default()
    });
    vehicle.ahrs.dcm.matrix = Matrix3f::from_euler(0.0, 0.0, radians(45.0));

    vehicle.ahrs_update();

    assert!(vehicle.compass.is_some());
    let gps = vehicle.gps_yaw.expect("gps yaw published");
    assert_eq!(gps.ground_course_deg, 90.0);
    assert_eq!(gps.ground_speed, 10.0);
    assert_eq!(gps.last_fix_time_ms, 100);
    assert!(vehicle.yaw_ctx.have_gps);
    assert_eq!(vehicle.ticks.ahrs_update, 1);
}
