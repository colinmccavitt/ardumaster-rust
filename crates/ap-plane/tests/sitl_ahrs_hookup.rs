//! SITL AHRS publish extension: airspeed TAS and EAS2TAS into drift motion.

use ap_ahrs::GPS_SPEED_MIN;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::radians;
use ap_plane::ahrs_hookup::drift_motion_inputs;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_ahrs_hookup::{publish_sitl_ahrs_samples, SitlAhrsPublish};
use ap_plane::sitl_yaw_hookup::SitlYawPublish;

#[test]
fn sitl_ahrs_publish_passes_through_airspeed_tas_and_eas2tas() {
    let source = SitlAhrsPublish {
        yaw: SitlYawPublish {
            have_gps: true,
            ground_speed_mps: GPS_SPEED_MIN + 1.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 100,
            now_ms: 100,
            ..SitlYawPublish::default()
        },
        airspeed_tas_mps: 22.5,
        eas2tas: 1.15,
    };
    let attitude = Matrix3f::from_euler(0.0, 0.0, 0.0);
    let samples = publish_sitl_ahrs_samples(&source, attitude, 0.0025);
    assert!((samples.airspeed_tas - 22.5).abs() < 1e-6);
    assert!((samples.eas2tas - 1.15).abs() < 1e-6);
    assert!(samples.yaw.gps_yaw.is_some());
}

#[test]
fn ahrs_update_wires_sitl_ahrs_airspeed_and_eas2tas_into_drift_motion() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_ahrs = Some(SitlAhrsPublish {
        yaw: SitlYawPublish {
            have_gps: true,
            ground_speed_mps: 12.0,
            ground_course_deg: 90.0,
            last_fix_time_ms: 200,
            fly_forward: true,
            now_ms: 200,
            ..SitlYawPublish::default()
        },
        airspeed_tas_mps: 18.0,
        eas2tas: 1.2,
    });
    vehicle.ahrs.dcm.matrix = Matrix3f::from_euler(0.0, 0.0, radians(30.0));

    vehicle.ahrs_update();

    assert!((vehicle.airspeed_tas - 18.0).abs() < 1e-6);
    assert!((vehicle.eas2tas - 1.2).abs() < 1e-6);
    assert!(vehicle.gps_yaw.is_some());
    assert_eq!(vehicle.ticks.ahrs_update, 1);

    let mut last_fix = 0_u32;
    let motion = drift_motion_inputs(
        vehicle.yaw_ctx,
        vehicle.gps_yaw,
        vehicle.gps_velocity,
        vehicle.airspeed_tas,
        vehicle.eas2tas,
        &mut last_fix,
    );
    assert!((motion.airspeed_tas - 18.0).abs() < 1e-6);
    assert!((motion.eas2tas - 1.2).abs() < 1e-6);
    assert!(motion.have_gps);
}

#[test]
fn sitl_ahrs_gps_lat_lng_flows_to_dead_reckoning() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_ahrs = Some(SitlAhrsPublish {
        yaw: SitlYawPublish {
            latitude_deg: 47.35821,
            longitude_deg: -122.234567,
            have_gps: true,
            ground_speed_mps: GPS_SPEED_MIN + 2.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 100,
            now_ms: 100,
            ..SitlYawPublish::default()
        },
        airspeed_tas_mps: 15.0,
        eas2tas: 1.0,
    });

    vehicle.ahrs_update();

    let expected_lat = (47.35821_f32 * 1e7_f32) as i32;
    let expected_lng = (-122.234567_f32 * 1e7_f32) as i32;
    assert_eq!(vehicle.yaw_ctx.gps_lat_e7, Some(expected_lat));
    assert_eq!(vehicle.yaw_ctx.gps_lng_e7, Some(expected_lng));
    assert!(vehicle.have_dead_reckoning_position);
}

