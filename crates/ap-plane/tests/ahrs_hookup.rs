//! AHRS attitude feed from DCM into the main loop.

use ap_ahrs::Dcm;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::cd_to_rad;
use ap_plane::ahrs_hookup::{attitude_from_dcm, AhrsAttitude, AhrsFeed};
use ap_plane::main_loop::PlaneMainLoop;

#[test]
fn attitude_from_dcm_matches_euler() {
    let roll = cd_to_rad(4500.0_f32);
    let pitch = cd_to_rad(-2000.0_f32);
    let yaw = cd_to_rad(9000.0_f32);
    let mut dcm = Dcm::new();
    dcm.matrix = Matrix3f::from_euler(roll, pitch, yaw);

    let attitude = attitude_from_dcm(&dcm);
    assert_eq!(attitude.roll_sensor_cd, 4500);
    assert_eq!(attitude.pitch_sensor_cd, -2000);
    assert_eq!(attitude.yaw_sensor_cd, 9000);
}

#[test]
fn ahrs_update_publishes_attitude_on_main_loop() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let roll = cd_to_rad(3000.0_f32);
    let pitch = cd_to_rad(1000.0_f32);
    vehicle.ahrs.dcm.matrix = Matrix3f::from_euler(roll, pitch, 0.0);

    vehicle.ahrs_update();

    assert_eq!(vehicle.ticks.ahrs_update, 1);
    assert_eq!(
        vehicle.attitude,
        AhrsAttitude {
            roll_sensor_cd: 3000,
            pitch_sensor_cd: 1000,
            yaw_sensor_cd: 0,
        }
    );
}

#[test]
fn ahrs_feed_update_from_ins_with_no_samples_keeps_attitude() {
    let mut feed = AhrsFeed::default();
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);

    let (health, attitude) = feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());

    assert_eq!(health, ap_ahrs::MatrixHealth::Ok);
    assert_eq!(attitude, AhrsAttitude::default());
}

#[test]
fn yaw_update_inputs_includes_gps_when_context_set() {
    use ap_ahrs::{YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::ahrs_hookup::yaw_update_inputs;

    let gps = YawGpsSample {
        ground_course_deg: 90.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 1000,
    };
    let ctx = YawDriftContext {
        fly_forward: true,
        have_gps: true,
        compass_use_for_yaw: false,
        ..YawDriftContext::default()
    };
    let inputs = yaw_update_inputs(None, Some(gps), ctx).expect("gps yaw inputs");
    assert!(inputs.compass.is_none());
    let got = inputs.gps.expect("gps sample");
    assert_eq!(got.ground_course_deg, gps.ground_course_deg);
    assert!(inputs.ctx.have_gps);
}

#[test]
fn drift_motion_inputs_builds_gps_velocity() {
    use ap_ahrs::{YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::ahrs_hookup::drift_motion_inputs;

    let mut last_fix = 0;
    let gps = YawGpsSample {
        ground_course_deg: 90.0,
        ground_speed: GPS_SPEED_MIN + 2.0,
        last_fix_time_ms: 500,
    };
    let ctx = YawDriftContext {
        have_gps: true,
        now_ms: 500,
        ..YawDriftContext::default()
    };
    let motion = drift_motion_inputs(ctx, Some(gps), 0.0, &mut last_fix);
    assert!(motion.new_gps_fix);
    let vel = motion.gps_velocity.expect("gps velocity");
    assert!(vel.x.abs() < 0.01, "east course => near-zero north");
    assert!(vel.y > GPS_SPEED_MIN);
}

#[test]
fn backend_selection_defaults_to_dcm() {
    let feed = AhrsFeed::default();
    assert_eq!(feed.configured_backend, ap_ahrs::AhrsBackendKind::Dcm);
    assert_eq!(feed.active_backend, ap_ahrs::AhrsBackendKind::Dcm);
}

#[test]
fn configured_ekf3_starts_unhealthy_until_first_update() {
    let mut feed = AhrsFeed::default();
    feed.set_configured_backend(ap_ahrs::AhrsBackendKind::Ekf3);
    assert_eq!(feed.configured_backend, ap_ahrs::AhrsBackendKind::Ekf3);
    assert_eq!(feed.active_backend, ap_ahrs::AhrsBackendKind::Ekf3);
    assert!(!feed.ekf_healthy);
}

#[test]
fn ahrs_update_refreshes_active_backend_after_ekf3() {
    let mut feed = AhrsFeed::default();
    feed.set_configured_backend(ap_ahrs::AhrsBackendKind::Ekf3);
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);
    feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());
    assert_eq!(feed.active_backend, ap_ahrs::AhrsBackendKind::Ekf3);
    assert!(feed.ekf_healthy);
}

#[test]
fn ekf3_path_dispatches_through_update_hook() {
    let mut feed = AhrsFeed::default();
    feed.set_configured_backend(ap_ahrs::AhrsBackendKind::Ekf3);
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);
    feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());
    assert!(feed.ekf3.initialized);
    assert!(feed.ekf_healthy);
    assert_eq!(feed.active_backend, ap_ahrs::AhrsBackendKind::Ekf3);
}



#[test]
fn gps_lag_buffer_wired_through_drift_motion_with_gps() {
    use ap_ahrs::{YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::ahrs_hookup::drift_motion_inputs;

    let mut feed = AhrsFeed::default();
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);
    let mut last_fix = 0;
    let gps = YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    };
    let ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        ..YawDriftContext::default()
    };
    let motion = drift_motion_inputs(ctx, Some(gps), 0.0, &mut last_fix);
    assert!(motion.have_gps);
    assert!(motion.new_gps_fix);
    feed.update_from_ins(&ins, &timing, None, motion);
    let gps2 = YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 600,
    };
    let ctx2 = YawDriftContext {
        have_gps: true,
        now_ms: 600,
        ..YawDriftContext::default()
    };
    let motion2 = drift_motion_inputs(ctx2, Some(gps2), 0.0, &mut last_fix);
    let (health, _) = feed.update_from_ins(&ins, &timing, None, motion2);
    assert_eq!(health, ap_ahrs::MatrixHealth::Ok);
}
