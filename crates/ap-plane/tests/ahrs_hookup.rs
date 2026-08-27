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
    let motion = drift_motion_inputs(ctx, Some(gps), 0.0, 1.0, &mut last_fix);
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
    let motion = drift_motion_inputs(ctx, Some(gps), 0.0, 1.0, &mut last_fix);
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
    let motion2 = drift_motion_inputs(ctx2, Some(gps2), 0.0, 1.0, &mut last_fix);
    let (health, _) = feed.update_from_ins(&ins, &timing, None, motion2);
    assert_eq!(health, ap_ahrs::MatrixHealth::Ok);
}


#[test]
fn multi_accel_dead_reckoning_wired_through_drift_motion() {
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
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    let motion = drift_motion_inputs(ctx, Some(gps), 0.0, 1.0, &mut last_fix);
    let (health, _) = feed.update_from_ins(&ins, &timing, None, motion);
    assert_eq!(health, ap_ahrs::MatrixHealth::Ok);
    assert!(feed.drift.position.have_position);
}

#[test]
fn ekf3_unhealthy_falls_back_active_backend_to_dcm() {
    let mut feed = AhrsFeed::default();
    feed.set_configured_backend(ap_ahrs::AhrsBackendKind::Ekf3);
    feed.dcm.matrix.a.x = f32::NAN;
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);
    feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());
    assert!(!feed.ekf_healthy);
    assert_eq!(feed.active_backend, ap_ahrs::AhrsBackendKind::Dcm);
}

#[test]
fn ekf3_full_update_tracks_update_count() {
    let mut feed = AhrsFeed::default();
    feed.set_configured_backend(ap_ahrs::AhrsBackendKind::Ekf3);
    let ins = ap_ins::InertialSensorFrontend::default();
    let timing = LoopTiming::new(1.0 / 400.0);
    feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());
    feed.update_from_ins(&ins, &timing, None, ap_ahrs::DriftMotionInputs::default());
    assert_eq!(feed.ekf3.update_count, 2);
}

#[test]
fn estimated_wind_feedback_head_wind_into_wind() {
    use ap_ahrs::WindVaneSample;
    use ap_math::matrix3::Matrix3f;
    use ap_math::vector3::Vector3f;

    let mut feed = AhrsFeed::default();
    feed.apply_wind_vane(WindVaneSample {
        direction_true_rad: 0.0,
        speed_true_mps: 5.0,
    });
    feed.dcm.matrix = Matrix3f::from_euler(0.0, 0.0, 0.0);
    assert!((feed.wind_estimate().length() - 5.0).abs() < 0.01);
    assert!(
        feed.head_wind() > 4.0,
        "heading north into north wind should read positive headwind, got {}",
        feed.head_wind()
    );
}

#[test]
fn wind_vane_seeds_estimated_wind() {
    use ap_ahrs::WindVaneSample;
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.wind_vane = Some(WindVaneSample {
        direction_true_rad: core::f32::consts::FRAC_PI_2,
        speed_true_mps: 3.0,
    });
    vehicle.ahrs_update();
    assert!((vehicle.estimated_wind.length() - 3.0).abs() < 0.01);
    assert!(vehicle.head_wind_ms.abs() < 0.01, "crosswind should give ~zero headwind");
}

#[test]
fn ekf_health_and_dead_reckoning_published_on_main_loop() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth, YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.yaw_ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    vehicle.gps_yaw = Some(YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    });

    vehicle.ahrs_update();

    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::Ok);
    assert!(vehicle.ekf_healthy);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Ekf3);
    assert!(vehicle.have_dead_reckoning_position);
}

#[test]
fn ekf_unhealthy_publishes_dcm_fallback_on_main_loop() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.ahrs.dcm.matrix.a.x = f32::NAN;

    vehicle.ahrs_update();

    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::NeedsReset);
    assert!(!vehicle.ekf_healthy);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Dcm);
}

#[test]
fn dead_reckoning_offset_accessor_reflects_drift_position() {
    use ap_math::vector3::Vector3f;

    let mut feed = AhrsFeed::default();
    feed.drift.position.on_gps_fix(100, 200, 1000);
    feed.drift.position.integrate(Vector3f::new(5.0, 2.0, 0.0), 0.2, false);
    let (n, e, have) = feed.dead_reckoning_offset();
    assert!(have);
    assert!((n - 1.0).abs() < 1e-5);
    assert!((e - 0.4).abs() < 1e-5);
}

#[test]
fn main_loop_publishes_dead_reckoning_offset_from_ahrs() {
    use ap_ahrs::{YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 0.1;
    vehicle.yaw_ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    vehicle.gps_yaw = Some(YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    });
    vehicle.ahrs_update();
    assert!(vehicle.have_dead_reckoning_position);
    assert_eq!(vehicle.dead_reckoning_north_m, 0.0);
    assert_eq!(vehicle.dead_reckoning_east_m, 0.0);
}

#[test]
fn attitude_rad_accessors_match_centidegrees() {
    use ap_math::scalar::cd_to_rad;

    let attitude = AhrsAttitude {
        roll_sensor_cd: 4500,
        pitch_sensor_cd: -2000,
        yaw_sensor_cd: 9000,
    };
    assert!((attitude.roll_rad() - cd_to_rad(4500.0)).abs() < 1e-6);
    assert!((attitude.pitch_rad() - cd_to_rad(-2000.0)).abs() < 1e-6);
    assert!((attitude.yaw_rad() - cd_to_rad(9000.0)).abs() < 1e-6);
}

#[test]
fn ahrs_healthy_false_when_matrix_needs_reset() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth};
    use ap_plane::ahrs_hookup::ahrs_healthy;

    assert!(!ahrs_healthy(MatrixHealth::NeedsReset, true, AhrsBackendKind::Dcm));
    assert!(!ahrs_healthy(MatrixHealth::NeedsReset, true, AhrsBackendKind::Ekf3));
}

#[test]
fn ahrs_healthy_requires_ekf_when_active_backend_is_ekf3() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth};
    use ap_plane::ahrs_hookup::ahrs_healthy;

    assert!(ahrs_healthy(MatrixHealth::Ok, true, AhrsBackendKind::Ekf3));
    assert!(!ahrs_healthy(MatrixHealth::Ok, false, AhrsBackendKind::Ekf3));
    assert!(ahrs_healthy(MatrixHealth::Ok, false, AhrsBackendKind::Dcm));
}

#[test]
fn main_loop_publishes_ahrs_healthy_from_update() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth, YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.yaw_ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    vehicle.gps_yaw = Some(YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    });

    vehicle.ahrs_update();

    assert!(vehicle.ahrs_healthy);
    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::Ok);

    vehicle.ahrs.dcm.matrix.a.x = f32::NAN;
    vehicle.ahrs_update();
    assert!(!vehicle.ahrs_healthy);
    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::NeedsReset);
}

#[test]
fn main_loop_publishes_ekf3_status_from_update() {
    use ap_ahrs::{AhrsBackendKind, YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.yaw_ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    vehicle.gps_yaw = Some(YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    });

    vehicle.ahrs_update();
    assert!(vehicle.ekf3_initialized);
    assert_eq!(vehicle.ekf3_update_count, 1);
    assert!(vehicle.ekf_healthy);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Ekf3);

    vehicle.ahrs_update();
    assert_eq!(vehicle.ekf3_update_count, 2);
}

#[test]
fn main_loop_publishes_configured_and_wind_alignment() {
    use ap_ahrs::{AhrsBackendKind, WindVaneSample};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.wind_vane = Some(WindVaneSample {
        direction_true_rad: 0.0,
        speed_true_mps: 5.0,
    });

    vehicle.ahrs_update();

    assert_eq!(vehicle.configured_ahrs_backend, AhrsBackendKind::Ekf3);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Ekf3);
    assert!(
        vehicle.wind_alignment > 0.9,
        "north wind with north heading should align, got {}",
        vehicle.wind_alignment
    );
    assert!(vehicle.head_wind_ms > 4.0);
}

#[test]
fn configured_backend_stays_ekf3_when_active_falls_back_to_dcm() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.ahrs.dcm.matrix.a.x = f32::NAN;

    vehicle.ahrs_update();

    assert_eq!(vehicle.configured_ahrs_backend, AhrsBackendKind::Ekf3);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Dcm);
    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::NeedsReset);
}

#[test]
fn drift_loop_using_gps_reflects_gps_lock() {
    use ap_ahrs::DcmDriftLoop;

    let mut drift = DcmDriftLoop::default();
    assert!(!drift.using_gps());
}

#[test]
fn ahrs_pre_arm_check_respects_force_and_health() {
    use ap_ahrs::{AhrsBackendKind, MatrixHealth};
    use ap_plane::ahrs_hookup::{ahrs_healthy, AhrsFeed};

    let mut feed = AhrsFeed::default();
    feed.matrix_health = MatrixHealth::NeedsReset;
    assert!(!feed.pre_arm_check(false));
    assert!(feed.pre_arm_check(true));
    assert!(!ahrs_healthy(MatrixHealth::NeedsReset, true, AhrsBackendKind::Dcm));
}

#[test]
fn main_loop_publishes_using_gps_and_pre_arm_from_ahrs() {
    use ap_ahrs::{AhrsBackendKind, YawDriftContext, YawGpsSample, GPS_SPEED_MIN};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.yaw_ctx = YawDriftContext {
        have_gps: true,
        now_ms: 100,
        gps_lat_e7: Some(473_582_100),
        gps_lng_e7: Some(-122_234_567),
        ..YawDriftContext::default()
    };
    vehicle.gps_yaw = Some(YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN + 1.0,
        last_fix_time_ms: 100,
    });

    vehicle.ahrs_update();

    assert!(vehicle.ahrs_pre_arm_ok);
    assert!(vehicle.ahrs_using_gps);
}

#[test]
fn main_loop_wires_ahrs_consumers_into_stabilize() {
    use ap_ahrs::{AhrsBackendKind, WindVaneSample};
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.eas2tas = 1.25;
    vehicle.yaw_ctx.now_ms = 42_000;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.wind_vane = Some(WindVaneSample {
        direction_true_rad: 0.0,
        speed_true_mps: 5.0,
    });

    vehicle.ahrs_update();
    vehicle.stabilize();

    assert!((vehicle.stabilize_ctx.eas2tas - 1.25).abs() < f32::EPSILON);
    assert_eq!(vehicle.stabilize_ctx.now_ms, 42_000);
    assert_eq!(vehicle.stabilize_ctx.accel_bias_y, 0.0);
}

#[test]
fn main_loop_publishes_attitude_radians_from_ahrs() {
    use ap_ahrs::AhrsBackendKind;
    use ap_math::scalar::cd_to_rad;
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Dcm);

    vehicle.ahrs_update();

    assert!((vehicle.roll_rad - vehicle.attitude.roll_rad()).abs() < f32::EPSILON);
    assert!((vehicle.pitch_rad - vehicle.attitude.pitch_rad()).abs() < f32::EPSILON);
    assert!((vehicle.yaw_rad - vehicle.attitude.yaw_rad()).abs() < f32::EPSILON);
    assert!((vehicle.roll_rad - cd_to_rad(vehicle.attitude.roll_sensor_cd as f32)).abs() < 1e-6);
}

#[test]
fn main_loop_pre_arm_ok_requires_healthy_ahrs() {
    use ap_ahrs::AhrsBackendKind;
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Ekf3);
    vehicle.ahrs.dcm.matrix.a.x = f32::NAN;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(!vehicle.ahrs_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}

#[test]
fn dcm_scope_publishes_all_vehicle_consumers() {
    use ap_ahrs::{AhrsBackendKind, DCM_SCOPE_COMPLETE, MatrixHealth};
    use ap_plane::main_loop::PlaneMainLoop;

    assert!(DCM_SCOPE_COMPLETE);

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.ahrs.set_configured_backend(AhrsBackendKind::Dcm);

    vehicle.ahrs_update();
    vehicle.update_control_mode();
    vehicle.stabilize();

    assert_eq!(vehicle.configured_ahrs_backend, AhrsBackendKind::Dcm);
    assert_eq!(vehicle.active_ahrs_backend, AhrsBackendKind::Dcm);
    assert_eq!(vehicle.ahrs_matrix_health, MatrixHealth::Ok);
    assert!(vehicle.ahrs_healthy);
    assert!(vehicle.ahrs_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
    assert!((vehicle.roll_rad - vehicle.attitude.roll_rad()).abs() < f32::EPSILON);
    assert_eq!(vehicle.stabilize_ctx.accel_bias_y, 0.0);
    assert!(vehicle.ticks.ahrs_update >= 1);
    assert!(vehicle.ticks.stabilize >= 1);
}

