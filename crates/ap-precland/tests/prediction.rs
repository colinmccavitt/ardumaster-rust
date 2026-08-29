//! `AC_PrecLand` output-prediction leftover: `run_output_prediction`
//! and the getters that read `_target_*_out_*`.
//!
//! Tracked as **COP-028**. Backends, `Write_Precland`, the inertial ring,
//! and `AC_PrecLand_StateMachine` stay later.

use ap_math::location::Location;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{is_equal, radians};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_precland::{
    EstimatorInput, EstimatorType, EstimatorWorld, InertialSample, LosSample,
    OutputPredictionWorld, PrecLand, PrecLandParams, TargetState, Type, VectorFrame,
    LANDING_TARGET_LOST_DIST_THRESH_M, LANDING_TARGET_LOST_TIMEOUT_MS, LANDING_TARGET_TIMEOUT_MS,
    OPTION_MOVING_TARGET, REMAINING, SENSOR_MAX_ALT_M_DEFAULT, SENSOR_MIN_ALT_M_DEFAULT,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec2(got: Vector2f, want: Vector2f) {
    assert!(
        is_equal(got.x, want.x) && is_equal(got.y, want.y),
        "({} {}) != ({} {})",
        got.x,
        got.y,
        want.x,
        want.y
    );
}

fn mavlink_inited(estimator_type: EstimatorType) -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        estimator_type,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    plnd
}

fn down_los(time_ms: u32) -> LosSample {
    LosSample {
        time_ms,
        vec_unit: Vector3f::new(0.0, 0.0, 1.0),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 0.0,
    }
}

fn meas_input(now_ms: u32, los: Option<LosSample>) -> EstimatorInput {
    EstimatorInput {
        rangefinder_alt_m: 2.0,
        rangefinder_alt_valid: true,
        now_ms,
        delayed: InertialSample::default(),
        any_inertial_nav_invalid: false,
        los,
        world: EstimatorWorld::default(),
    }
}

fn acquire_raw(plnd: &mut PrecLand, now_ms: u32) {
    let mut input = meas_input(now_ms, Some(down_los(now_ms)));
    input.delayed.inertial_nav_velocity = Vector3f::new(1.0, 2.0, 0.0);
    let leftover = plnd.run_estimator(input);
    assert!(leftover.constructed_pos_meas);
    assert!(plnd.estimator_target_acquired());
}

#[test]
fn identity_prediction_copies_estimate() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    almost_vec2(plnd.target_pos_rel_est_ne_m(), Vector2f::new(0.0, 0.0));
    almost_vec2(plnd.target_vel_rel_est_ne_ms(), Vector2f::new(-1.0, -2.0));

    let leftover = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            ..OutputPredictionWorld::default()
        },
    );
    assert!(!leftover.stored_last_target_pos);
    assert!(!leftover.stored_vehicle_velocity);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(0.0, 0.0));
    almost_vec2(plnd.target_vel_rel_out_ne_ms(), Vector2f::new(-1.0, -2.0));
    assert_eq!(plnd.last_valid_target_ms(), 10);
}

#[test]
fn later_frames_predict_forward_from_delayed_horizon() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);

    // est pos (0,0), est vel (-1,-2). One later frame: dvel (0.1, 0.2), dt 0.1
    // vel' = (-1,-2) - (0.1, 0.2) = (-1.1, -2.2)
    // pos' = (0,0) + (-1.1, -2.2)*0.1 = (-0.11, -0.22)
    let later = [InertialSample {
        corrected_vehicle_delta_velocity_ned: Vector3f::new(0.1, 0.2, 0.0),
        dt: 0.1,
        ..InertialSample::default()
    }];
    let leftover = plnd.run_output_prediction(
        &later,
        &OutputPredictionWorld {
            now_ms: 10,
            ..OutputPredictionWorld::default()
        },
    );
    assert!(!leftover.stored_last_target_pos);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(-0.11, -0.22));
    almost_vec2(plnd.target_vel_rel_out_ne_ms(), Vector2f::new(-1.1, -2.2));
}

#[test]
fn imu_offset_adds_ned_to_position() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let world = OutputPredictionWorld {
        imu_pos_offset: Vector3f::new(0.3, 0.0, 0.0),
        now_ms: 10,
        ..OutputPredictionWorld::default()
    };
    let leftover = plnd.run_output_prediction(&[], &world);
    assert!(!leftover.stored_last_target_pos);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(0.3, 0.0));
}

#[test]
fn camera_horizontal_offset_subtracts_ned() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    // Apply after construct so this leftover is the prediction correction
    // only. Upstream zeros the camera Z before `Tbn * cam`.
    plnd.set_cam_offset_m(Vector3f::new(0.4, 0.1, 9.0));
    let leftover = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            ..OutputPredictionWorld::default()
        },
    );
    assert!(!leftover.stored_last_target_pos);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(-0.4, -0.1));
}

#[test]
fn gyro_cross_imu_offset_corrects_velocity() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    // gyro (0,0,1) % -offset(-0.2,0,0) wait: gyro % (-accel)
    // accel = (0.2, 0, 0); -accel = (-0.2, 0, 0)
    // (0,0,1) × (-0.2, 0, 0) = (0, -0.2, 0)
    // vel -= (0, -0.2) → (-1, -2) - (0, -0.2) = (-1, -1.8)
    let world = OutputPredictionWorld {
        imu_pos_offset: Vector3f::new(0.2, 0.0, 0.0),
        gyro: Vector3f::new(0.0, 0.0, 1.0),
        now_ms: 10,
        ..OutputPredictionWorld::default()
    };
    let leftover = plnd.run_output_prediction(&[], &world);
    assert!(!leftover.stored_last_target_pos);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(0.2, 0.0));
    almost_vec2(plnd.target_vel_rel_out_ne_ms(), Vector2f::new(-1.0, -1.8));
}

#[test]
fn land_offset_uses_current_body_to_ned() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        estimator_type: EstimatorType::RawSensor,
        land_ofs_cm_x: 100.0,
        land_ofs_cm_y: 0.0,
        ..PrecLandParams::default()
    });
    assert!(!plnd.init(400).skipped);
    acquire_raw(&mut plnd, 10);
    // yaw 90°: body +X (forward 1 m) → NED +Y
    let world = OutputPredictionWorld {
        rotation_body_to_ned: Matrix3f::from_euler(0.0, 0.0, radians(90.0)),
        now_ms: 10,
        ..OutputPredictionWorld::default()
    };
    let leftover = plnd.run_output_prediction(&[], &world);
    assert!(!leftover.stored_last_target_pos);
    almost_vec2(plnd.target_pos_rel_out_ne_m(), Vector2f::new(0.0, 1.0));
}

#[test]
fn prediction_stores_last_target_when_origin_available() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let world = OutputPredictionWorld {
        relative_pos_ne_origin: Some(Vector2f::new(10.0, 20.0)),
        velocity_ned: Some(Vector3f::new(3.0, 4.0, 0.5)),
        now_ms: 50,
        ..OutputPredictionWorld::default()
    };
    let leftover = plnd.run_output_prediction(&[], &world);
    assert!(leftover.stored_last_target_pos);
    assert!(leftover.stored_vehicle_velocity);
    assert_eq!(plnd.last_valid_target_ms(), 50);
    almost_vec2(
        Vector2f::new(
            plnd.last_target_pos_rel_origin_ned_m().x,
            plnd.last_target_pos_rel_origin_ned_m().y,
        ),
        Vector2f::new(10.0, 20.0),
    );
    almost(plnd.last_veh_velocity_ned_ms().x, 3.0);
    almost(plnd.last_veh_velocity_ned_ms().y, 4.0);
}

#[test]
fn getters_fail_when_target_not_acquired() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    assert!(!plnd.target_acquired(0));
    assert!(plnd
        .get_target_position_m(0, Some(Vector2f::new(1.0, 2.0)))
        .is_none());
    assert!(plnd.get_target_position_relative_ne_m(0).is_none());
    assert!(plnd.get_target_velocity_relative_ne_ms(0).is_none());
    assert!(plnd.get_target_velocity(0).is_none());
    assert!(plnd
        .get_target_location(0, Some(Location::new(1, 2)))
        .is_none());
    almost_vec2(
        plnd.get_target_velocity_ms(Vector2f::new(1.0, 1.0), 0),
        Vector2f::zero(),
    );
}

#[test]
fn getters_read_predicted_output() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let world = OutputPredictionWorld {
        relative_pos_ne_origin: Some(Vector2f::new(5.0, 7.0)),
        now_ms: 10,
        ..OutputPredictionWorld::default()
    };
    let leftover = plnd.run_output_prediction(&[], &world);
    assert!(leftover.stored_last_target_pos);

    assert!(plnd.target_acquired(10));
    almost_vec2(
        plnd.get_target_position_relative_ne_m(10).unwrap(),
        Vector2f::new(0.0, 0.0),
    );
    almost_vec2(
        plnd.get_target_velocity_relative_ne_ms(10).unwrap(),
        Vector2f::new(-1.0, -2.0),
    );
    almost_vec2(
        plnd.get_target_position_m(10, Some(Vector2f::new(5.0, 7.0)))
            .unwrap(),
        Vector2f::new(5.0, 7.0),
    );
    almost(plnd.get_target_position_measurement_ned_m().z, 2.0);
}

#[test]
fn get_target_velocity_ms_zeros_without_moving_option() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    almost_vec2(
        plnd.get_target_velocity_ms(Vector2f::new(9.0, 8.0), 0),
        Vector2f::zero(),
    );
}

#[test]
fn get_target_velocity_ms_zeros_for_raw_sensor() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        estimator_type: EstimatorType::RawSensor,
        options: OPTION_MOVING_TARGET,
        ..PrecLandParams::default()
    });
    assert!(!plnd.init(400).skipped);
    acquire_raw(&mut plnd, 10);
    let _ = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            ..OutputPredictionWorld::default()
        },
    );
    almost_vec2(
        plnd.get_target_velocity_ms(Vector2f::new(9.0, 8.0), 10),
        Vector2f::zero(),
    );
    assert!(plnd.get_target_velocity(10).is_none());
}

#[test]
fn get_target_velocity_ms_adds_vehicle_when_moving_kalman() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        estimator_type: EstimatorType::KalmanFilter,
        options: OPTION_MOVING_TARGET,
        ..PrecLandParams::default()
    });
    assert!(!plnd.init(400).skipped);
    // Kalman needs EKF_INIT_TIME_MS of good sensor before acquire. A
    // mid-window measurement keeps last_update fresh so the 500 ms
    // sensor-stale check does not abort init.
    let first = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    assert!(first.need_ekf_init);
    assert!(!plnd.estimator_target_acquired());
    let mid = plnd.run_estimator(meas_input(400, Some(down_los(2))));
    assert!(mid.need_ekf_fuse);
    assert!(!plnd.estimator_target_acquired());
    let later = plnd.run_estimator(meas_input(2_100, Some(down_los(3))));
    assert!(later.need_gcs_init_complete);
    assert!(plnd.estimator_target_acquired());
    let world = OutputPredictionWorld {
        velocity_ned: Some(Vector3f::new(3.0, 4.0, 0.0)),
        now_ms: 2_100,
        ..OutputPredictionWorld::default()
    };
    let _ = plnd.run_output_prediction(&[], &world);
    let rel = plnd.get_target_velocity_relative_ne_ms(2_100).unwrap();
    almost_vec2(
        plnd.get_target_velocity_ms(Vector2f::new(3.0, 4.0), 2_100),
        Vector2f::new(rel.x + 3.0, rel.y + 4.0),
    );
    almost_vec2(
        plnd.get_target_velocity(2_100).unwrap(),
        Vector2f::new(rel.x + 3.0, rel.y + 4.0),
    );
}

#[test]
fn target_acquired_times_out() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    assert!(plnd.target_acquired(10));
    assert!(!plnd.target_acquired(10 + LANDING_TARGET_TIMEOUT_MS + 1));
    assert!(!plnd.estimator_initialized());
}

#[test]
fn get_target_location_offsets_origin() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let world = OutputPredictionWorld {
        relative_pos_ne_origin: Some(Vector2f::new(0.0, 0.0)),
        now_ms: 10,
        ..OutputPredictionWorld::default()
    };
    let _ = plnd.run_output_prediction(&[], &world);
    let origin = Location::new(1_000_000, 2_000_000);
    let loc = plnd.get_target_location(10, Some(origin)).unwrap();
    assert_eq!(loc.lat, origin.lat);
    assert_eq!(loc.lng, origin.lng);
}

#[test]
fn check_if_sensor_in_range_defaults_and_limits() {
    let plnd = PrecLand::new();
    // Defaults are 0.75 / 8, so zero-limits "always in range" is not the default.
    assert!(!plnd.check_if_sensor_in_range(2.0, false));
    assert!(plnd.check_if_sensor_in_range(2.0, true));
    assert!(!plnd.check_if_sensor_in_range(9.0, true));
    assert!(!plnd.check_if_sensor_in_range(0.5, true));

    let mut unlimited = PrecLand::from_params(PrecLandParams {
        sensor_min_alt_m: 0.0,
        sensor_max_alt_m: 0.0,
        ..PrecLandParams::default()
    });
    let _ = unlimited.init(400);
    assert!(unlimited.check_if_sensor_in_range(99.0, false));
    assert!(unlimited.check_if_sensor_in_range(0.1, true));
    almost(SENSOR_MIN_ALT_M_DEFAULT, 0.75);
    almost(SENSOR_MAX_ALT_M_DEFAULT, 8.0);
}

#[test]
fn check_target_status_found_and_out_of_range() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let _ = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            relative_pos_ne_origin: Some(Vector2f::zero()),
            ..OutputPredictionWorld::default()
        },
    );
    plnd.check_target_status(2.0, true, 10, Some(Vector2f::zero()));
    assert_eq!(plnd.target_state(), TargetState::Found);

    // Timeout clears acquired; default alt limits + valid RF stay in range
    // so a first-seen-then-lost path becomes RecentlyLost, not OutOfRange.
    plnd.check_target_status(2.0, true, 10 + LANDING_TARGET_TIMEOUT_MS + 1, None);
    assert_eq!(plnd.target_state(), TargetState::RecentlyLost);

    plnd.check_target_status(20.0, true, 10 + LANDING_TARGET_TIMEOUT_MS + 1, None);
    assert_eq!(plnd.target_state(), TargetState::OutOfRange);
}

#[test]
fn check_target_status_recently_lost_demotes_when_far() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let _ = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            relative_pos_ne_origin: Some(Vector2f::zero()),
            ..OutputPredictionWorld::default()
        },
    );
    plnd.check_target_status(2.0, true, 10, Some(Vector2f::zero()));
    assert_eq!(plnd.target_state(), TargetState::Found);

    let far = Vector2f::new(LANDING_TARGET_LOST_DIST_THRESH_M + 1.0, 0.0);
    plnd.check_target_status(2.0, true, 10 + LANDING_TARGET_TIMEOUT_MS + 1, Some(far));
    assert_eq!(plnd.target_state(), TargetState::NeverSeen);
}

#[test]
fn check_target_status_recently_lost_demotes_on_stale_ms() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    acquire_raw(&mut plnd, 10);
    let _ = plnd.run_output_prediction(
        &[],
        &OutputPredictionWorld {
            now_ms: 10,
            relative_pos_ne_origin: Some(Vector2f::zero()),
            ..OutputPredictionWorld::default()
        },
    );
    plnd.check_target_status(2.0, true, 10, Some(Vector2f::zero()));

    plnd.check_target_status(
        2.0,
        true,
        10 + LANDING_TARGET_LOST_TIMEOUT_MS + 1,
        Some(Vector2f::zero()),
    );
    // acquired already timed out at +2000 ms; this call is far later.
    // After timeout, state goes RecentlyLost then demotes on lost-timeout.
    assert_eq!(plnd.target_state(), TargetState::NeverSeen);
}

#[test]
fn remaining_drops_prediction_and_getters() {
    assert!(REMAINING.len() >= 8, "backends and StateMachine stay later");
    assert!(!REMAINING.contains(&"AC_PrecLand::run_output_prediction"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_position_m"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_position_measurement_NED_m"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_position_relative_NE_m"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_velocity_relative_NE_ms"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_velocity_ms"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_velocity"));
    assert!(!REMAINING.contains(&"AC_PrecLand::target_acquired"));
    assert!(!REMAINING.contains(&"AC_PrecLand::get_target_location"));
    assert!(!REMAINING.contains(&"AC_PrecLand::check_target_status"));
    assert!(!REMAINING.contains(&"AC_PrecLand::check_if_sensor_in_range"));
    assert!(!REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"inertial_data_frame_s"));
    assert!(!REMAINING.contains(&"AC_PrecLand_Backend::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(!REMAINING.contains(&"PosVelEKF"));
}
