//! `AC_PrecLand` estimator leftover: `run_estimator`,
//! `check_ekf_init_timeout`, `construct_pos_meas_using_rangefinder`,
//! `retrieve_los_meas`.
//!
//! Tracked as **COP-028**. Backends and the retry state machine stay
//! later. `PosVelEKF` and `run_output_prediction` are wired.

use ap_math::matrix3::Matrix3f;
use ap_math::rotations_gen::Rotation;
use ap_math::scalar::{is_equal, radians};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_precland::{
    EstimatorInput, EstimatorType, EstimatorWorld, InertialSample, LosSample, PrecLand,
    PrecLandParams, Type, VectorFrame, EKF_INIT_SENSOR_MIN_UPDATE_MS, EKF_INIT_TIME_MS,
    EKF_INIT_VEL_VAR_NAV_INVALID, EKF_INIT_VEL_VAR_NAV_VALID, EKF_NIS_REJECT_THRESHOLD,
    EKF_OUTLIER_REJECT_LIMIT, REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec3(got: Vector3f, want: Vector3f) {
    assert!(
        is_equal(got.x, want.x) && is_equal(got.y, want.y) && is_equal(got.z, want.z),
        "({} {} {}) != ({} {} {})",
        got.x,
        got.y,
        got.z,
        want.x,
        want.y,
        want.z
    );
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

#[test]
fn retrieve_los_skips_same_timestamp() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let los = down_los(100);
    let first = plnd.retrieve_los_meas(Some(los));
    assert!(first.is_some());
    assert_eq!(plnd.last_backend_los_meas_ms(), 100);
    assert!(plnd.retrieve_los_meas(Some(los)).is_none());
    assert!(plnd.retrieve_los_meas(None).is_none());
}

#[test]
fn retrieve_los_pitch270_skips_extra_rotation() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let (vec, frame) = plnd.retrieve_los_meas(Some(down_los(1))).expect("new LOS");
    almost_vec3(vec, Vector3f::new(0.0, 0.0, 1.0));
    assert_eq!(frame, VectorFrame::BodyFrd);
}

#[test]
fn retrieve_los_yaw_align_rotates_xy() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    plnd.set_yaw_align_cd(9_000.0);
    let (vec, _) = plnd
        .retrieve_los_meas(Some(LosSample {
            time_ms: 1,
            vec_unit: Vector3f::new(1.0, 0.0, 0.0),
            frame: VectorFrame::BodyFrd,
            distance_to_target_m: 0.0,
        }))
        .expect("new LOS");
    almost_vec3(vec, Vector3f::new(0.0, 1.0, 0.0));
}

#[test]
fn retrieve_los_non_pitch270_brings_vector_forward() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        orient: Rotation::None,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    let (vec, _) = plnd.retrieve_los_meas(Some(down_los(1))).expect("new LOS");
    // Pitch90 of (0,0,1) is (1,0,0); ROTATION_NONE leaves it.
    almost_vec3(vec, Vector3f::new(1.0, 0.0, 0.0));
}

#[test]
fn construct_pos_meas_down_rangefinder() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let delayed = InertialSample::default();
    let world = EstimatorWorld::default();
    assert!(plnd.construct_pos_meas_using_rangefinder(
        2.0,
        true,
        &delayed,
        Some(down_los(1)),
        &world
    ));
    almost_vec3(
        plnd.target_pos_rel_meas_ned_m(),
        Vector3f::new(0.0, 0.0, 2.0),
    );
}

#[test]
fn construct_pos_meas_rejects_away_from_approach() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let delayed = InertialSample::default();
    let world = EstimatorWorld::default();
    let away = LosSample {
        time_ms: 1,
        vec_unit: Vector3f::new(0.0, 0.0, -1.0),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 0.0,
    };
    assert!(!plnd.construct_pos_meas_using_rangefinder(2.0, true, &delayed, Some(away), &world));
}

#[test]
fn construct_pos_meas_tilted_los_scales_by_approach() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let delayed = InertialSample::default();
    let world = EstimatorWorld::default();
    let tilted = LosSample {
        time_ms: 1,
        vec_unit: Vector3f::new(0.6, 0.0, 0.8),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 0.0,
    };
    assert!(plnd.construct_pos_meas_using_rangefinder(2.0, true, &delayed, Some(tilted), &world));
    // dist = 2.0 / 0.8 = 2.5; meas = (0.6,0,0.8)*2.5 = (1.5, 0, 2.0)
    almost_vec3(
        plnd.target_pos_rel_meas_ned_m(),
        Vector3f::new(1.5, 0.0, 2.0),
    );
}

#[test]
fn construct_pos_meas_uses_backend_distance() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let delayed = InertialSample::default();
    let world = EstimatorWorld::default();
    let los = LosSample {
        time_ms: 1,
        vec_unit: Vector3f::new(0.0, 0.0, 1.0),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 3.0,
    };
    assert!(plnd.construct_pos_meas_using_rangefinder(2.0, false, &delayed, Some(los), &world));
    almost_vec3(
        plnd.target_pos_rel_meas_ned_m(),
        Vector3f::new(0.0, 0.0, 3.0),
    );
}

#[test]
fn construct_pos_meas_cam_and_imu_offset() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    // 0.125 / 0.0625 are exact in f32; 0.1 is not.
    plnd.set_cam_offset_m(Vector3f::new(0.0, 0.0, 0.125));
    let delayed = InertialSample::default();
    let world = EstimatorWorld {
        imu_pos_offset: Vector3f::new(0.0, 0.0, 0.062_5),
        relative_pos_ned: Some(Vector3f::new(4.0, 5.0, -3.0)),
        ..EstimatorWorld::default()
    };
    assert!(plnd.construct_pos_meas_using_rangefinder(
        2.0,
        true,
        &delayed,
        Some(down_los(1)),
        &world
    ));
    // dist_along = 2.0 - 0.125 = 1.875; meas = (0,0,1.875) + (0,0,0.0625)
    almost_vec3(
        plnd.target_pos_rel_meas_ned_m(),
        Vector3f::new(0.0, 0.0, 1.937_5),
    );
    almost(plnd.last_target_pos_rel_origin_ned_m().z, -3.0);
    almost_vec3(plnd.last_vehicle_pos_ned_m(), Vector3f::new(4.0, 5.0, -3.0));
}

#[test]
fn construct_pos_meas_local_frd_rotates_horizontal() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    // Approach after Pitch270 is body (0,0,1). Identity Tbn keeps it NED down.
    let delayed = InertialSample {
        tbn: Matrix3f::from_euler(0.0, 0.0, radians(90.0)),
        ..InertialSample::default()
    };
    let world = EstimatorWorld::default();
    let los = LosSample {
        time_ms: 1,
        vec_unit: Vector3f::new(0.6, 0.0, 0.8),
        frame: VectorFrame::LocalFrd,
        distance_to_target_m: 0.0,
    };
    assert!(plnd.construct_pos_meas_using_rangefinder(2.0, true, &delayed, Some(los), &world));
    // rotate_xy(90°) of (0.6, 0, 0.8) = (0, 0.6, 0.8)
    // approach_NED = Tbn * (0,0,1). from_euler(0,0,90) * (0,0,1) = (0,0,1)
    // dist = 2.0 / 0.8 = 2.5; meas = (0, 0.6, 0.8)*2.5 = (0, 1.5, 2.0)
    almost_vec3(
        plnd.target_pos_rel_meas_ned_m(),
        Vector3f::new(0.0, 1.5, 2.0),
    );
}

#[test]
fn raw_sensor_invalid_velocity_clears_acquired() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    let first = plnd.run_estimator(meas_input(10, Some(down_los(1))));
    assert!(first.constructed_pos_meas);
    assert!(plnd.estimator_target_acquired());
    assert!(first.need_output_prediction);

    let mut input = meas_input(20, Some(down_los(2)));
    input.any_inertial_nav_invalid = true;
    let second = plnd.run_estimator(input);
    assert!(second.raw_sensor_invalid_velocity);
    assert!(!second.constructed_pos_meas);
    assert!(!plnd.estimator_target_acquired());
    assert!(!second.need_output_prediction);
}

#[test]
fn raw_sensor_sets_estimate_and_predicts() {
    let mut plnd = mavlink_inited(EstimatorType::RawSensor);
    let mut first_in = meas_input(10, Some(down_los(1)));
    first_in.delayed.inertial_nav_velocity = Vector3f::new(1.0, 2.0, 0.0);
    let first = plnd.run_estimator(first_in);
    assert!(first.need_gcs_target_found);
    assert!(plnd.estimator_initialized());
    assert!(plnd.estimator_target_acquired());
    almost_vec2(plnd.target_pos_rel_est_ne_m(), Vector2f::new(0.0, 0.0));
    almost_vec2(plnd.target_vel_rel_est_ne_ms(), Vector2f::new(-1.0, -2.0));

    let mut second_in = meas_input(20, None);
    second_in.delayed.inertial_nav_velocity = Vector3f::new(1.0, 2.0, 0.0);
    second_in.delayed.dt = 0.1;
    let second = plnd.run_estimator(second_in);
    assert!(!second.constructed_pos_meas);
    assert!(second.need_output_prediction);
    almost_vec2(plnd.target_pos_rel_est_ne_m(), Vector2f::new(-0.1, -0.2));
    almost_vec2(plnd.target_vel_rel_est_ne_ms(), Vector2f::new(-1.0, -2.0));
}

#[test]
fn kalman_first_meas_inits_but_does_not_acquire() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let leftover = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    assert!(leftover.constructed_pos_meas);
    assert!(leftover.need_gcs_target_found);
    assert!(leftover.need_ekf_init);
    assert!(!leftover.need_ekf_predict);
    assert!(!leftover.need_output_prediction);
    assert!(plnd.estimator_initialized());
    assert!(!plnd.estimator_target_acquired());
    almost(leftover.ekf_init_vel_var, EKF_INIT_VEL_VAR_NAV_VALID);
    // meas.z=2, gyro=0 → sq(2*(0.01)+0.02) = sq(0.04) = 0.0016
    almost(leftover.ekf_pos_var, 0.001_6);
    almost(plnd.ekf_x().pos(), 0.0);
    almost(plnd.ekf_y().pos(), 0.0);
    almost(plnd.ekf_x().vel(), 0.0);
    almost(plnd.ekf_y().vel(), 0.0);
    assert_eq!(
        plnd.ekf_x().cov(),
        [0.001_6, 0.0, EKF_INIT_VEL_VAR_NAV_VALID]
    );
}

#[test]
fn kalman_init_uses_wide_vel_var_when_nav_invalid() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let mut input = meas_input(0, Some(down_los(1)));
    input.delayed.inertial_nav_velocity_valid = false;
    let leftover = plnd.run_estimator(input);
    assert!(leftover.need_ekf_init);
    almost(leftover.ekf_init_vel_var, EKF_INIT_VEL_VAR_NAV_INVALID);
    almost(plnd.ekf_x().vel(), 0.0);
    almost(plnd.ekf_y().vel(), 0.0);
    almost(plnd.ekf_x().cov()[2], EKF_INIT_VEL_VAR_NAV_INVALID);
}

#[test]
fn kalman_init_fails_if_sensor_goes_stale() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    assert!(plnd.estimator_initialized());

    let leftover = plnd.run_estimator(meas_input(EKF_INIT_SENSOR_MIN_UPDATE_MS + 1, None));
    assert!(leftover.need_gcs_init_failed);
    assert!(!plnd.estimator_initialized());
    assert!(!plnd.estimator_target_acquired());
}

fn kalman_settle(plnd: &mut PrecLand) -> ap_precland::RunEstimatorLeftover {
    // Init-complete needs a recent fuse (`now - last_update <= 500`)
    // *and* `now - estimator_init > 2000`. A single jump to t=2001
    // trips `LANDING_TARGET_TIMEOUT_MS` and restarts init.
    let _ = plnd.run_estimator(meas_input(1_800, Some(down_los(2))));
    plnd.run_estimator(meas_input(EKF_INIT_TIME_MS + 1, Some(down_los(3))))
}

fn tilted_los(time_ms: u32) -> LosSample {
    LosSample {
        time_ms,
        vec_unit: Vector3f::new(0.6, 0.0, 0.8),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 0.0,
    }
}

#[test]
fn kalman_predict_records_del_vel_leftover() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));

    let mut later = meas_input(10, None);
    later.delayed.dt = 0.01;
    later.delayed.corrected_vehicle_delta_velocity_ned = Vector3f::new(0.2, -0.4, 0.0);
    let leftover = plnd.run_estimator(later);
    assert!(leftover.need_ekf_predict);
    almost(leftover.ekf_predict_dt, 0.01);
    almost_vec2(leftover.ekf_predict_del_vel_ne, Vector2f::new(-0.2, 0.4));
    almost(leftover.ekf_predict_accel_noise, 2.5 * 0.01);
    // init vel is 0; predict vel' = dVel + vel = (-0.2, 0.4)
    almost(plnd.ekf_x().vel(), -0.2);
    almost(plnd.ekf_y().vel(), 0.4);
    almost(plnd.ekf_x().pos(), 0.0);
    almost(plnd.ekf_y().pos(), 0.0);
}

#[test]
fn kalman_outlier_rejects_then_accepts() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));

    for i in 0..EKF_OUTLIER_REJECT_LIMIT {
        let leftover = plnd.run_estimator(meas_input(10 + i * 10, Some(tilted_los(10 + i * 10))));
        assert!(leftover.outlier_rejected);
        assert!(!leftover.need_ekf_fuse);
        assert!(leftover.ekf_max_nis >= EKF_NIS_REJECT_THRESHOLD);
        assert_eq!(plnd.outlier_reject_count(), i + 1);
    }

    let leftover = plnd.run_estimator(meas_input(100, Some(tilted_los(100))));
    assert!(!leftover.outlier_rejected);
    assert!(leftover.need_ekf_fuse);
    assert_eq!(plnd.outlier_reject_count(), 0);
}

#[test]
fn kalman_second_good_meas_computes_nis_and_fuses() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    let leftover = plnd.run_estimator(meas_input(10, Some(down_los(2))));
    assert!(leftover.need_ekf_predict);
    assert!(leftover.need_ekf_fuse);
    assert!(leftover.ekf_max_nis < EKF_NIS_REJECT_THRESHOLD);
    assert!(!leftover.outlier_rejected);
}

#[test]
fn kalman_init_completes_after_two_seconds() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    let leftover = kalman_settle(&mut plnd);
    assert!(leftover.need_ekf_predict);
    assert!(leftover.need_ekf_fuse);
    assert!(leftover.need_gcs_init_complete);
    assert!(plnd.estimator_target_acquired());
    assert!(leftover.need_output_prediction);
    almost_vec2(
        plnd.target_pos_rel_est_ne_m(),
        Vector2f::new(plnd.ekf_x().pos(), plnd.ekf_y().pos()),
    );
    almost_vec2(
        plnd.target_vel_rel_est_ne_ms(),
        Vector2f::new(plnd.ekf_x().vel(), plnd.ekf_y().vel()),
    );
}

#[test]
fn kalman_init_uses_negative_inertial_nav_velocity() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let mut input = meas_input(0, Some(down_los(1)));
    input.delayed.inertial_nav_velocity = Vector3f::new(1.0, 2.0, 0.0);
    let leftover = plnd.run_estimator(input);
    assert!(leftover.need_ekf_init);
    almost(plnd.ekf_x().vel(), -1.0);
    almost(plnd.ekf_y().vel(), -2.0);
}

#[test]
fn check_ekf_init_timeout_is_a_noop_when_acquired() {
    let mut plnd = mavlink_inited(EstimatorType::KalmanFilter);
    let _ = plnd.run_estimator(meas_input(0, Some(down_los(1))));
    let _ = kalman_settle(&mut plnd);
    assert!(plnd.estimator_target_acquired());

    let leftover = plnd.check_ekf_init_timeout(EKF_INIT_TIME_MS + 50);
    assert!(!leftover.need_gcs_init_failed);
    assert!(!leftover.need_gcs_init_complete);
}

#[test]
fn leftover_catalog_drops_estimator_symbols() {
    assert!(
        REMAINING.len() > 10,
        "estimator slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"AC_PrecLand::run_estimator"));
    assert!(!REMAINING.contains(&"AC_PrecLand::check_ekf_init_timeout"));
    assert!(!REMAINING.contains(&"AC_PrecLand::construct_pos_meas_using_rangefinder"));
    assert!(!REMAINING.contains(&"AC_PrecLand::retrieve_los_meas"));
    assert!(!REMAINING.contains(&"AC_PrecLand::run_output_prediction"));
    assert!(!REMAINING.contains(&"AC_PrecLand::target_acquired"));
    assert!(REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"PosVelEKF"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(REMAINING.contains(&"inertial_data_frame_s"));
}
