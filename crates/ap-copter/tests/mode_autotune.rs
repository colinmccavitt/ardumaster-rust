//! `ModeAutoTune` init / run leftover, upstream `ArduCopter/mode_autotune.cpp`.

use ap_copter::mode_autotune::{
    allows_autotune, autotune_currently_level, autotune_has_user_takeoff, autotune_mode_flags,
    autotune_test_run, autotune_use_poshold, direction_sign, first_enabled_axis,
    mode_autotune_init, mode_autotune_run, pitch_enabled, reverse_test_direction, roll_enabled,
    twitch_is_angle_p, twitch_is_heli_only, twitch_lean_angle_cd, twitching_abort_rate,
    twitching_measure_acceleration, twitching_test_angle, twitching_test_rate, yaw_d_enabled,
    yaw_enabled, AutoTuneInitFail, AutoTuneInitView, AutoTuneRunView, AutoTuneTwitchView, AxisType,
    CurrentlyLevelView, GainType, Step, TuneMode, TuneType, TwitchTick, AUTOTUNE_AGGR_DEFAULT,
    AUTOTUNE_AXIS_BITMASK_DEFAULT, AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_YAW,
    AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_LEVEL_ANGLE_CD, AUTOTUNE_LEVEL_RATE_RP_CD,
    AUTOTUNE_LEVEL_TIMEOUT_MS, AUTOTUNE_MESSAGE_STARTED, AUTOTUNE_MESSAGE_TESTING,
    AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS, AUTOTUNE_REQUIRED_LEVEL_TIME_MS, AUTOTUNE_SUCCESS_COUNT,
    AUTOTUNE_TARGET_RATE_RLLPIT_CDS, AUTOTUNE_TESTING_STEP_TIMEOUT_MS, MODE_NUMBER_ALT_HOLD,
    MODE_NUMBER_AUTOTUNE, MODE_NUMBER_STABILIZE,
};
use ap_copter::mode_loiter::MODE_NUMBER_LOITER;
use ap_copter::mode_poshold::MODE_NUMBER_POSHOLD;
use ap_math::scalar::cd_to_rad;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn autotune_number_is_fifteen() {
    assert_eq!(MODE_NUMBER_AUTOTUNE, 15);
    assert_eq!(autotune_mode_flags().mode_number, MODE_NUMBER_AUTOTUNE);
}

#[test]
fn autotune_flags_are_auto_throttle_no_arming() {
    let flags = autotune_mode_flags();
    assert!(!flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(!flags.allows_arming);
    assert!(!flags.is_autopilot);
}

#[test]
fn user_takeoff_is_never_allowed() {
    assert!(!autotune_has_user_takeoff(false));
    assert!(!autotune_has_user_takeoff(true));
}

#[test]
fn only_four_from_modes_allow_autotune() {
    assert!(allows_autotune(MODE_NUMBER_STABILIZE));
    assert!(allows_autotune(MODE_NUMBER_ALT_HOLD));
    assert!(allows_autotune(MODE_NUMBER_LOITER));
    assert!(allows_autotune(MODE_NUMBER_POSHOLD));
    assert!(!allows_autotune(MODE_NUMBER_AUTOTUNE));
    assert!(!allows_autotune(1)); // ACRO
    assert!(!allows_autotune(3)); // AUTO
    assert!(!allows_autotune(6)); // RTL
}

#[test]
fn poshold_only_from_loiter_or_poshold() {
    assert!(!autotune_use_poshold(MODE_NUMBER_STABILIZE));
    assert!(!autotune_use_poshold(MODE_NUMBER_ALT_HOLD));
    assert!(autotune_use_poshold(MODE_NUMBER_LOITER));
    assert!(autotune_use_poshold(MODE_NUMBER_POSHOLD));
}

#[test]
fn default_axis_mask_is_roll_pitch_yaw() {
    assert_eq!(AUTOTUNE_AXIS_BITMASK_DEFAULT, 7);
    assert_eq!(AUTOTUNE_SUCCESS_COUNT, 4);
    assert!(roll_enabled(AUTOTUNE_AXIS_BITMASK_DEFAULT));
    assert!(pitch_enabled(AUTOTUNE_AXIS_BITMASK_DEFAULT));
    assert!(yaw_enabled(AUTOTUNE_AXIS_BITMASK_DEFAULT));
    assert!(!yaw_d_enabled(AUTOTUNE_AXIS_BITMASK_DEFAULT));
    assert_eq!(
        first_enabled_axis(AUTOTUNE_AXIS_BITMASK_DEFAULT),
        Some(AxisType::Roll)
    );
}

#[test]
fn first_axis_follows_roll_pitch_yaw_yaw_d() {
    assert_eq!(first_enabled_axis(0), None);
    assert_eq!(
        first_enabled_axis(AUTOTUNE_AXIS_BITMASK_PITCH),
        Some(AxisType::Pitch)
    );
    assert_eq!(
        first_enabled_axis(AUTOTUNE_AXIS_BITMASK_YAW),
        Some(AxisType::Yaw)
    );
    assert_eq!(
        first_enabled_axis(AUTOTUNE_AXIS_BITMASK_YAW_D),
        Some(AxisType::YawD)
    );
    assert_eq!(
        first_enabled_axis(AUTOTUNE_AXIS_BITMASK_PITCH | AUTOTUNE_AXIS_BITMASK_YAW),
        Some(AxisType::Pitch)
    );
}

#[test]
fn from_mode_refused_before_flying_or_internals() {
    let mut view = AutoTuneInitView::typical();
    view.from_mode_number = MODE_NUMBER_AUTOTUNE;
    view.armed = false;
    let out = mode_autotune_init(true, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(AutoTuneInitFail::FromModeRefused));
    assert!(!out.init_position_controller);
    assert!(!out.backup_gains);
    assert!(out.mode.is_none());
}

#[test]
fn throttle_zero_refuses() {
    let mut view = AutoTuneInitView::typical();
    view.throttle_zero = true;
    let out = mode_autotune_init(false, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(AutoTuneInitFail::ThrottleZero));
    assert!(!out.backup_gains);
}

#[test]
fn not_flying_fails_when_disarmed_or_landed() {
    let mut disarmed = AutoTuneInitView::typical();
    disarmed.armed = false;
    assert_eq!(
        mode_autotune_init(false, &disarmed).fail,
        Some(AutoTuneInitFail::NotFlying)
    );

    let mut not_auto = AutoTuneInitView::typical();
    not_auto.auto_armed = false;
    assert_eq!(
        mode_autotune_init(true, &not_auto).fail,
        Some(AutoTuneInitFail::NotFlying)
    );

    let mut landed = AutoTuneInitView::typical();
    landed.land_complete = true;
    assert_eq!(
        mode_autotune_init(true, &landed).fail,
        Some(AutoTuneInitFail::NotFlying)
    );
}

#[test]
fn motors_missing_fails_inside_init_internals() {
    let mut view = AutoTuneInitView::typical();
    view.motors_present = false;
    let out = mode_autotune_init(false, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(AutoTuneInitFail::MotorsNotArmed));
    assert!(!out.init_position_controller);
}

#[test]
fn ignore_checks_cannot_bypass_gates() {
    let mut view = AutoTuneInitView::typical();
    view.throttle_zero = true;
    assert_eq!(
        mode_autotune_init(true, &view).fail,
        Some(AutoTuneInitFail::ThrottleZero)
    );
    assert_eq!(
        mode_autotune_init(false, &view).fail,
        Some(AutoTuneInitFail::ThrottleZero)
    );
}

#[test]
fn first_start_from_stabilize_tunes_without_poshold() {
    let view = AutoTuneInitView::typical();
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert!(out.fail.is_none());
    assert_eq!(out.use_poshold, Some(false));
    assert!(out.init_position_controller);
    assert!(out.backup_gains);
    assert_eq!(out.mode, Some(TuneMode::Tuning));
    assert_eq!(out.axis, Some(AxisType::Roll));
    assert_eq!(out.axes_completed, Some(0));
    assert_eq!(out.step, Some(Step::WaitingForLevel));
    assert_eq!(out.have_position, Some(false));
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_STARTED));
}

#[test]
fn first_start_from_loiter_asks_for_poshold() {
    let view = AutoTuneInitView::typical_loiter();
    let out = mode_autotune_init(true, &view);
    assert!(out.ok);
    assert_eq!(out.use_poshold, Some(true));
    assert_eq!(out.mode, Some(TuneMode::Tuning));
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_STARTED));
}

#[test]
fn first_start_from_poshold_asks_for_poshold() {
    let mut view = AutoTuneInitView::typical();
    view.from_mode_number = MODE_NUMBER_POSHOLD;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert_eq!(out.use_poshold, Some(true));
}

#[test]
fn first_start_from_althold_has_no_poshold() {
    let mut view = AutoTuneInitView::typical();
    view.from_mode_number = MODE_NUMBER_ALT_HOLD;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert_eq!(out.use_poshold, Some(false));
}

#[test]
fn failed_restarts_like_uninitialised() {
    let mut view = AutoTuneInitView::typical();
    view.mode = TuneMode::Failed;
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_PITCH;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert!(out.backup_gains);
    assert_eq!(out.mode, Some(TuneMode::Tuning));
    assert_eq!(out.axis, Some(AxisType::Pitch));
    assert_eq!(out.axes_completed, Some(0));
    assert_eq!(out.step, Some(Step::WaitingForLevel));
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_STARTED));
}

#[test]
fn tuning_resume_keeps_axis_and_does_not_backup() {
    let mut view = AutoTuneInitView::typical();
    view.mode = TuneMode::Tuning;
    view.axis = AxisType::Yaw;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert!(!out.backup_gains);
    assert_eq!(out.mode, Some(TuneMode::Tuning));
    assert_eq!(out.axis, Some(AxisType::Yaw));
    assert!(out.axes_completed.is_none());
    assert_eq!(out.step, Some(Step::WaitingForLevel));
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_STARTED));
}

#[test]
fn finished_and_validating_enter_pilot_testing() {
    for prior in [TuneMode::Finished, TuneMode::Validating] {
        let mut view = AutoTuneInitView::typical();
        view.mode = prior;
        view.axis = AxisType::Pitch;
        let out = mode_autotune_init(true, &view);
        assert!(out.ok, "{prior:?}");
        assert!(!out.backup_gains);
        assert_eq!(out.mode, Some(TuneMode::Validating));
        assert_eq!(out.axis, Some(AxisType::Pitch));
        assert!(out.step.is_none());
        assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_TESTING));
        assert_eq!(out.have_position, Some(false));
    }
}

#[test]
fn yaw_d_only_mask_starts_on_yaw_d() {
    let mut view = AutoTuneInitView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW_D;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert_eq!(out.axis, Some(AxisType::YawD));
    assert!(yaw_d_enabled(view.axis_bitmask));
    assert!(!roll_enabled(view.axis_bitmask));
}

#[test]
fn landed_at_ground_idle_disarms_and_skips_library() {
    let mut view = AutoTuneRunView::typical();
    view.land_complete = true;
    view.spool_state = SpoolState::GroundIdle;
    let out = mode_autotune_run(&view);
    assert!(out.update_simple_mode);
    assert!(out.disarmed_landed);
    assert!(out.make_safe_ground_handling);
    assert!(!out.library_run);
    assert!(!out.init_z_limits);
    assert!(out.desired_spool.is_none());
}

#[test]
fn landed_not_idle_is_safe_ground_without_disarm() {
    let mut view = AutoTuneRunView::typical();
    view.land_complete = true;
    view.spool_state = SpoolState::ThrottleUnlimited;
    let out = mode_autotune_run(&view);
    assert!(!out.disarmed_landed);
    assert!(out.make_safe_ground_handling);
    assert!(!out.library_run);
}

#[test]
fn disarmed_or_interlock_off_idles_without_d_update() {
    let mut disarmed = AutoTuneRunView::typical();
    disarmed.armed = false;
    let out = mode_autotune_run(&disarmed);
    assert!(out.library_run);
    assert!(out.init_z_limits);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::GroundIdle));
    assert!(out.throttle_out_zero);
    assert!(out.d_relax);
    assert!(!out.d_update);
    assert!(!out.set_climb_rate);
    assert!(!out.control_attitude);

    let mut locked = AutoTuneRunView::typical();
    locked.interlock = false;
    let out = mode_autotune_run(&locked);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::GroundIdle));
    assert!(!out.d_update);
}

#[test]
fn typical_waiting_holds_intra_test_until_level_time() {
    let view = AutoTuneRunView::typical();
    let out = mode_autotune_run(&view);
    assert!(out.library_run);
    assert!(out.init_z_limits);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert!(out.set_climb_rate);
    assert!(out.d_update);
    assert!(out.poshold_called);
    assert!(out.control_attitude);
    assert!(out.do_gcs_announcements);
    assert_eq!(out.currently_level, Some(true));
    assert_eq!(out.loaded_gains, Some(GainType::IntraTest));
    assert!(out.input_euler_rp_yaw);
    assert!(!out.test_init);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.mode, TuneMode::Tuning);
}

#[test]
fn waiting_level_for_required_time_starts_executing() {
    let mut view = AutoTuneRunView::typical();
    view.step_start_time_ms = view.now_ms - AUTOTUNE_REQUIRED_LEVEL_TIME_MS - 1;
    let out = mode_autotune_run(&view);
    assert_eq!(out.currently_level, Some(true));
    assert!(out.test_init);
    assert_eq!(out.step, Step::ExecutingTest);
    assert_eq!(out.loaded_gains, Some(GainType::Test));
    assert_eq!(out.step_start_time_ms, view.now_ms);
    assert_eq!(out.step_timeout_ms, AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
}

#[test]
fn waiting_not_level_resets_step_start() {
    let mut view = AutoTuneRunView::typical();
    view.roll_rad = 0.2;
    view.step_start_time_ms = view.now_ms - AUTOTUNE_REQUIRED_LEVEL_TIME_MS - 1;
    let out = mode_autotune_run(&view);
    assert_eq!(out.currently_level, Some(false));
    assert!(!out.test_init);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.step_start_time_ms, view.now_ms);
}

#[test]
fn fail_to_level_sets_failed_and_stays_waiting() {
    let mut view = AutoTuneRunView::typical();
    view.level_start_time_ms = view.now_ms - (3 * AUTOTUNE_LEVEL_TIMEOUT_MS + 1);
    view.roll_rad = 0.2;
    let out = mode_autotune_run(&view);
    assert!(out.failed_to_level);
    assert_eq!(out.mode, TuneMode::Failed);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.currently_level, Some(false));
}

#[test]
fn yaw_slew_resets_level_start() {
    let mut view = CurrentlyLevelView {
        now_ms: 10_000,
        level_start_time_ms: 1_000,
        desired_roll_rad: 0.0,
        desired_pitch_rad: 0.0,
        desired_yaw_rad: 0.0,
        roll_rad: 0.0,
        pitch_rad: 0.0,
        yaw_rad: 0.0,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,
        yaw_rate_ef_target_rads: 0.6,
        slew_yaw_max_rads: 1.0,
    };
    let out = autotune_currently_level(&view);
    assert_eq!(out.level_start_time_ms, 10_000);
    assert!(!out.failed);

    view.yaw_rate_ef_target_rads = 0.4;
    let out = autotune_currently_level(&view);
    assert_eq!(out.level_start_time_ms, 1_000);
}

#[test]
fn currently_level_gyro_is_signed_not_abs() {
    let mut view = CurrentlyLevelView {
        now_ms: 10_000,
        level_start_time_ms: 8_000,
        desired_roll_rad: 0.0,
        desired_pitch_rad: 0.0,
        desired_yaw_rad: 0.0,
        roll_rad: 0.0,
        pitch_rad: 0.0,
        yaw_rad: 0.0,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,
        yaw_rate_ef_target_rads: 0.0,
        slew_yaw_max_rads: 1.0,
    };
    assert!(autotune_currently_level(&view).level);

    view.gyro_x = cd_to_rad(AUTOTUNE_LEVEL_RATE_RP_CD) + 0.01;
    assert!(!autotune_currently_level(&view).level);

    view.gyro_x = -(cd_to_rad(AUTOTUNE_LEVEL_RATE_RP_CD) + 0.01);
    assert!(
        autotune_currently_level(&view).level,
        "upstream gyro checks are `>` not fabsf"
    );
}

#[test]
fn currently_level_angle_uses_abs() {
    let mut view = CurrentlyLevelView {
        now_ms: 10_000,
        level_start_time_ms: 8_000,
        desired_roll_rad: 0.0,
        desired_pitch_rad: 0.0,
        desired_yaw_rad: 0.0,
        roll_rad: -(cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD) + 0.01),
        pitch_rad: 0.0,
        yaw_rad: 0.0,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,
        yaw_rate_ef_target_rads: 0.0,
        slew_yaw_max_rads: 1.0,
    };
    assert!(!autotune_currently_level(&view).level);
    view.roll_rad = cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD) + 0.01;
    assert!(!autotune_currently_level(&view).level);
}

#[test]
fn stick_input_enters_pilot_override() {
    let mut view = AutoTuneRunView::typical();
    view.desired_roll_rad = 0.1;
    view.have_position = true;
    let out = mode_autotune_run(&view);
    assert!(out.pilot_override);
    assert_eq!(out.override_time, view.now_ms);
    assert!(!out.have_position);
    assert_eq!(out.loaded_gains, Some(GainType::Original));
    assert!(out.input_euler_rp_yaw_rate);
    assert!(!out.control_attitude);
    assert!(out.pilot_override_warning);
    assert!(!out.poshold_called);
}

#[test]
fn climb_or_yaw_also_overrides() {
    let mut climb = AutoTuneRunView::typical();
    climb.target_climb_rate_ms = 0.5;
    let out = mode_autotune_run(&climb);
    assert!(out.pilot_override);
    assert!(out.have_position == climb.have_position);

    let mut yaw = AutoTuneRunView::typical();
    yaw.desired_yaw_rate_rads = 0.2;
    assert!(mode_autotune_run(&yaw).pilot_override);
}

#[test]
fn override_releases_after_timeout_and_runs_attitude() {
    let mut view = AutoTuneRunView::typical();
    view.pilot_override = true;
    view.override_time = view.now_ms - AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS - 1;
    view.yaw_rad = 0.4;
    let out = mode_autotune_run(&view);
    assert!(!out.pilot_override);
    assert!(out.control_attitude);
    assert!(out.do_gcs_announcements);
    assert_eq!(out.desired_yaw_rad, 0.4);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.step_start_time_ms, view.now_ms);
    assert_eq!(out.level_start_time_ms, view.now_ms);
}

#[test]
fn override_holds_until_timeout() {
    let mut view = AutoTuneRunView::typical();
    view.pilot_override = true;
    view.override_time = view.now_ms - AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS;
    let out = mode_autotune_run(&view);
    assert!(out.pilot_override);
    assert!(!out.control_attitude);
    assert_eq!(out.loaded_gains, Some(GainType::Original));
}

#[test]
fn finished_and_failed_fly_original() {
    for mode in [TuneMode::Finished, TuneMode::Failed] {
        let mut view = AutoTuneRunView::typical();
        view.mode = mode;
        let out = mode_autotune_run(&view);
        assert_eq!(out.loaded_gains, Some(GainType::Original));
        assert!(out.input_euler_rp_yaw_rate);
        assert!(!out.control_attitude);
        assert_eq!(
            out.desired_spool,
            Some(DesiredSpoolState::ThrottleUnlimited)
        );
    }
}

#[test]
fn validating_flies_tuned() {
    let mut view = AutoTuneRunView::typical();
    view.mode = TuneMode::Validating;
    let out = mode_autotune_run(&view);
    assert_eq!(out.loaded_gains, Some(GainType::Tuned));
    assert!(out.input_euler_rp_yaw_rate);
    assert!(!out.control_attitude);
}

#[test]
fn uninitialised_is_flow_of_control_then_original() {
    let mut view = AutoTuneRunView::typical();
    view.mode = TuneMode::Uninitialised;
    let out = mode_autotune_run(&view);
    assert!(out.flow_of_control);
    assert_eq!(out.loaded_gains, Some(GainType::Original));
    assert!(out.input_euler_rp_yaw_rate);
    assert!(out.d_update);
}

#[test]
fn executing_test_stays_until_twitch_decides() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    let out = mode_autotune_run(&view);
    assert!(out.test_run);
    assert!(!out.test_init);
    assert!(!out.update_gains);
    assert_eq!(out.step, Step::ExecutingTest);
    assert_eq!(out.loaded_gains, Some(GainType::Test));
    assert!(out.input_rate_step);
    assert!(!out.input_angle_step);
}

#[test]
fn executing_test_done_writes_update_gains_same_tick() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    view.rotation_rate = view.target_rate + 1.0;
    let out = mode_autotune_run(&view);
    assert!(out.test_run);
    assert!(!out.update_gains);
    assert_eq!(out.step, Step::UpdateGains);
}

#[test]
fn executing_test_twitch_abort_writes_abort() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    view.roll_rad = cd_to_rad(view.angle_abort);
    view.rotation_rate = 10.0;
    let out = mode_autotune_run(&view);
    assert_eq!(out.step, Step::Abort);
    assert!(!out.update_gains);
    assert!((out.step_scaler - 0.9).abs() < f32::EPSILON);
}

#[test]
fn lean_abort_overrides_twitch_done() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    view.rotation_rate = view.target_rate + 1.0;
    view.lean_angle_deg = 40.0;
    view.angle_lim_max_rp_cd = 3750.0;
    let out = mode_autotune_run(&view);
    assert_eq!(out.step, Step::Abort);

    let mut neg = AutoTuneRunView::typical();
    neg.step = Step::ExecutingTest;
    neg.roll_rad = cd_to_rad(-901.0);
    neg.angle_lim_neg_rpy_cd = 900.0;
    assert_eq!(mode_autotune_run(&neg).step, Step::Abort);
}

#[test]
fn yaw_twitch_updates_desired_yaw() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    view.axis = AxisType::Yaw;
    view.yaw_rad = 1.2;
    let out = mode_autotune_run(&view);
    assert_eq!(out.desired_yaw_rad, 1.2);

    view.axis = AxisType::YawD;
    view.yaw_rad = -0.5;
    assert_eq!(mode_autotune_run(&view).desired_yaw_rad, -0.5);

    view.axis = AxisType::Roll;
    view.yaw_rad = 2.0;
    assert_eq!(
        mode_autotune_run(&view).desired_yaw_rad,
        view.desired_yaw_rad
    );
}

#[test]
fn update_gains_falls_through_to_waiting() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.positive_direction = true;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.loaded_gains, Some(GainType::IntraTest));
    assert!(out.input_euler_rp_yaw);
    assert!(!out.positive_direction);
    assert_eq!(out.step_start_time_ms, view.now_ms);
    assert_eq!(out.level_start_time_ms, view.now_ms);
    assert_eq!(out.step_timeout_ms, AUTOTUNE_REQUIRED_LEVEL_TIME_MS);
}

#[test]
fn abort_recovers_to_waiting_and_reverses() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::Abort;
    view.positive_direction = false;
    let out = mode_autotune_run(&view);
    assert!(!out.update_gains);
    assert_eq!(out.step, Step::WaitingForLevel);
    assert!(out.positive_direction);
    assert_eq!(reverse_test_direction(false), true);
}

#[test]
fn poshold_latches_when_zero_rp_and_ok() {
    let mut view = AutoTuneRunView::typical();
    view.use_poshold = true;
    view.position_ok = true;
    view.have_position = false;
    let out = mode_autotune_run(&view);
    assert!(out.poshold_called);
    assert!(out.have_position);

    view.position_ok = false;
    assert!(!mode_autotune_run(&view).have_position);
}

#[test]
fn copter_level_constants_are_not_plane() {
    assert_eq!(AUTOTUNE_LEVEL_ANGLE_CD, 250.0);
    assert_eq!(AUTOTUNE_LEVEL_RATE_RP_CD, 500.0);
    assert_eq!(AUTOTUNE_REQUIRED_LEVEL_TIME_MS, 250);
    assert_eq!(AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS, 500);
}

#[test]
fn direction_sign_follows_positive_direction() {
    assert_eq!(direction_sign(true), 1.0);
    assert_eq!(direction_sign(false), -1.0);
}

#[test]
fn twitch_type_helpers_split_angle_p_and_heli() {
    assert!(twitch_is_angle_p(TuneType::AnglePUp));
    assert!(twitch_is_angle_p(TuneType::AnglePDown));
    assert!(!twitch_is_angle_p(TuneType::RateDUp));
    assert!(twitch_is_heli_only(TuneType::RateFfUp));
    assert!(twitch_is_heli_only(TuneType::MaxGains));
    assert!(twitch_is_heli_only(TuneType::TuneCheck));
    assert!(!twitch_is_heli_only(TuneType::RatePUp));
}

#[test]
fn twitch_lean_angle_uses_dir_sign_and_start() {
    let lean = twitch_lean_angle_cd(AxisType::Roll, 1.0, cd_to_rad(500.0), 0.0, 0.0, 100.0);
    assert_eq!(lean, 400.0);

    let neg = twitch_lean_angle_cd(AxisType::Pitch, -1.0, 0.0, cd_to_rad(300.0), 0.0, 0.0);
    assert_eq!(neg, -300.0);
}

#[test]
fn twitch_lean_angle_yaw_wraps_180_cd() {
    let lean = twitch_lean_angle_cd(AxisType::Yaw, 1.0, 0.0, 0.0, cd_to_rad(19000.0), 0.0);
    assert_eq!(lean, -17000.0);
}

#[test]
fn twitching_rate_stays_running_under_target() {
    let out = twitching_test_rate(
        100.0,
        1_000.0,
        AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
        0.0,
        0.0,
        0.0,
        10_000,
        9_800,
        AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
        AUTOTUNE_AGGR_DEFAULT,
    );
    assert!(!out.done);
    assert_eq!(out.meas_rate_max, 1_000.0);
    assert_eq!(out.meas_rate_min, 1_000.0);
    assert_eq!(out.meas_angle_min, 100.0);
    assert_eq!(out.step_timeout_ms, 600);
}

#[test]
fn twitching_rate_done_when_rate_exceeds_target() {
    let out = twitching_test_rate(
        200.0,
        18_001.0,
        AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
        0.0,
        0.0,
        0.0,
        10_000,
        9_800,
        AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
        AUTOTUNE_AGGR_DEFAULT,
    );
    assert!(out.done);
    assert_eq!(out.meas_rate_max, 18_001.0);
}

#[test]
fn twitching_rate_done_on_bounce_back() {
    let out = twitching_test_rate(
        150.0,
        4_000.0,
        10_000.0,
        8_000.0,
        8_000.0,
        200.0,
        10_000,
        9_800,
        AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
        0.1,
    );
    assert!(out.done);
    assert_eq!(out.meas_rate_min, 4_000.0);
    assert_eq!(out.meas_angle_min, 150.0);
}

#[test]
fn twitching_rate_done_on_timeout() {
    let out = twitching_test_rate(
        0.0,
        12_000.0,
        AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
        12_000.0,
        12_000.0,
        0.0,
        12_000,
        10_000,
        2_000,
        AUTOTUNE_AGGR_DEFAULT,
    );
    assert!(out.done);
    assert_eq!(out.step_timeout_ms, 2_000);
}

#[test]
fn twitching_abort_ignores_angle_below_max() {
    let out = twitching_abort_rate(1_999.0, 10.0, 2_000.0, 10.0, 0.0, 1.0);
    assert!(out.tick.is_none());
    assert_eq!(out.step_scaler, 1.0);
}

#[test]
fn twitching_abort_shrinks_scaler_when_rate_still_at_min() {
    let out = twitching_abort_rate(2_000.0, 10.0, 2_000.0, 10.0, 0.0, 1.0);
    assert_eq!(out.tick, Some(TwitchTick::Aborted));
    assert!((out.step_scaler - 0.9).abs() < f32::EPSILON);
    assert!(!out.failed);
}

#[test]
fn twitching_abort_fails_when_scaler_at_floor() {
    let out = twitching_abort_rate(2_000.0, 10.0, 2_000.0, 10.0, 0.0, 0.2);
    assert_eq!(out.tick, Some(TwitchTick::Aborted));
    assert!(out.failed);
    assert!(out.reached_limit);
    assert_eq!(out.step_scaler, 0.2);
}

#[test]
fn twitching_abort_completes_when_bounce_already_measured() {
    let out = twitching_abort_rate(2_000.0, 4_000.0, 2_000.0, 1_000.0, 100.0, 1.0);
    assert_eq!(out.tick, Some(TwitchTick::Done));
    assert!(!out.failed);
}

#[test]
fn twitching_angle_done_when_angle_exceeds_target() {
    let out = twitching_test_angle(
        2_100.0,
        100.0,
        2_000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10_000,
        9_800,
        AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
        AUTOTUNE_AGGR_DEFAULT,
    );
    assert!(out.done);
    assert_eq!(out.meas_angle_max, 2_100.0);
}

#[test]
fn twitching_accel_updates_only_on_new_max() {
    let first = twitching_measure_acceleration(0.0, 200.0, 0.0, 10_200, 10_000);
    assert_eq!(first.rate_max, 200.0);
    assert!((first.accel_average - 1_000.0).abs() < 1e-3);

    let held =
        twitching_measure_acceleration(first.accel_average, 150.0, first.rate_max, 10_400, 10_000);
    assert_eq!(held.rate_max, 200.0);
    assert_eq!(held.accel_average, first.accel_average);
}

#[test]
fn test_run_rate_d_up_commands_rate_step() {
    let view = AutoTuneTwitchView::typical();
    let out = autotune_test_run(&view);
    assert_eq!(out.tick, TwitchTick::Running);
    assert!(out.input_rate_step);
    assert!(!out.input_angle_step);
    assert_eq!(out.lean_angle, 0.0);
    assert_eq!(out.step_timeout_ms, 600);
}

#[test]
fn test_run_rate_p_up_uses_aggressiveness_target() {
    let mut view = AutoTuneTwitchView::typical();
    view.tune_type = TuneType::RatePUp;
    view.rotation_rate = view.target_rate + 1.0;
    let under = autotune_test_run(&view);
    assert_eq!(under.tick, TwitchTick::Running);

    view.rotation_rate = view.target_rate * (1.0 + 0.5 * view.aggressiveness) + 1.0;
    let over = autotune_test_run(&view);
    assert_eq!(over.tick, TwitchTick::Done);
}

#[test]
fn test_run_angle_p_steps_once_then_holds() {
    let mut view = AutoTuneTwitchView::typical();
    view.tune_type = TuneType::AnglePUp;
    let first = autotune_test_run(&view);
    assert!(first.input_angle_step);
    assert!(!first.input_rate_hold);
    assert!(first.angle_step_commanded);

    view.angle_step_commanded = true;
    let hold = autotune_test_run(&view);
    assert!(!hold.input_angle_step);
    assert!(hold.input_rate_hold);
}

#[test]
fn test_run_heli_type_is_flow_of_control() {
    let mut view = AutoTuneTwitchView::typical();
    view.tune_type = TuneType::MaxGains;
    let out = autotune_test_run(&view);
    assert!(out.flow_of_control);
    assert_eq!(out.tick, TwitchTick::Running);
    assert!(!out.input_rate_step);
}

#[test]
fn test_run_abort_overwrites_rate_done() {
    let mut view = AutoTuneTwitchView::typical();
    view.rotation_rate = view.target_rate + 1.0;
    view.roll_rad = cd_to_rad(view.angle_abort);
    let out = autotune_test_run(&view);
    assert_eq!(out.tick, TwitchTick::Aborted);
    assert!((out.step_scaler - 0.9).abs() < f32::EPSILON);
}

#[test]
fn executing_test_propagates_twitch_timeout() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::ExecutingTest;
    let out = mode_autotune_run(&view);
    assert_eq!(out.step_timeout_ms, 600);
    assert_eq!(out.lean_angle, 0.0);
}
