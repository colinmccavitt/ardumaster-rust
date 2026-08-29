//! `ModeAutoTune` init leftover, upstream `ArduCopter/mode_autotune.cpp`.

use ap_copter::mode_autotune::{
    allows_autotune, autotune_has_user_takeoff, autotune_mode_flags, autotune_use_poshold,
    first_enabled_axis, mode_autotune_init, pitch_enabled, roll_enabled, yaw_d_enabled,
    yaw_enabled, AutoTuneInitFail, AutoTuneInitView, AxisType, Step, TuneMode,
    AUTOTUNE_AXIS_BITMASK_DEFAULT, AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_YAW,
    AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_STARTED, AUTOTUNE_MESSAGE_TESTING,
    AUTOTUNE_SUCCESS_COUNT, MODE_NUMBER_ALT_HOLD, MODE_NUMBER_AUTOTUNE, MODE_NUMBER_STABILIZE,
};
use ap_copter::mode_loiter::MODE_NUMBER_LOITER;
use ap_copter::mode_poshold::MODE_NUMBER_POSHOLD;

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
