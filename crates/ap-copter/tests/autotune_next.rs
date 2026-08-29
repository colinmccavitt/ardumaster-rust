//! Multi `next_tune_type` / next-axis / backoff leftover, upstream `AC_AutoTune`.

use ap_copter::autotune_next::{
    autotune_advance, axis_complete_bit, multi_tune_sequence, next_axis, next_tune_type,
    set_tuning_gains_with_backoff, AdvanceView, BackoffView, AUTOTUNE_ACCEL_RP_BACKOFF,
    AUTOTUNE_ACCEL_Y_BACKOFF, AUTOTUNE_GAIN_BACKOFF_DEFAULT, AUTOTUNE_GAIN_BACKOFF_MAX,
    AUTOTUNE_RP_ACCEL_MIN, AUTOTUNE_TUNE_SEQ_LEN, AUTOTUNE_Y_ACCEL_MIN,
};
use ap_copter::mode_autotune::{
    mode_autotune_init, mode_autotune_run, AutoTuneInitView, AutoTuneRunView, AxisType, Step,
    TuneMode, TuneType, AUTOTUNE_AGGR_DEFAULT, AUTOTUNE_AXIS_BITMASK_DEFAULT,
    AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_ROLL, AUTOTUNE_AXIS_BITMASK_YAW,
    AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_SUCCESS, AUTOTUNE_SUCCESS_COUNT,
};
use ap_math::scalar::cd_to_rad;

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn multi_sequence_is_not_heli() {
    assert_eq!(AUTOTUNE_TUNE_SEQ_LEN, 6);
    assert_eq!(
        multi_tune_sequence(),
        [
            TuneType::RateDUp,
            TuneType::RateDDown,
            TuneType::RatePUp,
            TuneType::AnglePDown,
            TuneType::AnglePUp,
            TuneType::TuneComplete,
        ]
    );
    assert_eq!(AUTOTUNE_GAIN_BACKOFF_DEFAULT, 0.25);
    assert_eq!(AUTOTUNE_GAIN_BACKOFF_MAX, 0.5);
    assert_eq!(AUTOTUNE_RP_ACCEL_MIN, 4_000.0);
    assert_eq!(AUTOTUNE_Y_ACCEL_MIN, 1_000.0);
    assert_eq!(AUTOTUNE_ACCEL_RP_BACKOFF, 1.0);
    assert_eq!(AUTOTUNE_ACCEL_Y_BACKOFF, 1.0);
}

#[test]
fn next_tune_type_reset_starts_rate_d_up() {
    let out = next_tune_type(TuneType::AnglePUp, true, 4);
    assert_eq!(out.tune_type, TuneType::RateDUp);
    assert_eq!(out.tune_seq_index, 0);
    assert!(out.sequence_reset);
}

#[test]
fn next_tune_type_walks_multi_sequence() {
    let mut typ = TuneType::RateDUp;
    let mut idx = 0u8;
    for expect in [
        TuneType::RateDDown,
        TuneType::RatePUp,
        TuneType::AnglePDown,
        TuneType::AnglePUp,
        TuneType::TuneComplete,
    ] {
        let out = next_tune_type(typ, false, idx);
        assert_eq!(out.tune_type, expect);
        assert_eq!(out.tune_seq_index, idx + 1);
        assert!(!out.sequence_reset);
        typ = out.tune_type;
        idx = out.tune_seq_index;
    }
}

#[test]
fn next_tune_type_leaves_complete_without_reset() {
    let out = next_tune_type(TuneType::TuneComplete, false, 5);
    assert_eq!(out.tune_type, TuneType::TuneComplete);
    assert_eq!(out.tune_seq_index, 5);
    assert!(!out.sequence_reset);
}

#[test]
fn next_axis_default_mask_roll_to_pitch() {
    let out = next_axis(AxisType::Roll, AUTOTUNE_AXIS_BITMASK_DEFAULT, 0);
    assert_eq!(out.axis, AxisType::Pitch);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_ROLL);
    assert!(!out.complete);
}

#[test]
fn next_axis_skips_disabled_pitch() {
    let mask = AUTOTUNE_AXIS_BITMASK_ROLL | AUTOTUNE_AXIS_BITMASK_YAW;
    let out = next_axis(AxisType::Roll, mask, 0);
    assert_eq!(out.axis, AxisType::Yaw);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_ROLL);
    assert!(!out.complete);
}

#[test]
fn next_axis_yaw_to_yaw_d() {
    let mask = AUTOTUNE_AXIS_BITMASK_YAW | AUTOTUNE_AXIS_BITMASK_YAW_D;
    let out = next_axis(AxisType::Yaw, mask, AUTOTUNE_AXIS_BITMASK_YAW);
    assert_eq!(out.axis, AxisType::YawD);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_YAW);
    assert!(!out.complete);
}

#[test]
fn next_axis_last_axis_finishes() {
    let out = next_axis(AxisType::Yaw, AUTOTUNE_AXIS_BITMASK_YAW, 0);
    assert_eq!(out.axis, AxisType::Yaw);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_YAW);
    assert!(out.complete);
}

#[test]
fn next_axis_yaw_d_always_finishes() {
    let out = next_axis(AxisType::YawD, 0xFF, 0);
    assert_eq!(out.axis, AxisType::YawD);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_YAW_D);
    assert!(out.complete);
}

#[test]
fn axis_complete_bits_match_mask() {
    assert_eq!(axis_complete_bit(AxisType::Roll), AUTOTUNE_AXIS_BITMASK_ROLL);
    assert_eq!(
        axis_complete_bit(AxisType::Pitch),
        AUTOTUNE_AXIS_BITMASK_PITCH
    );
    assert_eq!(axis_complete_bit(AxisType::Yaw), AUTOTUNE_AXIS_BITMASK_YAW);
    assert_eq!(axis_complete_bit(AxisType::YawD), AUTOTUNE_AXIS_BITMASK_YAW_D);
}

#[test]
fn backoff_rate_p_up_scales_roll_p_and_d() {
    let out = set_tuning_gains_with_backoff(&BackoffView::typical());
    almost(out.tune_p, 0.15 * 0.75);
    almost(out.tune_d, 0.004 * 0.75);
    assert!(out.applied);
    assert!(!out.flow_of_control);
    almost(out.gain_backoff, AUTOTUNE_GAIN_BACKOFF_DEFAULT);
}

#[test]
fn backoff_rate_p_up_yaw_scales_only_p() {
    let mut view = BackoffView::typical();
    view.axis = AxisType::Yaw;
    let out = set_tuning_gains_with_backoff(&view);
    almost(out.tune_p, 0.15 * 0.75);
    almost(out.tune_d, 0.004);
    assert!(out.applied);
}

#[test]
fn backoff_rate_d_up_is_noop() {
    let mut view = BackoffView::typical();
    view.tune_type = TuneType::RateDUp;
    let out = set_tuning_gains_with_backoff(&view);
    almost(out.tune_p, view.tune_p);
    almost(out.tune_d, view.tune_d);
    assert!(!out.applied);
}

#[test]
fn backoff_angle_p_up_scales_sp_and_sets_accel() {
    let mut view = BackoffView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.tune_p = 4.5;
    view.test_accel_max_cdss = 2_000.0;
    let out = set_tuning_gains_with_backoff(&view);
    almost(out.tune_p, 4.5 * 0.75 * (1.0 - AUTOTUNE_AGGR_DEFAULT));
    almost(out.tune_d, view.tune_d);
    almost(out.tune_accel_radss, cd_to_rad(AUTOTUNE_RP_ACCEL_MIN));
    assert!(out.applied);
}

#[test]
fn backoff_angle_p_up_yaw_uses_yaw_accel_min() {
    let mut view = BackoffView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.axis = AxisType::Yaw;
    view.tune_p = 4.5;
    view.test_accel_max_cdss = 500.0;
    let out = set_tuning_gains_with_backoff(&view);
    almost(out.tune_accel_radss, cd_to_rad(AUTOTUNE_Y_ACCEL_MIN));
}

#[test]
fn backoff_angle_p_up_uses_measured_accel_when_above_min() {
    let mut view = BackoffView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.test_accel_max_cdss = 8_000.0;
    let out = set_tuning_gains_with_backoff(&view);
    almost(
        out.tune_accel_radss,
        cd_to_rad(8_000.0 * AUTOTUNE_ACCEL_RP_BACKOFF),
    );
}

#[test]
fn backoff_clamps_gain_backoff() {
    let mut view = BackoffView::typical();
    view.gain_backoff = 0.9;
    let out = set_tuning_gains_with_backoff(&view);
    almost(out.gain_backoff, AUTOTUNE_GAIN_BACKOFF_MAX);
    almost(out.tune_p, 0.15 * 0.5);
}

#[test]
fn backoff_heli_type_is_flow_of_control() {
    for typ in [TuneType::RateFfUp, TuneType::MaxGains, TuneType::TuneCheck] {
        let mut view = BackoffView::typical();
        view.tune_type = typ;
        let out = set_tuning_gains_with_backoff(&view);
        assert!(out.flow_of_control, "{typ:?}");
        assert!(!out.applied);
        almost(out.tune_p, view.tune_p);
    }
}

#[test]
fn advance_rate_d_up_goes_to_rate_d_down() {
    let out = autotune_advance(&AdvanceView::typical());
    assert_eq!(out.tune_type, TuneType::RateDDown);
    assert_eq!(out.tune_seq_index, 1);
    assert_eq!(out.success_counter, 0);
    almost(out.step_scaler, 1.0);
    assert_eq!(out.axis, AxisType::Roll);
    assert_eq!(out.axes_completed, 0);
    assert!(!out.backoff_applied);
    assert!(!out.reported_final_gains);
    assert!(!out.next_axis);
    assert!(!out.complete);
    assert_eq!(out.mode, TuneMode::Tuning);
}

#[test]
fn advance_angle_p_up_moves_to_next_axis() {
    let mut view = AdvanceView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.tune_seq_index = 4;
    view.tune_p = 4.5;
    let out = autotune_advance(&view);
    assert_eq!(out.tune_type, TuneType::RateDUp);
    assert_eq!(out.tune_seq_index, 0);
    assert_eq!(out.axis, AxisType::Pitch);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_ROLL);
    assert!(out.reported_final_gains);
    assert!(out.next_axis);
    assert!(!out.complete);
    assert!(out.backoff_applied);
    almost(out.tune_p, 4.5 * 0.75 * (1.0 - AUTOTUNE_AGGR_DEFAULT));
    assert_eq!(out.mode, TuneMode::Tuning);
}

#[test]
fn advance_last_axis_finishes() {
    let mut view = AdvanceView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.tune_seq_index = 4;
    view.axis = AxisType::Yaw;
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW;
    view.axes_completed = 0;
    let out = autotune_advance(&view);
    assert_eq!(out.tune_type, TuneType::RateDUp);
    assert_eq!(out.axis, AxisType::Yaw);
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_YAW);
    assert!(out.complete);
    assert!(!out.next_axis);
    assert_eq!(out.mode, TuneMode::Finished);
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_SUCCESS));
    assert!(out.autotune_complete);
    assert!(out.loaded_gains.is_some());
}

#[test]
fn advance_incomplete_is_noop() {
    let mut view = AdvanceView::typical();
    view.tune_type_complete = false;
    let out = autotune_advance(&view);
    assert_eq!(out.tune_type, TuneType::RateDUp);
    assert_eq!(out.tune_seq_index, 0);
    assert!(!out.next_axis);
    assert!(!out.complete);
}

#[test]
fn init_first_start_seats_rate_d_up() {
    let out = mode_autotune_init(false, &AutoTuneInitView::typical());
    assert!(out.ok);
    assert!(out.backup_gains);
    assert_eq!(out.tune_type, Some(TuneType::RateDUp));
    assert_eq!(out.tune_seq_index, Some(0));
}

#[test]
fn init_resume_does_not_reset_sequence() {
    let mut view = AutoTuneInitView::typical();
    view.mode = TuneMode::Tuning;
    let out = mode_autotune_init(false, &view);
    assert!(out.ok);
    assert!(!out.backup_gains);
    assert!(out.tune_type.is_none());
    assert!(out.tune_seq_index.is_none());
}

#[test]
fn run_update_gains_complete_advances_type() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.tune_type = TuneType::RateDUp;
    view.tune_seq_index = 0;
    view.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
    view.test_rate_max = 1_000.0;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains);
    assert!(out.update_gains_complete);
    assert!(out.sequencing);
    assert_eq!(out.tune_type, TuneType::RateDDown);
    assert_eq!(out.tune_seq_index, 1);
    assert_eq!(out.success_counter, 0);
    almost(out.step_scaler, 1.0);
    assert_eq!(out.axis, AxisType::Roll);
    assert_eq!(out.mode, TuneMode::Tuning);
    assert_eq!(out.step, Step::WaitingForLevel);
}

#[test]
fn run_last_type_on_last_axis_finishes() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.tune_type = TuneType::AnglePUp;
    view.tune_seq_index = 4;
    view.axis = AxisType::Yaw;
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW;
    view.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
    view.tune_p = 4.5;
    view.target_angle = 2_000.0;
    view.test_angle_max = 2_200.0;
    view.test_rate_min = 0.0;
    view.test_rate_max = 100.0;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains_complete);
    assert!(out.sequencing);
    assert!(out.autotune_complete);
    assert!(!out.next_axis);
    assert_eq!(out.mode, TuneMode::Finished);
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_SUCCESS));
    assert_eq!(out.axes_completed, AUTOTUNE_AXIS_BITMASK_YAW);
    assert_eq!(out.step, Step::WaitingForLevel);
}
