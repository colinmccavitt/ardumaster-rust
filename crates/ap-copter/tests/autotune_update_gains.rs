//! Multi `Step::UPDATE_GAINS` leftover, upstream `AC_AutoTune_Multi`.

use ap_copter::autotune_update_gains::{
    autotune_update_gains, rate_d_limits, rate_p_fail_min_d, updating_angle_p_down,
    updating_angle_p_up, updating_rate_d_down, updating_rate_d_up, updating_rate_p_up_d_down,
    UpdateGainsView, AUTOTUNE_D_UP_DOWN_MARGIN, AUTOTUNE_MIN_D_DEFAULT, AUTOTUNE_RD_MAX,
    AUTOTUNE_RD_STEP, AUTOTUNE_RLPF_MAX, AUTOTUNE_RLPF_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_MIN,
    AUTOTUNE_RP_STEP, AUTOTUNE_SP_MAX, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, AUTOTUNE_TUNE_D_DEFAULT,
    AUTOTUNE_TUNE_P_DEFAULT, AUTOTUNE_TUNE_SP_DEFAULT,
};
use ap_copter::mode_autotune::{
    mode_autotune_run, AutoTuneRunView, AxisType, Step, TuneMode, TuneType, AUTOTUNE_AGGR_DEFAULT,
    AUTOTUNE_REQUIRED_LEVEL_TIME_MS, AUTOTUNE_SUCCESS_COUNT, AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
};

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn copter_sp_max_is_not_plane() {
    assert_eq!(AUTOTUNE_SP_MAX, 40.0);
    assert_eq!(AUTOTUNE_SP_MIN, 0.5);
    assert_eq!(AUTOTUNE_RD_STEP, 0.05);
    assert_eq!(AUTOTUNE_RP_STEP, 0.05);
    assert_eq!(AUTOTUNE_SP_STEP, 0.05);
    assert_eq!(AUTOTUNE_RP_MIN, 0.01);
    assert_eq!(AUTOTUNE_RP_MAX, 2.0);
    assert_eq!(AUTOTUNE_RD_MAX, 0.200);
    assert_eq!(AUTOTUNE_D_UP_DOWN_MARGIN, 0.2);
    assert_eq!(AUTOTUNE_MIN_D_DEFAULT, 0.0005);
}

#[test]
fn yaw_rate_d_uses_rlpf_limits() {
    assert_eq!(
        rate_d_limits(AxisType::Yaw, AUTOTUNE_MIN_D_DEFAULT),
        (AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX)
    );
    assert_eq!(
        rate_d_limits(AxisType::Roll, AUTOTUNE_MIN_D_DEFAULT),
        (AUTOTUNE_MIN_D_DEFAULT, AUTOTUNE_RD_MAX)
    );
    assert_eq!(
        rate_d_limits(AxisType::YawD, AUTOTUNE_MIN_D_DEFAULT),
        (AUTOTUNE_MIN_D_DEFAULT, AUTOTUNE_RD_MAX)
    );
    assert!(!rate_p_fail_min_d(AxisType::Yaw));
    assert!(rate_p_fail_min_d(AxisType::Roll));
    assert!(rate_p_fail_min_d(AxisType::YawD));
}

#[test]
fn rate_d_up_raises_p_when_peak_is_low() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = 1_000.0;
    view.target_rate = AUTOTUNE_TARGET_RATE_RLLPIT_CDS;
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_P_DEFAULT * (1.0 + AUTOTUNE_RP_STEP),
    );
    almost(out.tune_d, AUTOTUNE_TUNE_D_DEFAULT);
    assert_eq!(out.success_counter, 0);
    assert!(!out.reached_limit);
}

#[test]
fn rate_d_up_clamps_p_max_and_flags_limit() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_RP_MAX;
    view.meas_rate_max = 1_000.0;
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(out.tune_p, AUTOTUNE_RP_MAX);
    assert!(out.reached_limit);
    assert_eq!(out.success_counter, 0);
}

#[test]
fn rate_d_up_lowers_p_when_peak_overshoots() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = view.target_rate + 1.0;
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_P_DEFAULT * (1.0 - AUTOTUNE_RP_STEP),
    );
    almost(out.tune_d, AUTOTUNE_TUNE_D_DEFAULT);
}

#[test]
fn rate_d_up_cuts_d_once_p_is_at_min() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_RP_MIN * 0.5;
    view.meas_rate_max = view.target_rate + 1.0;
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(out.tune_p, AUTOTUNE_RP_MIN);
    almost(
        out.tune_d,
        AUTOTUNE_TUNE_D_DEFAULT * (1.0 - AUTOTUNE_RD_STEP),
    );
    assert!(!out.min_rate_d_limit);
}

#[test]
fn rate_d_up_min_d_completes_tune_type() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_RP_MIN * 0.5;
    view.tune_d = view.min_d;
    view.meas_rate_max = view.target_rate + 1.0;
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(out.tune_d, view.min_d);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.reached_limit);
    assert!(out.min_rate_d_limit);
    assert!(out.tune_type_complete);
}

#[test]
fn rate_d_up_bounce_increments_success() {
    let mut view = UpdateGainsView::typical();
    // In the D-tune window: peak just under target, bounce above AGGR.
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 2.0 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    assert_eq!(out.success_counter, 1);
    assert!(out.ignore_next);
    almost(out.tune_d, AUTOTUNE_TUNE_D_DEFAULT);
}

#[test]
fn rate_d_up_small_bounce_raises_d() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 0.5 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(
        out.tune_d,
        AUTOTUNE_TUNE_D_DEFAULT * (1.0 + AUTOTUNE_RD_STEP * 2.0),
    );
    assert_eq!(out.success_counter, 0);
}

#[test]
fn rate_d_up_ignore_next_skips_d_raise() {
    let mut view = UpdateGainsView::typical();
    view.ignore_next = true;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 0.5 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(out.tune_d, AUTOTUNE_TUNE_D_DEFAULT);
    assert!(!out.ignore_next);
}

#[test]
fn rate_d_up_hits_d_max() {
    let mut view = UpdateGainsView::typical();
    view.tune_d = AUTOTUNE_RD_MAX;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 0.5 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(out.tune_d, AUTOTUNE_RD_MAX);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.reached_limit);
}

#[test]
fn rate_d_down_success_when_bounce_is_small() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::RateDDown;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 0.5 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_down(&view, view.min_d);
    assert_eq!(out.success_counter, 1);
    almost(out.tune_d, AUTOTUNE_TUNE_D_DEFAULT);
}

#[test]
fn rate_d_down_cuts_d_when_bounce_is_large() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 2.0 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_down(&view, view.min_d);
    almost(
        out.tune_d,
        AUTOTUNE_TUNE_D_DEFAULT * (1.0 - AUTOTUNE_RD_STEP),
    );
    assert!(out.ignore_next);
    assert_eq!(out.success_counter, 0);
}

#[test]
fn rate_d_down_min_d_completes() {
    let mut view = UpdateGainsView::typical();
    view.tune_d = view.min_d;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 2.0 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_d_down(&view, view.min_d);
    almost(out.tune_d, view.min_d);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.min_rate_d_limit);
}

#[test]
fn rate_p_up_success_when_peak_overshoots_aggr() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::RatePUp;
    view.meas_rate_max = view.target_rate * (1.0 + AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_p_up_d_down(&view, view.min_d, true);
    assert_eq!(out.success_counter, 1);
    assert!(out.ignore_next);
    almost(out.tune_p, AUTOTUNE_TUNE_P_DEFAULT);
}

#[test]
fn rate_p_up_raises_p_when_peak_is_low() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = view.target_rate * 0.5;
    let out = updating_rate_p_up_d_down(&view, view.min_d, true);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_P_DEFAULT * (1.0 + AUTOTUNE_RP_STEP),
    );
}

#[test]
fn rate_p_up_max_p_completes() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_RP_MAX;
    view.meas_rate_max = view.target_rate * 0.5;
    let out = updating_rate_p_up_d_down(&view, view.min_d, true);
    almost(out.tune_p, AUTOTUNE_RP_MAX);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.reached_limit);
}

#[test]
fn rate_p_up_cuts_d_on_bounce_in_window() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 2.0 * AUTOTUNE_AGGR_DEFAULT);
    let out = updating_rate_p_up_d_down(&view, view.min_d, true);
    almost(
        out.tune_d,
        AUTOTUNE_TUNE_D_DEFAULT * (1.0 - AUTOTUNE_RD_STEP),
    );
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_P_DEFAULT * (1.0 - AUTOTUNE_RP_STEP),
    );
}

#[test]
fn rate_p_up_fail_min_d_trips_failed() {
    let mut view = UpdateGainsView::typical();
    view.tune_d = view.min_d * 1.01;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 2.0 * AUTOTUNE_AGGR_DEFAULT);
    let fail = updating_rate_p_up_d_down(&view, view.min_d, true);
    assert!(fail.failed);
    assert!(fail.rate_d_failed);
    assert_eq!(fail.mode, TuneMode::Failed);

    let yaw = updating_rate_p_up_d_down(&view, view.min_d, false);
    assert!(!yaw.rate_d_failed);
    almost(yaw.tune_d, view.min_d);
}

#[test]
fn angle_p_down_success_when_under_target() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::AnglePDown;
    view.tune_p = AUTOTUNE_TUNE_SP_DEFAULT;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 1_800.0;
    let out = updating_angle_p_down(&view);
    assert_eq!(out.success_counter, 1);
    almost(out.tune_p, AUTOTUNE_TUNE_SP_DEFAULT);
}

#[test]
fn angle_p_down_cuts_p_when_overshooting() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_TUNE_SP_DEFAULT;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 2_200.0;
    let out = updating_angle_p_down(&view);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_SP_DEFAULT * (1.0 - AUTOTUNE_SP_STEP),
    );
    assert!(out.ignore_next);
}

#[test]
fn angle_p_down_min_p_fails() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_SP_MIN;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 2_200.0;
    let out = updating_angle_p_down(&view);
    almost(out.tune_p, AUTOTUNE_SP_MIN);
    assert!(out.failed);
    assert!(out.angle_p_failed);
    assert_eq!(out.mode, TuneMode::Failed);
}

#[test]
fn angle_p_up_success_when_overshooting() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.tune_p = AUTOTUNE_TUNE_SP_DEFAULT;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 2_200.0;
    let out = updating_angle_p_up(&view);
    assert_eq!(out.success_counter, 1);
    assert!(out.ignore_next);
}

#[test]
fn angle_p_up_success_on_rate_bounce() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_TUNE_SP_DEFAULT;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 2_050.0;
    view.meas_rate_max = 1_000.0;
    view.meas_rate_min = -200.0;
    let out = updating_angle_p_up(&view);
    assert_eq!(out.success_counter, 1);
}

#[test]
fn angle_p_up_raises_p_when_short() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_TUNE_SP_DEFAULT;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 1_000.0;
    let out = updating_angle_p_up(&view);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_SP_DEFAULT * (1.0 + AUTOTUNE_SP_STEP),
    );
}

#[test]
fn angle_p_up_max_p_completes() {
    let mut view = UpdateGainsView::typical();
    view.tune_p = AUTOTUNE_SP_MAX;
    view.target_angle = 2_000.0;
    view.meas_angle_max = 1_000.0;
    let out = updating_angle_p_up(&view);
    almost(out.tune_p, AUTOTUNE_SP_MAX);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.reached_limit);
}

#[test]
fn dispatcher_heli_types_are_flow_of_control() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::RateFfUp;
    let ff = autotune_update_gains(&view);
    assert!(ff.flow_of_control);
    almost(ff.tune_p, view.tune_p);

    view.tune_type = TuneType::MaxGains;
    assert!(autotune_update_gains(&view).flow_of_control);
}

#[test]
fn dispatcher_tune_check_forces_success_count() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::TuneCheck;
    let out = autotune_update_gains(&view);
    assert_eq!(out.success_counter, AUTOTUNE_SUCCESS_COUNT as i8);
    assert!(out.tune_type_complete);
    assert!(!out.flow_of_control);
}

#[test]
fn dispatcher_tune_complete_is_noop() {
    let mut view = UpdateGainsView::typical();
    view.tune_type = TuneType::TuneComplete;
    view.success_counter = 2;
    let out = autotune_update_gains(&view);
    assert_eq!(out.success_counter, 2);
    assert!(!out.tune_type_complete);
    almost(out.tune_p, view.tune_p);
}

#[test]
fn dispatcher_rate_d_up_matches_direct() {
    let mut view = UpdateGainsView::typical();
    view.meas_rate_max = 1_000.0;
    let via = autotune_update_gains(&view);
    let direct = updating_rate_d_up(&view, view.min_d, AUTOTUNE_RD_MAX);
    almost(via.tune_p, direct.tune_p);
    almost(via.tune_d, direct.tune_d);
}

#[test]
fn dispatcher_yaw_rate_d_uses_rlpf_max() {
    let mut view = UpdateGainsView::typical();
    view.axis = AxisType::Yaw;
    view.tune_d = AUTOTUNE_RLPF_MAX;
    view.meas_rate_max = view.target_rate * 0.95;
    view.meas_rate_min = view.meas_rate_max * (1.0 - 0.5 * AUTOTUNE_AGGR_DEFAULT);
    let out = autotune_update_gains(&view);
    almost(out.tune_d, AUTOTUNE_RLPF_MAX);
    assert!(out.reached_limit);
}

#[test]
fn run_update_gains_applies_math_then_aborts() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.tune_type = TuneType::RateDUp;
    view.test_rate_max = 1_000.0;
    view.positive_direction = true;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains);
    almost(
        out.tune_p,
        AUTOTUNE_TUNE_P_DEFAULT * (1.0 + AUTOTUNE_RP_STEP),
    );
    assert_eq!(out.step, Step::WaitingForLevel);
    assert_eq!(out.step_timeout_ms, AUTOTUNE_REQUIRED_LEVEL_TIME_MS);
    assert!(!out.positive_direction);
    assert!(!out.update_gains_failed);
}

#[test]
fn run_update_gains_can_fail_the_tune() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.tune_type = TuneType::AnglePDown;
    view.tune_p = AUTOTUNE_SP_MIN;
    view.target_angle = 2_000.0;
    view.test_angle_max = 2_200.0;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains);
    assert!(out.update_gains_failed);
    assert_eq!(out.mode, TuneMode::Failed);
    assert_eq!(out.step, Step::WaitingForLevel);
}

#[test]
fn run_update_gains_heli_type_is_flow_of_control() {
    let mut view = AutoTuneRunView::typical();
    view.step = Step::UpdateGains;
    view.tune_type = TuneType::RateFfUp;
    let out = mode_autotune_run(&view);
    assert!(out.update_gains_flow_of_control);
    almost(out.tune_p, view.tune_p);
}
