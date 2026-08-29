//! Multi load / save leftover, upstream `AC_AutoTune_Multi`.

use ap_copter::autotune_load_save::{
    autotune_disarmed, autotune_stop, backup_multi_gains, constrain_aggressiveness, load_gains,
    load_gains_already, load_intra_test_gains, load_orig_gains, load_test_gains, load_tuned_gains,
    save_tuning_gains, BackupView, DisarmAction, LiveAxis, LoadView, AUTOTUNE_AGGR_MAX,
    AUTOTUNE_AGGR_MIN, AUTOTUNE_FLTE_MIN, AUTOTUNE_LIVE_ACCEL_DEFAULT, AUTOTUNE_LIVE_FLTE_DEFAULT,
    AUTOTUNE_LIVE_FLTT_DEFAULT, AUTOTUNE_LIVE_RD_DEFAULT, AUTOTUNE_LIVE_RI_DEFAULT,
    AUTOTUNE_LIVE_RP_DEFAULT, AUTOTUNE_LIVE_SP_DEFAULT, AUTOTUNE_PI_RATIO_FINAL,
    AUTOTUNE_PI_RATIO_FOR_TESTING, AUTOTUNE_TEST_I_RATIO, AUTOTUNE_YAW_PI_RATIO_FINAL,
};
use ap_copter::autotune_update_gains::AUTOTUNE_MIN_D_DEFAULT;
use ap_copter::mode_autotune::{
    AxisType, GainType, AUTOTUNE_AGGR_DEFAULT, AUTOTUNE_AXIS_BITMASK_DEFAULT,
    AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_ROLL, AUTOTUNE_AXIS_BITMASK_YAW,
    AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_SAVED_GAINS, AUTOTUNE_MESSAGE_STOPPED,
};

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn multi_ratios_are_not_heli() {
    assert_eq!(AUTOTUNE_PI_RATIO_FOR_TESTING, 0.1);
    assert_eq!(AUTOTUNE_PI_RATIO_FINAL, 1.0);
    assert_eq!(AUTOTUNE_YAW_PI_RATIO_FINAL, 0.1);
    assert_eq!(AUTOTUNE_FLTE_MIN, 2.5);
    assert_eq!(AUTOTUNE_TEST_I_RATIO, 0.01);
    assert_eq!(AUTOTUNE_AGGR_MIN, 0.05);
    assert_eq!(AUTOTUNE_AGGR_MAX, 0.2);
}

#[test]
fn aggressiveness_constrain_matches_backup() {
    almost(constrain_aggressiveness(0.01), AUTOTUNE_AGGR_MIN);
    almost(constrain_aggressiveness(0.5), AUTOTUNE_AGGR_MAX);
    almost(
        constrain_aggressiveness(AUTOTUNE_AGGR_DEFAULT),
        AUTOTUNE_AGGR_DEFAULT,
    );
}

#[test]
fn backup_copies_live_into_orig_and_tune() {
    let out = backup_multi_gains(&BackupView::typical());
    almost(out.aggressiveness, AUTOTUNE_AGGR_DEFAULT);
    assert!(out.orig_bf_feedforward);
    almost(out.orig_roll.rp, AUTOTUNE_LIVE_RP_DEFAULT);
    almost(out.orig_roll.ri, AUTOTUNE_LIVE_RI_DEFAULT);
    almost(out.orig_roll.rd, AUTOTUNE_LIVE_RD_DEFAULT);
    almost(out.orig_roll.sp, AUTOTUNE_LIVE_SP_DEFAULT);
    almost(out.orig_roll.accel_radss, AUTOTUNE_LIVE_ACCEL_DEFAULT);
    almost(out.tune_roll.rp, AUTOTUNE_LIVE_RP_DEFAULT);
    almost(out.tune_roll.rd, AUTOTUNE_LIVE_RD_DEFAULT);
    almost(out.tune_yaw.r_lpf, AUTOTUNE_LIVE_FLTE_DEFAULT);
    almost(out.orig_yaw.r_lpf, AUTOTUNE_LIVE_FLTE_DEFAULT);
    assert!(!out.yaw_rd_seeded);
    assert!(!out.yaw_rlpf_seeded);
}

#[test]
fn backup_seeds_yaw_rd_when_yaw_d_is_zero() {
    let mut view = BackupView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW_D;
    view.yaw.rd = 0.0;
    let out = backup_multi_gains(&view);
    almost(out.tune_yaw.rd, AUTOTUNE_MIN_D_DEFAULT);
    assert!(out.yaw_rd_seeded);
    assert!(!out.yaw_rlpf_seeded);
}

#[test]
fn backup_seeds_yaw_rlpf_when_yaw_filter_is_zero() {
    let mut view = BackupView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW;
    view.yaw.flte = 0.0;
    let out = backup_multi_gains(&view);
    almost(out.tune_yaw.r_lpf, AUTOTUNE_FLTE_MIN);
    almost(out.orig_yaw.r_lpf, 0.0);
    assert!(out.yaw_rlpf_seeded);
    assert!(!out.yaw_rd_seeded);
}

#[test]
fn backup_does_not_seed_yaw_rd_without_yaw_d_bit() {
    let mut view = BackupView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW;
    view.yaw.rd = 0.0;
    let out = backup_multi_gains(&view);
    almost(out.tune_yaw.rd, 0.0);
    assert!(!out.yaw_rd_seeded);
}

#[test]
fn load_orig_restores_all_enabled_axes() {
    let out = load_orig_gains(&LoadView::typical());
    assert!(out.use_sqrt_controller);
    assert_eq!(out.bf_feedforward, Some(true));
    assert!(out.roll.written);
    almost(out.roll.ri, AUTOTUNE_LIVE_RI_DEFAULT);
    almost(out.roll.fltt.unwrap(), AUTOTUNE_LIVE_FLTT_DEFAULT);
    almost(out.roll.accel_radss.unwrap(), AUTOTUNE_LIVE_ACCEL_DEFAULT);
    assert!(out.pitch.written);
    assert!(out.yaw.written);
    almost(out.yaw.flte.unwrap(), AUTOTUNE_LIVE_FLTE_DEFAULT);
    assert!(!out.roll.saved);
}

#[test]
fn load_orig_skips_zero_rp_and_disabled_axes() {
    let mut view = LoadView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_ROLL;
    view.orig_roll.rp = 0.0;
    let out = load_orig_gains(&view);
    assert!(!out.roll.written);
    assert!(!out.pitch.written);
    assert!(!out.yaw.written);
}

#[test]
fn load_tuned_uses_final_i_ratio_and_needs_completed_bit() {
    let mut view = LoadView::typical();
    view.tune_roll.rp = 0.20;
    view.tune_roll.rd = 0.008;
    view.tune_roll.sp = 6.0;
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_ROLL;
    let out = load_tuned_gains(&view);
    assert!(out.use_sqrt_controller);
    assert!(out.roll.written);
    almost(out.roll.rp, 0.20);
    almost(out.roll.ri, 0.20 * AUTOTUNE_PI_RATIO_FINAL);
    almost(out.roll.rd, 0.008);
    almost(out.roll.sp, 6.0);
    assert!(out.roll.fltt.is_none());
    assert!(!out.pitch.written);
    assert!(!out.yaw.written);
}

#[test]
fn load_tuned_yaw_uses_yaw_final_ratio() {
    let mut view = LoadView::typical();
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_YAW;
    view.tune_yaw.rp = 0.30;
    let out = load_tuned_gains(&view);
    assert!(out.yaw.written);
    almost(out.yaw.ri, 0.30 * AUTOTUNE_YAW_PI_RATIO_FINAL);
    assert!(!out.yaw.rd_written);
    almost(out.yaw.flte.unwrap(), AUTOTUNE_LIVE_FLTE_DEFAULT);
}

#[test]
fn load_tuned_yaw_d_writes_d_not_flte() {
    let mut view = LoadView::typical();
    view.axis_bitmask = AUTOTUNE_AXIS_BITMASK_YAW_D;
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_YAW_D;
    view.tune_yaw.rd = 0.007;
    let out = load_tuned_gains(&view);
    assert!(out.yaw.written);
    assert!(out.yaw.rd_written);
    almost(out.yaw.rd, 0.007);
    assert!(out.yaw.flte.is_none());
}

#[test]
fn load_tuned_enables_ff_and_zeros_accel_when_live_ff_off() {
    let mut view = LoadView::typical();
    view.live_bf_feedforward = false;
    view.axes_completed = 0;
    let out = load_tuned_gains(&view);
    assert_eq!(out.bf_feedforward, Some(true));
    assert!(out.accel_roll_forced_zero);
    assert!(out.accel_pitch_forced_zero);
    assert!(!out.roll.written);
}

#[test]
fn load_intra_rewrites_i_and_skips_accel() {
    let out = load_intra_test_gains(&LoadView::typical());
    assert!(out.use_sqrt_controller);
    assert_eq!(out.bf_feedforward, Some(true));
    almost(
        out.roll.ri,
        AUTOTUNE_LIVE_RP_DEFAULT * AUTOTUNE_PI_RATIO_FOR_TESTING,
    );
    almost(out.roll.rp, AUTOTUNE_LIVE_RP_DEFAULT);
    almost(out.roll.rd, AUTOTUNE_LIVE_RD_DEFAULT);
    assert!(out.roll.accel_radss.is_none());
    almost(out.yaw.flte.unwrap(), AUTOTUNE_LIVE_FLTE_DEFAULT);
}

#[test]
fn load_test_roll_zeros_ff_and_filters() {
    let out = load_test_gains(&LoadView::typical());
    assert!(!out.use_sqrt_controller);
    assert!(out.bf_feedforward.is_none());
    assert!(out.roll.written);
    assert!(!out.pitch.written);
    almost(
        out.roll.ri,
        AUTOTUNE_LIVE_RP_DEFAULT * AUTOTUNE_TEST_I_RATIO,
    );
    almost(out.roll.rff, 0.0);
    almost(out.roll.dff, 0.0);
    almost(out.roll.fltt.unwrap(), 0.0);
    almost(out.roll.smax.unwrap(), 0.0);
    assert!(out.roll.accel_radss.is_none());
}

#[test]
fn load_test_yaw_zeros_d_and_writes_rlpf() {
    let mut view = LoadView::typical();
    view.axis = AxisType::Yaw;
    let out = load_test_gains(&view);
    assert!(out.yaw.written);
    almost(out.yaw.rd, 0.0);
    almost(out.yaw.flte.unwrap(), AUTOTUNE_LIVE_FLTE_DEFAULT);
}

#[test]
fn load_test_yaw_d_writes_d_not_flte() {
    let mut view = LoadView::typical();
    view.axis = AxisType::YawD;
    view.tune_yaw.rd = 0.009;
    let out = load_test_gains(&view);
    almost(out.yaw.rd, 0.009);
    assert!(out.yaw.flte.is_none());
}

#[test]
fn load_gains_skips_when_already_loaded() {
    assert!(load_gains_already(GainType::Original, GainType::Original));
    let out = load_gains(GainType::Test, GainType::Test, &LoadView::typical());
    assert!(out.skipped);
    assert!(out.loaded.is_none());
    assert_eq!(out.gain_type, GainType::Test);
}

#[test]
fn load_gains_dispatches_when_type_changes() {
    let out = load_gains(GainType::Original, GainType::IntraTest, &LoadView::typical());
    assert!(!out.skipped);
    let loaded = out.loaded.expect("intra load");
    almost(
        loaded.roll.ri,
        AUTOTUNE_LIVE_RP_DEFAULT * AUTOTUNE_PI_RATIO_FOR_TESTING,
    );
}

#[test]
fn save_skips_when_no_axis_completed() {
    let mut view = LoadView::typical();
    view.axes_completed = 0;
    let out = save_tuning_gains(&view);
    assert!(out.skipped);
    assert!(!out.reset);
    assert!(out.gcs_message.is_none());
    assert!(!out.roll.written);
}

#[test]
fn save_writes_final_gains_and_resaves_orig() {
    let mut view = LoadView::typical();
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_ROLL;
    view.tune_roll.rp = 0.22;
    view.tune_roll.rd = 0.006;
    view.tune_roll.sp = 7.0;
    view.tune_roll.accel_radss = 2.5;
    let out = save_tuning_gains(&view);
    assert!(!out.skipped);
    assert!(out.reset);
    assert_eq!(out.gcs_message, Some(AUTOTUNE_MESSAGE_SAVED_GAINS));
    assert!(out.roll.saved);
    almost(out.roll.rp, 0.22);
    almost(out.roll.ri, 0.22 * AUTOTUNE_PI_RATIO_FINAL);
    almost(out.roll.fltt.unwrap(), AUTOTUNE_LIVE_FLTT_DEFAULT);
    let orig = out.orig_roll.expect("orig resave");
    almost(orig.rp, 0.22);
    almost(orig.ri, 0.22);
    almost(orig.sp, 7.0);
    almost(orig.accel_radss, 2.5);
    almost(orig.fltt, AUTOTUNE_LIVE_FLTT_DEFAULT);
    assert!(!out.pitch.written);
}

#[test]
fn save_yaw_uses_yaw_final_ratio() {
    let mut view = LoadView::typical();
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_YAW;
    view.tune_yaw.rp = 0.25;
    view.tune_yaw.r_lpf = 3.5;
    let out = save_tuning_gains(&view);
    assert!(out.yaw.saved);
    almost(out.yaw.ri, 0.25 * AUTOTUNE_YAW_PI_RATIO_FINAL);
    almost(out.yaw.flte.unwrap(), 3.5);
    let orig = out.orig_yaw.expect("yaw orig");
    almost(orig.r_lpf, 3.5);
    almost(orig.rp, 0.25);
}

#[test]
fn save_enables_ff_when_live_ff_off() {
    let mut view = LoadView::typical();
    view.live_bf_feedforward = false;
    view.axes_completed = AUTOTUNE_AXIS_BITMASK_PITCH;
    let out = save_tuning_gains(&view);
    assert!(out.bf_feedforward_saved);
    assert!(out.accel_rp_saved_zero);
    assert!(out.pitch.saved);
}

#[test]
fn disarmed_saves_when_tune_complete_without_testing() {
    assert_eq!(
        autotune_disarmed(true, false, GainType::Original),
        DisarmAction::Save
    );
}

#[test]
fn disarmed_saves_when_pilot_is_testing_tuned() {
    assert_eq!(
        autotune_disarmed(false, true, GainType::Tuned),
        DisarmAction::Save
    );
}

#[test]
fn disarmed_resets_when_testing_original() {
    assert_eq!(
        autotune_disarmed(true, true, GainType::Original),
        DisarmAction::Reset
    );
    assert_eq!(
        autotune_disarmed(false, false, GainType::Tuned),
        DisarmAction::Reset
    );
}

#[test]
fn stop_loads_original_and_sends_stopped() {
    let out = autotune_stop(GainType::Test, &LoadView::typical());
    assert_eq!(out.gcs_message, AUTOTUNE_MESSAGE_STOPPED);
    assert!(!out.load.skipped);
    assert_eq!(out.load.gain_type, GainType::Original);
    let loaded = out.load.loaded.expect("orig");
    almost(loaded.roll.ri, AUTOTUNE_LIVE_RI_DEFAULT);
}

#[test]
fn stop_skips_orig_load_when_already_original() {
    let out = autotune_stop(GainType::Original, &LoadView::typical());
    assert!(out.load.skipped);
    assert_eq!(out.gcs_message, AUTOTUNE_MESSAGE_STOPPED);
}

#[test]
fn typical_live_axis_is_stable() {
    let live = LiveAxis::typical();
    almost(live.rp, AUTOTUNE_LIVE_RP_DEFAULT);
    assert_eq!(AUTOTUNE_AXIS_BITMASK_DEFAULT, 7);
}
