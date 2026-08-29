//! Multi AutoTune logging leftover, upstream `AC_AutoTune_Multi::Log_AutoTune`
//! / `Log_AutoTuneDetails` / `log_pids`.

use ap_copter::autotune_log::{
    executing_test_logs, log_autotune, log_autotune_details, log_autotune_gains, log_autotune_meas,
    log_autotune_sweep, log_pids, log_write_autotune, log_write_autotune_details,
    update_gains_logs, LogAutoTuneView, ATDE_FMT, ATDE_LABELS, ATDE_MSG, ATDE_MULTS, ATDE_UNITS,
    ATUN_FMT, ATUN_LABELS, ATUN_MSG, ATUN_MULTS, ATUN_UNITS, AUTOTUNE_LOG_CD_TO_DEG,
    LOG_PIDP_MSG_NAME, LOG_PIDR_MSG_NAME, LOG_PIDY_MSG_NAME,
};
use ap_copter::mode_autotune::{AxisType, TuneType};

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-4, "{a} != {b}");
}

#[test]
fn constants_match_upstream() {
    assert_eq!(ATUN_MSG, "ATUN");
    assert_eq!(
        ATUN_LABELS,
        "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt"
    );
    assert_eq!(ATUN_UNITS, "s--ddd---o");
    assert_eq!(ATUN_MULTS, "F--000---0");
    assert_eq!(ATUN_FMT, "QBBfffffff");
    assert_eq!(ATDE_MSG, "ATDE");
    assert_eq!(ATDE_LABELS, "TimeUS,Angle,Rate");
    assert_eq!(ATDE_UNITS, "sdk");
    assert_eq!(ATDE_MULTS, "F00");
    assert_eq!(ATDE_FMT, "Qff");
    almost(AUTOTUNE_LOG_CD_TO_DEG, 0.01);
    assert_eq!(LOG_PIDR_MSG_NAME, "PIDR");
    assert_eq!(LOG_PIDP_MSG_NAME, "PIDP");
    assert_eq!(LOG_PIDY_MSG_NAME, "PIDY");
}

#[test]
fn write_autotune_scales_cd_to_deg() {
    let pkt = log_write_autotune(
        AxisType::Pitch,
        TuneType::RateDUp,
        1_800.0,
        -200.0,
        1_600.0,
        0.12,
        0.003,
        4.2,
        75_000.0,
        1_234,
    );
    assert_eq!(pkt.time_us, 1_234);
    assert_eq!(pkt.axis, AxisType::Pitch as u8);
    assert_eq!(pkt.tune_step, TuneType::RateDUp as u8);
    almost(pkt.targ, 18.0);
    almost(pkt.min, -2.0);
    almost(pkt.max, 16.0);
    almost(pkt.rp, 0.12);
    almost(pkt.rd, 0.003);
    almost(pkt.sp, 4.2);
    almost(pkt.ddt, 75_000.0);
}

#[test]
fn write_details_scales_cd_to_deg() {
    let pkt = log_write_autotune_details(1_250.0, -3_600.0, 99);
    assert_eq!(pkt.time_us, 99);
    almost(pkt.angle, 12.5);
    almost(pkt.rate, -36.0);
}

#[test]
fn details_forwards_lean_and_rotation() {
    let pkt = log_autotune_details(800.0, 4_500.0, 7);
    almost(pkt.angle, 8.0);
    almost(pkt.rate, 45.0);
}

#[test]
fn angle_p_uses_angle_measurements() {
    let mut view = LogAutoTuneView::typical();
    view.tune_type = TuneType::AnglePUp;
    let (targ, min, max) = log_autotune_meas(&view);
    almost(targ, view.target_angle);
    almost(min, view.test_angle_min);
    almost(max, view.test_angle_max);

    view.tune_type = TuneType::AnglePDown;
    let (targ, min, max) = log_autotune_meas(&view);
    almost(targ, view.target_angle);
    almost(min, view.test_angle_min);
    almost(max, view.test_angle_max);
}

#[test]
fn rate_tunes_use_rate_measurements() {
    for tune_type in [
        TuneType::RateDUp,
        TuneType::RateDDown,
        TuneType::RatePUp,
        TuneType::RateFfUp,
        TuneType::MaxGains,
        TuneType::TuneCheck,
        TuneType::TuneComplete,
    ] {
        let mut view = LogAutoTuneView::typical();
        view.tune_type = tune_type;
        let (targ, min, max) = log_autotune_meas(&view);
        almost(targ, view.target_rate);
        almost(min, view.test_rate_min);
        almost(max, view.test_rate_max);
    }
}

#[test]
fn roll_and_pitch_gains_follow_axis() {
    let mut view = LogAutoTuneView::typical();
    view.axis = AxisType::Roll;
    let (rp, rd, sp) = log_autotune_gains(&view);
    almost(rp, view.tune_roll_rp);
    almost(rd, view.tune_roll_rd);
    almost(sp, view.tune_roll_sp);

    view.axis = AxisType::Pitch;
    let (rp, rd, sp) = log_autotune_gains(&view);
    almost(rp, view.tune_pitch_rp);
    almost(rd, view.tune_pitch_rd);
    almost(sp, view.tune_pitch_sp);
}

#[test]
fn yaw_rd_column_is_rlpf_yaw_d_uses_rd() {
    let mut view = LogAutoTuneView::typical();
    view.axis = AxisType::Yaw;
    let (rp, rd, sp) = log_autotune_gains(&view);
    almost(rp, view.tune_yaw_rp);
    almost(rd, view.tune_yaw_r_lpf);
    almost(sp, view.tune_yaw_sp);

    view.axis = AxisType::YawD;
    let (rp, rd, sp) = log_autotune_gains(&view);
    almost(rp, view.tune_yaw_rp);
    almost(rd, view.tune_yaw_rd);
    almost(sp, view.tune_yaw_sp);
}

#[test]
fn log_autotune_angle_p_roll_packs_atun() {
    let mut view = LogAutoTuneView::typical();
    view.axis = AxisType::Roll;
    view.tune_type = TuneType::AnglePUp;
    let pkt = log_autotune(&view, 50);
    assert_eq!(pkt.axis, AxisType::Roll as u8);
    assert_eq!(pkt.tune_step, TuneType::AnglePUp as u8);
    almost(pkt.targ, 15.0);
    almost(pkt.min, -2.0);
    almost(pkt.max, 14.0);
    almost(pkt.rp, 0.15);
    almost(pkt.rd, 0.004);
    almost(pkt.sp, 4.5);
    almost(pkt.ddt, 80_000.0);
}

#[test]
fn log_autotune_rate_yaw_packs_rlpf() {
    let mut view = LogAutoTuneView::typical();
    view.axis = AxisType::Yaw;
    view.tune_type = TuneType::RateDDown;
    let pkt = log_autotune(&view, 8);
    assert_eq!(pkt.axis, AxisType::Yaw as u8);
    almost(pkt.targ, 180.0);
    almost(pkt.min, -5.0);
    almost(pkt.max, 160.0);
    almost(pkt.rd, view.tune_yaw_r_lpf);
}

#[test]
fn sweep_is_noop() {
    assert!(!log_autotune_sweep());
}

#[test]
fn log_pids_writes_three_rate_axes() {
    let pids = log_pids();
    assert!(pids.write_pidr);
    assert!(pids.write_pidp);
    assert!(pids.write_pidy);
}

#[test]
fn executing_test_logs_details_rate_and_pids() {
    let out = executing_test_logs(900.0, 1_200.0, 11);
    almost(out.details.angle, 9.0);
    almost(out.details.rate, 12.0);
    assert_eq!(out.details.time_us, 11);
    assert!(out.need_write_rate);
    assert!(out.pids.write_pidr && out.pids.write_pidp && out.pids.write_pidy);
}

#[test]
fn update_gains_logs_atun() {
    let view = LogAutoTuneView::typical();
    let pkt = update_gains_logs(&view, 3);
    let expected = log_autotune(&view, 3);
    assert_eq!(pkt, expected);
}
