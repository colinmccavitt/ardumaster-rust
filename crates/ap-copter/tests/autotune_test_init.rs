//! Multi `test_init` leftover, upstream `AC_AutoTune_Multi::test_init`.

use ap_copter::autotune_test_init::{
    target_angle_max_rp_cd, target_angle_max_y_cd, target_angle_min_rp_cd, target_angle_min_y_cd,
    test_init, TestInitView, AUTOTUNE_FILT_D_HZ_DEFAULT, AUTOTUNE_LEAN_ANGLE_MAX_CD_DEFAULT,
    AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT, AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT,
    AUTOTUNE_RATE_FILT_D_SCALE,
    AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE, AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE,
    AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE, AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE,
    AUTOTUNE_YAW_STEP_SCALE,
};
use ap_copter::mode_autotune::{
    AxisType, TuneType, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS,
    AUTOTUNE_TARGET_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS, AUTOTUNE_Y_FILT_FREQ,
};
use ap_math::scalar::degrees;

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-3, "{a} != {b}");
}

#[test]
fn angle_scales_match_upstream() {
    almost(AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE, 0.5);
    almost(AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE, 1.0);
    almost(AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE, 1.0 / 3.0);
    almost(AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE, 1.0 / 6.0);
    almost(AUTOTUNE_YAW_STEP_SCALE, 0.75);
    almost(AUTOTUNE_RATE_FILT_D_SCALE, 2.0);
}

#[test]
fn target_angle_helpers_scale_lean_max() {
    let lean = AUTOTUNE_LEAN_ANGLE_MAX_CD_DEFAULT;
    almost(target_angle_max_rp_cd(lean), 1_500.0);
    almost(target_angle_min_rp_cd(lean), 1_000.0);
    almost(target_angle_max_y_cd(lean), 3_000.0);
    almost(target_angle_min_y_cd(lean), 500.0);
}

#[test]
fn roll_rate_tune_seats_rp_targets_and_zeros_accumulators() {
    let out = test_init(&TestInitView::typical());
    almost(out.angle_abort, 1_500.0);
    almost(out.target_rate, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
    almost(out.target_angle, 1_500.0);
    almost(
        out.rotation_rate_filt_hz,
        AUTOTUNE_FILT_D_HZ_DEFAULT * AUTOTUNE_RATE_FILT_D_SCALE,
    );
    almost(out.rotation_rate_filt_reset, 0.0);
    assert!(!out.angle_step_commanded);
    almost(out.test_rate_max, 0.0);
    almost(out.test_rate_min, 0.0);
    almost(out.test_angle_max, 0.0);
    almost(out.test_angle_min, 0.0);
    almost(out.accel_measure_rate_max, 0.0);
}

#[test]
fn pitch_uses_pitch_step_reads() {
    let mut view = TestInitView::typical();
    view.axis = AxisType::Pitch;
    view.max_rate_step_pitch_rad = AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS / 100.0
        * (core::f32::consts::PI / 180.0);
    view.max_angle_step_pitch_rad = 0.209_439_51;
    let out = test_init(&view);
    almost(out.angle_abort, 1_500.0);
    almost(out.target_rate, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS);
    almost(out.target_angle, 1_200.0);
    almost(
        out.rotation_rate_filt_hz,
        AUTOTUNE_FILT_D_HZ_DEFAULT * AUTOTUNE_RATE_FILT_D_SCALE,
    );
}

#[test]
fn yaw_uses_fixed_filt_and_three_quarter_step() {
    let mut view = TestInitView::typical();
    view.axis = AxisType::Yaw;
    let out = test_init(&view);
    almost(out.angle_abort, 3_000.0);
    let expected_rate = (degrees(AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT * AUTOTUNE_YAW_STEP_SCALE)
        * 100.0)
        .clamp(AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS);
    almost(out.target_rate, expected_rate);
    let expected_angle = (degrees(AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT * AUTOTUNE_YAW_STEP_SCALE)
        * 100.0)
        .clamp(500.0, 3_000.0);
    almost(out.target_angle, expected_angle);
    almost(out.rotation_rate_filt_hz, AUTOTUNE_Y_FILT_FREQ);
}

#[test]
fn yaw_d_uses_yaw_d_filter() {
    let mut view = TestInitView::typical();
    view.axis = AxisType::YawD;
    view.filt_d_hz_yaw = 8.0;
    let out = test_init(&view);
    almost(out.angle_abort, 3_000.0);
    almost(out.rotation_rate_filt_hz, 16.0);
}

#[test]
fn step_scaler_shrinks_rate_ceiling() {
    let mut view = TestInitView::typical();
    view.step_scaler = 0.2;
    let out = test_init(&view);
    almost(
        out.target_rate,
        AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS.max(0.2 * AUTOTUNE_TARGET_RATE_RLLPIT_CDS),
    );
}

#[test]
fn rate_below_min_clamps_up() {
    let mut view = TestInitView::typical();
    view.max_rate_step_roll_rad = 0.1;
    let out = test_init(&view);
    almost(out.target_rate, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS);
}

#[test]
fn rate_above_max_clamps_down() {
    let mut view = TestInitView::typical();
    view.max_rate_step_roll_rad = 10.0;
    let out = test_init(&view);
    almost(out.target_rate, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
}

#[test]
fn angle_below_min_clamps_up() {
    let mut view = TestInitView::typical();
    view.max_angle_step_roll_rad = 0.05;
    let out = test_init(&view);
    almost(out.target_angle, 1_000.0);
}

#[test]
fn angle_above_max_clamps_down() {
    let mut view = TestInitView::typical();
    view.max_angle_step_roll_rad = 1.0;
    let out = test_init(&view);
    almost(out.target_angle, 1_500.0);
}

#[test]
fn angle_p_resets_filter_from_start_rate() {
    let mut view = TestInitView::typical();
    view.tune_type = TuneType::AnglePUp;
    view.start_rate = 1_250.0;
    let out = test_init(&view);
    almost(out.rotation_rate_filt_reset, 1_250.0);
}

#[test]
fn angle_p_down_also_resets_from_start_rate() {
    let mut view = TestInitView::typical();
    view.tune_type = TuneType::AnglePDown;
    view.start_rate = -800.0;
    let out = test_init(&view);
    almost(out.rotation_rate_filt_reset, -800.0);
}

#[test]
fn rate_p_up_resets_filter_to_zero() {
    let mut view = TestInitView::typical();
    view.tune_type = TuneType::RatePUp;
    view.start_rate = 400.0;
    let out = test_init(&view);
    almost(out.rotation_rate_filt_reset, 0.0);
}

#[test]
fn yaw_step_scaler_uses_yaw_min_floor() {
    let mut view = TestInitView::typical();
    view.axis = AxisType::Yaw;
    view.step_scaler = 0.1;
    view.max_rate_step_yaw_rad = 10.0;
    let out = test_init(&view);
    almost(out.target_rate, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS);
}

#[test]
fn small_lean_max_shrinks_yaw_twitch() {
    let mut view = TestInitView::typical();
    view.axis = AxisType::Yaw;
    view.lean_angle_max_cd = 1_200.0;
    view.max_angle_step_yaw_rad = 1.0;
    let out = test_init(&view);
    almost(out.angle_abort, 1_200.0);
    almost(out.target_angle, 1_200.0);
}
