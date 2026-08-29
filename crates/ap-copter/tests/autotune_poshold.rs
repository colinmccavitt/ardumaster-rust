//! AutoTune PosHold lean leftover, upstream `AC_AutoTune::get_poshold_attitude_rad`.

use ap_copter::autotune_poshold::{
    get_poshold_attitude_rad, poshold_angle_max_rad, PosHoldAttitudeView,
    AUTOTUNE_POSHOLD_ANGLE_MAX_DEG, AUTOTUNE_POSHOLD_DEADZONE_M, AUTOTUNE_POSHOLD_DIST_LIMIT_M,
    AUTOTUNE_POSHOLD_YAW_DIST_LIMIT_M, AUTOTUNE_POSHOLD_YAW_SLOP_DEG,
};
use ap_copter::mode_autotune::{mode_autotune_run, AutoTuneRunView, AxisType};
use ap_math::scalar::{radians, wrap_pi};

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-4, "{a} != {b}");
}

#[test]
fn constants_match_upstream() {
    almost(AUTOTUNE_POSHOLD_ANGLE_MAX_DEG, 10.0);
    almost(AUTOTUNE_POSHOLD_DIST_LIMIT_M, 20.0);
    almost(AUTOTUNE_POSHOLD_YAW_DIST_LIMIT_M, 5.0);
    almost(AUTOTUNE_POSHOLD_DEADZONE_M, 0.10);
    almost(AUTOTUNE_POSHOLD_YAW_SLOP_DEG, 95.0);
    almost(poshold_angle_max_rad(), radians(10.0));
}

#[test]
fn disabled_or_no_fix_returns_zeros_without_latch() {
    let mut view = PosHoldAttitudeView::typical();
    view.use_poshold = false;
    view.have_position = false;
    view.pos_n_m = 8.0;
    let out = get_poshold_attitude_rad(&view);
    assert!(!out.have_position);
    assert!(!out.latched_start);
    assert!(!out.applied);
    almost(out.roll_out_rad, 0.0);
    almost(out.pitch_out_rad, 0.0);
    almost(out.yaw_out_rad, 0.0);

    view.use_poshold = true;
    view.position_ok = false;
    let out = get_poshold_attitude_rad(&view);
    assert!(!out.have_position);
    assert!(!out.applied);
}

#[test]
fn first_good_fix_latches_start_and_stays_in_deadzone() {
    let mut view = PosHoldAttitudeView::typical();
    view.have_position = false;
    view.pos_n_m = 3.0;
    view.pos_e_m = -1.5;
    view.start_n_m = 99.0;
    view.start_e_m = 99.0;
    let out = get_poshold_attitude_rad(&view);
    assert!(out.have_position);
    assert!(out.latched_start);
    assert!(!out.applied);
    almost(out.start_n_m, 3.0);
    almost(out.start_e_m, -1.5);
    almost(out.roll_out_rad, 0.0);
    almost(out.pitch_out_rad, 0.0);
}

#[test]
fn inside_ten_cm_writes_no_lean() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 0.099;
    let out = get_poshold_attitude_rad(&view);
    assert!(!out.applied);
    almost(out.pitch_out_rad, 0.0);
    almost(out.roll_out_rad, 0.0);
}

#[test]
fn twenty_metres_north_hits_ten_deg_pitch() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 20.0;
    let out = get_poshold_attitude_rad(&view);
    assert!(out.applied);
    almost(out.pitch_out_rad, radians(10.0));
    almost(out.roll_out_rad, 0.0);
}

#[test]
fn ten_metres_north_is_five_deg() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 10.0;
    let out = get_poshold_attitude_rad(&view);
    almost(out.pitch_out_rad, radians(5.0));
    almost(out.roll_out_rad, 0.0);
}

#[test]
fn past_twenty_metres_clamps_at_ten_deg() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 40.0;
    let out = get_poshold_attitude_rad(&view);
    almost(out.pitch_out_rad, radians(10.0));
}

#[test]
fn twenty_metres_east_is_negative_roll() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_e_m = 20.0;
    let out = get_poshold_attitude_rad(&view);
    almost(out.roll_out_rad, -radians(10.0));
    almost(out.pitch_out_rad, 0.0);
}

#[test]
fn yaw_east_rotates_north_error_into_roll() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 20.0;
    view.cos_yaw = 0.0;
    view.sin_yaw = 1.0;
    let out = get_poshold_attitude_rad(&view);
    almost(out.pitch_out_rad, 0.0);
    almost(out.roll_out_rad, radians(10.0));
}

#[test]
fn yaw_stays_until_five_metres() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 4.99;
    view.desired_yaw_rad = 0.3;
    let out = get_poshold_attitude_rad(&view);
    assert!(out.applied);
    almost(out.yaw_out_rad, 0.3);
}

#[test]
fn five_metres_north_yaws_along_the_wind() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 5.0;
    view.desired_yaw_rad = 0.0;
    view.axis = AxisType::Roll;
    let out = get_poshold_attitude_rad(&view);
    almost(out.yaw_out_rad, 0.0);
}

#[test]
fn pitch_axis_points_across_the_wind() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 8.0;
    view.axis = AxisType::Pitch;
    let out = get_poshold_attitude_rad(&view);
    almost(out.yaw_out_rad, radians(90.0));
}

#[test]
fn yaw_slop_picks_the_nearest_180() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = 8.0;
    view.axis = AxisType::Pitch;
    view.desired_yaw_rad = radians(190.0);
    let out = get_poshold_attitude_rad(&view);
    let expected = radians(90.0) + radians(180.0);
    almost(out.yaw_out_rad, expected);
    let slop: f32 = wrap_pi(radians(190.0_f32) - radians(90.0_f32));
    assert!(slop.abs() > radians(95.0_f32));
}

#[test]
fn deadzone_boundary_applies_at_ten_cm() {
    let mut view = PosHoldAttitudeView::typical();
    view.pos_n_m = AUTOTUNE_POSHOLD_DEADZONE_M;
    let out = get_poshold_attitude_rad(&view);
    assert!(out.applied);
    almost(
        out.pitch_out_rad,
        poshold_angle_max_rad() * AUTOTUNE_POSHOLD_DEADZONE_M / AUTOTUNE_POSHOLD_DIST_LIMIT_M,
    );
}

#[test]
fn run_wires_poshold_lean_when_sticks_centered() {
    let mut view = AutoTuneRunView::typical();
    view.use_poshold = true;
    view.position_ok = true;
    view.have_position = true;
    view.pos_n_m = 20.0;
    view.pos_e_m = 0.0;
    view.poshold_start_n_m = 0.0;
    view.poshold_start_e_m = 0.0;
    view.cos_yaw = 1.0;
    view.sin_yaw = 0.0;
    let out = mode_autotune_run(&view);
    assert!(out.poshold_called);
    almost(out.poshold_roll_rad, 0.0);
    almost(out.poshold_pitch_rad, radians(10.0));
}
