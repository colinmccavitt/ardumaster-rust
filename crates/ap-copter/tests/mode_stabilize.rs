//! `ModeStabilize::run` leftovers: pilot conversions into attitude / throttle.

use ap_copter::mode_stabilize::{
    manual_throttle_desired_spool, stabilize_run, RateIReset, StabilizeRunView,
};
use ap_copter::pilot_input::{pilot_desired_throttle, pilot_desired_yaw_rate_rads};
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn flying_asks_unlimited_and_keeps_pilot_throttle() {
    let view = StabilizeRunView::flying();
    let out = stabilize_run(&view);

    assert_eq!(out.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(out.angle_boost);
    assert!(!out.reset_yaw_target_and_rate);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(out.clear_land_complete);
    assert_eq!(
        out.throttle_out.to_bits(),
        pilot_desired_throttle(500, 500, 0.5).to_bits()
    );
}

#[test]
fn throttle_zero_asks_ground_idle() {
    let mut view = StabilizeRunView::flying();
    view.throttle_zero = true;
    let out = stabilize_run(&view);
    assert_eq!(out.desired_spool, DesiredSpoolState::GroundIdle);
    assert_eq!(
        manual_throttle_desired_spool(true),
        DesiredSpoolState::GroundIdle
    );
    assert_eq!(
        manual_throttle_desired_spool(false),
        DesiredSpoolState::ThrottleUnlimited
    );
}

#[test]
fn shut_down_zeros_throttle_and_hard_resets() {
    let mut view = StabilizeRunView::flying();
    view.spool_state = SpoolState::ShutDown;
    view.throttle_control = 800;
    let out = stabilize_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_yaw_target_and_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(!out.clear_land_complete);
}

#[test]
fn ground_idle_zeros_throttle_and_smooth_resets() {
    let mut view = StabilizeRunView::flying();
    view.spool_state = SpoolState::GroundIdle;
    view.throttle_control = 800;
    let out = stabilize_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_yaw_target_and_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
    assert!(!out.clear_land_complete);
}

#[test]
fn unlimited_clears_land_only_above_lower_limit() {
    let mut view = StabilizeRunView::flying();
    view.spool_state = SpoolState::ThrottleUnlimited;
    view.throttle_lower_limited = true;
    let limited = stabilize_run(&view);
    assert!(!limited.clear_land_complete);

    view.throttle_lower_limited = false;
    let raised = stabilize_run(&view);
    assert!(raised.clear_land_complete);
}

#[test]
fn spooling_keeps_pilot_throttle_and_skips_resets() {
    for state in [SpoolState::SpoolingUp, SpoolState::SpoolingDown] {
        let mut view = StabilizeRunView::flying();
        view.spool_state = state;
        view.throttle_control = 700;
        let out = stabilize_run(&view);
        assert!(!out.reset_yaw_target_and_rate);
        assert_eq!(out.reset_rate_i, RateIReset::None);
        assert!(!out.clear_land_complete);
        assert_eq!(
            out.throttle_out.to_bits(),
            pilot_desired_throttle(700, 500, 0.5).to_bits()
        );
    }
}

#[test]
fn invalid_radio_is_neutral_attitude() {
    let mut view = StabilizeRunView::flying();
    view.has_valid_input = false;
    view.roll_in_norm = 1.0;
    view.pitch_in_norm = -1.0;
    view.yaw_in_norm = 0.5;
    let out = stabilize_run(&view);
    assert_eq!(out.target_roll_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), 0.0f32.to_bits());
}

#[test]
fn flying_attitude_is_the_pilot_conversion() {
    let mut view = StabilizeRunView::flying();
    view.roll_in_norm = 0.4;
    view.pitch_in_norm = -0.2;
    view.yaw_in_norm = 0.5;
    let out = stabilize_run(&view);
    let (roll, pitch) = pilot_desired_lean_angles_rad(
        0.4,
        -0.2,
        view.lean_angle_max_rad,
        view.lean_angle_max_rad,
        true,
    );
    let yaw = pilot_desired_yaw_rate_rads(0.5, view.yaw_rate_degs, view.yaw_expo, true);
    assert_eq!(out.target_roll_rad.to_bits(), roll.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), pitch.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), yaw.to_bits());
}
