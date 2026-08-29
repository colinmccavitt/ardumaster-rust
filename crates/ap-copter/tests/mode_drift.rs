//! `ModeDrift` leftovers: sideslip roll, pitch brake, and throttle assist.

use ap_copter::mode_drift::{
    drift_init, drift_run, drift_throttle_assist, Drift, DriftRunView, DRIFT_SPEEDGAIN_RAD,
    DRIFT_SPEEDLIMIT_MS, DRIFT_THR_ASSIST_MAX, MODE_NUMBER_DRIFT,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_throttle;
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{cd_to_rad, radians};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn mode_number_is_drift() {
    assert_eq!(MODE_NUMBER_DRIFT, 11);
}

#[test]
fn init_always_succeeds() {
    assert!(drift_init(false).ok);
    assert!(drift_init(true).ok);
}

#[test]
fn flying_hover_is_neutral_roll_and_ramps_braker() {
    let mut drift = Drift::new();
    let view = DriftRunView::flying();
    let out = drift_run(&mut drift, &view);

    assert_eq!(out.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(!out.reset_yaw_target_and_rate);
    assert!(out.clear_land_complete);
    assert!(out.input_euler_angle_roll_pitch_euler_rate_yaw);
    assert!(out.angle_boost);
    assert_eq!(out.target_roll_rad, 0.0);
    assert_eq!(out.target_yaw_rate_rads, 0.0);
    assert_eq!(out.braker.to_bits(), 0.03f32.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(
        out.throttle_out.to_bits(),
        drift_throttle_assist(0.0, pilot_desired_throttle(500, 500, 0.5)).to_bits()
    );
}

#[test]
fn yaw_is_scheduled_from_pilot_roll_not_sideslip_roll() {
    let mut drift = Drift::new();
    let mut view = DriftRunView::flying();
    view.roll_in_norm = 0.5;
    view.vel_e_ms = 1.0;
    let out = drift_run(&mut drift, &view);

    let (pilot_roll, _) = pilot_desired_lean_angles_rad(
        0.5,
        0.0,
        view.attitude_lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        true,
    );
    let expected_yaw = (pilot_roll / radians(45.0)) * radians(view.acro_yaw_rate_degs);
    assert_eq!(out.pilot_roll_rad.to_bits(), pilot_roll.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), expected_yaw.to_bits());
    assert_eq!(out.vel_right_ms.to_bits(), 1.0f32.to_bits());
    // Sideslip overwrites roll: +right velocity commands left lean.
    assert!(out.target_roll_rad < 0.0);
    assert_eq!(out.braker.to_bits(), 0.03f32.to_bits());
}

#[test]
fn pitch_stick_held_zeros_braker() {
    let mut drift = Drift::new();
    drift.braker = 0.1;
    let mut view = DriftRunView::flying();
    view.pitch_in_norm = 0.4;
    let out = drift_run(&mut drift, &view);
    assert_eq!(out.braker.to_bits(), 0.0f32.to_bits());
    let (_, pilot_pitch) = pilot_desired_lean_angles_rad(
        0.0,
        0.4,
        view.attitude_lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        true,
    );
    assert_eq!(out.pilot_pitch_rad.to_bits(), pilot_pitch.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), pilot_pitch.to_bits());
}

#[test]
fn released_pitch_brakes_against_forward_speed() {
    let mut drift = Drift::new();
    let mut view = DriftRunView::flying();
    view.vel_n_ms = 2.0;
    let out = drift_run(&mut drift, &view);
    assert_eq!(out.braker.to_bits(), 0.03f32.to_bits());
    assert_eq!(out.vel_forward_ms.to_bits(), 2.0f32.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), (2.0_f32 * 0.03).to_bits());
}

#[test]
fn braker_caps_at_speedgain() {
    let mut drift = Drift::new();
    let view = DriftRunView::flying();
    for _ in 0..10 {
        let _ = drift_run(&mut drift, &view);
    }
    assert_eq!(drift.braker.to_bits(), DRIFT_SPEEDGAIN_RAD.to_bits());
}

#[test]
fn body_velocity_is_clamped() {
    let mut drift = Drift::new();
    let mut view = DriftRunView::flying();
    view.vel_n_ms = 40.0;
    view.vel_e_ms = 40.0;
    let out = drift_run(&mut drift, &view);
    assert_eq!(
        out.vel_forward_ms.to_bits(),
        DRIFT_SPEEDLIMIT_MS.to_bits()
    );
    assert_eq!(out.vel_right_ms.to_bits(), DRIFT_SPEEDLIMIT_MS.to_bits());
}

#[test]
fn yaw_stick_filters_into_roll_input() {
    let mut drift = Drift::new();
    let mut view = DriftRunView::flying();
    view.yaw_control_cd = 4_500.0;
    let _ = drift_run(&mut drift, &view);
    assert_eq!(
        drift.roll_input_rad.to_bits(),
        (cd_to_rad(4_500.0) * 0.04_f32).to_bits()
    );
}

#[test]
fn shut_down_resets_yaw_without_rate_and_keeps_assisted_throttle() {
    let mut drift = Drift::new();
    let mut view = DriftRunView::flying();
    view.spool_state = SpoolState::ShutDown;
    view.throttle_zero = true;
    let out = drift_run(&mut drift, &view);
    assert_eq!(out.desired_spool, DesiredSpoolState::GroundIdle);
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(out.reset_yaw_target_and_rate);
    assert!(!out.reset_yaw_rate);
    assert!(!out.clear_land_complete);
    assert!(out.throttle_out > 0.0);
}

#[test]
fn throttle_assist_is_identity_outside_the_band() {
    assert_eq!(
        drift_throttle_assist(2.0, 0.1).to_bits(),
        0.1f32.to_bits()
    );
    assert_eq!(
        drift_throttle_assist(2.0, 0.9).to_bits(),
        0.9f32.to_bits()
    );
}

#[test]
fn throttle_assist_at_mid_adds_capped_d_velocity_term() {
    let mid = drift_throttle_assist(2.0, 0.5);
    let expected = (0.5_f32 + (1.2_f32 * 0.18 * 2.0).min(DRIFT_THR_ASSIST_MAX)).min(1.0);
    assert!((mid - expected).abs() < 1.0e-6);

    let capped = drift_throttle_assist(20.0, 0.5);
    assert_eq!(capped.to_bits(), (0.5 + DRIFT_THR_ASSIST_MAX).to_bits());
}
