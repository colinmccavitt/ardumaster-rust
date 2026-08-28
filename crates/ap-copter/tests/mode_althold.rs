//! `ModeAltHold` leftovers: `run()` attitude / vertical, and `init`.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_althold::{
    althold_init, althold_run, AltHoldInitView, AltHoldRunView, AltHoldVertical,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_yaw_rate_rads;
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn flying_sends_climb_rate_and_always_updates_d() {
    let mut view = AltHoldRunView::flying();
    view.target_climb_rate_ms = 1.0;
    view.roll_in_norm = 0.3;
    view.yaw_in_norm = -0.4;
    let out = althold_run(&view);

    assert_eq!(out.state, AltHoldModeState::Flying);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::ThrottleUnlimited));
    assert_eq!(out.vertical, AltHoldVertical::ClimbRate);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(!out.reset_yaw_target_and_rate);
    assert!(!out.start_takeoff);
    assert!(out.update_d_controller);
    assert_eq!(out.target_climb_rate_ms.to_bits(), 1.0f32.to_bits());

    let (roll, pitch) = pilot_desired_lean_angles_rad(
        0.3,
        0.0,
        view.lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        true,
    );
    let yaw = pilot_desired_yaw_rate_rads(-0.4, view.yaw_rate_degs, view.yaw_expo, true);
    assert_eq!(out.target_roll_rad.to_bits(), roll.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), pitch.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), yaw.to_bits());
}

#[test]
fn climb_rate_is_clamped_to_pilot_speeds() {
    let mut view = AltHoldRunView::flying();
    view.target_climb_rate_ms = 10.0;
    view.speed_up_ms = 2.5;
    view.speed_dn_ms = 1.5;
    let up = althold_run(&view);
    assert_eq!(up.target_climb_rate_ms.to_bits(), 2.5f32.to_bits());

    view.target_climb_rate_ms = -10.0;
    let dn = althold_run(&view);
    assert_eq!(dn.target_climb_rate_ms.to_bits(), (-1.5f32).to_bits());
}

#[test]
fn motor_stopped_hard_resets_and_relaxes_d() {
    let mut view = AltHoldRunView::flying();
    view.armed = false;
    view.spool_state = SpoolState::ShutDown;
    let out = althold_run(&view);
    assert_eq!(out.state, AltHoldModeState::MotorStopped);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::ShutDown));
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(out.reset_yaw_target_and_rate);
    assert!(!out.reset_yaw_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
    assert!(out.update_d_controller);
}

#[test]
fn landed_ground_idle_falls_through_to_smooth_reset() {
    let mut view = AltHoldRunView::flying();
    view.land_complete = true;
    view.auto_armed = false;
    view.spool_state = SpoolState::GroundIdle;
    view.target_climb_rate_ms = 0.0;
    let out = althold_run(&view);
    assert_eq!(out.state, AltHoldModeState::LandedGroundIdle);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
    assert!(out.reset_yaw_target_and_rate);
    assert!(out.reset_yaw_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
}

#[test]
fn landed_pre_takeoff_does_not_reset_yaw() {
    let mut view = AltHoldRunView::flying();
    view.land_complete = true;
    view.auto_armed = false;
    view.spool_state = SpoolState::ThrottleUnlimited;
    let out = althold_run(&view);
    assert_eq!(out.state, AltHoldModeState::LandedPreTakeoff);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
    assert!(!out.reset_yaw_target_and_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
}

#[test]
fn takeoff_starts_the_helper_once() {
    let mut view = AltHoldRunView::flying();
    view.land_complete = true;
    view.auto_armed = true;
    view.target_climb_rate_ms = 1.0;
    view.takeoff_running = false;
    view.takeoff_alt_m = 25.0;
    let start = althold_run(&view);
    assert_eq!(start.state, AltHoldModeState::Takeoff);
    assert_eq!(start.desired_spool, None);
    assert_eq!(start.vertical, AltHoldVertical::Takeoff);
    assert!(start.start_takeoff);
    assert_eq!(start.takeoff_start_alt_m.to_bits(), 10.0f32.to_bits());

    view.takeoff_running = true;
    let running = althold_run(&view);
    assert_eq!(running.state, AltHoldModeState::Takeoff);
    assert!(!running.start_takeoff);
}

#[test]
fn invalid_radio_is_neutral_lean_and_yaw() {
    let mut view = AltHoldRunView::flying();
    view.has_valid_input = false;
    view.roll_in_norm = 1.0;
    view.yaw_in_norm = 1.0;
    let out = althold_run(&view);
    assert_eq!(out.target_roll_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), 0.0f32.to_bits());
}

#[test]
fn init_starts_d_only_when_inactive() {
    let view = AltHoldInitView {
        d_is_active: false,
        speed_dn_ms: 1.5,
        speed_up_ms: 2.5,
        accel_d_mss: 2.0,
    };
    let cold = althold_init(false, &view);
    assert!(cold.init_d_controller);
    assert!(cold.set_max_speed_accel);
    assert!(cold.set_correction_speed_accel);
    assert!(cold.ok);
    assert_eq!(cold.speed_dn_ms.to_bits(), 1.5f32.to_bits());
    assert_eq!(cold.speed_up_ms.to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.accel_d_mss.to_bits(), 2.0f32.to_bits());

    let mut hot_view = view;
    hot_view.d_is_active = true;
    let hot = althold_init(true, &hot_view);
    assert!(!hot.init_d_controller);
    assert!(hot.set_max_speed_accel);
    assert!(hot.set_correction_speed_accel);
    assert!(hot.ok);
    assert_eq!(hot.speed_dn_ms.to_bits(), cold.speed_dn_ms.to_bits());
    assert_eq!(hot.speed_up_ms.to_bits(), cold.speed_up_ms.to_bits());
    assert_eq!(hot.accel_d_mss.to_bits(), cold.accel_d_mss.to_bits());
}
