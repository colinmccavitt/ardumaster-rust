//! `ModeSport` init / run leftover, upstream `ArduCopter/mode_sport.cpp`.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_acro::{ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_OVERSHOOT_RAD};
use ap_copter::mode_althold::AltHoldVertical;
use ap_copter::mode_sport::{
    sport_has_user_takeoff, sport_init, sport_mode_flags, sport_run, sport_target_rates_rads,
    SportInitView, SportRunView, MODE_NUMBER_SPORT,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_yaw_rate_rads;
use ap_math::control::sqrt_controller;
use ap_math::scalar::{constrain_value, radians, wrap_pi};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn sport_number_is_thirteen() {
    assert_eq!(MODE_NUMBER_SPORT, 13);
    assert_eq!(sport_mode_flags().mode_number, MODE_NUMBER_SPORT);
}

#[test]
fn sport_flags_are_rate_mode_without_manual_throttle() {
    let flags = sport_mode_flags();
    assert!(!flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(flags.allows_arming);
    assert!(!flags.is_autopilot);
}

#[test]
fn user_takeoff_is_in_place_only() {
    assert!(sport_has_user_takeoff(false));
    assert!(!sport_has_user_takeoff(true));
}

#[test]
fn init_starts_d_only_when_inactive() {
    let view = SportInitView {
        d_is_active: false,
        speed_dn_ms: 1.5,
        speed_up_ms: 2.5,
        accel_d_mss: 2.0,
    };
    let cold = sport_init(false, &view);
    assert!(cold.init_d_controller);
    assert!(cold.set_max_speed_accel);
    assert!(cold.set_correction_speed_accel);
    assert!(cold.ok);
    assert_eq!(cold.speed_dn_ms.to_bits(), 1.5f32.to_bits());
    assert_eq!(cold.speed_up_ms.to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.accel_d_mss.to_bits(), 2.0f32.to_bits());

    let mut hot_view = view;
    hot_view.d_is_active = true;
    let hot = sport_init(true, &hot_view);
    assert!(!hot.init_d_controller);
    assert!(hot.set_max_speed_accel);
    assert!(hot.set_correction_speed_accel);
    assert!(hot.ok);
    assert_eq!(hot.speed_dn_ms.to_bits(), cold.speed_dn_ms.to_bits());
    assert_eq!(hot.speed_up_ms.to_bits(), cold.speed_up_ms.to_bits());
    assert_eq!(hot.accel_d_mss.to_bits(), cold.accel_d_mss.to_bits());
}

#[test]
fn flying_sends_climb_rate_and_euler_rates() {
    let mut view = SportRunView::flying();
    view.target_climb_rate_ms = 1.0;
    view.roll_in_norm = 0.25;
    view.yaw_in_norm = -0.4;
    let out = sport_run(&view);

    assert_eq!(out.state, AltHoldModeState::Flying);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert_eq!(out.vertical, AltHoldVertical::ClimbRate);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(!out.reset_yaw_target_and_rate);
    assert!(!out.start_takeoff);
    assert!(out.set_max_speed_accel);
    assert!(out.input_euler_rate);
    assert!(out.update_d_controller);
    assert_eq!(out.target_climb_rate_ms.to_bits(), 1.0f32.to_bits());

    let (roll, pitch, yaw) = sport_target_rates_rads(&view);
    assert_eq!(out.target_roll_rads.to_bits(), roll.to_bits());
    assert_eq!(out.target_pitch_rads.to_bits(), pitch.to_bits());
    assert_eq!(out.target_yaw_rads.to_bits(), yaw.to_bits());
    assert_eq!(
        out.target_roll_rads.to_bits(),
        (0.25f32 * radians(360.0)).to_bits()
    );
    assert_eq!(out.target_pitch_rads.to_bits(), 0.0f32.to_bits());
    assert_eq!(
        out.target_yaw_rads.to_bits(),
        pilot_desired_yaw_rate_rads(-0.4, view.yaw_rate_degs, view.yaw_expo, true).to_bits()
    );
}

#[test]
fn roll_pitch_are_stick_times_rate_without_expo() {
    let mut view = SportRunView::flying();
    view.roll_in_norm = 0.5;
    view.pitch_in_norm = -1.0;
    view.rp_rate_degs = 180.0;
    let (roll, pitch, yaw) = sport_target_rates_rads(&view);
    assert_eq!(roll.to_bits(), (0.5f32 * radians(180.0)).to_bits());
    assert_eq!(pitch.to_bits(), ((-1.0f32) * radians(180.0)).to_bits());
    assert_eq!(yaw.to_bits(), 0.0f32.to_bits());
}

#[test]
fn balance_pulls_toward_level_from_the_attitude_target() {
    let mut view = SportRunView::flying();
    view.att_target_roll_rad = 0.2;
    view.att_target_pitch_rad = -0.1;
    view.balance_roll = 1.0;
    view.balance_pitch = 0.5;
    let (roll, pitch, _) = sport_target_rates_rads(&view);
    let roll_angle = wrap_pi(0.2);
    let pitch_angle = wrap_pi(-0.1);
    assert_eq!(
        roll.to_bits(),
        (-constrain_value(
            roll_angle,
            -ACRO_LEVEL_MAX_ANGLE_RAD,
            ACRO_LEVEL_MAX_ANGLE_RAD,
        ) * 1.0)
            .to_bits()
    );
    assert_eq!(
        pitch.to_bits(),
        (-constrain_value(
            pitch_angle,
            -ACRO_LEVEL_MAX_ANGLE_RAD,
            ACRO_LEVEL_MAX_ANGLE_RAD,
        ) * 0.5)
            .to_bits()
    );
}

#[test]
fn balance_clamps_to_acro_level_max() {
    let mut view = SportRunView::flying();
    view.att_target_roll_rad = 1.2;
    view.lean_angle_max_rad = 2.0;
    let (roll, _, _) = sport_target_rates_rads(&view);
    assert_eq!(roll.to_bits(), (-ACRO_LEVEL_MAX_ANGLE_RAD).to_bits());
}

#[test]
fn lean_max_overshoot_adds_sqrt_controller() {
    let mut view = SportRunView::flying();
    view.att_target_roll_rad = 0.8;
    view.lean_angle_max_rad = 0.5;
    view.accel_roll_max_radss = 4.0;
    view.dt = 0.0025;
    let (roll, _, _) = sport_target_rates_rads(&view);

    let roll_angle = wrap_pi(0.8);
    let balance = constrain_value(
        roll_angle,
        -ACRO_LEVEL_MAX_ANGLE_RAD,
        ACRO_LEVEL_MAX_ANGLE_RAD,
    );
    let p = radians(view.rp_rate_degs) / ACRO_LEVEL_MAX_OVERSHOOT_RAD;
    let shove = sqrt_controller(0.5 - roll_angle, p, 4.0, 0.0025);
    assert_eq!(roll.to_bits(), (-balance + shove).to_bits());
}

#[test]
fn climb_rate_is_clamped_to_pilot_speeds() {
    let mut view = SportRunView::flying();
    view.target_climb_rate_ms = 10.0;
    view.speed_up_ms = 2.5;
    view.speed_dn_ms = 1.5;
    let up = sport_run(&view);
    assert_eq!(up.target_climb_rate_ms.to_bits(), 2.5f32.to_bits());

    view.target_climb_rate_ms = -10.0;
    let dn = sport_run(&view);
    assert_eq!(dn.target_climb_rate_ms.to_bits(), (-1.5f32).to_bits());
}

#[test]
fn motor_stopped_hard_resets_and_relaxes_d() {
    let mut view = SportRunView::flying();
    view.armed = false;
    view.spool_state = SpoolState::ShutDown;
    let out = sport_run(&view);
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
    let mut view = SportRunView::flying();
    view.land_complete = true;
    view.auto_armed = false;
    view.spool_state = SpoolState::GroundIdle;
    view.target_climb_rate_ms = 0.0;
    let out = sport_run(&view);
    assert_eq!(out.state, AltHoldModeState::LandedGroundIdle);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
    assert!(out.reset_yaw_target_and_rate);
    assert!(out.reset_yaw_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
}

#[test]
fn landed_pre_takeoff_does_not_reset_yaw() {
    let mut view = SportRunView::flying();
    view.land_complete = true;
    view.auto_armed = false;
    view.spool_state = SpoolState::ThrottleUnlimited;
    let out = sport_run(&view);
    assert_eq!(out.state, AltHoldModeState::LandedPreTakeoff);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
    assert!(!out.reset_yaw_target_and_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
}

#[test]
fn takeoff_starts_the_helper_once() {
    let mut view = SportRunView::flying();
    view.land_complete = true;
    view.auto_armed = true;
    view.target_climb_rate_ms = 1.0;
    view.takeoff_running = false;
    view.takeoff_alt_m = 25.0;
    let start = sport_run(&view);
    assert_eq!(start.state, AltHoldModeState::Takeoff);
    assert_eq!(start.desired_spool, None);
    assert_eq!(start.vertical, AltHoldVertical::Takeoff);
    assert!(start.start_takeoff);
    assert_eq!(start.takeoff_start_alt_m.to_bits(), 10.0f32.to_bits());

    view.takeoff_running = true;
    let running = sport_run(&view);
    assert_eq!(running.state, AltHoldModeState::Takeoff);
    assert!(!running.start_takeoff);
}

#[test]
fn invalid_radio_zeros_yaw_but_not_roll_pitch() {
    let mut view = SportRunView::flying();
    view.has_valid_input = false;
    view.roll_in_norm = 0.5;
    view.yaw_in_norm = 1.0;
    let out = sport_run(&view);
    assert_eq!(
        out.target_roll_rads.to_bits(),
        (0.5f32 * radians(360.0)).to_bits()
    );
    assert_eq!(out.target_yaw_rads.to_bits(), 0.0f32.to_bits());
}
