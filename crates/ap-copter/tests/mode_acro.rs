//! `ModeAcro::run` leftovers: trainer-off rates into attitude / throttle.

use ap_copter::mode_acro::{
    acro_pilot_desired_rates_rads, acro_run, AcroRunView, ACRO_OPTION_RATE_LOOP_ONLY,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_throttle;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn flying_asks_unlimited_without_angle_boost() {
    let view = AcroRunView::flying();
    let out = acro_run(&view);
    assert_eq!(out.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(!out.angle_boost);
    assert!(!out.rate_loop_only);
    assert!(!out.scale_i_to_angle_p);
    assert!(!out.reset_target_and_rate);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(out.clear_land_complete);
    assert_eq!(
        out.throttle_out.to_bits(),
        pilot_desired_throttle(500, 500, 0.5).to_bits()
    );
}

#[test]
fn rate_loop_only_scales_i_and_keeps_the_same_rates() {
    let mut view = AcroRunView::flying();
    view.roll_in_norm = 0.5;
    view.pitch_in_norm = -0.25;
    view.yaw_in_norm = 1.0;
    let attitude = acro_run(&view);
    view.rate_loop_only = true;
    let rate_only = acro_run(&view);

    assert!(rate_only.rate_loop_only);
    assert!(rate_only.scale_i_to_angle_p);
    assert!(!attitude.rate_loop_only);
    assert_eq!(
        rate_only.rates.roll_rads.to_bits(),
        attitude.rates.roll_rads.to_bits()
    );
    assert_eq!(
        rate_only.rates.pitch_rads.to_bits(),
        attitude.rates.pitch_rads.to_bits()
    );
    assert_eq!(
        rate_only.rates.yaw_rads.to_bits(),
        attitude.rates.yaw_rads.to_bits()
    );
    assert_eq!(ACRO_OPTION_RATE_LOOP_ONLY, 1 << 1);
}

#[test]
fn trainer_off_rates_scale_with_the_stick() {
    let half = acro_pilot_desired_rates_rads(0.25, -0.125, 1.0, 360.0, 0.0, 202.5, 0.0);
    let full = acro_pilot_desired_rates_rads(0.5, -0.25, 1.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(half.roll_rads.to_bits(), (full.roll_rads * 0.5).to_bits());
    assert_eq!(half.pitch_rads.to_bits(), (full.pitch_rads * 0.5).to_bits());
    assert_eq!(half.yaw_rads.to_bits(), full.yaw_rads.to_bits());
}

#[test]
fn circular_limit_caps_diagonal_stick() {
    let unlimited = acro_pilot_desired_rates_rads(1.0, 0.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    let diagonal = acro_pilot_desired_rates_rads(1.0, 1.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(diagonal.roll_rads.to_bits(), diagonal.pitch_rads.to_bits());
    assert!(diagonal.roll_rads < unlimited.roll_rads);
    assert!(diagonal.roll_rads > 0.0);
}

#[test]
fn shut_down_resets_the_whole_target() {
    let mut view = AcroRunView::flying();
    view.spool_state = SpoolState::ShutDown;
    view.throttle_control = 800;
    let out = acro_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_target_and_rate);
    assert!(out.reset_target_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
}

#[test]
fn ground_idle_smooth_resets_the_whole_target() {
    let mut view = AcroRunView::flying();
    view.spool_state = SpoolState::GroundIdle;
    let out = acro_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_target_and_rate);
    assert!(out.reset_target_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
}

#[test]
fn throttle_zero_asks_ground_idle() {
    let mut view = AcroRunView::flying();
    view.throttle_zero = true;
    let out = acro_run(&view);
    assert_eq!(out.desired_spool, DesiredSpoolState::GroundIdle);
}
