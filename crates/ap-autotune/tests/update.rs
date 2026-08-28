//! Saturation / overshoot gain-update stub.

use ap_autotune::gains::AtGains;
use ap_autotune::state::{AtType, AutoTune};
use ap_autotune::update::{
    apply_p_step, couple_tau_rmax, gain_action, slew_rmax, slew_tau, update_gains, GainAction,
    LOWER_P_MUL, RAISE_P_MUL, RMAX_DEFAULT, RMAX_STEP, TAU_SLEW_DOWN, TAU_SLEW_UP,
};

fn sample_roll() -> AtGains {
    AtGains {
        tau: 0.50,
        rmax_pos: 75.0,
        rmax_neg: 75.0,
        p: 0.40,
        i: 0.15,
        d: 0.02,
    }
}

fn close(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn saturated_raises_p() {
    assert_eq!(gain_action(true, false), GainAction::RaiseP);
    close(apply_p_step(0.40, GainAction::RaiseP), 0.40 * RAISE_P_MUL);
    let next = update_gains(sample_roll(), true, false, 0.50, 75.0);
    close(next.p, 0.40 * 1.3);
    close(next.i, 0.15);
    close(next.d, 0.02);
}

#[test]
fn overshoot_lowers_p() {
    assert_eq!(gain_action(false, true), GainAction::LowerP);
    close(apply_p_step(0.40, GainAction::LowerP), 0.40 * LOWER_P_MUL);
    let next = update_gains(sample_roll(), false, true, 0.50, 75.0);
    close(next.p, 0.40 * 0.35);
}

#[test]
fn overshoot_wins_over_saturation() {
    assert_eq!(gain_action(true, true), GainAction::LowerP);
    let next = update_gains(sample_roll(), true, true, 0.50, 75.0);
    close(next.p, 0.40 * 0.35);
}

#[test]
fn neither_flag_leaves_p_alone() {
    assert_eq!(gain_action(false, false), GainAction::None);
    close(apply_p_step(0.40, GainAction::None), 0.40);
    let next = update_gains(sample_roll(), false, false, 0.50, 75.0);
    close(next.p, 0.40);
}

#[test]
fn rmax_steps_at_most_twenty() {
    close(RMAX_STEP, 20.0);
    close(slew_rmax(75.0, 75.0), 75.0);
    close(slew_rmax(75.0, 120.0), 95.0);
    close(slew_rmax(75.0, 40.0), 55.0);
    close(slew_rmax(0.0, 300.0), RMAX_DEFAULT + RMAX_STEP);
}

#[test]
fn tau_slews_at_most_fifteen_percent() {
    close(TAU_SLEW_DOWN, 0.85);
    close(TAU_SLEW_UP, 1.15);
    close(slew_tau(0.50, 0.50), 0.50);
    close(slew_tau(0.50, 0.10), 0.50 * 0.85);
    close(slew_tau(0.50, 1.00), 0.50 * 1.15);
}

#[test]
fn tau_rmax_coupling_copies_rmax_neg() {
    let next = couple_tau_rmax(sample_roll(), 0.30, 120.0);
    close(next.tau, 0.50 * 0.85);
    close(next.rmax_pos, 75.0 + 20.0);
    close(next.rmax_neg, next.rmax_pos);
    close(next.p, 0.40);
}

#[test]
fn update_gains_is_noop_when_not_running() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.update_gains(true, false, 0.30, 90.0);
    close(tuner.current.p, 0.40);
    close(tuner.current.tau, 0.50);
}

#[test]
fn running_session_raises_p_and_slews_tau_rmax() {
    let mut tuner = AutoTune::with_gains(AtType::Pitch, sample_roll());
    tuner.start();
    tuner.update_gains(true, false, 0.30, 120.0);
    close(tuner.current.p, 0.40 * 1.3);
    close(tuner.current.tau, 0.50 * 0.85);
    close(tuner.current.rmax_pos, 95.0);
    close(tuner.current.rmax_neg, 95.0);
}
