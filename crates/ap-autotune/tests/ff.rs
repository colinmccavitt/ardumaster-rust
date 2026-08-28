//! I-term / FF coupling stub.

use ap_autotune::ff::{
    apply_ff_i, constrain_ff_step, constrain_imax, couple_ff_i, couple_i,
    AUTOTUNE_DECREASE_FF_STEP, AUTOTUNE_INCREASE_FF_STEP, AUTOTUNE_I_RATIO, AUTOTUNE_MAX_IMAX,
    AUTOTUNE_MIN_IMAX, TRIM_TCONST,
};
use ap_autotune::gains::AtGains;
use ap_autotune::state::{AtType, AutoTune};

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
fn upstream_step_and_ratio_constants() {
    close(AUTOTUNE_INCREASE_FF_STEP, 12.0);
    close(AUTOTUNE_DECREASE_FF_STEP, 15.0);
    close(AUTOTUNE_I_RATIO, 0.75);
    close(AUTOTUNE_MIN_IMAX, 0.4);
    close(AUTOTUNE_MAX_IMAX, 0.9);
    close(TRIM_TCONST, 1.0);
}

#[test]
fn ff_increase_is_capped_at_twelve_percent() {
    close(constrain_ff_step(1.00, 2.00), 1.00 * 1.12);
    close(constrain_ff_step(0.50, 0.80), 0.50 * 1.12);
}

#[test]
fn ff_decrease_is_capped_at_fifteen_percent() {
    close(constrain_ff_step(1.00, 0.10), 1.00 * 0.85);
    close(constrain_ff_step(0.40, 0.20), 0.40 * 0.85);
}

#[test]
fn ff_inside_the_step_band_is_unchanged() {
    close(constrain_ff_step(1.00, 1.05), 1.05);
    close(constrain_ff_step(1.00, 0.90), 0.90);
}

#[test]
fn roll_i_is_the_smaller_of_ff_or_p() {
    close(couple_i(AtType::Roll, 0.40, 0.20), 0.20);
    close(couple_i(AtType::Roll, 0.10, 0.20), 0.10);
    close(couple_i(AtType::Roll, 0.25, 0.25), 0.25);
}

#[test]
fn pitch_i_uses_i_ratio_or_ff() {
    close(couple_i(AtType::Pitch, 0.40, 0.20), 0.40 * 0.75);
    close(couple_i(AtType::Pitch, 0.20, 0.40), 0.40);
}

#[test]
fn yaw_i_matches_pitch() {
    close(
        couple_i(AtType::Yaw, 0.40, 0.20),
        couple_i(AtType::Pitch, 0.40, 0.20),
    );
    close(
        couple_i(AtType::Yaw, 0.20, 0.40),
        couple_i(AtType::Pitch, 0.20, 0.40),
    );
}

#[test]
fn imax_clamps_to_autotune_band() {
    close(constrain_imax(0.10), AUTOTUNE_MIN_IMAX);
    close(constrain_imax(1.00), AUTOTUNE_MAX_IMAX);
    close(constrain_imax(0.60), 0.60);
}

#[test]
fn couple_ff_i_constrains_then_writes_roll_i() {
    let (ff, i) = couple_ff_i(AtType::Roll, 0.40, 1.00, 2.00);
    close(ff, 1.12);
    close(i, 0.40);
}

#[test]
fn apply_ff_i_rewrites_i_and_leaves_p() {
    let (next, ff) = apply_ff_i(AtType::Pitch, sample_roll(), 0.20, 0.10);
    close(ff, 0.20 * 0.85);
    close(next.i, 0.40 * 0.75);
    close(next.p, 0.40);
    close(next.d, 0.02);
}

#[test]
fn couple_ff_i_is_noop_when_not_running() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    let ff = tuner.couple_ff_i(0.20, 0.80);
    close(ff, 0.20);
    close(tuner.current.i, 0.15);
}

#[test]
fn running_session_constrains_ff_and_couples_roll_i() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    let ff = tuner.couple_ff_i(0.20, 0.05);
    close(ff, 0.20 * 0.85);
    close(tuner.current.i, 0.20 * 0.85);
    close(tuner.current.p, 0.40);
}
