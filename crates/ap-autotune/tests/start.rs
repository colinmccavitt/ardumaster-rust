//! `AP_AutoTune::start` zero-FF floor (0.01) stub.

use ap_autotune::start::{floor_start_ff, AUTOTUNE_MIN_FF};
use ap_autotune::state::{AtType, AutoTune};

fn close(got: f32, expect: f32) {
    assert!((got - expect).abs() < 1e-6, "{got} != {expect}");
}

#[test]
fn min_ff_matches_upstream_floor() {
    close(AUTOTUNE_MIN_FF, 0.01);
}

#[test]
fn floor_start_ff_raises_zero_and_tiny_ff() {
    close(floor_start_ff(0.0), AUTOTUNE_MIN_FF);
    close(floor_start_ff(0.009), AUTOTUNE_MIN_FF);
    close(floor_start_ff(-0.5), AUTOTUNE_MIN_FF);
}

#[test]
fn floor_start_ff_keeps_ff_at_or_above_the_floor() {
    close(floor_start_ff(0.01), 0.01);
    close(floor_start_ff(0.25), 0.25);
}

#[test]
fn start_floors_zero_ff_so_the_tuner_never_starts_at_zero() {
    let mut tuner = AutoTune::new(AtType::Roll);
    close(tuner.ff, 0.0);
    tuner.start();
    close(tuner.ff, AUTOTUNE_MIN_FF);
    assert!(tuner.running);
}

#[test]
fn start_keeps_ff_already_above_the_floor() {
    let mut tuner = AutoTune::new(AtType::Pitch);
    tuner.ff = 0.25;
    tuner.start();
    close(tuner.ff, 0.25);
}

#[test]
fn start_floors_negative_ff() {
    let mut tuner = AutoTune::new(AtType::Yaw);
    tuner.ff = -0.1;
    tuner.start();
    close(tuner.ff, AUTOTUNE_MIN_FF);
}
