//! AP_AutoTune save_gains / restore_gains snapshot stub.

use ap_autotune::gains::{
    apply_stop_gains, should_save_on_stop, snapshot_gains, AtGains,
};
use ap_autotune::state::{AtType, AutoTune};

fn sample_roll() -> AtGains {
    AtGains {
        tau: 0.5,
        rmax_pos: 75.0,
        rmax_neg: 75.0,
        p: 0.4,
        i: 0.3,
        d: 0.02,
    }
}

fn sample_pitch() -> AtGains {
    AtGains {
        tau: 0.75,
        rmax_pos: 60.0,
        rmax_neg: 45.0,
        p: 0.8,
        i: 0.6,
        d: 0.05,
    }
}

fn tuned(base: AtGains) -> AtGains {
    AtGains {
        tau: base.tau * 0.8,
        rmax_pos: base.rmax_pos + 20.0,
        rmax_neg: base.rmax_neg + 20.0,
        p: base.p * 1.2,
        i: base.i * 1.1,
        d: base.d * 0.5,
    }
}

fn gains_eq(got: AtGains, expect: AtGains) {
    let pairs = [
        ("tau", got.tau, expect.tau),
        ("rmax_pos", got.rmax_pos, expect.rmax_pos),
        ("rmax_neg", got.rmax_neg, expect.rmax_neg),
        ("p", got.p, expect.p),
        ("i", got.i, expect.i),
        ("d", got.d, expect.d),
    ];
    for (name, a, b) in pairs {
        assert!(
            (a - b).abs() < 1e-6,
            "{name} {a} != {b}"
        );
    }
}

#[test]
fn snapshot_copies_current_into_restore_and_last_save() {
    let current = sample_roll();
    let (restore, last_save) = snapshot_gains(current);
    gains_eq(restore, current);
    gains_eq(last_save, current);
}

#[test]
fn abort_stop_restores_and_leaves_last_save() {
    let original = sample_roll();
    let (current, last_save) = apply_stop_gains(
        tuned(original),
        original,
        original,
        0.0,
        0.0,
    );
    gains_eq(current, original);
    gains_eq(last_save, original);
}

#[test]
fn completed_stop_keeps_current_and_updates_last_save() {
    let original = sample_pitch();
    let next = tuned(original);
    let (current, last_save) = apply_stop_gains(next, original, original, 0.5, 0.02);
    gains_eq(current, next);
    gains_eq(last_save, next);
}

#[test]
fn should_save_requires_both_limits_positive() {
    assert!(!should_save_on_stop(0.0, 0.0));
    assert!(!should_save_on_stop(0.4, 0.0));
    assert!(!should_save_on_stop(0.0, 0.02));
    assert!(!should_save_on_stop(-0.1, 0.02));
    assert!(should_save_on_stop(0.4, 0.02));
}

#[test]
fn start_snapshots_roll_gains() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    gains_eq(tuner.current, sample_roll());
    gains_eq(tuner.restore, sample_roll());
    gains_eq(tuner.last_save, sample_roll());
    assert!((tuner.p_limit - 0.0).abs() < 1e-6);
    assert!((tuner.d_limit - 0.0).abs() < 1e-6);
}

#[test]
fn stop_without_limits_restores_roll_gains() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    tuner.current = tuned(sample_roll());
    tuner.stop();
    assert!(!tuner.running);
    gains_eq(tuner.current, sample_roll());
    gains_eq(tuner.last_save, sample_roll());
}

#[test]
fn stop_without_limits_restores_pitch_gains() {
    let mut tuner = AutoTune::with_gains(AtType::Pitch, sample_pitch());
    tuner.start();
    tuner.current = tuned(sample_pitch());
    tuner.stop();
    assert!(!tuner.running);
    gains_eq(tuner.current, sample_pitch());
    gains_eq(tuner.last_save, sample_pitch());
}

#[test]
fn stop_with_limits_saves_tuned_gains() {
    let mut tuner = AutoTune::with_gains(AtType::Pitch, sample_pitch());
    tuner.start();
    let next = tuned(sample_pitch());
    tuner.current = next;
    tuner.p_limit = 0.5;
    tuner.d_limit = 0.02;
    tuner.stop();
    assert!(!tuner.running);
    gains_eq(tuner.current, next);
    gains_eq(tuner.last_save, next);
}

#[test]
fn restore_gains_puts_the_start_snapshot_back() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    tuner.current = tuned(sample_roll());
    tuner.restore_gains();
    gains_eq(tuner.current, sample_roll());
    gains_eq(tuner.last_save, sample_roll());
}

#[test]
fn save_gains_records_last_save_without_changing_current() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    let next = tuned(sample_roll());
    tuner.current = next;
    tuner.save_gains();
    gains_eq(tuner.current, next);
    gains_eq(tuner.last_save, next);
    gains_eq(tuner.restore, sample_roll());
}

#[test]
fn stop_is_a_noop_when_not_running() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.current = tuned(sample_roll());
    tuner.stop();
    assert!(!tuner.running);
    gains_eq(tuner.current, tuned(sample_roll()));
}
