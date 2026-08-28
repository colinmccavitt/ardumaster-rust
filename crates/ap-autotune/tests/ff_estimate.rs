//! FF estimate / `ff_filter` single-event step stub.

use ap_autotune::ff_estimate::{
    apply_ff_count_gate, apply_ff_count_gains, ff_estimate_pending, ff_estimate_ready, ff_single,
    FfEstimate, AUTOTUNE_MIN_D, AUTOTUNE_MIN_P, FF_COUNT_FIRST, FF_COUNT_READY,
    FF_FILTER_RETURN_ELEMENT, FF_READY_P_SCALE,
};
use ap_autotune::gains::AtGains;
use ap_autotune::state::{AtState, AtType, AutoTune};

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
fn upstream_ff_count_and_filter_constants() {
    assert_eq!(FF_FILTER_RETURN_ELEMENT, 2);
    assert_eq!(FF_COUNT_FIRST, 1);
    assert_eq!(FF_COUNT_READY, 4);
    close(AUTOTUNE_MIN_D, 0.0005);
    close(AUTOTUNE_MIN_P, 0.01);
    close(FF_READY_P_SCALE, 0.5);
}

#[test]
fn positive_demand_uses_max_actuator_over_max_rate() {
    close(
        ff_single(AtState::DemandPos, 10.0, -4.0, 50.0, -20.0, 1.0),
        10.0 / 50.0,
    );
    close(
        ff_single(AtState::DemandPos, 8.0, -3.0, 40.0, -15.0, 2.0),
        8.0 / (40.0 * 2.0),
    );
}

#[test]
fn negative_demand_uses_min_actuator_over_min_rate() {
    close(
        ff_single(AtState::DemandNeg, 10.0, -4.0, 50.0, -20.0, 1.0),
        -4.0 / -20.0,
    );
    close(
        ff_single(AtState::DemandNeg, 8.0, -6.0, 40.0, -30.0, 2.0),
        -6.0 / (-30.0 * 2.0),
    );
}

#[test]
fn idle_follows_the_upstream_else_min_pair() {
    close(
        ff_single(AtState::Idle, 10.0, -4.0, 50.0, -20.0, 1.0),
        ff_single(AtState::DemandNeg, 10.0, -4.0, 50.0, -20.0, 1.0),
    );
}

#[test]
fn ff_filter_apply_returns_the_first_sample() {
    let mut est = FfEstimate::new();
    close(est.apply(0.20), 0.20);
    close(est.filtered(), 0.20);
}

#[test]
fn apply_event_stores_ff_single_and_bumps_count() {
    let mut est = FfEstimate::new();
    let ff = est.apply_event(AtState::DemandPos, 10.0, -4.0, 50.0, -20.0, 1.0);
    close(est.ff_single, 0.20);
    close(ff, 0.20);
    assert_eq!(est.ff_count, 1);
    assert!(est.estimate_pending());
    assert!(!est.estimate_ready());
}

#[test]
fn first_event_floors_p_and_d() {
    let (p, d) = apply_ff_count_gate(0.0, 0.0, 1);
    close(p, AUTOTUNE_MIN_P);
    close(d, AUTOTUNE_MIN_D);
    let (p, d) = apply_ff_count_gate(0.40, 0.02, 1);
    close(p, 0.40);
    close(d, 0.02);
    assert!(ff_estimate_pending(1));
    assert!(!ff_estimate_ready(1));
}

#[test]
fn counts_two_and_three_leave_gains_and_stay_pending() {
    let (p, d) = apply_ff_count_gate(0.40, 0.02, 2);
    close(p, 0.40);
    close(d, 0.02);
    let (p, d) = apply_ff_count_gate(0.40, 0.02, 3);
    close(p, 0.40);
    close(d, 0.02);
    assert!(ff_estimate_pending(2));
    assert!(ff_estimate_pending(3));
    assert!(!ff_estimate_ready(3));
}

#[test]
fn fourth_event_accepts_the_estimate_and_halves_p() {
    let (p, d) = apply_ff_count_gate(0.40, 0.02, 4);
    close(p, 0.40 * FF_READY_P_SCALE);
    close(d, 0.02);
    assert!(!ff_estimate_pending(4));
    assert!(ff_estimate_ready(4));
    assert!(ff_estimate_ready(5));
}

#[test]
fn four_events_accept_the_filtered_estimate() {
    let mut est = FfEstimate::new();
    for _ in 0..4 {
        est.apply_event(AtState::DemandPos, 10.0, -4.0, 50.0, -20.0, 1.0);
    }
    assert_eq!(est.ff_count, 4);
    close(est.ff_single, 0.20);
    close(est.filtered(), 0.20);
    assert!(est.estimate_ready());
    assert!(!est.estimate_pending());
}

#[test]
fn apply_ff_count_gains_rewrites_p_d_and_leaves_i() {
    let next = apply_ff_count_gains(sample_roll(), 4);
    close(next.p, 0.20);
    close(next.d, 0.02);
    close(next.i, 0.15);
}

#[test]
fn apply_ff_count_gate_is_noop_when_not_running() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.apply_ff_count_gate(4);
    close(tuner.current.p, 0.40);
    close(tuner.current.d, 0.02);
}

#[test]
fn running_session_halves_p_on_the_ready_gate() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    tuner.start();
    tuner.apply_ff_count_gate(4);
    close(tuner.current.p, 0.20);
    close(tuner.current.d, 0.02);
}

#[test]
fn reset_clears_count_and_ff_single() {
    let mut est = FfEstimate::new();
    est.apply_event(AtState::DemandNeg, 10.0, -4.0, 50.0, -20.0, 1.0);
    est.reset();
    assert_eq!(est.ff_count, 0);
    close(est.ff_single, 0.0);
    close(est.filtered(), 0.0);
    assert!(est.estimate_pending());
}
