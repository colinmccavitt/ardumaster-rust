//! Action / D-limit hunting stub (`RAISE_D` / `LOWER_D` / `LOWER_PD` /
//! `IDLE_LOWER_PD`).

use ap_autotune::action::{
    apply_d_hunt, apply_idle_lower_pd, d_dominates_p, d_limit_is_set, hunt_d_action, hunt_d_gains,
    linear_interpolate, min_limit, p_dominates_d, should_idle_lower_pd, Action, D_DOMINATES_P,
    D_SET_SETTLE_MS, IDLE_DMOD_THRESH, IDLE_LOWER_GAIN_MUL, IDLE_OSCILLATE_MS, LOWER_D_AGAIN_MUL,
    LOWER_D_FIRST_MUL, LOWER_PD_D_MUL, LOWER_PD_P_MUL, P_DOMINATES_D, RAISE_D_MUL,
};
use ap_autotune::completeness::{completeness_has, PortStatus};
use ap_autotune::gains::AtGains;
use ap_autotune::state::{AtType, AutoTune};

fn sample_roll() -> AtGains {
    AtGains {
        tau: 0.50,
        rmax_pos: 75.0,
        rmax_neg: 75.0,
        p: 0.40,
        i: 0.15,
        d: 0.020,
    }
}

fn close(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn action_discriminants_match_upstream() {
    assert_eq!(Action::None.as_u8(), 0);
    assert_eq!(Action::LowerPd.as_u8(), 4);
    assert_eq!(Action::IdleLowerPd.as_u8(), 5);
    assert_eq!(Action::RaiseD.as_u8(), 6);
    assert_eq!(Action::LowerD.as_u8(), 8);
    assert_eq!(Action::from_u8(6), Some(Action::RaiseD));
    assert_eq!(Action::from_u8(10), None);
}

#[test]
fn raise_d_when_not_oscillating_and_no_limit() {
    close(RAISE_D_MUL, 1.3);
    assert_eq!(
        hunt_d_action(false, 0.0, 0.10, 0.10, true, false),
        Action::RaiseD
    );
    let (p, d, limit) = apply_d_hunt(0.40, 0.020, 0.0, Action::RaiseD);
    close(p, 0.40);
    close(d, 0.020 * 1.3);
    close(limit, 0.0);
}

#[test]
fn lower_pd_when_oscillating_and_p_dominates() {
    close(P_DOMINATES_D, 0.5);
    close(LOWER_PD_P_MUL, 0.35);
    close(LOWER_PD_D_MUL, 0.75);
    assert!(p_dominates_d(0.20, 0.10));
    assert!(!p_dominates_d(0.04, 0.10));
    assert_eq!(
        hunt_d_action(true, 0.0, 0.20, 0.10, true, false),
        Action::LowerPd
    );
    let (p, d, limit) = apply_d_hunt(0.40, 0.020, 0.0, Action::LowerPd);
    close(p, 0.40 * 0.35);
    close(d, 0.020 * 0.75);
    close(limit, 0.0);
    assert!(!d_limit_is_set(limit));
}

#[test]
fn lower_d_first_sets_d_limit_to_thirty_percent() {
    close(LOWER_D_FIRST_MUL, 0.3);
    assert_eq!(
        hunt_d_action(true, 0.0, 0.04, 0.10, true, false),
        Action::LowerD
    );
    let (p, d, limit) = apply_d_hunt(0.40, 0.020, 0.0, Action::LowerD);
    close(p, 0.40);
    close(d, 0.020 * 0.3);
    close(limit, 0.020 * 0.3);
    assert!(d_limit_is_set(limit));
}

#[test]
fn lower_d_again_uses_thirty_five_percent_after_limit() {
    close(LOWER_D_AGAIN_MUL, 0.35);
    close(D_DOMINATES_P, 0.8);
    close(D_SET_SETTLE_MS as f32, 2000.0);
    assert!(d_dominates_p(0.10, 0.10));
    assert_eq!(
        hunt_d_action(true, 0.010, 0.10, 0.10, true, true),
        Action::LowerD
    );
    let (p, d, limit) = apply_d_hunt(0.40, 0.020, 0.010, Action::LowerD);
    close(p, 0.40);
    close(d, 0.020 * 0.35);
    close(limit, 0.020 * 0.35);
}

#[test]
fn no_raise_d_until_ff_ready() {
    assert_eq!(
        hunt_d_action(false, 0.0, 0.10, 0.10, false, false),
        Action::None
    );
}

#[test]
fn hunt_d_gains_rewrites_only_p_and_d() {
    let (next, limit, action) = hunt_d_gains(sample_roll(), 0.0, false, 0.10, 0.10, true, false);
    assert_eq!(action, Action::RaiseD);
    close(next.p, 0.40);
    close(next.d, 0.020 * 1.3);
    close(next.i, 0.15);
    close(next.tau, 0.50);
    close(limit, 0.0);
}

#[test]
fn idle_lower_pd_scales_by_slew_share() {
    close(IDLE_LOWER_GAIN_MUL, 0.5);
    close(IDLE_DMOD_THRESH, 0.9);
    assert_eq!(IDLE_OSCILLATE_MS, 500);
    assert!(should_idle_lower_pd(501, 0.80));
    assert!(!should_idle_lower_pd(500, 0.80));
    assert!(!should_idle_lower_pd(600, 0.90));

    // All P slew → P *= 0.5, D *= 1.0.
    let (p, d) = apply_idle_lower_pd(0.40, 0.020, 1.0, 0.0);
    close(p, 0.40 * 0.5);
    close(d, 0.020 * 1.0);

    // All D slew → P *= 1.0, D *= 0.5.
    let (p, d) = apply_idle_lower_pd(0.40, 0.020, 0.0, 1.0);
    close(p, 0.40);
    close(d, 0.020 * 0.5);

    close(linear_interpolate(0.5, 1.0, 1.0, 1.0, 0.0), 0.5);
    close(linear_interpolate(0.5, 1.0, 0.0, 1.0, 0.0), 1.0);
}

#[test]
fn min_limit_caps_stored_ceiling() {
    close(min_limit(0.50, 0.20), 0.20);
    close(min_limit(0.10, 0.20), 0.10);
    close(min_limit(0.0, 0.20), 0.0);
}

#[test]
fn hunt_d_limit_is_noop_when_not_running() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_roll());
    assert_eq!(
        tuner.hunt_d_limit(true, 0.04, 0.10, true, false),
        Action::None
    );
    close(tuner.current.d, 0.020);
    close(tuner.d_limit, 0.0);
}

#[test]
fn running_session_discovers_d_limit() {
    let mut tuner = AutoTune::with_gains(AtType::Pitch, sample_roll());
    tuner.start();
    tuner.done_count = 2;
    assert_eq!(
        tuner.hunt_d_limit(true, 0.04, 0.10, true, false),
        Action::LowerD
    );
    close(tuner.current.d, 0.020 * 0.3);
    close(tuner.d_limit, 0.020 * 0.3);
    // First discovery does not clear done_count.
    assert_eq!(tuner.done_count, 2);

    assert_eq!(
        tuner.hunt_d_limit(true, 0.10, 0.10, true, true),
        Action::LowerD
    );
    close(tuner.current.d, 0.020 * 0.3 * 0.35);
    assert_eq!(tuner.done_count, 0);
}

#[test]
fn running_idle_lower_pd_caps_limits() {
    let mut tuner = AutoTune::with_gains(AtType::Yaw, sample_roll());
    tuner.start();
    tuner.p_limit = 0.80;
    tuner.d_limit = 0.050;
    assert!(tuner.idle_lower_pd(600, 0.80, 1.0, 0.0));
    close(tuner.current.p, 0.40 * 0.5);
    close(tuner.current.d, 0.020);
    close(tuner.p_limit, 0.40 * 0.5);
    close(tuner.d_limit, 0.020);
}

#[test]
fn completeness_lists_d_limit_hunting_this_slice() {
    assert!(completeness_has(
        "Action / D-limit hunting",
        PortStatus::ThisSlice
    ));
    assert!(!completeness_has(
        "Action / D-limit hunting",
        PortStatus::Remaining
    ));
}
