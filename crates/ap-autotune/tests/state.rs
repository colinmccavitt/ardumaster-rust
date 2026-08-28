//! AP_AutoTune ATState Idle / DemandPos / DemandNeg machine.

use ap_autotune::state::{
    in_att_demand, next_demand_state, rate_threshold1, rate_threshold2, AtState, AtType, AutoTune,
    ATT_DEMAND_FRAC, RATE_THRESHOLD1_FRAC, RATE_THRESHOLD2_FRAC,
};

#[test]
fn at_type_matches_upstream_discriminants() {
    assert_eq!(AtType::Roll.as_u8(), 0);
    assert_eq!(AtType::Pitch.as_u8(), 1);
    assert_eq!(AtType::Yaw.as_u8(), 2);
    assert_eq!(AtType::from_u8(0), Some(AtType::Roll));
    assert_eq!(AtType::from_u8(1), Some(AtType::Pitch));
    assert_eq!(AtType::from_u8(2), Some(AtType::Yaw));
    assert_eq!(AtType::from_u8(3), None);
    assert_eq!(AtType::Roll.axis_string(), "Roll");
    assert_eq!(AtType::Pitch.axis_string(), "Pitch");
    assert_eq!(AtType::Yaw.axis_string(), "Yaw");
}

#[test]
fn at_state_matches_upstream_discriminants() {
    assert_eq!(AtState::Idle.as_u8(), 0);
    assert_eq!(AtState::DemandPos.as_u8(), 1);
    assert_eq!(AtState::DemandNeg.as_u8(), 2);
    assert_eq!(AtState::from_u8(0), Some(AtState::Idle));
    assert_eq!(AtState::from_u8(1), Some(AtState::DemandPos));
    assert_eq!(AtState::from_u8(2), Some(AtState::DemandNeg));
    assert_eq!(AtState::from_u8(3), None);
}

#[test]
fn rate_thresholds_use_upstream_fractions() {
    assert_eq!(RATE_THRESHOLD1_FRAC, 0.4);
    assert_eq!(RATE_THRESHOLD2_FRAC, 0.25);
    assert_eq!(ATT_DEMAND_FRAC, 0.3);

    // att_limit/tau = 45/0.5 = 90, rmax_pos = 75 -> min is 75.
    let t1 = rate_threshold1(45.0, 0.5, 75.0);
    assert!((t1 - 0.4 * 75.0).abs() < 1e-6);
    let t2 = rate_threshold2(t1);
    assert!((t2 - 0.25 * t1).abs() < 1e-6);

    // att_limit/tau = 20/1 = 20, rmax_pos = 75 -> min is 20.
    let t1_tau = rate_threshold1(20.0, 1.0, 75.0);
    assert!((t1_tau - 0.4 * 20.0).abs() < 1e-6);
}

#[test]
fn in_att_demand_needs_30_percent_of_the_axis_limit() {
    let limit = 45.0;
    let edge = ATT_DEMAND_FRAC * limit;
    assert!(!in_att_demand(edge - 0.1, limit));
    assert!(in_att_demand(edge, limit));
    assert!(in_att_demand(-edge, limit));
}

#[test]
fn idle_enters_demand_pos_when_rate_and_attitude_qualify() {
    assert_eq!(
        next_demand_state(AtState::Idle, 40.0, 30.0, 7.5, true),
        AtState::DemandPos
    );
}

#[test]
fn idle_enters_demand_neg_when_rate_and_attitude_qualify() {
    assert_eq!(
        next_demand_state(AtState::Idle, -40.0, 30.0, 7.5, true),
        AtState::DemandNeg
    );
}

#[test]
fn idle_stays_idle_without_attitude_demand() {
    assert_eq!(
        next_demand_state(AtState::Idle, 40.0, 30.0, 7.5, false),
        AtState::Idle
    );
}

#[test]
fn idle_stays_idle_below_the_entry_threshold() {
    assert_eq!(
        next_demand_state(AtState::Idle, 29.0, 30.0, 7.5, true),
        AtState::Idle
    );
}

#[test]
fn demand_pos_returns_to_idle_below_the_exit_threshold() {
    assert_eq!(
        next_demand_state(AtState::DemandPos, 7.0, 30.0, 7.5, true),
        AtState::Idle
    );
}

#[test]
fn demand_pos_holds_while_rate_stays_at_or_above_exit() {
    assert_eq!(
        next_demand_state(AtState::DemandPos, 7.5, 30.0, 7.5, false),
        AtState::DemandPos
    );
}

#[test]
fn demand_neg_returns_to_idle_above_the_negative_exit_threshold() {
    assert_eq!(
        next_demand_state(AtState::DemandNeg, -7.0, 30.0, 7.5, true),
        AtState::Idle
    );
}

#[test]
fn demand_neg_holds_while_rate_stays_at_or_below_exit() {
    assert_eq!(
        next_demand_state(AtState::DemandNeg, -7.5, 30.0, 7.5, false),
        AtState::DemandNeg
    );
}

#[test]
fn start_sets_running_and_forces_idle() {
    let mut tuner = AutoTune::new(AtType::Roll);
    assert!(!tuner.running);
    assert_eq!(tuner.state, AtState::Idle);
    tuner.state = AtState::DemandPos;
    tuner.start();
    assert!(tuner.running);
    assert_eq!(tuner.state, AtState::Idle);
    assert_eq!(tuner.axis, AtType::Roll);
}

#[test]
fn stop_clears_running() {
    let mut tuner = AutoTune::new(AtType::Pitch);
    tuner.start();
    tuner.stop();
    assert!(!tuner.running);
}

#[test]
fn update_demand_is_a_noop_when_not_running() {
    let mut tuner = AutoTune::new(AtType::Yaw);
    tuner.update_demand(40.0, 30.0, 7.5, true);
    assert_eq!(tuner.state, AtState::Idle);
    assert!(!tuner.running);
}

#[test]
fn update_demand_walks_idle_to_pos_and_back() {
    let mut tuner = AutoTune::new(AtType::Roll);
    tuner.start();
    tuner.update_demand(40.0, 30.0, 7.5, true);
    assert_eq!(tuner.state, AtState::DemandPos);
    tuner.update_demand(7.0, 30.0, 7.5, true);
    assert_eq!(tuner.state, AtState::Idle);
}
