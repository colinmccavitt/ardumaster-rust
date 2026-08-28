//! VTOL_Assist force-assist / Q_OPTIONS stub.

use ap_vtol_assist::{
    disable_synthetic_airspeed_assist_set, evaluate_force, evaluate_speed_alt,
    force_assist_latched, requested_overriding_speed_alt, synthetic_airspeed_assist_allowed,
    AssistState, ForceSample, SpeedAltSample, VtolAssist, DISABLE_SYNTHETIC_AIRSPEED_ASSIST,
    Q_ASSIST_FORCE_ENABLE,
};

#[test]
fn q_options_force_bit_matches_upstream() {
    assert_eq!(Q_ASSIST_FORCE_ENABLE, 1 << 7);
    assert_eq!(DISABLE_SYNTHETIC_AIRSPEED_ASSIST, 1 << 12);
    assert!(!disable_synthetic_airspeed_assist_set(0));
    assert!(disable_synthetic_airspeed_assist_set(
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST
    ));
    assert!(!disable_synthetic_airspeed_assist_set(
        Q_ASSIST_FORCE_ENABLE
    ));
}

#[test]
fn q_options_force_bit_requests_when_speed_gate_is_closed() {
    let assist = VtolAssist::new();
    assert!(!assist.speed_checks_enabled());
    assert_eq!(assist.state(), AssistState::AssistEnabled);
    assert!(force_assist_latched(&assist, Q_ASSIST_FORCE_ENABLE));

    let d = evaluate_force(&assist, ForceSample::new(Q_ASSIST_FORCE_ENABLE, true));
    assert!(d.force_assist());
    assert!(d.requested());
    assert!(d.overrides_speed_alt());
    assert!(d.spin_while_armed());
}

#[test]
fn zero_q_options_does_not_force() {
    let assist = VtolAssist::new();
    assert!(!force_assist_latched(&assist, 0));

    let d = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(!d.force_assist());
    assert!(!d.requested());
    assert!(!d.overrides_speed_alt());
    assert!(!d.spin_while_armed());
}

#[test]
fn force_enabled_state_forces_without_the_option_bit() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::ForceEnabled);

    let d = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(d.force_assist());
    assert!(d.requested());
    assert!(d.overrides_speed_alt());
    assert!(d.spin_while_armed());
}

#[test]
fn aux_disabled_wins_over_q_options_force_bit() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::AssistDisabled);
    assert!(!force_assist_latched(&assist, Q_ASSIST_FORCE_ENABLE));

    let d = evaluate_force(&assist, ForceSample::new(Q_ASSIST_FORCE_ENABLE, true));
    assert!(!d.force_assist());
    assert!(!d.requested());
    assert!(!d.spin_while_armed());
}

#[test]
fn spin_while_armed_requires_force_and_armed() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::ForceEnabled);

    let armed = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(armed.force_assist());
    assert!(armed.spin_while_armed());

    let disarmed = evaluate_force(&assist, ForceSample::new(0, false));
    assert!(disarmed.force_assist());
    assert!(!disarmed.spin_while_armed());

    let idle = VtolAssist::new();
    let no_force = evaluate_force(&idle, ForceSample::new(0, true));
    assert!(!no_force.force_assist());
    assert!(!no_force.spin_while_armed());
}

#[test]
fn force_overrides_closed_speed_alt_gate() {
    let assist = VtolAssist::new();
    assert!(!assist.speed_checks_enabled());

    let speed_alt = evaluate_speed_alt(&assist, SpeedAltSample::new(1.0, true, 2.0));
    assert!(!speed_alt.force_assist());
    assert!(!speed_alt.speed_assist());
    assert!(!speed_alt.alt_assist());
    assert!(!speed_alt.requested());

    let force = evaluate_force(&assist, ForceSample::new(Q_ASSIST_FORCE_ENABLE, true));
    assert!(force.overrides_speed_alt());
    assert!(requested_overriding_speed_alt(force, speed_alt));
}

#[test]
fn force_still_requests_when_speed_and_alt_are_healthy() {
    let mut assist = VtolAssist::new();
    assist.set_speed(8.0);
    assist.set_alt(15);

    let speed_alt = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 80.0));
    assert!(!speed_alt.requested());

    assist.set_state(AssistState::ForceEnabled);
    let force = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(force.requested());
    assert!(requested_overriding_speed_alt(force, speed_alt));
}

#[test]
fn disable_synthetic_airspeed_does_not_force_assist() {
    let assist = VtolAssist::new();
    assert!(!force_assist_latched(
        &assist,
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST
    ));

    let d = evaluate_force(
        &assist,
        ForceSample::new(DISABLE_SYNTHETIC_AIRSPEED_ASSIST, true),
    );
    assert!(!d.force_assist());
    assert!(!d.requested());
    assert!(!d.spin_while_armed());
}

#[test]
fn synthetic_airspeed_assist_option_bit() {
    assert!(synthetic_airspeed_assist_allowed(0, false));
    assert!(synthetic_airspeed_assist_allowed(0, true));
    assert!(!synthetic_airspeed_assist_allowed(
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST,
        false
    ));
    assert!(synthetic_airspeed_assist_allowed(
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST,
        true
    ));
    assert!(synthetic_airspeed_assist_allowed(
        Q_ASSIST_FORCE_ENABLE,
        false
    ));
}

#[test]
fn apply_q_options_and_force_module_agree() {
    let mut assist = VtolAssist::new();
    assist.apply_q_options(Q_ASSIST_FORCE_ENABLE);
    assert_eq!(assist.state(), AssistState::ForceEnabled);

    let d = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(d.force_assist());
    assert!(d.spin_while_armed());
}
