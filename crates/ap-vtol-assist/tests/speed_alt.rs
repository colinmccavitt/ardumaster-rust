//! VTOL_Assist speed / altitude trigger.

use ap_vtol_assist::{evaluate_speed_alt, AssistState, SpeedAltSample, VtolAssist};

fn gated(speed: f32, alt: i16) -> VtolAssist {
    let mut assist = VtolAssist::new();
    assist.set_speed(speed);
    assist.set_alt(alt);
    assist
}

#[test]
fn low_airspeed_requests_speed_assist() {
    let assist = gated(8.0, 0);
    assert!(assist.should_check());
    assert!(assist.is_enabled());

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(4.0, true, 100.0));
    assert!(d.speed_assist());
    assert!(!d.alt_assist());
    assert!(!d.force_assist());
    assert!(d.requested());
}

#[test]
fn airspeed_at_or_above_threshold_does_not_request() {
    let assist = gated(8.0, 0);
    let at = evaluate_speed_alt(&assist, SpeedAltSample::new(8.0, true, 100.0));
    assert!(!at.speed_assist());
    assert!(!at.requested());

    let above = evaluate_speed_alt(&assist, SpeedAltSample::new(12.0, true, 100.0));
    assert!(!above.speed_assist());
    assert!(!above.requested());
}

#[test]
fn missing_airspeed_does_not_request_speed_assist() {
    let assist = gated(8.0, 0);
    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(1.0, false, 100.0));
    assert!(!d.speed_assist());
    assert!(!d.requested());
}

#[test]
fn low_altitude_requests_alt_assist() {
    let assist = gated(8.0, 15);
    assert!(assist.alt_check_enabled());

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 8.0));
    assert!(!d.speed_assist());
    assert!(d.alt_assist());
    assert!(d.requested());
}

#[test]
fn altitude_at_or_above_threshold_does_not_request_alt() {
    let assist = gated(8.0, 15);
    let at = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 15.0));
    assert!(!at.alt_assist());
    assert!(!at.requested());

    let above = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 40.0));
    assert!(!above.alt_assist());
    assert!(!above.requested());
}

#[test]
fn zero_assist_alt_disables_alt_trigger() {
    let assist = gated(8.0, 0);
    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 1.0));
    assert!(!d.alt_assist());
    assert!(!d.requested());
}

#[test]
fn closed_speed_gate_clears_speed_and_alt() {
    let mut assist = VtolAssist::new();
    assist.set_alt(15);
    assert!(!assist.should_check());

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(1.0, true, 2.0));
    assert!(!d.speed_assist());
    assert!(!d.alt_assist());
    assert!(!d.requested());
}

#[test]
fn disabled_state_clears_every_flag() {
    let mut assist = gated(8.0, 15);
    assist.set_state(AssistState::AssistDisabled);

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(1.0, true, 2.0));
    assert!(!d.force_assist());
    assert!(!d.speed_assist());
    assert!(!d.alt_assist());
    assert!(!d.requested());
}

#[test]
fn force_enable_requests_when_speed_gate_is_closed() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::ForceEnabled);

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(1.0, true, 2.0));
    assert!(d.force_assist());
    assert!(!d.speed_assist());
    assert!(!d.alt_assist());
    assert!(d.requested());
}

#[test]
fn speed_or_alt_either_side_requests() {
    let assist = gated(8.0, 15);

    let speed_only = evaluate_speed_alt(&assist, SpeedAltSample::new(3.0, true, 80.0));
    assert!(speed_only.speed_assist());
    assert!(!speed_only.alt_assist());
    assert!(speed_only.requested());

    let alt_only = evaluate_speed_alt(&assist, SpeedAltSample::new(20.0, true, 4.0));
    assert!(!alt_only.speed_assist());
    assert!(alt_only.alt_assist());
    assert!(alt_only.requested());

    let both = evaluate_speed_alt(&assist, SpeedAltSample::new(3.0, true, 4.0));
    assert!(both.speed_assist());
    assert!(both.alt_assist());
    assert!(both.requested());
}

#[test]
fn force_plus_open_gate_keeps_speed_and_alt() {
    let mut assist = gated(8.0, 15);
    assist.set_state(AssistState::ForceEnabled);

    let d = evaluate_speed_alt(&assist, SpeedAltSample::new(3.0, true, 4.0));
    assert!(d.force_assist());
    assert!(d.speed_assist());
    assert!(d.alt_assist());
    assert!(d.requested());
}
