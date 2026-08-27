//! Deepstall override_servos HAL output.

use ap_landing::deepstall_override::{
    deepstall_override_servos_step, DeepstallOverrideInputs, DeepstallOverrideOutputs,
};
use ap_landing::deepstall_stage::DeepstallStage;

fn land_inputs() -> DeepstallOverrideInputs {
    DeepstallOverrideInputs {
        stage: DeepstallStage::Land,
        stall_entry_ms: 0,
        now_ms: 5000,
        slew_speed: 1.0,
        initial_elevator_pwm: 1500,
        target_elevator_pwm: 1900,
        airspeed_ms: Some(10.0),
        handoff_airspeed_ms: 12.0,
        handoff_lower_limit_ms: 8.0,
        steering_pid: 0.5,
        aileron_scalar: 1.0,
        elevator_present: true,
    }
}

#[test]
fn inactive_outside_land_stage() {
    let mut inp = land_inputs();
    inp.stage = DeepstallStage::Approach;
    let out = deepstall_override_servos_step(&inp);
    assert_eq!(out, DeepstallOverrideOutputs::default());
}

#[test]
fn missing_elevator_requests_go_around() {
    let mut inp = land_inputs();
    inp.elevator_present = false;
    let out = deepstall_override_servos_step(&inp);
    assert!(out.missing_elevator);
    assert!(!out.overrides);
}

#[test]
fn elevator_slews_toward_target() {
    let inp = land_inputs();
    let out = deepstall_override_servos_step(&inp);
    assert!(out.overrides);
    assert_eq!(out.elevator_pwm, 1900);
}

#[test]
fn steering_runs_after_slew_at_low_airspeed() {
    let inp = land_inputs();
    let out = deepstall_override_servos_step(&inp);
    assert!((out.aileron_scaled.unwrap() - 2250.0).abs() < 1.0);
    assert!((out.rudder_scaled.unwrap() - 2250.0).abs() < 1.0);
    assert_eq!(out.throttle_scaled, Some(0.0));
}

#[test]
fn steering_waits_until_slew_and_airspeed_handoff() {
    let mut inp = land_inputs();
    inp.now_ms = 50;
    inp.airspeed_ms = Some(15.0);
    let out = deepstall_override_servos_step(&inp);
    assert_eq!(out.elevator_pwm, 1700);
    assert!(out.aileron_scaled.is_none());
    assert!(out.rudder_scaled.is_none());
    assert!(out.throttle_scaled.is_none());
}

#[test]
fn aileron_scalar_scales_steering() {
    let mut inp = land_inputs();
    inp.aileron_scalar = 0.5;
    let out = deepstall_override_servos_step(&inp);
    assert!((out.aileron_scaled.unwrap() - 1125.0).abs() < 1.0);
    assert!((out.rudder_scaled.unwrap() - 2250.0).abs() < 1.0);
}
