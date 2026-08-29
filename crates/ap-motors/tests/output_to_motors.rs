//! Matrix PWM-pass leftover — `output_to_motors`. COP-005.
//!
//! The mixer already writes `_thrust_rpyt_out`. These tests check the
//! function that turns those thrusts into actuator values and PWM: the
//! three spool branches, the immediate zero in shut-down (no slew), the
//! tilt-quad override mask that only shut-down consults, and the PWM
//! write that still fires for an overridden motor.

#![allow(
    clippy::float_cmp,
    reason = "catalog flags and shutdown zeros are exact; PWM pulses \
are integers; actuator steps use a tight epsilon against the helpers"
)]

use ap_motors::armed::REMAINING;
use ap_motors::output::{output_to_pwm, RcWrite, SlewParams};
use ap_motors::output_to_motors::{
    analog_pwm, motor_enabled_mask, OutputToMotors, OutputToMotorsInputs,
};
use ap_motors::spool::SpoolState;
use ap_motors::thrust_linearization::{ThrustLinParams, ThrustLinearization};
use ap_motors::{MotorMatrix, MAX_NUM_MOTORS};

fn quad_x() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1), "QUAD X");
    m
}

fn flying(thrusts: &[f32]) -> OutputToMotorsInputs {
    let mut thrust_rpyt_out = [0.0_f32; MAX_NUM_MOTORS];
    for (i, &t) in thrusts.iter().enumerate() {
        if let Some(slot) = thrust_rpyt_out.get_mut(i) {
            *slot = t;
        }
    }
    OutputToMotorsInputs {
        spool_state: SpoolState::ThrottleUnlimited,
        armed: true,
        thrust_rpyt_out,
        ..OutputToMotorsInputs::default()
    }
}

fn write_at(writes: &[Option<RcWrite>; MAX_NUM_MOTORS], i: usize) -> Option<RcWrite> {
    writes.get(i).copied().unwrap_or(None)
}

fn pwm_of(write: Option<RcWrite>) -> u16 {
    match write {
        Some(RcWrite::Pwm(p)) => p,
        other => panic!("expected PWM write, got {other:?}"),
    }
}

#[test]
fn leftover_catalog_drops_pwm_pass_and_setup_helpers() {
    assert!(!REMAINING.contains(&"output_to_motors"));
    assert!(!REMAINING.contains(&"check_for_failed_motor"));
    assert!(!REMAINING.contains(&"output_armed_stabilizing"));
    assert!(REMAINING.is_empty());
}

#[test]
fn shut_down_zeros_enabled_actuators_immediately() {
    let mut state = OutputToMotors::new();
    let mut inputs = flying(&[0.5, 0.5, 0.5, 0.5]);
    state.output_to_motors(&quad_x(), &inputs);
    assert!(state.actuator(0) > 0.2, "flying must have left zero");

    inputs.spool_state = SpoolState::ShutDown;
    let writes = state.output_to_motors(&quad_x(), &inputs);
    for i in 0..4_u8 {
        assert_eq!(state.actuator(i), 0.0, "motor {i}");
        assert_eq!(pwm_of(write_at(&writes, usize::from(i))), 1000);
    }
    assert_eq!(state.actuator(4), 0.0);
    assert!(write_at(&writes, 4).is_none());
}

#[test]
fn shut_down_writes_zero_pwm_when_disarm_disables_it() {
    let mut state = OutputToMotors::new();
    let mut inputs = OutputToMotorsInputs {
        spool_state: SpoolState::ShutDown,
        armed: false,
        ..OutputToMotorsInputs::default()
    };
    inputs.pwm.disarm_disable_pwm = true;
    let writes = state.output_to_motors(&quad_x(), &inputs);
    assert_eq!(pwm_of(write_at(&writes, 0)), 0);
    assert_eq!(state.actuator(0), 0.0);
}

#[test]
fn shut_down_does_not_slew() {
    let mut state = OutputToMotors::new();
    let mut inputs = flying(&[1.0, 1.0, 1.0, 1.0]);
    state.output_to_motors(&quad_x(), &inputs);
    inputs.spool_state = SpoolState::ShutDown;
    inputs.slew = SlewParams {
        slew_up_time: 0.5,
        slew_dn_time: 0.5,
    };
    state.output_to_motors(&quad_x(), &inputs);
    assert_eq!(
        state.actuator(0),
        0.0,
        "shutdown assigns zero, it does not slew"
    );
}

#[test]
fn ground_idle_slews_toward_spin_min_times_ramp() {
    let mut state = OutputToMotors::new();
    let inputs = OutputToMotorsInputs {
        spool_state: SpoolState::GroundIdle,
        spin_up_ratio: 1.0,
        ..OutputToMotorsInputs::default()
    };
    state.output_to_motors(&quad_x(), &inputs);
    let spin_min = ThrustLinParams::default().spin_min;
    assert!((state.actuator(0) - spin_min).abs() < 1e-6);
    let expected = output_to_pwm(SpoolState::GroundIdle, true, &analog_pwm(), spin_min);
    let writes = state.output_to_motors(&quad_x(), &inputs);
    let expected_u = u16::try_from(expected.max(0)).expect("pwm non-negative");
    assert_eq!(pwm_of(write_at(&writes, 0)), expected_u);
}

#[test]
fn flying_states_use_thrust_to_actuator() {
    let tl = ThrustLinearization::new();
    let params = ThrustLinParams::default();
    let target = tl.thrust_to_actuator(&params, 0.5);

    for state_name in [
        SpoolState::SpoolingUp,
        SpoolState::ThrottleUnlimited,
        SpoolState::SpoolingDown,
    ] {
        let mut state = OutputToMotors::new();
        let mut inputs = flying(&[0.5, 0.5, 0.5, 0.5]);
        inputs.spool_state = state_name;
        state.output_to_motors(&quad_x(), &inputs);
        assert!(
            (state.actuator(0) - target).abs() < 1e-6,
            "{state_name:?} actuator {}",
            state.actuator(0)
        );
    }
}

#[test]
fn slew_limits_the_first_up_step() {
    let mut state = OutputToMotors::new();
    let mut inputs = flying(&[1.0, 1.0, 1.0, 1.0]);
    inputs.slew = SlewParams {
        slew_up_time: 0.5,
        slew_dn_time: 0.0,
    };
    inputs.dt_s = 0.0025;
    state.output_to_motors(&quad_x(), &inputs);
    // delta_up_max = 0.0025 / 0.5 = 0.005, from 0.
    assert!((state.actuator(0) - 0.005).abs() < 1e-6);
}

#[test]
fn disabled_slots_are_not_written() {
    let mut state = OutputToMotors::new();
    let writes = state.output_to_motors(&quad_x(), &flying(&[1.0; 8]));
    assert!(write_at(&writes, 0).is_some());
    assert!(write_at(&writes, 4).is_none());
    assert_eq!(state.actuator(4), 0.0);
}

#[test]
fn override_mask_leaves_shutdown_actuator_alone() {
    let mut state = OutputToMotors::new();
    let mut inputs = flying(&[0.8, 0.8, 0.8, 0.8]);
    state.output_to_motors(&quad_x(), &inputs);
    let flown = state.actuator(0);
    assert!(flown > 0.5);

    inputs.spool_state = SpoolState::ShutDown;
    inputs.motor_mask_override = 1;
    let writes = state.output_to_motors(&quad_x(), &inputs);
    assert!(
        (state.actuator(0) - flown).abs() < 1e-6,
        "overridden motor must keep its last actuator"
    );
    assert_eq!(state.actuator(1), 0.0);
    // PWM write still fires for the overridden motor.
    assert_eq!(pwm_of(write_at(&writes, 0)), 1000);
    assert_eq!(pwm_of(write_at(&writes, 1)), 1000);
    assert!(!motor_enabled_mask(&quad_x(), 0, 1));
}

#[test]
fn zero_thrust_flying_sits_at_spin_min() {
    let mut state = OutputToMotors::new();
    state.output_to_motors(&quad_x(), &flying(&[0.0, 0.0, 0.0, 0.0]));
    let spin_min = ThrustLinParams::default().spin_min;
    assert!((state.actuator(0) - spin_min).abs() < 1e-6);
}
