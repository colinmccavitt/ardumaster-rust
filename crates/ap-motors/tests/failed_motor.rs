//! Failed-motor leftover — `check_for_failed_motor`. COP-005.
//!
//! The mixer already writes `_thrust_rpyt_out`. These tests check the
//! function that watches those thrusts: the 0.5 s filter, the peak/mean
//! trip on hexa-and-above, the co-rotating exemption, the held lost-motor
//! index while boost is on, and the boost drop once the pack has room.

#![allow(
    clippy::float_cmp,
    reason = "filter endpoints and leftover catalog flags are exact; \
balance trips are boolean"
)]

use ap_motors::failed_motor::{
    is_corotating_frame, FailedMotor, FailedMotorInputs, BOOST_DROP_HIGH, FILTER_TIME_CONSTANT_S,
    FRAME_TYPE_CW_X_COR, FRAME_TYPE_X_COR, MIN_MOTORS_FOR_UNBALANCE, REBALANCE_THRESHOLD,
    RPYT_SUM_MIN, UNBALANCE_THRESHOLD,
};
use ap_motors::armed::REMAINING;
use ap_motors::{MotorMatrix, MAX_NUM_MOTORS};

fn hexa_x() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(2, 1), "HEXA X");
    m
}

fn quad_x() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1), "QUAD X");
    m
}

fn octaquad_x_cor() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(4, 20), "OCTAQUAD X_COR");
    m
}

/// Inputs whose filter settles in one step (`dt >> 0.5`).
fn settled(thrusts: &[f32], frame_type: u8) -> FailedMotorInputs {
    let mut thrust_rpyt_out = [0.0_f32; MAX_NUM_MOTORS];
    for (i, &t) in thrusts.iter().enumerate() {
        if let Some(slot) = thrust_rpyt_out.get_mut(i) {
            *slot = t;
        }
    }
    FailedMotorInputs {
        dt_s: 100.0,
        thrust_rpyt_out,
        throttle_thrust_best_plus_adj: 0.5,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        frame_type,
    }
}

#[test]
fn leftover_catalog_drops_failed_motor_and_keeps_pwm_pass() {
    assert!(!REMAINING.contains(&"check_for_failed_motor"));
    assert!(REMAINING.contains(&"output_to_motors"));
    assert!(!REMAINING.contains(&"output_armed_stabilizing"));
}

#[test]
fn filter_is_the_first_order_lag_with_half_second_constant() {
    let mut state = FailedMotor::new();
    let mut inputs = FailedMotorInputs {
        dt_s: FILTER_TIME_CONSTANT_S,
        ..FailedMotorInputs::default()
    };
    // alpha = 0.5 / (0.5 + 0.5) = 0.5
    if let Some(slot) = inputs.thrust_rpyt_out.get_mut(0) {
        *slot = 1.0;
    }
    state.check_for_failed_motor(&quad_x(), &inputs);
    assert!((state.thrust_rpyt_out_filt(0) - 0.5).abs() < 1e-6);
    state.check_for_failed_motor(&quad_x(), &inputs);
    assert!((state.thrust_rpyt_out_filt(0) - 0.75).abs() < 1e-6);
}

#[test]
fn disabled_slots_are_not_filtered() {
    let mut state = FailedMotor::new();
    let mut inputs = settled(&[1.0, 1.0, 1.0, 1.0, 1.0], 1);
    state.check_for_failed_motor(&quad_x(), &inputs);
    assert!(state.thrust_rpyt_out_filt(0) > 0.9);
    assert_eq!(
        state.thrust_rpyt_out_filt(4),
        0.0,
        "fifth slot is not a quad motor"
    );
    // Writing a raw value into a disabled slot must not move the filter.
    if let Some(slot) = inputs.thrust_rpyt_out.get_mut(4) {
        *slot = 1.0;
    }
    state.check_for_failed_motor(&quad_x(), &inputs);
    assert_eq!(state.thrust_rpyt_out_filt(4), 0.0);
}

#[test]
fn the_loudest_motor_is_named_while_boost_is_off() {
    let mut state = FailedMotor::new();
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.9, 0.4, 0.4, 0.4], 1));
    assert_eq!(state.motor_lost_index(), 2);
}

#[test]
fn lost_index_holds_while_boost_is_on() {
    let mut state = FailedMotor::new();
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.9, 0.4, 0.4, 0.4], 1));
    assert_eq!(state.motor_lost_index(), 2);
    state.set_thrust_boost(true);
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.4, 0.4, 0.4, 1.0], 1));
    assert_eq!(
        state.motor_lost_index(),
        2,
        "boost must pin the name crash-check already acted on"
    );
}

#[test]
fn a_quad_never_trips_unbalance() {
    let mut state = FailedMotor::new();
    // Peak/mean = 1.0 * 4 / (0.3*3 + 1.0) = 4 / 1.9 ≈ 2.1, well over 1.5,
    // but a quad is below MIN_MOTORS_FOR_UNBALANCE.
    state.check_for_failed_motor(&quad_x(), &settled(&[0.3, 0.3, 0.3, 1.0], 1));
    assert!(state.thrust_balanced());
    assert_eq!(MIN_MOTORS_FOR_UNBALANCE, 6);
}

#[test]
fn a_hexa_with_one_loud_motor_trips_unbalance() {
    let mut state = FailedMotor::new();
    // 5 * 0.4 + 1.0 = 3.0; high = 1.0; balance = 6 / 3.0 = 2.0 >= 1.5
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.4, 1.0, 0.4, 0.4], 1));
    assert!(!state.thrust_balanced());
    assert_eq!(state.motor_lost_index(), 3);
    assert!(UNBALANCE_THRESHOLD > REBALANCE_THRESHOLD);
}

#[test]
fn a_corotating_x8_does_not_trip() {
    let mut state = FailedMotor::new();
    let thrusts = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 1.0];
    state.check_for_failed_motor(
        &octaquad_x_cor(),
        &settled(&thrusts, FRAME_TYPE_X_COR),
    );
    assert!(
        state.thrust_balanced(),
        "X_COR scales its top layer on purpose"
    );
    assert!(is_corotating_frame(FRAME_TYPE_X_COR));
    assert!(is_corotating_frame(FRAME_TYPE_CW_X_COR));
}

#[test]
fn equal_thrusts_clear_a_previous_unbalance() {
    let mut state = FailedMotor::new();
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.4, 1.0, 0.4, 0.4], 1));
    assert!(!state.thrust_balanced());
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 1));
    assert!(state.thrust_balanced());
}

#[test]
fn a_tiny_sum_forces_balance_of_one() {
    let mut state = FailedMotor::new();
    // All thrusts well under RPYT_SUM_MIN so the peak/mean is never taken.
    state.check_for_failed_motor(
        &hexa_x(),
        &settled(&[0.01, 0.0, 0.0, 0.01, 0.0, 0.0], 1),
    );
    assert!(state.thrust_balanced());
    assert!(RPYT_SUM_MIN > 0.05);
}

#[test]
fn boost_drops_when_the_pack_has_headroom() {
    let mut state = FailedMotor::new();
    state.set_thrust_boost(true);
    let mut inputs = settled(&[0.4, 0.4, 0.4, 0.4, 0.4, 0.4], 1);
    inputs.throttle_thrust_max = 1.0;
    inputs.compensation_gain = 1.0;
    inputs.throttle_thrust_best_plus_adj = 0.5;
    state.check_for_failed_motor(&hexa_x(), &inputs);
    assert!(!state.thrust_boost());
    assert!(0.4 < BOOST_DROP_HIGH);
}

#[test]
fn boost_holds_while_a_motor_is_pegged() {
    let mut state = FailedMotor::new();
    state.set_thrust_boost(true);
    let mut inputs = settled(&[0.5, 0.5, 0.5, 0.95, 0.5, 0.5], 1);
    inputs.throttle_thrust_max = 1.0;
    inputs.compensation_gain = 1.0;
    inputs.throttle_thrust_best_plus_adj = 0.5;
    state.check_for_failed_motor(&hexa_x(), &inputs);
    assert!(
        state.thrust_boost(),
        "rpyt_high >= 0.9 must keep boost on"
    );
}

#[test]
fn a_fresh_unbalance_blocks_the_boost_drop_in_the_same_call() {
    // Order: the function updates `_thrust_balanced` and then asks it.
    // A trip this iteration must leave boost on even if the peak is
    // under 0.9 and throttle headroom is present.
    let mut state = FailedMotor::new();
    state.set_thrust_boost(true);
    let mut inputs = settled(&[0.2, 0.2, 0.2, 0.5, 0.2, 0.2], 1);
    inputs.throttle_thrust_best_plus_adj = 0.3;
    state.check_for_failed_motor(&hexa_x(), &inputs);
    assert!(!state.thrust_balanced());
    assert!(
        state.thrust_boost(),
        "boost drop must see the just-tripped flag"
    );
}

#[test]
fn rebalance_can_drop_boost_in_the_same_call() {
    let mut state = FailedMotor::new();
    state.check_for_failed_motor(&hexa_x(), &settled(&[0.4, 0.4, 0.4, 1.0, 0.4, 0.4], 1));
    assert!(!state.thrust_balanced());
    state.set_thrust_boost(true);
    let mut inputs = settled(&[0.4, 0.4, 0.4, 0.4, 0.4, 0.4], 1);
    inputs.throttle_thrust_best_plus_adj = 0.3;
    state.check_for_failed_motor(&hexa_x(), &inputs);
    assert!(state.thrust_balanced());
    assert!(
        !state.thrust_boost(),
        "a just-cleared pack with headroom must drop boost"
    );
}

#[test]
fn no_throttle_headroom_keeps_boost() {
    let mut state = FailedMotor::new();
    state.set_thrust_boost(true);
    let mut inputs = settled(&[0.4, 0.4, 0.4, 0.4, 0.4, 0.4], 1);
    // `_throttle_thrust_max * gain > throttle_thrust_best_plus_adj` fails
    // when the mixer is already using the whole ceiling.
    inputs.throttle_thrust_max = 0.5;
    inputs.compensation_gain = 1.0;
    inputs.throttle_thrust_best_plus_adj = 0.5;
    state.check_for_failed_motor(&hexa_x(), &inputs);
    assert!(state.thrust_boost());
}
