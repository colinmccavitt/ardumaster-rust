//! Armed-stabilizing mixer leftover — `output_armed_stabilizing` and
//! `boost_ratio`. COP-005.
//!
//! The frame tables already say what each motor contributes. These tests
//! check that the mixer uses those factors the way upstream does: hover
//! sits in the middle of the range, a yaw demand that will not fit is
//! clipped rather than allowed to saturate a motor, and compensation is
//! divided back out of `throttle_out`.

#![allow(
    clippy::float_cmp,
    reason = "limit flags and leftover catalog rows are exact; hover \
balance is checked with a tight epsilon against a known frame"
)]

use ap_motors::armed::{
    boost_ratio, output_armed_stabilizing, ArmedDemand, REMAINING, YAW_HEADROOM_DEFAULT,
};
use ap_motors::MotorMatrix;

fn quad_x() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1), "QUAD X");
    m
}

fn hover(throttle: f32) -> ArmedDemand {
    ArmedDemand {
        throttle,
        throttle_avg_max: throttle,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: YAW_HEADROOM_DEFAULT,
        ..ArmedDemand::default()
    }
}

#[test]
fn leftover_catalog_leaves_failed_motor_and_pwm_pass() {
    assert_eq!(
        REMAINING,
        [
            "check_for_failed_motor",
            "output_to_motors",
            "set_throttle_factor",
            "set_frame_class_and_type",
            "disable_yaw_torque",
            "get_factors",
            "thrust_compensation",
        ]
    );
}

#[test]
fn boost_ratio_pins_the_endpoints() {
    assert_eq!(boost_ratio(0.0, 0.9, 0.1), 0.1);
    assert_eq!(boost_ratio(1.0, 0.9, 0.1), 0.9);
}

#[test]
fn zero_throttle_raises_the_lower_limit_and_writes_nothing() {
    let out = output_armed_stabilizing(&quad_x(), &hover(0.0));
    assert!(out.limits.throttle_lower);
    assert!(!out.limits.throttle_upper);
    assert_eq!(out.throttle_out, 0.0);
    for i in 0..4 {
        assert!(
            out.get_thrust_rpyt_out(i) < 1e-6,
            "motor {i} {}",
            out.get_thrust_rpyt_out(i)
        );
    }
}

#[test]
fn a_hover_quad_sits_in_the_middle_of_the_range() {
    let out = output_armed_stabilizing(&quad_x(), &hover(0.5));
    assert!(!out.limits.roll);
    assert!(!out.limits.pitch);
    assert!(!out.limits.yaw);
    assert!(!out.limits.throttle_lower);
    assert!(!out.limits.throttle_upper);
    assert!((out.throttle_out - 0.5).abs() < 1e-5);
    for i in 0..4 {
        let t = out.get_thrust_rpyt_out(i);
        assert!(
            (t - 0.5).abs() < 1e-5,
            "motor {i} should hover at 0.5, got {t}"
        );
    }
    assert_eq!(out.get_thrust_rpyt_out(4), 0.0, "fifth slot stays empty");
}

#[test]
fn throttle_above_the_ceiling_is_clipped() {
    let demand = ArmedDemand {
        throttle: 1.0,
        throttle_avg_max: 1.0,
        throttle_thrust_max: 0.6,
        compensation_gain: 1.0,
        ..ArmedDemand::default()
    };
    let out = output_armed_stabilizing(&quad_x(), &demand);
    assert!(out.limits.throttle_upper);
    assert!((out.throttle_out - 0.6).abs() < 1e-5);
}

#[test]
fn a_yaw_that_will_not_fit_sets_the_yaw_limit() {
    // Pure yaw at hover still fits a quad: the normalised factors are
    // ±0.5, so a ±1 demand lands on 0 and 1. A roll demand already
    // using most of the room is what makes yaw miss.
    let demand = ArmedDemand {
        roll: 0.8,
        yaw: 1.0,
        throttle: 0.5,
        throttle_avg_max: 0.5,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 0,
        ..ArmedDemand::default()
    };
    let out = output_armed_stabilizing(&quad_x(), &demand);
    assert!(
        out.limits.yaw,
        "full-scale yaw on top of a hard roll must clip"
    );
    for i in 0..4 {
        let t = out.get_thrust_rpyt_out(i);
        assert!(
            (0.0..=1.0).contains(&t),
            "motor {i} left the range: {t}"
        );
    }
}

#[test]
fn a_roll_that_will_not_fit_scales_rpy_and_flags_the_axes() {
    let demand = ArmedDemand {
        roll: 1.0,
        pitch: 1.0,
        yaw: 1.0,
        throttle: 0.5,
        throttle_avg_max: 0.5,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 0,
        ..ArmedDemand::default()
    };
    let out = output_armed_stabilizing(&quad_x(), &demand);
    assert!(out.limits.roll);
    assert!(out.limits.pitch);
    assert!(out.limits.yaw);
    for i in 0..4 {
        let t = out.get_thrust_rpyt_out(i);
        assert!(
            (-0.01..=1.01).contains(&t),
            "motor {i} left the range: {t}"
        );
    }
}

#[test]
fn compensation_is_divided_back_out_of_throttle_out() {
    let demand = ArmedDemand {
        throttle: 0.4,
        throttle_avg_max: 0.4,
        throttle_thrust_max: 1.0,
        compensation_gain: 2.0,
        ..ArmedDemand::default()
    };
    let out = output_armed_stabilizing(&quad_x(), &demand);
    assert!(
        (out.throttle_out - 0.4).abs() < 1e-5,
        "notch throttle must be uncompensated, got {}",
        out.throttle_out
    );
    for i in 0..4 {
        let t = out.get_thrust_rpyt_out(i);
        assert!(
            (t - 0.8).abs() < 1e-5,
            "motor {i} should see the compensated hover 0.8, got {t}"
        );
    }
}

#[test]
fn thrust_boost_lets_the_lost_motor_exceed_the_pack() {
    // Mark the motor that is already working hardest. Excluding it from
    // `rpy_high` is what lets that slot run past the pack — the point of
    // `_thrust_boost`.
    let probe = ArmedDemand {
        roll: 1.0,
        pitch: 1.0,
        throttle: 0.5,
        throttle_avg_max: 0.5,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 0,
        ..ArmedDemand::default()
    };
    let normal = output_armed_stabilizing(&quad_x(), &probe);
    let lost_index = (0..4_u8)
        .max_by(|a, b| {
            normal
                .get_thrust_rpyt_out(*a)
                .total_cmp(&normal.get_thrust_rpyt_out(*b))
        })
        .expect("quad has motors");
    let boosted = output_armed_stabilizing(
        &quad_x(),
        &ArmedDemand {
            thrust_boost: true,
            thrust_boost_ratio: 1.0,
            motor_lost_index: lost_index,
            ..probe
        },
    );
    let lost = boosted.get_thrust_rpyt_out(lost_index);
    let packed = (0..4_u8)
        .filter(|&i| i != lost_index)
        .map(|i| boosted.get_thrust_rpyt_out(i))
        .fold(0.0_f32, f32::max);
    assert!(
        lost + 1e-5 >= packed,
        "lost motor {lost_index} at {lost} should be free to run at least as hard as the pack {packed}"
    );
    assert!(
        lost + 1e-5 >= normal.get_thrust_rpyt_out(lost_index),
        "boost must not hold the lost motor below its unboosted output"
    );
}

#[test]
fn an_empty_frame_does_not_write_thrusts() {
    let out = output_armed_stabilizing(&MotorMatrix::new(), &hover(0.5));
    for i in 0_u8..32 {
        assert_eq!(out.get_thrust_rpyt_out(i), 0.0, "motor {i}");
    }
}

#[test]
fn throttle_factors_scale_the_collective() {
    let mut m = MotorMatrix::new();
    m.add_motor_raw(0, 0.5, 0.5, 0.5, 1, 1.0);
    m.add_motor_raw(1, -0.5, -0.5, -0.5, 2, 0.5);
    m.normalise_rpy_factors();
    let out = output_armed_stabilizing(&m, &hover(0.5));
    let a = out.get_thrust_rpyt_out(0);
    let b = out.get_thrust_rpyt_out(1);
    assert!(
        a > b,
        "full-throttle-factor motor {a} must sit above the half-factor one {b}"
    );
}
