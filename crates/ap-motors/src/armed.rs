//! Armed-stabilizing mixer, upstream `AP_MotorsMatrix::output_armed_stabilizing`.
//! COP-005 leftover after the frame tables.
//!
//! The tables in [`crate::MotorMatrix`] say what each motor contributes.
//! This is the function that uses them: it takes a roll / pitch / yaw /
//! throttle demand, fits it into the 0..1 range the ESCs can actually
//! produce, and writes one thrust per motor.
//!
//! Voltage and air-density compensation arrive as a single gain (upstream
//! `thr_lin.get_compensation_gain`). The mixer scales the demand by that
//! gain, then divides it back out of `_throttle_out` so the harmonic
//! notch still sees the uncompensated throttle.
//!
//! `check_for_failed_motor` is called at the end of the upstream
//! function; that leftover lives in [`crate::failed_motor`]. The matrix
//! `output_to_motors` that turns `_thrust_rpyt_out` into PWM lives in
//! [`crate::output_to_motors`].

use ap_math::scalar::{constrain_value, is_positive, is_zero};

use crate::spool::Limits;
use crate::{MotorMatrix, MAX_NUM_MOTORS};

/// Default `MOT_YAW_HEADROOM`, upstream `AP_MOTORS_YAW_HEADROOM_DEFAULT`.
pub const YAW_HEADROOM_DEFAULT: i16 = 200;

/// Remaining COP-005 leftovers after the mixer, failed-motor, PWM, and
/// setup-helper slices.
///
/// Frame tables, the factor model, this mixer,
/// `check_for_failed_motor`, `output_to_motors`, and the setup helpers
/// (`set_throttle_factor`, `set_frame_class_and_type`,
/// `disable_yaw_torque`, `get_factors`, `thrust_compensation`) are on
/// the crate. The catalog is empty.
pub const REMAINING: &[&str] = &[];

/// Blend between a failed-motor value and the normal one, upstream
/// `AP_MotorsMatrix::boost_ratio`.
///
/// `_thrust_boost_ratio` of 1 returns `boost_value`; 0 returns
/// `normal_value`. The mix is linear. The `1.0` is a float — ArduPilot
/// builds with `-fsingle-precision-constant`, so the complement never
/// promotes to double.
#[must_use]
pub fn boost_ratio(thrust_boost_ratio: f32, boost_value: f32, normal_value: f32) -> f32 {
    thrust_boost_ratio * boost_value + (1.0 - thrust_boost_ratio) * normal_value
}

/// What the attitude controller asked for this iteration.
///
/// Per ADR-0004 there is no singleton. Upstream reads these off the
/// motors object (`_roll_in`, `get_throttle`, `_yaw_headroom`, …); they
/// arrive here as one struct so the mixer can be tested without
/// constructing `AP_MotorsMatrix`.
#[derive(Debug, Clone, Copy)]
pub struct ArmedDemand {
    /// `_roll_in + _roll_in_ff`, ±1.
    pub roll: f32,
    /// `_pitch_in + _pitch_in_ff`, ±1.
    pub pitch: f32,
    /// `_yaw_in + _yaw_in_ff`, ±1.
    pub yaw: f32,
    /// Filtered throttle, upstream `get_throttle()`, 0..1.
    pub throttle: f32,
    /// `_throttle_avg_max` before compensation, 0..1.
    pub throttle_avg_max: f32,
    /// The spool's moving ceiling, upstream `_throttle_thrust_max`.
    pub throttle_thrust_max: f32,
    /// `thr_lin.get_compensation_gain()`. Never zero upstream.
    pub compensation_gain: f32,
    /// `MOT_YAW_HEADROOM` in PWM microseconds. Multiplied by 0.001 to
    /// become a 0..1 fraction. Default [`YAW_HEADROOM_DEFAULT`].
    pub yaw_headroom: i16,
    /// Whether failed-motor handling is active, upstream `_thrust_boost`.
    pub thrust_boost: bool,
    /// How far that handling has slewed in, 0..1.
    pub thrust_boost_ratio: f32,
    /// Index of the motor treated as lost while boost is active.
    pub motor_lost_index: u8,
}

impl Default for ArmedDemand {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            throttle_avg_max: 0.0,
            throttle_thrust_max: 1.0,
            compensation_gain: 1.0,
            yaw_headroom: YAW_HEADROOM_DEFAULT,
            thrust_boost: false,
            thrust_boost_ratio: 0.0,
            motor_lost_index: 0,
        }
    }
}

/// What the mixer wrote, including the per-motor thrusts and the
/// limit flags it raised.
#[derive(Debug, Clone, Copy)]
pub struct ArmedOutput {
    /// `_thrust_rpyt_out` — one slot per motor number.
    pub thrust_rpyt_out: [f32; MAX_NUM_MOTORS],
    /// `_throttle_out` after dividing compensation back out.
    pub throttle_out: f32,
    /// Axes that could not be met in full.
    pub limits: Limits,
}

impl ArmedOutput {
    /// Upstream `get_thrust_rpyt_out`.
    ///
    /// Disabled slots stay 0.0 because this function starts from a
    /// zeroed array; a later leftover that persists the array across
    /// calls would keep the last written value instead.
    #[must_use]
    pub fn get_thrust_rpyt_out(&self, i: u8) -> f32 {
        self.thrust_rpyt_out
            .get(usize::from(i))
            .copied()
            .unwrap_or(0.0)
    }
}

/// Mix one demand into per-motor thrusts, upstream
/// `output_armed_stabilizing`.
///
/// `check_for_failed_motor` is deliberately not called. That leftover
/// now lives in [`crate::failed_motor`]; wiring it in here would hide
/// the filter state the caller has to own.
#[must_use]
pub fn output_armed_stabilizing(matrix: &MotorMatrix, demand: &ArmedDemand) -> ArmedOutput {
    let mut thrust_rpyt_out = [0.0_f32; MAX_NUM_MOTORS];
    let mut limits = Limits::default();

    let compensation_gain = demand.compensation_gain;
    let roll_thrust = demand.roll * compensation_gain;
    let pitch_thrust = demand.pitch * compensation_gain;
    let mut yaw_thrust = demand.yaw * compensation_gain;
    let mut throttle_thrust = demand.throttle * compensation_gain;
    let mut throttle_avg_max = demand.throttle_avg_max * compensation_gain;
    let throttle_thrust_max = boost_ratio(
        demand.thrust_boost_ratio,
        1.0,
        demand.throttle_thrust_max * compensation_gain,
    );

    if throttle_thrust <= 0.0 {
        throttle_thrust = 0.0;
        limits.throttle_lower = true;
    }
    if throttle_thrust >= throttle_thrust_max {
        throttle_thrust = throttle_thrust_max;
        limits.throttle_upper = true;
    }

    throttle_avg_max = constrain_value(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    let mut throttle_thrust_best_rpy = 0.5_f32.min(throttle_avg_max);

    let mut yaw_allowed = 1.0_f32;
    for i in 0..MAX_NUM_MOTORS {
        let Some(f) = matrix.motor(i) else {
            continue;
        };
        let Some(slot) = thrust_rpyt_out.get_mut(i) else {
            continue;
        };
        *slot = roll_thrust * f.roll + pitch_thrust * f.pitch;

        let lost = demand.thrust_boost && i == usize::from(demand.motor_lost_index);
        if !is_zero(f.yaw) && !lost {
            let thrust_rp_best_throttle = throttle_thrust_best_rpy + *slot;
            let motor_room = if is_positive(yaw_thrust * f.yaw) {
                1.0 - thrust_rp_best_throttle
            } else {
                thrust_rp_best_throttle
            };
            let motor_yaw_allowed = motor_room.max(0.0) / f.yaw.abs();
            yaw_allowed = yaw_allowed.min(motor_yaw_allowed);
        }
    }

    let mut yaw_allowed_min = f32::from(demand.yaw_headroom) * 0.001;
    yaw_allowed_min = boost_ratio(demand.thrust_boost_ratio, 0.5, yaw_allowed_min);
    yaw_allowed = yaw_allowed.max(yaw_allowed_min);

    if demand.thrust_boost {
        if let (Some(f), Some(&out)) = (
            matrix.motor(usize::from(demand.motor_lost_index)),
            thrust_rpyt_out.get(usize::from(demand.motor_lost_index)),
        ) {
            if !is_zero(f.yaw) {
                let thrust_rp_best_throttle = throttle_thrust_best_rpy + out;
                let motor_room = if is_positive(yaw_thrust * f.yaw) {
                    1.0 - thrust_rp_best_throttle
                } else {
                    thrust_rp_best_throttle
                };
                let motor_yaw_allowed = motor_room.max(0.0) / f.yaw.abs();
                yaw_allowed = boost_ratio(
                    demand.thrust_boost_ratio,
                    yaw_allowed,
                    yaw_allowed.min(motor_yaw_allowed),
                );
            }
        }
    }

    if yaw_thrust.abs() > yaw_allowed {
        yaw_thrust = constrain_value(yaw_thrust, -yaw_allowed, yaw_allowed);
        limits.yaw = true;
    }

    let mut rpy_low = 1.0_f32;
    let mut rpy_high = -1.0_f32;
    for i in 0..MAX_NUM_MOTORS {
        let Some(f) = matrix.motor(i) else {
            continue;
        };
        let Some(slot) = thrust_rpyt_out.get_mut(i) else {
            continue;
        };
        *slot += yaw_thrust * f.yaw;
        if *slot < rpy_low {
            rpy_low = *slot;
        }
        let lost = demand.thrust_boost && i == usize::from(demand.motor_lost_index);
        if *slot > rpy_high && !lost {
            rpy_high = *slot;
        }
    }

    if demand.thrust_boost {
        if let Some(&lost_out) = thrust_rpyt_out.get(usize::from(demand.motor_lost_index)) {
            if lost_out > rpy_high && matrix.is_enabled(usize::from(demand.motor_lost_index)) {
                rpy_high = boost_ratio(demand.thrust_boost_ratio, rpy_high, lost_out);
            }
        }
    }

    let mut rpy_scale = 1.0_f32;
    if rpy_high - rpy_low > 1.0 {
        rpy_scale = 1.0 / (rpy_high - rpy_low);
    }
    if throttle_avg_max + rpy_low < 0.0 {
        rpy_scale = rpy_scale.min(-throttle_avg_max / rpy_low);
    }

    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    let mut thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if rpy_scale < 1.0 {
        limits.set_rpy(true);
        if thr_adj > 0.0 {
            limits.throttle_upper = true;
        }
        thr_adj = 0.0;
    } else if thr_adj < 0.0 {
        thr_adj = 0.0;
    } else if thr_adj > 1.0 - (throttle_thrust_best_rpy + rpy_high) {
        thr_adj = 1.0 - (throttle_thrust_best_rpy + rpy_high);
        limits.throttle_upper = true;
    }

    let throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    for i in 0..MAX_NUM_MOTORS {
        let Some(f) = matrix.motor(i) else {
            continue;
        };
        let Some(slot) = thrust_rpyt_out.get_mut(i) else {
            continue;
        };
        *slot = (throttle_thrust_best_plus_adj * f.throttle) + (rpy_scale * *slot);
    }

    let throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    ArmedOutput {
        thrust_rpyt_out,
        throttle_out,
        limits,
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "boost_ratio at the endpoints must return the input exactly"
    )]

    use super::*;

    #[test]
    fn boost_ratio_is_the_linear_blend() {
        assert_eq!(boost_ratio(0.0, 1.0, 0.2), 0.2);
        assert_eq!(boost_ratio(1.0, 1.0, 0.2), 1.0);
        assert!((boost_ratio(0.5, 1.0, 0.0) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn remaining_is_empty_after_the_setup_helpers() {
        assert!(REMAINING.is_empty());
        assert!(!REMAINING.contains(&"check_for_failed_motor"));
        assert!(!REMAINING.contains(&"output_to_motors"));
        assert!(!REMAINING.contains(&"output_armed_stabilizing"));
        assert!(!REMAINING.contains(&"setup_motors"));
        assert!(!REMAINING.contains(&"set_throttle_factor"));
        assert!(!REMAINING.contains(&"set_frame_class_and_type"));
        assert!(!REMAINING.contains(&"disable_yaw_torque"));
        assert!(!REMAINING.contains(&"get_factors"));
        assert!(!REMAINING.contains(&"thrust_compensation"));
    }
}
