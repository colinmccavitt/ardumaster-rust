//! Failed-motor detection, upstream `AP_MotorsMatrix::check_for_failed_motor`.
//! COP-005 leftover after the armed-stabilizing mixer.
//!
//! The mixer writes one thrust per motor. This is the function that watches
//! those thrusts for a motor that has stopped contributing: it filters them,
//! compares the loudest slot to the mean, and — on a six-or-more-motor frame
//! that is not co-rotating — trips `_thrust_balanced` when one motor is
//! asking for far more than the others.
//!
//! It does **not** turn thrust-boost on. That is the vehicle crash-check
//! (`set_thrust_boost(true)`). This function only names the lost motor while
//! boost is off, and turns boost *off* again once the remaining motors have
//! headroom and the pack looks balanced.
//!
//! Per ADR-0004 there is no singleton. Upstream reads `_thrust_rpyt_out`,
//! `_dt_s`, `_throttle_thrust_max`, the compensation gain, and
//! `_active_frame_type` off the motors object; they arrive here as
//! [`FailedMotorInputs`]. The filtered array and the three flags live on
//! [`FailedMotor`].
//!
//! The header comment on `_thrust_rpyt_out_filt` says "1 second time
//! constant". The code uses `0.5f`. The code wins; see
//! [`FILTER_TIME_CONSTANT_S`].

use crate::{MotorMatrix, MAX_NUM_MOTORS};

/// The `0.5f` in `_dt_s / (_dt_s + 0.5f)`.
///
/// A float, not a double — ArduPilot builds with
/// `-fsingle-precision-constant`. Writing `0.5` here as f64 would promote
/// the denominator and put `alpha` a few ulp out.
pub const FILTER_TIME_CONSTANT_S: f32 = 0.5;

/// Filtered-sum below which `thrust_balance` is forced to 1.0.
///
/// Upstream's `rpyt_sum > 0.1f` gate. Near-zero thrusts would otherwise
/// make the peak/mean ratio explode.
pub const RPYT_SUM_MIN: f32 = 0.1;

/// Peak-over-mean that declares the pack unbalanced.
pub const UNBALANCE_THRESHOLD: f32 = 1.5;

/// Peak-over-mean that clears a previous unbalance.
pub const REBALANCE_THRESHOLD: f32 = 1.25;

/// Filtered peak below which, with throttle headroom, boost is dropped.
pub const BOOST_DROP_HIGH: f32 = 0.9;

/// Fewest enabled motors that may trip unbalance.
///
/// A quad that loses a motor cannot redistribute; the check is reserved
/// for hexa and above.
pub const MIN_MOTORS_FOR_UNBALANCE: u8 = 6;

/// `MOTOR_FRAME_TYPE_X_COR` — X8 co-rotating, old motor ordering.
pub const FRAME_TYPE_X_COR: u8 = 20;

/// `MOTOR_FRAME_TYPE_CW_X_COR` — X8 co-rotating, clockwise ordering.
pub const FRAME_TYPE_CW_X_COR: u8 = 21;

/// Whether this frame type skips the unbalance trip, upstream
/// `is_corotating`.
///
/// Co-rotating X8 frames scale their top rotor layer by 0.9, so their
/// throttle factors differ on purpose. Treating that as a failed motor
/// would fire on every hover.
#[must_use]
pub fn is_corotating_frame(frame_type: u8) -> bool {
    frame_type == FRAME_TYPE_X_COR || frame_type == FRAME_TYPE_CW_X_COR
}

/// What the mixer and spool wrote this iteration.
///
/// `throttle_thrust_best_plus_adj` is the first argument of the C++
/// function: `throttle_thrust_best_rpy + thr_adj`, which is also
/// `throttle_out * compensation_gain` after the mixer returns.
#[derive(Debug, Clone, Copy)]
pub struct FailedMotorInputs {
    /// Loop time, upstream `_dt_s`.
    pub dt_s: f32,
    /// `_thrust_rpyt_out` from the mixer.
    pub thrust_rpyt_out: [f32; MAX_NUM_MOTORS],
    /// Mixer's `throttle_thrust_best_plus_adj`.
    pub throttle_thrust_best_plus_adj: f32,
    /// Spool ceiling, upstream `_throttle_thrust_max`.
    pub throttle_thrust_max: f32,
    /// `thr_lin.get_compensation_gain()`.
    pub compensation_gain: f32,
    /// `_active_frame_type` as the `FRAME_TYPE` integer.
    pub frame_type: u8,
}

impl Default for FailedMotorInputs {
    fn default() -> Self {
        Self {
            dt_s: 0.0025,
            thrust_rpyt_out: [0.0; MAX_NUM_MOTORS],
            throttle_thrust_best_plus_adj: 0.5,
            throttle_thrust_max: 1.0,
            compensation_gain: 1.0,
            frame_type: 1,
        }
    }
}

/// Persistent state for `check_for_failed_motor`.
///
/// Starts the way the C++ constructor leaves the members: boost off,
/// balanced on, lost-motor index 0, filter at zero.
#[derive(Debug, Clone, Copy)]
pub struct FailedMotor {
    thrust_rpyt_out_filt: [f32; MAX_NUM_MOTORS],
    thrust_balanced: bool,
    thrust_boost: bool,
    motor_lost_index: u8,
}

impl Default for FailedMotor {
    fn default() -> Self {
        Self::new()
    }
}

impl FailedMotor {
    /// Constructor defaults: `_thrust_boost = false`, `_thrust_balanced =
    /// true`, filter and lost-index at zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            thrust_rpyt_out_filt: [0.0; MAX_NUM_MOTORS],
            thrust_balanced: true,
            thrust_boost: false,
            motor_lost_index: 0,
        }
    }

    /// Filtered thrust for one slot, upstream `_thrust_rpyt_out_filt[i]`.
    #[must_use]
    pub fn thrust_rpyt_out_filt(&self, i: u8) -> f32 {
        self.thrust_rpyt_out_filt
            .get(usize::from(i))
            .copied()
            .unwrap_or(0.0)
    }

    /// Whether the pack looks balanced, upstream `_thrust_balanced`.
    #[must_use]
    pub fn thrust_balanced(&self) -> bool {
        self.thrust_balanced
    }

    /// Whether failed-motor handling is active, upstream `_thrust_boost`.
    #[must_use]
    pub fn thrust_boost(&self) -> bool {
        self.thrust_boost
    }

    /// Index of the motor treated as lost, upstream `_motor_lost_index`.
    #[must_use]
    pub fn motor_lost_index(&self) -> u8 {
        self.motor_lost_index
    }

    /// Vehicle crash-check path, upstream `set_thrust_boost`.
    ///
    /// This leftover never sets the flag to true. Crash-check does, then
    /// this function may clear it once the remaining motors have room.
    pub fn set_thrust_boost(&mut self, enable: bool) {
        self.thrust_boost = enable;
    }

    /// Filter the mixer outputs and decide balance / boost / lost index,
    /// upstream `check_for_failed_motor`.
    pub fn check_for_failed_motor(&mut self, matrix: &MotorMatrix, inputs: &FailedMotorInputs) {
        let alpha = inputs.dt_s / (inputs.dt_s + FILTER_TIME_CONSTANT_S);
        for i in 0..MAX_NUM_MOTORS {
            if !matrix.is_enabled(i) {
                continue;
            }
            let Some(&raw) = inputs.thrust_rpyt_out.get(i) else {
                continue;
            };
            let Some(filt) = self.thrust_rpyt_out_filt.get_mut(i) else {
                continue;
            };
            *filt += alpha * (raw - *filt);
        }

        let mut rpyt_high = 0.0_f32;
        let mut rpyt_sum = 0.0_f32;
        let mut number_motors: u8 = 0;
        for i in 0..MAX_NUM_MOTORS {
            if !matrix.is_enabled(i) {
                continue;
            }
            let Some(&filt) = self.thrust_rpyt_out_filt.get(i) else {
                continue;
            };
            number_motors = number_motors.saturating_add(1);
            rpyt_sum += filt;
            if filt > rpyt_high {
                rpyt_high = filt;
                if !self.thrust_boost {
                    if let Ok(idx) = u8::try_from(i) {
                        self.motor_lost_index = idx;
                    }
                }
            }
        }

        let mut thrust_balance = 1.0_f32;
        if rpyt_sum > RPYT_SUM_MIN {
            thrust_balance = rpyt_high * f32::from(number_motors) / rpyt_sum;
        }

        if number_motors >= MIN_MOTORS_FOR_UNBALANCE
            && thrust_balance >= UNBALANCE_THRESHOLD
            && self.thrust_balanced
            && !is_corotating_frame(inputs.frame_type)
        {
            self.thrust_balanced = false;
        }
        if thrust_balance <= REBALANCE_THRESHOLD && !self.thrust_balanced {
            self.thrust_balanced = true;
        }

        if inputs.throttle_thrust_max * inputs.compensation_gain
            > inputs.throttle_thrust_best_plus_adj
            && rpyt_high < BOOST_DROP_HIGH
            && self.thrust_balanced
        {
            self.thrust_boost = false;
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "filter endpoints and catalog flags are exact; the \
first-order step is checked against the closed form"
    )]

    use super::*;

    #[test]
    fn constructor_matches_the_cpp_defaults() {
        let s = FailedMotor::new();
        assert!(!s.thrust_boost());
        assert!(s.thrust_balanced());
        assert_eq!(s.motor_lost_index(), 0);
        assert_eq!(s.thrust_rpyt_out_filt(0), 0.0);
    }

    #[test]
    fn corotating_types_are_the_x8_pair() {
        assert!(!is_corotating_frame(1));
        assert!(is_corotating_frame(FRAME_TYPE_X_COR));
        assert!(is_corotating_frame(FRAME_TYPE_CW_X_COR));
    }
}
