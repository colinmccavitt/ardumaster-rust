//! Matrix PWM pass, upstream `AP_MotorsMatrix::output_to_motors`.
//! COP-005 leftover after the failed-motor slice.
//!
//! The mixer writes `_thrust_rpyt_out`. This is the function that turns
//! those thrusts into actuator values and PWM: shut-down zeros the
//! actuators, ground-idle slews toward spin-up idle, and the three
//! flying spool states slew toward `thr_lin.thrust_to_actuator` of each
//! mixer thrust. Then every enabled motor is written through
//! `output_to_pwm` + `rc_write`.
//!
//! Per ADR-0004 there is no singleton. The persistent `_actuator` array
//! lives on [`OutputToMotors`] because slew is a step from the last
//! written value. Spool state, slew times, PWM endpoints, spin-up
//! ratio, the mixer thrusts, thrust-linearization, and the tilt-quad
//! override mask arrive as [`OutputToMotorsInputs`].
//!
//! `SHUT_DOWN` uses `motor_enabled_mask` (enabled and not overridden);
//! the other spool cases and the PWM write loop use `motor_enabled`
//! alone. That is not a transcription slip: tilt-quadplanes leave an
//! overridden motor's actuator alone while the frame is shut down so
//! the next ground-idle slew starts from the last flying value.

use crate::output::{
    actuator_spin_up_to_ground_idle, output_to_pwm, rc_write, set_actuator_with_slew,
    MotorPwmScaled, PwmParams, RcWrite, SlewParams,
};
use crate::spool::SpoolState;
use crate::thrust_linearization::{ThrustLinParams, ThrustLinearization};
use crate::{MotorMatrix, MAX_NUM_MOTORS};

/// What the mixer, spool, and ESC setup wrote this iteration.
///
/// `thr_lin` and `thr_lin_params` are owned copies because
/// [`ThrustLinearization`] is `Copy` and this leftover only *reads*
/// `thrust_to_actuator` / `spin_min`.
#[derive(Debug, Clone, Copy)]
pub struct OutputToMotorsInputs {
    /// Upstream `_spool_state`.
    pub spool_state: SpoolState,
    /// Upstream `armed()`.
    pub armed: bool,
    /// `_thrust_rpyt_out` from the mixer.
    pub thrust_rpyt_out: [f32; MAX_NUM_MOTORS],
    /// Loop time, upstream `_dt_s`.
    pub dt_s: f32,
    /// `MOT_SLEW_UP_TIME` / `MOT_SLEW_DN_TIME`.
    pub slew: SlewParams,
    /// `MOT_PWM_MIN` / `MOT_PWM_MAX` / `MOT_SAFE_DISARM`.
    pub pwm: PwmParams,
    /// `_spin_up_ratio`, 0..1 (may overshoot one step; the idle helper clamps).
    pub spin_up_ratio: f32,
    /// Tilt-quad override mask, upstream `_motor_mask_override`. Zero on a
    /// copter. `SHUT_DOWN` skips zeroing a set bit; the other states and
    /// the PWM write do not consult it.
    pub motor_mask_override: u32,
    /// Scaled-output routing for `rc_write`.
    pub scaled: MotorPwmScaled,
    /// Thrust-curve + voltage compensation. Default is no voltage sag.
    pub thr_lin: ThrustLinearization,
    /// `THST_EXPO` / `SPIN_MIN` / `SPIN_MAX` and the battery bounds.
    pub thr_lin_params: ThrustLinParams,
}

impl Default for OutputToMotorsInputs {
    fn default() -> Self {
        Self {
            spool_state: SpoolState::ThrottleUnlimited,
            armed: true,
            thrust_rpyt_out: [0.0; MAX_NUM_MOTORS],
            dt_s: 0.0025,
            slew: SlewParams {
                slew_up_time: 0.0,
                slew_dn_time: 0.0,
            },
            pwm: analog_pwm(),
            spin_up_ratio: 0.0,
            motor_mask_override: 0,
            scaled: MotorPwmScaled::default(),
            thr_lin: ThrustLinearization::new(),
            thr_lin_params: ThrustLinParams::default(),
        }
    }
}

/// Analog 1000..2000 endpoints, the non-digital default.
#[must_use]
pub fn analog_pwm() -> PwmParams {
    PwmParams {
        pwm_min: 1000,
        pwm_max: 2000,
        disarm_disable_pwm: false,
        pwm_min_default: 1000,
        pwm_max_default: 2000,
    }
}

/// Enabled and not overridden, upstream `motor_enabled_mask`.
///
/// `_motor_mask_override` is only set for tilt quadplanes. A copter
/// passes 0 and this is just [`MotorMatrix::is_enabled`].
#[must_use]
pub fn motor_enabled_mask(matrix: &MotorMatrix, i: usize, override_mask: u32) -> bool {
    if !matrix.is_enabled(i) {
        return false;
    }
    let Ok(bit) = u8::try_from(i) else {
        return false;
    };
    if bit >= 32 {
        return false;
    }
    (override_mask & (1_u32 << bit)) == 0
}

/// One motor's write, or `None` if that motor is not fitted.
pub type MotorWrites = [Option<RcWrite>; MAX_NUM_MOTORS];

/// Persistent actuator array for `output_to_motors`.
///
/// Starts the way the C++ constructor leaves `_actuator`: zero.
#[derive(Debug, Clone, Copy)]
pub struct OutputToMotors {
    actuator: [f32; MAX_NUM_MOTORS],
}

impl Default for OutputToMotors {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputToMotors {
    /// Constructor default: `_actuator` at zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            actuator: [0.0; MAX_NUM_MOTORS],
        }
    }

    /// One slot of `_actuator`, or 0.0 if the index is out of range.
    #[must_use]
    pub fn actuator(&self, i: u8) -> f32 {
        self.actuator.get(usize::from(i)).copied().unwrap_or(0.0)
    }

    /// Slew `_actuator` from the mixer thrusts and write PWM, upstream
    /// `output_to_motors`.
    ///
    /// Disabled slots are not touched and produce `None` in the write
    /// array. An overridden motor is skipped only in `SHUT_DOWN`; the
    /// PWM loop still writes it if the slot is enabled.
    pub fn output_to_motors(
        &mut self,
        matrix: &MotorMatrix,
        inputs: &OutputToMotorsInputs,
    ) -> MotorWrites {
        match inputs.spool_state {
            SpoolState::ShutDown => {
                for i in 0..MAX_NUM_MOTORS {
                    if motor_enabled_mask(matrix, i, inputs.motor_mask_override) {
                        if let Some(slot) = self.actuator.get_mut(i) {
                            *slot = 0.0;
                        }
                    }
                }
            }
            SpoolState::GroundIdle => {
                let idle = actuator_spin_up_to_ground_idle(
                    inputs.spin_up_ratio,
                    inputs.thr_lin_params.spin_min,
                );
                for i in 0..MAX_NUM_MOTORS {
                    if !matrix.is_enabled(i) {
                        continue;
                    }
                    if let Some(slot) = self.actuator.get_mut(i) {
                        set_actuator_with_slew(slot, idle, inputs.dt_s, &inputs.slew);
                    }
                }
            }
            SpoolState::SpoolingUp
            | SpoolState::ThrottleUnlimited
            | SpoolState::SpoolingDown => {
                for i in 0..MAX_NUM_MOTORS {
                    if !matrix.is_enabled(i) {
                        continue;
                    }
                    let Some(&thrust) = inputs.thrust_rpyt_out.get(i) else {
                        continue;
                    };
                    let target = inputs
                        .thr_lin
                        .thrust_to_actuator(&inputs.thr_lin_params, thrust);
                    if let Some(slot) = self.actuator.get_mut(i) {
                        set_actuator_with_slew(slot, target, inputs.dt_s, &inputs.slew);
                    }
                }
            }
        }

        let mut writes = [None; MAX_NUM_MOTORS];
        for i in 0..MAX_NUM_MOTORS {
            if !matrix.is_enabled(i) {
                continue;
            }
            let Some(&actuator) = self.actuator.get(i) else {
                continue;
            };
            let pwm_i = output_to_pwm(inputs.spool_state, inputs.armed, &inputs.pwm, actuator);
            // C++ passes int16 into uint16. Valid endpoints are
            // non-negative; a negative pulse is not a thing an ESC sees.
            let pwm = u16::try_from(pwm_i.max(0)).unwrap_or(0);
            if let (Ok(chan), Some(slot)) = (u8::try_from(i), writes.get_mut(i)) {
                *slot = Some(rc_write(chan, pwm, &inputs.scaled));
            }
        }
        writes
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "catalog flags and shutdown zeros are exact; actuator \
steps are checked against the already-ported helpers"
    )]

    use super::*;
    use crate::armed::REMAINING;

    fn quad_x() -> MotorMatrix {
        let mut m = MotorMatrix::new();
        assert!(m.setup_motors(1, 1), "QUAD X");
        m
    }

    #[test]
    fn constructor_starts_actuators_at_zero() {
        let s = OutputToMotors::new();
        assert_eq!(s.actuator(0), 0.0);
        assert_eq!(s.actuator(31), 0.0);
        assert_eq!(s.actuator(99), 0.0);
    }

    #[test]
    fn leftover_catalog_drops_the_pwm_pass() {
        assert!(!REMAINING.contains(&"output_to_motors"));
        assert!(!REMAINING.contains(&"check_for_failed_motor"));
        assert!(REMAINING.contains(&"set_throttle_factor"));
        assert!(REMAINING.contains(&"thrust_compensation"));
    }

    #[test]
    fn override_mask_is_just_enabled_when_empty() {
        let m = quad_x();
        assert!(motor_enabled_mask(&m, 0, 0));
        assert!(!motor_enabled_mask(&m, 4, 0));
        assert!(!motor_enabled_mask(&m, 0, 1));
        assert!(motor_enabled_mask(&m, 1, 1));
    }
}
