//! Setup leftovers, upstream `AP_MotorsMatrix` helpers after the PWM pass.
//! COP-005 leftover after `output_to_motors`.
//!
//! The factor table, frame layouts, mixer, failed-motor detector, and
//! PWM pass are on the crate. These are the remaining setup helpers:
//! scripting can rewrite one motor's throttle factor before the frame
//! is locked; `FRAME_CLASS` / `FRAME_TYPE` rebuild the table when the
//! vehicle is disarmed; yaw torque can be dropped for a vectored
//! airframe; examples can read the factors back; and a vehicle callback
//! can retouch `_thrust_rpyt_out` for a tiltrotor or tiltwing.
//!
//! Per ADR-0004 there is no singleton. `_active_frame_class`,
//! `_active_frame_type`, and `_initialised_ok` live on [`MotorSetup`].
//! `armed()` arrives as an argument. The callback arrives as an
//! argument too -- there is no stored functor on this leftover.
//!
//! `init`'s `set_update_rate` / `rc_set_freq` write is HAL and is not
//! this leftover. Scripting's dedicated `init(expected_num_motors)`
//! (MAV type + motor-count check) is not either: only the class/type
//! path that `set_frame_class_and_type` actually calls.

use crate::{MotorFactors, MotorMatrix, MAX_NUM_MOTORS};

/// Upstream `MOTOR_FRAME_SCRIPTING_MATRIX`.
///
/// A scripting frame skips `setup_motors` in `init` so Lua can add the
/// motors itself. `set_throttle_factor` only answers for this class,
/// and only before `initialised_ok` is set.
pub const FRAME_CLASS_SCRIPTING_MATRIX: u8 = 15;

/// Persistent setup state, upstream `_active_frame_class`,
/// `_active_frame_type`, and `_initialised_ok`.
///
/// Defaults match a zeroed C++ object: class and type 0
/// (`MOTOR_FRAME_UNDEFINED` / `MOTOR_FRAME_TYPE_PLUS`), not yet
/// initialised.
#[derive(Debug, Clone, Copy)]
pub struct MotorSetup {
    active_frame_class: u8,
    active_frame_type: u8,
    initialised_ok: bool,
}

impl Default for MotorSetup {
    fn default() -> Self {
        Self {
            active_frame_class: 0,
            active_frame_type: 0,
            initialised_ok: false,
        }
    }
}

impl MotorSetup {
    /// An uninitialised setup, class and type still 0.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `_active_frame_class`.
    #[must_use]
    pub fn active_frame_class(&self) -> u8 {
        self.active_frame_class
    }

    /// Upstream `_active_frame_type`.
    #[must_use]
    pub fn active_frame_type(&self) -> u8 {
        self.active_frame_type
    }

    /// Upstream `initialised_ok()`.
    #[must_use]
    pub fn initialised_ok(&self) -> bool {
        self.initialised_ok
    }

    /// Upstream `set_initialised_ok`. Scripting's dedicated init is
    /// what sets this after the motors have been added; tests use it
    /// to lock the table the same way.
    pub fn set_initialised_ok(&mut self, ok: bool) {
        self.initialised_ok = ok;
    }

    /// Rebuild the table for a new class/type, upstream
    /// `set_frame_class_and_type`.
    ///
    /// Armed, or the same class and type as last time, is a no-op.
    /// Otherwise the new pair is recorded and `init` runs: a scripting
    /// class stops there so Lua can add motors; every other class
    /// calls [`MotorMatrix::setup_motors`] and stores its success as
    /// `initialised_ok`.
    pub fn set_frame_class_and_type(
        &mut self,
        matrix: &mut MotorMatrix,
        armed: bool,
        frame_class: u8,
        frame_type: u8,
    ) {
        if armed || (frame_class == self.active_frame_class && frame_type == self.active_frame_type)
        {
            return;
        }
        self.active_frame_class = frame_class;
        self.active_frame_type = frame_type;
        self.init(matrix, frame_class, frame_type);
    }

    /// Upstream `AP_MotorsMatrix::init(frame_class, frame_type)`.
    ///
    /// Records the pair again (same writes `set_frame_class_and_type`
    /// just did) and either returns for a scripting frame or rebuilds
    /// the table. The PWM-rate write that follows `setup_motors`
    /// upstream is HAL and is not done here.
    fn init(&mut self, matrix: &mut MotorMatrix, frame_class: u8, frame_type: u8) {
        self.active_frame_class = frame_class;
        self.active_frame_type = frame_type;
        if frame_class == FRAME_CLASS_SCRIPTING_MATRIX {
            return;
        }
        self.initialised_ok = matrix.setup_motors(frame_class, frame_type);
    }

    /// Set one motor's throttle factor from scripting, upstream
    /// `set_throttle_factor`.
    ///
    /// Fails unless the active class is [`FRAME_CLASS_SCRIPTING_MATRIX`],
    /// the frame is not yet initialised, and the motor is fitted.
    /// Out-of-range motor numbers fail rather than index off the end --
    /// upstream has no bound check, which is UB; this port does what
    /// [`MotorMatrix::add_motor_raw`] does.
    ///
    /// The write is raw. It does not re-normalise.
    pub fn set_throttle_factor(
        &self,
        matrix: &mut MotorMatrix,
        motor_num: i8,
        throttle_factor: f32,
    ) -> bool {
        if self.active_frame_class != FRAME_CLASS_SCRIPTING_MATRIX {
            return false;
        }
        if self.initialised_ok || !matrix.is_enabled_i8(motor_num) {
            return false;
        }
        matrix.write_throttle_factor(motor_num, throttle_factor)
    }
}

/// Zero every yaw factor, fitted or not. Upstream `disable_yaw_torque`.
///
/// Used when an external mechanism such as vectoring owns yaw. The
/// loop does not consult `motor_enabled` -- a disabled slot's yaw is
/// cleared too.
pub fn disable_yaw_torque(matrix: &mut MotorMatrix) {
    matrix.disable_yaw_torque();
}

/// One motor's factors and test order, upstream `get_factors`.
///
/// `None` if the index is out of range or the motor is not fitted.
/// The C++ form writes five out-parameters and returns whether they
/// were filled.
#[must_use]
pub fn get_factors(matrix: &MotorMatrix, i: u8) -> Option<(MotorFactors, u8)> {
    matrix.get_factors(i)
}

/// Vehicle-supplied retouch of the mixer thrusts, upstream
/// `thrust_compensation`.
///
/// Tiltrotors and tiltwings install a callback that rewrites
/// `_thrust_rpyt_out` in place. No callback is a no-op. The length
/// argument upstream is always `AP_MOTORS_MAX_NUM_MOTORS`; the slice
/// here is that array, so the count is its length.
pub fn thrust_compensation(
    thrust_rpyt_out: &mut [f32; MAX_NUM_MOTORS],
    callback: Option<fn(&mut [f32; MAX_NUM_MOTORS])>,
) {
    if let Some(cb) = callback {
        cb(thrust_rpyt_out);
    }
}
