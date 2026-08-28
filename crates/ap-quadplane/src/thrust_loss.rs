//! Leftover thrust-loss / ESC-cal / takeoff-failure stub, upstream
//! `QuadPlane::thrust_loss_check` / `run_esc_calibration` /
//! `takeoff_failure_scalar` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. Plane owns AHRS velocity, attitude-control
//! targets, and the motors thrust-boost latch; the caller passes a
//! [`ThrustLossView`] / [`EscCalView`]. This is not a rewrite of
//! [`crate::motors_output`] (which only reports the inactive window),
//! [`crate::auto_vtol`] `do_vtol_takeoff` / `verify_vtol_takeoff`, or
//! the leftover TECS / stick-mix / stopping-distance row.

use crate::quadplane_completeness::{
    esc_cal_passthrough, takeoff_failure_timed_out, thrust_loss_already_engaged_or_idle,
    thrust_loss_attitude_lost, thrust_loss_disabled, thrust_loss_not_descending,
    thrust_loss_option_is_set, thrust_loss_throttle_not_saturated, thrust_loss_throttle_too_low,
    thrust_loss_tilt_too_steep, thrust_loss_vtol_only_skip, ThrustLossOption,
};
use crate::QuadPlane;

/// Default `Q_ESC_CAL`, upstream `AP_GROUPINFO("ESC_CAL", ..., 0)`.
pub const Q_ESC_CAL_DEFAULT: i8 = 0;

/// Default `Q_THRST_LOSS_OPT`, upstream `AP_GROUPINFO("THRST_LOSS_OPT", ..., 0)`.
pub const Q_THRST_LOSS_OPT_DEFAULT: i32 = 0;

/// Inputs [`QuadPlane::thrust_loss_check`] reads from Plane / motors.
///
/// This crate does not own AHRS, attitude-control, or the motors
/// thrust-boost latch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustLossView {
    /// `reset` argument — motors inactive this tick.
    pub reset: bool,
    /// `in_vtol_mode()` for the `VTOL_ONLY` option.
    pub in_vtol_mode: bool,
    /// `motors->get_thrust_boost()`.
    pub thrust_boost: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `plane.is_flying()`.
    pub is_flying: bool,
    /// `motors->get_desired_spool_state() == THROTTLE_UNLIMITED`.
    pub spool_unlimited: bool,
    /// `attitude_control->get_att_target_euler_rad().xy().length_squared()`.
    pub att_target_xy_rad_len_sq: f32,
    /// `attitude_control->get_throttle_in()`.
    pub throttle_in: f32,
    /// `motors->limit.throttle_upper`.
    pub throttle_upper: bool,
    /// `ahrs.get_velocity_NED` succeeded.
    pub have_vel_ned: bool,
    /// NED down velocity, m/s (`vel_NED.z`).
    pub vel_ned_z: f32,
    /// `attitude_control->get_att_error_angle_deg()`.
    pub att_error_deg: f32,
    /// `plane.scheduler.get_loop_rate_hz()`.
    pub loop_rate_hz: u16,
}

impl ThrustLossView {
    /// All increment gates open; `loop_rate_hz` is 2 so two ticks engage.
    #[must_use]
    pub const fn losing() -> Self {
        Self {
            reset: false,
            in_vtol_mode: true,
            thrust_boost: false,
            armed: true,
            is_flying: true,
            spool_unlimited: true,
            att_target_xy_rad_len_sq: 0.0,
            throttle_in: 0.95,
            throttle_upper: true,
            have_vel_ned: true,
            vel_ned_z: 1.0,
            att_error_deg: 0.0,
            loop_rate_hz: 2,
        }
    }

    /// Motors inactive — `thrust_loss_check(true)` clears the counter.
    #[must_use]
    pub const fn inactive() -> Self {
        let mut view = Self::losing();
        view.reset = true;
        view
    }
}

/// Outcome of one [`QuadPlane::thrust_loss_check`] tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThrustLossTick {
    /// `motors->set_thrust_boost(true)` ran this tick.
    pub engaged: bool,
    /// Counter after the tick (0 when cleared or just engaged).
    pub counter: u16,
    /// Counter was forced to 0 (reset / option / idle / reject).
    pub cleared: bool,
}

/// Inputs [`QuadPlane::run_esc_calibration`] reads from Plane / motors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscCalView {
    /// `motors->armed()`.
    pub armed: bool,
    /// `plane.get_throttle_input()` (percent, 0..100).
    pub throttle_input: f32,
}

impl EscCalView {
    /// Armed, mid-stick.
    #[must_use]
    pub const fn armed_mid() -> Self {
        Self {
            armed: true,
            throttle_input: 50.0,
        }
    }

    /// Disarmed — passthrough forced to 0.
    #[must_use]
    pub const fn disarmed() -> Self {
        Self {
            armed: false,
            throttle_input: 50.0,
        }
    }
}

/// Outcome of one [`QuadPlane::run_esc_calibration`] tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscCalTick {
    /// `motors->set_throttle_passthrough_for_esc_calibration`.
    pub passthrough: f32,
    /// `AP_Notify::flags.esc_calibration` after the tick.
    pub notify: bool,
    /// First armed tick this session (GCS "Starting ESC calibration").
    pub started: bool,
}

/// Upstream `QuadPlane::thrust_loss` block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThrustLoss {
    /// Iterations that looked like lost thrust.
    counter: u16,
    /// `Q_THRST_LOSS_OPT`.
    options: i32,
}

impl ThrustLoss {
    /// Zeroed counter, options default 0 (check enabled in all modes).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            counter: 0,
            options: Q_THRST_LOSS_OPT_DEFAULT,
        }
    }

    /// Current `Q_THRST_LOSS_OPT` bitmask.
    #[must_use]
    pub const fn options(&self) -> i32 {
        self.options
    }

    /// Write `Q_THRST_LOSS_OPT`.
    pub fn set_options(&mut self, options: i32) {
        self.options = options;
    }

    /// Current loss-suspect counter.
    #[must_use]
    pub const fn counter(&self) -> u16 {
        self.counter
    }
}

impl Default for ThrustLoss {
    fn default() -> Self {
        Self::new()
    }
}

/// Leftover `Q_ESC_CAL` + Notify latch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscCalibration {
    /// `Q_ESC_CAL` (0 off, 1 throttle, 2 full-range).
    mode: i8,
    /// `AP_Notify::flags.esc_calibration`.
    notify: bool,
    /// Last passthrough demand (0..1).
    passthrough: f32,
}

impl EscCalibration {
    /// Parameter default, notify clear, passthrough 0.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: Q_ESC_CAL_DEFAULT,
            notify: false,
            passthrough: 0.0,
        }
    }

    /// Current `Q_ESC_CAL`.
    #[must_use]
    pub const fn mode(&self) -> i8 {
        self.mode
    }

    /// Write `Q_ESC_CAL`.
    pub fn set_mode(&mut self, mode: i8) {
        self.mode = mode;
    }

    /// Current Notify latch.
    #[must_use]
    pub const fn notify(&self) -> bool {
        self.notify
    }

    /// Last passthrough demand.
    #[must_use]
    pub const fn passthrough(&self) -> f32 {
        self.passthrough
    }
}

impl Default for EscCalibration {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadPlane {
    /// Leftover `thrust_loss` block.
    #[must_use]
    pub const fn thrust_loss(&self) -> &ThrustLoss {
        &self.thrust_loss
    }

    /// Write `Q_THRST_LOSS_OPT`.
    pub fn set_thrust_loss_options(&mut self, options: i32) {
        self.thrust_loss.set_options(options);
    }

    /// Current `Q_ESC_CAL`.
    #[must_use]
    pub const fn esc_calibration(&self) -> i8 {
        self.esc_calibration.mode()
    }

    /// Write `Q_ESC_CAL`.
    pub fn set_esc_calibration(&mut self, mode: i8) {
        self.esc_calibration.set_mode(mode);
    }

    /// `AP_Notify::flags.esc_calibration` leftover latch.
    #[must_use]
    pub const fn esc_cal_notify(&self) -> bool {
        self.esc_calibration.notify()
    }

    /// Last ESC-cal passthrough demand.
    #[must_use]
    pub const fn esc_cal_passthrough(&self) -> f32 {
        self.esc_calibration.passthrough()
    }

    /// `Q_TKOFF_FAIL_SCL` from the leftover AUTO takeoff stub.
    ///
    /// Does not rewrite [`crate::auto_vtol`] takeoff start / verify.
    #[must_use]
    pub const fn takeoff_failure_scalar(&self) -> f32 {
        self.auto_vtol.takeoff_failure_scalar()
    }

    /// Write `Q_TKOFF_FAIL_SCL` onto the leftover AUTO takeoff stub.
    pub fn set_takeoff_failure_scalar(&mut self, scalar: f32) {
        self.auto_vtol.set_takeoff_failure_scalar(scalar);
    }

    /// `is_positive(scalar) && (now - start) > takeoff_time_limit_ms`.
    #[must_use]
    pub const fn takeoff_failure_timed_out(&self, now_ms: u32) -> bool {
        takeoff_failure_timed_out(
            self.auto_vtol.takeoff_failure_scalar(),
            now_ms.wrapping_sub(self.auto_vtol.takeoff_start_time_ms()),
            self.auto_vtol.takeoff_time_limit_ms(),
        )
    }

    /// `setup_defaults` leftover: a non-zero `Q_ESC_CAL` is saved as 0.
    pub fn esc_calibration_reset_on_setup_defaults(&mut self) -> bool {
        if self.esc_calibration.mode() != 0 {
            self.esc_calibration.set_mode(0);
            true
        } else {
            false
        }
    }

    /// Upstream `QuadPlane::thrust_loss_check`.
    ///
    /// Reset / disabled / idle / reject gates clear the counter. A
    /// descending, high-throttle, wings-level tick increments; one
    /// second (`loop_rate_hz` samples) engages thrust boost.
    pub fn thrust_loss_check(&mut self, view: ThrustLossView) -> ThrustLossTick {
        if view.reset
            || thrust_loss_disabled(self.thrust_loss.options())
            || thrust_loss_vtol_only_skip(self.thrust_loss.options(), view.in_vtol_mode)
            || thrust_loss_already_engaged_or_idle(
                view.thrust_boost,
                view.armed,
                view.is_flying,
                view.spool_unlimited,
            )
            || thrust_loss_tilt_too_steep(view.att_target_xy_rad_len_sq)
            || thrust_loss_throttle_not_saturated(view.throttle_in, view.throttle_upper)
            || thrust_loss_throttle_too_low(view.throttle_in)
            || thrust_loss_not_descending(view.have_vel_ned, view.vel_ned_z)
            || thrust_loss_attitude_lost(view.att_error_deg)
        {
            self.thrust_loss.counter = 0;
            return ThrustLossTick {
                engaged: false,
                counter: 0,
                cleared: true,
            };
        }
        self.thrust_loss.counter = self.thrust_loss.counter.saturating_add(1);
        if view.loop_rate_hz != 0 && self.thrust_loss.counter >= view.loop_rate_hz {
            self.thrust_loss.counter = 0;
            return ThrustLossTick {
                engaged: true,
                counter: 0,
                cleared: false,
            };
        }
        ThrustLossTick {
            engaged: false,
            counter: self.thrust_loss.counter,
            cleared: false,
        }
    }

    /// Upstream `QuadPlane::run_esc_calibration`.
    ///
    /// Disarmed forces passthrough 0 and clears Notify. Armed latches
    /// Notify and writes throttle (`Q_ESC_CAL==1`) or full-range
    /// (`Q_ESC_CAL==2`) passthrough.
    pub fn run_esc_calibration(&mut self, view: EscCalView) -> EscCalTick {
        if !view.armed {
            self.esc_calibration.passthrough = 0.0;
            self.esc_calibration.notify = false;
            return EscCalTick {
                passthrough: 0.0,
                notify: false,
                started: false,
            };
        }
        let started = !self.esc_calibration.notify;
        self.esc_calibration.notify = true;
        let passthrough =
            esc_cal_passthrough(self.esc_calibration.mode(), view.armed, view.throttle_input);
        if self.esc_calibration.mode() == 1 || self.esc_calibration.mode() == 2 {
            self.esc_calibration.passthrough = passthrough;
        }
        EscCalTick {
            passthrough: self.esc_calibration.passthrough,
            notify: true,
            started,
        }
    }

    /// Upstream `ThrustLoss::option_is_set`.
    #[must_use]
    pub const fn thrust_loss_option_is_set(&self, option: ThrustLossOption) -> bool {
        thrust_loss_option_is_set(self.thrust_loss.options(), option)
    }
}
