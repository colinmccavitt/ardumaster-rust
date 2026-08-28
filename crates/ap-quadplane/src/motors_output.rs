//! Leftover VTOL motors-output / hold / arm stub, upstream
//! `QuadPlane::motors_output` / `hold_hover` / `hold_stabilize` /
//! `set_armed` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. Plane owns arming, AFS, emergency-stop, ESC
//! calibration, and the tailsitter transition clock; the caller passes
//! a [`MotorsOutputView`] / [`SetArmedView`]. This is not a rewrite of
//! [`crate::motor_test`] PWM sequencing, [`crate::throttle`] mix /
//! suppression, [`crate::mode_q`] `run()` dispatch, or the leftover
//! `thrust_loss_check` / `run_esc_calibration` rows.

use crate::quadplane_completeness::{
    att_control_relax_stale, climb_rate_ms_from_cms, hold_stabilize_ground_idle,
    hold_stabilize_should_boost, motors_inactive, motors_output_skip_tailsitter_transition,
    motors_were_active,
};
use crate::{QuadPlane, VtolAirframe};

/// `AP_Motors::DesiredSpoolState` values this stub records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DesiredSpoolState {
    /// `AP_Motors::DesiredSpoolState::SHUT_DOWN`.
    ShutDown,
    /// `AP_Motors::DesiredSpoolState::GROUND_IDLE`.
    GroundIdle,
    /// `AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED`.
    ThrottleUnlimited,
}

/// Why `motors_output` returned this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorsOutputAction {
    /// `DELAY_ARMING` / `DISARMED_TILT` plus `get_delay_arming()`.
    DelayArming,
    /// `!armed_and_safety_off`, emergency-stop, or AFS crash.
    Disarmed,
    /// ESC-cal in QStabilize — output comes from `run_esc_calibration`.
    EscCalibration,
    /// Tailsitter VTOL transition, not assisted.
    TailsitterTransition,
    /// `motors->output()` path (rate controller + last-active latch).
    Output,
}

/// Outcome of one [`QuadPlane::motors_output`] tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorsOutputTick {
    /// Which gate or output path ran.
    pub action: MotorsOutputAction,
    /// Desired spool after this tick.
    pub desired_spool: DesiredSpoolState,
    /// `motors->output()` ran (delay / disarmed / output paths).
    pub motors_output_ran: bool,
    /// `attitude_control->rate_controller_run()` ran.
    pub rate_controller_ran: bool,
    /// Relaxed because `last_att_control_ms` was stale.
    pub attitude_relaxed: bool,
    /// `(now - last_motors_active_ms) > 100` at the thrust-loss check.
    pub motors_inactive: bool,
}

/// Inputs `motors_output` reads from Plane / Notify / tailsitter.
///
/// This crate does not own arming, AFS, `SRV_Channels`, or the
/// tailsitter transition clock.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorsOutputView {
    /// `plane.arming.get_delay_arming()`.
    pub arming_delay_active: bool,
    /// `plane.arming.is_armed_and_safety_off()`.
    pub armed_and_safety_off: bool,
    /// `SRV_Channels::get_emergency_stop()`.
    pub emergency_stop: bool,
    /// AFS crash that is not a landing terminate.
    pub afs_should_crash: bool,
    /// `esc_calibration && AP_Notify::flags.esc_calibration && QStabilize`.
    pub esc_calibration_qstabilize: bool,
    /// `tailsitter.in_vtol_transition(now)`.
    pub tailsitter_in_vtol_transition: bool,
    /// `motors_output(run_rate_controller)` argument.
    pub run_rate_controller: bool,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `motors->get_throttle()`.
    pub motors_throttle: f32,
    /// `tiltrotor.motors_active()`.
    pub tiltrotor_motors_active: bool,
}

impl MotorsOutputView {
    /// Armed, no gates, rate controller on, motors recently active.
    #[must_use]
    pub const fn armed_output(now_ms: u32) -> Self {
        Self {
            arming_delay_active: false,
            armed_and_safety_off: true,
            emergency_stop: false,
            afs_should_crash: false,
            esc_calibration_qstabilize: false,
            tailsitter_in_vtol_transition: false,
            run_rate_controller: true,
            now_ms,
            motors_throttle: 0.5,
            tiltrotor_motors_active: false,
        }
    }
}

/// Plane-owned inputs [`QuadPlane::set_armed`] reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SetArmedView {
    /// `plane.control_mode == &plane.mode_guided`.
    pub in_guided: bool,
    /// `plane.get_throttle_input()` for `init_throttle_wait`.
    pub throttle_input: i16,
    /// `plane.is_flying()` for `init_throttle_wait`.
    pub is_flying: bool,
}

impl SetArmedView {
    /// Not GUIDED, stick below the wait threshold, not flying.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_guided: false,
            throttle_input: 0,
            is_flying: false,
        }
    }

    /// GUIDED, same wait inputs as [`Self::new`].
    #[must_use]
    pub const fn guided() -> Self {
        Self {
            in_guided: true,
            throttle_input: 0,
            is_flying: false,
        }
    }
}

impl Default for SetArmedView {
    fn default() -> Self {
        Self::new()
    }
}

/// Outcome of [`QuadPlane::hold_stabilize`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldStabilize {
    /// Spool `hold_stabilize` requested.
    pub desired_spool: DesiredSpoolState,
    /// `attitude_control->set_throttle_out` demand.
    pub throttle_out: f32,
    /// Angle-boost flag passed to `set_throttle_out`.
    pub should_boost: bool,
    /// `relax_attitude_control()` ran (ground-idle branch).
    pub attitude_relaxed: bool,
}

/// Outcome of [`QuadPlane::hold_hover`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldHover {
    /// Always `THROTTLE_UNLIMITED`.
    pub desired_spool: DesiredSpoolState,
    /// Pilot / caller climb demand, cm/s.
    pub climb_rate_cms: f32,
    /// `set_climb_rate_ms` demand (`cms * 0.01`).
    pub climb_rate_ms: f32,
}

/// Last motors-output / hold latches this leftover records.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorsOutputState {
    /// Last `set_desired_spool_state`.
    desired_spool: DesiredSpoolState,
    /// Upstream `last_motors_active_ms`.
    last_motors_active_ms: u32,
    /// Upstream `last_att_control_ms`.
    last_att_control_ms: u32,
    /// Last `set_throttle_out` demand from `hold_stabilize`.
    throttle_out: f32,
    /// Last `hold_hover` climb demand, cm/s.
    climb_rate_cms: f32,
}

impl MotorsOutputState {
    /// Zeroed timers, spool shut down.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            desired_spool: DesiredSpoolState::ShutDown,
            last_motors_active_ms: 0,
            last_att_control_ms: 0,
            throttle_out: 0.0,
            climb_rate_cms: 0.0,
        }
    }

    /// Last `set_desired_spool_state`.
    #[must_use]
    pub const fn desired_spool(&self) -> DesiredSpoolState {
        self.desired_spool
    }

    /// Upstream `last_motors_active_ms`.
    #[must_use]
    pub const fn last_motors_active_ms(&self) -> u32 {
        self.last_motors_active_ms
    }

    /// Upstream `last_att_control_ms`.
    #[must_use]
    pub const fn last_att_control_ms(&self) -> u32 {
        self.last_att_control_ms
    }

    /// Last `hold_stabilize` throttle-out demand.
    #[must_use]
    pub const fn throttle_out(&self) -> f32 {
        self.throttle_out
    }

    /// Last `hold_hover` climb demand, cm/s.
    #[must_use]
    pub const fn climb_rate_cms(&self) -> f32 {
        self.climb_rate_cms
    }
}

impl Default for MotorsOutputState {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadPlane {
    /// Leftover motors-output / hold latches.
    #[must_use]
    pub const fn motors_output_state(&self) -> &MotorsOutputState {
        &self.motors_output_state
    }

    /// Upstream `QuadPlane::set_armed`.
    ///
    /// No-op when `!initialised`. Otherwise writes `motors->armed()`,
    /// latches `guided_wait_takeoff` in GUIDED, and re-inits
    /// `throttle_wait` when air-mode is off.
    pub fn set_armed(&mut self, armed: bool, view: SetArmedView) {
        if !self.initialised {
            return;
        }
        self.set_motors_armed(armed);
        if view.in_guided {
            self.set_guided_wait_takeoff(armed);
        }
        if !self.air_mode_active() {
            self.init_throttle_wait(view.throttle_input, view.is_flying);
        }
    }

    /// Upstream `QuadPlane::hold_stabilize`.
    ///
    /// Zero throttle without air-mode is `GROUND_IDLE` + throttle 0 +
    /// relax. Otherwise `THROTTLE_UNLIMITED` and the caller throttle;
    /// tailsitter + assist drops angle boost.
    pub fn hold_stabilize(&mut self, throttle_in: f32) -> HoldStabilize {
        if hold_stabilize_ground_idle(throttle_in, self.air_mode_active()) {
            self.motors_output_state.desired_spool = DesiredSpoolState::GroundIdle;
            self.motors_output_state.throttle_out = 0.0;
            return HoldStabilize {
                desired_spool: DesiredSpoolState::GroundIdle,
                throttle_out: 0.0,
                should_boost: false,
                attitude_relaxed: true,
            };
        }
        let tailsitter = matches!(self.vtol_airframe(), Some(VtolAirframe::Tailsitter));
        let should_boost = hold_stabilize_should_boost(tailsitter, self.assisted_flight());
        self.motors_output_state.desired_spool = DesiredSpoolState::ThrottleUnlimited;
        self.motors_output_state.throttle_out = throttle_in;
        HoldStabilize {
            desired_spool: DesiredSpoolState::ThrottleUnlimited,
            throttle_out: throttle_in,
            should_boost,
            attitude_relaxed: false,
        }
    }

    /// Upstream `QuadPlane::hold_hover`.
    ///
    /// Always `THROTTLE_UNLIMITED`, then `set_climb_rate_ms(cms * 0.01)`.
    /// Attitude / Z-controller objects live in COP.
    pub fn hold_hover(&mut self, target_climb_rate_cms: f32) -> HoldHover {
        let climb_rate_ms = climb_rate_ms_from_cms(target_climb_rate_cms);
        self.motors_output_state.desired_spool = DesiredSpoolState::ThrottleUnlimited;
        self.motors_output_state.climb_rate_cms = target_climb_rate_cms;
        HoldHover {
            desired_spool: DesiredSpoolState::ThrottleUnlimited,
            climb_rate_cms: target_climb_rate_cms,
            climb_rate_ms,
        }
    }

    /// Upstream `QuadPlane::motors_output`.
    ///
    /// Delay-arming / disarmed / e-stop / AFS force `SHUT_DOWN` and
    /// still run `motors->output()`. ESC-cal and unassisted tailsitter
    /// transition return without that output. The output path may run
    /// the rate controller and latches `last_motors_active_ms`.
    /// `thrust_loss_check` lives on [`crate::thrust_loss`] — this only reports
    /// the inactive flag that check would see.
    pub fn motors_output(&mut self, view: MotorsOutputView) -> MotorsOutputTick {
        if self.leftover_motors_delay_arming(view.arming_delay_active) {
            return self.motors_output_shutdown(MotorsOutputAction::DelayArming, view);
        }
        if !view.armed_and_safety_off || view.emergency_stop || view.afs_should_crash {
            return self.motors_output_shutdown(MotorsOutputAction::Disarmed, view);
        }
        if view.esc_calibration_qstabilize {
            return MotorsOutputTick {
                action: MotorsOutputAction::EscCalibration,
                desired_spool: self.motors_output_state.desired_spool,
                motors_output_ran: false,
                rate_controller_ran: false,
                attitude_relaxed: false,
                motors_inactive: motors_inactive(
                    view.now_ms,
                    self.motors_output_state.last_motors_active_ms,
                ),
            };
        }
        if motors_output_skip_tailsitter_transition(
            view.tailsitter_in_vtol_transition,
            self.assisted_flight(),
        ) {
            return MotorsOutputTick {
                action: MotorsOutputAction::TailsitterTransition,
                desired_spool: self.motors_output_state.desired_spool,
                motors_output_ran: false,
                rate_controller_ran: false,
                attitude_relaxed: false,
                motors_inactive: motors_inactive(
                    view.now_ms,
                    self.motors_output_state.last_motors_active_ms,
                ),
            };
        }

        let mut attitude_relaxed = false;
        if view.run_rate_controller {
            if att_control_relax_stale(view.now_ms, self.motors_output_state.last_att_control_ms) {
                attitude_relaxed = true;
            }
            self.motors_output_state.last_att_control_ms = view.now_ms;
        }
        let inactive = motors_inactive(view.now_ms, self.motors_output_state.last_motors_active_ms);
        if motors_were_active(view.motors_throttle, view.tiltrotor_motors_active) {
            self.motors_output_state.last_motors_active_ms = view.now_ms;
        }
        MotorsOutputTick {
            action: MotorsOutputAction::Output,
            desired_spool: self.motors_output_state.desired_spool,
            motors_output_ran: true,
            rate_controller_ran: view.run_rate_controller,
            attitude_relaxed,
            motors_inactive: inactive,
        }
    }

    fn motors_output_shutdown(
        &mut self,
        action: MotorsOutputAction,
        view: MotorsOutputView,
    ) -> MotorsOutputTick {
        self.motors_output_state.desired_spool = DesiredSpoolState::ShutDown;
        MotorsOutputTick {
            action,
            desired_spool: DesiredSpoolState::ShutDown,
            motors_output_ran: true,
            rate_controller_ran: false,
            attitude_relaxed: false,
            motors_inactive: motors_inactive(
                view.now_ms,
                self.motors_output_state.last_motors_active_ms,
            ),
        }
    }
}
