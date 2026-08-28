//! QSTABILIZE / QHOVER / QACRO `_enter` stub, upstream
//! `ArduPlane/mode_qstabilize.cpp` / `mode_qhover.cpp` / `mode_qacro.cpp`
//! (Plane-4.7.0).
//!
//! Tracked as **VT-004**. `Mode::enter` always calls
//! [`QuadPlane::mode_enter`] then the mode's `_enter`. QStabilize and
//! QAcro force `throttle_wait = false`. QHover calls
//! [`QuadPlane::init_throttle_wait`] after zeroing the climb-rate
//! demand and latching the D-axis speed / accel limits. QAcro also
//! `transition->force_transition_complete()`, relaxes attitude, and
//! zeros the yaw-rate time constant.
//!
//! `run()` / `update()` are later slices.

use crate::transition_fsm::SltTransition;
use crate::QuadPlane;

/// `Mode::Number::QSTABILIZE`.
pub const MODE_QSTABILIZE: u8 = 17;
/// `Mode::Number::QHOVER`.
pub const MODE_QHOVER: u8 = 18;
/// `Mode::Number::QACRO`.
pub const MODE_QACRO: u8 = 23;

/// The three manual Q modes this slice ports.
///
/// Discriminants match `Mode::Number`. All three are
/// `is_vtol_mode` / `is_vtol_man_mode`. Only QStabilize and QAcro
/// override `is_vtol_man_throttle` (pilot throttle).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum QManualMode {
    /// `ModeQStabilize` — angle-mode VTOL, pilot throttle.
    Stabilize = 17,
    /// `ModeQHover` — angle-mode VTOL, altitude-hold throttle.
    Hover = 18,
    /// `ModeQAcro` — rate-mode VTOL, pilot throttle.
    Acro = 23,
}

impl QManualMode {
    /// Inverse of the upstream `Mode::Number` discriminant.
    #[must_use]
    pub const fn from_number(number: u8) -> Option<Self> {
        match number {
            MODE_QSTABILIZE => Some(Self::Stabilize),
            MODE_QHOVER => Some(Self::Hover),
            MODE_QACRO => Some(Self::Acro),
            _ => None,
        }
    }

    /// Upstream `Mode::mode_number`.
    #[must_use]
    pub const fn mode_number(self) -> u8 {
        self as u8
    }

    /// Upstream `Mode::is_vtol_mode` — true for every Q* mode.
    #[must_use]
    pub const fn is_vtol_mode(self) -> bool {
        true
    }

    /// Upstream `Mode::is_vtol_man_mode` — true for these three.
    #[must_use]
    pub const fn is_vtol_man_mode(self) -> bool {
        true
    }

    /// Upstream `Mode::is_vtol_man_throttle`.
    ///
    /// QStabilize / QAcro override this to true. QHover keeps the
    /// base `false` (altitude-hold throttle via `init_throttle_wait`).
    #[must_use]
    pub const fn is_vtol_man_throttle(self) -> bool {
        matches!(self, Self::Stabilize | Self::Acro)
    }
}

/// Pilot / flying view QHover `_enter` needs for `init_throttle_wait`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QHoverEnterView {
    /// `get_throttle_input()`, the stick `init_throttle_wait` reads.
    pub throttle_input: i16,
    /// `plane.is_flying()`.
    pub is_flying: bool,
}

impl QHoverEnterView {
    /// Stick + flying flags as `ModeQHover::_enter` would read them.
    #[must_use]
    pub const fn new(throttle_input: i16, is_flying: bool) -> Self {
        Self {
            throttle_input,
            is_flying,
        }
    }

    /// Parked on the ground at idle throttle — `throttle_wait` becomes true.
    #[must_use]
    pub const fn parked_idle() -> Self {
        Self {
            throttle_input: 0,
            is_flying: false,
        }
    }
}

/// Side effects QHover `_enter` records besides `throttle_wait`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QHoverEnterState {
    /// `pos_control->D_set_max_speed_accel_m` ran.
    pub d_speed_accel_set: bool,
    /// `pos_control->D_set_correction_speed_accel_m` ran.
    pub d_correction_set: bool,
    /// `quadplane.set_climb_rate_ms(0)` ran.
    pub climb_rate_zeroed: bool,
}

impl Default for QHoverEnterState {
    fn default() -> Self {
        Self::new()
    }
}

impl QHoverEnterState {
    /// Nothing latched yet — before `_enter`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            d_speed_accel_set: false,
            d_correction_set: false,
            climb_rate_zeroed: false,
        }
    }
}

/// QAcro `_enter` attitude / ACRO-state latch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QAcroEnterState {
    /// `attitude_control->relax_attitude_controllers()`.
    pub attitude_relaxed: bool,
    /// `set_yaw_rate_tc(0)` via `disable_yaw_rate_time_constant`.
    pub yaw_rate_tc_cleared: bool,
    /// `ahrs.get_quaternion(plane.mode_acro.acro_state.q)` ran.
    pub acro_quat_latched: bool,
}

impl Default for QAcroEnterState {
    fn default() -> Self {
        Self::new()
    }
}

impl QAcroEnterState {
    /// Nothing latched yet — before `_enter`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            attitude_relaxed: false,
            yaw_rate_tc_cleared: false,
            acro_quat_latched: false,
        }
    }
}

/// Combined `Mode::enter` for QStabilize: `mode_enter` then `_enter`.
///
/// Upstream `ModeQStabilize::_enter` is `throttle_wait = false`.
/// Always returns true.
pub fn qstabilize_enter(qp: &mut QuadPlane) -> bool {
    qp.mode_enter();
    qp.set_throttle_wait(false);
    true
}

/// Combined `Mode::enter` for QHover: `mode_enter` then `_enter`.
///
/// Upstream `ModeQHover::_enter` sets the D-axis speed / accel
/// limits, `set_climb_rate_ms(0)`, then `init_throttle_wait()`.
/// Always returns true.
pub fn qhover_enter(
    qp: &mut QuadPlane,
    view: QHoverEnterView,
    state: &mut QHoverEnterState,
) -> bool {
    qp.mode_enter();
    state.d_speed_accel_set = true;
    state.d_correction_set = true;
    state.climb_rate_zeroed = true;
    qp.init_throttle_wait(view.throttle_input, view.is_flying);
    true
}

/// Combined `Mode::enter` for QAcro: `mode_enter` then `_enter`.
///
/// Upstream `ModeQAcro::_enter` forces `throttle_wait = false`,
/// `transition->force_transition_complete()`, relaxes attitude,
/// zeros the yaw-rate time constant, and snapshots the ACRO
/// quaternion. Always returns true.
pub fn qacro_enter(
    qp: &mut QuadPlane,
    transition: &mut SltTransition,
    state: &mut QAcroEnterState,
) -> bool {
    qp.mode_enter();
    qp.set_throttle_wait(false);
    transition.force_transition_complete();
    state.attitude_relaxed = true;
    state.yaw_rate_tc_cleared = true;
    state.acro_quat_latched = true;
    true
}
