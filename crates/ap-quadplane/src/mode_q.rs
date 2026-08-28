//! QSTABILIZE / QHOVER / QACRO enter + `run()` stub, upstream
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
//! `run()` is the attitude / throttle tick:
//! [`qstabilize_run`] -> [`QManualRunAction::HoldStabilize`]
//! (`hold_stabilize(get_pilot_throttle())`);
//! [`qhover_run`] -> [`QManualRunAction::HoldHover`]
//! (`hold_hover(get_pilot_desired_climb_rate_cms())`) or the
//! `throttle_wait` leftover;
//! [`qacro_run`] -> [`QManualRunAction::AcroRates`] (body-frame
//! rates + pilot throttle) or the same leftover. Tailsitter FW
//! pull-up is [`QManualRunAction::FwControllers`]. `update()`
//! (nav_roll / nav_pitch from sticks) is a later slice.

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

/// Default `Q_ACRO_RLL_RATE` (deg/s).
pub const Q_ACRO_ROLL_RATE_DEFAULT: f32 = 360.0;
/// Default `Q_ACRO_PIT_RATE` (deg/s).
pub const Q_ACRO_PITCH_RATE_DEFAULT: f32 = 180.0;
/// Default `Q_ACRO_YAW_RATE` (deg/s).
pub const Q_ACRO_YAW_RATE_DEFAULT: f32 = 90.0;

/// Motors spool a Q-manual `run()` would request this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QManualSpool {
    /// Path did not call `set_desired_spool_state` (FW / ESC-cal).
    Unchanged,
    /// `AP_Motors::DesiredSpoolState::GROUND_IDLE`.
    GroundIdle,
    /// `AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED`.
    ThrottleUnlimited,
}

/// Which attitude / throttle path a Q-manual `run()` took.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QManualRunAction {
    /// Tailsitter FW pull-up: `Mode::run()`.
    FwControllers,
    /// QStabilize `esc_calibration != 0`: `run_esc_calibration`.
    EscCalibration,
    /// QStabilize normal: `hold_stabilize(get_pilot_throttle())`.
    HoldStabilize,
    /// QHover / QAcro `throttle_wait` leftover.
    ThrottleWait,
    /// QHover flying: `hold_hover(get_pilot_desired_climb_rate_cms())`.
    HoldHover,
    /// QAcro flying: body-frame rate demand + pilot throttle.
    AcroRates,
}

/// QAcro body-frame rate demand (`input_rate_bf_roll_pitch_yaw_*_cds`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QAcroRateDemand {
    /// `target_roll` (cd/s).
    pub roll_cds: f32,
    /// `target_pitch` (cd/s).
    pub pitch_cds: f32,
    /// `target_yaw` (cd/s).
    pub yaw_cds: f32,
    /// `plane.g.acro_locking` -> `*_3_cds` (true) vs `*_2_cds`.
    pub locking: bool,
}

/// Outcome of one Q-manual `run()` tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QManualRun {
    /// Stabilize / hover / acro / leftover / FW / ESC-cal.
    pub action: QManualRunAction,
    /// Spool this path would request.
    pub spool: QManualSpool,
    /// `assign_tilt_to_fwd_thr()` ran (QStabilize after FW check;
    /// QHover only on the flying branch; QAcro never).
    pub tilt_assigned: bool,
    /// QHover wait calls `pos_control->D_relax_controller(0)`.
    pub d_relaxed: bool,
    /// QAcro flying-branch rate demand; `None` otherwise.
    pub acro_rates: Option<QAcroRateDemand>,
}

impl QManualRun {
    const fn fw_controllers() -> Self {
        Self {
            action: QManualRunAction::FwControllers,
            spool: QManualSpool::Unchanged,
            tilt_assigned: false,
            d_relaxed: false,
            acro_rates: None,
        }
    }

    const fn esc_calibration() -> Self {
        Self {
            action: QManualRunAction::EscCalibration,
            spool: QManualSpool::Unchanged,
            tilt_assigned: true,
            d_relaxed: false,
            acro_rates: None,
        }
    }

    const fn hold_stabilize() -> Self {
        Self {
            action: QManualRunAction::HoldStabilize,
            spool: QManualSpool::Unchanged,
            tilt_assigned: true,
            d_relaxed: false,
            acro_rates: None,
        }
    }

    const fn hold_hover() -> Self {
        Self {
            action: QManualRunAction::HoldHover,
            spool: QManualSpool::ThrottleUnlimited,
            tilt_assigned: true,
            d_relaxed: false,
            acro_rates: None,
        }
    }

    const fn throttle_wait(d_relaxed: bool) -> Self {
        Self {
            action: QManualRunAction::ThrottleWait,
            spool: QManualSpool::GroundIdle,
            tilt_assigned: false,
            d_relaxed,
            acro_rates: None,
        }
    }

    const fn acro_rates(rates: QAcroRateDemand) -> Self {
        Self {
            action: QManualRunAction::AcroRates,
            spool: QManualSpool::ThrottleUnlimited,
            tilt_assigned: false,
            d_relaxed: false,
            acro_rates: Some(rates),
        }
    }
}

/// Pilot / vehicle view Q-manual `run()` reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QManualRunView {
    /// `tailsitter.in_vtol_transition(now)` -- FW pull-up early-out.
    pub tailsitter_in_vtol_transition: bool,
    /// `quadplane.esc_calibration` (QStabilize only).
    pub esc_calibration: i8,
    /// `tailsitter.enabled()` -- QAcro swaps body-frame roll / yaw.
    pub tailsitter_enabled: bool,
    /// `plane.g.acro_locking`.
    pub acro_locking: bool,
    /// `channel_roll->norm_input()` (QAcro).
    pub roll_norm: f32,
    /// `channel_pitch->norm_input()` (QAcro).
    pub pitch_norm: f32,
    /// `channel_rudder->norm_input()` (QAcro).
    pub rudder_norm: f32,
    /// `Q_ACRO_RLL_RATE`.
    pub acro_roll_rate: f32,
    /// `Q_ACRO_PIT_RATE`.
    pub acro_pitch_rate: f32,
    /// `Q_ACRO_YAW_RATE`.
    pub acro_yaw_rate: f32,
}

impl QManualRunView {
    /// Level sticks, no FW pull-up, no ESC-cal, conventional airframe.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            tailsitter_in_vtol_transition: false,
            esc_calibration: 0,
            tailsitter_enabled: false,
            acro_locking: false,
            roll_norm: 0.0,
            pitch_norm: 0.0,
            rudder_norm: 0.0,
            acro_roll_rate: Q_ACRO_ROLL_RATE_DEFAULT,
            acro_pitch_rate: Q_ACRO_PITCH_RATE_DEFAULT,
            acro_yaw_rate: Q_ACRO_YAW_RATE_DEFAULT,
        }
    }

    /// Tailsitter FW pull-up phase of VTOL transition.
    #[must_use]
    pub const fn tailsitter_fw_transition() -> Self {
        let mut v = Self::flying();
        v.tailsitter_in_vtol_transition = true;
        v
    }
}

/// QAcro body-frame rates from sticks, upstream `ModeQAcro::run`.
///
/// Pitch is always `pitch_norm * Q_ACRO_PIT_RATE * 100`. Conventional
/// airframes map roll / rudder to roll / yaw. Tailsitters swap those
/// axes (`+rudder -> roll`, `-roll -> yaw`) because the 90 degree Y
/// rotation for copter mode swaps body-frame roll and yaw.
#[must_use]
pub const fn qacro_rate_demand(view: &QManualRunView) -> QAcroRateDemand {
    let pitch_cds = view.pitch_norm * view.acro_pitch_rate * 100.0;
    let (roll_cds, yaw_cds) = if view.tailsitter_enabled {
        (
            view.rudder_norm * view.acro_yaw_rate * 100.0,
            -view.roll_norm * view.acro_roll_rate * 100.0,
        )
    } else {
        (
            view.roll_norm * view.acro_roll_rate * 100.0,
            view.rudder_norm * view.acro_yaw_rate * 100.0,
        )
    };
    QAcroRateDemand {
        roll_cds,
        pitch_cds,
        yaw_cds,
        locking: view.acro_locking,
    }
}

/// Upstream `ModeQStabilize::run`.
///
/// Tailsitter FW pull-up runs `Mode::run()`. Otherwise
/// `assign_tilt_to_fwd_thr()`, then ESC-cal or
/// `hold_stabilize(get_pilot_throttle())`.
#[must_use]
pub const fn qstabilize_run(view: &QManualRunView) -> QManualRun {
    if view.tailsitter_in_vtol_transition {
        return QManualRun::fw_controllers();
    }
    if view.esc_calibration != 0 {
        return QManualRun::esc_calibration();
    }
    QManualRun::hold_stabilize()
}

/// Upstream `ModeQHover::run`.
///
/// Tailsitter FW pull-up runs `Mode::run()`. `throttle_wait` is the
/// leftover: ground idle, throttle out 0, relax attitude, D-relax.
/// Otherwise `assign_tilt_to_fwd_thr()` then
/// `hold_hover(get_pilot_desired_climb_rate_cms())`.
#[must_use]
pub const fn qhover_run(qp: &QuadPlane, view: &QManualRunView) -> QManualRun {
    if view.tailsitter_in_vtol_transition {
        return QManualRun::fw_controllers();
    }
    if qp.throttle_wait() {
        return QManualRun::throttle_wait(true);
    }
    QManualRun::hold_hover()
}

/// Upstream `ModeQAcro::run`.
///
/// Tailsitter FW pull-up runs `Mode::run()`. `throttle_wait` is the
/// leftover (no D-relax). Otherwise `THROTTLE_UNLIMITED`, body-frame
/// rates, and `set_throttle_out(get_pilot_throttle(), false, 10)`.
#[must_use]
pub const fn qacro_run(qp: &QuadPlane, view: &QManualRunView) -> QManualRun {
    if view.tailsitter_in_vtol_transition {
        return QManualRun::fw_controllers();
    }
    if qp.throttle_wait() {
        return QManualRun::throttle_wait(false);
    }
    QManualRun::acro_rates(qacro_rate_demand(view))
}
