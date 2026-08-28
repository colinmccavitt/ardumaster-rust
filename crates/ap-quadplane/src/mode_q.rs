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
//! pull-up is [`QManualRunAction::FwControllers`]. `update()` is
//! the nav leftover: QStabilize / QHover scale stick
//! `control_in / range` into `nav_roll_cd` / `nav_pitch_cd`
//! (tailsitter / FW-limited / Q_ANGLE_MAX). QAcro copies the
//! attitude-controller euler target. The three `.cpp` files are
//! leftover-complete after that (`MODE_Q_CPP_SURFACES`).

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

/// `Q_OPTIONS` bit 14, upstream `Option::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES`
/// (the misspelling is upstream).
pub const Q_OPTIONS_IGNORE_FW_ANGLE_LIMITS: i32 = 1 << 14;

/// Default `Q_A_ANGLE_MAX` lean limit (cd) used by the update view.
pub const Q_ANGLE_MAX_DEFAULT_CD: i16 = 3000;
/// Default `ROLL_LIMIT_DEG` in centidegrees.
pub const ROLL_LIMIT_DEFAULT_CD: i16 = 4500;
/// Default `PTCH_LIM_MAX_DEG` in centidegrees.
pub const PITCH_LIMIT_MAX_DEFAULT_CD: i16 = 2000;
/// Default `PTCH_LIM_MIN_DEG` in centidegrees (negative).
pub const PITCH_LIMIT_MIN_DEFAULT_CD: i16 = -2500;

/// Which `update()` path wrote `nav_roll_cd` / `nav_pitch_cd`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QManualUpdatePath {
    /// `tailsitter.active()` → `set_tailsitter_roll_pitch`.
    Tailsitter,
    /// Default: `set_limited_roll_pitch` (FW LIM_* + Q_ANGLE_MAX).
    LimitedFw,
    /// `Q_OPTIONS` bit 14 set: both axes use `lean_angle_max_cd`.
    AngleMax,
    /// QAcro: `get_att_target_euler_cd()` x/y.
    AcroAttTarget,
}

/// Outcome of one Q-manual `update()` tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QManualUpdate {
    /// `plane.nav_roll_cd`.
    pub nav_roll_cd: i32,
    /// `plane.nav_pitch_cd`.
    pub nav_pitch_cd: i32,
    /// Which branch wrote the demands.
    pub path: QManualUpdatePath,
    /// `transition->set_VTOL_roll_pitch_limit` ran (tailsitter only).
    pub vtol_roll_pitch_limit: bool,
}

/// Stick / limit view QStabilize / QHover `update()` reads.
///
/// `roll_input` / `pitch_input` are already
/// `get_control_in() / get_range()` (not `norm_input()` — tailsitter
/// `check_input` rewrites `control_in` only).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QManualUpdateView {
    /// Normalized roll stick, `[-1, 1]`.
    pub roll_input: f32,
    /// Normalized pitch stick, `[-1, 1]`.
    pub pitch_input: f32,
    /// `quadplane.tailsitter.active()`.
    pub tailsitter_active: bool,
    /// `Q_TAILSIT_MAX_ROLL` (deg). `<= 0` uses `lean_angle_max_cd`.
    pub tailsitter_max_roll_angle_deg: f32,
    /// `option_is_set(INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)`.
    pub ignore_fw_angle_limits: bool,
    /// `attitude_control->lean_angle_max_cd()`.
    pub lean_angle_max_cd: i16,
    /// `plane.roll_limit_cd`.
    pub roll_limit_cd: i16,
    /// `plane.aparm.pitch_limit_max * 100`.
    pub pitch_limit_max_cd: i16,
    /// `plane.pitch_limit_min * 100` (negative).
    pub pitch_limit_min_cd: i16,
}

impl QManualUpdateView {
    /// Level-ish sticks, conventional airframe, FW angle limits on.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_input: 0.0,
            pitch_input: 0.0,
            tailsitter_active: false,
            tailsitter_max_roll_angle_deg: 0.0,
            ignore_fw_angle_limits: false,
            lean_angle_max_cd: Q_ANGLE_MAX_DEFAULT_CD,
            roll_limit_cd: ROLL_LIMIT_DEFAULT_CD,
            pitch_limit_max_cd: PITCH_LIMIT_MAX_DEFAULT_CD,
            pitch_limit_min_cd: PITCH_LIMIT_MIN_DEFAULT_CD,
        }
    }
}

/// Attitude-controller target QAcro `update()` copies into nav.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QAcroUpdateView {
    /// `get_att_target_euler_cd().x`.
    pub att_target_roll_cd: f32,
    /// `get_att_target_euler_cd().y`.
    pub att_target_pitch_cd: f32,
}

impl QAcroUpdateView {
    /// Zeroed attitude target.
    #[must_use]
    pub const fn level() -> Self {
        Self {
            att_target_roll_cd: 0.0,
            att_target_pitch_cd: 0.0,
        }
    }
}

/// `control_in / range` — the normalize `ModeQStabilize::update` uses.
///
/// Must use `get_control_in()`, not `norm_input()`, because
/// `tailsitter_check_input` rewrites `control_in` only.
#[must_use]
pub const fn q_stick_norm(control_in: i16, range: i16) -> f32 {
    if range == 0 {
        0.0
    } else {
        control_in as f32 / range as f32
    }
}

const fn min_i16(a: i16, b: i16) -> i16 {
    if a < b {
        a
    } else {
        b
    }
}

/// Upstream `ModeQStabilize::set_tailsitter_roll_pitch`.
#[must_use]
pub const fn set_tailsitter_roll_pitch(view: &QManualUpdateView) -> QManualUpdate {
    let angle_max = view.lean_angle_max_cd as f32;
    let nav_roll_cd = if view.tailsitter_max_roll_angle_deg > 0.0 {
        (view.tailsitter_max_roll_angle_deg * 100.0 * view.roll_input) as i32
    } else {
        (view.roll_input * angle_max) as i32
    };
    let nav_pitch_cd = (view.pitch_input * angle_max) as i32;
    QManualUpdate {
        nav_roll_cd,
        nav_pitch_cd,
        path: QManualUpdatePath::Tailsitter,
        vtol_roll_pitch_limit: true,
    }
}

/// Upstream `ModeQStabilize::set_limited_roll_pitch`.
#[must_use]
pub const fn set_limited_roll_pitch(view: &QManualUpdateView) -> QManualUpdate {
    let angle_max = view.lean_angle_max_cd;
    let nav_roll_cd = (view.roll_input * min_i16(view.roll_limit_cd, angle_max) as f32) as i32;
    let nav_pitch_cd = if view.pitch_input > 0.0 {
        (view.pitch_input * min_i16(view.pitch_limit_max_cd, angle_max) as f32) as i32
    } else {
        (view.pitch_input * min_i16(-view.pitch_limit_min_cd, angle_max) as f32) as i32
    };
    QManualUpdate {
        nav_roll_cd,
        nav_pitch_cd,
        path: QManualUpdatePath::LimitedFw,
        vtol_roll_pitch_limit: false,
    }
}

/// Upstream `ModeQStabilize::update`.
///
/// Stick-normalized roll / pitch become `nav_roll_cd` / `nav_pitch_cd`.
/// Tailsitter takes `set_tailsitter_roll_pitch` and always calls
/// `set_VTOL_roll_pitch_limit`. Otherwise bit 14 picks Q_ANGLE_MAX
/// on both axes vs `set_limited_roll_pitch`.
#[must_use]
pub const fn qstabilize_update(view: &QManualUpdateView) -> QManualUpdate {
    if view.tailsitter_active {
        return set_tailsitter_roll_pitch(view);
    }
    if view.ignore_fw_angle_limits {
        let angle_max = view.lean_angle_max_cd as f32;
        return QManualUpdate {
            nav_roll_cd: (view.roll_input * angle_max) as i32,
            nav_pitch_cd: (view.pitch_input * angle_max) as i32,
            path: QManualUpdatePath::AngleMax,
            vtol_roll_pitch_limit: false,
        };
    }
    set_limited_roll_pitch(view)
}

/// Upstream `ModeQHover::update` — `plane.mode_qstabilize.update()`.
#[must_use]
pub const fn qhover_update(view: &QManualUpdateView) -> QManualUpdate {
    qstabilize_update(view)
}

/// Upstream `ModeQAcro::update`.
///
/// Copies the multicopter attitude-controller euler target into
/// `nav_roll_cd` / `nav_pitch_cd` (`att_target.x` / `.y`).
#[must_use]
pub const fn qacro_update(view: &QAcroUpdateView) -> QManualUpdate {
    QManualUpdate {
        nav_roll_cd: view.att_target_roll_cd as i32,
        nav_pitch_cd: view.att_target_pitch_cd as i32,
        path: QManualUpdatePath::AcroAttTarget,
        vtol_roll_pitch_limit: false,
    }
}

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModeQPortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by this slice (`update()` + helpers).
    ThisSlice,
}

/// One `mode_qstabilize` / `mode_qhover` / `mode_qacro.cpp` surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModeQSurface {
    /// Upstream `.cpp` file.
    pub file: &'static str,
    /// Function name.
    pub name: &'static str,
    /// Hooked up on main or this slice.
    pub status: ModeQPortStatus,
    /// Short note (Rust symbol).
    pub note: &'static str,
}

/// Completeness closer: every function in the three VT-004 cpp files.
pub const MODE_Q_CPP_SURFACES: &[ModeQSurface] = &[
    ModeQSurface {
        file: "mode_qstabilize.cpp",
        name: "_enter",
        status: ModeQPortStatus::OnMain,
        note: "qstabilize_enter / throttle_wait = false",
    },
    ModeQSurface {
        file: "mode_qstabilize.cpp",
        name: "update",
        status: ModeQPortStatus::ThisSlice,
        note: "qstabilize_update / nav_roll nav_pitch from sticks",
    },
    ModeQSurface {
        file: "mode_qstabilize.cpp",
        name: "run",
        status: ModeQPortStatus::OnMain,
        note: "qstabilize_run / hold_stabilize",
    },
    ModeQSurface {
        file: "mode_qstabilize.cpp",
        name: "set_tailsitter_roll_pitch",
        status: ModeQPortStatus::ThisSlice,
        note: "set_tailsitter_roll_pitch + set_VTOL_roll_pitch_limit",
    },
    ModeQSurface {
        file: "mode_qstabilize.cpp",
        name: "set_limited_roll_pitch",
        status: ModeQPortStatus::ThisSlice,
        note: "set_limited_roll_pitch / FW LIM_* + Q_ANGLE_MAX",
    },
    ModeQSurface {
        file: "mode_qhover.cpp",
        name: "_enter",
        status: ModeQPortStatus::OnMain,
        note: "qhover_enter / init_throttle_wait",
    },
    ModeQSurface {
        file: "mode_qhover.cpp",
        name: "update",
        status: ModeQPortStatus::ThisSlice,
        note: "qhover_update delegates to qstabilize_update",
    },
    ModeQSurface {
        file: "mode_qhover.cpp",
        name: "run",
        status: ModeQPortStatus::OnMain,
        note: "qhover_run / hold_hover vs throttle_wait",
    },
    ModeQSurface {
        file: "mode_qacro.cpp",
        name: "_enter",
        status: ModeQPortStatus::OnMain,
        note: "qacro_enter / force_transition_complete",
    },
    ModeQSurface {
        file: "mode_qacro.cpp",
        name: "update",
        status: ModeQPortStatus::ThisSlice,
        note: "qacro_update / att_target euler x/y",
    },
    ModeQSurface {
        file: "mode_qacro.cpp",
        name: "run",
        status: ModeQPortStatus::OnMain,
        note: "qacro_run / acro rates vs throttle_wait",
    },
];

/// True when every listed surface is `OnMain` or `ThisSlice` (no leftover).
#[must_use]
pub const fn mode_q_surfaces_complete() -> bool {
    let mut i = 0;
    while i < MODE_Q_CPP_SURFACES.len() {
        match MODE_Q_CPP_SURFACES[i].status {
            ModeQPortStatus::OnMain | ModeQPortStatus::ThisSlice => {}
        }
        i += 1;
    }
    MODE_Q_CPP_SURFACES.len() == 11
}
