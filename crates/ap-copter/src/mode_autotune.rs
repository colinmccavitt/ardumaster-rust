//! `ModeAutoTune` init leftover, upstream `ArduCopter/mode_autotune.cpp`.
//!
//! Tracked as **COP-027**. Copter AutoTune is a thin wrapper around
//! `AC_AutoTune_Multi` (`libraries/AC_AutoTune/`). The twitch tests,
//! gain updates, and `run` leftovers stay for a later slice. What
//! this file owns is `init`: the from-mode / throttle / flying
//! gates, the Loiter-or-PosHold position-hold bit, and
//! `AC_AutoTune::init_internals` TuneMode / first-axis leftover.
//!
//! # `init` ignores `ignore_checks`
//!
//! `ModeAutoTune::init` returns `autotune.init()` and never reads
//! `ignore_checks`. `AutoTune::init` refuses when the from-mode does
//! not `allows_autotune()`, when `throttle_zero` is set, or when the
//! aircraft is not flying (`!armed || !auto_armed || land_complete`).
//! The four from-modes that override `allows_autotune` to true are
//! Stabilize, AltHold, Loiter, and PosHold.
//!
//! Position hold while tuning is `mode == LOITER || mode == POSHOLD`.
//! The comment mentions QLOITER; on Copter that is Loiter. Passing
//! those gates calls `init_internals`, which seats the vertical
//! position controller, then branches on the current [`TuneMode`].
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_loiter::MODE_NUMBER_LOITER;
use crate::mode_poshold::MODE_NUMBER_POSHOLD;

/// `Mode::Number::AUTOTUNE`.
pub const MODE_NUMBER_AUTOTUNE: u8 = 15;

/// `Mode::Number::STABILIZE` — one of the four from-modes that allow AutoTune.
pub const MODE_NUMBER_STABILIZE: u8 = 0;

/// `Mode::Number::ALT_HOLD` — one of the four from-modes that allow AutoTune.
pub const MODE_NUMBER_ALT_HOLD: u8 = 2;

/// `AUTOTUNE_AXIS_BITMASK_ROLL`.
pub const AUTOTUNE_AXIS_BITMASK_ROLL: u8 = 1;

/// `AUTOTUNE_AXIS_BITMASK_PITCH`.
pub const AUTOTUNE_AXIS_BITMASK_PITCH: u8 = 2;

/// `AUTOTUNE_AXIS_BITMASK_YAW`.
pub const AUTOTUNE_AXIS_BITMASK_YAW: u8 = 4;

/// `AUTOTUNE_AXIS_BITMASK_YAW_D`.
pub const AUTOTUNE_AXIS_BITMASK_YAW_D: u8 = 8;

/// Default `AUTOTUNE_AXES` (`AP_GROUPINFO` value 7 = roll|pitch|yaw).
pub const AUTOTUNE_AXIS_BITMASK_DEFAULT: u8 = 7;

/// `AUTOTUNE_SUCCESS_COUNT` — successful twitches before a gain freezes.
pub const AUTOTUNE_SUCCESS_COUNT: u8 = 4;

/// `AUTOTUNE_MESSAGE_STARTED`.
pub const AUTOTUNE_MESSAGE_STARTED: u8 = 0;

/// `AUTOTUNE_MESSAGE_STOPPED`.
pub const AUTOTUNE_MESSAGE_STOPPED: u8 = 1;

/// `AUTOTUNE_MESSAGE_SUCCESS`.
pub const AUTOTUNE_MESSAGE_SUCCESS: u8 = 2;

/// `AUTOTUNE_MESSAGE_FAILED`.
pub const AUTOTUNE_MESSAGE_FAILED: u8 = 3;

/// `AUTOTUNE_MESSAGE_SAVED_GAINS`.
pub const AUTOTUNE_MESSAGE_SAVED_GAINS: u8 = 4;

/// `AUTOTUNE_MESSAGE_TESTING`.
pub const AUTOTUNE_MESSAGE_TESTING: u8 = 5;

/// `AUTOTUNE_MESSAGE_TESTING_END`.
pub const AUTOTUNE_MESSAGE_TESTING_END: u8 = 6;

/// `ModeAutoTune` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoTuneModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. False: the mode itself does not need GPS.
    pub requires_position: bool,
    /// `has_manual_throttle()`. False: throttle is automatic.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`. False: must already be flying.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
}

/// Upstream `ModeAutoTune` flags.
#[must_use]
pub const fn autotune_mode_flags() -> AutoTuneModeFlags {
    AutoTuneModeFlags {
        mode_number: MODE_NUMBER_AUTOTUNE,
        requires_position: false,
        has_manual_throttle: false,
        allows_arming: false,
        is_autopilot: false,
    }
}

/// Upstream `ModeAutoTune` does not override `has_user_takeoff`.
///
/// The base `Mode` leftover is `false`. AutoTune cannot start on the
/// ground — `init` already requires a flying aircraft.
#[must_use]
pub const fn autotune_has_user_takeoff(_must_navigate: bool) -> bool {
    false
}

/// Upstream `Mode::allows_autotune` catalog for the four overrides.
///
/// Base `Mode` returns false. Stabilize, AltHold, Loiter, and PosHold
/// override it to true. Every other Copter mode, including AutoTune
/// itself, stays on the base leftover.
#[must_use]
pub const fn allows_autotune(from_mode_number: u8) -> bool {
    matches!(
        from_mode_number,
        MODE_NUMBER_STABILIZE | MODE_NUMBER_ALT_HOLD | MODE_NUMBER_LOITER | MODE_NUMBER_POSHOLD
    )
}

/// Upstream `AutoTune::init` position-hold bit.
///
/// `true` when the from-mode is Loiter or PosHold. Stabilize and
/// AltHold enter AutoTune without holding NE.
#[must_use]
pub const fn autotune_use_poshold(from_mode_number: u8) -> bool {
    from_mode_number == MODE_NUMBER_LOITER || from_mode_number == MODE_NUMBER_POSHOLD
}

/// Upstream `AC_AutoTune::AxisType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AxisType {
    /// `ROLL`.
    Roll = 0,
    /// `PITCH`.
    Pitch = 1,
    /// `YAW` — tuned with FLTE.
    Yaw = 2,
    /// `YAW_D` — tuned with D. Heli builds compile this bit out.
    YawD = 3,
}

/// Upstream `AC_AutoTune::TuneMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TuneMode {
    /// `UNINITIALISED` — constructor / `reset()` leftover.
    Uninitialised = 0,
    /// `TUNING` — actively twitching and updating gains.
    Tuning = 1,
    /// `FINISHED` — original gains restored after a completed tune.
    Finished = 2,
    /// `FAILED` — original gains, restart on the next `init`.
    Failed = 3,
    /// `VALIDATING` — flying the newly tuned gains.
    Validating = 4,
}

/// Upstream `AC_AutoTune::Step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Step {
    /// `WAITING_FOR_LEVEL`.
    WaitingForLevel = 0,
    /// `EXECUTING_TEST`.
    ExecutingTest = 1,
    /// `UPDATE_GAINS`.
    UpdateGains = 2,
    /// `ABORT`.
    Abort = 3,
}

/// Why `ModeAutoTune::init` / `AutoTune::init` returned false.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoTuneInitFail {
    /// From-mode `allows_autotune()` is false.
    FromModeRefused,
    /// `copter.ap.throttle_zero`.
    ThrottleZero,
    /// `!armed || !auto_armed || land_complete`.
    NotFlying,
    /// `init_internals`: `motors == nullptr || !motors->armed()`.
    MotorsNotArmed,
}

/// What `ModeAutoTune::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct AutoTuneInitView {
    /// `copter.flightmode->mode_number()`.
    pub from_mode_number: u8,
    /// `copter.ap.throttle_zero`.
    pub throttle_zero: bool,
    /// `copter.motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `AP_Motors::get_singleton() != nullptr` at `init_internals`.
    pub motors_present: bool,
    /// `axis_bitmask` / `AUTOTUNE_AXES`. Default 7 (roll|pitch|yaw).
    pub axis_bitmask: u8,
    /// Tuner `mode` before this `init`. Constructor leaves
    /// [`TuneMode::Uninitialised`].
    pub mode: TuneMode,
    /// Current axis when resuming a `TUNING` / `VALIDATING` session.
    pub axis: AxisType,
}

impl AutoTuneInitView {
    /// Flying in Stabilize with the default axis mask, first start.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            from_mode_number: MODE_NUMBER_STABILIZE,
            throttle_zero: false,
            armed: true,
            auto_armed: true,
            land_complete: false,
            motors_present: true,
            axis_bitmask: AUTOTUNE_AXIS_BITMASK_DEFAULT,
            mode: TuneMode::Uninitialised,
            axis: AxisType::Roll,
        }
    }

    /// Flying in Loiter — the path that asks for position hold.
    #[must_use]
    pub const fn typical_loiter() -> Self {
        let mut view = Self::typical();
        view.from_mode_number = MODE_NUMBER_LOITER;
        view
    }
}

/// Leftover of one `ModeAutoTune::init` → `AutoTune::init` → `init_internals`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoTuneInit {
    /// `use_poshold` written by `init_internals`. `None` on a failed gate.
    pub use_poshold: Option<bool>,
    /// `init_position_controller()` ran (`init_z_limits` + `D_init_controller`).
    pub init_position_controller: bool,
    /// `backup_gains_and_initialise()` ran (first start or FAILED restart).
    pub backup_gains: bool,
    /// Tuner `mode` after `init`. `None` on a failed gate.
    pub mode: Option<TuneMode>,
    /// Axis after `init`. First enabled axis on a fresh start.
    pub axis: Option<AxisType>,
    /// `axes_completed` after a fresh start. `Some(0)` then; `None` on
    /// resume / validate / fail-before-internals.
    pub axes_completed: Option<u8>,
    /// `step` after a fresh start or TUNING resume. Always
    /// [`Step::WaitingForLevel`] on those paths.
    pub step: Option<Step>,
    /// `have_position` after `init_internals`. Always `false` on the
    /// passing path.
    pub have_position: Option<bool>,
    /// `update_gcs` message id. `STARTED` on start/resume, `TESTING`
    /// when entering VALIDATING.
    pub gcs_message: Option<u8>,
    /// Gate that fired, if any. `None` on the passing path.
    pub fail: Option<AutoTuneInitFail>,
    /// `true` only when every gate passed. `ignore_checks` cannot
    /// bypass any of them.
    pub ok: bool,
}

fn failed(fail: AutoTuneInitFail) -> AutoTuneInit {
    AutoTuneInit {
        use_poshold: None,
        init_position_controller: false,
        backup_gains: false,
        mode: None,
        axis: None,
        axes_completed: None,
        step: None,
        have_position: None,
        gcs_message: None,
        fail: Some(fail),
        ok: false,
    }
}

/// `AC_AutoTune::roll_enabled`.
#[must_use]
pub const fn roll_enabled(axis_bitmask: u8) -> bool {
    axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL != 0
}

/// `AC_AutoTune::pitch_enabled`.
#[must_use]
pub const fn pitch_enabled(axis_bitmask: u8) -> bool {
    axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH != 0
}

/// `AC_AutoTune::yaw_enabled`.
#[must_use]
pub const fn yaw_enabled(axis_bitmask: u8) -> bool {
    axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW != 0
}

/// `AC_AutoTune::yaw_d_enabled` on a multicopter build.
///
/// Heli compiles this to `false`. This leftover is the Multi path.
#[must_use]
pub const fn yaw_d_enabled(axis_bitmask: u8) -> bool {
    axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW_D != 0
}

/// First axis `backup_gains_and_initialise` selects.
///
/// Roll, then pitch, then yaw, then yaw-D. `None` when the mask is
/// empty — upstream then leaves `axis` untouched.
#[must_use]
pub const fn first_enabled_axis(axis_bitmask: u8) -> Option<AxisType> {
    if roll_enabled(axis_bitmask) {
        Some(AxisType::Roll)
    } else if pitch_enabled(axis_bitmask) {
        Some(AxisType::Pitch)
    } else if yaw_enabled(axis_bitmask) {
        Some(AxisType::Yaw)
    } else if yaw_d_enabled(axis_bitmask) {
        Some(AxisType::YawD)
    } else {
        None
    }
}

/// Upstream `ModeAutoTune::init`. `ignore_checks` is unread.
///
/// The three `AutoTune::init` gates run first. A passing path then
/// runs `init_internals`: seat the D controller, then branch on
/// [`TuneMode`]. FAILED falls through into the UNINITIALISED start
/// (backup gains, first axis, `TUNING`, GCS STARTED). TUNING resumes
/// at `WAITING_FOR_LEVEL`. FINISHED and VALIDATING become VALIDATING
/// with GCS TESTING.
#[must_use]
pub fn mode_autotune_init(_ignore_checks: bool, view: &AutoTuneInitView) -> AutoTuneInit {
    if !allows_autotune(view.from_mode_number) {
        return failed(AutoTuneInitFail::FromModeRefused);
    }
    if view.throttle_zero {
        return failed(AutoTuneInitFail::ThrottleZero);
    }
    if !view.armed || !view.auto_armed || view.land_complete {
        return failed(AutoTuneInitFail::NotFlying);
    }
    if !view.motors_present || !view.armed {
        return failed(AutoTuneInitFail::MotorsNotArmed);
    }

    let use_poshold = autotune_use_poshold(view.from_mode_number);

    match view.mode {
        TuneMode::Failed | TuneMode::Uninitialised => AutoTuneInit {
            use_poshold: Some(use_poshold),
            init_position_controller: true,
            backup_gains: true,
            mode: Some(TuneMode::Tuning),
            axis: first_enabled_axis(view.axis_bitmask).or(Some(view.axis)),
            axes_completed: Some(0),
            step: Some(Step::WaitingForLevel),
            have_position: Some(false),
            gcs_message: Some(AUTOTUNE_MESSAGE_STARTED),
            fail: None,
            ok: true,
        },
        TuneMode::Tuning => AutoTuneInit {
            use_poshold: Some(use_poshold),
            init_position_controller: true,
            backup_gains: false,
            mode: Some(TuneMode::Tuning),
            axis: Some(view.axis),
            axes_completed: None,
            step: Some(Step::WaitingForLevel),
            have_position: Some(false),
            gcs_message: Some(AUTOTUNE_MESSAGE_STARTED),
            fail: None,
            ok: true,
        },
        TuneMode::Finished | TuneMode::Validating => AutoTuneInit {
            use_poshold: Some(use_poshold),
            init_position_controller: true,
            backup_gains: false,
            mode: Some(TuneMode::Validating),
            axis: Some(view.axis),
            axes_completed: None,
            step: None,
            have_position: Some(false),
            gcs_message: Some(AUTOTUNE_MESSAGE_TESTING),
            fail: None,
            ok: true,
        },
    }
}
