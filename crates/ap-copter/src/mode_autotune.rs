//! `ModeAutoTune` init / run leftover, upstream `ArduCopter/mode_autotune.cpp`.
//!
//! Tracked as **COP-027**. Copter AutoTune is a thin wrapper around
//! `AC_AutoTune` / `AC_AutoTune_Multi` (`libraries/AC_AutoTune/`). The
//! twitch tests (`test_run` / `Step::EXECUTING_TEST` physics), the
//! `Step::UPDATE_GAINS` tune-type switch, and the Multi library stay
//! for a later slice. What this file owns is `init` (from-mode /
//! throttle / flying gates, Loiter-or-PosHold, TuneMode / first-axis)
//! and `run` (Copter land/disarm wrapper, TuneMode dispatch, pilot
//! override, and the level / execute / abort loop).
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
//! # `run` is a Copter wrapper around `AC_AutoTune::run`
//!
//! `ModeAutoTune::run` is `autotune.run()`. The Copter subclass
//! applies SIMPLE, disarms when landed at ground idle, and returns
//! through `make_safe_ground_handling` whenever `land_complete` is
//! set. Only a flying tick reaches the library loop: `init_z_limits`,
//! the armed/interlock gate, pilot RP/yaw/climb, the optional poshold
//! latch, then the [`TuneMode`] switch. TUNING either flies original
//! gains under a stick override or runs [`control_attitude`]. FINISHED
//! / FAILED fly original; VALIDATING flies tuned. UNINITIALISED is a
//! flow-of-control error and falls through into the original-gains
//! path. A passing tick always ends on `THROTTLE_UNLIMITED` and a D
//! controller update.
//!
//! `control_attitude` is the twitch / level / execute / abort loop.
//! WAITING_FOR_LEVEL holds intra-test gains until [`currently_level`]
//! has been true for [`AUTOTUNE_REQUIRED_LEVEL_TIME_MS`], then starts
//! EXECUTING_TEST. The twitch body is a later leftover — this tick
//! takes an already-decided [`TwitchTick`]. UPDATE_GAINS is catalogued
//! as a flag and falls through into ABORT, which returns to
//! WAITING_FOR_LEVEL and reverses the Multi test direction.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_loiter::MODE_NUMBER_LOITER;
use crate::mode_poshold::MODE_NUMBER_POSHOLD;
use ap_math::scalar::{cd_to_rad, constrain_value, is_zero, wrap_pi};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

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

/// Copter `AUTOTUNE_LEVEL_ANGLE_CD`. Plane uses 500.
pub const AUTOTUNE_LEVEL_ANGLE_CD: f32 = 250.0;

/// Copter `AUTOTUNE_LEVEL_RATE_RP_CD`. Plane uses 1000.
pub const AUTOTUNE_LEVEL_RATE_RP_CD: f32 = 500.0;

/// `AUTOTUNE_LEVEL_RATE_Y_CD`.
pub const AUTOTUNE_LEVEL_RATE_Y_CD: f32 = 750.0;

/// `AUTOTUNE_REQUIRED_LEVEL_TIME_MS`.
pub const AUTOTUNE_REQUIRED_LEVEL_TIME_MS: u32 = 250;

/// `AUTOTUNE_LEVEL_TIMEOUT_MS`.
pub const AUTOTUNE_LEVEL_TIMEOUT_MS: u32 = 2000;

/// `AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS`. Comment says two seconds; the
/// define is 500 ms.
pub const AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS: u32 = 500;

/// Pilot-override GCS warning interval, ms.
pub const AUTOTUNE_PILOT_OVERRIDE_WARN_MS: u32 = 1000;

/// Multi `AUTOTUNE_TESTING_STEP_TIMEOUT_MS`. Twitch leftover input.
pub const AUTOTUNE_TESTING_STEP_TIMEOUT_MS: u32 = 2000;

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

/// Upstream `AC_AutoTune::GainType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GainType {
    /// Gains as configured before autotune started.
    Original = 0,
    /// Gains applied during an active test.
    Test = 1,
    /// Gains between tests, slower I-term buildup.
    IntraTest = 2,
    /// Gains discovered by the autotune process.
    Tuned = 3,
}

/// What Multi `test_run` decided this tick.
///
/// The twitch body (`AC_AutoTune_Multi::test_run`) stays a later
/// leftover. `control_attitude` takes the already-decided step write
/// the same way SystemID takes an already-computed chirp sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwitchTick {
    /// Still running. `step` stays [`Step::ExecutingTest`].
    Running,
    /// `test_run` wrote [`Step::UpdateGains`].
    Done,
    /// `test_run` wrote [`Step::Abort`].
    Aborted,
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

/// Multi `reverse_test_direction`. Heli is out of scope.
#[must_use]
pub const fn reverse_test_direction(positive_direction: bool) -> bool {
    !positive_direction
}

/// What `currently_level` returns, including the writes it does on
/// the way out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentlyLevel {
    /// Return value.
    pub level: bool,
    /// `level_start_time_ms` after the yaw-slew reset, if any.
    pub level_start_time_ms: u32,
    /// `mode` was written to [`TuneMode::Failed`] (3 × timeout).
    pub failed: bool,
}

/// Attitude / rate leftover that `currently_level` reads.
#[derive(Debug, Clone, Copy)]
pub struct CurrentlyLevelView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `level_start_time_ms` before this call.
    pub level_start_time_ms: u32,
    /// Target roll, rad.
    pub desired_roll_rad: f32,
    /// Target pitch, rad.
    pub desired_pitch_rad: f32,
    /// Target yaw, rad.
    pub desired_yaw_rad: f32,
    /// `ahrs_view->get_roll_rad()`.
    pub roll_rad: f32,
    /// `ahrs_view->get_pitch_rad()`.
    pub pitch_rad: f32,
    /// `ahrs_view->get_yaw_rad()`.
    pub yaw_rad: f32,
    /// `ahrs_view->get_gyro().x`.
    pub gyro_x: f32,
    /// `ahrs_view->get_gyro().y`.
    pub gyro_y: f32,
    /// `ahrs_view->get_gyro().z`.
    pub gyro_z: f32,
    /// `attitude_control->get_rate_ef_target_rads().z`.
    pub yaw_rate_ef_target_rads: f32,
    /// `attitude_control->get_slew_yaw_max_rads()`.
    pub slew_yaw_max_rads: f32,
}

/// Upstream `AC_AutoTune::currently_level`.
///
/// The gyro checks are `>` not `fabsf` — a leftover of the C++ as
/// written. Negative body rates do not fail the level gate.
#[must_use]
pub fn autotune_currently_level(view: &CurrentlyLevelView) -> CurrentlyLevel {
    let mut level_start_time_ms = view.level_start_time_ms;
    let mut failed = false;

    if view.yaw_rate_ef_target_rads.abs() > 0.5 * view.slew_yaw_max_rads {
        level_start_time_ms = view.now_ms;
    }
    if view.now_ms.wrapping_sub(level_start_time_ms) > 3 * AUTOTUNE_LEVEL_TIMEOUT_MS {
        failed = true;
    }

    let elapsed = view.now_ms.wrapping_sub(level_start_time_ms) as f32;
    let threshold_mul = constrain_value(elapsed / AUTOTUNE_LEVEL_TIMEOUT_MS as f32, 0.0, 2.0);
    let angle_lim = threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD);
    let rate_rp_lim = threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_RATE_RP_CD);
    let rate_y_lim = threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_RATE_Y_CD);

    let level = (view.roll_rad - view.desired_roll_rad).abs() <= angle_lim
        && (view.pitch_rad - view.desired_pitch_rad).abs() <= angle_lim
        && wrap_pi(view.yaw_rad - view.desired_yaw_rad).abs() <= angle_lim
        && view.gyro_x <= rate_rp_lim
        && view.gyro_y <= rate_rp_lim
        && view.gyro_z <= rate_y_lim;

    CurrentlyLevel {
        level,
        level_start_time_ms,
        failed,
    }
}

/// What `ModeAutoTune::run` / `AC_AutoTune::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct AutoTuneRunView {
    /// Tuner `mode` before this tick.
    pub mode: TuneMode,
    /// Tuner `step` before this tick.
    pub step: Step,
    /// Current axis.
    pub axis: AxisType,
    /// `motors->armed()`.
    pub armed: bool,
    /// `motors->get_interlock()`.
    pub interlock: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `use_poshold` from `init_internals`.
    pub use_poshold: bool,
    /// `have_position` before this tick.
    pub have_position: bool,
    /// `position_ok()` — Copter `copter.position_ok()`.
    pub position_ok: bool,
    /// Pilot roll after SIMPLE, rad. Read before poshold overwrites it.
    pub desired_roll_rad: f32,
    /// Pilot pitch after SIMPLE, rad.
    pub desired_pitch_rad: f32,
    /// Pilot yaw rate, rad/s.
    pub desired_yaw_rate_rads: f32,
    /// Held yaw target, rad.
    pub desired_yaw_rad: f32,
    /// Pilot climb rate after avoidance, m/s.
    pub target_climb_rate_ms: f32,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `override_time` before this tick.
    pub override_time: u32,
    /// `last_pilot_override_warning` before this tick.
    pub last_pilot_override_warning: u32,
    /// `pilot_override` before this tick.
    pub pilot_override: bool,
    /// `step_start_time_ms` before this tick.
    pub step_start_time_ms: u32,
    /// `level_start_time_ms` before this tick.
    pub level_start_time_ms: u32,
    /// `step_timeout_ms` before this tick.
    pub step_timeout_ms: u32,
    /// Multi `get_testing_step_timeout_ms()`. Twitch leftover input.
    pub testing_step_timeout_ms: u32,
    /// `positive_direction` before this tick.
    pub positive_direction: bool,
    /// `ahrs_view->get_roll_rad()`.
    pub roll_rad: f32,
    /// `ahrs_view->get_pitch_rad()`.
    pub pitch_rad: f32,
    /// `ahrs_view->get_yaw_rad()`.
    pub yaw_rad: f32,
    /// `ahrs_view->get_gyro().x`.
    pub gyro_x: f32,
    /// `ahrs_view->get_gyro().y`.
    pub gyro_y: f32,
    /// `ahrs_view->get_gyro().z`.
    pub gyro_z: f32,
    /// `attitude_control->get_rate_ef_target_rads().z`.
    pub yaw_rate_ef_target_rads: f32,
    /// `attitude_control->get_slew_yaw_max_rads()`.
    pub slew_yaw_max_rads: f32,
    /// Already-decided Multi `test_run` leftover.
    pub twitch: TwitchTick,
    /// `lean_angle` member after `test_run`, centidegrees.
    pub lean_angle_cd: f32,
    /// `attitude_control->lean_angle_deg()`.
    pub lean_angle_deg: f32,
    /// Multi `angle_lim_neg_rpy_cd()`.
    pub angle_lim_neg_rpy_cd: f32,
    /// Multi `angle_lim_max_rp_cd()`.
    pub angle_lim_max_rp_cd: f32,
}

impl AutoTuneRunView {
    /// Flying, TUNING, WAITING_FOR_LEVEL, sticks centered, not yet
    /// held long enough to start a twitch.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            mode: TuneMode::Tuning,
            step: Step::WaitingForLevel,
            axis: AxisType::Roll,
            armed: true,
            interlock: true,
            land_complete: false,
            spool_state: SpoolState::ThrottleUnlimited,
            use_poshold: false,
            have_position: false,
            position_ok: false,
            desired_roll_rad: 0.0,
            desired_pitch_rad: 0.0,
            desired_yaw_rate_rads: 0.0,
            desired_yaw_rad: 0.0,
            target_climb_rate_ms: 0.0,
            now_ms: 10_000,
            override_time: 0,
            last_pilot_override_warning: 0,
            pilot_override: false,
            step_start_time_ms: 9_800,
            level_start_time_ms: 8_000,
            step_timeout_ms: AUTOTUNE_REQUIRED_LEVEL_TIME_MS,
            testing_step_timeout_ms: AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
            positive_direction: true,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            yaw_rate_ef_target_rads: 0.0,
            slew_yaw_max_rads: 1.0,
            twitch: TwitchTick::Running,
            lean_angle_cd: 0.0,
            lean_angle_deg: 0.0,
            angle_lim_neg_rpy_cd: 900.0,
            angle_lim_max_rp_cd: 3750.0,
        }
    }
}

/// Leftover of one `ModeAutoTune::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTuneRun {
    /// Always true: `copter.update_simple_mode()`.
    pub update_simple_mode: bool,
    /// `arming.disarm(LANDED)` — landed at ground idle.
    pub disarmed_landed: bool,
    /// `make_safe_ground_handling()` ran and the Copter wrapper returned.
    pub make_safe_ground_handling: bool,
    /// `AC_AutoTune::run()` was entered.
    pub library_run: bool,
    /// `init_z_limits()` ran at the top of the library loop.
    pub init_z_limits: bool,
    /// Desired spool write. `None` on the Copter wrapper early return.
    pub desired_spool: Option<DesiredSpoolState>,
    /// `set_throttle_out(0)` on the armed/interlock gate.
    pub throttle_out_zero: bool,
    /// `D_relax_controller(0)` on the armed/interlock gate.
    pub d_relax: bool,
    /// `D_set_pos_target_from_climb_rate_ms` ran.
    pub set_climb_rate: bool,
    /// `D_update_controller` ran.
    pub d_update: bool,
    /// `get_poshold_attitude_rad` was called (zero RP input).
    pub poshold_called: bool,
    /// `have_position` after the tick.
    pub have_position: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — `UNINITIALISED` on `run`.
    pub flow_of_control: bool,
    /// Tuner `mode` after the tick.
    pub mode: TuneMode,
    /// Tuner `step` after the tick.
    pub step: Step,
    /// `pilot_override` after the tick.
    pub pilot_override: bool,
    /// `override_time` after the tick.
    pub override_time: u32,
    /// `last_pilot_override_warning` after the tick.
    pub last_pilot_override_warning: u32,
    /// GCS "pilot overrides active" this tick.
    pub pilot_override_warning: bool,
    /// `load_gains` this tick. `None` on early returns.
    pub loaded_gains: Option<GainType>,
    /// `input_euler_angle_roll_pitch_euler_rate_yaw_rad` (pilot fly).
    pub input_euler_rp_yaw_rate: bool,
    /// `input_euler_angle_roll_pitch_yaw_rad` (level hold / abort).
    pub input_euler_rp_yaw: bool,
    /// `control_attitude()` ran.
    pub control_attitude: bool,
    /// `do_gcs_announcements()` ran.
    pub do_gcs_announcements: bool,
    /// `currently_level()` return, if it ran.
    pub currently_level: Option<bool>,
    /// `currently_level` wrote [`TuneMode::Failed`].
    pub failed_to_level: bool,
    /// `test_init()` ran (WAITING → EXECUTING).
    pub test_init: bool,
    /// `test_run()` leftover was invoked.
    pub test_run: bool,
    /// `Step::UPDATE_GAINS` body ran. The tune-type switch stays leftover.
    pub update_gains: bool,
    /// `positive_direction` after Multi reverse, if ABORT ran.
    pub positive_direction: bool,
    /// Held yaw after override-release / yaw-twitch update.
    pub desired_yaw_rad: f32,
    /// `step_start_time_ms` after the tick.
    pub step_start_time_ms: u32,
    /// `level_start_time_ms` after the tick.
    pub level_start_time_ms: u32,
    /// `step_timeout_ms` after the tick.
    pub step_timeout_ms: u32,
}

fn run_passthrough(view: &AutoTuneRunView) -> AutoTuneRun {
    AutoTuneRun {
        update_simple_mode: true,
        disarmed_landed: false,
        make_safe_ground_handling: false,
        library_run: false,
        init_z_limits: false,
        desired_spool: None,
        throttle_out_zero: false,
        d_relax: false,
        set_climb_rate: false,
        d_update: false,
        poshold_called: false,
        have_position: view.have_position,
        flow_of_control: false,
        mode: view.mode,
        step: view.step,
        pilot_override: view.pilot_override,
        override_time: view.override_time,
        last_pilot_override_warning: view.last_pilot_override_warning,
        pilot_override_warning: false,
        loaded_gains: None,
        input_euler_rp_yaw_rate: false,
        input_euler_rp_yaw: false,
        control_attitude: false,
        do_gcs_announcements: false,
        currently_level: None,
        failed_to_level: false,
        test_init: false,
        test_run: false,
        update_gains: false,
        positive_direction: view.positive_direction,
        desired_yaw_rad: view.desired_yaw_rad,
        step_start_time_ms: view.step_start_time_ms,
        level_start_time_ms: view.level_start_time_ms,
        step_timeout_ms: view.step_timeout_ms,
    }
}

fn control_attitude(view: &AutoTuneRunView, out: &mut AutoTuneRun) {
    out.control_attitude = true;
    let now = view.now_ms;

    match view.step {
        Step::WaitingForLevel => {
            out.loaded_gains = Some(GainType::IntraTest);
            out.input_euler_rp_yaw = true;

            let level = autotune_currently_level(&CurrentlyLevelView {
                now_ms: now,
                level_start_time_ms: view.level_start_time_ms,
                desired_roll_rad: view.desired_roll_rad,
                desired_pitch_rad: view.desired_pitch_rad,
                desired_yaw_rad: out.desired_yaw_rad,
                roll_rad: view.roll_rad,
                pitch_rad: view.pitch_rad,
                yaw_rad: view.yaw_rad,
                gyro_x: view.gyro_x,
                gyro_y: view.gyro_y,
                gyro_z: view.gyro_z,
                yaw_rate_ef_target_rads: view.yaw_rate_ef_target_rads,
                slew_yaw_max_rads: view.slew_yaw_max_rads,
            });
            out.currently_level = Some(level.level);
            out.level_start_time_ms = level.level_start_time_ms;
            if level.failed {
                out.failed_to_level = true;
                out.mode = TuneMode::Failed;
            }
            if !level.level {
                out.step_start_time_ms = now;
            }
            if now.wrapping_sub(out.step_start_time_ms) > AUTOTUNE_REQUIRED_LEVEL_TIME_MS {
                out.step = Step::ExecutingTest;
                out.step_start_time_ms = now;
                out.step_timeout_ms = view.testing_step_timeout_ms;
                out.loaded_gains = Some(GainType::Test);
                out.test_init = true;
            }
        }
        Step::ExecutingTest => {
            out.loaded_gains = Some(GainType::Test);
            out.test_run = true;
            out.step = match view.twitch {
                TwitchTick::Running => Step::ExecutingTest,
                TwitchTick::Done => Step::UpdateGains,
                TwitchTick::Aborted => Step::Abort,
            };
            if view.lean_angle_cd <= -view.angle_lim_neg_rpy_cd
                || view.lean_angle_deg * 100.0 > view.angle_lim_max_rp_cd
            {
                out.step = Step::Abort;
            }
            if matches!(view.axis, AxisType::Yaw | AxisType::YawD) {
                out.desired_yaw_rad = view.yaw_rad;
            }
        }
        Step::UpdateGains => {
            // Tune-type switch / success_counter / next-axis stay leftover.
            out.update_gains = true;
            abort_to_level(view, out, now);
        }
        Step::Abort => {
            abort_to_level(view, out, now);
        }
    }
}

fn abort_to_level(view: &AutoTuneRunView, out: &mut AutoTuneRun, now: u32) {
    out.input_euler_rp_yaw = true;
    out.loaded_gains = Some(GainType::IntraTest);
    out.step = Step::WaitingForLevel;
    out.positive_direction = reverse_test_direction(view.positive_direction);
    out.step_start_time_ms = now;
    out.level_start_time_ms = now;
    out.step_timeout_ms = AUTOTUNE_REQUIRED_LEVEL_TIME_MS;
}

/// Upstream `ModeAutoTune::run` → Copter `AutoTune::run` → `AC_AutoTune::run`.
///
/// Twitch physics and the UPDATE_GAINS tune-type switch stay leftovers.
/// Poshold lean math (`get_poshold_attitude_rad` 10° / 20 m) is also
/// leftover — this catalogs the call and the `have_position` latch.
#[must_use]
pub fn mode_autotune_run(view: &AutoTuneRunView) -> AutoTuneRun {
    let mut out = run_passthrough(view);
    out.update_simple_mode = true;

    if view.land_complete && view.spool_state == SpoolState::GroundIdle {
        out.disarmed_landed = true;
    }
    if view.land_complete {
        out.make_safe_ground_handling = true;
        return out;
    }

    out.library_run = true;
    out.init_z_limits = true;

    if !view.armed || !view.interlock {
        out.desired_spool = Some(DesiredSpoolState::GroundIdle);
        out.throttle_out_zero = true;
        out.d_relax = true;
        return out;
    }

    let zero_rp = is_zero(view.desired_roll_rad) && is_zero(view.desired_pitch_rad);
    if zero_rp {
        out.poshold_called = true;
        if view.use_poshold && view.position_ok && !view.have_position {
            out.have_position = true;
        }
    }

    let mut desired_yaw_rad = view.desired_yaw_rad;
    let mut pilot_override = view.pilot_override;
    let mut override_time = view.override_time;
    let mut last_warn = view.last_pilot_override_warning;
    let mut step = view.step;
    let mut step_start = view.step_start_time_ms;
    let mut level_start = view.level_start_time_ms;

    match view.mode {
        TuneMode::Tuning => {
            if !zero_rp
                || !is_zero(view.desired_yaw_rate_rads)
                || !is_zero(view.target_climb_rate_ms)
            {
                if !pilot_override {
                    pilot_override = true;
                }
                override_time = view.now_ms;
                if !zero_rp {
                    out.have_position = false;
                }
            } else if pilot_override
                && view.now_ms.wrapping_sub(override_time) > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS
            {
                pilot_override = false;
                step = Step::WaitingForLevel;
                step_start = view.now_ms;
                level_start = view.now_ms;
                desired_yaw_rad = view.yaw_rad;
            }

            out.pilot_override = pilot_override;
            out.override_time = override_time;
            out.step = step;
            out.step_start_time_ms = step_start;
            out.level_start_time_ms = level_start;
            out.desired_yaw_rad = desired_yaw_rad;

            if pilot_override {
                if view.now_ms.wrapping_sub(last_warn) > AUTOTUNE_PILOT_OVERRIDE_WARN_MS {
                    out.pilot_override_warning = true;
                    last_warn = view.now_ms;
                }
                out.last_pilot_override_warning = last_warn;
                out.loaded_gains = Some(GainType::Original);
                out.input_euler_rp_yaw_rate = true;
            } else {
                out.last_pilot_override_warning = last_warn;
                let mut attitude_view = *view;
                attitude_view.step = step;
                attitude_view.step_start_time_ms = step_start;
                attitude_view.level_start_time_ms = level_start;
                attitude_view.desired_yaw_rad = desired_yaw_rad;
                control_attitude(&attitude_view, &mut out);
                out.do_gcs_announcements = true;
            }
        }
        TuneMode::Uninitialised => {
            out.flow_of_control = true;
            out.loaded_gains = Some(GainType::Original);
            out.input_euler_rp_yaw_rate = true;
        }
        TuneMode::Failed | TuneMode::Finished => {
            out.loaded_gains = Some(GainType::Original);
            out.input_euler_rp_yaw_rate = true;
        }
        TuneMode::Validating => {
            out.loaded_gains = Some(GainType::Tuned);
            out.input_euler_rp_yaw_rate = true;
        }
    }

    out.desired_spool = Some(DesiredSpoolState::ThrottleUnlimited);
    out.set_climb_rate = true;
    out.d_update = true;
    out
}
