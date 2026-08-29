//! `ModeAutoTune` init / run leftover, upstream `ArduCopter/mode_autotune.cpp`.
//!
//! Tracked as **COP-027**. Copter AutoTune is a thin wrapper around
//! `AC_AutoTune` / `AC_AutoTune_Multi` (`libraries/AC_AutoTune/`). The
//! Multi `updating_*` math now lives in [`crate::autotune_update_gains`].
//! `next_tune_type`, next-axis, backoff, and the rest of the Multi
//! library stay for a later slice. What this file owns is `init`
//! (from-mode / throttle / flying gates, Loiter-or-PosHold, TuneMode /
//! first-axis), `run` (Copter land/disarm wrapper, TuneMode dispatch,
//! pilot override, and the level / execute / abort loop), and the Multi
//! `test_run` / twitching leftover that decides [`TwitchTick`].
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
//! EXECUTING_TEST. Multi `test_run` leftover then captures lean
//! angle, catalogues the attitude command, and runs the twitching
//! helpers that write [`TwitchTick`]. The rotation-rate LPF stays
//! leftover — this tick takes the already-filtered `rotation_rate`.
//! UPDATE_GAINS runs the Multi tune-type switch leftover, then falls
//! through into ABORT, which returns to WAITING_FOR_LEVEL and reverses
//! the Multi test direction. Sequencing (`next_tune_type` / next-axis)
//! stays leftover.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_loiter::MODE_NUMBER_LOITER;
use crate::mode_poshold::MODE_NUMBER_POSHOLD;
use ap_math::scalar::{
    cd_to_rad, constrain_value, is_equal, is_zero, rad_to_cd, wrap_180_cd, wrap_pi,
};
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

/// What Multi `test_run` leftover writes onto `step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwitchTick {
    /// Still running. `step` stays [`Step::ExecutingTest`].
    Running,
    /// `test_run` wrote [`Step::UpdateGains`].
    Done,
    /// `test_run` wrote [`Step::Abort`].
    Aborted,
}

/// Multi `AC_AutoTune::TuneType`.
///
/// `test_run` switches on these. `RATE_FF_UP` / `MAX_GAINS` /
/// `TUNE_CHECK` are Heli leftovers and trip
/// `INTERNAL_ERROR(flow_of_control)` on Multi. `TUNE_COMPLETE` is the
/// sequence terminator and is not a twitch type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TuneType {
    /// `RATE_D_UP`.
    RateDUp = 0,
    /// `RATE_D_DOWN`.
    RateDDown = 1,
    /// `RATE_P_UP`.
    RatePUp = 2,
    /// `RATE_FF_UP` — Heli. Multi `test_run` treats this as flow-of-control.
    RateFfUp = 3,
    /// `ANGLE_P_DOWN`.
    AnglePDown = 4,
    /// `ANGLE_P_UP`.
    AnglePUp = 5,
    /// `MAX_GAINS` — Heli. Multi `test_run` treats this as flow-of-control.
    MaxGains = 6,
    /// `TUNE_CHECK` — Heli. Multi `test_run` treats this as flow-of-control.
    TuneCheck = 7,
    /// `TUNE_COMPLETE`.
    TuneComplete = 8,
}

/// Default `AUTOTUNE_AGGR`.
pub const AUTOTUNE_AGGR_DEFAULT: f32 = 0.075;

/// `AUTOTUNE_TARGET_RATE_RLLPIT_CDS`.
pub const AUTOTUNE_TARGET_RATE_RLLPIT_CDS: f32 = 18_000.0;

/// `AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS`.
pub const AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS: f32 = 4_500.0;

/// `AUTOTUNE_TARGET_RATE_YAW_CDS`.
pub const AUTOTUNE_TARGET_RATE_YAW_CDS: f32 = 9_000.0;

/// `AUTOTUNE_TARGET_MIN_RATE_YAW_CDS`.
pub const AUTOTUNE_TARGET_MIN_RATE_YAW_CDS: f32 = 1_500.0;

/// `AUTOTUNE_Y_FILT_FREQ` — yaw rate-filter leftover input, not LPF math.
pub const AUTOTUNE_Y_FILT_FREQ: f32 = 10.0;

/// Multi `direction_sign` from `positive_direction`.
#[must_use]
pub const fn direction_sign(positive_direction: bool) -> f32 {
    if positive_direction {
        1.0
    } else {
        -1.0
    }
}

/// Whether this Multi tune type steps an angle target instead of a rate.
#[must_use]
pub const fn twitch_is_angle_p(tune_type: TuneType) -> bool {
    matches!(tune_type, TuneType::AnglePDown | TuneType::AnglePUp)
}

/// Heli-only tune types that Multi `test_run` must not see.
#[must_use]
pub const fn twitch_is_heli_only(tune_type: TuneType) -> bool {
    matches!(
        tune_type,
        TuneType::RateFfUp | TuneType::MaxGains | TuneType::TuneCheck
    )
}

/// Axis lean angle in centidegrees after `dir_sign * (sensor - start)`.
///
/// Yaw uses integer `wrap_180_cd`. Roll and pitch are raw sensor
/// subtraction. `start_angle` is truncated to `int32` the same way
/// upstream casts it.
#[must_use]
pub fn twitch_lean_angle_cd(
    axis: AxisType,
    dir_sign: f32,
    roll_rad: f32,
    pitch_rad: f32,
    yaw_rad: f32,
    start_angle: f32,
) -> f32 {
    let start = start_angle as i32;
    match axis {
        AxisType::Roll => dir_sign * (rad_to_cd(roll_rad) as i32 - start) as f32,
        AxisType::Pitch => dir_sign * (rad_to_cd(pitch_rad) as i32 - start) as f32,
        AxisType::Yaw | AxisType::YawD => {
            dir_sign * wrap_180_cd(rad_to_cd(yaw_rad) as i32 - start) as f32
        }
    }
}

/// What `twitching_test_rate` writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwitchingRate {
    /// `meas_rate_min` after the tick.
    pub meas_rate_min: f32,
    /// `meas_rate_max` after the tick.
    pub meas_rate_max: f32,
    /// `meas_angle_min` after the tick.
    pub meas_angle_min: f32,
    /// `step_timeout_ms` after the 63.21% stretch.
    pub step_timeout_ms: u32,
    /// `Step::UPDATE_GAINS` when the rate, bounce, or timeout fires.
    pub done: bool,
}

/// Upstream `AC_AutoTune_Multi::twitching_test_rate`.
#[must_use]
pub fn twitching_test_rate(
    angle: f32,
    rate: f32,
    rate_target_max: f32,
    meas_rate_min: f32,
    meas_rate_max: f32,
    meas_angle_min: f32,
    now_ms: u32,
    step_start_time_ms: u32,
    step_timeout_ms: u32,
    aggressiveness: f32,
) -> TwitchingRate {
    let mut meas_rate_min = meas_rate_min;
    let mut meas_rate_max = meas_rate_max;
    let mut meas_angle_min = meas_angle_min;
    let mut step_timeout_ms = step_timeout_ms;
    let mut done = false;

    if rate > meas_rate_max {
        meas_rate_max = rate;
        meas_rate_min = rate;
        meas_angle_min = angle;
    }
    if rate < meas_rate_min && meas_rate_max > rate_target_max * 0.25 {
        meas_rate_min = rate;
        meas_angle_min = angle;
    }

    let elapsed = now_ms.wrapping_sub(step_start_time_ms);
    if meas_rate_max < rate_target_max * 0.6321 {
        step_timeout_ms = elapsed
            .wrapping_mul(3)
            .min(AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }
    if meas_rate_max > rate_target_max {
        done = true;
    }
    if meas_rate_max - meas_rate_min > meas_rate_max * aggressiveness {
        done = true;
    }
    if elapsed >= step_timeout_ms {
        done = true;
    }

    TwitchingRate {
        meas_rate_min,
        meas_rate_max,
        meas_angle_min,
        step_timeout_ms,
        done,
    }
}

/// What `twitching_abort_rate` writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwitchingAbort {
    /// `step_scaler` after a 0.9 shrink, if any.
    pub step_scaler: f32,
    /// `mode` was written to [`TuneMode::Failed`].
    pub failed: bool,
    /// `LogEvent::AUTOTUNE_REACHED_LIMIT` / GCS critical.
    pub reached_limit: bool,
    /// Step write. `None` when `angle < angle_max`.
    pub tick: Option<TwitchTick>,
}

/// Upstream `AC_AutoTune_Multi::twitching_abort_rate`.
#[must_use]
pub fn twitching_abort_rate(
    angle: f32,
    rate: f32,
    angle_max: f32,
    meas_rate_min: f32,
    angle_min: f32,
    step_scaler: f32,
) -> TwitchingAbort {
    if angle < angle_max {
        return TwitchingAbort {
            step_scaler,
            failed: false,
            reached_limit: false,
            tick: None,
        };
    }
    if is_equal(rate, meas_rate_min) || angle_min > 0.95 * angle_max {
        if step_scaler > 0.2 {
            TwitchingAbort {
                step_scaler: step_scaler * 0.9,
                failed: false,
                reached_limit: false,
                tick: Some(TwitchTick::Aborted),
            }
        } else {
            TwitchingAbort {
                step_scaler,
                failed: true,
                reached_limit: true,
                tick: Some(TwitchTick::Aborted),
            }
        }
    } else {
        TwitchingAbort {
            step_scaler,
            failed: false,
            reached_limit: false,
            tick: Some(TwitchTick::Done),
        }
    }
}

/// What `twitching_test_angle` writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwitchingAngle {
    /// `meas_angle_min` after the tick.
    pub meas_angle_min: f32,
    /// `meas_angle_max` after the tick.
    pub meas_angle_max: f32,
    /// `meas_rate_min` after the tick.
    pub meas_rate_min: f32,
    /// `meas_rate_max` after the tick.
    pub meas_rate_max: f32,
    /// `step_timeout_ms` after the 63.21% stretch.
    pub step_timeout_ms: u32,
    /// `Step::UPDATE_GAINS` when the angle, bounce, or timeout fires.
    pub done: bool,
}

/// Upstream `AC_AutoTune_Multi::twitching_test_angle`.
#[must_use]
pub fn twitching_test_angle(
    angle: f32,
    rate: f32,
    angle_target_max: f32,
    meas_angle_min: f32,
    meas_angle_max: f32,
    meas_rate_min: f32,
    meas_rate_max: f32,
    now_ms: u32,
    step_start_time_ms: u32,
    step_timeout_ms: u32,
    aggressiveness: f32,
) -> TwitchingAngle {
    let mut meas_angle_min = meas_angle_min;
    let mut meas_angle_max = meas_angle_max;
    let mut meas_rate_min = meas_rate_min;
    let mut meas_rate_max = meas_rate_max;
    let mut step_timeout_ms = step_timeout_ms;
    let mut done = false;

    if angle > meas_angle_max {
        meas_angle_max = angle;
        meas_angle_min = angle;
    }
    if angle < meas_angle_min && meas_angle_max > angle_target_max * 0.25 {
        meas_angle_min = angle;
    }
    if rate > meas_rate_max {
        meas_rate_max = rate;
        meas_rate_min = rate;
    }
    if rate < meas_rate_min {
        meas_rate_min = rate;
    }

    let elapsed = now_ms.wrapping_sub(step_start_time_ms);
    if meas_angle_max < angle_target_max * 0.6321 {
        step_timeout_ms = elapsed
            .wrapping_mul(3)
            .min(AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }
    if meas_angle_max > angle_target_max {
        done = true;
    }
    if meas_angle_max - meas_angle_min > meas_angle_max * aggressiveness {
        done = true;
    }
    if elapsed >= step_timeout_ms {
        done = true;
    }

    TwitchingAngle {
        meas_angle_min,
        meas_angle_max,
        meas_rate_min,
        meas_rate_max,
        step_timeout_ms,
        done,
    }
}

/// What `twitching_measure_acceleration` writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwitchingAccel {
    /// `test_accel_max_cdss` after the tick.
    pub accel_average: f32,
    /// `accel_measure_rate_max` after the tick.
    pub rate_max: f32,
}

/// Upstream `AC_AutoTune_Multi::twitching_measure_acceleration`.
///
/// When `now_ms == step_start_time_ms` the C++ divide is by zero and
/// the leftover keeps that Inf write.
#[must_use]
pub fn twitching_measure_acceleration(
    accel_average: f32,
    rate: f32,
    rate_max: f32,
    now_ms: u32,
    step_start_time_ms: u32,
) -> TwitchingAccel {
    if rate_max < rate {
        let rate_max = rate;
        let elapsed = now_ms.wrapping_sub(step_start_time_ms) as f32;
        TwitchingAccel {
            accel_average: (1000.0 * rate_max) / elapsed,
            rate_max,
        }
    } else {
        TwitchingAccel {
            accel_average,
            rate_max,
        }
    }
}

/// What Multi `test_run` reads. The rotation-rate LPF stays leftover
/// — this takes the already-filtered `rotation_rate` the same way
/// SystemID takes an already-computed chirp sample.
#[derive(Debug, Clone, Copy)]
pub struct AutoTuneTwitchView {
    /// Current axis.
    pub axis: AxisType,
    /// Current Multi tune type.
    pub tune_type: TuneType,
    /// `positive_direction` before this tick.
    pub positive_direction: bool,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `step_start_time_ms` before this tick.
    pub step_start_time_ms: u32,
    /// `step_timeout_ms` before this tick.
    pub step_timeout_ms: u32,
    /// `aggressiveness` (already constrained 0.05..0.2 on backup).
    pub aggressiveness: f32,
    /// `target_rate` from `test_init`, centidegrees/s.
    pub target_rate: f32,
    /// `target_angle` from `test_init`, centidegrees.
    pub target_angle: f32,
    /// `angle_abort` from `test_init`.
    pub angle_abort: f32,
    /// `start_angle` at test start, centidegrees.
    pub start_angle: f32,
    /// `start_rate` at test start, centidegrees/s.
    pub start_rate: f32,
    /// Already-filtered `rotation_rate`.
    pub rotation_rate: f32,
    /// `ahrs_view->get_roll_rad()`.
    pub roll_rad: f32,
    /// `ahrs_view->get_pitch_rad()`.
    pub pitch_rad: f32,
    /// `ahrs_view->get_yaw_rad()`.
    pub yaw_rad: f32,
    /// `test_rate_min` before this tick.
    pub test_rate_min: f32,
    /// `test_rate_max` before this tick.
    pub test_rate_max: f32,
    /// `test_angle_min` before this tick.
    pub test_angle_min: f32,
    /// `test_angle_max` before this tick.
    pub test_angle_max: f32,
    /// `accel_measure_rate_max` before this tick.
    pub accel_measure_rate_max: f32,
    /// `test_accel_max_cdss` before this tick.
    pub test_accel_max_cdss: f32,
    /// `step_scaler` before this tick.
    pub step_scaler: f32,
    /// `angle_step_commanded` before this tick.
    pub angle_step_commanded: bool,
}

impl AutoTuneTwitchView {
    /// Rate-D-up roll twitch, just started, still well under target.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: AxisType::Roll,
            tune_type: TuneType::RateDUp,
            positive_direction: true,
            now_ms: 10_000,
            step_start_time_ms: 9_800,
            step_timeout_ms: AUTOTUNE_TESTING_STEP_TIMEOUT_MS,
            aggressiveness: AUTOTUNE_AGGR_DEFAULT,
            target_rate: AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
            target_angle: 2_000.0,
            angle_abort: 2_000.0,
            start_angle: 0.0,
            start_rate: 0.0,
            rotation_rate: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            test_rate_min: 0.0,
            test_rate_max: 0.0,
            test_angle_min: 0.0,
            test_angle_max: 0.0,
            accel_measure_rate_max: 0.0,
            test_accel_max_cdss: 0.0,
            step_scaler: 1.0,
            angle_step_commanded: false,
        }
    }
}

/// Leftover of one Multi `test_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTuneTwitch {
    /// `lean_angle` after the sensor capture.
    pub lean_angle: f32,
    /// Step write `test_run` decided.
    pub tick: TwitchTick,
    /// `step_timeout_ms` after the twitching helper.
    pub step_timeout_ms: u32,
    /// `step_scaler` after an abort shrink.
    pub step_scaler: f32,
    /// `test_rate_min` after the tick.
    pub test_rate_min: f32,
    /// `test_rate_max` after the tick.
    pub test_rate_max: f32,
    /// `test_angle_min` after the tick.
    pub test_angle_min: f32,
    /// `test_angle_max` after the tick.
    pub test_angle_max: f32,
    /// `accel_measure_rate_max` after the tick.
    pub accel_measure_rate_max: f32,
    /// `test_accel_max_cdss` after the tick.
    pub test_accel_max_cdss: f32,
    /// `input_angle_step_bf_roll_pitch_yaw_rad` this tick.
    pub input_angle_step: bool,
    /// `input_rate_step_bf_roll_pitch_yaw_rads` this tick.
    pub input_rate_step: bool,
    /// `input_rate_bf_roll_pitch_yaw_rads(0,0,0)` hold after the angle step.
    pub input_rate_hold: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — Heli type on Multi.
    pub flow_of_control: bool,
    /// `mode` was written to [`TuneMode::Failed`].
    pub failed: bool,
    /// `LogEvent::AUTOTUNE_REACHED_LIMIT`.
    pub reached_limit: bool,
    /// `angle_step_commanded` after the tick.
    pub angle_step_commanded: bool,
}

/// Upstream `AC_AutoTune_Multi::test_run`.
///
/// Attitude command is catalogued (angle step vs rate step vs hold).
/// Lean angle is captured from the AHRS leftover. The rotation-rate
/// LPF stays leftover — `rotation_rate` is already filtered. The
/// twitching helpers then write [`TwitchTick`].
#[must_use]
pub fn autotune_test_run(view: &AutoTuneTwitchView) -> AutoTuneTwitch {
    let dir_sign = direction_sign(view.positive_direction);
    let mut input_angle_step = false;
    let mut input_rate_step = false;
    let mut input_rate_hold = false;
    let mut angle_step_commanded = view.angle_step_commanded;

    if twitch_is_angle_p(view.tune_type) {
        if !angle_step_commanded {
            angle_step_commanded = true;
            input_angle_step = true;
        } else {
            input_rate_hold = true;
        }
    } else if !twitch_is_heli_only(view.tune_type) && view.tune_type != TuneType::TuneComplete {
        input_rate_step = true;
    }

    let lean_angle = twitch_lean_angle_cd(
        view.axis,
        dir_sign,
        view.roll_rad,
        view.pitch_rad,
        view.yaw_rad,
        view.start_angle,
    );

    let mut out = AutoTuneTwitch {
        lean_angle,
        tick: TwitchTick::Running,
        step_timeout_ms: view.step_timeout_ms,
        step_scaler: view.step_scaler,
        test_rate_min: view.test_rate_min,
        test_rate_max: view.test_rate_max,
        test_angle_min: view.test_angle_min,
        test_angle_max: view.test_angle_max,
        accel_measure_rate_max: view.accel_measure_rate_max,
        test_accel_max_cdss: view.test_accel_max_cdss,
        input_angle_step,
        input_rate_step,
        input_rate_hold,
        flow_of_control: false,
        failed: false,
        reached_limit: false,
        angle_step_commanded,
    };

    match view.tune_type {
        TuneType::RateDUp | TuneType::RateDDown => {
            apply_rate_twitch(view, &mut out, view.target_rate);
        }
        TuneType::RatePUp => {
            apply_rate_twitch(
                view,
                &mut out,
                view.target_rate * (1.0 + 0.5 * view.aggressiveness),
            );
        }
        TuneType::AnglePDown | TuneType::AnglePUp => {
            apply_angle_twitch(view, &mut out, dir_sign);
        }
        TuneType::RateFfUp | TuneType::MaxGains | TuneType::TuneCheck => {
            out.flow_of_control = true;
        }
        TuneType::TuneComplete => {}
    }

    out
}

fn apply_rate_twitch(view: &AutoTuneTwitchView, out: &mut AutoTuneTwitch, rate_target: f32) {
    let rate = twitching_test_rate(
        out.lean_angle,
        view.rotation_rate,
        rate_target,
        view.test_rate_min,
        view.test_rate_max,
        view.test_angle_min,
        view.now_ms,
        view.step_start_time_ms,
        view.step_timeout_ms,
        view.aggressiveness,
    );
    out.test_rate_min = rate.meas_rate_min;
    out.test_rate_max = rate.meas_rate_max;
    out.test_angle_min = rate.meas_angle_min;
    out.step_timeout_ms = rate.step_timeout_ms;
    if rate.done {
        out.tick = TwitchTick::Done;
    }

    let accel = twitching_measure_acceleration(
        view.test_accel_max_cdss,
        view.rotation_rate,
        view.accel_measure_rate_max,
        view.now_ms,
        view.step_start_time_ms,
    );
    out.test_accel_max_cdss = accel.accel_average;
    out.accel_measure_rate_max = accel.rate_max;

    let abort = twitching_abort_rate(
        out.lean_angle,
        view.rotation_rate,
        view.angle_abort,
        out.test_rate_min,
        out.test_angle_min,
        view.step_scaler,
    );
    out.step_scaler = abort.step_scaler;
    out.failed = abort.failed;
    out.reached_limit = abort.reached_limit;
    if let Some(tick) = abort.tick {
        out.tick = tick;
    }
}

fn apply_angle_twitch(view: &AutoTuneTwitchView, out: &mut AutoTuneTwitch, dir_sign: f32) {
    let angle = twitching_test_angle(
        out.lean_angle,
        view.rotation_rate,
        view.target_angle * (1.0 + 0.5 * view.aggressiveness),
        view.test_angle_min,
        view.test_angle_max,
        view.test_rate_min,
        view.test_rate_max,
        view.now_ms,
        view.step_start_time_ms,
        view.step_timeout_ms,
        view.aggressiveness,
    );
    out.test_angle_min = angle.meas_angle_min;
    out.test_angle_max = angle.meas_angle_max;
    out.test_rate_min = angle.meas_rate_min;
    out.test_rate_max = angle.meas_rate_max;
    out.step_timeout_ms = angle.step_timeout_ms;
    if angle.done {
        out.tick = TwitchTick::Done;
    }

    let accel = twitching_measure_acceleration(
        view.test_accel_max_cdss,
        view.rotation_rate - dir_sign * view.start_rate,
        view.accel_measure_rate_max,
        view.now_ms,
        view.step_start_time_ms,
    );
    out.test_accel_max_cdss = accel.accel_average;
    out.accel_measure_rate_max = accel.rate_max;
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
    /// Multi `get_testing_step_timeout_ms()`.
    pub testing_step_timeout_ms: u32,
    /// Current Multi [`TuneType`].
    pub tune_type: TuneType,
    /// `target_rate` from `test_init`, centidegrees/s.
    pub target_rate: f32,
    /// `target_angle` from `test_init`, centidegrees.
    pub target_angle: f32,
    /// `aggressiveness` after backup constrain.
    pub aggressiveness: f32,
    /// `angle_abort` from `test_init`.
    pub angle_abort: f32,
    /// `start_angle` at test start, centidegrees.
    pub start_angle: f32,
    /// `start_rate` at test start, centidegrees/s.
    pub start_rate: f32,
    /// Already-filtered `rotation_rate`.
    pub rotation_rate: f32,
    /// `test_rate_min` before this tick.
    pub test_rate_min: f32,
    /// `test_rate_max` before this tick.
    pub test_rate_max: f32,
    /// `test_angle_min` before this tick.
    pub test_angle_min: f32,
    /// `test_angle_max` before this tick.
    pub test_angle_max: f32,
    /// `accel_measure_rate_max` before this tick.
    pub accel_measure_rate_max: f32,
    /// `test_accel_max_cdss` before this tick.
    pub test_accel_max_cdss: f32,
    /// `step_scaler` before this tick.
    pub step_scaler: f32,
    /// `angle_step_commanded` before this tick.
    pub angle_step_commanded: bool,
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
    /// `attitude_control->lean_angle_deg()`.
    pub lean_angle_deg: f32,
    /// Multi `angle_lim_neg_rpy_cd()`.
    pub angle_lim_neg_rpy_cd: f32,
    /// Multi `angle_lim_max_rp_cd()`.
    pub angle_lim_max_rp_cd: f32,
    /// `tune_*_rp` / `tune_*_sp` for the active UPDATE_GAINS axis.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` for the active UPDATE_GAINS axis.
    pub tune_d: f32,
    /// `success_counter` before this tick. Upstream is `int8_t`.
    pub success_counter: i8,
    /// `ignore_next` before this tick.
    pub ignore_next: bool,
    /// `min_d` param.
    pub min_d: f32,
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
            tune_type: TuneType::RateDUp,
            target_rate: AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
            target_angle: 2_000.0,
            aggressiveness: AUTOTUNE_AGGR_DEFAULT,
            angle_abort: 2_000.0,
            start_angle: 0.0,
            start_rate: 0.0,
            rotation_rate: 0.0,
            test_rate_min: 0.0,
            test_rate_max: 0.0,
            test_angle_min: 0.0,
            test_angle_max: 0.0,
            accel_measure_rate_max: 0.0,
            test_accel_max_cdss: 0.0,
            step_scaler: 1.0,
            angle_step_commanded: false,
            positive_direction: true,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            yaw_rate_ef_target_rads: 0.0,
            slew_yaw_max_rads: 1.0,
            lean_angle_deg: 0.0,
            angle_lim_neg_rpy_cd: 900.0,
            angle_lim_max_rp_cd: 3750.0,
            tune_p: 0.15,
            tune_d: 0.004,
            success_counter: 0,
            ignore_next: false,
            min_d: 0.0005,
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
    /// `lean_angle` after Multi `test_run`.
    pub lean_angle: f32,
    /// `input_angle_step_bf_roll_pitch_yaw_rad` this tick.
    pub input_angle_step: bool,
    /// `input_rate_step_bf_roll_pitch_yaw_rads` this tick.
    pub input_rate_step: bool,
    /// `input_rate_bf_roll_pitch_yaw_rads(0,0,0)` after the angle step.
    pub input_rate_hold: bool,
    /// Heli tune type on Multi `test_run`.
    pub twitch_flow_of_control: bool,
    /// Twitch abort wrote [`TuneMode::Failed`].
    pub twitch_failed: bool,
    /// `LogEvent::AUTOTUNE_REACHED_LIMIT`.
    pub twitch_reached_limit: bool,
    /// `step_scaler` after a twitch abort shrink.
    pub step_scaler: f32,
    /// `Step::UPDATE_GAINS` body ran.
    pub update_gains: bool,
    /// `tune_*_rp` / `tune_*_sp` after UPDATE_GAINS leftover.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` after UPDATE_GAINS leftover.
    pub tune_d: f32,
    /// `success_counter` after UPDATE_GAINS leftover.
    pub success_counter: i8,
    /// `ignore_next` after UPDATE_GAINS leftover.
    pub ignore_next: bool,
    /// `LogEvent::AUTOTUNE_REACHED_LIMIT` this tick.
    pub update_gains_reached_limit: bool,
    /// UPDATE_GAINS leftover wrote [`TuneMode::Failed`].
    pub update_gains_failed: bool,
    /// Heli type on Multi UPDATE_GAINS leftover.
    pub update_gains_flow_of_control: bool,
    /// `success_counter >= AUTOTUNE_SUCCESS_COUNT`. Sequencing stays leftover.
    pub update_gains_complete: bool,
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
        lean_angle: 0.0,
        input_angle_step: false,
        input_rate_step: false,
        input_rate_hold: false,
        twitch_flow_of_control: false,
        twitch_failed: false,
        twitch_reached_limit: false,
        step_scaler: view.step_scaler,
        update_gains: false,
        tune_p: view.tune_p,
        tune_d: view.tune_d,
        success_counter: view.success_counter,
        ignore_next: view.ignore_next,
        update_gains_reached_limit: false,
        update_gains_failed: false,
        update_gains_flow_of_control: false,
        update_gains_complete: false,
        positive_direction: view.positive_direction,
        desired_yaw_rad: view.desired_yaw_rad,
        step_start_time_ms: view.step_start_time_ms,
        level_start_time_ms: view.level_start_time_ms,
        step_timeout_ms: view.step_timeout_ms,
    }
}

fn twitch_view_from_run(view: &AutoTuneRunView) -> AutoTuneTwitchView {
    AutoTuneTwitchView {
        axis: view.axis,
        tune_type: view.tune_type,
        positive_direction: view.positive_direction,
        now_ms: view.now_ms,
        step_start_time_ms: view.step_start_time_ms,
        step_timeout_ms: view.step_timeout_ms,
        aggressiveness: view.aggressiveness,
        target_rate: view.target_rate,
        target_angle: view.target_angle,
        angle_abort: view.angle_abort,
        start_angle: view.start_angle,
        start_rate: view.start_rate,
        rotation_rate: view.rotation_rate,
        roll_rad: view.roll_rad,
        pitch_rad: view.pitch_rad,
        yaw_rad: view.yaw_rad,
        test_rate_min: view.test_rate_min,
        test_rate_max: view.test_rate_max,
        test_angle_min: view.test_angle_min,
        test_angle_max: view.test_angle_max,
        accel_measure_rate_max: view.accel_measure_rate_max,
        test_accel_max_cdss: view.test_accel_max_cdss,
        step_scaler: view.step_scaler,
        angle_step_commanded: view.angle_step_commanded,
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
            let twitch = autotune_test_run(&twitch_view_from_run(view));
            out.lean_angle = twitch.lean_angle;
            out.input_angle_step = twitch.input_angle_step;
            out.input_rate_step = twitch.input_rate_step;
            out.input_rate_hold = twitch.input_rate_hold;
            out.twitch_flow_of_control = twitch.flow_of_control;
            out.twitch_failed = twitch.failed;
            out.twitch_reached_limit = twitch.reached_limit;
            out.step_scaler = twitch.step_scaler;
            out.step_timeout_ms = twitch.step_timeout_ms;
            if twitch.failed {
                out.mode = TuneMode::Failed;
            }
            out.step = match twitch.tick {
                TwitchTick::Running => Step::ExecutingTest,
                TwitchTick::Done => Step::UpdateGains,
                TwitchTick::Aborted => Step::Abort,
            };
            if twitch.lean_angle <= -view.angle_lim_neg_rpy_cd
                || view.lean_angle_deg * 100.0 > view.angle_lim_max_rp_cd
            {
                out.step = Step::Abort;
            }
            if matches!(view.axis, AxisType::Yaw | AxisType::YawD) {
                out.desired_yaw_rad = view.yaw_rad;
            }
        }
        Step::UpdateGains => {
            // next_tune_type / next-axis / backoff stay leftover.
            out.update_gains = true;
            let gains = crate::autotune_update_gains::autotune_update_gains(
                &crate::autotune_update_gains::UpdateGainsView::from_run(view),
            );
            out.tune_p = gains.tune_p;
            out.tune_d = gains.tune_d;
            out.success_counter = gains.success_counter;
            out.ignore_next = gains.ignore_next;
            out.update_gains_reached_limit = gains.reached_limit;
            out.update_gains_failed = gains.failed;
            out.update_gains_flow_of_control = gains.flow_of_control;
            out.update_gains_complete = gains.tune_type_complete;
            if gains.failed {
                out.mode = TuneMode::Failed;
            }
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
/// UPDATE_GAINS runs the Multi `updating_*` leftover. `next_tune_type`
/// / next-axis / backoff stay leftover. Poshold lean math
/// (`get_poshold_attitude_rad` 10° / 20 m) is also leftover — this
/// catalogs the call and the `have_position` latch.
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
