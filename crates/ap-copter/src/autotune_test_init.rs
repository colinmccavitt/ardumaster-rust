//! Multi `test_init` leftover, upstream `AC_AutoTune_Multi::test_init`.
//!
//! Tracked as **COP-027**. The Copter wrapper already flags when
//! WAITING_FOR_LEVEL has been held long enough to call `test_init()`.
//! This leftover is the Multi math that seats `angle_abort`,
//! `target_rate`, `target_angle`, the rotation-rate LPF cutoff / reset,
//! and the measurement accumulators before `test_run` twitches.
//!
//! `max_rate_step_bf_*` / `max_angle_step_bf_*` stay attitude-control
//! reads. The LPF object itself stays leftover — this tick records
//! `set_cutoff_frequency` and `reset`. GCS announcements stay for a
//! later slice. PosHold lean is [`crate::autotune_poshold`]. Heli
//! `test_init` is out of scope.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::{
    twitch_is_angle_p, AxisType, TuneType, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS,
    AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
    AUTOTUNE_TARGET_RATE_YAW_CDS, AUTOTUNE_Y_FILT_FREQ,
};
use ap_math::scalar::{constrain_value, degrees};

/// `AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE`.
pub const AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE: f32 = 1.0 / 2.0;

/// `AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE`.
pub const AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE: f32 = 1.0;

/// `AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE`.
pub const AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE: f32 = 1.0 / 3.0;

/// `AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE`.
pub const AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE: f32 = 1.0 / 6.0;

/// Yaw `max_*_step_bf_yaw() * 0.75` before `degrees`.
pub const AUTOTUNE_YAW_STEP_SCALE: f32 = 0.75;

/// Rate-PID `filt_D_hz() * 2.0` becomes the rotation-rate LPF cutoff.
pub const AUTOTUNE_RATE_FILT_D_SCALE: f32 = 2.0;

/// Typical Copter `ANGLE_MAX` leftover, centidegrees.
pub const AUTOTUNE_LEAN_ANGLE_MAX_CD_DEFAULT: f32 = 3_000.0;

/// Typical `max_rate_step_bf_*` leftover, rad/s (`π` → 18000 cd/s).
pub const AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT: f32 = core::f32::consts::PI;

/// Typical `max_angle_step_bf_*` leftover, rad (~20 deg → 2000 cd).
pub const AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT: f32 = 0.349_065_85;

/// Typical rate-PID `filt_D_hz()`.
pub const AUTOTUNE_FILT_D_HZ_DEFAULT: f32 = 20.0;

/// `target_angle_max_rp_cd()` — `lean_angle_max_cd * 1/2`.
#[must_use]
pub fn target_angle_max_rp_cd(lean_angle_max_cd: f32) -> f32 {
    lean_angle_max_cd * AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE
}

/// `target_angle_max_y_cd()` — `lean_angle_max_cd * 1`.
#[must_use]
pub fn target_angle_max_y_cd(lean_angle_max_cd: f32) -> f32 {
    lean_angle_max_cd * AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE
}

/// `target_angle_min_rp_cd()` — `lean_angle_max_cd * 1/3`.
#[must_use]
pub fn target_angle_min_rp_cd(lean_angle_max_cd: f32) -> f32 {
    lean_angle_max_cd * AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE
}

/// `target_angle_min_y_cd()` — `lean_angle_max_cd * 1/6`.
#[must_use]
pub fn target_angle_min_y_cd(lean_angle_max_cd: f32) -> f32 {
    lean_angle_max_cd * AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE
}

/// Attitude-control reads `test_init` consumes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TestInitView {
    /// Current Multi axis.
    pub axis: AxisType,
    /// Current Multi [`TuneType`].
    pub tune_type: TuneType,
    /// `step_scaler` after any abort shrink.
    pub step_scaler: f32,
    /// `start_rate` captured when WAITING → EXECUTING, centidegrees/s.
    pub start_rate: f32,
    /// `attitude_control->lean_angle_max_cd()`.
    pub lean_angle_max_cd: f32,
    /// `max_rate_step_bf_roll()`, rad/s.
    pub max_rate_step_roll_rad: f32,
    /// `max_rate_step_bf_pitch()`, rad/s.
    pub max_rate_step_pitch_rad: f32,
    /// `max_rate_step_bf_yaw()`, rad/s.
    pub max_rate_step_yaw_rad: f32,
    /// `max_angle_step_bf_roll()`, rad.
    pub max_angle_step_roll_rad: f32,
    /// `max_angle_step_bf_pitch()`, rad.
    pub max_angle_step_pitch_rad: f32,
    /// `max_angle_step_bf_yaw()`, rad.
    pub max_angle_step_yaw_rad: f32,
    /// Rate-roll `filt_D_hz()`.
    pub filt_d_hz_roll: f32,
    /// Rate-pitch `filt_D_hz()`.
    pub filt_d_hz_pitch: f32,
    /// Rate-yaw `filt_D_hz()`.
    pub filt_d_hz_yaw: f32,
}

impl TestInitView {
    /// Mid-range leftover view: roll, RATE_D_UP, `step_scaler` 1.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: AxisType::Roll,
            tune_type: TuneType::RateDUp,
            step_scaler: 1.0,
            start_rate: 0.0,
            lean_angle_max_cd: AUTOTUNE_LEAN_ANGLE_MAX_CD_DEFAULT,
            max_rate_step_roll_rad: AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT,
            max_rate_step_pitch_rad: AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT,
            max_rate_step_yaw_rad: AUTOTUNE_MAX_RATE_STEP_RAD_DEFAULT,
            max_angle_step_roll_rad: AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT,
            max_angle_step_pitch_rad: AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT,
            max_angle_step_yaw_rad: AUTOTUNE_MAX_ANGLE_STEP_RAD_DEFAULT,
            filt_d_hz_roll: AUTOTUNE_FILT_D_HZ_DEFAULT,
            filt_d_hz_pitch: AUTOTUNE_FILT_D_HZ_DEFAULT,
            filt_d_hz_yaw: AUTOTUNE_FILT_D_HZ_DEFAULT,
        }
    }
}

/// Multi `test_init()` writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TestInit {
    /// `angle_abort`.
    pub angle_abort: f32,
    /// `target_rate`, centidegrees/s.
    pub target_rate: f32,
    /// `target_angle`, centidegrees.
    pub target_angle: f32,
    /// `rotation_rate_filt.set_cutoff_frequency`.
    pub rotation_rate_filt_hz: f32,
    /// `rotation_rate_filt.reset` argument.
    pub rotation_rate_filt_reset: f32,
    /// `angle_step_commanded` — always cleared.
    pub angle_step_commanded: bool,
    /// `test_rate_max` — always cleared.
    pub test_rate_max: f32,
    /// `test_rate_min` — always cleared.
    pub test_rate_min: f32,
    /// `test_angle_max` — always cleared.
    pub test_angle_max: f32,
    /// `test_angle_min` — always cleared.
    pub test_angle_min: f32,
    /// `accel_measure_rate_max` — always cleared.
    pub accel_measure_rate_max: f32,
}

fn step_to_cd(step_rad: f32) -> f32 {
    degrees(step_rad) * 100.0
}

/// `AC_AutoTune_Multi::test_init`.
#[must_use]
pub fn test_init(view: &TestInitView) -> TestInit {
    let (angle_abort, target_rate, target_angle, rotation_rate_filt_hz) = match view.axis {
        AxisType::Roll => {
            let target_max_rate = AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS
                .max(view.step_scaler * AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            (
                target_angle_max_rp_cd(view.lean_angle_max_cd),
                constrain_value(
                    step_to_cd(view.max_rate_step_roll_rad),
                    AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS,
                    target_max_rate,
                ),
                constrain_value(
                    step_to_cd(view.max_angle_step_roll_rad),
                    target_angle_min_rp_cd(view.lean_angle_max_cd),
                    target_angle_max_rp_cd(view.lean_angle_max_cd),
                ),
                view.filt_d_hz_roll * AUTOTUNE_RATE_FILT_D_SCALE,
            )
        }
        AxisType::Pitch => {
            let target_max_rate = AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS
                .max(view.step_scaler * AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            (
                target_angle_max_rp_cd(view.lean_angle_max_cd),
                constrain_value(
                    step_to_cd(view.max_rate_step_pitch_rad),
                    AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS,
                    target_max_rate,
                ),
                constrain_value(
                    step_to_cd(view.max_angle_step_pitch_rad),
                    target_angle_min_rp_cd(view.lean_angle_max_cd),
                    target_angle_max_rp_cd(view.lean_angle_max_cd),
                ),
                view.filt_d_hz_pitch * AUTOTUNE_RATE_FILT_D_SCALE,
            )
        }
        AxisType::Yaw | AxisType::YawD => {
            let target_max_rate = AUTOTUNE_TARGET_MIN_RATE_YAW_CDS
                .max(view.step_scaler * AUTOTUNE_TARGET_RATE_YAW_CDS);
            let filt_hz = if view.axis == AxisType::YawD {
                view.filt_d_hz_yaw * AUTOTUNE_RATE_FILT_D_SCALE
            } else {
                AUTOTUNE_Y_FILT_FREQ
            };
            (
                target_angle_max_y_cd(view.lean_angle_max_cd),
                constrain_value(
                    step_to_cd(view.max_rate_step_yaw_rad * AUTOTUNE_YAW_STEP_SCALE),
                    AUTOTUNE_TARGET_MIN_RATE_YAW_CDS,
                    target_max_rate,
                ),
                constrain_value(
                    step_to_cd(view.max_angle_step_yaw_rad * AUTOTUNE_YAW_STEP_SCALE),
                    target_angle_min_y_cd(view.lean_angle_max_cd),
                    target_angle_max_y_cd(view.lean_angle_max_cd),
                ),
                filt_hz,
            )
        }
    };

    let rotation_rate_filt_reset = if twitch_is_angle_p(view.tune_type) {
        view.start_rate
    } else {
        0.0
    };

    TestInit {
        angle_abort,
        target_rate,
        target_angle,
        rotation_rate_filt_hz,
        rotation_rate_filt_reset,
        angle_step_commanded: false,
        test_rate_max: 0.0,
        test_rate_min: 0.0,
        test_angle_max: 0.0,
        test_angle_min: 0.0,
        accel_measure_rate_max: 0.0,
    }
}
