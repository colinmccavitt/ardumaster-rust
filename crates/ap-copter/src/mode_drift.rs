//! `ModeDrift` leftover, upstream `ArduCopter/mode_drift.cpp`.
//!
//! Tracked as **COP-015**. Drift is the other position-mode leftover
//! alongside [`crate::mode_poshold`]: it holds a body-frame sideslip
//! loop and a pitch brake, not AC_Loiter. Horizontal hold is not seated
//! here.
//!
//! # `init` is a no-op that always succeeds
//!
//! `ModeDrift::init` returns true and does not read `ignore_checks`. There
//! is no controller to seat.
//!
//! # Run converts the stick, then overwrites roll from sideslip
//!
//! The pilot lean conversion uses the attitude lean-angle max (the same
//! pair [`crate::mode_poshold`] uses, not [`ap_wpnav::Loiter::get_angle_max_rad`]).
//! Yaw rate is scheduled from the *pilot* roll stick and the forward
//! speed, then roll is replaced by a filtered yaw-stick / body-right
//! velocity error. Releasing pitch ramps `braker` up to
//! [`DRIFT_SPEEDGAIN_RAD`] and commands a stopping pitch.
//!
//! # Throttle is assisted, not raw
//!
//! The spool switch resets yaw and rate-I the same way Stabilize does,
//! but it does not zero throttle — `set_throttle_out` always takes
//! [`drift_throttle_assist`]. Assist is a mid-stick band that adds a
//! D-axis velocity term, capped at [`DRIFT_THR_ASSIST_MAX`]. Attitude is
//! `input_euler_angle_roll_pitch_euler_rate_yaw_rad` with angle boost on.

use crate::mode_stabilize::{manual_throttle_desired_spool, RateIReset};
use crate::pilot_input::pilot_desired_throttle;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{cd_to_rad, constrain_value, is_zero, radians};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `Mode::Number::DRIFT`.
pub const MODE_NUMBER_DRIFT: u8 = 11;

/// Sideslip gain, rad/(m/s). Upstream `DRIFT_SPEEDGAIN_RAD`.
pub const DRIFT_SPEEDGAIN_RAD: f32 = 0.139_626_34;

/// Body-velocity clamp, m/s. Upstream `DRIFT_SPEEDLIMIT_MS`.
pub const DRIFT_SPEEDLIMIT_MS: f32 = 5.60;

/// Forward-speed cap used by the yaw schedule, m/s. Upstream
/// `DRIFT_VEL_FORWARD_MIN_MS`.
pub const DRIFT_VEL_FORWARD_MIN_MS: f32 = 20.0;

/// Throttle-assist gain against D-axis velocity. Upstream
/// `DRIFT_THR_ASSIST_GAIN_MS`.
pub const DRIFT_THR_ASSIST_GAIN_MS: f32 = 0.18;

/// Maximum |assist| added to the pilot throttle. Upstream
/// `DRIFT_THR_ASSIST_MAX`.
pub const DRIFT_THR_ASSIST_MAX: f32 = 0.3;

/// Lower edge of the assist band. Upstream `DRIFT_THR_MIN`.
pub const DRIFT_THR_MIN: f32 = 0.213;

/// Upper edge of the assist band. Upstream `DRIFT_THR_MAX`.
pub const DRIFT_THR_MAX: f32 = 0.787;

/// Persisted Drift leftovers. Upstream's `static` `braker` and
/// `roll_input_rad`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drift {
    /// Pitch-brake gain ramped while the pitch stick is released.
    pub braker: f32,
    /// Low-pass of the yaw stick, rad.
    pub roll_input_rad: f32,
}

impl Drift {
    /// BSS-zeroed construction; [`drift_init`] records the leftover.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            braker: 0.0,
            roll_input_rad: 0.0,
        }
    }
}

impl Default for Drift {
    fn default() -> Self {
        Self::new()
    }
}

/// Leftover of one `ModeDrift::init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriftInit {
    /// Always true. `ignore_checks` is unread.
    pub ok: bool,
}

/// Pilot / vehicle view `ModeDrift::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct DriftRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`.
    pub attitude_lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// `channel_yaw->get_control_in()`, centidegrees.
    pub yaw_control_cd: f32,
    /// `g2.command_model_acro_y.get_rate()`, deg/s.
    pub acro_yaw_rate_degs: f32,
    /// `pos_control->get_vel_estimate_NED_ms()` north, m/s.
    pub vel_n_ms: f32,
    /// `pos_control->get_vel_estimate_NED_ms()` east, m/s.
    pub vel_e_ms: f32,
    /// `pos_control->get_vel_estimate_NED_ms()` down, m/s.
    pub vel_d_ms: f32,
    /// `ahrs.cos_yaw()`.
    pub cos_yaw: f32,
    /// `ahrs.sin_yaw()`.
    pub sin_yaw: f32,
    /// `channel_throttle->get_control_in()`.
    pub throttle_control: i16,
    /// Mid-stick PWM used by [`pilot_desired_throttle`].
    pub mid_stick: i16,
    /// Hover throttle used by [`pilot_desired_throttle`].
    pub throttle_hover: f32,
    /// `copter.ap.throttle_zero`.
    pub throttle_zero: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `motors->limit.throttle_lower`.
    pub throttle_lower_limited: bool,
}

impl DriftRunView {
    /// Airborne, motors unlimited, mid throttle.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            has_valid_input: true,
            attitude_lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_control_cd: 0.0,
            acro_yaw_rate_degs: 360.0,
            vel_n_ms: 0.0,
            vel_e_ms: 0.0,
            vel_d_ms: 0.0,
            cos_yaw: 1.0,
            sin_yaw: 0.0,
            throttle_control: 500,
            mid_stick: 500,
            throttle_hover: 0.5,
            throttle_zero: false,
            spool_state: SpoolState::ThrottleUnlimited,
            throttle_lower_limited: false,
        }
    }
}

/// Attitude / throttle leftover of one `ModeDrift::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftRun {
    /// Pilot roll before the sideslip overwrite.
    pub pilot_roll_rad: f32,
    /// Pilot pitch before the brake overwrite.
    pub pilot_pitch_rad: f32,
    /// Final roll sent to the attitude controller.
    pub target_roll_rad: f32,
    /// Final pitch sent to the attitude controller.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand, rad/s, from the *pilot* roll stick.
    pub target_yaw_rate_rads: f32,
    /// Body-right velocity after the speed clamp, m/s.
    pub vel_right_ms: f32,
    /// Body-forward velocity after the speed clamp, m/s.
    pub vel_forward_ms: f32,
    /// `braker` after this tick.
    pub braker: f32,
    /// Desired spool from the throttle-zero flag.
    pub desired_spool: DesiredSpoolState,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` on shut-down and ground-idle.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. ShutDown passes `false`; GroundIdle
    /// uses the default `true`.
    pub reset_yaw_rate: bool,
    /// `set_land_complete(false)` on the unlimited branch above the
    /// throttle-lower limit.
    pub clear_land_complete: bool,
    /// Assisted throttle passed to `set_throttle_out`. Never zeroed by
    /// the spool switch.
    pub throttle_out: f32,
    /// Always true: `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
    pub input_euler_angle_roll_pitch_euler_rate_yaw: bool,
    /// Always true: `set_throttle_out(..., true, g.throttle_filt)`.
    pub angle_boost: bool,
}

/// Upstream `ModeDrift::get_throttle_assist`.
///
/// `vel_d_ms` is D-axis velocity, positive down. Assist is only active
/// inside [`DRIFT_THR_MIN`]..[`DRIFT_THR_MAX`], strongest at mid-stick,
/// and never larger than [`DRIFT_THR_ASSIST_MAX`].
#[must_use]
pub fn drift_throttle_assist(vel_d_ms: f32, pilot_throttle_scaled: f32) -> f32 {
    let mut thr_assist = 0.0;
    if pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX {
        thr_assist = 1.2 - ((pilot_throttle_scaled - 0.5).abs() / 0.24);
        thr_assist = constrain_value(thr_assist, 0.0, 1.0) * DRIFT_THR_ASSIST_GAIN_MS * vel_d_ms;
        thr_assist = constrain_value(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    constrain_value(pilot_throttle_scaled + thr_assist, 0.0, 1.0)
}

/// Upstream `ModeDrift::init`. Always succeeds; `ignore_checks` is unread.
#[must_use]
pub fn drift_init(_ignore_checks: bool) -> DriftInit {
    DriftInit { ok: true }
}

/// Upstream `ModeDrift::run`.
///
/// Converts the pilot, schedules yaw from the pilot roll stick, replaces
/// roll with the sideslip leftover, and ramps a pitch brake when the
/// pitch stick is released. Throttle is always the assisted leftover —
/// the spool switch resets integrators but does not zero it.
#[must_use]
pub fn drift_run(drift: &mut Drift, view: &DriftRunView) -> DriftRun {
    let (pilot_roll_rad, pilot_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.attitude_lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let mut target_pitch_rad = pilot_pitch_rad;

    let mut vel_right_ms = view.vel_e_ms * view.cos_yaw - view.vel_n_ms * view.sin_yaw;
    let mut vel_forward_ms = view.vel_e_ms * view.sin_yaw + view.vel_n_ms * view.cos_yaw;

    let vel_forward_2_ms = vel_forward_ms.abs().min(DRIFT_VEL_FORWARD_MIN_MS);
    let yaw_rate_max_rads = radians(view.acro_yaw_rate_degs);
    let target_yaw_rate_rads =
        (pilot_roll_rad / radians(45.0)) * yaw_rate_max_rads * (1.0 - (vel_forward_2_ms / 50.0));

    vel_right_ms = constrain_value(vel_right_ms, -DRIFT_SPEEDLIMIT_MS, DRIFT_SPEEDLIMIT_MS);
    vel_forward_ms = constrain_value(vel_forward_ms, -DRIFT_SPEEDLIMIT_MS, DRIFT_SPEEDLIMIT_MS);

    let yaw_stick_rad = cd_to_rad(view.yaw_control_cd);
    drift.roll_input_rad = drift.roll_input_rad * 0.96 + yaw_stick_rad * 0.04;

    let roll_vel_error_ms = vel_right_ms - (drift.roll_input_rad / DRIFT_SPEEDGAIN_RAD);
    let mut target_roll_rad = roll_vel_error_ms * -DRIFT_SPEEDGAIN_RAD;
    target_roll_rad = constrain_value(target_roll_rad, -radians(45.0), radians(45.0));

    if is_zero(target_pitch_rad) {
        drift.braker += 0.03;
        drift.braker = drift.braker.min(DRIFT_SPEEDGAIN_RAD);
        target_pitch_rad = vel_forward_ms * drift.braker;
    } else {
        drift.braker = 0.0;
    }

    let desired_spool = manual_throttle_desired_spool(view.throttle_zero);
    let (reset_rate_i, reset_yaw, reset_yaw_rate, clear_land_complete) = match view.spool_state {
        SpoolState::ShutDown => (RateIReset::Hard, true, false, false),
        SpoolState::GroundIdle => (RateIReset::Smooth, true, true, false),
        SpoolState::ThrottleUnlimited => {
            (RateIReset::None, false, false, !view.throttle_lower_limited)
        }
        SpoolState::SpoolingUp | SpoolState::SpoolingDown => {
            (RateIReset::None, false, false, false)
        }
    };

    let pilot_throttle =
        pilot_desired_throttle(view.throttle_control, view.mid_stick, view.throttle_hover);
    let throttle_out = drift_throttle_assist(view.vel_d_ms, pilot_throttle);

    DriftRun {
        pilot_roll_rad,
        pilot_pitch_rad,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        vel_right_ms,
        vel_forward_ms,
        braker: drift.braker,
        desired_spool,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        clear_land_complete,
        throttle_out,
        input_euler_angle_roll_pitch_euler_rate_yaw: true,
        angle_boost: true,
    }
}
