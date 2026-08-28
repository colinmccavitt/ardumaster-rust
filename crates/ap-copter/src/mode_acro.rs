//! `ModeAcro`, upstream `ArduCopter/mode_acro.cpp`.
//!
//! Acro is Stabilize's rate-mode sibling. The spool leftover is the same
//! function — [`crate::mode_stabilize::manual_spool_leftover`] — because the
//! C++ switch is the same four-way split. What changes is the demand: body-
//! frame rates instead of euler angles, a different attitude reset, and
//! throttle without angle boost.
//!
//! # Trainer is the conversion, not `run()`
//!
//! `run()` only *calls* `get_pilot_desired_rates_rads`. The trainer-off path
//! is circular-limited sticks, expo, scaled by the ACRO command model. The
//! trainer-on path then pulls the aircraft back to level from the attitude-
//! controller target — LEVELING mixes that pull down as the sticks come off
//! centre, LIMITED adds it in full and extra-corrects past the lean limit.
//!
//! # `init` / `exit` own air-mode, not the rates
//!
//! `ACRO_OPTIONS` bit 0 latches `copter.air_mode` on the way in and clears
//! it on the way out, unless an AUX switch already set it (`air_mode_aux_changed`
//! raises `disable_air_mode_reset` so `exit` will not fight the switch).
//! `init` always returns true; `ignore_checks` is unread.
//!
//! # `RATE_LOOP_ONLY` is a different controller call, not a different rate
//!
//! The option does not change the numbers. It changes which input function
//! they go to, and scales the I-term to angle-P first so a Betaflight-style
//! tune still holds. The leftover therefore carries the flag rather than
//! two sets of rates.

use crate::mode_stabilize::{
    manual_spool_leftover, manual_throttle_desired_spool, RateIReset,
};
use crate::pilot_input::pilot_desired_throttle;
use ap_control::attitude_kinematics::euler_derivative_to_body;
use ap_math::control::{input_expo, sqrt_controller};
use ap_math::quaternion::Quaternion;
use ap_math::scalar::{constrain_value, is_positive, norm2, radians, wrap_pi};
use ap_math::vector3::Vector3f;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `AcroOptions::RATE_LOOP_ONLY`, upstream `ModeAcro::AcroOptions`.
pub const ACRO_OPTION_RATE_LOOP_ONLY: u8 = 1 << 1;

/// `AcroOptions::AIR_MODE`, the bit [`acro_init`] / [`acro_exit`] consume.
pub const ACRO_OPTION_AIR_MODE: u8 = 1 << 0;

/// Maximum trainer lean used by the balance term, radians.
///
/// Upstream `ACRO_LEVEL_MAX_ANGLE_RAD` = `radians(30)`.
pub const ACRO_LEVEL_MAX_ANGLE_RAD: f32 = 30.0 * (core::f32::consts::PI / 180.0);

/// Trainer overshoot used as the LIMITED sqrt-controller distance, radians.
///
/// Upstream `ACRO_LEVEL_MAX_OVERSHOOT_RAD` = `radians(10)`.
pub const ACRO_LEVEL_MAX_OVERSHOOT_RAD: f32 = 10.0 * (core::f32::consts::PI / 180.0);

/// `ModeAcro::Trainer` (`ACRO_TRAINER`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AcroTrainer {
    /// 0 — no levelling. The stick rates are the demand.
    Off = 0,
    /// 1 — pull toward level, mixed down as the sticks leave centre.
    Leveling = 1,
    /// 2 — always add the level rates, and extra-correct past lean-max.
    Limited = 2,
}

/// Upstream `AirMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AirMode {
    /// 0 — `AIRMODE_NONE`. Never written by Acro `init` / `exit`.
    None = 0,
    /// 1 — `AIRMODE_DISABLED`. What [`acro_exit`] writes when it is allowed to.
    Disabled = 1,
    /// 2 — `AIRMODE_ENABLED`. What [`acro_init`] writes when the option is set.
    Enabled = 2,
}

/// Air-mode leftover `ModeAcro::init` / `exit` / `air_mode_aux_changed` share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcroAirMode {
    /// `copter.air_mode`.
    pub air_mode: AirMode,
    /// `ModeAcro::disable_air_mode_reset`.
    pub disable_air_mode_reset: bool,
}

impl AcroAirMode {
    /// Vehicle default: no air-mode latch, `exit` is allowed to reset.
    #[must_use]
    pub const fn fresh() -> Self {
        Self {
            air_mode: AirMode::None,
            disable_air_mode_reset: false,
        }
    }
}

/// Pilot / vehicle view `ModeAcro::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct AcroRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `channel_throttle` control-in, 0..1000.
    pub throttle_control: i16,
    /// Throttle mid-stick, control-in units.
    pub mid_stick: i16,
    /// Hover throttle, 0..1. Prefer [`acro_throttle_hover`] when the
    /// vehicle's override is the source.
    pub throttle_hover: f32,
    /// `g2.command_model_acro_rp.get_rate()`, deg/s.
    pub rp_rate_degs: f32,
    /// `g2.command_model_acro_rp.get_expo()`.
    pub rp_expo: f32,
    /// `g2.command_model_acro_y.get_rate()`, deg/s.
    pub yaw_rate_degs: f32,
    /// `g2.command_model_acro_y.get_expo()`.
    pub yaw_expo: f32,
    /// `g2.acro_options & RATE_LOOP_ONLY`.
    pub rate_loop_only: bool,
    /// `copter.ap.throttle_zero`.
    pub throttle_zero: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `motors->limit.throttle_lower`.
    pub throttle_lower_limited: bool,
}

impl AcroRunView {
    /// Valid radio, mid throttle, motors unlimited, trainer off.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            throttle_control: 500,
            mid_stick: 500,
            throttle_hover: 0.5,
            rp_rate_degs: 360.0,
            rp_expo: 0.0,
            yaw_rate_degs: 202.5,
            yaw_expo: 0.0,
            rate_loop_only: false,
            throttle_zero: false,
            spool_state: SpoolState::ThrottleUnlimited,
            throttle_lower_limited: false,
        }
    }
}

/// Inputs `ModeAcro::get_pilot_desired_rates_rads` reads once trainer is on.
#[derive(Debug, Clone, Copy)]
pub struct AcroRatesView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `g2.command_model_acro_rp.get_rate()`, deg/s.
    pub rp_rate_degs: f32,
    /// `g2.command_model_acro_rp.get_expo()`.
    pub rp_expo: f32,
    /// `g2.command_model_acro_y.get_rate()`, deg/s.
    pub yaw_rate_degs: f32,
    /// `g2.command_model_acro_y.get_expo()`.
    pub yaw_expo: f32,
    /// `g.acro_trainer`.
    pub trainer: AcroTrainer,
    /// `attitude_control->get_att_target_euler_rad().x`.
    pub att_target_roll_rad: f32,
    /// `attitude_control->get_att_target_euler_rad().y`.
    pub att_target_pitch_rad: f32,
    /// `attitude_control->get_attitude_target_quat()`.
    pub att_target: Quaternion,
    /// `g.acro_balance_roll`.
    pub balance_roll: f32,
    /// `g.acro_balance_pitch`.
    pub balance_pitch: f32,
    /// `attitude_control->lean_angle_max_rad()`. LIMITED only.
    pub lean_angle_max_rad: f32,
    /// `attitude_control->get_accel_roll_max_radss()`. LIMITED only.
    pub accel_roll_max_radss: f32,
    /// `attitude_control->get_accel_pitch_max_radss()`. LIMITED only.
    pub accel_pitch_max_radss: f32,
    /// `G_Dt`. LIMITED only.
    pub dt: f32,
    /// `ahrs.cos_pitch()`. LEVELING only.
    pub cos_pitch: f32,
}

impl AcroRatesView {
    /// Trainer off, identity target, default command-model rates.
    #[must_use]
    pub fn trainer_off() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            rp_rate_degs: 360.0,
            rp_expo: 0.0,
            yaw_rate_degs: 202.5,
            yaw_expo: 0.0,
            trainer: AcroTrainer::Off,
            att_target_roll_rad: 0.0,
            att_target_pitch_rad: 0.0,
            att_target: Quaternion::identity(),
            balance_roll: 1.0,
            balance_pitch: 1.0,
            lean_angle_max_rad: ACRO_LEVEL_MAX_ANGLE_RAD,
            accel_roll_max_radss: 0.0,
            accel_pitch_max_radss: 0.0,
            dt: 0.0025,
            cos_pitch: 1.0,
        }
    }
}

/// Body-frame rate demand, rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcroRateDemand {
    /// Roll rate, body-frame, rad/s.
    pub roll_rads: f32,
    /// Pitch rate, body-frame, rad/s.
    pub pitch_rads: f32,
    /// Yaw rate, body-frame, rad/s.
    pub yaw_rads: f32,
}

/// Attitude / throttle leftover of one `ModeAcro::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcroRun {
    /// Where the motors should be heading.
    pub desired_spool: DesiredSpoolState,
    /// Body-frame rate demand.
    pub rates: AcroRateDemand,
    /// `set_throttle_out` throttle after the spool switch.
    pub throttle_out: f32,
    /// Always false: Acro does not boost throttle for lean.
    pub angle_boost: bool,
    /// `input_rate_bf_roll_pitch_yaw_2_rads` (true) vs `_rads` (false).
    pub rate_loop_only: bool,
    /// `scale_I_to_angle_P()` — only with [`Self::rate_loop_only`].
    pub scale_i_to_angle_p: bool,
    /// `reset_target_and_rate` on shut-down and ground-idle.
    pub reset_target_and_rate: bool,
    /// The `reset_rate` argument. Upstream passes `true` on shut-down and
    /// uses the default `true` on ground-idle.
    pub reset_target_rate: bool,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `set_land_complete(false)`.
    pub clear_land_complete: bool,
}

/// Circular-limit roll and pitch sticks, then expo-scale all three axes.
///
/// # The circular limit is on the stick, not the rate
///
/// Full roll and full pitch together is a corner of the stick's square,
/// √2 from centre. Left alone that would ask for √2 times the configured
/// rate on the diagonal. The scaling is applied to the *normalised* inputs
/// before expo, so a pilot at the limit still points the same direction and
/// never exceeds the command-model rate on any axis.
///
/// Yaw is not part of that circle. It is expo-scaled on its own.
fn acro_circular_limit_rp(roll_in_norm: f32, pitch_in_norm: f32) -> (f32, f32) {
    let mut roll_in_norm = roll_in_norm;
    let mut pitch_in_norm = pitch_in_norm;
    let norm_in_length = norm2(pitch_in_norm, roll_in_norm);
    if norm_in_length > 1.0 {
        let ratio = 1.0 / norm_in_length;
        roll_in_norm *= ratio;
        pitch_in_norm *= ratio;
    }
    (roll_in_norm, pitch_in_norm)
}

fn acro_request_from_sticks(
    roll_in_norm: f32,
    pitch_in_norm: f32,
    yaw_in_norm: f32,
    rp_rate_degs: f32,
    rp_expo: f32,
    yaw_rate_degs: f32,
    yaw_expo: f32,
) -> (AcroRateDemand, f32, f32) {
    let (roll_in_norm, pitch_in_norm) = acro_circular_limit_rp(roll_in_norm, pitch_in_norm);
    let demand = AcroRateDemand {
        roll_rads: radians(rp_rate_degs) * input_expo(roll_in_norm, rp_expo),
        pitch_rads: radians(rp_rate_degs) * input_expo(pitch_in_norm, rp_expo),
        yaw_rads: radians(yaw_rate_degs) * input_expo(yaw_in_norm, yaw_expo),
    };
    (demand, roll_in_norm, pitch_in_norm)
}

/// Trainer-off body-frame rates, upstream `ModeAcro::get_pilot_desired_rates_rads`
/// when `ACRO_TRAINER` is `OFF`.
#[must_use]
pub fn acro_pilot_desired_rates_rads(
    roll_in_norm: f32,
    pitch_in_norm: f32,
    yaw_in_norm: f32,
    rp_rate_degs: f32,
    rp_expo: f32,
    yaw_rate_degs: f32,
    yaw_expo: f32,
) -> AcroRateDemand {
    acro_request_from_sticks(
        roll_in_norm,
        pitch_in_norm,
        yaw_in_norm,
        rp_rate_degs,
        rp_expo,
        yaw_rate_degs,
        yaw_expo,
    )
    .0
}

/// Clip one LEVELING axis so adding the level rate cannot reverse the request
/// through inverted, upstream's `rate_delta_max_rads` constrain.
fn acro_leveling_axis(request: f32, level: f32) -> f32 {
    let rate_delta_max_rads = (request.abs() - level.abs()).abs();
    constrain_value(request + level, -rate_delta_max_rads, rate_delta_max_rads)
}

/// Trainer branch of `ModeAcro::get_pilot_desired_rates_rads`.
///
/// Earth-frame levelling rates come from the wrapped attitude-controller
/// target, clamped to [`ACRO_LEVEL_MAX_ANGLE_RAD`] and scaled by the balance
/// gains. LIMITED then adds a sqrt-controller shove when that target is
/// already past `lean_angle_max`. The earth-frame vector is converted to
/// body frame before it is mixed with the stick request — LEVELING fades
/// the mix with the largest stick and `ahrs.cos_pitch()`, then constrains
/// each axis so the sum cannot reverse through inverted; LIMITED just adds.
fn acro_trainer_blend(
    mut request: AcroRateDemand,
    roll_in_norm: f32,
    pitch_in_norm: f32,
    view: &AcroRatesView,
) -> AcroRateDemand {
    let roll_angle_rad = wrap_pi(view.att_target_roll_rad);
    let pitch_angle_rad = wrap_pi(view.att_target_pitch_rad);

    let mut rate_ef_level = Vector3f::new(
        -constrain_value(
            roll_angle_rad,
            -ACRO_LEVEL_MAX_ANGLE_RAD,
            ACRO_LEVEL_MAX_ANGLE_RAD,
        ) * view.balance_roll,
        -constrain_value(
            pitch_angle_rad,
            -ACRO_LEVEL_MAX_ANGLE_RAD,
            ACRO_LEVEL_MAX_ANGLE_RAD,
        ) * view.balance_pitch,
        0.0,
    );

    if view.trainer == AcroTrainer::Limited {
        let angle_max_rad = view.lean_angle_max_rad;
        let p = radians(view.rp_rate_degs) / ACRO_LEVEL_MAX_OVERSHOOT_RAD;
        if roll_angle_rad > angle_max_rad {
            rate_ef_level.x += sqrt_controller(
                angle_max_rad - roll_angle_rad,
                p,
                view.accel_roll_max_radss,
                view.dt,
            );
        } else if roll_angle_rad < -angle_max_rad {
            rate_ef_level.x += sqrt_controller(
                -angle_max_rad - roll_angle_rad,
                p,
                view.accel_roll_max_radss,
                view.dt,
            );
        }
        if pitch_angle_rad > angle_max_rad {
            rate_ef_level.y += sqrt_controller(
                angle_max_rad - pitch_angle_rad,
                p,
                view.accel_pitch_max_radss,
                view.dt,
            );
        } else if pitch_angle_rad < -angle_max_rad {
            rate_ef_level.y += sqrt_controller(
                -angle_max_rad - pitch_angle_rad,
                p,
                view.accel_pitch_max_radss,
                view.dt,
            );
        }
    }

    let rate_bf_level = euler_derivative_to_body(view.att_target, rate_ef_level);

    if view.trainer == AcroTrainer::Limited {
        request.roll_rads += rate_bf_level.x;
        request.pitch_rads += rate_bf_level.y;
        request.yaw_rads += rate_bf_level.z;
        return request;
    }

    let stick = roll_in_norm
        .abs()
        .max(pitch_in_norm.abs())
        .max(view.yaw_in_norm.abs());
    let acro_level_mix = constrain_value(1.0 - stick, 0.0, 1.0) * view.cos_pitch;
    request.roll_rads = acro_leveling_axis(request.roll_rads, rate_bf_level.x * acro_level_mix);
    request.pitch_rads = acro_leveling_axis(request.pitch_rads, rate_bf_level.y * acro_level_mix);
    request.yaw_rads = acro_leveling_axis(request.yaw_rads, rate_bf_level.z * acro_level_mix);
    request
}

/// Full `ModeAcro::get_pilot_desired_rates_rads`, trainer branch included.
#[must_use]
pub fn acro_get_pilot_desired_rates_rads(view: &AcroRatesView) -> AcroRateDemand {
    let (request, roll_in_norm, pitch_in_norm) = acro_request_from_sticks(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.yaw_in_norm,
        view.rp_rate_degs,
        view.rp_expo,
        view.yaw_rate_degs,
        view.yaw_expo,
    );
    if view.trainer == AcroTrainer::Off {
        return request;
    }
    acro_trainer_blend(request, roll_in_norm, pitch_in_norm, view)
}

/// Upstream `ModeAcro::init`. Always succeeds; `ignore_checks` is unread.
///
/// The air-mode option is the only thing it looks at. When the bit is set it
/// clears `disable_air_mode_reset` and writes `AIRMODE_ENABLED`. When the bit
/// is clear it touches nothing — a vehicle that entered Acro without the
/// option keeps whatever air-mode state it already had.
#[must_use]
pub fn acro_init(_ignore_checks: bool, air_mode_option: bool, state: &mut AcroAirMode) -> bool {
    if air_mode_option {
        state.disable_air_mode_reset = false;
        state.air_mode = AirMode::Enabled;
    }
    true
}

/// Upstream `ModeAcro::exit`.
///
/// Disables air-mode only when the option is set *and* an AUX switch has not
/// claimed the latch. The disable flag is always cleared, so a later `init`
/// starts from a clean slate even if this exit did not write `air_mode`.
pub fn acro_exit(air_mode_option: bool, state: &mut AcroAirMode) {
    if !state.disable_air_mode_reset && air_mode_option {
        state.air_mode = AirMode::Disabled;
    }
    state.disable_air_mode_reset = false;
}

/// Upstream `ModeAcro::air_mode_aux_changed`.
///
/// The AUX switch owns air-mode now. `exit` must not reset it.
pub fn acro_air_mode_aux_changed(state: &mut AcroAirMode) {
    state.disable_air_mode_reset = true;
}

/// Upstream `ModeAcro::throttle_hover`.
///
/// `ACRO_THR_MID` wins when it is positive. Zero and negative fall through
/// to the base mode's hover — `is_positive` is the test, not "non-zero".
#[must_use]
pub fn acro_throttle_hover(acro_thr_mid: f32, mode_hover: f32) -> f32 {
    if is_positive(acro_thr_mid) {
        acro_thr_mid
    } else {
        mode_hover
    }
}

/// Upstream `ModeAcro::run`.
///
/// Trainer-off rates, then the shared spool leftover, then the rate-loop
/// versus attitude-stabilised controller choice. `update_simple_mode` is
/// not called — Acro does not transform the sticks into a heading frame.
/// Trainer blending lives in [`acro_get_pilot_desired_rates_rads`].
#[must_use]
pub fn acro_run(view: &AcroRunView) -> AcroRun {
    let rates = acro_pilot_desired_rates_rads(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.yaw_in_norm,
        view.rp_rate_degs,
        view.rp_expo,
        view.yaw_rate_degs,
        view.yaw_expo,
    );

    let desired_spool = manual_throttle_desired_spool(view.throttle_zero);
    let pilot_throttle =
        pilot_desired_throttle(view.throttle_control, view.mid_stick, view.throttle_hover);
    let leftover = manual_spool_leftover(
        view.spool_state,
        view.throttle_lower_limited,
        pilot_throttle,
    );

    AcroRun {
        desired_spool,
        rates,
        throttle_out: leftover.throttle_out,
        angle_boost: false,
        rate_loop_only: view.rate_loop_only,
        scale_i_to_angle_p: view.rate_loop_only,
        reset_target_and_rate: leftover.reset_attitude,
        reset_target_rate: leftover.reset_attitude,
        reset_rate_i: leftover.reset_rate_i,
        clear_land_complete: leftover.clear_land_complete,
    }
}
