//! `ModeAcro::run`, upstream `ArduCopter/mode_acro.cpp`.
//!
//! Acro is Stabilize's rate-mode sibling. The spool leftover is the same
//! function — [`crate::mode_stabilize::manual_spool_leftover`] — because the
//! C++ switch is the same four-way split. What changes is the demand: body-
//! frame rates instead of euler angles, a different attitude reset, and
//! throttle without angle boost.
//!
//! # Trainer is not here
//!
//! `get_pilot_desired_rates_rads` has a long trainer branch that pulls the
//! aircraft back to level from the attitude-controller target. That branch
//! is a leftover of this file, not of `run()` itself: `run()` only *calls*
//! the conversion. The conversion ported here is the trainer-off path —
//! circular-limited sticks, expo, scaled by the ACRO command model — which
//! is what `run()` hands the rate controller when `ACRO_TRAINER` is `OFF`.
//!
//! # `RATE_LOOP_ONLY` is a different controller call, not a different rate
//!
//! The option does not change the numbers. It changes which input function
//! they go to, and scales the I-term to angle-P first so a Betaflight-style
//! tune still holds. The leftover therefore carries the flag rather than
//! two sets of rates.

use crate::mode_stabilize::{manual_spool_leftover, manual_throttle_desired_spool, RateIReset};
use crate::pilot_input::pilot_desired_throttle;
use ap_math::control::input_expo;
use ap_math::scalar::{norm2, radians};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `AcroOptions::RATE_LOOP_ONLY`, upstream `ModeAcro::AcroOptions`.
pub const ACRO_OPTION_RATE_LOOP_ONLY: u8 = 1 << 1;

/// `AcroOptions::AIR_MODE`. Not consumed by `run()`; recorded so the
/// leftover catalog of this file names the bit `init` / `exit` still own.
pub const ACRO_OPTION_AIR_MODE: u8 = 1 << 0;

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
    /// Hover throttle, 0..1. Acro's `throttle_hover()` override is a
    /// leftover of `init` / the command model, not of `run()`.
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

/// Trainer-off body-frame rates, upstream `ModeAcro::get_pilot_desired_rates_rads`.
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
    let mut roll_in_norm = roll_in_norm;
    let mut pitch_in_norm = pitch_in_norm;
    let norm_in_length = norm2(pitch_in_norm, roll_in_norm);
    if norm_in_length > 1.0 {
        let ratio = 1.0 / norm_in_length;
        roll_in_norm *= ratio;
        pitch_in_norm *= ratio;
    }

    AcroRateDemand {
        roll_rads: radians(rp_rate_degs) * input_expo(roll_in_norm, rp_expo),
        pitch_rads: radians(rp_rate_degs) * input_expo(pitch_in_norm, rp_expo),
        yaw_rads: radians(yaw_rate_degs) * input_expo(yaw_in_norm, yaw_expo),
    }
}

/// Upstream `ModeAcro::run`.
///
/// Trainer-off rates, then the shared spool leftover, then the rate-loop
/// versus attitude-stabilised controller choice. `update_simple_mode` is
/// not called — Acro does not transform the sticks into a heading frame.
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
