//! `ModeStabilize::run`, upstream `ArduCopter/mode_stabilize.cpp`.
//!
//! The stick conversions this mode uses — lean angles, yaw rate, throttle —
//! already live in [`crate::stick_nav`] and [`crate::pilot_input`]. What this
//! file still owns is the leftover those conversions do not decide: where the
//! motors should be heading, which attitude-controller resets fire on the
//! ground, and the throttle that actually leaves after the spool switch has
//! had its say.
//!
//! # Why the spool switch is half of the function
//!
//! Stabilize always ends the same way — euler roll/pitch plus a yaw rate,
//! then `set_throttle_out` with angle boost on. The thing that changes is
//! what those numbers *are* after the motors have been asked to idle or fly.
//! A shut-down or idle aircraft is forced to zero throttle and has its yaw
//! target and rate-controller integrators cleared, so a twitch on the ground
//! cannot wind up a demand that fires on takeoff. Only once the motors read
//! `THROTTLE_UNLIMITED` does the pilot's throttle mean what it says, and only
//! then does a raised stick clear the landing flag.

use crate::pilot_input::{pilot_desired_throttle, pilot_desired_yaw_rate_rads};
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use core::f32::consts::FRAC_PI_6;

/// How a manual-throttle mode resets the rate-controller integrators.
///
/// Upstream has two calls. The hard one is used while the motors are stopped,
/// where there is nothing to smooth. The smooth one is used at ground idle,
/// so the integrators decay rather than snap — a landed aircraft with rotors
/// turning still has a rate loop running, and a hard reset would be a jerk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateIReset {
    /// No reset this iteration.
    None,
    /// `reset_rate_controller_I_terms`.
    Hard,
    /// `reset_rate_controller_I_terms_smoothly`.
    Smooth,
}

/// Desired spool from the throttle stick, identical in Stabilize and Acro.
///
/// The motors' own setter still holds a disarmed aircraft at `SHUT_DOWN`;
/// this is only the mode's ask. `throttle_zero` is the vehicle flag, not the
/// stick sitting at zero — a dead-zone around the bottom of the stick is
/// already folded in before the flag is set.
#[must_use]
pub fn manual_throttle_desired_spool(throttle_zero: bool) -> DesiredSpoolState {
    if throttle_zero {
        DesiredSpoolState::GroundIdle
    } else {
        DesiredSpoolState::ThrottleUnlimited
    }
}

/// What the Stabilize / Acro spool switch decides about throttle and resets.
///
/// Shared because the two modes run the same four-way switch on
/// `get_spool_state()`. They differ only in *which* attitude reset they
/// issue — Stabilize resets yaw, Acro resets the whole target — so that
/// choice is left to the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManualSpoolLeftover {
    /// Throttle after the switch. Forced to zero while shut down or idle.
    pub throttle_out: f32,
    /// Rate-controller I-term reset, if any.
    pub reset_rate_i: RateIReset,
    /// Whether this branch resets an attitude target.
    pub reset_attitude: bool,
    /// `set_land_complete(false)` on the unlimited branch above the
    /// throttle-lower limit.
    pub clear_land_complete: bool,
}

/// The spool-state leftover Stabilize and Acro share.
///
/// # Spooling is a no-op on purpose
///
/// `SPOOLING_UP` and `SPOOLING_DOWN` fall through without touching throttle
/// or integrators. The aircraft is between idle and flying; the pilot's
/// throttle is already the one that will apply when the ramp finishes, and
/// resetting integrators mid-ramp would throw away the attitude loop that is
/// already holding the airframe still.
#[must_use]
pub fn manual_spool_leftover(
    spool_state: SpoolState,
    throttle_lower_limited: bool,
    pilot_desired_throttle: f32,
) -> ManualSpoolLeftover {
    match spool_state {
        SpoolState::ShutDown => ManualSpoolLeftover {
            throttle_out: 0.0,
            reset_rate_i: RateIReset::Hard,
            reset_attitude: true,
            clear_land_complete: false,
        },
        SpoolState::GroundIdle => ManualSpoolLeftover {
            throttle_out: 0.0,
            reset_rate_i: RateIReset::Smooth,
            reset_attitude: true,
            clear_land_complete: false,
        },
        SpoolState::ThrottleUnlimited => ManualSpoolLeftover {
            throttle_out: pilot_desired_throttle,
            reset_rate_i: RateIReset::None,
            reset_attitude: false,
            clear_land_complete: !throttle_lower_limited,
        },
        SpoolState::SpoolingUp | SpoolState::SpoolingDown => ManualSpoolLeftover {
            throttle_out: pilot_desired_throttle,
            reset_rate_i: RateIReset::None,
            reset_attitude: false,
            clear_land_complete: false,
        },
    }
}

/// Pilot / vehicle view `ModeStabilize::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct StabilizeRunView {
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
    /// Hover throttle, 0..1, used to shape the throttle curve.
    pub throttle_hover: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`. Used as both `angle_max`
    /// and `angle_limit` — Stabilize does not tighten the limit the way
    /// AltHold does.
    pub lean_angle_max_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// `copter.ap.throttle_zero`.
    pub throttle_zero: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `motors->limit.throttle_lower`.
    pub throttle_lower_limited: bool,
}

impl StabilizeRunView {
    /// Valid radio, mid throttle, motors unlimited — the flying path.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            throttle_control: 500,
            mid_stick: 500,
            throttle_hover: 0.5,
            has_valid_input: true,
            lean_angle_max_rad: FRAC_PI_6,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            throttle_zero: false,
            spool_state: SpoolState::ThrottleUnlimited,
            throttle_lower_limited: false,
        }
    }
}

/// Attitude / throttle leftover of one `ModeStabilize::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StabilizeRun {
    /// Where the motors should be heading.
    pub desired_spool: DesiredSpoolState,
    /// Euler roll demand, radians.
    pub target_roll_rad: f32,
    /// Euler pitch demand, radians.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// `set_throttle_out` throttle after the spool switch.
    pub throttle_out: f32,
    /// Always true: Stabilize boosts throttle for lean.
    pub angle_boost: bool,
    /// `reset_yaw_target_and_rate()` on shut-down and ground-idle.
    pub reset_yaw_target_and_rate: bool,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `set_land_complete(false)`.
    pub clear_land_complete: bool,
}

/// Upstream `ModeStabilize::run`.
///
/// Converts the pilot, asks for a spool state, then applies the shared
/// spool leftover. The attitude call is always
/// `input_euler_angle_roll_pitch_euler_rate_yaw_rad`; the leftover is the
/// numbers it is given and the throttle that follows.
#[must_use]
pub fn stabilize_run(view: &StabilizeRunView) -> StabilizeRun {
    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.lean_angle_max_rad,
        view.lean_angle_max_rad,
        view.has_valid_input,
    );
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );

    let desired_spool = manual_throttle_desired_spool(view.throttle_zero);
    let pilot_throttle =
        pilot_desired_throttle(view.throttle_control, view.mid_stick, view.throttle_hover);
    let leftover = manual_spool_leftover(
        view.spool_state,
        view.throttle_lower_limited,
        pilot_throttle,
    );

    StabilizeRun {
        desired_spool,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        throttle_out: leftover.throttle_out,
        angle_boost: true,
        reset_yaw_target_and_rate: leftover.reset_attitude,
        reset_rate_i: leftover.reset_rate_i,
        clear_land_complete: leftover.clear_land_complete,
    }
}
