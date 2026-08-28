//! `ModeAltHold`, upstream `ArduCopter/mode_althold.cpp`.
//!
//! The ground-to-air machine this mode runs is already
//! [`crate::alt_hold::alt_hold_state`]. The stick conversions are
//! [`crate::stick_nav::pilot_desired_lean_angles_rad`] and
//! [`crate::pilot_input::pilot_desired_yaw_rate_rads`]. What this file
//! still owns is the leftover those two do not decide: [`althold_init`],
//! the per-state attitude / vertical-controller actions, and the climb-rate
//! clamp that sits between the pilot conversion and the machine.
//!
//! # `init` arms the D controller, it does not fly
//!
//! `ModeAltHold::init` always succeeds (`ignore_checks` is unread). It
//! inits the vertical position controller only when it is inactive — a
//! mode change mid-climb must not reset a controller that is already
//! tracking — then writes the same pilot speed / accel limits to both the
//! max and the correction setters.
//!
//! # Avoidance and surface-tracking are not here
//!
//! The flying branch calls `get_avoidance_adjusted_climbrate_ms` and, when
//! compiled in, `surface_tracking.update_surface_offset` and
//! `avoid.adjust_roll_pitch_rad`. Those read `AC_Avoid` and the rangefinder
//! surface tracker, which are not ported. With both compiled out the climb
//! rate and the lean angles pass through unchanged — which is the path this
//! leftover records. A caller that has avoidance or surface tracking active
//! must not use the climb / lean leftovers unmodified.

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::constrain_value;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// Pilot / vehicle view `ModeAltHold::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct AltHoldRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`.
    pub lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// Already-converted climb rate, m/s, up positive. The conversion
    /// itself is `Copter::get_pilot_desired_climb_rate_ms` (COP-021).
    pub target_climb_rate_ms: f32,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `motors->armed()`.
    pub armed: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `takeoff.running()`.
    pub takeoff_running: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.using_interlock`.
    pub using_interlock: bool,
    /// `g2.pilot_takeoff_alt_m`.
    pub takeoff_alt_m: f32,
}

impl AltHoldRunView {
    /// Armed, auto-armed, airborne, motors unlimited.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            target_climb_rate_ms: 0.0,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            armed: true,
            spool_state: SpoolState::ThrottleUnlimited,
            takeoff_running: false,
            auto_armed: true,
            land_complete: false,
            using_interlock: false,
            takeoff_alt_m: 2.5,
        }
    }
}

/// What `ModeAltHold::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct AltHoldInitView {
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
}

/// Leftover of one `ModeAltHold::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltHoldInit {
    /// `D_init_controller()` — only when the controller was inactive.
    pub init_d_controller: bool,
    /// Speed / accel written to both limit setters.
    pub speed_dn_ms: f32,
    /// Climb speed written to both limit setters.
    pub speed_up_ms: f32,
    /// Vertical accel written to both limit setters.
    pub accel_d_mss: f32,
    /// Always true: `D_set_max_speed_accel_m`.
    pub set_max_speed_accel: bool,
    /// Always true: `D_set_correction_speed_accel_m`, same three numbers.
    pub set_correction_speed_accel: bool,
    /// Always true. `ignore_checks` is unread.
    pub ok: bool,
}

/// Per-state vertical leftover of one AltHold tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AltHoldVertical {
    /// `D_relax_controller(0)` — motors stopped or landed.
    Relax,
    /// Start the takeoff helper if it is not already running, then
    /// `takeoff.do_pilot_takeoff_ms`.
    Takeoff,
    /// `D_set_pos_target_from_climb_rate_ms`.
    ClimbRate,
}

/// Attitude / throttle leftover of one `ModeAltHold::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltHoldRun {
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command from the machine, if any.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Euler roll demand, radians.
    pub target_roll_rad: f32,
    /// Euler pitch demand, radians.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Climb rate after the speed clamp (and, on takeoff / flying, after
    /// the avoidance identity).
    pub target_climb_rate_ms: f32,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` this iteration.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. MotorStopped passes `false`; the landed
    /// ground-idle fallthrough uses the default `true`.
    pub reset_yaw_rate: bool,
    /// Vertical-controller leftover.
    pub vertical: AltHoldVertical,
    /// `takeoff.start_m` should run (Takeoff and the helper is idle).
    pub start_takeoff: bool,
    /// Altitude handed to `takeoff.start_m`, clamped to `[0, 10]`.
    pub takeoff_start_alt_m: f32,
    /// Always true: `pos_control->D_update_controller()` runs after the
    /// switch, on every state.
    pub update_d_controller: bool,
}

/// Upstream `ModeAltHold::init`. Always succeeds; `ignore_checks` is unread.
///
/// The D controller is initialised only when it is not already active. Both
/// limit setters then receive the same three pilot numbers — max and
/// correction are the same write, not two different policies.
#[must_use]
pub fn althold_init(_ignore_checks: bool, view: &AltHoldInitView) -> AltHoldInit {
    AltHoldInit {
        init_d_controller: !view.d_is_active,
        speed_dn_ms: view.speed_dn_ms,
        speed_up_ms: view.speed_up_ms,
        accel_d_mss: view.accel_d_mss,
        set_max_speed_accel: true,
        set_correction_speed_accel: true,
        ok: true,
    }
}

/// Upstream `ModeAltHold::run`.
///
/// Clamps the already-converted climb rate, runs the altitude-hold machine,
/// then records the per-state leftover. The attitude call is always
/// `input_euler_angle_roll_pitch_euler_rate_yaw_rad`; the vertical
/// controller is always updated afterwards.
#[must_use]
pub fn althold_run(view: &AltHoldRunView) -> AltHoldRun {
    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );

    let target_climb_rate_ms = constrain_value(
        view.target_climb_rate_ms,
        -view.speed_dn_ms,
        view.speed_up_ms,
    );

    let decision = alt_hold_state(&AltHoldInputs {
        armed: view.armed,
        spool_state: view.spool_state,
        takeoff_running: view.takeoff_running,
        auto_armed: view.auto_armed,
        land_complete: view.land_complete,
        using_interlock: view.using_interlock,
        target_climb_rate_ms,
    });

    let (reset_rate_i, reset_yaw, reset_yaw_rate, vertical, start_takeoff) = match decision.state {
        AltHoldModeState::MotorStopped => (
            RateIReset::Hard,
            true,
            false,
            AltHoldVertical::Relax,
            false,
        ),
        AltHoldModeState::LandedGroundIdle => (
            RateIReset::Smooth,
            true,
            true,
            AltHoldVertical::Relax,
            false,
        ),
        AltHoldModeState::LandedPreTakeoff => (
            RateIReset::Smooth,
            false,
            false,
            AltHoldVertical::Relax,
            false,
        ),
        AltHoldModeState::Takeoff => (
            RateIReset::None,
            false,
            false,
            AltHoldVertical::Takeoff,
            !view.takeoff_running,
        ),
        AltHoldModeState::Flying => (
            RateIReset::None,
            false,
            false,
            AltHoldVertical::ClimbRate,
            false,
        ),
    };

    AltHoldRun {
        state: decision.state,
        desired_spool: decision.desired_spool,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        target_climb_rate_ms,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        update_d_controller: true,
    }
}
