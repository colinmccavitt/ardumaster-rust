//! `ModeSport` init / run leftover, upstream `ArduCopter/mode_sport.cpp`.
//!
//! Tracked as **COP-024**. Sport is AltHold's vertical machine with Acro-style
//! earth-frame rate sticks. Roll and pitch are `norm_input_dz * radians(ACRO_RP)`
//! — no expo, no circular limit — then a trainer-style balance pull and a
//! lean-max overshoot `sqrt_controller` that always runs (not gated on
//! `ACRO_TRAINER`). Yaw is the shared pilot leftover. The attitude call is
//! `input_euler_rate_roll_pitch_yaw_rads`.
//!
//! # `init` is AltHold's D start
//!
//! `ModeSport::init` always succeeds (`ignore_checks` is unread). It writes
//! the same pilot speed / accel limits to both the max and the correction
//! setters, then inits the vertical position controller only when it is
//! inactive — a mode change mid-climb must not reset a controller that is
//! already tracking.
//!
//! # Rate sticks, not lean angles
//!
//! Acro circular-limits and expo-scales the stick before the command-model
//! rate. Sport does neither. A diagonal stick asks for both axes at once.
//! `update_simple_mode` still runs first; the leftover takes the already-
//! transformed stick.
//!
//! The balance term is the wrapped attitude-controller target, clamped to
//! [`ACRO_LEVEL_MAX_ANGLE_RAD`] and scaled by `ACRO_BALANCE_*`, subtracted
//! from the stick rate. When that same wrapped target is already past
//! `lean_angle_max`, a `sqrt_controller` shove is added — the LIMITED Acro
//! trainer path, but Sport always applies it. There is no earth-to-body
//! conversion: the rates stay earth-frame and go to
//! `input_euler_rate_roll_pitch_yaw_rads`.
//!
//! # Avoidance and surface-tracking are not here
//!
//! The flying and takeoff branches call `get_avoidance_adjusted_climbrate_ms`
//! and, when compiled in, `surface_tracking.update_surface_offset`. Those
//! read `AC_Avoid` and the rangefinder surface tracker, which are not
//! ported. With both compiled out the climb rate passes through unchanged —
//! which is the path this leftover records.

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_acro::{ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_OVERSHOOT_RAD};
use crate::mode_althold::AltHoldVertical;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use ap_math::control::sqrt_controller;
use ap_math::scalar::{constrain_value, radians, wrap_pi};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `Mode::Number::SPORT`.
pub const MODE_NUMBER_SPORT: u8 = 13;

/// `ModeSport` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SportModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. Sport is a rate mode; GPS is not required.
    pub requires_position: bool,
    /// `has_manual_throttle()`. False: the D controller owns throttle.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
}

/// Upstream `ModeSport` flags.
#[must_use]
pub const fn sport_mode_flags() -> SportModeFlags {
    SportModeFlags {
        mode_number: MODE_NUMBER_SPORT,
        requires_position: false,
        has_manual_throttle: false,
        allows_arming: true,
        is_autopilot: false,
    }
}

/// Upstream `ModeSport::has_user_takeoff`.
///
/// Sport can climb in place. A caller that needs the takeoff to navigate
/// (`must_navigate`) is refused.
#[must_use]
pub const fn sport_has_user_takeoff(must_navigate: bool) -> bool {
    !must_navigate
}

/// What `ModeSport::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct SportInitView {
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
}

impl SportInitView {
    /// Pilot defaults, D controller already running.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            d_is_active: true,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
        }
    }
}

/// Leftover of one `ModeSport::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SportInit {
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

/// Pilot / vehicle view `ModeSport::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct SportRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`. Consulted only for yaw.
    pub has_valid_input: bool,
    /// `g2.command_model_acro_rp.get_rate()`, deg/s.
    pub rp_rate_degs: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// `attitude_control->get_att_target_euler_rad().x`.
    pub att_target_roll_rad: f32,
    /// `attitude_control->get_att_target_euler_rad().y`.
    pub att_target_pitch_rad: f32,
    /// `g.acro_balance_roll`.
    pub balance_roll: f32,
    /// `g.acro_balance_pitch`.
    pub balance_pitch: f32,
    /// `attitude_control->lean_angle_max_rad()`.
    pub lean_angle_max_rad: f32,
    /// `attitude_control->get_accel_roll_max_radss()`.
    pub accel_roll_max_radss: f32,
    /// `attitude_control->get_accel_pitch_max_radss()`.
    pub accel_pitch_max_radss: f32,
    /// `G_Dt`.
    pub dt: f32,
    /// Already-converted climb rate, m/s, up positive.
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

impl SportRunView {
    /// Armed, auto-armed, airborne, motors unlimited, level target.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            rp_rate_degs: 360.0,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            att_target_roll_rad: 0.0,
            att_target_pitch_rad: 0.0,
            balance_roll: 1.0,
            balance_pitch: 1.0,
            lean_angle_max_rad: ACRO_LEVEL_MAX_ANGLE_RAD,
            accel_roll_max_radss: 0.0,
            accel_pitch_max_radss: 0.0,
            dt: 0.0025,
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

/// Attitude / throttle leftover of one `ModeSport::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SportRun {
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command from the machine, if any.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Earth-frame roll-rate demand, rad/s.
    pub target_roll_rads: f32,
    /// Earth-frame pitch-rate demand, rad/s.
    pub target_pitch_rads: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rads: f32,
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
    /// Always true: `D_set_max_speed_accel_m` on every tick. Sport `run`
    /// does not rewrite the correction limits.
    pub set_max_speed_accel: bool,
    /// Always true: `input_euler_rate_roll_pitch_yaw_rads`.
    pub input_euler_rate: bool,
    /// Always true: `pos_control->D_update_controller()` runs after the
    /// switch, on every state.
    pub update_d_controller: bool,
}

/// Sport roll / pitch / yaw rate leftover.
///
/// Roll and pitch are the raw stick times the ACRO_RP rate, then the
/// balance pull and the lean-max overshoot. Yaw is
/// [`pilot_desired_yaw_rate_rads`].
#[must_use]
pub fn sport_target_rates_rads(view: &SportRunView) -> (f32, f32, f32) {
    let mut target_roll_rads = view.roll_in_norm * radians(view.rp_rate_degs);
    let mut target_pitch_rads = view.pitch_in_norm * radians(view.rp_rate_degs);
    let target_yaw_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );

    let roll_angle_rad = wrap_pi(view.att_target_roll_rad);
    let pitch_angle_rad = wrap_pi(view.att_target_pitch_rad);

    target_roll_rads -= constrain_value(
        roll_angle_rad,
        -ACRO_LEVEL_MAX_ANGLE_RAD,
        ACRO_LEVEL_MAX_ANGLE_RAD,
    ) * view.balance_roll;
    target_pitch_rads -= constrain_value(
        pitch_angle_rad,
        -ACRO_LEVEL_MAX_ANGLE_RAD,
        ACRO_LEVEL_MAX_ANGLE_RAD,
    ) * view.balance_pitch;

    let angle_max_rad = view.lean_angle_max_rad;
    let p = radians(view.rp_rate_degs) / ACRO_LEVEL_MAX_OVERSHOOT_RAD;
    if roll_angle_rad > angle_max_rad {
        target_roll_rads += sqrt_controller(
            angle_max_rad - roll_angle_rad,
            p,
            view.accel_roll_max_radss,
            view.dt,
        );
    } else if roll_angle_rad < -angle_max_rad {
        target_roll_rads += sqrt_controller(
            -angle_max_rad - roll_angle_rad,
            p,
            view.accel_roll_max_radss,
            view.dt,
        );
    }
    if pitch_angle_rad > angle_max_rad {
        target_pitch_rads += sqrt_controller(
            angle_max_rad - pitch_angle_rad,
            p,
            view.accel_pitch_max_radss,
            view.dt,
        );
    } else if pitch_angle_rad < -angle_max_rad {
        target_pitch_rads += sqrt_controller(
            -angle_max_rad - pitch_angle_rad,
            p,
            view.accel_pitch_max_radss,
            view.dt,
        );
    }

    (target_roll_rads, target_pitch_rads, target_yaw_rads)
}

/// Upstream `ModeSport::init`. Always succeeds; `ignore_checks` is unread.
///
/// The D controller is initialised only when it is not already active. Both
/// limit setters then receive the same three pilot numbers — max and
/// correction are the same write, not two different policies.
#[must_use]
pub fn sport_init(_ignore_checks: bool, view: &SportInitView) -> SportInit {
    SportInit {
        init_d_controller: !view.d_is_active,
        speed_dn_ms: view.speed_dn_ms,
        speed_up_ms: view.speed_up_ms,
        accel_d_mss: view.accel_d_mss,
        set_max_speed_accel: true,
        set_correction_speed_accel: true,
        ok: true,
    }
}

/// Upstream `ModeSport::run`.
///
/// Writes the D max limits, converts sticks to earth-frame rates, runs the
/// altitude-hold machine, then records the per-state leftover. The
/// attitude call is always `input_euler_rate_roll_pitch_yaw_rads`; the
/// vertical controller is always updated afterwards.
#[must_use]
pub fn sport_run(view: &SportRunView) -> SportRun {
    let (target_roll_rads, target_pitch_rads, target_yaw_rads) = sport_target_rates_rads(view);

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
        AltHoldModeState::MotorStopped => {
            (RateIReset::Hard, true, false, AltHoldVertical::Relax, false)
        }
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

    SportRun {
        state: decision.state,
        desired_spool: decision.desired_spool,
        target_roll_rads,
        target_pitch_rads,
        target_yaw_rads,
        target_climb_rate_ms,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        set_max_speed_accel: true,
        input_euler_rate: true,
        update_d_controller: true,
    }
}
