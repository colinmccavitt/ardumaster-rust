//! `ModeLoiter` init / run leftover, upstream `ArduCopter/mode_loiter.cpp`.
//!
//! Tracked as **COP-015**. Horizontal hold is [`ap_wpnav::Loiter`] (COP-011);
//! this file does not rewrite it. What it still owns is the ModeLoiter
//! leftover those leftovers do not decide: convert the pilot, seat or tick
//! AC_Loiter, and run the same altitude-hold machine
//! [`crate::mode_althold`] already uses.
//!
//! # `init` seats AC_Loiter, it does not fly
//!
//! `ModeLoiter::init` always succeeds (`ignore_checks` is unread). It
//! converts the stick through [`Loiter::get_angle_max_rad`], records
//! `set_pilot_desired_acceleration_rad` (COP-011's later pilot-accel
//! slice; not rewritten here), then calls [`Loiter::init_target`]. The
//! vertical controller is initialised only when it is inactive — a mode
//! change mid-climb must not reset a controller that is already
//! tracking — then writes the same pilot speed / accel limits to both
//! the max and the correction setters.
//!
//! # Run feeds AC_Loiter, then the altitude-hold machine
//!
//! Every tick writes the vertical speed / accel limits, converts the
//! pilot, records the pilot-accel leftover, and constrains climb rate.
//! `land_complete_maybe` calls [`Loiter::soften_for_landing`]. Stopped
//! and landed states re-seat with [`Loiter::init_target`]; takeoff and
//! flying tick [`Loiter::update`]. Precision loiter (`AC_PRECLAND`) is
//! not compiled in — the flying branch is the `#else` `update()` path.
//!
//! # Avoidance, surface-tracking, and thrust vector are not here
//!
//! The flying / takeoff branches call
//! `get_avoidance_adjusted_climbrate_ms` and, when compiled in,
//! `surface_tracking.update_surface_offset`. Those read `AC_Avoid` and
//! the rangefinder surface tracker, which are not ported. With both
//! compiled out the climb rate passes through unchanged — which is the
//! path this leftover records.
//!
//! Attitude is always `input_thrust_vector_rate_heading_rads` with
//! `loiter_nav->get_thrust_vector()` and `slew_yaw = false`. The
//! thrust vector itself is a PosControl leftover (COP-009); this
//! records that the call uses it.

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_althold::AltHoldVertical;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::constrain_value;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use ap_wpnav::{
    InitTargetContext, InitTargetLeftover, Loiter, UpdateLoiterContext, UpdateLoiterLeftover,
};

/// `Mode::Number::LOITER`.
pub const MODE_NUMBER_LOITER: u8 = 5;

/// Pilot / vehicle view `ModeLoiter::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct LoiterRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `_attitude_control.lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub attitude_lean_angle_max_rad: f32,
    /// `_pos_control.get_lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub pos_lean_angle_max_rad: f32,
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
    /// `get_pilot_accel_D_mss()`. Written every run tick.
    pub accel_d_mss: f32,
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
    /// `copter.ap.land_complete_maybe`.
    pub land_complete_maybe: bool,
    /// `copter.ap.using_interlock`.
    pub using_interlock: bool,
    /// `g2.pilot_takeoff_alt_m`.
    pub takeoff_alt_m: f32,
    /// Context [`Loiter::init_target`] reads on stopped / landed states.
    pub init_target_ctx: InitTargetContext,
    /// Context [`Loiter::update`] reads on takeoff / flying.
    pub update_ctx: UpdateLoiterContext,
}

impl LoiterRunView {
    /// Armed, auto-armed, airborne, motors unlimited.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            attitude_lean_angle_max_rad: 0.523_598_8,
            pos_lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            target_climb_rate_ms: 0.0,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            armed: true,
            spool_state: SpoolState::ThrottleUnlimited,
            takeoff_running: false,
            auto_armed: true,
            land_complete: false,
            land_complete_maybe: false,
            using_interlock: false,
            takeoff_alt_m: 2.5,
            init_target_ctx: InitTargetContext {
                lean_angle_max_rad: 0.523_598_8,
                accel_target_ne_mss: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                roll_rad: 0.0,
                pitch_rad: 0.0,
            },
            update_ctx: UpdateLoiterContext {
                now_ms: 0,
                dt_s: 0.01,
                ekf_gnd_spd_limit_ms: 50.0,
                vel_desired_ne_ms: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                pos_desired_ne_m: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                vel_pid_kp: 1.0,
                attitude_lean_angle_max_rad: 0.523_598_8,
                pos_lean_angle_max_rad: 0.523_598_8,
                avoidance_on: true,
            },
        }
    }
}

/// What `ModeLoiter::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct LoiterInitView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `_attitude_control.lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub attitude_lean_angle_max_rad: f32,
    /// `_pos_control.get_lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub pos_lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// Context [`Loiter::init_target`] reads.
    pub init_target_ctx: InitTargetContext,
}

/// Horizontal leftover of one `ModeLoiter` tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoiterNavAction {
    /// `loiter_nav->init_target()` — init, MotorStopped, and both landed states.
    InitTarget,
    /// `loiter_nav->update()` — Takeoff and Flying (no precland).
    Update,
}

/// Leftover of one `ModeLoiter::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoiterInit {
    /// Euler roll demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_roll_rad: f32,
    /// Euler pitch demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_pitch_rad: f32,
    /// Always true: `update_simple_mode` runs before the conversion.
    pub update_simple_mode: bool,
    /// Always true. Pilot-accel shaping stays on COP-011; this records the call.
    pub set_pilot_desired_acceleration: bool,
    /// [`Loiter::init_target`] leftover.
    pub init_target: InitTargetLeftover,
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

/// Attitude / altitude leftover of one `ModeLoiter::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoiterRun {
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command from the machine, if any.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Euler roll demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_roll_rad: f32,
    /// Euler pitch demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Climb rate after the speed clamp (and, on takeoff / flying, after
    /// the avoidance identity).
    pub target_climb_rate_ms: f32,
    /// Always true: `D_set_max_speed_accel_m` at the top of `run`.
    pub set_max_speed_accel: bool,
    /// Always true: `update_simple_mode` runs before the conversion.
    pub update_simple_mode: bool,
    /// Always true. Pilot-accel shaping stays on COP-011.
    pub set_pilot_desired_acceleration: bool,
    /// `loiter_nav->soften_for_landing` when `land_complete_maybe`.
    pub soften_for_landing: bool,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` this iteration.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. Loiter uses the default `true` on both
    /// MotorStopped and LandedGroundIdle — unlike AltHold, which passes
    /// `false` on MotorStopped.
    pub reset_yaw_rate: bool,
    /// Vertical-controller leftover.
    pub vertical: AltHoldVertical,
    /// Whether this tick re-seated or ticked AC_Loiter.
    pub nav: LoiterNavAction,
    /// [`Loiter::init_target`] leftover when [`LoiterNavAction::InitTarget`].
    pub init_target: Option<InitTargetLeftover>,
    /// [`Loiter::update`] leftover when [`LoiterNavAction::Update`].
    pub update: Option<UpdateLoiterLeftover>,
    /// `takeoff.start_m` should run (Takeoff and the helper is idle).
    pub start_takeoff: bool,
    /// Altitude handed to `takeoff.start_m`, clamped to `[0, 10]`.
    pub takeoff_start_alt_m: f32,
    /// Always true: `input_thrust_vector_rate_heading_rads`.
    pub input_thrust_vector_rate_heading: bool,
    /// `slew_yaw` argument, always false.
    pub slew_yaw: bool,
    /// Always true: `pos_control->D_update_controller()` after the switch.
    pub update_d_controller: bool,
}

/// Upstream `ModeLoiter::init`. Always succeeds; `ignore_checks` is unread.
///
/// Converts the pilot through [`Loiter::get_angle_max_rad`], records the
/// pilot-accel leftover, then seats AC_Loiter with [`Loiter::init_target`].
/// The D controller is initialised only when it is not already active.
#[must_use]
pub fn loiter_init(_ignore_checks: bool, loiter: &mut Loiter, view: &LoiterInitView) -> LoiterInit {
    let angle_max_rad = loiter.get_angle_max_rad(
        view.attitude_lean_angle_max_rad,
        view.pos_lean_angle_max_rad,
    );
    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let init_target = loiter.init_target(view.init_target_ctx);
    LoiterInit {
        target_roll_rad,
        target_pitch_rad,
        update_simple_mode: true,
        set_pilot_desired_acceleration: true,
        init_target,
        init_d_controller: !view.d_is_active,
        speed_dn_ms: view.speed_dn_ms,
        speed_up_ms: view.speed_up_ms,
        accel_d_mss: view.accel_d_mss,
        set_max_speed_accel: true,
        set_correction_speed_accel: true,
        ok: true,
    }
}

/// Upstream `ModeLoiter::run`.
///
/// Converts the pilot through [`Loiter::get_angle_max_rad`], records the
/// pilot-accel leftover, softens when maybe-landed, then runs the
/// altitude-hold machine. Stopped / landed re-seat AC_Loiter; takeoff /
/// flying tick it. Attitude is always the thrust-vector heading-rate
/// call; the vertical controller is always updated afterwards.
#[must_use]
pub fn loiter_run(loiter: &mut Loiter, view: &LoiterRunView) -> LoiterRun {
    let angle_max_rad = loiter.get_angle_max_rad(
        view.attitude_lean_angle_max_rad,
        view.pos_lean_angle_max_rad,
    );
    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        angle_max_rad,
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

    let soften_for_landing = if view.land_complete_maybe {
        loiter.soften_for_landing()
    } else {
        false
    };

    let decision = alt_hold_state(&AltHoldInputs {
        armed: view.armed,
        spool_state: view.spool_state,
        takeoff_running: view.takeoff_running,
        auto_armed: view.auto_armed,
        land_complete: view.land_complete,
        using_interlock: view.using_interlock,
        target_climb_rate_ms,
    });

    let (reset_rate_i, reset_yaw, reset_yaw_rate, vertical, start_takeoff, nav) =
        match decision.state {
            AltHoldModeState::MotorStopped => (
                RateIReset::Hard,
                true,
                true,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
            ),
            AltHoldModeState::LandedGroundIdle => (
                RateIReset::Smooth,
                true,
                true,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
            ),
            AltHoldModeState::LandedPreTakeoff => (
                RateIReset::Smooth,
                false,
                false,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
            ),
            AltHoldModeState::Takeoff => (
                RateIReset::None,
                false,
                false,
                AltHoldVertical::Takeoff,
                !view.takeoff_running,
                LoiterNavAction::Update,
            ),
            AltHoldModeState::Flying => (
                RateIReset::None,
                false,
                false,
                AltHoldVertical::ClimbRate,
                false,
                LoiterNavAction::Update,
            ),
        };

    let (init_target, update) = match nav {
        LoiterNavAction::InitTarget => (Some(loiter.init_target(view.init_target_ctx)), None),
        LoiterNavAction::Update => (None, Some(loiter.update(view.update_ctx))),
    };

    LoiterRun {
        state: decision.state,
        desired_spool: decision.desired_spool,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        target_climb_rate_ms,
        set_max_speed_accel: true,
        update_simple_mode: true,
        set_pilot_desired_acceleration: true,
        soften_for_landing,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        nav,
        init_target,
        update,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        input_thrust_vector_rate_heading: true,
        slew_yaw: false,
        update_d_controller: true,
    }
}
