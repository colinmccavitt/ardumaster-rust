//! `ModePosHold` init / run leftover, upstream `ArduCopter/mode_poshold.cpp`.
//!
//! Tracked as **COP-015**. Horizontal hold is [`ap_wpnav::Loiter`] (COP-011);
//! this file does not rewrite it. What it still owns is the PosHold leftover
//! those leftovers do not decide: mix the pilot with a per-axis brake
//! machine, blend into AC_Loiter only when both axes are ready, and run the
//! same altitude-hold machine [`crate::mode_althold`] already uses.
//!
//! # `init` seats AC_Loiter and picks the starting RP mode
//!
//! `ModePosHold::init` always succeeds (`ignore_checks` is unread). It does
//! not convert the stick — unlike [`crate::mode_loiter`]. Lean filters start
//! at zero, `brake.gain` is computed from `PHLD_BRK_RATE`, and both axes
//! start in `LOITER` when landed or `PILOT_OVERRIDE` when airborne so a mode
//! change mid-flight cannot twitch. AC_Loiter is cleared and seated; the
//! wind-compensation estimate is zeroed. The vertical controller is
//! initialised only when it is inactive.
//!
//! # Run is an altitude-hold machine plus two roll/pitch machines
//!
//! Every tick writes the vertical speed / accel limits, clears the loiter
//! pilot-accel leftover, converts the stick through the attitude lean-angle
//! max (not [`Loiter::get_angle_max_rad`]), and constrains climb rate.
//! `land_complete_maybe` calls [`Loiter::soften_for_landing`]. Stopped and
//! takeoff re-seat with [`Loiter::init_target`]; flying does not tick
//! AC_Loiter from the altitude switch — only the combined `BRAKE_TO_LOITER`
//! / `LOITER` RP modes do, and they pass `avoidance_on = false`.
//!
//! # Avoidance and surface-tracking are not here
//!
//! The flying / takeoff branches call
//! `get_avoidance_adjusted_climbrate_ms` and, when compiled in,
//! `surface_tracking.update_surface_offset`. Those read `AC_Avoid` and the
//! rangefinder surface tracker, which are not ported. With both compiled out
//! the climb rate passes through unchanged — which is the path this leftover
//! records.
//!
//! Attitude is always `input_euler_angle_roll_pitch_euler_rate_yaw_rad`
//! with the mixed roll / pitch leftover and the pilot yaw rate. That is
//! not Loiter's thrust-vector call.

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_althold::AltHoldVertical;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{constrain_value, is_zero, radians, GRAVITY_MSS};
use ap_math::vector2::Vector2f;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use ap_wpnav::{
    InitTargetContext, InitTargetLeftover, Loiter, UpdateLoiterContext, UpdateLoiterLeftover,
};

/// `Mode::Number::POSHOLD`.
pub const MODE_NUMBER_POSHOLD: u8 = 16;

/// Speed (m/s) below which braking may shorten its timeout. Upstream
/// `POSHOLD_SPEED_0`.
pub const POSHOLD_SPEED_0_MS: f32 = 10.0;

/// Maximum braking-timeout estimate, ms. Upstream
/// `POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS`.
pub const POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS: u32 = 6_000;

/// Brake-to-loiter blend window, ms. Upstream `POSHOLD_BRAKE_TO_LOITER_TIME_MS`.
pub const POSHOLD_BRAKE_TO_LOITER_TIME_MS: u32 = 1_500;

/// Delay after entering loiter before wind-comp updates. Upstream
/// `POSHOLD_WIND_COMP_START_TIME_MS`.
pub const POSHOLD_WIND_COMP_START_TIME_MS: u32 = 1_500;

/// Controller-to-pilot blend window, ms. Upstream
/// `POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS`.
pub const POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS: u32 = 500;

/// Stick-release low-pass factor. Upstream `POSHOLD_SMOOTH_RATE_FACTOR`.
pub const POSHOLD_SMOOTH_RATE_FACTOR: f32 = 0.0125;

/// Wind-comp accel low-pass time constant. Upstream `TC_WIND_COMP`.
pub const TC_WIND_COMP: f32 = 0.0025;

/// Stick-release snap threshold, rad. Upstream
/// `POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD`.
pub const POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD: f32 = 0.314_159_27;

/// Horizontal speed above which wind-comp is frozen, m/s. Upstream
/// `POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX_MS`.
pub const POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX_MS: f32 = 0.10;

/// Wind-comp lean capped at this fraction of angle max. Upstream
/// `POSHOLD_WIND_COMP_LEAN_PCT_MAX`.
pub const POSHOLD_WIND_COMP_LEAN_PCT_MAX: f32 = 0.6666;

/// Default `PHLD_BRK_ANGLE` for a multirotor. Upstream
/// `POSHOLD_BRAKE_ANGLE_DEG_DEFAULT`.
pub const POSHOLD_BRAKE_ANGLE_DEG_DEFAULT: f32 = 30.0;

/// Default `PHLD_BRK_RATE`, deg/s. Upstream `POSHOLD_BRAKE_RATE_DEFAULT`.
pub const POSHOLD_BRAKE_RATE_DEFAULT_DEGS: f32 = 8.0;

/// Floor enforced at the top of `run`. Upstream `POSHOLD_BRAKE_RATE_MIN`.
pub const POSHOLD_BRAKE_RATE_MIN_DEGS: f32 = 4.0;

/// Per-axis roll / pitch submode. Upstream `ModePosHold::RPMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RpMode {
    /// Pilot is controlling this axis.
    PilotOverride = 0,
    /// This axis is braking towards zero.
    Brake,
    /// Braking finished; loiter waits for the other axis.
    BrakeReadyToLoiter,
    /// Both axes blending brake into loiter.
    BrakeToLoiter,
    /// Both axes holding position through AC_Loiter.
    Loiter,
    /// Blending the last controller output into the pilot.
    ControllerToPilotOverride,
}

/// Braking leftovers persisted across `run` ticks. Upstream `ModePosHold::brake`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHoldBrake {
    /// `brake.time_updated_roll`.
    pub time_updated_roll: bool,
    /// `brake.time_updated_pitch`.
    pub time_updated_pitch: bool,
    /// `brake.gain`, rad/(m/s).
    pub gain: f32,
    /// `brake.roll_rad`.
    pub roll_rad: f32,
    /// `brake.pitch_rad`.
    pub pitch_rad: f32,
    /// `brake.start_time_roll_ms`.
    pub start_time_roll_ms: u32,
    /// `brake.start_time_pitch_ms`.
    pub start_time_pitch_ms: u32,
    /// `brake.angle_max_roll_rad`.
    pub angle_max_roll_rad: f32,
    /// `brake.angle_max_pitch_rad`.
    pub angle_max_pitch_rad: f32,
    /// `brake.loiter_transition_start_time_ms`.
    pub loiter_transition_start_time_ms: u32,
}

impl PosHoldBrake {
    #[must_use]
    const fn zero() -> Self {
        Self {
            time_updated_roll: false,
            time_updated_pitch: false,
            gain: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            start_time_roll_ms: 0,
            start_time_pitch_ms: 0,
            angle_max_roll_rad: 0.0,
            angle_max_pitch_rad: 0.0,
            loiter_transition_start_time_ms: 0,
        }
    }
}

/// Persisted PosHold state. Upstream `ModePosHold` members.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHold {
    /// `roll_mode`.
    pub roll_mode: RpMode,
    /// `pitch_mode`.
    pub pitch_mode: RpMode,
    /// Filtered pilot roll, rad.
    pub pilot_roll_rad: f32,
    /// Filtered pilot pitch, rad.
    pub pilot_pitch_rad: f32,
    /// Braking leftovers.
    pub brake: PosHoldBrake,
    /// `controller_to_pilot_start_time_roll_ms`.
    pub controller_to_pilot_start_time_roll_ms: u32,
    /// `controller_to_pilot_start_time_pitch_ms`.
    pub controller_to_pilot_start_time_pitch_ms: u32,
    /// `controller_final_roll_rad`.
    pub controller_final_roll_rad: f32,
    /// `controller_final_pitch_rad`.
    pub controller_final_pitch_rad: f32,
    /// Earth-frame wind-comp accel, m/s².
    pub wind_comp_ne_mss: Vector2f,
    /// Body-frame wind-comp roll, rad.
    pub wind_comp_roll_rad: f32,
    /// Body-frame wind-comp pitch, rad.
    pub wind_comp_pitch_rad: f32,
    /// `wind_comp_start_time_ms`.
    pub wind_comp_start_time_ms: u32,
    /// Final roll sent to the attitude controller.
    pub roll_rad: f32,
    /// Final pitch sent to the attitude controller.
    pub pitch_rad: f32,
}

impl PosHold {
    /// BSS-zeroed construction; [`poshold_init`] seats the leftover.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            roll_mode: RpMode::PilotOverride,
            pitch_mode: RpMode::PilotOverride,
            pilot_roll_rad: 0.0,
            pilot_pitch_rad: 0.0,
            brake: PosHoldBrake::zero(),
            controller_to_pilot_start_time_roll_ms: 0,
            controller_to_pilot_start_time_pitch_ms: 0,
            controller_final_roll_rad: 0.0,
            controller_final_pitch_rad: 0.0,
            wind_comp_ne_mss: Vector2f { x: 0.0, y: 0.0 },
            wind_comp_roll_rad: 0.0,
            wind_comp_pitch_rad: 0.0,
            wind_comp_start_time_ms: 0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
        }
    }
}

impl Default for PosHold {
    fn default() -> Self {
        Self::new()
    }
}

/// What `ModePosHold::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct PosHoldInitView {
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `g.poshold_brake_rate_degs`.
    pub brake_rate_degs: f32,
    /// Context [`Loiter::init_target`] reads.
    pub init_target_ctx: InitTargetContext,
}

/// Leftover of one `ModePosHold::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHoldInit {
    /// Starting [`RpMode`] for both axes.
    pub rp_mode: RpMode,
    /// `brake.gain` written this init.
    pub brake_gain: f32,
    /// Always true: lean filters start at zero.
    pub zero_pilot_lean: bool,
    /// Always true: `init_wind_comp_estimate`.
    pub init_wind_comp: bool,
    /// Always true: `clear_pilot_desired_acceleration`.
    pub clear_pilot_desired_acceleration: bool,
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

/// Pilot / vehicle view `ModePosHold::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct PosHoldRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`.
    pub attitude_lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// Already-converted climb rate, m/s, up positive.
    pub target_climb_rate_ms: f32,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`. Written every run tick.
    pub accel_d_mss: f32,
    /// `g.poshold_brake_rate_degs` before the min-rate floor.
    pub brake_rate_degs: f32,
    /// `PHLD_BRK_ANGLE`, deg.
    pub brake_angle_max_deg: f32,
    /// `G_Dt` for the lean-filter and brake slews.
    pub dt_s: f32,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `pos_control->get_vel_estimate_NED_ms()` north, m/s.
    pub vel_n_ms: f32,
    /// `pos_control->get_vel_estimate_NED_ms()` east, m/s.
    pub vel_e_ms: f32,
    /// `ahrs.cos_yaw()`.
    pub cos_yaw: f32,
    /// `ahrs.sin_yaw()`.
    pub sin_yaw: f32,
    /// `pos_control->get_accel_target_NED_mss()` north, m/s².
    pub accel_target_n_mss: f32,
    /// `pos_control->get_accel_target_NED_mss()` east, m/s².
    pub accel_target_e_mss: f32,
    /// `loiter_nav->get_roll_rad()` after `update` (PosControl leftover).
    pub loiter_roll_rad: f32,
    /// `loiter_nav->get_pitch_rad()` after `update` (PosControl leftover).
    pub loiter_pitch_rad: f32,
    /// `pos_control->get_pos_estimate_NED_m().xy() - get_pos_offset_NED_m().xy()`.
    pub pos_target_ne_m: Vector2f,
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
    /// Context [`Loiter::init_target`] reads on stopped / takeoff.
    pub init_target_ctx: InitTargetContext,
    /// Context [`Loiter::update`] reads in the combined RP modes.
    pub update_ctx: UpdateLoiterContext,
}

impl PosHoldRunView {
    /// Armed, auto-armed, airborne, motors unlimited.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            attitude_lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            target_climb_rate_ms: 0.0,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            brake_rate_degs: POSHOLD_BRAKE_RATE_DEFAULT_DEGS,
            brake_angle_max_deg: POSHOLD_BRAKE_ANGLE_DEG_DEFAULT,
            dt_s: 0.01,
            now_ms: 0,
            vel_n_ms: 0.0,
            vel_e_ms: 0.0,
            cos_yaw: 1.0,
            sin_yaw: 0.0,
            accel_target_n_mss: 0.0,
            accel_target_e_mss: 0.0,
            loiter_roll_rad: 0.0,
            loiter_pitch_rad: 0.0,
            pos_target_ne_m: Vector2f { x: 0.0, y: 0.0 },
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
                accel_target_ne_mss: Vector2f { x: 0.0, y: 0.0 },
                roll_rad: 0.0,
                pitch_rad: 0.0,
            },
            update_ctx: UpdateLoiterContext {
                now_ms: 0,
                dt_s: 0.01,
                ekf_gnd_spd_limit_ms: 50.0,
                vel_desired_ne_ms: Vector2f { x: 0.0, y: 0.0 },
                pos_desired_ne_m: Vector2f { x: 0.0, y: 0.0 },
                vel_pid_kp: 1.0,
                attitude_lean_angle_max_rad: 0.523_598_8,
                pos_lean_angle_max_rad: 0.523_598_8,
                avoidance_on: true,
            },
        }
    }
}

/// Horizontal leftover of one `ModePosHold` tick against AC_Loiter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PosHoldNavAction {
    /// No AC_Loiter call this tick (flying while both axes are not in
    /// the combined loiter modes).
    None,
    /// `loiter_nav->init_target()` — MotorStopped, LandedGroundIdle, Takeoff.
    InitTarget,
    /// `loiter_nav->init_target_m` when both axes become ready to loiter.
    InitTargetM,
    /// `loiter_nav->update(false)` — `BRAKE_TO_LOITER` and `LOITER`.
    Update,
}

/// Attitude / altitude leftover of one `ModePosHold::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHoldRun {
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command from the machine, if any.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Raw stick conversion (before the RP machines).
    pub target_roll_rad: f32,
    /// Raw stick conversion (before the RP machines).
    pub target_pitch_rad: f32,
    /// Final roll sent to the attitude controller.
    pub roll_rad: f32,
    /// Final pitch sent to the attitude controller.
    pub pitch_rad: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Climb rate after the speed clamp (and, on takeoff / flying, after
    /// the avoidance identity).
    pub target_climb_rate_ms: f32,
    /// `PHLD_BRK_RATE` after the min-rate floor.
    pub brake_rate_degs: f32,
    /// True when the incoming rate was below [`POSHOLD_BRAKE_RATE_MIN_DEGS`].
    pub brake_rate_clamped: bool,
    /// Always true: `D_set_max_speed_accel_m` at the top of `run`.
    pub set_max_speed_accel: bool,
    /// Always true: `update_simple_mode` runs before the conversion.
    pub update_simple_mode: bool,
    /// Always true: `clear_pilot_desired_acceleration` at the top of `run`.
    pub clear_pilot_desired_acceleration: bool,
    /// `loiter_nav->soften_for_landing` when `land_complete_maybe`.
    pub soften_for_landing: bool,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` this iteration.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. MotorStopped passes `false`;
    /// LandedGroundIdle uses the default `true`.
    pub reset_yaw_rate: bool,
    /// Vertical-controller leftover.
    pub vertical: AltHoldVertical,
    /// Whether this tick re-seated or ticked AC_Loiter.
    pub nav: PosHoldNavAction,
    /// [`Loiter::init_target`] leftover when [`PosHoldNavAction::InitTarget`].
    pub init_target: Option<InitTargetLeftover>,
    /// [`Loiter::init_target_m`] leftover when [`PosHoldNavAction::InitTargetM`].
    pub init_target_m: Option<InitTargetLeftover>,
    /// [`Loiter::update`] leftover when [`PosHoldNavAction::Update`].
    pub update: Option<UpdateLoiterLeftover>,
    /// `takeoff.start_m` should run (Takeoff and the helper is idle).
    pub start_takeoff: bool,
    /// Altitude handed to `takeoff.start_m`, clamped to `[0, 10]`.
    pub takeoff_start_alt_m: f32,
    /// Always true: `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
    pub input_euler_angle_roll_pitch_euler_rate_yaw: bool,
    /// Always true: `pos_control->D_update_controller()` after the machines.
    pub update_d_controller: bool,
    /// Roll RP mode after this tick.
    pub roll_mode: RpMode,
    /// Pitch RP mode after this tick.
    pub pitch_mode: RpMode,
}

/// `brake.gain` from `PHLD_BRK_RATE`. Upstream `ModePosHold::init`.
#[must_use]
pub fn poshold_brake_gain(brake_rate_degs: f32) -> f32 {
    radians((15.0 * brake_rate_degs + 95.0) * 0.01)
}

/// Mix two controls. Upstream `ModePosHold::mix_controls`.
///
/// `mix_ratio` of 1 is `first` completely; 0 is `second` completely.
#[must_use]
pub fn mix_controls(mix_ratio: f32, first_control: f32, second_control: f32) -> f32 {
    let mix_ratio = constrain_value(mix_ratio, 0.0, 1.0);
    mix_ratio * first_control + (1.0 - mix_ratio) * second_control
}

/// Filter the pilot lean angle. Upstream `update_pilot_lean_angle_rad`.
pub fn update_pilot_lean_angle_rad(
    lean_angle_filtered_rad: &mut f32,
    lean_angle_raw_rad: f32,
    brake_rate_degs: f32,
    dt_s: f32,
) {
    if (*lean_angle_filtered_rad > 0.0 && lean_angle_raw_rad < 0.0)
        || (*lean_angle_filtered_rad < 0.0 && lean_angle_raw_rad > 0.0)
        || lean_angle_raw_rad.abs() > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD
    {
        *lean_angle_filtered_rad = lean_angle_raw_rad;
        return;
    }
    let brake_rate_step_rad = radians(brake_rate_degs) * dt_s;
    if *lean_angle_filtered_rad > 0.0 {
        *lean_angle_filtered_rad -=
            (*lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR).max(brake_rate_step_rad);
        *lean_angle_filtered_rad = (*lean_angle_filtered_rad).max(lean_angle_raw_rad);
    } else {
        *lean_angle_filtered_rad +=
            (-*lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR).max(brake_rate_step_rad);
        *lean_angle_filtered_rad = (*lean_angle_filtered_rad).min(lean_angle_raw_rad);
    }
}

/// Slewed brake lean from body-frame velocity. Upstream
/// `update_brake_angle_from_velocity`.
pub fn update_brake_angle_from_velocity(
    brake_angle_rad: &mut f32,
    velocity_ms: f32,
    brake_gain: f32,
    brake_rate_degs: f32,
    brake_angle_max_deg: f32,
    dt_s: f32,
) {
    let brake_delta_rad = radians(brake_rate_degs) * dt_s;
    let lean_angle_rad =
        -brake_gain * velocity_ms * (1.0 + 5.0 / (velocity_ms.abs() + 0.60));
    *brake_angle_rad = constrain_value(
        lean_angle_rad,
        *brake_angle_rad - brake_delta_rad,
        *brake_angle_rad + brake_delta_rad,
    );
    let brake_angle_max_rad = radians(brake_angle_max_deg);
    *brake_angle_rad = constrain_value(*brake_angle_rad, -brake_angle_max_rad, brake_angle_max_rad);
}

/// Body-frame wind-comp lean from the earth-frame accel estimate.
/// Upstream `get_wind_comp_lean_angles_rad`.
#[must_use]
pub fn wind_comp_lean_angles_rad(
    wind_comp_ne_mss: Vector2f,
    cos_yaw: f32,
    sin_yaw: f32,
) -> (f32, f32) {
    let roll_angle_rad = libm::atanf(
        (-wind_comp_ne_mss.x * sin_yaw + wind_comp_ne_mss.y * cos_yaw) / GRAVITY_MSS,
    );
    let pitch_angle_rad = libm::atanf(
        -(wind_comp_ne_mss.x * cos_yaw + wind_comp_ne_mss.y * sin_yaw) / GRAVITY_MSS,
    );
    (roll_angle_rad, pitch_angle_rad)
}

fn init_wind_comp_estimate(poshold: &mut PosHold) {
    poshold.wind_comp_ne_mss = Vector2f::zero();
    poshold.wind_comp_roll_rad = 0.0;
    poshold.wind_comp_pitch_rad = 0.0;
}

fn update_wind_comp_estimate(poshold: &mut PosHold, view: &PosHoldRunView) {
    if view.now_ms.wrapping_sub(poshold.wind_comp_start_time_ms) < POSHOLD_WIND_COMP_START_TIME_MS {
        return;
    }
    let vel_ne = Vector2f::new(view.vel_n_ms, view.vel_e_ms);
    if vel_ne.length() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX_MS {
        return;
    }
    if is_zero(poshold.wind_comp_ne_mss.x) {
        poshold.wind_comp_ne_mss.x = view.accel_target_n_mss;
    } else {
        poshold.wind_comp_ne_mss.x =
            (1.0 - TC_WIND_COMP) * poshold.wind_comp_ne_mss.x + TC_WIND_COMP * view.accel_target_n_mss;
    }
    if is_zero(poshold.wind_comp_ne_mss.y) {
        poshold.wind_comp_ne_mss.y = view.accel_target_e_mss;
    } else {
        poshold.wind_comp_ne_mss.y =
            (1.0 - TC_WIND_COMP) * poshold.wind_comp_ne_mss.y + TC_WIND_COMP * view.accel_target_e_mss;
    }
    let accel_lim_mss =
        libm::tanf(POSHOLD_WIND_COMP_LEAN_PCT_MAX * view.attitude_lean_angle_max_rad) * GRAVITY_MSS;
    let wind_comp_ef_len = poshold.wind_comp_ne_mss.length();
    if !is_zero(accel_lim_mss) && wind_comp_ef_len > accel_lim_mss {
        poshold.wind_comp_ne_mss *= accel_lim_mss / wind_comp_ef_len;
    }
}

fn roll_controller_to_pilot_override(poshold: &mut PosHold, now_ms: u32) {
    poshold.roll_mode = RpMode::ControllerToPilotOverride;
    poshold.controller_to_pilot_start_time_roll_ms = now_ms;
    poshold.pilot_roll_rad = 0.0;
    poshold.controller_final_roll_rad = poshold.roll_rad;
}

fn pitch_controller_to_pilot_override(poshold: &mut PosHold, now_ms: u32) {
    poshold.pitch_mode = RpMode::ControllerToPilotOverride;
    poshold.controller_to_pilot_start_time_pitch_ms = now_ms;
    poshold.pilot_pitch_rad = 0.0;
    poshold.controller_final_pitch_rad = poshold.pitch_rad;
}

fn update_brake_timeout(
    time_updated: &mut bool,
    angle_max_rad: &mut f32,
    start_time_ms: &mut u32,
    brake_angle_rad: f32,
    vel_ms: f32,
    now_ms: u32,
    brake_rate_degs: f32,
) -> bool {
    if !*time_updated {
        if brake_angle_rad.abs() >= *angle_max_rad {
            *angle_max_rad = brake_angle_rad.abs();
        } else {
            *start_time_ms = now_ms;
            *time_updated = true;
        }
        return false;
    }
    let rate_rad = radians(brake_rate_degs);
    let estimated_ms = (1.5 * 1_000.0 * (*angle_max_rad / rate_rad)) as u32;
    let brake_timeout_ms = estimated_ms.min(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS);
    if vel_ms.abs() <= POSHOLD_SPEED_0_MS
        && now_ms.wrapping_sub(*start_time_ms) > 500
        && brake_timeout_ms > 500
    {
        *start_time_ms = now_ms.wrapping_sub(brake_timeout_ms).wrapping_add(500);
    }
    now_ms.wrapping_sub(*start_time_ms) > brake_timeout_ms
}

/// Upstream `ModePosHold::init`. Always succeeds; `ignore_checks` is unread.
///
/// Does not convert the pilot. Seats AC_Loiter, zeros the wind estimate,
/// and starts both axes in `LOITER` when landed or `PILOT_OVERRIDE` when
/// airborne. The D controller is initialised only when it is not already
/// active.
#[must_use]
pub fn poshold_init(
    _ignore_checks: bool,
    poshold: &mut PosHold,
    loiter: &mut Loiter,
    view: &PosHoldInitView,
) -> PosHoldInit {
    poshold.pilot_roll_rad = 0.0;
    poshold.pilot_pitch_rad = 0.0;
    poshold.brake.gain = poshold_brake_gain(view.brake_rate_degs);
    let rp_mode = if view.land_complete {
        RpMode::Loiter
    } else {
        RpMode::PilotOverride
    };
    poshold.roll_mode = rp_mode;
    poshold.pitch_mode = rp_mode;
    init_wind_comp_estimate(poshold);
    let init_target = loiter.init_target(view.init_target_ctx);
    PosHoldInit {
        rp_mode,
        brake_gain: poshold.brake.gain,
        zero_pilot_lean: true,
        init_wind_comp: true,
        clear_pilot_desired_acceleration: true,
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

/// Upstream `ModePosHold::run`.
///
/// Converts the pilot through the attitude lean-angle max, softens when
/// maybe-landed, then runs the altitude-hold machine. Stopped / takeoff
/// re-seat AC_Loiter; the combined RP modes tick it with `avoidance_on =
/// false`. Attitude is always the euler roll/pitch plus yaw-rate call; the
/// vertical controller is always updated afterwards.
#[must_use]
pub fn poshold_run(
    poshold: &mut PosHold,
    loiter: &mut Loiter,
    view: &PosHoldRunView,
) -> PosHoldRun {
    let brake_rate_clamped = view.brake_rate_degs < POSHOLD_BRAKE_RATE_MIN_DEGS;
    let brake_rate_degs = if brake_rate_clamped {
        POSHOLD_BRAKE_RATE_MIN_DEGS
    } else {
        view.brake_rate_degs
    };

    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.attitude_lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );
    let target_climb_rate_ms =
        constrain_value(view.target_climb_rate_ms, -view.speed_dn_ms, view.speed_up_ms);

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

    let mut init_target = None;
    let mut reset_rate_i = RateIReset::None;
    let mut reset_yaw = false;
    let mut reset_yaw_rate = false;
    let vertical;
    let mut start_takeoff = false;
    let mut nav = PosHoldNavAction::None;

    match decision.state {
        AltHoldModeState::MotorStopped => {
            reset_rate_i = RateIReset::Hard;
            reset_yaw = true;
            reset_yaw_rate = false;
            vertical = AltHoldVertical::Relax;
            init_target = Some(loiter.init_target(view.init_target_ctx));
            nav = PosHoldNavAction::InitTarget;
            poshold.roll_mode = RpMode::PilotOverride;
            poshold.pitch_mode = RpMode::PilotOverride;
            init_wind_comp_estimate(poshold);
        }
        AltHoldModeState::LandedGroundIdle => {
            init_target = Some(loiter.init_target(view.init_target_ctx));
            nav = PosHoldNavAction::InitTarget;
            reset_yaw = true;
            reset_yaw_rate = true;
            init_wind_comp_estimate(poshold);
            reset_rate_i = RateIReset::Smooth;
            vertical = AltHoldVertical::Relax;
            poshold.roll_mode = RpMode::PilotOverride;
            poshold.pitch_mode = RpMode::PilotOverride;
        }
        AltHoldModeState::LandedPreTakeoff => {
            reset_rate_i = RateIReset::Smooth;
            vertical = AltHoldVertical::Relax;
            poshold.roll_mode = RpMode::PilotOverride;
            poshold.pitch_mode = RpMode::PilotOverride;
        }
        AltHoldModeState::Takeoff => {
            vertical = AltHoldVertical::Takeoff;
            start_takeoff = !view.takeoff_running;
            init_target = Some(loiter.init_target(view.init_target_ctx));
            nav = PosHoldNavAction::InitTarget;
            poshold.roll_mode = RpMode::PilotOverride;
            poshold.pitch_mode = RpMode::PilotOverride;
        }
        AltHoldModeState::Flying => {
            vertical = AltHoldVertical::ClimbRate;
        }
    }

    let vel_fw_ms = view.vel_n_ms * view.cos_yaw + view.vel_e_ms * view.sin_yaw;
    let vel_right_ms = -view.vel_n_ms * view.sin_yaw + view.vel_e_ms * view.cos_yaw;

    if poshold.roll_mode != RpMode::Loiter || poshold.pitch_mode != RpMode::Loiter {
        let (roll, pitch) =
            wind_comp_lean_angles_rad(poshold.wind_comp_ne_mss, view.cos_yaw, view.sin_yaw);
        poshold.wind_comp_roll_rad = roll;
        poshold.wind_comp_pitch_rad = pitch;
    }

    match poshold.roll_mode {
        RpMode::PilotOverride => {
            update_pilot_lean_angle_rad(
                &mut poshold.pilot_roll_rad,
                target_roll_rad,
                brake_rate_degs,
                view.dt_s,
            );
            if is_zero(target_roll_rad)
                && poshold.pilot_roll_rad.abs() < radians(2.0 * brake_rate_degs)
            {
                poshold.roll_mode = RpMode::Brake;
                poshold.brake.roll_rad = 0.0;
                poshold.brake.angle_max_roll_rad = 0.0;
                poshold.brake.start_time_roll_ms = view.now_ms;
                poshold.brake.time_updated_roll = false;
            }
            poshold.roll_rad = poshold.pilot_roll_rad + poshold.wind_comp_roll_rad;
        }
        RpMode::Brake | RpMode::BrakeReadyToLoiter => {
            update_brake_angle_from_velocity(
                &mut poshold.brake.roll_rad,
                vel_right_ms,
                poshold.brake.gain,
                brake_rate_degs,
                view.brake_angle_max_deg,
                view.dt_s,
            );
            if update_brake_timeout(
                &mut poshold.brake.time_updated_roll,
                &mut poshold.brake.angle_max_roll_rad,
                &mut poshold.brake.start_time_roll_ms,
                poshold.brake.roll_rad,
                vel_right_ms,
                view.now_ms,
                brake_rate_degs,
            ) {
                poshold.roll_mode = RpMode::BrakeReadyToLoiter;
            }
            poshold.roll_rad = poshold.brake.roll_rad + poshold.wind_comp_roll_rad;
            if !is_zero(target_roll_rad) {
                roll_controller_to_pilot_override(poshold, view.now_ms);
            }
        }
        RpMode::BrakeToLoiter | RpMode::Loiter => {}
        RpMode::ControllerToPilotOverride => {
            update_pilot_lean_angle_rad(
                &mut poshold.pilot_roll_rad,
                target_roll_rad,
                brake_rate_degs,
                view.dt_s,
            );
            if view
                .now_ms
                .wrapping_sub(poshold.controller_to_pilot_start_time_roll_ms)
                > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS
            {
                poshold.roll_mode = RpMode::PilotOverride;
            }
            let mix = view
                .now_ms
                .wrapping_sub(poshold.controller_to_pilot_start_time_roll_ms)
                as f32
                / POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS as f32;
            poshold.roll_rad = mix_controls(
                mix,
                poshold.pilot_roll_rad + poshold.wind_comp_roll_rad,
                poshold.controller_final_roll_rad,
            );
        }
    }

    match poshold.pitch_mode {
        RpMode::PilotOverride => {
            update_pilot_lean_angle_rad(
                &mut poshold.pilot_pitch_rad,
                target_pitch_rad,
                brake_rate_degs,
                view.dt_s,
            );
            if is_zero(target_pitch_rad)
                && poshold.pilot_pitch_rad.abs() < radians(2.0 * brake_rate_degs)
            {
                poshold.pitch_mode = RpMode::Brake;
                poshold.brake.pitch_rad = 0.0;
                poshold.brake.angle_max_pitch_rad = 0.0;
                poshold.brake.start_time_pitch_ms = view.now_ms;
                poshold.brake.time_updated_pitch = false;
            }
            poshold.pitch_rad = poshold.pilot_pitch_rad + poshold.wind_comp_pitch_rad;
        }
        RpMode::Brake | RpMode::BrakeReadyToLoiter => {
            update_brake_angle_from_velocity(
                &mut poshold.brake.pitch_rad,
                -vel_fw_ms,
                poshold.brake.gain,
                brake_rate_degs,
                view.brake_angle_max_deg,
                view.dt_s,
            );
            if update_brake_timeout(
                &mut poshold.brake.time_updated_pitch,
                &mut poshold.brake.angle_max_pitch_rad,
                &mut poshold.brake.start_time_pitch_ms,
                poshold.brake.pitch_rad,
                vel_fw_ms,
                view.now_ms,
                brake_rate_degs,
            ) {
                poshold.pitch_mode = RpMode::BrakeReadyToLoiter;
            }
            poshold.pitch_rad = poshold.brake.pitch_rad + poshold.wind_comp_pitch_rad;
            if !is_zero(target_pitch_rad) {
                pitch_controller_to_pilot_override(poshold, view.now_ms);
            }
        }
        RpMode::BrakeToLoiter | RpMode::Loiter => {}
        RpMode::ControllerToPilotOverride => {
            update_pilot_lean_angle_rad(
                &mut poshold.pilot_pitch_rad,
                target_pitch_rad,
                brake_rate_degs,
                view.dt_s,
            );
            if view
                .now_ms
                .wrapping_sub(poshold.controller_to_pilot_start_time_pitch_ms)
                > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS
            {
                poshold.pitch_mode = RpMode::PilotOverride;
            }
            let mix = view
                .now_ms
                .wrapping_sub(poshold.controller_to_pilot_start_time_pitch_ms)
                as f32
                / POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS as f32;
            poshold.pitch_rad = mix_controls(
                mix,
                poshold.pilot_pitch_rad + poshold.wind_comp_pitch_rad,
                poshold.controller_final_pitch_rad,
            );
        }
    }

    let mut init_target_m = None;
    let mut update = None;
    if poshold.roll_mode == RpMode::BrakeReadyToLoiter
        && poshold.pitch_mode == RpMode::BrakeReadyToLoiter
    {
        poshold.roll_mode = RpMode::BrakeToLoiter;
        poshold.pitch_mode = RpMode::BrakeToLoiter;
        poshold.brake.loiter_transition_start_time_ms = view.now_ms;
        init_target_m = Some(loiter.init_target_m(view.pos_target_ne_m, view.init_target_ctx));
        poshold.wind_comp_start_time_ms = view.now_ms;
        nav = PosHoldNavAction::InitTargetM;
    }

    if poshold.roll_mode == RpMode::BrakeToLoiter || poshold.roll_mode == RpMode::Loiter {
        poshold.pitch_mode = poshold.roll_mode;
        match poshold.roll_mode {
            RpMode::BrakeToLoiter => {
                if view
                    .now_ms
                    .wrapping_sub(poshold.brake.loiter_transition_start_time_ms)
                    > POSHOLD_BRAKE_TO_LOITER_TIME_MS
                {
                    poshold.roll_mode = RpMode::Loiter;
                    poshold.pitch_mode = RpMode::Loiter;
                }
                let mix = view
                    .now_ms
                    .wrapping_sub(poshold.brake.loiter_transition_start_time_ms)
                    as f32
                    / POSHOLD_BRAKE_TO_LOITER_TIME_MS as f32;
                update_brake_angle_from_velocity(
                    &mut poshold.brake.roll_rad,
                    vel_right_ms,
                    poshold.brake.gain,
                    brake_rate_degs,
                    view.brake_angle_max_deg,
                    view.dt_s,
                );
                update_brake_angle_from_velocity(
                    &mut poshold.brake.pitch_rad,
                    -vel_fw_ms,
                    poshold.brake.gain,
                    brake_rate_degs,
                    view.brake_angle_max_deg,
                    view.dt_s,
                );
                let mut ctx = view.update_ctx;
                ctx.avoidance_on = false;
                ctx.now_ms = view.now_ms;
                update = Some(loiter.update(ctx));
                nav = PosHoldNavAction::Update;
                poshold.roll_rad = mix_controls(
                    mix,
                    poshold.brake.roll_rad + poshold.wind_comp_roll_rad,
                    view.loiter_roll_rad,
                );
                poshold.pitch_rad = mix_controls(
                    mix,
                    poshold.brake.pitch_rad + poshold.wind_comp_pitch_rad,
                    view.loiter_pitch_rad,
                );
                if !is_zero(target_roll_rad) {
                    roll_controller_to_pilot_override(poshold, view.now_ms);
                    poshold.pitch_mode = RpMode::BrakeReadyToLoiter;
                }
                if !is_zero(target_pitch_rad) {
                    pitch_controller_to_pilot_override(poshold, view.now_ms);
                    if is_zero(target_roll_rad) {
                        poshold.roll_mode = RpMode::BrakeReadyToLoiter;
                    }
                }
            }
            RpMode::Loiter => {
                let mut ctx = view.update_ctx;
                ctx.avoidance_on = false;
                ctx.now_ms = view.now_ms;
                update = Some(loiter.update(ctx));
                nav = PosHoldNavAction::Update;
                poshold.roll_rad = view.loiter_roll_rad;
                poshold.pitch_rad = view.loiter_pitch_rad;
                update_wind_comp_estimate(poshold, view);
                if !is_zero(target_roll_rad) {
                    roll_controller_to_pilot_override(poshold, view.now_ms);
                    poshold.pitch_mode = RpMode::BrakeReadyToLoiter;
                    poshold.brake.pitch_rad = 0.0;
                }
                if !is_zero(target_pitch_rad) {
                    pitch_controller_to_pilot_override(poshold, view.now_ms);
                    if is_zero(target_roll_rad) {
                        poshold.roll_mode = RpMode::BrakeReadyToLoiter;
                        poshold.brake.roll_rad = 0.0;
                    }
                }
            }
            _ => {}
        }
    }

    let angle_max_rad = view.attitude_lean_angle_max_rad;
    poshold.roll_rad = constrain_value(poshold.roll_rad, -angle_max_rad, angle_max_rad);
    poshold.pitch_rad = constrain_value(poshold.pitch_rad, -angle_max_rad, angle_max_rad);

    PosHoldRun {
        state: decision.state,
        desired_spool: decision.desired_spool,
        target_roll_rad,
        target_pitch_rad,
        roll_rad: poshold.roll_rad,
        pitch_rad: poshold.pitch_rad,
        target_yaw_rate_rads,
        target_climb_rate_ms,
        brake_rate_degs,
        brake_rate_clamped,
        set_max_speed_accel: true,
        update_simple_mode: true,
        clear_pilot_desired_acceleration: true,
        soften_for_landing,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        nav,
        init_target,
        init_target_m,
        update,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        input_euler_angle_roll_pitch_euler_rate_yaw: true,
        update_d_controller: true,
        roll_mode: poshold.roll_mode,
        pitch_mode: poshold.pitch_mode,
    }
}
