//! Vehicle loop landing integration, upstream the `landing.*` calls scattered
//! through `ArduPlane/commands_logic.cpp`, `mode_auto.cpp`, `Plane.cpp`,
//! `navigation.cpp`, and `servos.cpp`.
//!
//! `ap-landing` owns the stage machines and the AP_Landing-level dispatch;
//! this module is where the vehicle reads HAL measurements, advances the
//! machines, and applies the results to navigation, attitude, and servos.

use ap_landing::deepstall_stage::DeepstallVerifyInputs;
use ap_landing::go_around::{override_servos, LandingFlags, LandingType};
use ap_landing::landing_controller::{
    constrain_roll, get_target_airspeed_cm, get_target_altitude_location, is_flaring,
    is_flying_forward, is_on_approach, is_throttle_suppressed, TargetAirspeedInputs,
};
use ap_landing::landing_state_machine::{
    slope_transition_from_hal, verify_land_step, LandingMachineState, VerifyLandCommonInputs,
    VerifyLandEffects,
};
use ap_landing::slope_stage::FlareConfig;
use ap_math::location::Location;

use crate::target_altitude::TargetAltitudeInputs;

/// Persistent landing state the vehicle carries, upstream `AP_Landing`'s
/// flags, type, and stage machines together.
#[derive(Debug, Clone, Copy)]
pub struct LandingContext {
    pub flags: LandingFlags,
    pub landing_type: LandingType,
    pub machine: LandingMachineState,
}

impl Default for LandingContext {
    fn default() -> Self {
        Self {
            flags: LandingFlags::default(),
            landing_type: LandingType::StandardGlideSlope,
            machine: LandingMachineState::default(),
        }
    }
}

/// HAL measurements for one `verify_land` tick, upstream the locals built in
/// `Plane::verify_command` before `landing.verify_land`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLandVehicleInputs {
    /// Height above the landing point after rangefinder correction, metres.
    pub height_above_target_m: f32,
    /// Terrain correction subtracted before verify, metres.
    pub terrain_correction_m: f32,
    pub sink_rate_ms: f32,
    pub wp_proportion: f32,
    pub is_flying: bool,
    pub rangefinder_in_range: bool,
    pub bearing_error_cd: i32,
    pub crosstrack_error_m: f32,
    pub nav_data_is_stale: bool,
    pub below_prev_wp: bool,
    pub prev_cmd_is_loiter_to_alt: bool,
    pub crash_detection_enable: bool,
    pub flare_cfg: FlareConfig,
    pub deepstall: DeepstallVerifyInputs,
}

/// Height passed to verify_land after terrain correction is removed, upstream
/// `height -= auto_state.terrain_correction` in `commands_logic.cpp`.
#[must_use]
pub fn verify_land_height(inp: &VerifyLandVehicleInputs) -> f32 {
    inp.height_above_target_m - inp.terrain_correction_m
}

/// Advance the landing state machine one vehicle tick, upstream
/// `AP_Landing::verify_land`.
#[must_use]
pub fn verify_land_tick(
    ctx: &mut LandingContext,
    inp: &VerifyLandVehicleInputs,
) -> VerifyLandEffects {
    let common = VerifyLandCommonInputs {
        height_m: verify_land_height(inp),
        sink_rate_ms: inp.sink_rate_ms,
        wp_proportion: inp.wp_proportion,
        is_flying: inp.is_flying,
        rangefinder_in_range: inp.rangefinder_in_range,
    };
    let slope_transition = slope_transition_from_hal(
        &common,
        inp.bearing_error_cd,
        inp.crosstrack_error_m,
        inp.nav_data_is_stale,
        inp.below_prev_wp,
        inp.prev_cmd_is_loiter_to_alt,
        inp.crash_detection_enable,
    );
    let step = verify_land_step(
        ctx.landing_type,
        ctx.machine,
        &slope_transition,
        &inp.flare_cfg,
        &inp.deepstall,
    );
    ctx.machine = step.state;
    step.effects
}

/// Inputs to the AUTO-mode NAV_LAND run path, upstream `ModeAuto::run`.
#[derive(Debug, Clone, Copy)]
pub struct AutoLandRunInputs {
    pub nav_roll_cd: i32,
    pub level_roll_limit_cd: i32,
}

/// Outputs from the AUTO-mode NAV_LAND run path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoLandRunOutput {
    pub nav_roll_cd: i32,
    pub throttle_suppressed: bool,
}

/// Apply landing roll limits and decide throttle suppression, upstream
/// `ModeAuto::run` on `MAV_CMD_NAV_LAND`.
#[must_use]
pub fn auto_land_run(ctx: &LandingContext, inp: AutoLandRunInputs) -> AutoLandRunOutput {
    let nav_roll_cd = constrain_roll(
        ctx.landing_type,
        ctx.machine.slope_stage,
        inp.nav_roll_cd,
        inp.level_roll_limit_cd,
    );
    let throttle_suppressed = is_throttle_suppressed(
        &ctx.flags,
        ctx.landing_type,
        ctx.machine.slope_stage,
        ctx.machine.deepstall.stage,
    );
    AutoLandRunOutput {
        nav_roll_cd,
        throttle_suppressed,
    }
}

/// Whether AHRS should fuse forward flight during LAND, upstream
/// `Plane::set_fly_forward_state`.
#[must_use]
pub fn fly_forward_during_land(ctx: &LandingContext) -> bool {
    is_flying_forward(
        &ctx.flags,
        ctx.landing_type,
        ctx.machine.deepstall.stage,
    )
}

/// Landing airspeed target during LAND flight stage, upstream
/// `Plane::calc_target_airspeed_cm`.
#[must_use]
pub fn landing_target_airspeed_cm(
    ctx: &LandingContext,
    inp: &TargetAirspeedInputs,
) -> i32 {
    get_target_airspeed_cm(
        &ctx.flags,
        ctx.landing_type,
        ctx.machine.slope_stage,
        ctx.machine.deepstall.stage,
        inp,
    )
}

/// Whether deepstall overrides servos this tick, upstream
/// `AP_Landing::override_servos` in `servos.cpp`.
#[must_use]
pub fn landing_override_servos(ctx: &LandingContext) -> bool {
    override_servos(
        &ctx.flags,
        ctx.landing_type,
        Some(ctx.machine.deepstall.stage),
    )
}

/// Landing predicates for [`crate::target_altitude::target_altitude`], upstream
/// the three `landing.*` calls at the top of `Mode::update_target_altitude`.
#[must_use]
pub fn target_altitude_landing_inputs(
    ctx: &LandingContext,
    landing_point: Location,
) -> TargetAltitudeInputs {
    TargetAltitudeInputs {
        landing_is_flaring: is_flaring(
            &ctx.flags,
            ctx.landing_type,
            ctx.machine.slope_stage,
        ),
        landing_is_on_approach: is_on_approach(
            &ctx.flags,
            ctx.landing_type,
            ctx.machine.slope_stage,
            ctx.machine.deepstall.stage,
        ),
        landing_has_target_location: get_target_altitude_location(
            &ctx.flags,
            ctx.landing_type,
            landing_point,
        )
        .is_some(),
        soaring_gliding: false,
        reached_loiter_target: false,
        next_wp_is_terrain_alt: false,
        offset_cm: 0,
        past_interval_finish_line: false,
    }
}
