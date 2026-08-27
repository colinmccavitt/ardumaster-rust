//! Vehicle loop landing state machine hookup for the scheduler tick.
//!
//! Upstream `Plane::verify_command` calls `landing.verify_land` each cycle
//! while a NAV_LAND is active; `ModeAuto::run` applies roll limits and
//! throttle suppression from the landing controller.

use ap_landing::landing_state_machine::VerifyLandEffects;

use crate::landing_loop::{
    auto_land_run, verify_land_tick, AutoLandRunInputs, LandingContext, VerifyLandVehicleInputs,
};

/// HAL measurements and limits for one landing-loop scheduler tick.
#[derive(Debug, Clone, Copy)]
pub struct LandingLoopSchedulerInputs {
    pub verify: VerifyLandVehicleInputs,
    pub nav_roll_cd: i32,
    pub level_roll_limit_cd: i32,
}

/// Result of one landing-loop scheduler tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LandingLoopSchedulerOutput {
    pub effects: VerifyLandEffects,
    pub nav_roll_cd: i32,
    pub throttle_suppressed: bool,
    pub ran: bool,
}

/// Advance the landing state machine and apply AUTO LAND roll/throttle rules.
#[must_use]
pub fn landing_loop_scheduler_tick(
    ctx: &mut LandingContext,
    inp: &LandingLoopSchedulerInputs,
) -> LandingLoopSchedulerOutput {
    let effects = verify_land_tick(ctx, &inp.verify);
    let auto = auto_land_run(
        ctx,
        AutoLandRunInputs {
            nav_roll_cd: inp.nav_roll_cd,
            level_roll_limit_cd: inp.level_roll_limit_cd,
        },
    );
    LandingLoopSchedulerOutput {
        effects,
        nav_roll_cd: auto.nav_roll_cd,
        throttle_suppressed: auto.throttle_suppressed,
        ran: true,
    }
}
