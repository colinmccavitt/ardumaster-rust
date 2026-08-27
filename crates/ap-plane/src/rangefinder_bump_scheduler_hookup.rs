//! Vehicle loop rangefinder bump hookup for the scheduler tick.
//!
//! Upstream `Plane::update_flight_mode` calls
//! `landing.adjust_landing_slope_for_rangefinder_bump` each cycle in LAND.

use ap_landing::go_around::LandingType;
use ap_landing::rangefinder_bump::RangefinderBumpResult;

use crate::landing_loop::LandingContext;
use crate::rangefinder_bump_hookup::{
    rangefinder_bump_hookup, RangefinderBumpContext, RangefinderBumpHookupInputs,
};

/// HAL inputs for one rangefinder-bump scheduler tick.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpSchedulerInputs {
    pub hookup: RangefinderBumpHookupInputs,
}

/// Result of one rangefinder-bump scheduler tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct RangefinderBumpSchedulerOutput {
    pub result: Option<RangefinderBumpResult>,
    pub ran: bool,
}

/// Recalculate the glide slope when rangefinder correction jumps during LAND.
#[must_use]
pub fn rangefinder_bump_scheduler_tick(
    bump: &mut RangefinderBumpContext,
    landing: &mut LandingContext,
    flight_stage_is_land: bool,
    inp: &RangefinderBumpSchedulerInputs,
) -> RangefinderBumpSchedulerOutput {
    if !flight_stage_is_land {
        return RangefinderBumpSchedulerOutput::default();
    }

    bump.flags.in_progress = landing.flags.in_progress;

    let mut hookup = inp.hookup;
    hookup.flight_stage_is_land = flight_stage_is_land;
    hookup.landing_type = landing.landing_type;

    let result = rangefinder_bump_hookup(bump, &hookup);

    if bump.flags.commanded_go_around {
        landing.flags.commanded_go_around = true;
    }

    RangefinderBumpSchedulerOutput { result, ran: true }
}
