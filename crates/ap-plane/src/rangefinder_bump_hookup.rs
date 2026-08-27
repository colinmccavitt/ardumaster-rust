//! Vehicle loop rangefinder bump hookup, upstream the call to
//! `landing.adjust_landing_slope_for_rangefinder_bump` in `Plane.cpp`.

use ap_landing::go_around::{LandingFlags, LandingType, SlopeLandingFlags};
use ap_landing::rangefinder_bump::{
    adjust_landing_slope_for_rangefinder_bump, RangefinderBumpConfig, RangefinderBumpInputs,
    RangefinderBumpResult, RangefinderBumpState,
};
use ap_landing::slope_stage::RangefinderState;
use ap_landing::{SlopeConfig, SlopeInputs};

/// Persistent slope-landing state the vehicle carries for rangefinder bumps.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpContext {
    pub flags: LandingFlags,
    pub slope_flags: SlopeLandingFlags,
    pub slope: f32,
    pub initial_slope: f32,
    pub rf: RangefinderState,
}

impl Default for RangefinderBumpContext {
    fn default() -> Self {
        Self {
            flags: LandingFlags::default(),
            slope_flags: SlopeLandingFlags::default(),
            slope: 0.0,
            initial_slope: 0.0,
            rf: RangefinderState {
                in_use: false,
                correction: 0.0,
                last_stable_correction: 0.0,
            },
        }
    }
}

/// Inputs for one vehicle-loop rangefinder bump tick.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpHookupInputs {
    pub flight_stage_is_land: bool,
    pub landing_type: LandingType,
    pub bump_cfg: RangefinderBumpConfig,
    pub slope_cfg: SlopeConfig,
    pub slope_inp: SlopeInputs,
    pub bump: RangefinderBumpInputs,
}

/// Apply a rangefinder bump when in LAND on a slope landing, upstream the
/// `landing.adjust_landing_slope_for_rangefinder_bump(...)` call.
#[must_use]
pub fn rangefinder_bump_hookup(
    ctx: &mut RangefinderBumpContext,
    inp: &RangefinderBumpHookupInputs,
) -> Option<RangefinderBumpResult> {
    if !inp.flight_stage_is_land || !ctx.flags.in_progress {
        return None;
    }
    if inp.landing_type != LandingType::StandardGlideSlope {
        return None;
    }

    let mut state = RangefinderBumpState {
        slope: ctx.slope,
        initial_slope: ctx.initial_slope,
        landing: ctx.flags,
        slope_flags: ctx.slope_flags,
        rf: ctx.rf,
    };

    let result = adjust_landing_slope_for_rangefinder_bump(
        &inp.bump_cfg,
        &inp.slope_cfg,
        &inp.slope_inp,
        &mut state,
        &inp.bump,
    );

    ctx.slope = state.slope;
    ctx.flags = state.landing;
    ctx.slope_flags = state.slope_flags;
    ctx.rf = state.rf;

    if result.recalculated {
        Some(result)
    } else {
        None
    }
}
