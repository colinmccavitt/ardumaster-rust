//! AP_Landing-level verify_land dispatch, upstream `AP_Landing::verify_land`.
//!
//! Slope and deepstall each own their stage machines; this module wires the
//! HAL measurements upstream reads each control cycle into those machines.

use crate::deepstall_stage::{
    deepstall_verify_land_step, DeepstallVerifyInputs, DeepstallVerifyState,
};
use crate::go_around::LandingType;
use crate::slope_stage::{FlareConfig, SlopeStage, TransitionInputs};

/// Measurements common to both landing types, upstream the shared arguments
/// to `AP_Landing::verify_land`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLandCommonInputs {
    /// Height above the landing point, metres.
    pub height_m: f32,
    /// Current sink rate, m/s, positive downward.
    pub sink_rate_ms: f32,
    /// Proportion along the landing leg, 0 at previous waypoint and 1 at aim.
    pub wp_proportion: f32,
    /// Whether the vehicle believes it is flying.
    pub is_flying: bool,
    /// Whether the rangefinder is giving usable height.
    pub rangefinder_in_range: bool,
}

/// Persistent landing state across verify_land ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandingMachineState {
    pub slope_stage: SlopeStage,
    pub deepstall: DeepstallVerifyState,
}

impl Default for LandingMachineState {
    fn default() -> Self {
        Self {
            slope_stage: SlopeStage::Normal,
            deepstall: DeepstallVerifyState::default(),
        }
    }
}

/// Effects the vehicle should perform after a verify_land tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VerifyLandEffects {
    /// Deepstall: rebuild the approach path for wind refinement.
    pub rebuild_approach_path: bool,
    /// Deepstall: record the breakout location at the current position.
    pub record_breakout_at_current: bool,
    /// Deepstall: just entered the land stage.
    pub entered_deepstall_land: bool,
    /// Slope: just entered the flare stage.
    pub entered_slope_final: bool,
    /// Slope: just entered the pre-flare stage.
    pub entered_slope_preflare: bool,
}

/// Result of one verify_land tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyLandStep {
    pub state: LandingMachineState,
    pub effects: VerifyLandEffects,
}

/// Build slope transition inputs from HAL measurements, upstream the locals
/// in `type_slope_verify_land`.
#[must_use]
pub fn slope_transition_from_hal(
    common: &VerifyLandCommonInputs,
    bearing_error_cd: i32,
    crosstrack_error_m: f32,
    nav_data_is_stale: bool,
    below_prev_wp: bool,
    prev_cmd_is_loiter_to_alt: bool,
    crash_detection_enable: bool,
) -> TransitionInputs {
    TransitionInputs {
        wp_proportion: common.wp_proportion,
        height: common.height_m,
        sink_rate: common.sink_rate_ms,
        bearing_error_cd,
        crosstrack_error_m,
        nav_data_is_stale,
        below_prev_wp,
        prev_cmd_is_loiter_to_alt,
        rangefinder_in_range: common.rangefinder_in_range,
        is_flying: common.is_flying,
        crash_detection_enable,
    }
}

/// Advance the landing state machine one tick, upstream
/// `AP_Landing::verify_land`.
#[must_use]
pub fn verify_land_step(
    landing_type: LandingType,
    state: LandingMachineState,
    slope_transition: &TransitionInputs,
    flare_cfg: &FlareConfig,
    deepstall_inp: &DeepstallVerifyInputs,
) -> VerifyLandStep {
    match landing_type {
        LandingType::StandardGlideSlope => {
            let prev = state.slope_stage;
            let next = prev.next(slope_transition, flare_cfg);
            let mut effects = VerifyLandEffects::default();
            if next == SlopeStage::Final && prev != SlopeStage::Final {
                effects.entered_slope_final = true;
            }
            if next == SlopeStage::Preflare && prev == SlopeStage::Approach {
                effects.entered_slope_preflare = true;
            }
            VerifyLandStep {
                state: LandingMachineState {
                    slope_stage: next,
                    ..state
                },
                effects,
            }
        }
        LandingType::Deepstall => {
            let step = deepstall_verify_land_step(state.deepstall, deepstall_inp);
            VerifyLandStep {
                state: LandingMachineState {
                    deepstall: step.state,
                    ..state
                },
                effects: VerifyLandEffects {
                    rebuild_approach_path: step.effects.rebuild_approach_path,
                    record_breakout_at_current: step.effects.record_breakout_at_current,
                    entered_deepstall_land: step.effects.entered_land,
                    ..VerifyLandEffects::default()
                },
            }
        }
    }
}
