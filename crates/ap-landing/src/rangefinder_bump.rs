//! Rangefinder glide-slope bump, upstream
//! `AP_Landing::type_slope_adjust_landing_slope_for_rangefinder_bump`.

use ap_math::location::{AltContext, AltFrame, Location};

use crate::go_around::{apply_slope_abort_go_around, LandingFlags, SlopeLandingFlags};
use crate::slope_stage::{
    abort_decision, should_recalculate_slope, AbortDecision, RangefinderState,
};
use crate::{loc_alt_amsl_cm, setup_landing_glide_slope, SlopeConfig, SlopeInputs, SlopeResult};

/// Thresholds for slope recalculation and abort, upstream the landing params.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpConfig {
    /// Minimum correction change to trigger recalculation, metres. Upstream
    /// `slope_recalc_shallow_threshold`.
    pub shallow_threshold: f32,
    /// Maximum steepening, degrees, before commanding go-around. Upstream
    /// `slope_recalc_steep_threshold_to_abort`.
    pub steep_threshold_deg: f32,
}

/// HAL and navigation inputs for one bump tick.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpInputs {
    pub rf: RangefinderState,
    pub prev_wp: Location,
    pub next_wp: Location,
    pub current: Location,
    /// Distance to the landing point along the approach, metres.
    pub wp_distance_m: f32,
    /// Altitude the vehicle is navigating on, cm AMSL. Upstream
    /// `adjusted_altitude_cm_fn()`.
    pub adjusted_altitude_cm: i32,
    pub alt_ctx: AltContext,
}

/// Persistent landing state the bump reads and updates.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpState {
    pub slope: f32,
    pub initial_slope: f32,
    pub landing: LandingFlags,
    pub slope_flags: SlopeLandingFlags,
    pub rf: RangefinderState,
}

/// Result of one rangefinder bump evaluation.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderBumpResult {
    pub prev_wp: Location,
    pub slope: f32,
    pub target_altitude_offset_cm: i32,
    pub slope_setup: Option<SlopeResult>,
    pub recalculated: bool,
    pub go_around: bool,
    pub alt_offset: f32,
}

/// Recalculate the glide slope when the rangefinder reports a large correction
/// change, upstream `type_slope_adjust_landing_slope_for_rangefinder_bump`.
#[must_use]
pub fn adjust_landing_slope_for_rangefinder_bump(
    cfg: &RangefinderBumpConfig,
    slope_cfg: &SlopeConfig,
    slope_inp: &SlopeInputs,
    state: &mut RangefinderBumpState,
    inp: &RangefinderBumpInputs,
) -> RangefinderBumpResult {
    let no_change = RangefinderBumpResult {
        prev_wp: inp.prev_wp,
        slope: state.slope,
        target_altitude_offset_cm: 0,
        slope_setup: None,
        recalculated: false,
        go_around: false,
        alt_offset: state.slope_flags.alt_offset,
    };

    if !should_recalculate_slope(&inp.rf, cfg.shallow_threshold) {
        return no_change;
    }

    let Some(next_alt_cm) = loc_alt_amsl_cm(inp.next_wp, &inp.alt_ctx) else {
        return no_change;
    };

    let mut rf = inp.rf;
    rf.last_stable_correction = rf.correction;

    let mut prev_wp = inp.prev_wp;
    let corrected_alt_m =
        (inp.adjusted_altitude_cm - next_alt_cm) as f32 * 0.01 - rf.correction;
    let total_distance_m = prev_wp.get_distance(inp.next_wp) as f32;
    let mut wp_distance = inp.wp_distance_m;
    if wp_distance < 1.0 {
        wp_distance = 1.0;
    }
    let top_of_glide_slope_alt_m = total_distance_m * corrected_alt_m / wp_distance;
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream truncates top_of_glide_slope_alt_m*100 to int32"
    )]
    let new_prev_alt_cm = (top_of_glide_slope_alt_m * 100.0) as i32 + next_alt_cm;
    prev_wp.set_alt_cm(new_prev_alt_cm, AltFrame::Absolute);

    let mut slope = state.slope;
    let mut setup_inp = *slope_inp;
    setup_inp.prev_wp = prev_wp;
    setup_inp.next_wp = inp.next_wp;
    setup_inp.current = inp.current;

    let slope_setup = setup_landing_glide_slope(slope_cfg, &setup_inp, &mut slope);
    let target_altitude_offset_cm = slope_setup
        .as_ref()
        .map_or(0, |r| r.target_altitude_offset_cm);

    state.slope = slope;
    state.rf = rf;

    let decision = abort_decision(
        rf.correction,
        slope,
        state.initial_slope,
        cfg.steep_threshold_deg,
        state.slope_flags.has_aborted_due_to_slope_recalc,
    );

    let mut go_around = false;
    let mut alt_offset = state.slope_flags.alt_offset;
    if let AbortDecision::GoAround { alt_offset: off } = decision {
        let _ = apply_slope_abort_go_around(&mut state.landing, &mut state.slope_flags, off);
        go_around = true;
        alt_offset = off;
    }

    RangefinderBumpResult {
        prev_wp,
        slope,
        target_altitude_offset_cm,
        slope_setup,
        recalculated: true,
        go_around,
        alt_offset,
    }
}
