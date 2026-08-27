//! Deepstall landing stage machine, upstream `AP_Landing_Deepstall::verify_land`.
//!
//! The stages mirror mavlink `DEEPSTALL_STAGE` because they are logged. Query
//! helpers and transition predicates are pure: the caller supplies distances
//! and geometry; nothing here touches nav controllers or AHRS.

use ap_math::location::{AltFrame, Location};
use ap_math::scalar::{degrees, is_zero, wrap_180, wrap_180_cd};
use ap_math::vector2::Vector2f;

use crate::deepstall::{verify_breakout, LOITER_ALT_TOLERANCE_M};

/// One full loiter at the target altitude, upstream `loiter_sum_cd >= 36000`.
pub const LOITER_COMPLETE_CD: i32 = 36_000;

/// Where the aircraft is in a deepstall landing, upstream `DEEPSTALL_STAGE`.
///
/// Strictly ordered — `verify_land` only advances forward through these — and
/// the discriminants match mavlink because they are logged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum DeepstallStage {
    /// Flying to the landing point.
    FlyToLanding = 0,
    /// Building an estimate of the wind.
    EstimateWind = 1,
    /// Waiting to breakout of the loiter to fly the approach.
    WaitForBreakout = 2,
    /// Flying to the first arc point to turn around to the landing point.
    FlyToArc = 3,
    /// Turning around back to the deepstall landing point.
    Arc = 4,
    /// Approaching the landing point.
    Approach = 5,
    /// Stalling and steering towards the land point.
    Land = 6,
}

impl DeepstallStage {
    /// Initial stage after `do_land`, upstream `DEEPSTALL_STAGE_FLY_TO_LANDING`.
    pub const INITIAL: Self = Self::FlyToLanding;
}

/// Whether throttle is suppressed, upstream `is_throttle_suppressed`.
#[must_use]
pub fn is_throttle_suppressed(stage: DeepstallStage) -> bool {
    stage == DeepstallStage::Land
}

/// Whether the aircraft is flying forward, upstream `is_flying_forward`.
#[must_use]
pub fn is_flying_forward(stage: DeepstallStage) -> bool {
    stage != DeepstallStage::Land
}

/// Whether the aircraft is on approach, upstream `is_on_approach`.
///
/// Upstream returns `stage == LAND` — the name reflects the stall approach,
/// not the [`DeepstallStage::Approach`] stage.
#[must_use]
pub fn is_on_approach(stage: DeepstallStage) -> bool {
    stage == DeepstallStage::Land
}

/// Target airspeed in cm/s, upstream `get_target_airspeed_cm`.
#[must_use]
pub fn target_airspeed_cm(stage: DeepstallStage, cruise_cm: i32, pre_flare_cm: i32) -> i32 {
    if matches!(stage, DeepstallStage::Approach | DeepstallStage::Land) {
        pre_flare_cm
    } else {
        cruise_cm
    }
}

/// Whether fly-to-landing may advance to estimate-wind, upstream the distance
/// check at the head of `DEEPSTALL_STAGE_FLY_TO_LANDING` in `verify_land`.
#[must_use]
pub fn fly_to_landing_may_advance(distance_to_landing_m: f32, loiter_radius_m: f32) -> bool {
    distance_to_landing_m <= libm::fabsf(2.0 * loiter_radius_m)
}

/// Accumulate loiter progress in centidegrees, upstream the delta accumulation
/// in `DEEPSTALL_STAGE_ESTIMATE_WIND` and `WAIT_FOR_BREAKOUT`.
#[must_use]
pub fn accumulate_loiter_cd(
    loiter_sum_cd: i32,
    target_bearing_cd: i32,
    last_target_bearing_cd: i32,
    loiter_ccw: bool,
) -> (i32, i32) {
    let mut delta = wrap_180_cd(target_bearing_cd - last_target_bearing_cd);
    if loiter_ccw {
        delta = -delta;
    }
    let added = if delta > 0 { delta } else { 0 };
    (loiter_sum_cd + added, target_bearing_cd)
}

/// Whether estimate-wind may advance to wait-for-breakout.
#[must_use]
pub fn estimate_wind_may_advance(
    reached_loiter: bool,
    height_error_m: f32,
    loiter_sum_cd: i32,
) -> bool {
    reached_loiter
        && libm::fabsf(height_error_m) <= LOITER_ALT_TOLERANCE_M
        && loiter_sum_cd >= LOITER_COMPLETE_CD
}

/// Whether wait-for-breakout may advance to fly-to-arc.
#[must_use]
pub fn wait_for_breakout_may_advance(heading_error_deg: f32, height_error_m: f32) -> bool {
    verify_breakout(heading_error_deg, height_error_m)
}

/// Whether fly-to-arc may advance to arc, upstream the distance check in
/// `DEEPSTALL_STAGE_FLY_TO_ARC`.
#[must_use]
pub fn fly_to_arc_may_advance(distance_to_arc_entry_m: f32, loiter_radius_m: f32) -> bool {
    distance_to_arc_entry_m <= libm::fabsf(2.0 * loiter_radius_m)
}

/// Heading alignment margin for arc completion, upstream the 10° check in
/// `DEEPSTALL_STAGE_ARC`.
pub const ARC_HEADING_MARGIN_DEG: f32 = 10.0;

/// Groundspeed heading in degrees, upstream `degrees(atan2f(-y,-x)+PI)`.
#[must_use]
pub fn groundspeed_heading_deg(groundspeed_ne: Vector2f) -> f32 {
    degrees(libm::atan2f(-groundspeed_ne.y, -groundspeed_ne.x) + core::f32::consts::PI)
}

/// Heading error during the arc stage.
#[must_use]
pub fn arc_heading_error_deg(target_heading_deg: f32, groundspeed_ne: Vector2f) -> f32 {
    wrap_180(target_heading_deg - groundspeed_heading_deg(groundspeed_ne))
}

/// Whether arc may advance to approach, upstream `DEEPSTALL_STAGE_ARC`.
#[must_use]
pub fn arc_may_advance(
    reached_loiter: bool,
    target_heading_deg: f32,
    groundspeed_ne: Vector2f,
) -> bool {
    reached_loiter
        && libm::fabsf(arc_heading_error_deg(target_heading_deg, groundspeed_ne))
            < ARC_HEADING_MARGIN_DEG
}

/// Approach altitude offset from a mission command, upstream `do_land`.
#[must_use]
pub fn deepstall_approach_alt_offset_m(landing: Location, cmd_p1_m: f32) -> f32 {
    if landing.alt_frame() == AltFrame::Absolute {
        cmd_p1_m
    } else {
        0.0
    }
}

/// Next stage after an approach finish-line check, upstream the transitions in
/// `DEEPSTALL_STAGE_APPROACH`.
#[must_use]
pub fn deepstall_stage_after_approach(advance: ApproachAdvance) -> Option<DeepstallStage> {
    match advance {
        ApproachAdvance::Continue => None,
        ApproachAdvance::RecoverFlyToLanding => Some(DeepstallStage::FlyToLanding),
        ApproachAdvance::AdvanceToLand => Some(DeepstallStage::Land),
    }
}

/// Outcome of the approach finish-line checks in `DEEPSTALL_STAGE_APPROACH`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachAdvance {
    /// Still on the approach segment — keep navigating to the entry point.
    Continue,
    /// Passed the extended approach but not the entry point — reset upstream.
    RecoverFlyToLanding,
    /// Passed the predicted stall entry point — advance to land.
    AdvanceToLand,
}

/// Whether the approach stage may advance, upstream `DEEPSTALL_STAGE_APPROACH`.
#[must_use]
pub fn approach_advance(
    current: Location,
    arc_exit: Location,
    entry_point: Location,
    extended_approach: Location,
) -> ApproachAdvance {
    if !current.past_interval_finish_line(arc_exit, entry_point) {
        if current.past_interval_finish_line(arc_exit, extended_approach) {
            return ApproachAdvance::RecoverFlyToLanding;
        }
        return ApproachAdvance::Continue;
    }
    ApproachAdvance::AdvanceToLand
}

/// Height above the landing point for travel prediction, upstream the
/// `height_above_target` block in `DEEPSTALL_STAGE_APPROACH`.
#[must_use]
pub fn approach_height_above_target_m(
    approach_alt_offset_m: f32,
    position_alt_cm: Option<i32>,
    landing_point_alt_cm: i32,
    relative_d_home_m: Option<f32>,
) -> f32 {
    if is_zero(approach_alt_offset_m) {
        relative_d_home_m.map(|d| -d).unwrap_or(0.0)
    } else if let Some(alt) = position_alt_cm {
        (alt - landing_point_alt_cm) as f32 * 0.01 + approach_alt_offset_m
    } else {
        approach_alt_offset_m
    }
}

/// Persistent state carried across `verify_land` calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeepstallVerifyState {
    pub stage: DeepstallStage,
    pub loiter_sum_cd: i32,
    pub last_target_bearing_cd: i32,
}

impl Default for DeepstallVerifyState {
    fn default() -> Self {
        Self {
            stage: DeepstallStage::INITIAL,
            loiter_sum_cd: 0,
            last_target_bearing_cd: 0,
        }
    }
}

/// Measurements supplied each `verify_land` tick.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallVerifyInputs {
    pub distance_to_landing_m: f32,
    pub distance_to_arc_entry_m: f32,
    pub loiter_radius_m: f32,
    pub loiter_ccw: bool,
    pub reached_loiter: bool,
    pub height_error_m: f32,
    pub target_bearing_cd: i32,
    pub heading_error_deg: f32,
    pub target_heading_deg: f32,
    pub groundspeed_ne: Vector2f,
    pub current: Location,
    pub arc_exit: Location,
    pub arc_entry: Location,
    pub extended_approach: Location,
    pub entry_point: Location,
}

/// Side effects the vehicle should perform after a step.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DeepstallVerifyEffects {
    pub rebuild_approach_path: bool,
    pub record_breakout_at_current: bool,
    pub entered_land: bool,
}

/// Result of one `verify_land` tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeepstallVerifyStep {
    pub state: DeepstallVerifyState,
    pub effects: DeepstallVerifyEffects,
}

/// Advance the deepstall stage machine one tick, upstream
/// `AP_Landing_Deepstall::verify_land`.
#[must_use]
pub fn deepstall_verify_land_step(
    state: DeepstallVerifyState,
    inp: &DeepstallVerifyInputs,
) -> DeepstallVerifyStep {
    let mut effects = DeepstallVerifyEffects::default();
    let mut stage = state.stage;
    let mut loiter_sum_cd = state.loiter_sum_cd;
    let mut last_target_bearing_cd = state.last_target_bearing_cd;

    match stage {
        DeepstallStage::FlyToLanding => {
            if fly_to_landing_may_advance(inp.distance_to_landing_m, inp.loiter_radius_m) {
                stage = DeepstallStage::EstimateWind;
                loiter_sum_cd = 0;
            }
        }
        DeepstallStage::EstimateWind => {
            if inp.reached_loiter && libm::fabsf(inp.height_error_m) <= LOITER_ALT_TOLERANCE_M {
                let (sum, last) = accumulate_loiter_cd(
                    loiter_sum_cd,
                    inp.target_bearing_cd,
                    last_target_bearing_cd,
                    inp.loiter_ccw,
                );
                loiter_sum_cd = sum;
                last_target_bearing_cd = last;
            }
            if estimate_wind_may_advance(inp.reached_loiter, inp.height_error_m, loiter_sum_cd) {
                stage = DeepstallStage::WaitForBreakout;
                loiter_sum_cd = 0;
            }
        }
        DeepstallStage::WaitForBreakout => {
            if loiter_sum_cd < LOITER_COMPLETE_CD {
                effects.rebuild_approach_path = true;
            }
            if wait_for_breakout_may_advance(inp.heading_error_deg, inp.height_error_m) {
                stage = DeepstallStage::FlyToArc;
                effects.record_breakout_at_current = true;
            } else {
                let (sum, last) = accumulate_loiter_cd(
                    loiter_sum_cd,
                    inp.target_bearing_cd,
                    last_target_bearing_cd,
                    false,
                );
                loiter_sum_cd = sum;
                last_target_bearing_cd = last;
            }
        }
        DeepstallStage::FlyToArc => {
            if fly_to_arc_may_advance(inp.distance_to_arc_entry_m, inp.loiter_radius_m) {
                stage = DeepstallStage::Arc;
            }
        }
        DeepstallStage::Arc => {
            if arc_may_advance(inp.reached_loiter, inp.target_heading_deg, inp.groundspeed_ne) {
                stage = DeepstallStage::Approach;
            }
        }
        DeepstallStage::Approach => {
            if let Some(next) = deepstall_stage_after_approach(approach_advance(
                inp.current,
                inp.arc_exit,
                inp.entry_point,
                inp.extended_approach,
            )) {
                stage = next;
                if next == DeepstallStage::Land {
                    effects.entered_land = true;
                }
            }
        }
        DeepstallStage::Land => {}
    }

    DeepstallVerifyStep {
        state: DeepstallVerifyState {
            stage,
            loiter_sum_cd,
            last_target_bearing_cd,
        },
        effects,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discriminants_match_mavlink() {
        assert_eq!(DeepstallStage::FlyToLanding as u8, 0);
        assert_eq!(DeepstallStage::Land as u8, 6);
    }

    #[test]
    fn throttle_suppressed_only_in_land() {
        assert!(!is_throttle_suppressed(DeepstallStage::Approach));
        assert!(is_throttle_suppressed(DeepstallStage::Land));
    }

    #[test]
    fn fly_to_landing_advances_within_twice_loiter_radius() {
        assert!(!fly_to_landing_may_advance(250.0, 100.0));
        assert!(fly_to_landing_may_advance(200.0, 100.0));
    }

    #[test]
    fn loiter_accumulation_counts_forward_turns_only() {
        let (sum, last) = accumulate_loiter_cd(0, 1000, 0, false);
        assert_eq!(sum, 1000);
        assert_eq!(last, 1000);
        let (sum, _) = accumulate_loiter_cd(sum, 900, last, false);
        assert_eq!(sum, 1000, "backward delta ignored");
    }

    #[test]
    fn estimate_wind_requires_loiter_and_altitude() {
        assert!(!estimate_wind_may_advance(true, 6.0, LOITER_COMPLETE_CD));
        assert!(estimate_wind_may_advance(true, 2.0, LOITER_COMPLETE_CD));
        assert!(!estimate_wind_may_advance(true, 2.0, LOITER_COMPLETE_CD - 1));
    }

    #[test]
    fn fly_to_arc_matches_fly_to_landing_radius_rule() {
        assert!(fly_to_arc_may_advance(150.0, 100.0));
        assert!(!fly_to_arc_may_advance(250.0, 100.0));
    }

    #[test]
    fn arc_requires_loiter_and_heading_alignment() {
        let gs = Vector2f::new(10.0, 0.0);
        assert!(!arc_may_advance(false, 0.0, gs));
        assert!(arc_may_advance(true, 0.0, gs));
        assert!(!arc_may_advance(true, 0.0, Vector2f::new(0.0, 10.0)));
    }

    #[test]
    fn approach_height_uses_home_when_offset_zero() {
        assert!((approach_height_above_target_m(0.0, None, 0, Some(-50.0)) - 50.0).abs() < 1e-3);
    }

    #[test]
    fn approach_alt_offset_only_for_absolute_frame() {
        let abs = Location::new(-35_000_000, 149_000_000);
        assert!((deepstall_approach_alt_offset_m(abs, 12.0) - 12.0).abs() < 1e-6);
        let rel = Location::new_with_alt(-35_000_000, 149_000_000, 100, AltFrame::AboveHome);
        assert!(is_zero(deepstall_approach_alt_offset_m(rel, 12.0)));
    }

    #[test]
    fn stage_after_approach_maps_outcomes() {
        assert_eq!(
            deepstall_stage_after_approach(ApproachAdvance::AdvanceToLand),
            Some(DeepstallStage::Land)
        );
        assert_eq!(
            deepstall_stage_after_approach(ApproachAdvance::RecoverFlyToLanding),
            Some(DeepstallStage::FlyToLanding)
        );
        assert!(deepstall_stage_after_approach(ApproachAdvance::Continue).is_none());
    }

    #[test]
    fn approach_advance_waits_before_entry_point() {
        let arc_exit = Location::new(-35_000_000, 149_000_000);
        let mut entry = arc_exit;
        entry.offset_bearing(0.0, 200.0);
        let mut extended = arc_exit;
        extended.offset_bearing(0.0, 400.0);
        let mut mid = arc_exit;
        mid.offset_bearing(0.0, 100.0);
        assert_eq!(
            approach_advance(mid, arc_exit, entry, extended),
            ApproachAdvance::Continue
        );
    }

    #[test]
    fn approach_advance_recovers_on_flyaway() {
        let arc_exit = Location::new(-35_000_000, 149_000_000);
        let mut extended = arc_exit;
        extended.offset_bearing(0.0, 200.0);
        let mut entry = arc_exit;
        entry.offset_bearing(0.0, 400.0);
        let mut past_extended = arc_exit;
        past_extended.offset_bearing(0.0, 250.0);
        assert_eq!(
            approach_advance(past_extended, arc_exit, entry, extended),
            ApproachAdvance::RecoverFlyToLanding
        );
    }

    #[test]
    fn approach_advance_to_land_past_entry() {
        let arc_exit = Location::new(-35_000_000, 149_000_000);
        let mut entry = arc_exit;
        entry.offset_bearing(0.0, 200.0);
        let mut extended = arc_exit;
        extended.offset_bearing(0.0, 400.0);
        let mut past_entry = arc_exit;
        past_entry.offset_bearing(0.0, 250.0);
        assert_eq!(
            approach_advance(past_entry, arc_exit, entry, extended),
            ApproachAdvance::AdvanceToLand
        );
    }

    fn sample_verify_inputs() -> DeepstallVerifyInputs {
        DeepstallVerifyInputs {
            distance_to_landing_m: 50.0,
            distance_to_arc_entry_m: 150.0,
            loiter_radius_m: 100.0,
            loiter_ccw: false,
            reached_loiter: true,
            height_error_m: 1.0,
            target_bearing_cd: 500,
            heading_error_deg: 5.0,
            target_heading_deg: 0.0,
            groundspeed_ne: Vector2f::new(10.0, 0.0),
            current: Location::new(-35_000_000, 149_000_000),
            arc_exit: Location::new(-35_000_000, 149_000_000),
            arc_entry: Location::new(-35_000_000, 149_000_000),
            extended_approach: Location::new(-35_000_000, 149_000_000),
            entry_point: Location::new(-35_000_000, 149_000_000),
        }
    }

    #[test]
    fn verify_land_advances_fly_to_landing_near_target() {
        let state = DeepstallVerifyState::default();
        let mut inp = sample_verify_inputs();
        inp.distance_to_landing_m = 150.0;
        let step = deepstall_verify_land_step(state, &inp);
        assert_eq!(step.state.stage, DeepstallStage::EstimateWind);
        assert_eq!(step.state.loiter_sum_cd, 0);
    }

    #[test]
    fn verify_land_wind_loiter_then_breakout() {
        let mut state = DeepstallVerifyState {
            stage: DeepstallStage::EstimateWind,
            loiter_sum_cd: LOITER_COMPLETE_CD - 100,
            last_target_bearing_cd: 0,
        };
        let inp = sample_verify_inputs();
        let step = deepstall_verify_land_step(state, &inp);
        assert_eq!(step.state.stage, DeepstallStage::WaitForBreakout);
        state = step.state;
        let step = deepstall_verify_land_step(state, &inp);
        assert_eq!(step.state.stage, DeepstallStage::FlyToArc);
        assert!(step.effects.record_breakout_at_current);
    }

    #[test]
    fn verify_land_breakout_rebuilds_path_until_one_loiter() {
        let state = DeepstallVerifyState {
            stage: DeepstallStage::WaitForBreakout,
            loiter_sum_cd: 0,
            last_target_bearing_cd: 0,
        };
        let mut inp = sample_verify_inputs();
        inp.heading_error_deg = 30.0;
        inp.height_error_m = 20.0;
        let step = deepstall_verify_land_step(state, &inp);
        assert!(step.effects.rebuild_approach_path);
        assert_eq!(step.state.stage, DeepstallStage::WaitForBreakout);
    }
}
