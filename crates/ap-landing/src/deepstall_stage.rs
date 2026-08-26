//! Deepstall landing stage machine, upstream `AP_Landing_Deepstall::verify_land`.
//!
//! The stages mirror mavlink `DEEPSTALL_STAGE` because they are logged. Query
//! helpers and transition predicates are pure: the caller supplies distances
//! and geometry; nothing here touches nav controllers or AHRS.

use ap_math::scalar::wrap_180_cd;

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
}
