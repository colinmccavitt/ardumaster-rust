//! Deepstall landing stage machine, upstream `AP_Landing_Deepstall::verify_land`.
//!
//! The stages mirror mavlink `DEEPSTALL_STAGE` because they are logged. Query
//! helpers and transition predicates are pure: the caller supplies distances
//! and geometry; nothing here touches nav controllers or AHRS.

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
///
/// Upstream waits while `distance > abs(2 * loiter_radius)`; this returns
/// true once the aircraft is within that radius.
#[must_use]
pub fn fly_to_landing_may_advance(distance_to_landing_m: f32, loiter_radius_m: f32) -> bool {
    distance_to_landing_m <= libm::fabsf(2.0 * loiter_radius_m)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discriminants_match_mavlink() {
        assert_eq!(DeepstallStage::FlyToLanding as u8, 0);
        assert_eq!(DeepstallStage::EstimateWind as u8, 1);
        assert_eq!(DeepstallStage::WaitForBreakout as u8, 2);
        assert_eq!(DeepstallStage::FlyToArc as u8, 3);
        assert_eq!(DeepstallStage::Arc as u8, 4);
        assert_eq!(DeepstallStage::Approach as u8, 5);
        assert_eq!(DeepstallStage::Land as u8, 6);
    }

    #[test]
    fn throttle_suppressed_only_in_land() {
        assert!(!is_throttle_suppressed(DeepstallStage::Approach));
        assert!(is_throttle_suppressed(DeepstallStage::Land));
    }

    #[test]
    fn flying_forward_everywhere_but_land() {
        assert!(is_flying_forward(DeepstallStage::FlyToLanding));
        assert!(!is_flying_forward(DeepstallStage::Land));
    }

    #[test]
    fn on_approach_only_in_land_stage() {
        assert!(!is_on_approach(DeepstallStage::Approach));
        assert!(is_on_approach(DeepstallStage::Land));
    }

    #[test]
    fn target_airspeed_uses_pre_flare_on_approach_and_land() {
        let cruise = 2200;
        let pre_flare = 1800;
        assert_eq!(
            target_airspeed_cm(DeepstallStage::EstimateWind, cruise, pre_flare),
            cruise
        );
        assert_eq!(
            target_airspeed_cm(DeepstallStage::Approach, cruise, pre_flare),
            pre_flare
        );
        assert_eq!(
            target_airspeed_cm(DeepstallStage::Land, cruise, pre_flare),
            pre_flare
        );
    }

    #[test]
    fn fly_to_landing_advances_within_twice_loiter_radius() {
        assert!(!fly_to_landing_may_advance(250.0, 100.0));
        assert!(fly_to_landing_may_advance(200.0, 100.0));
        assert!(fly_to_landing_may_advance(200.0, -100.0));
    }
}
