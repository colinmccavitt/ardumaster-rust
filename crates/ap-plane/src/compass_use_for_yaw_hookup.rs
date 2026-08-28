//! Disable use-for-yaw when compasses are inconsistent.
//!
//! Upstream AHRS/EKF drop mag-for-yaw when `!Compass::consistent()`.
//! `COMPASS_USE` is left as configured; only the runtime yaw gate changes.

use crate::compass_consistent_hookup::compass_consistent_tick;
use crate::sitl_compass_hookup::SitlCompassHookup;
use ap_compass::consistent::use_for_yaw_if_consistent;

/// Snapshot of the consistency-gated `use_for_yaw` flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassUseForYawOutput {
    /// Configured `COMPASS_USE` on the primary instance.
    pub configured: bool,
    /// Upstream `Compass::consistent()`.
    pub consistent: bool,
    /// Runtime `use_for_yaw` after the consistency gate.
    pub use_for_yaw: bool,
}

/// Gate `SitlCompassHookup::compass_use_for_yaw` when instances disagree.
#[must_use]
pub fn compass_use_for_yaw_tick(hookup: &mut SitlCompassHookup) -> CompassUseForYawOutput {
    let configured = hookup.compass_params().primary_use_for_yaw();
    let consistent = compass_consistent_tick(hookup).consistent;
    let use_for_yaw = use_for_yaw_if_consistent(configured, consistent);
    hookup.compass_use_for_yaw = use_for_yaw;
    CompassUseForYawOutput {
        configured,
        consistent,
        use_for_yaw,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};
    use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
    use ap_compass::params::CompassParams;
    use ap_math::matrix3::Matrix3f;

    #[test]
    fn unpublished_zero_field_disables_yaw() {
        let mut hookup = SitlCompassHookup::default();
        let out = compass_use_for_yaw_tick(&mut hookup);
        assert!(out.configured);
        assert!(!out.consistent);
        assert!(!out.use_for_yaw);
        assert!(!hookup.compass_use_for_yaw);
    }

    #[test]
    fn published_single_instance_keeps_yaw() {
        let mut hookup = SitlCompassHookup::default();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        let out = compass_use_for_yaw_tick(&mut hookup);
        assert!(out.consistent);
        assert!(out.use_for_yaw);
        assert!(hookup.compass_use_for_yaw);
    }

    #[test]
    fn configured_off_stays_off_when_consistent() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.compass1.use_for_yaw = false;
        hookup.apply_compass_params(params);
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        let out = compass_use_for_yaw_tick(&mut hookup);
        assert!(!out.configured);
        assert!(out.consistent);
        assert!(!out.use_for_yaw);
    }

    #[test]
    fn yawed_secondary_disables_yaw() {
        let mut hookup = SitlCompassHookup::with_dual_backends();
        let mut params = CompassParams::default();
        params.compass2.orientation = COMPASS_ORIENT_YAW_90;
        hookup.apply_compass_params(params);
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let first = hookup.publish(Matrix3f::identity(), 0.0025, None);
        assert!(first.yaw_compass.is_some());
        let out = compass_use_for_yaw_tick(&mut hookup);
        assert!(!out.consistent);
        assert!(!out.use_for_yaw);
        let second = hookup.publish(Matrix3f::identity(), 0.0025, None);
        assert!(second.yaw_compass.is_none());
    }
}
