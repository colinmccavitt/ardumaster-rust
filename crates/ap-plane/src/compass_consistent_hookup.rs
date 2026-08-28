//! Compass consistency stub, upstream `Compass::consistent()`.
//!
//! Compares each `COMPASS_USE` instance field to the primary sample.

use ap_compass::consistent::{consistent, CompassInstanceField};
use ap_compass::sitl::SITL_COMPASS_MAX_INSTANCES;
use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the multi-instance consistency check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassConsistentOutput {
    /// Upstream `Compass::consistent()`.
    pub consistent: bool,
    /// Registered instance count.
    pub instance_count: u8,
    /// Number of `COMPASS_USE` instances compared to primary.
    pub checked: u8,
}

/// Run `Compass::consistent()` against the current cluster samples.
#[must_use]
pub fn compass_consistent_tick(hookup: &SitlCompassHookup) -> CompassConsistentOutput {
    let params = hookup.compass_params();
    let cluster = hookup.cluster();
    let primary_idx = cluster.primary();
    let primary_field = cluster
        .backend(primary_idx)
        .map(|backend| backend.state().mag_body)
        .unwrap_or_else(Vector3f::zero);

    let mut instances = [CompassInstanceField {
        field: Vector3f::zero(),
        use_for_yaw: false,
    }; SITL_COMPASS_MAX_INSTANCES];
    let count = cluster.instance_count() as usize;
    let mut checked = 0u8;
    for i in 0..count {
        let field = cluster
            .backend(i as u8)
            .map(|backend| backend.state().mag_body)
            .unwrap_or_else(Vector3f::zero);
        let use_for_yaw = if i == 0 {
            params.compass1.use_for_yaw
        } else {
            params.compass2.use_for_yaw
        };
        if use_for_yaw {
            checked = checked.saturating_add(1);
        }
        instances[i] = CompassInstanceField { field, use_for_yaw };
    }

    CompassConsistentOutput {
        consistent: consistent(primary_field, &instances[..count]),
        instance_count: cluster.instance_count(),
        checked,
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
    fn unpublished_zero_field_is_inconsistent() {
        let hookup = SitlCompassHookup::default();
        let out = compass_consistent_tick(&hookup);
        assert_eq!(out.instance_count, 1);
        assert_eq!(out.checked, 1);
        assert!(!out.consistent);
    }

    #[test]
    fn single_published_instance_is_consistent() {
        let mut hookup = SitlCompassHookup::default();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        let out = compass_consistent_tick(&hookup);
        assert!(out.consistent);
        assert_eq!(out.checked, 1);
    }

    #[test]
    fn yawed_secondary_is_inconsistent() {
        let mut hookup = SitlCompassHookup::with_dual_backends();
        let mut params = CompassParams::default();
        params.compass2.orientation = COMPASS_ORIENT_YAW_90;
        hookup.apply_compass_params(params);
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        let out = compass_consistent_tick(&hookup);
        assert_eq!(out.instance_count, 2);
        assert_eq!(out.checked, 2);
        assert!(!out.consistent);
    }
}
