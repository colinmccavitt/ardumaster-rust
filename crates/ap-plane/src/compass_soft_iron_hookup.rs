//! Compass soft-iron stub, upstream `COMPASS_DIA` / `COMPASS_ODI`.
//!
//! Frontend correction is the elliptical matrix from DIA/ODI when DIA is
//! non-zero. Default DIA is identity and ODI is zero.

use ap_compass::soft_iron::have_diagonals;
use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the soft-iron matrix applied to the primary instance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassSoftIronOutput {
    /// Primary `COMPASS_DIA`.
    pub diagonals: Vector3f,
    /// Primary `COMPASS_ODI`.
    pub offdiagonals: Vector3f,
    /// True when DIA is non-zero so the matrix is applied.
    pub applied: bool,
}

/// Report the DIA/ODI that `apply_soft_iron` will use on the next publish.
#[must_use]
pub fn compass_soft_iron_tick(hookup: &SitlCompassHookup) -> CompassSoftIronOutput {
    let params = hookup.compass_params();
    let inst = if params.primary == 0 {
        params.compass1
    } else {
        params.compass2
    };
    CompassSoftIronOutput {
        diagonals: inst.diagonals,
        offdiagonals: inst.offdiagonals,
        applied: have_diagonals(inst.diagonals),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::params::CompassParams;
    use ap_compass::soft_iron::{COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT};
    use ap_math::vector3::Vector3f;

    #[test]
    fn default_is_identity_applied() {
        let hookup = SitlCompassHookup::default();
        let out = compass_soft_iron_tick(&hookup);
        assert!((out.diagonals.x - COMPASS_DIA_DEFAULT.x).abs() < 1e-6);
        assert!((out.offdiagonals.x - COMPASS_ODI_DEFAULT.x).abs() < 1e-6);
        assert!(out.applied);
    }

    #[test]
    fn reports_configured_matrix() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.compass1.diagonals = Vector3f::new(1.1, 0.9, 1.0);
        params.compass1.offdiagonals = Vector3f::new(0.05, 0.0, 0.0);
        hookup.apply_compass_params(params);
        let out = compass_soft_iron_tick(&hookup);
        assert!((out.diagonals.x - 1.1).abs() < 1e-6);
        assert!((out.offdiagonals.x - 0.05).abs() < 1e-6);
        assert!(out.applied);
    }
}
