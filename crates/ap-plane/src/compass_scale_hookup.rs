//! Compass scale factor stub, upstream `COMPASS_SCALE`.
//!
//! Frontend correction is `mag *= COMPASS_SCALE` when the factor is inside
//! `[COMPASS_MIN_SCALE_FACTOR, COMPASS_MAX_SCALE_FACTOR]`. Default 0 is a no-op.

use ap_compass::scale::have_scale_factor;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the scale applied to the primary instance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassScaleOutput {
    /// Primary `COMPASS_SCALE`.
    pub scale: f32,
    /// True when the factor is inside the sanity range.
    pub applied: bool,
}

/// Report the scale that `apply_scale` will use on the next publish.
#[must_use]
pub fn compass_scale_tick(hookup: &SitlCompassHookup) -> CompassScaleOutput {
    let params = hookup.compass_params();
    let inst = if params.primary == 0 {
        params.compass1
    } else {
        params.compass2
    };
    CompassScaleOutput {
        scale: inst.scale,
        applied: have_scale_factor(inst.scale),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::params::CompassParams;
    use ap_compass::scale::COMPASS_SCALE_DEFAULT;

    #[test]
    fn default_is_unapplied_zero() {
        let hookup = SitlCompassHookup::default();
        let out = compass_scale_tick(&hookup);
        assert!((out.scale - COMPASS_SCALE_DEFAULT).abs() < 1e-6);
        assert!(!out.applied);
    }

    #[test]
    fn reports_applied_in_range_scale() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.compass1.scale = 1.1;
        hookup.apply_compass_params(params);
        let out = compass_scale_tick(&hookup);
        assert!((out.scale - 1.1).abs() < 1e-6);
        assert!(out.applied);
    }
}
