//! Compass external / orientation stub, upstream `COMPASS_ORIENT` / `COMPASS_EXTERNAL`.
//!
//! Instance orientation always applies. Internal compasses also apply AHRS
//! board orientation; `COMPASS_EXTERNAL=1` skips it.

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the orientation applied to the primary instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassOrientationOutput {
    /// Primary `COMPASS_ORIENT`.
    pub orientation: u8,
    /// Primary `COMPASS_EXTERNAL`.
    pub external: bool,
    /// AHRS board orientation applied to internal compasses.
    pub board_orientation: u8,
}

/// Report the orientation that `rotate_field` will apply on the next publish.
#[must_use]
pub fn compass_orientation_tick(hookup: &SitlCompassHookup) -> CompassOrientationOutput {
    let params = hookup.compass_params();
    let inst = if params.primary == 0 {
        params.compass1
    } else {
        params.compass2
    };
    CompassOrientationOutput {
        orientation: inst.orientation,
        external: inst.external,
        board_orientation: params.board_orientation,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::orientation::{
        COMPASS_EXTERNAL_DEFAULT, COMPASS_ORIENT_DEFAULT, COMPASS_ORIENT_YAW_90,
    };
    use ap_compass::params::CompassParams;

    #[test]
    fn defaults_are_internal_none() {
        let hookup = SitlCompassHookup::default();
        let out = compass_orientation_tick(&hookup);
        assert_eq!(out.orientation, COMPASS_ORIENT_DEFAULT);
        assert_eq!(out.external, COMPASS_EXTERNAL_DEFAULT);
        assert_eq!(out.board_orientation, COMPASS_ORIENT_DEFAULT);
    }

    #[test]
    fn reports_applied_orient_and_external() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.compass1.orientation = COMPASS_ORIENT_YAW_90;
        params.compass1.external = true;
        hookup.apply_compass_params(params);
        let out = compass_orientation_tick(&hookup);
        assert_eq!(out.orientation, COMPASS_ORIENT_YAW_90);
        assert!(out.external);
    }
}
