//! Compass primary instance selection stub, upstream `Compass::get_first_usable`.
//!
//! `_first_usable` is the first `COMPASS_USE` instance. The SITL cluster
//! primary is pointed at that index so `get_field()` follows the frontend.

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of primary selection from `COMPASS_USE` / `USE2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassPrimaryOutput {
    /// Computed `_first_usable`.
    pub first_usable: u8,
    /// Cluster primary after selection.
    pub primary: u8,
}

/// Select the first `COMPASS_USE` instance as the frontend primary.
#[must_use]
pub fn compass_primary_tick(hookup: &mut SitlCompassHookup) -> CompassPrimaryOutput {
    let first_usable = hookup.select_first_usable();
    CompassPrimaryOutput {
        first_usable,
        primary: hookup.cluster().primary(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::params::CompassParams;

    #[test]
    fn default_selects_instance_zero() {
        let mut hookup = SitlCompassHookup::default();
        let out = compass_primary_tick(&mut hookup);
        assert_eq!(out.first_usable, 0);
        assert_eq!(out.primary, 0);
    }

    #[test]
    fn use2_only_selects_secondary() {
        let mut hookup = SitlCompassHookup::with_dual_backends();
        let mut params = CompassParams::default();
        params.compass1.use_for_yaw = false;
        hookup.apply_compass_params(params);
        let out = compass_primary_tick(&mut hookup);
        assert_eq!(out.first_usable, 1);
        assert_eq!(out.primary, 1);
    }
}
