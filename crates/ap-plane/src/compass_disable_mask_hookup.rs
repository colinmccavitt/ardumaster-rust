//! Compass disable-mask stub, upstream `COMPASS_DISBLMSK`.
//!
//! `_driver_type_mask` gates driver probe. Masking `DRIVER_SITL` marks every
//! SITL instance disabled so `healthy()` stays false.

use ap_compass::disable_mask::{instance_disabled, sitl_enabled, DriverType};

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of `COMPASS_DISBLMSK` on the frontend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassDisableMaskOutput {
    /// Raw `COMPASS_DISBLMSK` / `_driver_type_mask`.
    pub disable_mask: u32,
    /// True when `DRIVER_SITL` is not masked.
    pub sitl_enabled: bool,
    /// Primary instance disabled by mask or `compass1.disabled`.
    pub primary_disabled: bool,
}

/// Report whether SITL instances are disabled by `COMPASS_DISBLMSK`.
#[must_use]
pub fn compass_disable_mask_tick(hookup: &SitlCompassHookup) -> CompassDisableMaskOutput {
    let params = hookup.compass_params();
    let disable_mask = params.disable_mask;
    CompassDisableMaskOutput {
        disable_mask,
        sitl_enabled: sitl_enabled(disable_mask),
        primary_disabled: instance_disabled(disable_mask, params.compass1.disabled),
    }
}

/// Apply `COMPASS_DISBLMSK` and push it onto the SITL cluster.
pub fn apply_disable_mask(hookup: &mut SitlCompassHookup, disable_mask: u32) {
    let mut params = *hookup.compass_params();
    params.disable_mask = disable_mask;
    hookup.apply_compass_params(params);
}

/// Convenience: mask `DRIVER_SITL` so no SITL instance is probed.
pub fn disable_sitl_driver(hookup: &mut SitlCompassHookup) {
    apply_disable_mask(hookup, DriverType::Sitl.mask_bit());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::disable_mask::COMPASS_DISBLMSK_DEFAULT;

    #[test]
    fn default_mask_keeps_sitl_enabled() {
        let hookup = SitlCompassHookup::default();
        let out = compass_disable_mask_tick(&hookup);
        assert_eq!(out.disable_mask, COMPASS_DISBLMSK_DEFAULT);
        assert!(out.sitl_enabled);
        assert!(!out.primary_disabled);
        assert!(!hookup.backend().expect("backend").config().disabled);
    }

    #[test]
    fn sitl_bit_disables_backend() {
        let mut hookup = SitlCompassHookup::default();
        disable_sitl_driver(&mut hookup);
        let out = compass_disable_mask_tick(&hookup);
        assert_eq!(out.disable_mask, DriverType::Sitl.mask_bit());
        assert!(!out.sitl_enabled);
        assert!(out.primary_disabled);
        assert!(hookup.backend().expect("backend").config().disabled);
    }
}
