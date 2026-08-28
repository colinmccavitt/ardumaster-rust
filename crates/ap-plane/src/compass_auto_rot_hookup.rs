//! Compass auto-orientation stub, upstream `COMPASS_AUTO_ROT`.
//!
//! Reports `_rotate_auto` and applies a detected orientation to an external
//! compass when the mode is CheckAndFix / CheckAndFix45.

use ap_compass::auto_rot::{
    accept_detected_orientation, always_45_deg, check_enabled, fix_orientation,
    settings_for_start, AutoRot, AutoRotSettings,
};

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of `COMPASS_AUTO_ROT` on the frontend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassAutoRotOutput {
    /// Raw `COMPASS_AUTO_ROT` parameter.
    pub rotate_auto: u8,
    /// Typed mode when the value is a known variant.
    pub mode: Option<AutoRot>,
    /// True when MAG_CAL will call `set_orientation`.
    pub check_enabled: bool,
    /// True when a successful cal may rewrite `COMPASS_ORIENT`.
    pub fix_orientation: bool,
    /// True when 45-degree candidates are included.
    pub always_45_deg: bool,
}

/// Report the auto-orientation mode MAG_CAL will honor.
#[must_use]
pub fn compass_auto_rot_tick(hookup: &SitlCompassHookup) -> CompassAutoRotOutput {
    let rotate_auto = hookup.compass_params().rotate_auto;
    CompassAutoRotOutput {
        rotate_auto,
        mode: AutoRot::from_u8(rotate_auto),
        check_enabled: check_enabled(rotate_auto),
        fix_orientation: fix_orientation(rotate_auto),
        always_45_deg: always_45_deg(rotate_auto),
    }
}

/// `set_orientation` settings for one instance at MAG_CAL start.
#[must_use]
pub fn compass_auto_rot_start_settings(
    hookup: &SitlCompassHookup,
    instance: u8,
) -> Option<AutoRotSettings> {
    let params = hookup.compass_params();
    let inst = if instance == 0 {
        params.compass1
    } else {
        params.compass2
    };
    settings_for_start(params.rotate_auto, inst.orientation, inst.external)
}

/// Persist a detected orientation when CheckAndFix and the instance is external.
#[must_use]
pub fn apply_auto_orientation(
    hookup: &mut SitlCompassHookup,
    instance: u8,
    detected: u8,
) -> bool {
    let params = *hookup.compass_params();
    let external = if instance == 0 {
        params.compass1.external
    } else if instance == 1 {
        params.compass2.external
    } else {
        return false;
    };
    let Some(orient) = accept_detected_orientation(params.rotate_auto, external, detected) else {
        return false;
    };
    let mut next = params;
    if instance == 0 {
        next.compass1.orientation = orient;
    } else {
        next.compass2.orientation = orient;
    }
    hookup.apply_compass_params(next);
    true
}

/// Whether `_accept_calibration` would save `detected` on this instance.
#[must_use]
pub fn would_save_detected(hookup: &SitlCompassHookup, instance: u8, detected: u8) -> bool {
    let params = hookup.compass_params();
    let external = if instance == 0 {
        params.compass1.external
    } else {
        params.compass2.external
    };
    accept_detected_orientation(params.rotate_auto, external, detected).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::auto_rot::{
        COMPASS_AUTO_ROT_CHECK_AND_FIX, COMPASS_AUTO_ROT_CHECK_ONLY, COMPASS_AUTO_ROT_DEFAULT,
        COMPASS_AUTO_ROT_DISABLED,
    };
    use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
    use ap_compass::params::CompassParams;

    #[test]
    fn default_is_check_and_fix() {
        let hookup = SitlCompassHookup::default();
        let out = compass_auto_rot_tick(&hookup);
        assert_eq!(out.rotate_auto, COMPASS_AUTO_ROT_DEFAULT);
        assert_eq!(out.mode, Some(AutoRot::CheckAndFix));
        assert!(out.check_enabled);
        assert!(out.fix_orientation);
        assert!(!out.always_45_deg);
    }

    #[test]
    fn check_only_does_not_save() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.rotate_auto = COMPASS_AUTO_ROT_CHECK_ONLY;
        params.compass1.external = true;
        hookup.apply_compass_params(params);
        let out = compass_auto_rot_tick(&hookup);
        assert_eq!(out.mode, Some(AutoRot::CheckOnly));
        assert!(out.check_enabled);
        assert!(!out.fix_orientation);
        assert!(!would_save_detected(&hookup, 0, COMPASS_ORIENT_YAW_90));
        assert!(!apply_auto_orientation(
            &mut hookup,
            0,
            COMPASS_ORIENT_YAW_90
        ));
    }

    #[test]
    fn disabled_has_no_start_settings() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.rotate_auto = COMPASS_AUTO_ROT_DISABLED;
        hookup.apply_compass_params(params);
        assert!(compass_auto_rot_start_settings(&hookup, 0).is_none());
        assert_eq!(
            compass_auto_rot_tick(&hookup).mode,
            Some(AutoRot::Disabled)
        );
    }

    #[test]
    fn check_and_fix_saves_external() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.rotate_auto = COMPASS_AUTO_ROT_CHECK_AND_FIX;
        params.compass1.external = true;
        hookup.apply_compass_params(params);
        assert!(would_save_detected(&hookup, 0, COMPASS_ORIENT_YAW_90));
        assert!(apply_auto_orientation(
            &mut hookup,
            0,
            COMPASS_ORIENT_YAW_90
        ));
        assert_eq!(
            hookup.compass_params().compass1.orientation,
            COMPASS_ORIENT_YAW_90
        );
    }
}
