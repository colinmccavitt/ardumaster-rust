//! Compass auto-orientation stub, upstream `COMPASS_AUTO_ROT`. FW-014.
//!
//! After a successful MAG_CAL, `_rotate_auto` decides whether the calibrator
//! checks orientation and whether an external compass may have
//! `COMPASS_ORIENT` rewritten. The sphere-fit / rotation-search solver is
//! not in this slice.

/// Upstream `COMPASS_AUTO_ROT` disabled (`0:Disabled`).
pub const COMPASS_AUTO_ROT_DISABLED: u8 = 0;
/// Upstream `1:CheckOnly` — check, do not rewrite `COMPASS_ORIENT`.
pub const COMPASS_AUTO_ROT_CHECK_ONLY: u8 = 1;
/// Upstream `2:CheckAndFix` — check and save on external compasses.
pub const COMPASS_AUTO_ROT_CHECK_AND_FIX: u8 = 2;
/// Upstream `3` — same as CheckAndFix, plus 45-degree candidates.
pub const COMPASS_AUTO_ROT_FIX_45: u8 = 3;
/// Upstream `HAL_COMPASS_AUTO_ROT_DEFAULT`.
pub const COMPASS_AUTO_ROT_DEFAULT: u8 = COMPASS_AUTO_ROT_CHECK_AND_FIX;

/// Upstream `COMPASS_AUTO_ROT` / `Compass::_rotate_auto`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoRot {
    /// `0:Disabled`.
    Disabled = 0,
    /// `1:CheckOnly`.
    CheckOnly = 1,
    /// `2:CheckAndFix`.
    CheckAndFix = 2,
    /// `3` — CheckAndFix including 45-degree rotations.
    CheckAndFix45 = 3,
}

impl AutoRot {
    /// Decode a known `COMPASS_AUTO_ROT` value.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            COMPASS_AUTO_ROT_DISABLED => Some(Self::Disabled),
            COMPASS_AUTO_ROT_CHECK_ONLY => Some(Self::CheckOnly),
            COMPASS_AUTO_ROT_CHECK_AND_FIX => Some(Self::CheckAndFix),
            COMPASS_AUTO_ROT_FIX_45 => Some(Self::CheckAndFix45),
            _ => None,
        }
    }

    /// Encode as the `COMPASS_AUTO_ROT` parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// True when `_rotate_auto` is nonzero, upstream `if (_rotate_auto)`.
    #[must_use]
    pub const fn check_enabled(self) -> bool {
        check_enabled(self.as_u8())
    }

    /// True when `_rotate_auto >= 2`, passed as `fix_orientation`.
    #[must_use]
    pub const fn fix_orientation(self) -> bool {
        fix_orientation(self.as_u8())
    }

    /// True when `_rotate_auto >= 3`, passed as `always_45_deg`.
    #[must_use]
    pub const fn always_45_deg(self) -> bool {
        always_45_deg(self.as_u8())
    }

    /// True when a successful cal may rewrite `COMPASS_ORIENT`.
    ///
    /// Upstream `_accept_calibration`: `check_orientation && external &&
    /// _rotate_auto >= 2`.
    #[must_use]
    pub const fn saves_orientation(self, external: bool) -> bool {
        saves_orientation(self.as_u8(), external)
    }
}

impl Default for AutoRot {
    fn default() -> Self {
        Self::CheckAndFix
    }
}

/// Settings handed to `CompassCalibrator::set_orientation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoRotSettings {
    /// `cal_settings.check_orientation`.
    pub check_orientation: bool,
    /// Starting orientation (`COMPASS_ORIENT` if external, else `ROTATION_NONE`).
    pub orientation: u8,
    /// `cal_settings.is_external`.
    pub is_external: bool,
    /// `cal_settings.fix_orientation` (`_rotate_auto >= 2`).
    pub fix_orientation: bool,
    /// `cal_settings.always_45_deg` (`_rotate_auto >= 3`).
    pub always_45_deg: bool,
}

/// `_rotate_auto != 0`.
#[must_use]
pub const fn check_enabled(rotate_auto: u8) -> bool {
    rotate_auto != 0
}

/// `_rotate_auto >= 2`.
#[must_use]
pub const fn fix_orientation(rotate_auto: u8) -> bool {
    rotate_auto >= COMPASS_AUTO_ROT_CHECK_AND_FIX
}

/// `_rotate_auto >= 3`.
#[must_use]
pub const fn always_45_deg(rotate_auto: u8) -> bool {
    rotate_auto >= COMPASS_AUTO_ROT_FIX_45
}

/// Whether `_accept_calibration` may `set_and_save_orientation`.
#[must_use]
pub const fn saves_orientation(rotate_auto: u8, external: bool) -> bool {
    check_enabled(rotate_auto) && external && fix_orientation(rotate_auto)
}

/// Build `set_orientation` settings for MAG_CAL start, or `None` if disabled.
///
/// Internal compasses start from `ROTATION_NONE`; external use `COMPASS_ORIENT`.
#[must_use]
pub const fn settings_for_start(
    rotate_auto: u8,
    instance_orient: u8,
    external: bool,
) -> Option<AutoRotSettings> {
    if !check_enabled(rotate_auto) {
        return None;
    }
    let orientation = if external { instance_orient } else { 0 };
    Some(AutoRotSettings {
        check_orientation: true,
        orientation,
        is_external: external,
        fix_orientation: fix_orientation(rotate_auto),
        always_45_deg: always_45_deg(rotate_auto),
    })
}

/// Detected orientation to persist, or `None` when CheckOnly / internal / off.
#[must_use]
pub const fn accept_detected_orientation(
    rotate_auto: u8,
    external: bool,
    detected: u8,
) -> Option<u8> {
    if saves_orientation(rotate_auto, external) {
        Some(detected)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_check_and_fix() {
        assert_eq!(AutoRot::default(), AutoRot::CheckAndFix);
        assert_eq!(AutoRot::default().as_u8(), COMPASS_AUTO_ROT_DEFAULT);
        assert_eq!(
            AutoRot::from_u8(COMPASS_AUTO_ROT_DEFAULT),
            Some(AutoRot::CheckAndFix)
        );
    }

    #[test]
    fn maps_upstream_values() {
        assert_eq!(AutoRot::from_u8(0), Some(AutoRot::Disabled));
        assert_eq!(AutoRot::from_u8(1), Some(AutoRot::CheckOnly));
        assert_eq!(AutoRot::from_u8(2), Some(AutoRot::CheckAndFix));
        assert_eq!(AutoRot::from_u8(3), Some(AutoRot::CheckAndFix45));
        assert_eq!(AutoRot::from_u8(4), None);
        assert!(!AutoRot::Disabled.check_enabled());
        assert!(AutoRot::CheckOnly.check_enabled());
        assert!(!AutoRot::CheckOnly.fix_orientation());
        assert!(AutoRot::CheckAndFix.fix_orientation());
        assert!(!AutoRot::CheckAndFix.always_45_deg());
        assert!(AutoRot::CheckAndFix45.always_45_deg());
    }

    #[test]
    fn start_settings_match_upstream() {
        assert!(settings_for_start(COMPASS_AUTO_ROT_DISABLED, 2, true).is_none());
        let check = settings_for_start(COMPASS_AUTO_ROT_CHECK_ONLY, 2, true).expect("check");
        assert!(check.check_orientation);
        assert_eq!(check.orientation, 2);
        assert!(check.is_external);
        assert!(!check.fix_orientation);
        assert!(!check.always_45_deg);
        let internal = settings_for_start(COMPASS_AUTO_ROT_CHECK_AND_FIX, 2, false).expect("int");
        assert_eq!(internal.orientation, 0);
        assert!(!internal.is_external);
        assert!(internal.fix_orientation);
        let fix45 = settings_for_start(COMPASS_AUTO_ROT_FIX_45, 2, true).expect("45");
        assert!(fix45.fix_orientation);
        assert!(fix45.always_45_deg);
    }

    #[test]
    fn accept_only_saves_external_check_and_fix() {
        assert_eq!(
            accept_detected_orientation(COMPASS_AUTO_ROT_CHECK_AND_FIX, true, 2),
            Some(2)
        );
        assert_eq!(
            accept_detected_orientation(COMPASS_AUTO_ROT_CHECK_AND_FIX, false, 2),
            None
        );
        assert_eq!(
            accept_detected_orientation(COMPASS_AUTO_ROT_CHECK_ONLY, true, 2),
            None
        );
        assert_eq!(
            accept_detected_orientation(COMPASS_AUTO_ROT_DISABLED, true, 2),
            None
        );
        assert_eq!(
            accept_detected_orientation(COMPASS_AUTO_ROT_FIX_45, true, 2),
            Some(2)
        );
        assert!(AutoRot::CheckAndFix.saves_orientation(true));
        assert!(!AutoRot::CheckAndFix.saves_orientation(false));
    }
}
