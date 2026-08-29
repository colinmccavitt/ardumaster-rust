//! Geofence breach failsafe action, Plane `FENCE_ACTION` / `AC_Fence::Action`.
//!
//! Upstream `Plane::fence_check` in `ArduPlane/fence.cpp` and
//! `AC_Fence::Action` in `libraries/AC_Fence/AC_Fence.h`. Plane 4.7
//! `@Values{Plane}` is 0 Report Only, 1 RTL, 6 Guided, 7 GuidedThrottlePass,
//! 8 Autoland or RTL. Copter-only values 2–5 are invalid on Plane and take
//! no action.
//!
//! This stub keeps the ticket's Report / RTL / Guided / GuidedThrottlePass
//! / Terminate table and adds value 8 (`AUTOLAND_OR_RTL`). Terminate is the
//! AFS-style hard action
//! (`afs.gcs_terminate` on a geofence trip) rather than a Plane 4.7
//! `FENCE_ACTION` token — `fence.cpp` has no terminate case. Circle / alt-max leftovers live in `ap-fence` (**COP-025**);
//! this hookup decodes `FENCE_ACTION` through [`ap_fence::Action`] and maps
//! a new-breach bitmask onto the Plane action table. Radio / GCS
//! / battery / short-long timers / terrain are left to their own modules.

/// Upstream Plane `FENCE_ACTION` / `AC_Fence::Action` as this stub uses it.
///
/// Default is [`Self::Rtl`] (`RTL_AND_LAND`, the Plane `@Values` "RTL"
/// token and the `AP_GROUPINFO` default).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceAction {
    /// 0 — report to GCS, no mode change (`REPORT_ONLY`).
    ReportOnly,
    /// 1 — RTL (`RTL_AND_LAND`).
    Rtl,
    /// 6 — Guided to the fence return point (`GUIDED`).
    Guided,
    /// 7 — Guided, pilot keeps throttle (`GUIDED_THROTTLE_PASS`).
    GuidedThrottlePass,
    /// 8 — Autoland if that mode can start, else RTL (`AUTOLAND_OR_RTL`).
    AutolandOrRtl,
    /// AFS-style terminate / disarm. Not a Plane 4.7 `@Values{Plane}` token.
    Terminate,
}

impl FenceAction {
    /// Decode Plane `FENCE_ACTION`. Unknown / Copter-only values are `None`.
    ///
    /// Plane 4.7 numbers: 0 Report, 1 RTL, 6 Guided, 7 GuidedThrottlePass,
    /// 8 Autoland-or-RTL. Terminate is not a `FENCE_ACTION` token; construct
    /// [`Self::Terminate`].
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match ap_fence::Action::from_param(value) {
            Some(ap_fence::Action::ReportOnly) => Some(Self::ReportOnly),
            Some(ap_fence::Action::RtlAndLand) => Some(Self::Rtl),
            Some(ap_fence::Action::Guided) => Some(Self::Guided),
            Some(ap_fence::Action::GuidedThrottlePass) => Some(Self::GuidedThrottlePass),
            Some(ap_fence::Action::AutolandOrRtl) => Some(Self::AutolandOrRtl),
            // Copter-only AC_Fence::Action tokens are invalid on Plane.
            Some(_) | None => None,
        }
    }

    /// Upstream `AC_Fence` default, `Action::RTL_AND_LAND`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::Rtl
    }

    /// Whether this action changes mode or terminates (not report-only).
    #[must_use]
    pub const fn changes_vehicle(self) -> bool {
        !matches!(self, Self::ReportOnly)
    }
}

/// What `fence_check` asks the vehicle to do on a new breach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceFailsafeResult {
    /// Stay put (disabled, disarmed, already recovering, or no new breach).
    None,
    /// GCS / log report only (`REPORT_ONLY`).
    Report,
    /// `set_mode(mode_rtl, ModeReason::FENCE_BREACHED)`.
    Rtl,
    /// `set_mode(mode_guided, ModeReason::FENCE_BREACHED)`.
    Guided,
    /// Guided plus `guided_throttle_passthru = true`.
    GuidedThrottlePass,
    /// `set_mode(mode_autoland, ModeReason::FENCE_BREACHED)`.
    Autoland,
    /// `afs.gcs_terminate` / disarm.
    Terminate,
}

/// Inputs for the geofence breach action stub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FenceFailsafeInputs {
    /// `FENCE_ACTION` / `fence.get_action()`.
    pub action: FenceAction,
    /// `fence.enabled()`.
    pub enabled: bool,
    /// `fence_breaches.new_breaches != 0`.
    pub new_breach: bool,
    /// `arming.is_armed()`. Actions other than stay-put require armed.
    pub armed: bool,
    /// `in_fence_recovery()` — already handling a breach.
    pub in_recovery: bool,
    /// `landing.is_expecting_impact()` — final landing stage suppresses.
    pub landing_impact: bool,
    /// `MODE_AUTOLAND_ENABLED` and `set_mode(mode_autoland, ...)` can start.
    pub autoland_available: bool,
}

impl Default for FenceFailsafeInputs {
    fn default() -> Self {
        Self {
            action: FenceAction::default_param(),
            enabled: false,
            new_breach: false,
            armed: false,
            in_recovery: false,
            landing_impact: false,
            autoland_available: false,
        }
    }
}

/// Resolve `Plane::fence_check` once a new breach is latched.
///
/// Disabled / disarmed / landing-impact / already-in-recovery / no new
/// breach all return [`FenceFailsafeResult::None`]. A live breach then
/// maps [`FenceAction`] onto the Report / RTL / Guided /
/// GuidedThrottlePass / Terminate table, plus value 8 Autoland-or-RTL.
#[must_use]
pub fn fence_failsafe_action(inp: &FenceFailsafeInputs) -> FenceFailsafeResult {
    if !inp.enabled || !inp.armed || inp.landing_impact || inp.in_recovery || !inp.new_breach {
        return FenceFailsafeResult::None;
    }
    match inp.action {
        FenceAction::ReportOnly => FenceFailsafeResult::Report,
        FenceAction::Rtl => FenceFailsafeResult::Rtl,
        FenceAction::Guided => FenceFailsafeResult::Guided,
        FenceAction::GuidedThrottlePass => FenceFailsafeResult::GuidedThrottlePass,
        FenceAction::AutolandOrRtl => {
            if inp.autoland_available {
                FenceFailsafeResult::Autoland
            } else {
                FenceFailsafeResult::Rtl
            }
        }
        FenceAction::Terminate => FenceFailsafeResult::Terminate,
    }
}

/// Map `AC_Fence::check` new-breach bits onto [`fence_failsafe_action`].
///
/// Circle / alt-max leftovers live in `ap-fence`. A zero mask is "no new
/// breach" — the same as `fence_breaches.new_breaches == 0`.
#[must_use]
pub fn fence_failsafe_from_new_breaches(
    mut inp: FenceFailsafeInputs,
    new_breaches: u8,
) -> FenceFailsafeResult {
    inp.new_breach = new_breaches != 0;
    fence_failsafe_action(&inp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_values_match_upstream_plane_fence_action() {
        assert_eq!(FenceAction::from_param(0), Some(FenceAction::ReportOnly));
        assert_eq!(FenceAction::from_param(1), Some(FenceAction::Rtl));
        assert_eq!(FenceAction::from_param(6), Some(FenceAction::Guided));
        assert_eq!(
            FenceAction::from_param(7),
            Some(FenceAction::GuidedThrottlePass)
        );
        assert_eq!(FenceAction::from_param(2), None);
        assert_eq!(FenceAction::from_param(3), None);
        assert_eq!(FenceAction::from_param(4), None);
        assert_eq!(FenceAction::from_param(5), None);
        assert_eq!(FenceAction::from_param(8), Some(FenceAction::AutolandOrRtl));
        assert_eq!(FenceAction::from_param(9), None);
        assert_eq!(FenceAction::default_param(), FenceAction::Rtl);
        assert!(!FenceAction::ReportOnly.changes_vehicle());
        assert!(FenceAction::Rtl.changes_vehicle());
        assert!(FenceAction::AutolandOrRtl.changes_vehicle());
        assert!(FenceAction::Terminate.changes_vehicle());
    }

    #[test]
    fn new_breach_bits_from_ap_fence_trip_the_plane_table() {
        use ap_fence::{TYPE_ALT_MAX, TYPE_CIRCLE};

        let quiet = FenceFailsafeInputs {
            action: FenceAction::Rtl,
            enabled: true,
            new_breach: true,
            armed: true,
            in_recovery: false,
            landing_impact: false,
            autoland_available: false,
        };
        assert_eq!(
            fence_failsafe_from_new_breaches(quiet, 0),
            FenceFailsafeResult::None
        );
        assert_eq!(
            fence_failsafe_from_new_breaches(quiet, TYPE_CIRCLE | TYPE_ALT_MAX),
            FenceFailsafeResult::Rtl
        );
        assert_eq!(
            FenceAction::from_param(ap_fence::Action::AlwaysLand as u8),
            None
        );
    }
}
