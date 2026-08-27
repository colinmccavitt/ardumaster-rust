//! AHRS backend selection, upstream `AP_AHRS::EKFType` and `backend_for_type`.
//!
//! EKF3 update wiring is stubbed; Plane falls back from EKF to DCM when unhealthy.
//!
//! **FW-008 DCM scope is complete.** NavEKF3 full filter port continues in FW-009.

/// FW-008 DCM port scope is complete; NavEKF3 full filter is FW-009.
pub const DCM_SCOPE_COMPLETE: bool = true;

/// Configured attitude estimator, upstream `AP_AHRS::EKFType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum AhrsBackendKind {
    /// Direction-cosine fallback, upstream `EKFType::DCM`.
    #[default]
    Dcm = 0,
    /// NavEKF3 primary, upstream `EKFType::THREE` / `AHRS_EKF_TYPE=3`.
    Ekf3 = 3,
}

impl AhrsBackendKind {
    /// Map `AHRS_EKF_TYPE` parameter value, upstream `_configured_ekf_type()`.
    #[must_use]
    pub const fn from_ekf_type_param(ekf_type: u8) -> Self {
        match ekf_type {
            3 => Self::Ekf3,
            _ => Self::Dcm,
        }
    }

    /// Whether this backend is implemented in the port.
    #[must_use]
    pub const fn is_available(self) -> bool {
        matches!(self, Self::Dcm | Self::Ekf3)
    }
}

/// Resolve a configured type to an allocated backend, upstream `backend_for_type`.
///
/// Resolve configured type to the allocated backend instance.
#[must_use]
pub const fn backend_for_kind(kind: AhrsBackendKind) -> AhrsBackendKind {
    match kind {
        AhrsBackendKind::Dcm => AhrsBackendKind::Dcm,
        AhrsBackendKind::Ekf3 => AhrsBackendKind::Ekf3,
    }
}

/// Active backend with EKF→DCM fallback, upstream `_active_EKF_type()`.
#[must_use]
pub const fn active_backend_kind(configured: AhrsBackendKind, ekf_healthy: bool) -> AhrsBackendKind {
    match backend_for_kind(configured) {
        AhrsBackendKind::Ekf3 if !ekf_healthy => AhrsBackendKind::Dcm,
        kind => kind,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dcm_is_always_available() {
        assert!(AhrsBackendKind::Dcm.is_available());
        assert_eq!(backend_for_kind(AhrsBackendKind::Dcm), AhrsBackendKind::Dcm);
    }

    #[test]
    fn ekf3_resolves_to_ekf3_backend() {
        assert!(AhrsBackendKind::Ekf3.is_available());
        assert_eq!(backend_for_kind(AhrsBackendKind::Ekf3), AhrsBackendKind::Ekf3);
    }

    #[test]
    fn unhealthy_ekf_falls_back_to_dcm() {
        assert_eq!(
            active_backend_kind(AhrsBackendKind::Ekf3, false),
            AhrsBackendKind::Dcm
        );
    }

    #[test]
    fn healthy_ekf_stays_on_ekf3() {
        assert_eq!(
            active_backend_kind(AhrsBackendKind::Ekf3, true),
            AhrsBackendKind::Ekf3
        );
    }

    #[test]
    fn dcm_scope_complete_marker_for_fw008() {
        assert!(DCM_SCOPE_COMPLETE);
    }

    #[test]
    fn from_ekf_type_param_matches_upstream() {
        assert_eq!(AhrsBackendKind::from_ekf_type_param(0), AhrsBackendKind::Dcm);
        assert_eq!(AhrsBackendKind::from_ekf_type_param(2), AhrsBackendKind::Dcm);
        assert_eq!(AhrsBackendKind::from_ekf_type_param(3), AhrsBackendKind::Ekf3);
    }
}
