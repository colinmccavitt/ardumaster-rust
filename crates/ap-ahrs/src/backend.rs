//! AHRS backend selection, upstream `AP_AHRS::EKFType` and `backend_for_type`.
//!
//! The full EKF is not ported yet; [`AhrsBackendKind::Ekf3`] resolves to DCM until
//! NavEKF3 lands. Plane still falls back from EKF to DCM when unhealthy.

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
        matches!(self, Self::Dcm)
    }
}

/// Resolve a configured type to an allocated backend, upstream `backend_for_type`.
///
/// Unimplemented backends fall back to DCM rather than leaving the vehicle
/// without an attitude source.
#[must_use]
pub const fn backend_for_kind(kind: AhrsBackendKind) -> AhrsBackendKind {
    match kind {
        AhrsBackendKind::Dcm => AhrsBackendKind::Dcm,
        AhrsBackendKind::Ekf3 => AhrsBackendKind::Dcm,
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
    fn ekf3_stub_falls_back_to_dcm() {
        assert!(!AhrsBackendKind::Ekf3.is_available());
        assert_eq!(backend_for_kind(AhrsBackendKind::Ekf3), AhrsBackendKind::Dcm);
    }

    #[test]
    fn unhealthy_ekf_falls_back_to_dcm() {
        assert_eq!(
            active_backend_kind(AhrsBackendKind::Ekf3, false),
            AhrsBackendKind::Dcm
        );
    }

    #[test]
    fn healthy_ekf_stays_on_dcm_until_ported() {
        assert_eq!(
            active_backend_kind(AhrsBackendKind::Ekf3, true),
            AhrsBackendKind::Dcm
        );
    }

    #[test]
    fn from_ekf_type_param_matches_upstream() {
        assert_eq!(AhrsBackendKind::from_ekf_type_param(0), AhrsBackendKind::Dcm);
        assert_eq!(AhrsBackendKind::from_ekf_type_param(2), AhrsBackendKind::Dcm);
        assert_eq!(AhrsBackendKind::from_ekf_type_param(3), AhrsBackendKind::Ekf3);
    }
}
