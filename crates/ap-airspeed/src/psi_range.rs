//! ARSPD_PSI_RANGE sensor pressure-range, upstream `AP_Airspeed_Params::psi_range`.
//!
//! Per-instance PSI full-scale used by analog and MS4525 backends
//! (`get_psi_range()`). Default matches `PSI_RANGE_DEFAULT`. Non-finite or
//! non-positive values clamp to the default so voltage-to-pressure conversion
//! never divides by zero.

/// Upstream `PSI_RANGE_DEFAULT` / `ARSPD_PSI_RANGE`.
pub const ARSPD_PSI_RANGE_DEFAULT: f32 = 1.0;

/// True when `ARSPD_PSI_RANGE` is a usable full-scale (finite and > 0).
#[must_use]
pub fn psi_range_valid(psi_range: f32) -> bool {
    psi_range.is_finite() && psi_range > 0.0
}

/// Clamp `ARSPD_PSI_RANGE` to a usable full-scale, falling back to the default.
#[must_use]
pub fn clamp_psi_range(psi_range: f32) -> f32 {
    if psi_range_valid(psi_range) {
        psi_range
    } else {
        ARSPD_PSI_RANGE_DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_upstream() {
        assert!((ARSPD_PSI_RANGE_DEFAULT - 1.0).abs() < 1e-6);
        assert!(psi_range_valid(ARSPD_PSI_RANGE_DEFAULT));
        assert!((clamp_psi_range(ARSPD_PSI_RANGE_DEFAULT) - 1.0).abs() < 1e-6);
        assert!((clamp_psi_range(2.0) - 2.0).abs() < 1e-6);
        assert!((clamp_psi_range(0.5) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn invalid_range_clamps_to_default() {
        assert!(!psi_range_valid(0.0));
        assert!(!psi_range_valid(-1.0));
        assert!(!psi_range_valid(f32::NAN));
        assert!(!psi_range_valid(f32::INFINITY));
        assert!((clamp_psi_range(0.0) - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
        assert!((clamp_psi_range(-2.0) - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
        assert!((clamp_psi_range(f32::NAN) - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
        assert!((clamp_psi_range(f32::INFINITY) - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
    }
}
