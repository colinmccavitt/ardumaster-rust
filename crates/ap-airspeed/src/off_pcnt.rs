//! ARSPD_OFF_PCNT offset-cal speed-error warning, upstream `AP_Airspeed::max_speed_pcnt`.
//!
//! Plane-only (`AP_PARAM_FRAME_PLANE`). Default 0 disables the check. When set,
//! a calibration whose pressure offset jumps more than `OFF_PCNT` percent of
//! `ARSPD_FBW_MIN` / `AIRSPEED_MIN` warns that the pitot was likely uncovered.

use crate::fbw::ARSPD_FBW_MIN_DEFAULT;

/// Upstream `ARSPD_OFF_PCNT` default: disabled.
pub const ARSPD_OFF_PCNT_DEFAULT: i8 = 0;

/// Whether the offset-change warning is enabled (`ARSPD_OFF_PCNT` > 0).
#[must_use]
pub const fn off_pcnt_enabled(off_pcnt: i8) -> bool {
    off_pcnt > 0
}

/// Allowed |offset| change, upstream `0.5*(sq((1+pct)*vmin) - sq(vmin))`.
///
/// `pct` is `ARSPD_OFF_PCNT` in percent of `AIRSPEED_MIN`. The 1/2 v^2 form
/// matches `AP_Airspeed::calibrate()` (rho = 1).
#[must_use]
pub fn offset_max_change(off_pcnt: i8, airspeed_min: f32) -> f32 {
    if !off_pcnt_enabled(off_pcnt) || !(airspeed_min > 0.0) {
        return 0.0;
    }
    let scale = 1.0 + (off_pcnt as f32) * 0.01;
    0.5 * (scale * scale * airspeed_min * airspeed_min - airspeed_min * airspeed_min)
}

/// True when a new calibration offset should warn, matching `calibrate()`.
///
/// Requires a prior stored offset (`|stored| > 0`) so the first cal is silent.
#[must_use]
pub fn offset_change_warns(
    stored_offset: f32,
    calibrated_offset: f32,
    off_pcnt: i8,
    airspeed_min: f32,
) -> bool {
    off_pcnt_enabled(off_pcnt)
        && stored_offset.abs() > 0.0
        && (calibrated_offset - stored_offset).abs() > offset_max_change(off_pcnt, airspeed_min)
}

/// Convenience: default `AIRSPEED_MIN` (`ARSPD_FBW_MIN` = 9 m/s).
#[must_use]
pub fn offset_change_warns_default_min(
    stored_offset: f32,
    calibrated_offset: f32,
    off_pcnt: i8,
) -> bool {
    offset_change_warns(
        stored_offset,
        calibrated_offset,
        off_pcnt,
        ARSPD_FBW_MIN_DEFAULT,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_disables_warning() {
        assert_eq!(ARSPD_OFF_PCNT_DEFAULT, 0);
        assert!(!off_pcnt_enabled(ARSPD_OFF_PCNT_DEFAULT));
        assert!(!off_pcnt_enabled(-1));
        assert_eq!(offset_max_change(0, ARSPD_FBW_MIN_DEFAULT), 0.0);
        assert!(!offset_change_warns(100.0, 0.0, 0, 9.0));
    }

    #[test]
    fn max_change_matches_upstream_half_v_squared() {
        // 10% of 9 m/s: 0.5 * (1.1^2 * 81 - 81) = 0.5 * 81 * 0.21 = 8.505
        let max = offset_max_change(10, 9.0);
        assert!((max - 8.505).abs() < 1e-4);
        assert_eq!(offset_max_change(5, 0.0), 0.0);
        assert_eq!(offset_max_change(5, -1.0), 0.0);
    }

    #[test]
    fn first_cal_with_zero_stored_offset_is_silent() {
        assert!(!offset_change_warns(0.0, 50.0, 5, 9.0));
    }

    #[test]
    fn large_offset_jump_warns_small_jump_does_not() {
        let vmin = 9.0;
        let max = offset_max_change(10, vmin);
        assert!(offset_change_warns(10.0, 10.0 + max + 0.1, 10, vmin));
        assert!(!offset_change_warns(10.0, 10.0 + max - 0.1, 10, vmin));
        assert!(offset_change_warns_default_min(10.0, 10.0 + max + 1.0, 10));
    }
}
