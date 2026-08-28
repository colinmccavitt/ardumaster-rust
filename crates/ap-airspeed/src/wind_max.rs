//! ARSPD_WIND_MAX max airspeed check, upstream `AP_Airspeed::_wind_max`.
//!
//! Vehicle-level (not per-instance). A value of 0 disables the check. When
//! enabled, `|airspeed - groundspeed| > ARSPD_WIND_MAX` flags a failing pitot
//! (paired with `ARSPD_OPTIONS` bit 0 to disable TAS use).

/// Upstream `ARSPD_WIND_MAX` default: check disabled.
pub const ARSPD_WIND_MAX_DEFAULT: f32 = 0.0;

/// Whether the WIND_MAX check is enabled (`ARSPD_WIND_MAX` > 0).
#[must_use]
pub const fn wind_max_enabled(wind_max: f32) -> bool {
    wind_max > 0.0
}

/// Absolute airspeed minus GPS groundspeed (m/s).
#[must_use]
pub fn airspeed_groundspeed_delta(airspeed_mps: f32, groundspeed_mps: f32) -> f32 {
    (airspeed_mps - groundspeed_mps).abs()
}

/// True when `|EAS - groundspeed|` exceeds `ARSPD_WIND_MAX` and the check is on.
#[must_use]
pub fn wind_max_exceeded(airspeed_mps: f32, groundspeed_mps: f32, wind_max: f32) -> bool {
    wind_max_enabled(wind_max)
        && airspeed_groundspeed_delta(airspeed_mps, groundspeed_mps) > wind_max
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_disables_check() {
        assert_eq!(ARSPD_WIND_MAX_DEFAULT, 0.0);
        assert!(!wind_max_enabled(ARSPD_WIND_MAX_DEFAULT));
        assert!(!wind_max_exceeded(40.0, 5.0, ARSPD_WIND_MAX_DEFAULT));
        assert!(!wind_max_exceeded(40.0, 5.0, -1.0));
    }

    #[test]
    fn exceeded_when_delta_above_limit() {
        assert!((airspeed_groundspeed_delta(20.0, 5.0) - 15.0).abs() < 1e-6);
        assert!(wind_max_exceeded(20.0, 5.0, 10.0));
        assert!(!wind_max_exceeded(20.0, 15.0, 10.0));
        assert!(!wind_max_exceeded(20.0, 10.0, 10.0));
        assert!(wind_max_exceeded(5.0, 20.0, 10.0));
    }
}
