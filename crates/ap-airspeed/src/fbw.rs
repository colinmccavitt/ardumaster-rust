//! ARSPD_FBW_MIN / ARSPD_FBW_MAX fly-by-wire airspeed limits.
//!
//! Vehicle-level demanded-airspeed envelope, upstream `aparm.airspeed_min` /
//! `aparm.airspeed_max` (renamed from `ARSPD_FBW_MIN` / `ARSPD_FBW_MAX`).
//! Defaults match `AIRSPEED_FBW_MIN` / `AIRSPEED_FBW_MAX` in ArduPlane `config.h`.

/// Upstream `AIRSPEED_FBW_MIN` / `ARSPD_FBW_MIN` default (m/s).
pub const ARSPD_FBW_MIN_DEFAULT: f32 = 9.0;

/// Upstream `AIRSPEED_FBW_MAX` / `ARSPD_FBW_MAX` default (m/s).
pub const ARSPD_FBW_MAX_DEFAULT: f32 = 22.0;

/// Ordered `(min, max)` envelope. Swaps if the params are inverted.
#[must_use]
pub const fn fbw_envelope(fbw_min: f32, fbw_max: f32) -> (f32, f32) {
    if fbw_min <= fbw_max {
        (fbw_min, fbw_max)
    } else {
        (fbw_max, fbw_min)
    }
}

/// Constrain demanded airspeed into `[ARSPD_FBW_MIN, ARSPD_FBW_MAX]`.
#[must_use]
pub fn clamp_fbw_airspeed(demanded_mps: f32, fbw_min: f32, fbw_max: f32) -> f32 {
    let (lo, hi) = fbw_envelope(fbw_min, fbw_max);
    if demanded_mps < lo {
        lo
    } else if demanded_mps > hi {
        hi
    } else {
        demanded_mps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_config() {
        assert!((ARSPD_FBW_MIN_DEFAULT - 9.0).abs() < 1e-6);
        assert!((ARSPD_FBW_MAX_DEFAULT - 22.0).abs() < 1e-6);
        assert_eq!(
            fbw_envelope(ARSPD_FBW_MIN_DEFAULT, ARSPD_FBW_MAX_DEFAULT),
            (9.0, 22.0)
        );
        assert!(
            (clamp_fbw_airspeed(15.0, ARSPD_FBW_MIN_DEFAULT, ARSPD_FBW_MAX_DEFAULT) - 15.0).abs()
                < 1e-6
        );
    }

    #[test]
    fn clamp_holds_demanded_inside_envelope() {
        assert!((clamp_fbw_airspeed(5.0, 9.0, 22.0) - 9.0).abs() < 1e-6);
        assert!((clamp_fbw_airspeed(30.0, 9.0, 22.0) - 22.0).abs() < 1e-6);
        assert!((clamp_fbw_airspeed(12.0, 9.0, 22.0) - 12.0).abs() < 1e-6);
        assert!((clamp_fbw_airspeed(9.0, 9.0, 22.0) - 9.0).abs() < 1e-6);
        assert!((clamp_fbw_airspeed(22.0, 9.0, 22.0) - 22.0).abs() < 1e-6);
    }

    #[test]
    fn inverted_limits_are_swapped() {
        assert_eq!(fbw_envelope(22.0, 9.0), (9.0, 22.0));
        assert!((clamp_fbw_airspeed(5.0, 22.0, 9.0) - 9.0).abs() < 1e-6);
        assert!((clamp_fbw_airspeed(30.0, 22.0, 9.0) - 22.0).abs() < 1e-6);
    }
}
