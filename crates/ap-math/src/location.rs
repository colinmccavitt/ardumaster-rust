//! Bearing, distance and coordinate validity, ported from
//! `AP_Math/location.{cpp,h}`.
//!
//! Used by `AP_L1_Control`, the fixed-wing navigation controller, and by the
//! mission and loiter logic in ArduPlane.
//!
//! # Not ported: the WGS84 ECEF conversions
//!
//! `location_double.cpp` implements `wgsllh2ecef` and `wgsecef2llh`. Its own
//! comment says "these are not currently used", and a search of the codebase
//! confirms it — nothing outside `AP_Math` references either. They are left
//! unported rather than carried over untested.

use crate::scalar::{wrap_2pi, Real};
use crate::vector2::{Vector2, Vector2f};

/// Distance between two points, upstream `get_horizontal_distance`.
///
/// Upstream declares the return type as `float` regardless of the vector's
/// element type, so a `Vector2d` caller would receive a narrowed result. The
/// port returns `T`. No caller uses the double form, so the two agree
/// everywhere it is actually used.
#[must_use]
pub fn get_horizontal_distance<T: Real>(origin: Vector2<T>, destination: Vector2<T>) -> T {
    (destination - origin).length()
}

/// Bearing from `origin` to `destination`, radians in `[0, 2*pi)`.
#[must_use]
pub fn get_bearing_rad(origin: Vector2f, destination: Vector2f) -> f32 {
    wrap_2pi((destination.y - origin.y).atan2(destination.x - origin.x))
}

/// Bearing from `origin` to `destination`, centidegrees.
#[must_use]
pub fn get_bearing_cd(origin: Vector2f, destination: Vector2f) -> f32 {
    crate::scalar::rad_to_cd(get_bearing_rad(origin, destination))
}

/// Whether a latitude in degrees is in range.
#[must_use]
pub fn check_lat_deg(lat: f32) -> bool {
    lat.abs() <= 90.0
}

/// Whether a longitude in degrees is in range.
#[must_use]
pub fn check_lng_deg(lng: f32) -> bool {
    lng.abs() <= 180.0
}

/// Whether a latitude in 1e7 degrees is in range.
///
/// # DIVERGENCE D-016
///
/// Upstream writes `labs(lat) <= 90*1e7`. With `-fsingle-precision-constant`
/// (D-015) the bound is a `float`, so the comparison converts the integer to
/// `float` first. At 9e8 the spacing between representable floats is 64, so
/// every value from 900000001 to 900000032 rounds down onto the bound and is
/// **accepted** — latitudes marginally beyond 90 degrees pass a check whose
/// entire purpose is to reject them.
///
/// The port compares as integers. The overshoot is about 3.2e-6 degrees, so
/// nothing downstream would notice, but the correct behaviour is unambiguous
/// and the fix costs nothing. See DIVERGENCES.md.
#[must_use]
pub fn check_lat_1e7(lat: i32) -> bool {
    lat.unsigned_abs() <= 90 * 10_000_000
}

/// Whether a longitude in 1e7 degrees is in range.
///
/// Same rounding issue as [`check_lat_1e7`]; see D-016.
#[must_use]
pub fn check_lng_1e7(lng: i32) -> bool {
    lng.unsigned_abs() <= 180 * 10_000_000
}

/// Whether both coordinates in degrees are in range.
#[must_use]
pub fn check_latlng_deg(lat: f32, lng: f32) -> bool {
    check_lat_deg(lat) && check_lng_deg(lng)
}

/// Whether both coordinates in 1e7 degrees are in range.
#[must_use]
pub fn check_latlng_1e7(lat: i32, lng: i32) -> bool {
    check_lat_1e7(lat) && check_lng_1e7(lng)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values throughout")]

    use super::*;

    #[test]
    fn bearing_is_measured_from_north_and_wrapped() {
        let o = Vector2f::new(0.0, 0.0);
        // upstream's convention is atan2(dy, dx) on a north-east frame, so
        // +x is north and the result grows clockwise
        assert_eq!(get_bearing_rad(o, Vector2f::new(1.0, 0.0)), 0.0);
        assert!(
            (get_bearing_rad(o, Vector2f::new(0.0, 1.0)) - core::f32::consts::FRAC_PI_2).abs()
                < 1e-6
        );
        // due west must come back as 3*pi/2 rather than -pi/2
        let west = get_bearing_rad(o, Vector2f::new(0.0, -1.0));
        assert!(
            west > 4.0,
            "bearing must be wrapped into [0, 2pi), got {west}"
        );
    }

    /// D-016: upstream accepts latitudes past the bound because the comparison
    /// happens in `float`. The port rejects them.
    #[test]
    fn d016_latitude_bound_is_checked_as_an_integer() {
        assert!(check_lat_1e7(900_000_000), "exactly 90 degrees is in range");
        assert!(
            !check_lat_1e7(900_000_001),
            "one unit past 90 degrees must be rejected; upstream accepts it \
             because 900000001 rounds onto 9e8 as a float"
        );
        assert!(!check_lat_1e7(900_000_032), "still past the bound");
        assert!(
            check_lat_1e7(-900_000_000),
            "and symmetrically for the south"
        );
        assert!(!check_lat_1e7(-900_000_001));

        assert!(check_lng_1e7(1_800_000_000));
        assert!(!check_lng_1e7(1_800_000_001));
    }

    /// `i32::MIN` has no positive counterpart, so a naive `abs()` would panic
    /// in debug. `unsigned_abs` is total.
    #[test]
    fn extreme_integers_do_not_panic() {
        assert!(!check_lat_1e7(i32::MIN));
        assert!(!check_lng_1e7(i32::MIN));
        assert!(!check_lat_1e7(i32::MAX));
    }

    #[test]
    fn degree_bounds_are_inclusive() {
        assert!(check_lat_deg(90.0));
        assert!(check_lat_deg(-90.0));
        assert!(!check_lat_deg(90.1));
        assert!(check_lng_deg(180.0));
        assert!(!check_lng_deg(-180.1));
        assert!(check_latlng_deg(45.0, 90.0));
        assert!(!check_latlng_deg(45.0, 181.0));
    }

    #[test]
    fn horizontal_distance_is_the_vector_length() {
        let a = Vector2f::new(1.0, 2.0);
        let b = Vector2f::new(4.0, 6.0);
        assert_eq!(get_horizontal_distance(a, b), 5.0);
        assert_eq!(get_horizontal_distance(a, a), 0.0);
    }
}
