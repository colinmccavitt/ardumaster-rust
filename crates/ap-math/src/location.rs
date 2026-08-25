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
use crate::Ftype;

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

/// Metres per 1e-7 degree of latitude, upstream `LOCATION_SCALING_FACTOR`.
///
/// Upstream declares it `constexpr float` from a double literal, so the value
/// in use is the float rounding of `0.011131884502145034`. Written as an `f32`
/// here for the same reason: widening it later gives a different number than
/// upstream ever computes with.
pub const LOCATION_SCALING_FACTOR: f32 = 0.011_131_884_5;

/// A geodetic position, upstream `Location`'s latitude and longitude.
///
/// Both are in units of 1e-7 degrees, as ArduPilot stores and logs them.
/// Altitude and the frame flags are not here: nothing in the navigation path
/// ported so far reads them, and inventing a representation for them before
/// there is a consumer would be guessing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Location {
    /// Latitude, 1e-7 degrees.
    pub lat: i32,
    /// Longitude, 1e-7 degrees.
    pub lng: i32,
}

impl Location {
    /// A position from latitude and longitude in 1e-7 degrees.
    #[must_use]
    pub const fn new(lat: i32, lng: i32) -> Self {
        Self { lat, lng }
    }

    /// How much a degree of longitude shrinks at this latitude, upstream
    /// `longitude_scale`.
    ///
    /// Floored at 0.01 so that the poles do not produce a division by
    /// something arbitrarily small.
    #[must_use]
    pub fn longitude_scale(lat: i32) -> Ftype {
        // The argument is built in SINGLE precision even though the cosine is
        // a double one: under D-015 every literal here is a float, so the
        // product folds in float and the int32 latitude converts to float to
        // meet it. Near the poles that costs real bits -- 9e8 does not fit a
        // 24-bit significand -- and computing it in Ftype disagrees with
        // upstream in the ninth digit.
        #[allow(
            clippy::cast_precision_loss,
            reason = "upstream converts the int32 to float here; the loss is \
upstream's and reproducing it is the point"
        )]
        let arg = (lat as f32) * (1.0e-7_f32 * (core::f32::consts::PI / 180.0_f32));
        // And the cosine is the FLOAT cosine, not the double one: cosF(x) is
        // cos(x), and with a float argument C++ overload resolution picks
        // cos(float). ftype being double only widens the RESULT -- which is
        // why upstream's answer round-trips exactly through a float.
        let scale = Ftype::from(Real::cos(arg));
        // The floor is a FLOAT 0.01 as well, so at the poles upstream returns
        // 0.00999999977648258 rather than a clean hundredth. D-015 once more.
        const MIN_SCALE_F32: f32 = 0.01;
        let floor = Ftype::from(MIN_SCALE_F32);
        if scale > floor {
            scale
        } else {
            floor
        }
    }

    /// Longitude difference in 1e-7 degrees, taking the short way round,
    /// upstream `diff_longitude`.
    ///
    /// The same-sign case is separated out and done in 32 bits; upstream's
    /// comment calls it the common case. Only when the two straddle the sign
    /// boundary does it widen to 64 bits and fold the result into +-180
    /// degrees, which is what makes the antimeridian work.
    #[must_use]
    pub const fn diff_longitude(lon1: i32, lon2: i32) -> i32 {
        if (lon1 as u32 & 0x8000_0000) == (lon2 as u32 & 0x8000_0000) {
            return lon1.wrapping_sub(lon2);
        }
        let mut dlon = lon1 as i64 - lon2 as i64;
        if dlon > 1_800_000_000 {
            dlon -= 3_600_000_000;
        } else if dlon < -1_800_000_000 {
            dlon += 3_600_000_000;
        }
        dlon as i32
    }

    /// Offset from this position to `other`, north and east in metres,
    /// upstream `get_distance_NE`.
    ///
    /// The longitude scale is taken at the midpoint of the two latitudes, so
    /// the answer is symmetric rather than biased toward the origin.
    #[must_use]
    pub fn get_distance_ne(self, other: Self) -> Vector2f {
        let north = (other.lat - self.lat) as f32 * LOCATION_SCALING_FACTOR;
        let scale = Self::longitude_scale((other.lat + self.lat) / 2);
        let east =
            Ftype::from(Self::diff_longitude(other.lng, self.lng) as f32 * LOCATION_SCALING_FACTOR)
                * scale;
        Vector2f::new(north, east.to_f64() as f32)
    }

    /// Bearing to `other`, radians clockwise from north in `0..2*PI`,
    /// upstream `get_bearing`.
    ///
    /// Upstream writes this as `PI/2 + atan2(-off_y, off_x)` with `off_y` the
    /// latitude difference *divided* by the longitude scale. That is the
    /// ordinary `atan2(east, north)` rotated a quarter turn and scaled through
    /// the other term rather than the obvious one; it is reproduced as written
    /// because the intermediate rounding differs from the tidier form.
    #[must_use]
    pub fn get_bearing(self, other: Self) -> Ftype {
        let off_x = Self::diff_longitude(other.lng, self.lng);
        // Upstream declares this , so the division's result is
        // TRUNCATED to an integer before the arctangent ever sees it. Keeping
        // it in floating point disagrees in the eleventh digit -- small, but
        // this is the bearing the aircraft steers to, and the truncation is
        // upstream's behaviour rather than an artefact of reading the source.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream truncates to int32 here; reproducing it is the point"
        )]
        let off_y = (Ftype::from_f64(f64::from(other.lat - self.lat))
            / Self::longitude_scale((self.lat + other.lat) / 2))
        .to_f64() as i32;
        // Both constants are FLOAT half-pi and float two-pi, not the double
        // ones: they come from `M_PI`, which D-015 makes a float literal. They
        // are added to a double atan2 result, so the sum is double but starts
        // from a single-precision offset.
        const HALF_PI_F32: f32 = core::f32::consts::PI * 0.5;
        const TWO_PI_F32: f32 = 2.0 * core::f32::consts::PI;

        let bearing = Ftype::from(HALF_PI_F32)
            + Real::atan2(
                Ftype::from_f64(f64::from(-off_y)),
                Ftype::from_f64(f64::from(off_x)),
            );
        if bearing < Ftype::from_f64(0.0) {
            bearing + Ftype::from(TWO_PI_F32)
        } else {
            bearing
        }
    }

    /// Bearing to `other` in centidegrees, `0..35999`, upstream
    /// `get_bearing_to`.
    ///
    /// Note the `+ 0.5` before truncation: upstream rounds to nearest here,
    /// unlike most of its centidegree conversions.
    #[must_use]
    pub fn get_bearing_to(self, other: Self) -> i32 {
        let cd =
            self.get_bearing(other) * (Ftype::from_f64(18000.0) / Ftype::PI) + Ftype::from_f64(0.5);
        cd.to_f64() as i32
    }
}
