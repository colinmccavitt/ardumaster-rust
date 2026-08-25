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

use crate::scalar::{radians, wrap_2pi, Real};
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

    /// A frame round-trips through the three storage bits mission storage
    /// keeps.
    #[test]
    fn every_frame_survives_the_storage_bits() {
        for frame in [
            AltFrame::Absolute,
            AltFrame::AboveHome,
            AltFrame::AboveOrigin,
            AltFrame::AboveTerrain,
        ] {
            let (rel, terr, orig) = frame.to_bits();
            assert_eq!(
                AltFrame::from_bits(rel, terr, orig),
                Some(frame),
                "{frame:?}"
            );
        }
    }

    /// Above-terrain sets two bits, not one. Upstream marks it relative as
    /// well, because a terrain altitude has not had home added either, and
    /// anything reading the bits has to know that.
    #[test]
    fn above_terrain_is_also_marked_relative() {
        assert_eq!(AltFrame::AboveTerrain.to_bits(), (true, true, false));
        assert_eq!(AltFrame::AboveHome.to_bits(), (true, false, false));
    }

    /// D-023. Terrain without relative is the combination upstream calls
    /// impossible and guards with a SITL-only panic. The port reports it.
    #[test]
    fn d023_terrain_without_relative_is_reported_not_fatal() {
        assert_eq!(AltFrame::from_bits(false, true, false), None);
        assert_eq!(AltFrame::from_bits(false, true, true), None);
    }

    /// Several bits set is not an error, and the precedence is upstream's:
    /// terrain over origin, origin over relative.
    #[test]
    fn the_bit_precedence_is_terrain_then_origin_then_relative() {
        assert_eq!(
            AltFrame::from_bits(true, true, true),
            Some(AltFrame::AboveTerrain)
        );
        assert_eq!(
            AltFrame::from_bits(true, false, true),
            Some(AltFrame::AboveOrigin)
        );
    }

    /// Converting to the frame a location is already in returns the altitude
    /// unchanged and needs no context -- which matters, because that is the
    /// common case and it must not fail before home is set.
    #[test]
    fn converting_to_the_same_frame_needs_nothing() {
        let loc = Location::new_with_alt(0, 0, 12_345, AltFrame::AboveHome);
        let empty = AltContext::default();
        assert_eq!(loc.get_alt_cm(AltFrame::AboveHome, &empty), Some(12_345));
    }

    /// A conversion that needs home before home is set has no answer. Not a
    /// zero, not the raw number -- no answer.
    #[test]
    fn a_conversion_without_home_has_no_answer() {
        let loc = Location::new_with_alt(0, 0, 100, AltFrame::AboveHome);
        assert_eq!(
            loc.get_alt_cm(AltFrame::Absolute, &AltContext::default()),
            None
        );

        let ctx = AltContext {
            home_alt_cm: Some(5_000),
            ..AltContext::default()
        };
        assert_eq!(loc.get_alt_cm(AltFrame::Absolute, &ctx), Some(5_100));
    }

    /// Every pair of frames converts through absolute and back to where it
    /// started.
    #[test]
    fn frames_round_trip_through_each_other() {
        let ctx = AltContext {
            home_alt_cm: Some(12_000),
            origin_alt_cm: Some(11_500),
            terrain_alt_cm: Some(10_000),
        };
        let frames = [
            AltFrame::Absolute,
            AltFrame::AboveHome,
            AltFrame::AboveOrigin,
            AltFrame::AboveTerrain,
        ];

        for from in frames {
            for to in frames {
                let mut loc = Location::new_with_alt(515_080_000, -1_268_000, 25_000, from);
                assert!(loc.change_alt_frame(to, &ctx), "{from:?} -> {to:?}");
                assert_eq!(loc.alt_frame(), to);
                assert!(loc.change_alt_frame(from, &ctx), "{to:?} -> {from:?}");
                assert_eq!(
                    loc.alt, 25_000,
                    "{from:?} -> {to:?} -> {from:?} should return the altitude"
                );
            }
        }
    }

    /// The conversions use the datums they name: 25,000 cm above a home at
    /// 12,000 cm AMSL is 37,000 cm AMSL.
    #[test]
    fn the_conversions_use_the_datums_they_name() {
        let ctx = AltContext {
            home_alt_cm: Some(12_000),
            origin_alt_cm: Some(11_500),
            terrain_alt_cm: Some(10_000),
        };
        let loc = Location::new_with_alt(0, 0, 25_000, AltFrame::AboveHome);

        assert_eq!(loc.get_alt_cm(AltFrame::Absolute, &ctx), Some(37_000));
        assert_eq!(loc.get_alt_cm(AltFrame::AboveOrigin, &ctx), Some(25_500));
        assert_eq!(loc.get_alt_cm(AltFrame::AboveTerrain, &ctx), Some(27_000));
    }

    /// A failed conversion leaves the location exactly as it was.
    #[test]
    fn a_failed_conversion_changes_nothing() {
        let mut loc = Location::new_with_alt(0, 0, 100, AltFrame::AboveHome);
        let before = loc;
        assert!(!loc.change_alt_frame(AltFrame::Absolute, &AltContext::default()));
        assert_eq!(loc.alt, before.alt);
        assert_eq!(loc.alt_frame(), before.alt_frame());
    }

    /// offset_up_m works in metres on a field held in centimetres, and leaves
    /// the frame alone.
    #[test]
    fn offset_up_moves_altitude_and_not_the_frame() {
        let mut loc = Location::new_with_alt(0, 0, 1_000, AltFrame::AboveHome);
        loc.offset_up_m(2.5);
        assert_eq!(loc.alt, 1_250);
        assert_eq!(loc.alt_frame(), AltFrame::AboveHome);

        loc.offset_up_m(-12.5);
        assert_eq!(loc.alt, 0);
    }

    /// Upstream treats lat and lng both zero as "no position". It is a real
    /// coordinate in the Atlantic, so a vehicle genuinely there cannot be
    /// told apart from one that has never had a fix.
    #[test]
    fn an_all_zero_location_reads_as_uninitialised() {
        assert!(!Location::new(0, 0).initialised());
        assert!(Location::new(1, 0).initialised());
        assert!(Location::new(0, 1).initialised());
    }
}

/// Radians to centidegrees in single precision, upstream `rad_to_cd`.
///
/// Upstream declares it `float rad_to_cd(float)`. That signature matters:
/// callers holding a double narrow on the call rather than scaling at full
/// precision.
#[inline]
fn rad_to_cd_f32(rad: f32) -> f32 {
    rad * (18000.0_f32 / core::f32::consts::PI)
}

/// Metres per 1e-7 degree of latitude, upstream `LOCATION_SCALING_FACTOR`.
///
/// Upstream declares it `constexpr float` from a double literal, so the value
/// in use is the float rounding of `0.011131884502145034`. Written as an `f32`
/// here for the same reason: widening it later gives a different number than
/// upstream ever computes with.
pub const LOCATION_SCALING_FACTOR: f32 = 0.011_131_884_5;

/// 1e-7 degrees of latitude per metre, upstream
/// `LOCATION_SCALING_FACTOR_INV` (`LATLON_TO_M_INV`).
///
/// Not computed as `1.0 / LOCATION_SCALING_FACTOR`: upstream carries both as
/// separate literals, and the reciprocal of the rounded `f32` forward factor
/// is not the rounded `f32` of the true reciprocal.
pub const LOCATION_SCALING_FACTOR_INV: f32 = 89.832_05;

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
    /// Altitude in centimetres, measured from whatever
    /// [`Location::alt_frame`] reports. A bare number here means nothing
    /// without that frame.
    ///
    /// Mission storage keeps only 24 bits of it, so about +/- 83 km.
    pub alt: i32,
    /// Which datum [`Location::alt`] is measured from.
    ///
    /// Private, because upstream's representation admits a state that means
    /// nothing -- see [`AltFrame::from_bits`].
    frame: AltFrame,
    /// Loiter direction: false clockwise, true counter-clockwise. Upstream
    /// `loiter_ccw`. Nothing to do with altitude; it shares the bitfield.
    pub loiter_ccw: bool,
    /// Whether to crosstrack from the waypoint centre (false) or the tangent
    /// exit (true). Upstream `loiter_xtrack`.
    pub loiter_xtrack: bool,
}

/// What [`Location::alt`] is measured from, upstream `Location::AltFrame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AltFrame {
    /// Above mean sea level.
    #[default]
    Absolute = 0,
    /// Above the home position.
    AboveHome = 1,
    /// Above the EKF origin.
    AboveOrigin = 2,
    /// Above the terrain beneath this point.
    AboveTerrain = 3,
}

impl AltFrame {
    /// The three storage bits, upstream `relative_alt`, `terrain_alt` and
    /// `origin_alt` in that order.
    ///
    /// `AboveTerrain` sets **two** of them: upstream marks it relative as
    /// well, because a terrain altitude has not had home added either.
    #[must_use]
    pub const fn to_bits(self) -> (bool, bool, bool) {
        match self {
            Self::Absolute => (false, false, false),
            Self::AboveHome => (true, false, false),
            Self::AboveOrigin => (false, false, true),
            Self::AboveTerrain => (true, true, false),
        }
    }

    /// Recover a frame from the three storage bits, upstream
    /// `get_alt_frame`.
    ///
    /// `None` for `terrain_alt` without `relative_alt`. Upstream calls that
    /// combination impossible and enforces it with `AP_HAL::panic` -- but
    /// only on SITL, so a real vehicle reading such a mission item carries on
    /// with whatever the bits happen to say.
    ///
    /// DIVERGENCE D-023: the port reports it. Flight code cannot panic, and a
    /// check compiled out on the target is not a check.
    ///
    /// The precedence is upstream's: terrain over origin, origin over
    /// relative. Several bits set is not an error; the highest one decides.
    #[must_use]
    pub const fn from_bits(
        relative_alt: bool,
        terrain_alt: bool,
        origin_alt: bool,
    ) -> Option<Self> {
        if terrain_alt {
            if !relative_alt {
                return None;
            }
            return Some(Self::AboveTerrain);
        }
        if origin_alt {
            return Some(Self::AboveOrigin);
        }
        if relative_alt {
            return Some(Self::AboveHome);
        }
        Some(Self::Absolute)
    }
}

/// The vehicle state an altitude conversion needs, which upstream reaches
/// through `AP::ahrs()` and `AP::terrain()`.
///
/// Passed explicitly per ADR-0004. Every field is optional because every one
/// of them can genuinely be unavailable -- home unset before arming, no EKF
/// origin before a fix, no terrain data for this square -- and each absence
/// makes a different subset of conversions impossible.
#[derive(Debug, Clone, Copy, Default)]
pub struct AltContext {
    /// Home altitude, cm AMSL. `None` before home is set.
    pub home_alt_cm: Option<i32>,
    /// EKF origin altitude, cm AMSL. `None` before the origin is set.
    pub origin_alt_cm: Option<i32>,
    /// Terrain height AMSL beneath this location, cm. `None` when the terrain
    /// database has no data for it.
    pub terrain_alt_cm: Option<i32>,
}

impl Location {
    /// A position from latitude and longitude in 1e-7 degrees, at zero
    /// altitude above mean sea level.
    ///
    /// Upstream's default-constructed `Location` is all zeros, which is what
    /// this reproduces. A zero absolute altitude is a real position at sea
    /// level rather than an "unset" marker -- upstream tests for unset with
    /// [`Location::initialised`].
    #[must_use]
    pub const fn new(lat: i32, lng: i32) -> Self {
        Self {
            lat,
            lng,
            alt: 0,
            frame: AltFrame::Absolute,
            loiter_ccw: false,
            loiter_xtrack: false,
        }
    }

    /// A position with an altitude, upstream's four-argument constructor.
    #[must_use]
    pub const fn new_with_alt(lat: i32, lng: i32, alt_cm: i32, frame: AltFrame) -> Self {
        Self {
            lat,
            lng,
            alt: alt_cm,
            frame,
            loiter_ccw: false,
            loiter_xtrack: false,
        }
    }

    /// Whether this location names a position at all, upstream
    /// `initialised`.
    ///
    /// Upstream treats latitude and longitude both zero as "never set" -- a
    /// point in the Atlantic off Ghana stands in for the absence of a
    /// position. It is a real coordinate, so a vehicle genuinely there cannot
    /// be told apart from one that has no position.
    #[must_use]
    pub const fn initialised(&self) -> bool {
        self.lat != 0 || self.lng != 0
    }

    /// Set the altitude and its frame together, upstream `set_alt_cm`.
    ///
    /// Together, because neither means anything alone.
    pub const fn set_alt_cm(&mut self, alt_cm: i32, frame: AltFrame) {
        self.alt = alt_cm;
        self.frame = frame;
    }

    /// Set the altitude in metres, upstream `set_alt_m`.
    pub fn set_alt_m(&mut self, alt_m: f32, frame: AltFrame) {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream assigns alt_m*100 to an int32; the truncation toward zero is the behaviour"
        )]
        self.set_alt_cm((alt_m * 100.0) as i32, frame);
    }

    /// What [`Location::alt`] is measured from, upstream `get_alt_frame`.
    #[must_use]
    pub const fn alt_frame(&self) -> AltFrame {
        self.frame
    }

    /// The three storage bits for this location's frame. See
    /// [`AltFrame::to_bits`].
    #[must_use]
    pub const fn alt_bits(&self) -> (bool, bool, bool) {
        self.frame.to_bits()
    }

    /// Raise or lower by metres, leaving the frame alone. Upstream
    /// `offset_up_m`.
    pub fn offset_up_m(&mut self, alt_offset_m: f32) {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream adds alt_offset_m * 100 to an int32 the same way"
        )]
        let d = (alt_offset_m * 100.0) as i32;
        self.alt = self.alt.saturating_add(d);
    }

    /// This location's altitude expressed in another frame, upstream
    /// `get_alt_cm`.
    ///
    /// `None` when the conversion needs something the context does not have --
    /// home before it is set, an EKF origin before there is one, terrain data
    /// for a square the database does not cover. That is a real answer, not a
    /// failure: an altitude above a home that does not exist is not a number.
    #[must_use]
    pub fn get_alt_cm(&self, desired: AltFrame, ctx: &AltContext) -> Option<i32> {
        if desired == self.frame {
            return Some(self.alt);
        }

        // Terrain height is needed if either end of the conversion is above
        // terrain, and is looked up once.
        let terrain = if self.frame == AltFrame::AboveTerrain || desired == AltFrame::AboveTerrain {
            Some(ctx.terrain_alt_cm?)
        } else {
            None
        };

        // Everything goes through absolute.
        let alt_abs = match self.frame {
            AltFrame::Absolute => self.alt,
            AltFrame::AboveHome => self.alt.saturating_add(ctx.home_alt_cm?),
            AltFrame::AboveOrigin => self.alt.saturating_add(ctx.origin_alt_cm?),
            AltFrame::AboveTerrain => self.alt.saturating_add(terrain?),
        };

        Some(match desired {
            AltFrame::Absolute => alt_abs,
            AltFrame::AboveHome => alt_abs.saturating_sub(ctx.home_alt_cm?),
            AltFrame::AboveOrigin => alt_abs.saturating_sub(ctx.origin_alt_cm?),
            AltFrame::AboveTerrain => alt_abs.saturating_sub(terrain?),
        })
    }

    /// Re-express this location's altitude in another frame, upstream
    /// `change_alt_frame`.
    ///
    /// Returns false and leaves the location untouched when the conversion is
    /// not available.
    pub fn change_alt_frame(&mut self, desired: AltFrame, ctx: &AltContext) -> bool {
        match self.get_alt_cm(desired, ctx) {
            Some(alt_cm) => {
                self.set_alt_cm(alt_cm, desired);
                true
            }
            None => false,
        }
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
    /// Wrap a longitude into -180e7..180e7, upstream `wrap_longitude`.
    ///
    /// One wrap only. A value more than a full turn out stays out — upstream
    /// subtracts 360 degrees once rather than looping, which is enough for an
    /// offset that started in range.
    #[must_use]
    pub const fn wrap_longitude(lon: i64) -> i32 {
        if lon > 1_800_000_000 {
            (lon - 3_600_000_000) as i32
        } else if lon < -1_800_000_000 {
            (lon + 3_600_000_000) as i32
        } else {
            lon as i32
        }
    }

    /// Fold a latitude back inside -90e7..90e7, upstream `limit_lattitude`.
    ///
    /// # This reflects rather than wraps
    ///
    /// Going north past the pole gives `1800000000 - lat`, which is the right
    /// latitude for a point that carried on over the top — but the longitude
    /// is left alone, and crossing a pole flips longitude by 180 degrees. So
    /// offsetting north from 89 degrees by 200 km lands back at 89 degrees on
    /// the *same* meridian, which is where the vehicle started rather than
    /// where it was going.
    ///
    /// Reproduced rather than corrected. The name says "limit": this is a
    /// guard against nonsense coordinates, not polar navigation, and a
    /// fixed-wing vehicle reaching it has already lost. Changing it would mean
    /// diverging on a path that only runs when the inputs are already wrong.
    #[must_use]
    pub const fn limit_latitude(lat: i32) -> i32 {
        if lat > 900_000_000 {
            (1_800_000_000_i64 - lat as i64) as i32
        } else if lat < -900_000_000 {
            -((1_800_000_000_i64 + lat as i64) as i32)
        } else {
            lat
        }
    }

    /// Move by a north and east offset in metres, upstream `offset`.
    ///
    /// The east conversion divides by the longitude scale at the *midpoint*
    /// latitude of the move, not at the start — `lat + dlat/2`. Over a long
    /// northward leg the meridians converge, and taking the scale at the start
    /// would put the endpoint progressively too far east.
    pub fn offset(&mut self, ofs_north: Ftype, ofs_east: Ftype) {
        // Everything in Ftype, as upstream does: `ofs_east` is ftype and the
        // scaling factor is a constexpr float, so the product is ftype and so
        // is `longitude_scale`. Widening part-way through changes the rounding
        // before the truncation and shifts the answer by about a centimetre.
        let inv = Ftype::from(LOCATION_SCALING_FACTOR_INV);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream assigns the product straight to an int32; the truncation toward zero is the behaviour, not an accident"
        )]
        let dlat = (ofs_north * inv) as i32;
        let scale = Self::longitude_scale(self.lat.saturating_add(dlat / 2));
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream truncates the ftype quotient into an int64 here; the narrowing to int32 happens later, in wrap_longitude"
        )]
        let dlng = ((ofs_east * inv) / scale) as i64;

        self.lat = Self::limit_latitude(self.lat.saturating_add(dlat));
        self.lng = Self::wrap_longitude(dlng + i64::from(self.lng));
    }

    /// Move `distance` metres along a compass bearing, upstream
    /// `offset_bearing`.
    ///
    /// A negative distance moves backwards along the bearing, which is how the
    /// landing aim point is placed short of the runway threshold.
    pub fn offset_bearing(&mut self, bearing_deg: Ftype, distance: Ftype) {
        let b = radians(bearing_deg);
        self.offset(Real::cos(b) * distance, Real::sin(b) * distance);
    }

    /// How far along the line from `point1` to `point2` this location sits,
    /// upstream `line_path_proportion`.
    ///
    /// Zero at `point1`, one at `point2`, and outside that range when the
    /// projection falls beyond either end — callers that need it bounded
    /// clamp it themselves. Two points closer together than about 3 cm report
    /// 1.0, because the direction of a line that short is noise.
    #[must_use]
    pub fn line_path_proportion(self, point1: Self, point2: Self) -> f32 {
        let vec1 = point1.get_distance_ne(point2);
        let vec2 = point1.get_distance_ne(self);
        let dsquared = vec1.x * vec1.x + vec1.y * vec1.y;
        if dsquared < 0.001 {
            return 1.0;
        }
        (vec1.x * vec2.x + vec1.y * vec2.y) / dsquared
    }

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
        // rad_to_cd is declared `float rad_to_cd(float)`, so a caller
        // holding a double narrows on the call and the scaling happens in
        // single precision. Doing it in Ftype put a third of the bearings one
        // centidegree out, always at a rounding boundary -- and this bearing
        // is what the indecision guard compares against 120 degrees, so a
        // boundary case can flip a turn decision.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream narrows to float and then truncates to int32; \
both are upstream's and reproducing them is the point"
        )]
        let cd = rad_to_cd_f32(self.get_bearing(other).to_f64() as f32) + 0.5_f32;
        #[allow(clippy::cast_possible_truncation, reason = "upstream's int32_t() cast")]
        let out = cd as i32;
        out
    }
}
