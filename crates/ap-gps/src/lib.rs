//! GPS time and fix state, upstream `libraries/AP_GPS`. FW-012.
//!
//! This slice is the part of AP_GPS that is arithmetic rather than protocol:
//! the conversions between GPS time and Unix time, the BCD date and time a
//! receiver reports, the fix quality ladder, and the velocity a receiver that
//! reports only speed and course implies.
//!
//! # Two epochs and eighteen seconds
//!
//! GPS counts weeks from 1980-01-06 and milliseconds within the week. Unix
//! counts seconds from 1970-01-01. The offset between them is 3,657 days, and
//! it is *not* constant in the way it looks: GPS time does not observe leap
//! seconds, so the gap has widened by one every time a leap second was added.
//! It stands at 18 as of the pinned version, baked in as
//! [`GPS_LEAPSECONDS_MILLIS`].
//!
//! That constant is a fact about the world, not about the code, and it will be
//! wrong the next time a leap second is announced. Upstream hardcodes it, so
//! the port does too — but it is worth knowing that a vehicle running old
//! firmware after a leap second has a UTC clock a second out.
//!
//! # What this slice does not include
//!
//! The receiver drivers (u-blox, NMEA, SBF, DroneCAN — the bulk of AP_GPS by
//! line count), multi-instance blending and failover, the
//! automatic baud-rate and protocol detection, and the parameter table.
//!
//! Moving-baseline RTK yaw lives in [`moving_baseline`].
//!
//! The SITL backend lives in [`sitl`].

#![no_std]

pub mod lag_buffer;
pub mod status;
pub mod velocity;
pub mod health;
pub mod blend;
pub mod dual;
pub mod moving_baseline;
pub mod params;
pub mod sitl;

pub use lag_buffer::GpsLagBuffer;
pub use status::GpsStatus;
pub use velocity::{GpsVelocityProducer, GpsVelocitySample};
pub use health::{GpsDualHealthFlags, GpsHealthFlags, GPS_MIN_NSATS};
pub use blend::{
    GpsAutoSwitch, GpsBlendAccuracy, GpsBlendInstance, GpsBlender, GPS_BLEND_MASK_DEFAULT,
    BLEND_MASK_USE_HPOS_ACC, BLEND_MASK_USE_SPD_ACC, BLEND_MASK_USE_VPOS_ACC,
    GPS_BLENDED_INSTANCE, GPS_MAX_RECEIVERS,
};
pub use dual::{GpsDualStub, GpsInstanceTruth};
pub use moving_baseline::{
    GpsMovingBaseline, GpsYawState, GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER,
    GPS_YAW_MAX_ACCURACY_DEG, GPS_YAW_TIMEOUT_MS,
};
pub use params::{
    GpsInstanceParams, GpsParams, GPS_BLEND_MASK_PARAM_DEFAULT, GPS_TYPE_NONE,
    GPS_TYPE_SITL,
};
pub use sitl::{
    velocity_to_speed_course, GpsFixState, SitlGpsBackend, SITL_GPS_DEFAULT_LAG_SEC,
    SITL_GPS_UPDATE_MS,
};

use ap_common::{ap_mktime, Tm};
use ap_math::scalar::{radians, Real};
use ap_math::vector3::Vector3f;

/// Seconds in a GPS week, upstream `AP_SEC_PER_WEEK`.
pub const SEC_PER_WEEK: u64 = 7 * 86_400;

/// Milliseconds in a second, upstream `AP_MSEC_PER_SEC`.
pub const MSEC_PER_SEC: u64 = 1000;

/// Milliseconds in a GPS week, upstream `AP_MSEC_PER_WEEK`.
pub const MSEC_PER_WEEK: u64 = SEC_PER_WEEK * MSEC_PER_SEC;

/// Leap seconds between GPS time and UTC, in milliseconds. Upstream
/// `GPS_LEAPSECONDS_MILLIS`.
///
/// Hardcoded upstream and here. See the module docs.
pub const GPS_LEAPSECONDS_MILLIS: u64 = 18_000;

/// Milliseconds from the Unix epoch to the GPS epoch, less leap seconds.
/// Upstream `UNIX_OFFSET_MSEC`.
///
/// Upstream writes this as `17000*86400 + 52*10*AP_MSEC_PER_WEEK -
/// GPS_LEAPSECONDS_MILLIS`, which is an odd decomposition of a plain number:
/// the two terms come to 315,964,800,000 ms, the 3,657 days between
/// 1970-01-01 and 1980-01-06. Reproduced as the same total.
pub const UNIX_OFFSET_MSEC: u64 =
    17_000 * 86_400 + 52 * 10 * MSEC_PER_WEEK - GPS_LEAPSECONDS_MILLIS;

/// Seconds from the Unix epoch to the GPS epoch, upstream's local
/// `unix_to_GPS_secs`.
pub const UNIX_TO_GPS_SECS: u32 = 315_964_800;

/// How good the fix is, upstream `AP_GPS::GPS_Status`.
///
/// Ordered, and the order is load-bearing: upstream compares against
/// `GPS_OK_FIX_3D` with `>=` to mean "3D or better", so anything added must
/// keep the ladder monotonic in quality.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
#[repr(u8)]
pub enum FixType {
    /// No receiver connected or detected.
    #[default]
    NoGps = 0,
    /// Valid messages, but no lock.
    NoFix = 1,
    /// 2D lock: position but no altitude.
    Fix2D = 2,
    /// 3D lock.
    Fix3D = 3,
    /// 3D with differential corrections.
    Fix3DDgps = 4,
    /// RTK with a floating ambiguity solution — decimetres.
    Fix3DRtkFloat = 5,
    /// RTK with integer ambiguities resolved — centimetres.
    Fix3DRtkFixed = 6,
    /// A static base station.
    FixStatic = 7,
    /// Precise point positioning.
    FixPpp = 8,
}

impl FixType {
    /// The tag as it appears on the wire and in logs.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Recover a fix type from its tag, or `None` for one this version does
    /// not define.
    #[must_use]
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::NoGps,
            1 => Self::NoFix,
            2 => Self::Fix2D,
            3 => Self::Fix3D,
            4 => Self::Fix3DDgps,
            5 => Self::Fix3DRtkFloat,
            6 => Self::Fix3DRtkFixed,
            7 => Self::FixStatic,
            8 => Self::FixPpp,
            _ => return None,
        })
    }

    /// Whether the fix is 3D or better, upstream's `>= GPS_OK_FIX_3D`.
    #[must_use]
    pub fn has_3d_fix(self) -> bool {
        self >= Self::Fix3D
    }

    /// Whether there is any position at all, upstream's `>= GPS_OK_FIX_2D`.
    #[must_use]
    pub fn has_position(self) -> bool {
        self >= Self::Fix2D
    }
}

/// Unix epoch milliseconds for a GPS week and time of week, upstream
/// `AP_GPS::istate_time_to_epoch_ms`.
#[must_use]
pub const fn istate_time_to_epoch_ms(gps_week: u16, gps_ms: u32) -> u64 {
    UNIX_OFFSET_MSEC + (gps_week as u64) * MSEC_PER_WEEK + (gps_ms as u64)
}

/// GPS week and time of week from a BCD date and time, upstream
/// `AP_GPS_Backend::BCD_to_gps_time`.
///
/// `bcd_date` is `DDMMYY` read as a decimal number and `bcd_time_ms` is
/// `HHMMSSmmm` — the shape several receivers report, NMEA among them. The
/// two-digit year is taken as 2000-something, so this stops working in 2100.
/// That limit is upstream's and is not the interesting one; see D-022 for the
/// `ap_mktime` overflow underneath it.
///
/// Returns `None` when the date does not describe a real instant, which
/// upstream cannot express — its `ap_mktime` returns `(time_t)-1` and the
/// caller carries on with it.
#[must_use]
pub fn bcd_to_gps_time(bcd_date: u32, bcd_time_ms: u32) -> Option<(u16, u32)> {
    let tm = Tm {
        year: 100 + i32::try_from(bcd_date % 100).ok()?,
        mon: i32::try_from((bcd_date / 100) % 100).ok()? - 1,
        mday: i32::try_from(bcd_date / 10_000).ok()?,
        sec: i32::try_from((bcd_time_ms / 1000) % 100).ok()?,
        min: i32::try_from((bcd_time_ms / 100_000) % 100).ok()?,
        hour: i32::try_from(bcd_time_ms / 10_000_000).ok()?,
    };
    let msec = bcd_time_ms % 1000;

    let unix_time = ap_mktime(&tm)?;

    let leap_seconds_unix = GPS_LEAPSECONDS_MILLIS / MSEC_PER_SEC;
    // Upstream narrows to uint32 here. Reproduced: the value fits until the
    // same 2106 horizon the two-digit year already rules out.
    let ret = u32::try_from(
        unix_time + i64::try_from(leap_seconds_unix).ok()? - i64::from(UNIX_TO_GPS_SECS),
    )
    .ok()?;

    let gps_week = u16::try_from(u64::from(ret) / SEC_PER_WEEK).ok()?;
    let gps_time_ms = u32::try_from((u64::from(ret) % SEC_PER_WEEK) * MSEC_PER_SEC).ok()? + msec;
    Some((gps_week, gps_time_ms))
}

/// Earth-frame velocity implied by ground speed and course, upstream
/// `AP_GPS_Backend::fill_3d_velocity`.
///
/// For receivers that report speed over the ground and a heading but no
/// velocity vector. The vertical component is zero and the caller should mark
/// vertical velocity unavailable — a flat zero is not a measurement, and an
/// estimator told otherwise would trust it.
///
/// `ground_course` is degrees; the returned vector is North-East-Down in m/s.
#[must_use]
pub fn fill_3d_velocity(ground_speed: f32, ground_course_deg: f32) -> Vector3f {
    let heading = radians(ground_course_deg);
    Vector3f::new(
        ground_speed * Real::cos(heading),
        ground_speed * Real::sin(heading),
        0.0,
    )
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "the vertical component is exactly zero because it is not measured; \nan epsilon there would accept a small nonzero value, which is precisely the thing that \nmust not appear"
    )]

    use super::*;

    /// The offset between the two epochs is 3,657 days, less leap seconds.
    #[test]
    fn the_epoch_offset_is_the_gap_between_1970_and_1980() {
        const DAYS: u64 = 3657;
        assert_eq!(
            UNIX_OFFSET_MSEC + GPS_LEAPSECONDS_MILLIS,
            DAYS * 86_400 * 1000,
            "upstream's decomposition should come to 3657 days"
        );
    }

    /// GPS week zero, time zero is the GPS epoch: 1980-01-06 00:00:00 UTC,
    /// which is 315,964,800 seconds after the Unix epoch — less the leap
    /// seconds GPS does not count.
    #[test]
    fn week_zero_is_the_gps_epoch() {
        let ms = istate_time_to_epoch_ms(0, 0);
        assert_eq!(ms, 315_964_800_000 - GPS_LEAPSECONDS_MILLIS);
    }

    /// A week later is a week later.
    #[test]
    fn a_week_advances_by_a_week() {
        let a = istate_time_to_epoch_ms(1000, 0);
        let b = istate_time_to_epoch_ms(1001, 0);
        assert_eq!(b - a, MSEC_PER_WEEK);
    }

    /// The week counter is 16-bit and the multiplication must not wrap.
    /// `gps_week * MSEC_PER_WEEK` at week 65535 is 3.96e13, far past what 32
    /// bits hold — upstream is safe because its constants are `ULL`, and this
    /// pins that.
    #[test]
    fn a_large_week_number_does_not_wrap() {
        let ms = istate_time_to_epoch_ms(u16::MAX, 0);
        let expected = UNIX_OFFSET_MSEC + 65_535 * MSEC_PER_WEEK;
        assert_eq!(ms, expected);
        assert!(ms > u64::from(u32::MAX), "the result exceeds 32 bits");
    }

    /// A known instant, checked end to end: 2024-02-29 12:34:56.789 UTC.
    #[test]
    fn a_known_date_converts_to_the_right_week_and_time() {
        // DDMMYY = 290224, HHMMSSmmm = 123456789
        let (week, ms) = bcd_to_gps_time(290_224, 123_456_789).expect("a real date");

        // Convert back through the epoch conversion and compare with the Unix
        // time the same instant has.
        let epoch_ms = istate_time_to_epoch_ms(week, ms);
        // 2024-02-29 12:34:56 UTC = 1709210096 s
        assert_eq!(epoch_ms, 1_709_210_096_000 + 789);
    }

    /// The milliseconds ride along untouched.
    #[test]
    fn the_millisecond_field_is_carried_through() {
        let (_, ms_a) = bcd_to_gps_time(290_224, 123_456_000).expect("real");
        let (_, ms_b) = bcd_to_gps_time(290_224, 123_456_999).expect("real");
        assert_eq!(ms_b - ms_a, 999);
    }

    /// Time of week resets at the week boundary rather than running on.
    #[test]
    fn the_time_of_week_wraps_at_the_boundary() {
        for (date, time) in [(290_224_u32, 123_456_789_u32), (10_324, 0)] {
            let (_, ms) = bcd_to_gps_time(date, time).expect("real");
            assert!(
                u64::from(ms) < MSEC_PER_WEEK,
                "time of week {ms} should be inside a week"
            );
        }
    }

    /// The fix ladder is ordered by quality, which upstream relies on when it
    /// tests `>= GPS_OK_FIX_3D`.
    #[test]
    fn the_fix_ladder_is_ordered_by_quality() {
        assert!(FixType::NoGps < FixType::NoFix);
        assert!(FixType::NoFix < FixType::Fix2D);
        assert!(FixType::Fix2D < FixType::Fix3D);
        assert!(FixType::Fix3D < FixType::Fix3DRtkFixed);

        assert!(!FixType::Fix2D.has_3d_fix());
        assert!(FixType::Fix3D.has_3d_fix());
        assert!(FixType::Fix3DRtkFixed.has_3d_fix());
        assert!(FixType::FixPpp.has_3d_fix());

        assert!(!FixType::NoFix.has_position());
        assert!(FixType::Fix2D.has_position());
    }

    #[test]
    fn fix_tags_round_trip() {
        for v in 0..=8_u8 {
            let f = FixType::from_u8(v).expect("defined");
            assert_eq!(f.as_u8(), v);
        }
        assert_eq!(FixType::from_u8(9), None);
    }

    /// Course is measured clockwise from north, so due east is +Y.
    #[test]
    fn velocity_points_where_the_course_says() {
        let north = fill_3d_velocity(10.0, 0.0);
        assert!((north.x - 10.0).abs() < 1e-4 && north.y.abs() < 1e-4);

        let east = fill_3d_velocity(10.0, 90.0);
        assert!(east.x.abs() < 1e-3 && (east.y - 10.0).abs() < 1e-4);

        let south = fill_3d_velocity(10.0, 180.0);
        assert!((south.x + 10.0).abs() < 1e-4 && south.y.abs() < 1e-3);

        let west = fill_3d_velocity(10.0, 270.0);
        assert!(west.x.abs() < 1e-3 && (west.y + 10.0).abs() < 1e-4);
    }

    /// The magnitude is the ground speed, whatever the course.
    #[test]
    fn velocity_magnitude_is_the_ground_speed() {
        for course in (0..360).step_by(17) {
            let v = fill_3d_velocity(7.5, course as f32);
            let mag = (v.x * v.x + v.y * v.y).sqrt();
            assert!((mag - 7.5).abs() < 1e-3, "at {course} deg: {mag}");
            assert_eq!(v.z, 0.0, "vertical velocity is not measured here");
        }
    }
}
