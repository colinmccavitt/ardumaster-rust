//! GPS health flags, upstream `AP_GPS::isHealthy()` / pre-arm checks. FW-012.
//!
//! Derives per-fix health from the lag-buffered status snapshot so AHRS and
//! arming can gate on fix quality and satellite count without re-reading drivers.

use crate::status::GpsStatus;

/// Minimum satellites for a healthy fix, upstream `GPS_MIN_NSATS`.
pub const GPS_MIN_NSATS: u8 = 6;

/// Health indicators for one GPS instance, upstream `AP_GPS::isHealthy()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpsHealthFlags {
    pub have_fix: bool,
    pub has_3d_fix: bool,
    pub num_sats_ok: bool,
    pub velocity_valid: bool,
}

impl GpsHealthFlags {
    #[must_use]
    pub fn from_status(status: &GpsStatus) -> Self {
        let has_3d_fix = status.has_3d_fix();
        Self {
            have_fix: status.have_fix,
            has_3d_fix,
            num_sats_ok: status.num_sats >= GPS_MIN_NSATS,
            velocity_valid: status.have_fix && has_3d_fix,
        }
    }

    /// Whether the receiver is healthy, upstream `AP_GPS::isHealthy(instance)`.
    #[must_use]
    pub fn is_healthy(self) -> bool {
        self.have_fix && self.has_3d_fix && self.num_sats_ok
    }

    /// Whether velocity may be fused for drift correction.
    #[must_use]
    pub fn usable_for_drift(self) -> bool {
        self.is_healthy() && self.velocity_valid
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FixType;
    use ap_math::vector3::Vector3f;
    use crate::sitl::GpsFixState;

    #[test]
    fn healthy_when_3d_fix_and_enough_sats() {
        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 12,
            velocity_ned: Vector3f::new(1.0, 0.0, 0.0),
            ground_speed: 1.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 200,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            have_fix: true,
        };
        let status = GpsStatus::from_fix(&fix, 0.1);
        let health = GpsHealthFlags::from_status(&status);
        assert!(health.is_healthy());
        assert!(health.usable_for_drift());
    }

    #[test]
    fn unhealthy_when_sat_count_low() {
        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 4,
            velocity_ned: Vector3f::zero(),
            ground_speed: 0.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 200,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            have_fix: true,
        };
        let status = GpsStatus::from_fix(&fix, 0.1);
        let health = GpsHealthFlags::from_status(&status);
        assert!(!health.num_sats_ok);
        assert!(!health.is_healthy());
        assert!(!health.usable_for_drift());
    }

    #[test]
    fn unhealthy_on_2d_fix_only() {
        let fix = GpsFixState {
            fix_type: FixType::Fix2D,
            num_sats: 10,
            velocity_ned: Vector3f::zero(),
            ground_speed: 0.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 200,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 0.0,
            have_fix: true,
        };
        let status = GpsStatus::from_fix(&fix, 0.1);
        let health = GpsHealthFlags::from_status(&status);
        assert!(!health.has_3d_fix);
        assert!(!health.is_healthy());
    }
}
