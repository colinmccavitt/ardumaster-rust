//! GPS health flags, upstream `AP_GPS::isHealthy()` / pre-arm checks. FW-012.
//!
//! Derives per-fix health from the lag-buffered status snapshot so AHRS and
//! arming can gate on fix quality and satellite count without re-reading drivers.

use crate::status::GpsStatus;

/// Minimum satellites for a healthy fix, upstream `GPS_MIN_NSATS`.
pub const GPS_MIN_NSATS: u8 = 6;

/// Maximum fix age before health fails, upstream `AP_GPS` timeout.
pub const GPS_FIX_TIMEOUT_MS: u32 = 4000;

/// Per-instance dual-GPS health, upstream multi-receiver `isHealthy()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpsDualHealthFlags {
    pub per_instance: [GpsHealthFlags; 2],
    pub instance_count: u8,
    pub primary: u8,
    pub have_gps_yaw: [bool; 2],
    pub rtk_yaw_fresh: bool,
}

impl GpsDualHealthFlags {
    #[must_use]
    pub fn any_healthy(self) -> bool {
        self.per_instance[..self.instance_count as usize]
            .iter()
            .any(|h| h.is_healthy())
    }

    #[must_use]
    pub fn primary_healthy(self) -> bool {
        let i = self.primary as usize;
        i < self.instance_count as usize && self.per_instance[i].is_healthy()
    }

    #[must_use]
    pub fn output_healthy(self, active: u8) -> bool {
        if active >= self.instance_count {
            return self.any_healthy();
        }
        self.per_instance[active as usize].is_healthy()
    }
}

/// Health indicators for one GPS instance, upstream `AP_GPS::isHealthy()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpsHealthFlags {
    pub have_fix: bool,
    pub has_3d_fix: bool,
    pub num_sats_ok: bool,
    pub velocity_valid: bool,
    pub fix_fresh: bool,
}

impl GpsHealthFlags {
    #[must_use]
    pub fn from_status(status: &GpsStatus) -> Self {
        Self::from_status_at(status, status.last_fix_time_ms)
    }

    #[must_use]
    pub fn from_status_at(status: &GpsStatus, now_ms: u32) -> Self {
        Self::from_status_at_min(status, now_ms, GPS_MIN_NSATS)
    }

    #[must_use]
    pub fn from_status_at_min(status: &GpsStatus, now_ms: u32, min_nsats: u8) -> Self {
        let has_3d_fix = status.has_3d_fix();
        let fix_fresh = status.have_fix
            && now_ms.wrapping_sub(status.last_fix_time_ms) <= GPS_FIX_TIMEOUT_MS;
        Self {
            have_fix: status.have_fix,
            has_3d_fix,
            num_sats_ok: status.num_sats >= min_nsats,
            velocity_valid: status.have_fix && has_3d_fix,
            fix_fresh,
        }
    }

    /// Whether the receiver is healthy, upstream `AP_GPS::isHealthy(instance)`.
    #[must_use]
    pub fn is_healthy(self) -> bool {
        self.have_fix && self.has_3d_fix && self.num_sats_ok && self.fix_fresh
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

    #[test]
    fn unhealthy_when_fix_is_stale() {
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
        let fresh = GpsHealthFlags::from_status_at(&status, 200);
        assert!(fresh.fix_fresh);
        assert!(fresh.is_healthy());
        let stale = GpsHealthFlags::from_status_at(&status, 5000);
        assert!(!stale.fix_fresh);
        assert!(!stale.is_healthy());
        assert!(!stale.usable_for_drift());
    }

    #[test]
    fn param_min_nsats_lowers_sat_gate() {
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
        let default_gate = GpsHealthFlags::from_status_at(&status, 200);
        assert!(!default_gate.num_sats_ok);
        let relaxed = GpsHealthFlags::from_status_at_min(&status, 200, 4);
        assert!(relaxed.num_sats_ok);
        assert!(relaxed.is_healthy());
    }
}
