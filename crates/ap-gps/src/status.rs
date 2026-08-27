//! GPS status snapshot for vehicle consumers, upstream `AP_GPS::status()`. FW-012.
//!
//! Publishes fix quality, satellite count, lag, and the lag-buffered velocity
//! vector so AHRS and navigation can read one struct instead of yaw-only fields.

use ap_math::vector3::Vector3f;

use crate::FixType;
use crate::sitl::GpsFixState;

/// Vehicle-visible GPS status, upstream `AP_GPS` state fields exposed to AHRS.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsStatus {
    pub fix_type: FixType,
    pub num_sats: u8,
    pub have_fix: bool,
    pub lag_sec: f32,
    pub velocity_ned: Vector3f,
    pub ground_speed: f32,
    pub ground_course_deg: f32,
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
    pub last_fix_time_ms: u32,
}

impl GpsStatus {
    #[must_use]
    pub fn from_fix(fix: &GpsFixState, lag_sec: f32) -> Self {
        Self {
            fix_type: fix.fix_type,
            num_sats: fix.num_sats,
            have_fix: fix.have_fix,
            lag_sec,
            velocity_ned: fix.velocity_ned,
            ground_speed: fix.ground_speed,
            ground_course_deg: fix.ground_course_deg,
            latitude_deg: fix.latitude_deg,
            longitude_deg: fix.longitude_deg,
            altitude_m: fix.altitude_m,
            last_fix_time_ms: fix.last_fix_time_ms,
        }
    }

    #[must_use]
    pub fn has_3d_fix(self) -> bool {
        self.fix_type.has_3d_fix()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FixType;

    #[test]
    fn status_reflects_lag_buffered_fix() {
        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 15,
            velocity_ned: Vector3f::new(12.0, 3.0, -1.0),
            ground_speed: 12.37,
            ground_course_deg: 14.0,
            last_fix_time_ms: 400,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            have_fix: true,
        };
        let status = GpsStatus::from_fix(&fix, 0.1);
        assert!(status.has_3d_fix());
        assert_eq!(status.num_sats, 15);
        assert!((status.velocity_ned.x - 12.0).abs() < 1e-4);
        assert!((status.lag_sec - 0.1).abs() < 1e-6);
    }
}
