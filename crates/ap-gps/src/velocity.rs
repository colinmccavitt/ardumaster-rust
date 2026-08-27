//! GPS velocity producer, upstream `AP_GPS::state().velocity`. FW-012.
//!
//! Publishes the lag-buffered NED velocity vector for AHRS drift correction,
//! replacing the 2D ground-speed/course reconstruction from yaw samples.

use ap_math::vector3::Vector3f;

use crate::sitl::GpsFixState;
use crate::status::GpsStatus;

/// Lag-buffered GPS velocity for drift consumers, upstream `state.velocity`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GpsVelocitySample {
    pub velocity_ned: Vector3f,
    pub have_velocity: bool,
    pub last_fix_time_ms: u32,
}

impl GpsVelocitySample {
    #[must_use]
    pub fn from_fix(fix: &GpsFixState) -> Self {
        Self {
            velocity_ned: fix.velocity_ned,
            have_velocity: fix.have_fix,
            last_fix_time_ms: fix.last_fix_time_ms,
        }
    }

    #[must_use]
    pub fn from_status(status: &GpsStatus) -> Self {
        Self {
            velocity_ned: status.velocity_ned,
            have_velocity: status.have_fix,
            last_fix_time_ms: status.last_fix_time_ms,
        }
    }
}

/// Produces lag-buffered velocity from fix state, upstream fix producer read path.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsVelocityProducer;

impl GpsVelocityProducer {
    #[must_use]
    pub fn publish(fix: &GpsFixState) -> GpsVelocitySample {
        GpsVelocitySample::from_fix(fix)
    }

    #[must_use]
    pub fn publish_status(status: &GpsStatus) -> GpsVelocitySample {
        GpsVelocitySample::from_status(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FixType;

    #[test]
    fn producer_reflects_full_ned_velocity() {
        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 15,
            velocity_ned: Vector3f::new(10.0, 2.0, -3.0),
            ground_speed: 10.2,
            ground_course_deg: 11.3,
            last_fix_time_ms: 400,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            have_fix: true,
        };
        let sample = GpsVelocityProducer::publish(&fix);
        assert!(sample.have_velocity);
        assert!((sample.velocity_ned.x - 10.0).abs() < 1e-4);
        assert!((sample.velocity_ned.z - (-3.0)).abs() < 1e-4);
        assert_eq!(sample.last_fix_time_ms, 400);
    }

    #[test]
    fn producer_from_status_matches_fix() {
        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 12,
            velocity_ned: Vector3f::new(5.0, -1.0, 0.5),
            ground_speed: 5.1,
            ground_course_deg: 349.0,
            last_fix_time_ms: 600,
            latitude_deg: 47.0,
            longitude_deg: -122.0,
            altitude_m: 50.0,
            have_fix: true,
        };
        let status = GpsStatus::from_fix(&fix, 0.1);
        let from_fix = GpsVelocityProducer::publish(&fix);
        let from_status = GpsVelocityProducer::publish_status(&status);
        assert_eq!(from_fix, from_status);
    }
}
