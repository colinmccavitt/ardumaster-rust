//! SITL GPS backend, upstream `AP_GPS_SITL::read()`. FW-012.
//!
//! Produces fix state from SITL ground-truth velocity at the 200 ms rate limit
//! upstream uses. No noise model — SITL reads `speedN/speedE/speedD` directly.

use ap_math::scalar::{degrees, wrap_360, Real};
use ap_math::vector3::Vector3f;

use crate::lag_buffer::GpsLagBuffer;
use crate::FixType;

/// Minimum interval between GPS updates, upstream `AP_GPS_SITL::read()`.
pub const SITL_GPS_UPDATE_MS: u32 = 200;

/// Default lag when no parameter override, upstream `AP_GPS::get_lag()`.
pub const SITL_GPS_DEFAULT_LAG_SEC: f32 = 0.1;

/// GPS fix state from one successful backend read.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsFixState {
    pub fix_type: FixType,
    pub num_sats: u8,
    pub velocity_ned: Vector3f,
    pub ground_speed: f32,
    pub ground_course_deg: f32,
    pub last_fix_time_ms: u32,
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
    pub have_fix: bool,
}

#[must_use]
pub fn velocity_to_speed_course(velocity: Vector3f) -> (f32, f32) {
    let ground_course_deg = wrap_360(degrees(Real::atan2(velocity.y, velocity.x)));
    let ground_speed = velocity.xy().length();
    (ground_speed, ground_course_deg)
}

#[derive(Debug, Clone, Copy)]
pub struct SitlGpsBackend {
    last_update_ms: u32,
    state: GpsFixState,
    lag_buffer: GpsLagBuffer,
}

impl Default for SitlGpsBackend {
    fn default() -> Self {
        Self {
            last_update_ms: 0,
            state: GpsFixState::default(),
            lag_buffer: GpsLagBuffer::new(SITL_GPS_DEFAULT_LAG_SEC),
        }
    }
}

impl SitlGpsBackend {
    #[must_use]
    pub const fn state(&self) -> &GpsFixState {
        &self.state
    }

    #[must_use]
    pub const fn lag_sec(&self) -> f32 {
        SITL_GPS_DEFAULT_LAG_SEC
    }

    #[must_use]
    pub fn delayed_state(&self, now_ms: u32) -> GpsFixState {
        self.lag_buffer.delayed_fix(now_ms)
    }

    pub fn read(
        &mut self,
        velocity_ned: Vector3f,
        latitude_deg: f32,
        longitude_deg: f32,
        altitude_m: f32,
        now_ms: u32,
    ) -> bool {
        if now_ms.wrapping_sub(self.last_update_ms) < SITL_GPS_UPDATE_MS {
            return false;
        }
        self.last_update_ms = now_ms;

        let (ground_speed, ground_course_deg) = velocity_to_speed_course(velocity_ned);

        let fix = GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 15,
            velocity_ned,
            ground_speed,
            ground_course_deg,
            last_fix_time_ms: now_ms,
            latitude_deg,
            longitude_deg,
            altitude_m,
            have_fix: true,
        };
        self.lag_buffer.push(fix);
        self.state = fix;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_limit_gates_first_fix_until_200ms() {
        let mut gps = SitlGpsBackend::default();
        let vel = Vector3f::new(10.0, 0.0, 0.0);
        assert!(!gps.read(vel, 51.0, -0.1, 100.0, 100));
        assert!(!gps.state().have_fix);
        assert!(gps.read(vel, 51.0, -0.1, 100.0, 200));
        assert!(gps.state().have_fix);
        assert!((gps.state().ground_speed - 10.0).abs() < 1e-4);
    }

    #[test]
    fn velocity_to_speed_course_east_is_ninety_degrees() {
        let vel = Vector3f::new(0.0, 12.0, 0.0);
        let (speed, course) = velocity_to_speed_course(vel);
        assert!((speed - 12.0).abs() < 1e-4);
        assert!((course - 90.0).abs() < 1e-3);
    }

    #[test]
    fn sitl_fix_is_always_3d_with_fifteen_sats() {
        let mut gps = SitlGpsBackend::default();
        assert!(gps.read(Vector3f::new(1.0, 1.0, 0.0), 0.0, 0.0, 0.0, 200));
        assert_eq!(gps.state().fix_type, FixType::Fix3D);
        assert_eq!(gps.state().num_sats, 15);
    }

    #[test]
    fn default_lag_is_one_tenth_second() {
        assert!((SitlGpsBackend::default().lag_sec() - 0.1).abs() < 1e-6);
    }

    #[test]
    fn backend_delayed_state_returns_previous_fix() {
        let mut gps = SitlGpsBackend::default();
        let vel_a = Vector3f::new(10.0, 0.0, 0.0);
        let vel_b = Vector3f::new(20.0, 0.0, 0.0);
        assert!(gps.read(vel_a, 51.0, -0.1, 100.0, 200));
        assert!(gps.read(vel_b, 51.0, -0.1, 100.0, 400));
        let delayed = gps.delayed_state(450);
        assert!((delayed.velocity_ned.x - 10.0).abs() < 1e-4);
    }
}
