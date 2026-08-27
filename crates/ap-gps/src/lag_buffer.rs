//! GPS measurement lag buffer, upstream `AP_GPS` timing compensation. FW-012.
//!
//! Stores the previous fix so consumers can read velocity and position delayed
//! by [`GpsLagBuffer::lag_sec`], matching upstream `get_lag()` behaviour.

use crate::sitl::GpsFixState;

/// Default lag when no parameter override, upstream `AP_GPS::get_lag()` for SITL.
pub const DEFAULT_LAG_SEC: f32 = 0.1;

/// Ring of two fixes — enough for 0.1 s lag at the 200 ms SITL update rate.
#[derive(Debug, Clone, Copy)]
pub struct GpsLagBuffer {
    current: GpsFixState,
    previous: GpsFixState,
    have_previous: bool,
    lag_sec: f32,
}

impl Default for GpsLagBuffer {
    fn default() -> Self {
        Self::new(DEFAULT_LAG_SEC)
    }
}

impl GpsLagBuffer {
    #[must_use]
    pub fn new(lag_sec: f32) -> Self {
        Self {
            current: GpsFixState::default(),
            previous: GpsFixState::default(),
            have_previous: false,
            lag_sec,
        }
    }

    #[must_use]
    pub const fn lag_sec(&self) -> f32 {
        self.lag_sec
    }

    pub fn push(&mut self, fix: GpsFixState) {
        if fix.have_fix && self.current.have_fix {
            self.previous = self.current;
            self.have_previous = true;
        }
        self.current = fix;
    }

    #[must_use]
    pub fn delayed_fix(&self, now_ms: u32) -> GpsFixState {
        if !self.have_previous || !self.current.have_fix {
            return self.current;
        }
        let lag_ms = (self.lag_sec * 1000.0) as u32;
        if now_ms.wrapping_sub(self.current.last_fix_time_ms) < lag_ms {
            self.previous
        } else {
            self.current
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;
    use crate::FixType;

    fn fix(vel_x: f32, time_ms: u32) -> GpsFixState {
        GpsFixState {
            fix_type: FixType::Fix3D,
            num_sats: 15,
            velocity_ned: Vector3f::new(vel_x, 0.0, 0.0),
            ground_speed: vel_x.abs(),
            ground_course_deg: 0.0,
            last_fix_time_ms: time_ms,
            latitude_deg: 51.0,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            have_fix: true,
        }
    }

    #[test]
    fn first_fix_has_no_delayed_predecessor() {
        let mut buf = GpsLagBuffer::new(0.1);
        buf.push(fix(10.0, 200));
        let delayed = buf.delayed_fix(250);
        assert!((delayed.velocity_ned.x - 10.0).abs() < 1e-4);
    }

    #[test]
    fn delayed_fix_returns_previous_inside_lag_window() {
        let mut buf = GpsLagBuffer::new(0.1);
        buf.push(fix(10.0, 200));
        buf.push(fix(20.0, 400));
        let delayed = buf.delayed_fix(450);
        assert!((delayed.velocity_ned.x - 10.0).abs() < 1e-4);
    }

    #[test]
    fn delayed_fix_catches_up_after_lag_elapses() {
        let mut buf = GpsLagBuffer::new(0.1);
        buf.push(fix(10.0, 200));
        buf.push(fix(20.0, 400));
        let delayed = buf.delayed_fix(600);
        assert!((delayed.velocity_ned.x - 20.0).abs() < 1e-4);
    }
}
