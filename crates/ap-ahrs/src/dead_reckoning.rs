//! Position dead-reckoning when GPS is unavailable, upstream
//! `AP_AHRS_DCM::drift_correction` `_position_offset_*` integration.

use ap_math::vector3::Vector3f;

/// NE offset from the last GPS fix, upstream `_position_offset_north/east`.
#[derive(Debug, Clone, Copy, Default)]
pub struct DeadReckoningPosition {
    pub offset_north_m: f32,
    pub offset_east_m: f32,
    pub have_position: bool,
    pub last_lat_e7: i32,
    pub last_lng_e7: i32,
    pub last_pos_ms: u32,
}

impl DeadReckoningPosition {
    /// Record a GPS fix and reset offsets, upstream `have_gps()` branch.
    pub fn on_gps_fix(&mut self, lat_e7: i32, lng_e7: i32, now_ms: u32) {
        self.last_lat_e7 = lat_e7;
        self.last_lng_e7 = lng_e7;
        self.last_pos_ms = now_ms;
        self.offset_north_m = 0.0;
        self.offset_east_m = 0.0;
        self.have_position = true;
    }

    /// Integrate ground velocity while GPS is absent, upstream no-GPS branch.
    pub fn integrate(&mut self, velocity: Vector3f, dt: f32, have_gps: bool) {
        if have_gps {
            return;
        }
        if self.have_position && dt > 0.0 {
            self.offset_north_m += velocity.x * dt;
            self.offset_east_m += velocity.y * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_fix_resets_offsets() {
        let mut pos = DeadReckoningPosition {
            offset_north_m: 12.0,
            offset_east_m: -3.0,
            have_position: false,
            ..DeadReckoningPosition::default()
        };
        pos.on_gps_fix(473_582_100, -122_234_567, 1000);
        assert!(pos.have_position);
        assert_eq!(pos.offset_north_m, 0.0);
        assert_eq!(pos.offset_east_m, 0.0);
    }

    #[test]
    fn integrates_velocity_without_gps() {
        let mut pos = DeadReckoningPosition::default();
        pos.on_gps_fix(0, 0, 0);
        pos.integrate(Vector3f::new(10.0, 5.0, 0.0), 0.2, false);
        assert!((pos.offset_north_m - 2.0).abs() < 1e-5);
        assert!((pos.offset_east_m - 1.0).abs() < 1e-5);
    }
}
