//! Moving-baseline RTK stub, upstream dual-GPS yaw / base-rover pair. FW-012.

use ap_math::scalar::{degrees, wrap_360, Real};

pub const GPS_TYPE_UBLOX_RTK_BASE: u8 = 25;
pub const GPS_TYPE_UBLOX_RTK_ROVER: u8 = 26;
pub const GPS_YAW_TIMEOUT_MS: u32 = 15_000;
pub const GPS_YAW_DEFAULT_ACCURACY_DEG: f32 = 10.0;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GpsYawState {
    pub have_gps_yaw: bool,
    pub gps_yaw_deg: f32,
    pub gps_yaw_accuracy_deg: f32,
    pub gps_yaw_time_ms: u32,
    pub have_gps_yaw_accuracy: bool,
}

impl GpsYawState {
    #[must_use]
    pub const fn from_heading(heading_deg: f32, now_ms: u32) -> Self {
        Self {
            have_gps_yaw: true,
            gps_yaw_deg: heading_deg,
            gps_yaw_accuracy_deg: GPS_YAW_DEFAULT_ACCURACY_DEG,
            gps_yaw_time_ms: now_ms,
            have_gps_yaw_accuracy: false,
        }
    }

    #[must_use]
    pub fn yaw_fresh(self, now_ms: u32) -> bool {
        self.have_gps_yaw && now_ms.wrapping_sub(self.gps_yaw_time_ms) <= GPS_YAW_TIMEOUT_MS
    }
}

#[must_use]
pub const fn is_rtk_base(gps_type: u8) -> bool {
    gps_type == GPS_TYPE_UBLOX_RTK_BASE
}

#[must_use]
pub const fn is_rtk_rover(gps_type: u8) -> bool {
    gps_type == GPS_TYPE_UBLOX_RTK_ROVER
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GpsMovingBaseline {
    pub gps1_type: u8,
    pub gps2_type: u8,
    pub rover_yaw: GpsYawState,
}

impl GpsMovingBaseline {
    #[must_use]
    pub const fn from_types(gps1_type: u8, gps2_type: u8) -> Self {
        Self {
            gps1_type,
            gps2_type,
            rover_yaw: GpsYawState {
                have_gps_yaw: false,
                gps_yaw_deg: 0.0,
                gps_yaw_accuracy_deg: GPS_YAW_DEFAULT_ACCURACY_DEG,
                gps_yaw_time_ms: 0,
                have_gps_yaw_accuracy: false,
            },
        }
    }

    #[must_use]
    pub const fn using_moving_base(self) -> bool {
        is_rtk_base(self.gps1_type)
            || is_rtk_base(self.gps2_type)
            || is_rtk_rover(self.gps1_type)
            || is_rtk_rover(self.gps2_type)
    }

    #[must_use]
    pub fn base_instance(self) -> Option<u8> {
        if is_rtk_base(self.gps1_type) && is_rtk_rover(self.gps2_type) {
            Some(0)
        } else if is_rtk_base(self.gps2_type) && is_rtk_rover(self.gps1_type) {
            Some(1)
        } else {
            None
        }
    }

    #[must_use]
    pub fn rover_instance(self) -> Option<u8> {
        self.base_instance().map(|base| base ^ 1)
    }

    #[must_use]
    pub fn heading_from_positions(
        base_lat_deg: f32,
        base_lon_deg: f32,
        rover_lat_deg: f32,
        rover_lon_deg: f32,
    ) -> f32 {
        let dlat = rover_lat_deg - base_lat_deg;
        let dlon = (rover_lon_deg - base_lon_deg)
            * Real::cos(ap_math::scalar::radians(base_lat_deg));
        wrap_360(degrees(Real::atan2(dlon, dlat)))
    }

    pub fn update_rover_yaw_from_positions(
        &mut self,
        base_lat_deg: f32,
        base_lon_deg: f32,
        rover_lat_deg: f32,
        rover_lon_deg: f32,
        now_ms: u32,
    ) {
        if self.base_instance().is_none() {
            return;
        }
        self.rover_yaw = GpsYawState::from_heading(
            Self::heading_from_positions(base_lat_deg, base_lon_deg, rover_lat_deg, rover_lon_deg),
            now_ms,
        );
    }

    #[must_use]
    pub fn gps_yaw_deg(self, instance: u8, now_ms: u32) -> Option<(f32, f32, u32)> {
        let rover = self.rover_instance()?;
        let query = if self.base_instance() == Some(instance) {
            rover
        } else {
            instance
        };
        if query != rover || !self.rover_yaw.have_gps_yaw {
            return None;
        }
        let accuracy = if self.rover_yaw.have_gps_yaw_accuracy {
            self.rover_yaw.gps_yaw_accuracy_deg
        } else {
            GPS_YAW_DEFAULT_ACCURACY_DEG
        };
        let _ = now_ms;
        Some((self.rover_yaw.gps_yaw_deg, accuracy, self.rover_yaw.gps_yaw_time_ms))
    }

    #[must_use]
    pub fn have_gps_yaw(self, instance: u8) -> bool {
        self.rover_instance() == Some(instance) && self.rover_yaw.have_gps_yaw
    }

    #[must_use]
    pub fn rover_yaw_pre_arm_ok(self, now_ms: u32) -> bool {
        self.rover_instance()
            .is_none_or(|_| self.rover_yaw.yaw_fresh(now_ms))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_base_rover_pair_on_instance_zero() {
        let mb = GpsMovingBaseline::from_types(GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER);
        assert!(mb.using_moving_base());
        assert_eq!(mb.base_instance(), Some(0));
        assert_eq!(mb.rover_instance(), Some(1));
    }

    #[test]
    fn gps_yaw_deg_redirects_base_query_to_rover() {
        let mut mb = GpsMovingBaseline::from_types(GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER);
        mb.rover_yaw = GpsYawState::from_heading(90.0, 500);
        let (yaw, _acc, t) = mb.gps_yaw_deg(0, 500).expect("yaw");
        assert!((yaw - 90.0).abs() < 1e-3);
        assert_eq!(t, 500);
    }

    #[test]
    fn stale_rover_yaw_fails_pre_arm() {
        let mut mb = GpsMovingBaseline::from_types(GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER);
        mb.rover_yaw = GpsYawState::from_heading(45.0, 100);
        assert!(mb.rover_yaw_pre_arm_ok(500));
        assert!(!mb.rover_yaw_pre_arm_ok(20_000));
    }
}
