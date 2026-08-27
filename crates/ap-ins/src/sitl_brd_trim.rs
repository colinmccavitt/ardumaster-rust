//! SIM_BRD_TRIM parameter binding, upstream `SITL::board_trim` applied in
//! `AP_InertialSensor_SITL` to both accel and gyro. FW-011.

use ap_math::vector3::Vector3f;

use crate::sitl::{SitlImuBackend, SitlInsCluster};

/// Board mounting trim from SIM_BRD_TRIM (roll, pitch, yaw in radians).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlBrdTrimParams {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
}

impl Default for SitlBrdTrimParams {
    fn default() -> Self {
        Self {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
        }
    }
}

impl SitlBrdTrimParams {
    /// Build from SIM_BRD_TRIM_X/Y/Z values (radians).
    #[must_use]
    pub const fn from_radians(roll_rad: f32, pitch_rad: f32, yaw_rad: f32) -> Self {
        Self {
            roll_rad,
            pitch_rad,
            yaw_rad,
        }
    }

    /// Euler trim as a vector, upstream `sitl->board_trim.get()`.
    #[must_use]
    pub fn vector(&self) -> Vector3f {
        Vector3f::new(self.roll_rad, self.pitch_rad, self.yaw_rad)
    }

    /// Apply to one SITL IMU backend's [`SitlImuBackend::board_trim`].
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend) {
        backend.board_trim = self.vector();
    }

    /// Apply shared trim to every registered backend, upstream one `board_trim`
    /// for all IMU instances.
    pub fn apply_to_cluster(&self, cluster: &mut SitlInsCluster) {
        cluster.set_board_trim(self.vector());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::{SitlBodyState, SitlImuBackend, SitlImuCalibration, sitl_accel_sample};

    #[test]
    fn default_trim_is_zero() {
        assert_eq!(SitlBrdTrimParams::default().vector(), Vector3f::zero());
    }

    #[test]
    fn apply_to_cluster_sets_every_backend() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();

        let params = SitlBrdTrimParams::from_radians(0.0, 0.12, 0.0);
        params.apply_to_cluster(&mut cluster);

        assert_eq!(cluster.backend(0).unwrap().board_trim, params.vector());
        assert_eq!(cluster.backend(1).unwrap().board_trim, params.vector());
    }

    #[test]
    fn bound_trim_tilts_accel_sample() {
        let params = SitlBrdTrimParams::from_radians(0.0, 0.1, 0.0);
        let mut backend = SitlImuBackend::new(1000, 1000);
        params.apply_to_backend(&mut backend);

        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let sample = sitl_accel_sample(&state, &SitlImuCalibration::default(), backend.board_trim);
        assert!(
            sample.x.abs() > 0.5,
            "pitch trim should leak gravity into x, got {}",
            sample.x
        );
    }
}
