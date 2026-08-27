//! SIM_ACC_FILE_RW / SIM_GYR_FILE_RW parameter binding, upstream
//! `SITL::accel_file_rw` / `SITL::gyro_file_rw` applied in `AP_InertialSensor_SITL`.
//! FW-011.

use crate::sitl::{SitlImuBackend, SitlInsCluster, SitlInsFileMode, SitlInsInstanceFiles};

/// File playback modes from SIM_ACC_FILE_RW and SIM_GYR_FILE_RW.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SitlInsFilePlaybackParams {
    pub accel_file_rw: i8,
    pub gyro_file_rw: i8,
}

impl SitlInsFilePlaybackParams {
    /// Build from SIM_ACC_FILE_RW / SIM_GYR_FILE_RW values.
    #[must_use]
    pub const fn from_sim_file_rw(accel_file_rw: i8, gyro_file_rw: i8) -> Self {
        Self {
            accel_file_rw,
            gyro_file_rw,
        }
    }

    /// Map upstream `INSFileMode` param value to [`SitlInsFileMode`].
    #[must_use]
    pub const fn param_to_mode(value: i8) -> SitlInsFileMode {
        match value {
            1 => SitlInsFileMode::Read,
            2 => SitlInsFileMode::Write,
            3 => SitlInsFileMode::ReadStopOnEof,
            _ => SitlInsFileMode::None,
        }
    }

    #[must_use]
    pub const fn accel_mode(&self) -> SitlInsFileMode {
        Self::param_to_mode(self.accel_file_rw)
    }

    #[must_use]
    pub const fn gyro_mode(&self) -> SitlInsFileMode {
        Self::param_to_mode(self.gyro_file_rw)
    }

    /// Apply to one SITL IMU backend's file-mode fields.
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend) {
        backend.accel_file_mode = self.accel_mode();
        backend.gyro_file_mode = self.gyro_mode();
    }

    /// Apply shared file modes to every registered backend, upstream one pair
    /// for all IMU instances.
    pub fn apply_to_cluster(&self, cluster: &mut SitlInsCluster) {
        cluster.set_file_modes(self.accel_mode(), self.gyro_mode());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;

    use crate::sitl::{SitlBodyState, SitlImuBackend};

    #[test]
    fn default_modes_are_none() {
        let params = SitlInsFilePlaybackParams::default();
        assert_eq!(params.accel_mode(), SitlInsFileMode::None);
        assert_eq!(params.gyro_mode(), SitlInsFileMode::None);
    }

    #[test]
    fn param_to_mode_matches_upstream_ins_file_mode() {
        assert_eq!(
            SitlInsFilePlaybackParams::param_to_mode(0),
            SitlInsFileMode::None
        );
        assert_eq!(
            SitlInsFilePlaybackParams::param_to_mode(1),
            SitlInsFileMode::Read
        );
        assert_eq!(
            SitlInsFilePlaybackParams::param_to_mode(2),
            SitlInsFileMode::Write
        );
        assert_eq!(
            SitlInsFilePlaybackParams::param_to_mode(3),
            SitlInsFileMode::ReadStopOnEof
        );
        assert_eq!(
            SitlInsFilePlaybackParams::param_to_mode(99),
            SitlInsFileMode::None
        );
    }

    #[test]
    fn apply_to_cluster_sets_every_backend() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();

        let params = SitlInsFilePlaybackParams::from_sim_file_rw(1, 2);
        params.apply_to_cluster(&mut cluster);

        assert_eq!(
            cluster.backend(0).unwrap().accel_file_mode,
            SitlInsFileMode::Read
        );
        assert_eq!(
            cluster.backend(0).unwrap().gyro_file_mode,
            SitlInsFileMode::Write
        );
        assert_eq!(
            cluster.backend(1).unwrap().accel_file_mode,
            SitlInsFileMode::Read
        );
        assert_eq!(
            cluster.backend(1).unwrap().gyro_file_mode,
            SitlInsFileMode::Write
        );
    }

    fn encode_ins_file_sample(v: Vector3f) -> [u8; 12] {
        let mut out = [0_u8; 12];
        for (i, component) in [v.x, v.y, v.z].into_iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&component.to_le_bytes());
        }
        out
    }

    #[test]
    fn bound_read_mode_uses_file_instead_of_kinematics() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();

        SitlInsFilePlaybackParams::from_sim_file_rw(1, 0).apply_to_cluster(&mut cluster);

        let file = encode_ins_file_sample(Vector3f::new(0.0, 0.0, -4.0));
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        cluster.timer_update(
            0,
            &state,
            &[SitlInsInstanceFiles {
                accel: Some(&file),
                gyro: None,
            }],
        );
        let backend = cluster.backend_mut(0).unwrap();
        backend.imu.update_accel();
        assert!(
            (backend.imu.accel().z + 4.0).abs() < 1e-4,
            "file playback should deliver -4 on z, got {}",
            backend.imu.accel().z
        );
    }
}
