//! SIM_ACCEL_FAIL_MSK / SIM_GYRO_FAIL_MSK parameter binding, upstream
//! `SITL::accel_fail` / `SITL::gyro_fail` applied in `AP_InertialSensor_SITL`.
//! FW-011.

use crate::sitl::{SitlImuBackend, SitlInsCluster};

/// Instance fail masks from SIM_ACCEL_FAIL_MSK and SIM_GYRO_FAIL_MSK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SitlFailMskParams {
    pub accel_fail_mask: u32,
    pub gyro_fail_mask: u32,
}

impl Default for SitlFailMskParams {
    fn default() -> Self {
        Self {
            accel_fail_mask: 0,
            gyro_fail_mask: 0,
        }
    }
}

impl SitlFailMskParams {
    /// Build from SIM_ACCEL_FAIL_MSK / SIM_GYRO_FAIL_MSK values.
    #[must_use]
    pub const fn from_masks(accel_fail_mask: u32, gyro_fail_mask: u32) -> Self {
        Self {
            accel_fail_mask,
            gyro_fail_mask,
        }
    }

    /// Apply to one SITL IMU backend's fail-mask fields.
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend) {
        backend.accel_fail_mask = self.accel_fail_mask;
        backend.gyro_fail_mask = self.gyro_fail_mask;
    }

    /// Apply shared masks to every registered backend, upstream one mask pair
    /// for all IMU instances.
    pub fn apply_to_cluster(&self, cluster: &mut SitlInsCluster) {
        cluster.set_fail_masks(self.accel_fail_mask, self.gyro_fail_mask);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::{SitlBodyState, SitlImuBackend};

    #[test]
    fn default_masks_are_zero() {
        let params = SitlFailMskParams::default();
        assert_eq!(params.accel_fail_mask, 0);
        assert_eq!(params.gyro_fail_mask, 0);
    }

    #[test]
    fn apply_to_cluster_sets_every_backend() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();

        let params = SitlFailMskParams::from_masks(1 << 1, 1 << 2);
        params.apply_to_cluster(&mut cluster);

        assert_eq!(cluster.backend(0).unwrap().accel_fail_mask, 1 << 1);
        assert_eq!(cluster.backend(0).unwrap().gyro_fail_mask, 1 << 2);
        assert_eq!(cluster.backend(1).unwrap().accel_fail_mask, 1 << 1);
        assert_eq!(cluster.backend(1).unwrap().gyro_fail_mask, 1 << 2);
    }

    #[test]
    fn bound_masks_suppress_masked_instance_samples() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();

        SitlFailMskParams::from_masks(1, 1).apply_to_cluster(&mut cluster);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let (g, a) = cluster.timer_update(0, &state, &[]);
        assert_eq!(g, 1);
        assert_eq!(a, 1);
    }
}
