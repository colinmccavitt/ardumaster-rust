//! SIM_IMUT_* temperature curve and SIM_IMUT{n}_* calibration binding, upstream
//! `SITL::imu_temp_*` and `SITL::imu_tcal[]` applied in `AP_InertialSensor_SITL`.
//! FW-011.

use ap_math::vector3::Vector3f;

use crate::sitl::{
    SitlImuBackend, SitlImuTemperature, SitlInsCluster, SitlInsTempCal, SitlInsTempCalCoeffs,
};

/// Shared IMU warm-up temperature curve from SIM_IMUT_START/END/TCONST/FIXED.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlImuTempParams {
    pub temp_start_c: f32,
    pub temp_end_c: f32,
    pub temp_tconst_s: f32,
    pub temp_fixed_c: f32,
}

impl Default for SitlImuTempParams {
    fn default() -> Self {
        Self {
            temp_start_c: 25.0,
            temp_end_c: 45.0,
            temp_tconst_s: 300.0,
            temp_fixed_c: 0.0,
        }
    }
}

impl SitlImuTempParams {
    /// Build from SIM_IMUT_START/END/TCONST/FIXED values.
    #[must_use]
    pub const fn from_sim_imut(
        temp_start_c: f32,
        temp_end_c: f32,
        temp_tconst_s: f32,
        temp_fixed_c: f32,
    ) -> Self {
        Self {
            temp_start_c,
            temp_end_c,
            temp_tconst_s,
            temp_fixed_c,
        }
    }

    /// Convert to backend warm-up model, upstream `get_temperature` inputs.
    #[must_use]
    pub fn to_temperature_config(&self) -> SitlImuTemperature {
        SitlImuTemperature {
            temp_fixed_c: self.temp_fixed_c,
            temp_start_c: self.temp_start_c,
            temp_end_c: self.temp_end_c,
            temp_tconst_s: self.temp_tconst_s,
        }
    }

    /// Apply shared curve to one backend's [`SitlImuBackend::temperature`].
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend) {
        backend.temperature = self.to_temperature_config();
    }

    /// Apply shared curve to every registered backend.
    pub fn apply_to_cluster(&self, cluster: &mut SitlInsCluster) {
        cluster.set_imu_temperature(self.to_temperature_config());
    }
}

/// Per-instance temperature calibration from SIM_IMUT{n}_* (AP_InertialSensor_TCal).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlInsTempCalParams {
    pub enable: bool,
    pub temp_min_c: f32,
    pub temp_max_c: f32,
    pub accel_c0: Vector3f,
    pub accel_c1: Vector3f,
    pub accel_c2: Vector3f,
    pub gyro_c0: Vector3f,
    pub gyro_c1: Vector3f,
    pub gyro_c2: Vector3f,
}

impl Default for SitlInsTempCalParams {
    fn default() -> Self {
        Self {
            enable: false,
            temp_min_c: 0.0,
            temp_max_c: 70.0,
            accel_c0: Vector3f::zero(),
            accel_c1: Vector3f::zero(),
            accel_c2: Vector3f::zero(),
            gyro_c0: Vector3f::zero(),
            gyro_c1: Vector3f::zero(),
            gyro_c2: Vector3f::zero(),
        }
    }
}

impl SitlInsTempCalParams {
    /// Build from SIM_IMUT{n}_ENABLE/TMIN/TMAX and ACC1/2/3, GYR1/2/3 vectors.
    #[must_use]
    pub const fn from_sim_imut_instance(
        enable: bool,
        temp_min_c: f32,
        temp_max_c: f32,
        accel_c0: Vector3f,
        accel_c1: Vector3f,
        accel_c2: Vector3f,
        gyro_c0: Vector3f,
        gyro_c1: Vector3f,
        gyro_c2: Vector3f,
    ) -> Self {
        Self {
            enable,
            temp_min_c,
            temp_max_c,
            accel_c0,
            accel_c1,
            accel_c2,
            gyro_c0,
            gyro_c1,
            gyro_c2,
        }
    }

    /// Convert to backend temp-cal model; disabled instances leave cal off.
    #[must_use]
    pub fn to_temp_cal(&self) -> Option<SitlInsTempCal> {
        if !self.enable {
            return None;
        }
        Some(SitlInsTempCal {
            temp_min_c: self.temp_min_c,
            temp_max_c: self.temp_max_c,
            accel: SitlInsTempCalCoeffs {
                c0: self.accel_c0,
                c1: self.accel_c1,
                c2: self.accel_c2,
            },
            gyro: SitlInsTempCalCoeffs {
                c0: self.gyro_c0,
                c1: self.gyro_c1,
                c2: self.gyro_c2,
            },
        })
    }

    /// Apply to one backend's [`SitlImuBackend::temp_cal`].
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend) {
        backend.temp_cal = self.to_temp_cal();
    }

    /// Apply per-instance cal params indexed by IMU instance.
    pub fn apply_instances_to_cluster(cluster: &mut SitlInsCluster, params: &[Self]) {
        for (i, p) in params.iter().enumerate() {
            if let Some(backend) = cluster.backend_mut(i as u8) {
                p.apply_to_backend(backend);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::{SitlBodyState, SitlImuBackend, SitlImuTemperature};

    #[test]
    fn default_imu_temp_matches_upstream() {
        let params = SitlImuTempParams::default();
        let cfg = params.to_temperature_config();
        assert_eq!(cfg.temp_start_c, 25.0);
        assert_eq!(cfg.temp_end_c, 45.0);
        assert_eq!(cfg.temp_tconst_s, 300.0);
        assert_eq!(cfg.temp_fixed_c, 0.0);
    }

    #[test]
    fn apply_temp_curve_to_cluster() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();

        let params = SitlImuTempParams::from_sim_imut(10.0, 50.0, 60.0, 0.0);
        params.apply_to_cluster(&mut cluster);

        let expected = params.to_temperature_config();
        let b0 = cluster.backend(0).unwrap();
        assert_eq!(b0.temperature.temp_start_c, expected.temp_start_c);
        assert_eq!(b0.temperature.temp_end_c, expected.temp_end_c);
        assert_eq!(b0.temperature.temp_tconst_s, expected.temp_tconst_s);
        assert_eq!(b0.temperature.temp_fixed_c, expected.temp_fixed_c);
        let b1 = cluster.backend(1).unwrap();
        assert_eq!(b1.temperature.temp_start_c, expected.temp_start_c);
    }

    #[test]
    fn disabled_instance_cal_clears_backend() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.temp_cal = Some(SitlInsTempCal::default());
        SitlInsTempCalParams::default().apply_to_backend(&mut backend);
        assert!(backend.temp_cal.is_none());
    }

    #[test]
    fn bound_instance_cal_applies_on_kinematic_path() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();

        SitlImuTempParams::from_sim_imut(25.0, 45.0, 300.0, 45.0).apply_to_cluster(&mut cluster);

        let cal = SitlInsTempCalParams::from_sim_imut_instance(
            true,
            0.0,
            70.0,
            Vector3f::new(1_000_000.0, 0.0, 0.0),
            Vector3f::zero(),
            Vector3f::zero(),
            Vector3f::zero(),
            Vector3f::zero(),
            Vector3f::zero(),
        );
        SitlInsTempCalParams::apply_instances_to_cluster(&mut cluster, &[cal]);

        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        cluster.timer_update(0, &state, &[]);
        {
            let backend = cluster.backend_mut(0).unwrap();
            backend.imu.update_accel();
            assert!(
                backend.imu.accel().x > 9.0,
                "bound temp cal should add +10 on x at 45C fixed, got {}",
                backend.imu.accel().x
            );
        }
    }
}
