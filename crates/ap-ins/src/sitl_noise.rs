//! SIM_VIB_* vibration/noise parameter binding, upstream `SITL::vibe_*` applied
//! in `AP_InertialSensor_SITL::generate_accel` / `generate_gyro`. FW-011.

use ap_math::vector3::Vector3f;

use crate::sitl::{
    SitlImuBackend, SitlInsCluster, SitlInsNoiseConfig, SitlMotorVibeConfig, SitlVibeConfig,
    SITL_DEFAULT_ACCEL_NOISE,
};

/// Per-tick motor/throttle state from the simulator frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SitlInsMotorRuntime {
    pub motors_on: bool,
    pub throttle: f32,
    pub motor_mask: u32,
    pub motor_rpm: [f32; 8],
}

/// Static vibration/noise params from SIM_VIB_FREQ_* and SIM_VIB_MOT_*.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlInsNoiseParams {
    pub vibe_freq_x_hz: f32,
    pub vibe_freq_y_hz: f32,
    pub vibe_freq_z_hz: f32,
    pub vibe_mot_max: f32,
    pub vibe_mot_mult: f32,
    pub vibe_mot_harmonics: u32,
    pub vibe_mot_mask: u32,
    /// Motor-on accel noise floor, upstream default 0.5 m/s².
    pub motor_accel_noise: f32,
    /// Motor-on gyro noise scale, upstream default 20 deg/s at full throttle.
    pub motor_gyro_noise_deg: f32,
}

impl Default for SitlInsNoiseParams {
    fn default() -> Self {
        Self {
            vibe_freq_x_hz: 0.0,
            vibe_freq_y_hz: 0.0,
            vibe_freq_z_hz: 0.0,
            vibe_mot_max: 0.0,
            vibe_mot_mult: 1.0,
            vibe_mot_harmonics: 1,
            vibe_mot_mask: 0,
            motor_accel_noise: 0.5,
            motor_gyro_noise_deg: 20.0,
        }
    }
}

impl SitlInsNoiseParams {
    /// Build from SIM_VIB_FREQ_X/Y/Z and SIM_VIB_MOT_MAX/MULT/HMNC/MASK.
    #[must_use]
    pub const fn from_sim_vib(
        vibe_freq_x_hz: f32,
        vibe_freq_y_hz: f32,
        vibe_freq_z_hz: f32,
        vibe_mot_max: f32,
        vibe_mot_mult: f32,
        vibe_mot_harmonics: u32,
        vibe_mot_mask: u32,
    ) -> Self {
        Self {
            vibe_freq_x_hz,
            vibe_freq_y_hz,
            vibe_freq_z_hz,
            vibe_mot_max,
            vibe_mot_mult,
            vibe_mot_harmonics,
            vibe_mot_mask,
            motor_accel_noise: 0.5,
            motor_gyro_noise_deg: 20.0,
        }
    }

    fn static_vibe_active(&self) -> bool {
        self.vibe_freq_x_hz != 0.0
            || self.vibe_freq_y_hz != 0.0
            || self.vibe_freq_z_hz != 0.0
            || self.vibe_mot_max != 0.0
    }

    /// True when upstream would inject white noise or vibration this tick.
    #[must_use]
    pub fn should_apply(&self, runtime: &SitlInsMotorRuntime) -> bool {
        self.static_vibe_active() || runtime.motors_on
    }

    /// Convert bound params plus runtime motor state to backend noise config.
    #[must_use]
    pub fn to_noise_config(&self, runtime: &SitlInsMotorRuntime) -> SitlInsNoiseConfig {
        let motors_on = runtime.motors_on;
        SitlInsNoiseConfig {
            motors_on,
            throttle: runtime.throttle,
            motor_accel_noise: self.motor_accel_noise,
            motor_gyro_noise_deg: self.motor_gyro_noise_deg,
            motor_mask: if runtime.motor_mask != 0 {
                runtime.motor_mask
            } else {
                self.vibe_mot_mask
            },
            motor_rpm: runtime.motor_rpm,
            vibe: SitlVibeConfig {
                vibe_freq_hz: Vector3f::new(
                    self.vibe_freq_x_hz,
                    self.vibe_freq_y_hz,
                    self.vibe_freq_z_hz,
                ),
                accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                noise_variation: 0.05,
                motors_on,
            },
            motor_vibe: SitlMotorVibeConfig {
                vibe_motor: self.vibe_mot_max,
                vibe_motor_scale: self.vibe_mot_mult,
                vibe_motor_harmonics: self.vibe_mot_harmonics,
                accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                noise_variation: 0.05,
                freq_variation: 0.12,
                motors_on,
            },
        }
    }

    /// Apply to one backend's [`SitlImuBackend::noise_config`].
    pub fn apply_to_backend(&self, backend: &mut SitlImuBackend, runtime: &SitlInsMotorRuntime) {
        backend.noise_config = if self.should_apply(runtime) {
            Some(self.to_noise_config(runtime))
        } else {
            None
        };
    }

    /// Apply shared noise config to every registered backend.
    pub fn apply_to_cluster(&self, cluster: &mut SitlInsCluster, runtime: &SitlInsMotorRuntime) {
        cluster.set_noise_config(if self.should_apply(runtime) {
            Some(self.to_noise_config(runtime))
        } else {
            None
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::{SitlBodyState, SitlImuBackend, sitl_accel_sample};

    #[test]
    fn default_params_do_not_apply_without_motors() {
        let params = SitlInsNoiseParams::default();
        let runtime = SitlInsMotorRuntime::default();
        assert!(!params.should_apply(&runtime));
    }

    #[test]
    fn motors_on_enables_noise_even_without_vibe_params() {
        let params = SitlInsNoiseParams::default();
        let runtime = SitlInsMotorRuntime {
            motors_on: true,
            throttle: 0.5,
            ..SitlInsMotorRuntime::default()
        };
        assert!(params.should_apply(&runtime));
    }

    #[test]
    fn apply_to_cluster_sets_every_backend() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();

        let params = SitlInsNoiseParams::from_sim_vib(10.0, 0.0, 0.0, 0.0, 1.0, 1, 0);
        let runtime = SitlInsMotorRuntime {
            motors_on: true,
            throttle: 0.8,
            ..SitlInsMotorRuntime::default()
        };
        params.apply_to_cluster(&mut cluster, &runtime);

        let cfg = cluster.backend(0).unwrap().noise_config.as_ref().unwrap();
        assert_eq!(cfg.vibe.vibe_freq_hz.x, 10.0);
        assert!(cfg.motors_on);
        assert_eq!(cluster.backend(1).unwrap().noise_config.as_ref().unwrap().throttle, 0.8);
    }

    #[test]
    fn bound_noise_changes_accel_sample_on_timer_update() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        let params = SitlInsNoiseParams::default();
        let runtime = SitlInsMotorRuntime {
            motors_on: true,
            throttle: 1.0,
            ..SitlInsMotorRuntime::default()
        };
        params.apply_to_backend(&mut backend, &runtime);

        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let clean = sitl_accel_sample(&state, &backend.cal, backend.board_trim);
        backend.timer_update(0, &state, Default::default());
        backend.imu.update_accel();
        assert!(
            (backend.imu.accel().z - clean.z).abs() > 1e-4,
            "motor-on noise should perturb z from clean kinematic sample"
        );
    }

    #[test]
    fn zero_vibe_and_motors_off_clears_noise_config() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.backend_mut(0).unwrap().noise_config = Some(SitlInsNoiseConfig::default());

        SitlInsNoiseParams::default().apply_to_cluster(&mut cluster, &SitlInsMotorRuntime::default());
        assert!(cluster.backend(0).unwrap().noise_config.is_none());
    }
}
