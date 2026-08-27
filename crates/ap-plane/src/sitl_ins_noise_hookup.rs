//! SITL INS noise/vibration cluster hookup for the scheduler tick.
//!
//! The main loop calls [`sitl_ins_noise_scheduler_tick`] before
//! [`PlaneMainLoop::ahrs_update`] so SIM_VIB noise and motor runtime reach
//! [`SitlInsCluster::timer_update`] and the shared INS frontend.

use ap_ins::sitl::{SitlBodyState, SitlImuBackend, SitlInsCluster, SitlInsFileMode, SitlInsInstanceFiles};
use ap_ins::{SitlInsFilePlaybackParams, SitlInsMotorRuntime, SitlInsNoiseParams};

/// SITL INS cluster plus bound SIM_VIB noise params.
#[derive(Debug, Clone)]
pub struct SitlInsNoiseHookup {
    /// Multi-instance SITL backends feeding the vehicle INS frontend.
    pub cluster: SitlInsCluster,
    /// Static SIM_VIB_FREQ_* / SIM_VIB_MOT_* parameters.
    pub noise_params: SitlInsNoiseParams,
    pub file_playback_params: SitlInsFilePlaybackParams,
}

impl Default for SitlInsNoiseHookup {
    fn default() -> Self {
        Self::with_default_backend()
    }
}

impl SitlInsNoiseHookup {
    /// One IMU backend at 1 kHz gyro/accel, upstream default SITL registration.
    #[must_use]
    pub fn with_default_backend() -> Self {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        Self {
            cluster,
            noise_params: SitlInsNoiseParams::default(),
            file_playback_params: SitlInsFilePlaybackParams::default(),
        }
    }
}

/// Kinematic body state and per-tick motor inputs for one scheduler pass.
#[derive(Debug, Clone, Copy)]
pub struct SitlInsNoiseSchedulerInputs<'a> {
    pub body: SitlBodyState,
    pub motor: SitlInsMotorRuntime,
    pub files: &'a [SitlInsInstanceFiles<'a>],
    pub now_us: u64,
}

/// Per-tick INS noise cluster accounting.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SitlInsNoiseSchedulerOutput {
    pub gyro_samples: u32,
    pub accel_samples: u32,
    pub noise_applied: bool,
    pub file_playback_active: bool,
}

/// Apply noise params, advance the SITL cluster, and refresh the INS frontend.
#[must_use]
pub fn sitl_ins_noise_scheduler_tick(
    hookup: &mut SitlInsNoiseHookup,
    inp: &SitlInsNoiseSchedulerInputs<'_>,
) -> SitlInsNoiseSchedulerOutput {
    hookup
        .noise_params
        .apply_to_cluster(&mut hookup.cluster, &inp.motor);
    hookup.file_playback_params.apply_to_cluster(&mut hookup.cluster);
    let noise_applied = hookup.noise_params.should_apply(&inp.motor);
    let file_playback_active = hookup.file_playback_params.accel_mode() != SitlInsFileMode::None
        || hookup.file_playback_params.gyro_mode() != SitlInsFileMode::None;
    let (gyro_samples, accel_samples) =
        hookup.cluster.timer_update(inp.now_us, &inp.body, inp.files);
    SitlInsNoiseSchedulerOutput {
        gyro_samples,
        accel_samples,
        noise_applied,
        file_playback_active,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_ins::sitl::SitlInsNoiseConfig;

    #[test]
    fn scheduler_tick_applies_motor_noise_to_cluster() {
        let mut hookup = SitlInsNoiseHookup::with_default_backend();
        hookup.noise_params = SitlInsNoiseParams::from_sim_vib(12.0, 0.0, 0.0, 0.0, 1.0, 1, 0);
        let inp = SitlInsNoiseSchedulerInputs {
            body: SitlBodyState {
                z_accel: -9.80665,
                ..SitlBodyState::default()
            },
            motor: SitlInsMotorRuntime {
                motors_on: true,
                throttle: 0.75,
                ..SitlInsMotorRuntime::default()
            },
            files: &[],
            now_us: 0,
        };
        let out = sitl_ins_noise_scheduler_tick(&mut hookup, &inp);
        assert!(out.noise_applied);
        assert_eq!(out.gyro_samples, 1);
        assert_eq!(out.accel_samples, 1);
        let cfg = hookup
            .cluster
            .backend(0)
            .unwrap()
            .noise_config
            .as_ref()
            .unwrap();
        assert_eq!(cfg.vibe.vibe_freq_hz.x, 12.0);
        assert_eq!(cfg.throttle, 0.75);
    }


    #[test]
    fn scheduler_tick_applies_file_playback_from_host_buffers() {
        use ap_math::vector3::Vector3f;

        fn encode(v: Vector3f) -> [u8; 12] {
            let mut out = [0_u8; 12];
            for (i, c) in [v.x, v.y, v.z].into_iter().enumerate() {
                out[i * 4..i * 4 + 4].copy_from_slice(&c.to_le_bytes());
            }
            out
        }

        let mut hookup = SitlInsNoiseHookup::with_default_backend();
        hookup.file_playback_params = SitlInsFilePlaybackParams::from_sim_file_rw(1, 0);
        let file = encode(Vector3f::new(0.0, 0.0, -4.0));
        let files = [SitlInsInstanceFiles { accel: Some(&file), gyro: None }];
        let inp = SitlInsNoiseSchedulerInputs {
            body: SitlBodyState { z_accel: -9.80665, ..SitlBodyState::default() },
            motor: SitlInsMotorRuntime::default(),
            files: &files,
            now_us: 0,
        };
        let out = sitl_ins_noise_scheduler_tick(&mut hookup, &inp);
        assert!(out.file_playback_active);
        hookup.cluster.backend_mut(0).unwrap().imu.update_accel();
        assert!((hookup.cluster.backend(0).unwrap().imu.accel().z + 4.0).abs() < 1e-4);
    }
    #[test]
    fn motors_off_without_vibe_clears_noise_config() {
        let mut hookup = SitlInsNoiseHookup::with_default_backend();
        hookup.cluster.backend_mut(0).unwrap().noise_config = Some(SitlInsNoiseConfig::default());
        let inp = SitlInsNoiseSchedulerInputs {
            body: SitlBodyState {
                z_accel: -9.80665,
                ..SitlBodyState::default()
            },
            motor: SitlInsMotorRuntime::default(),
            files: &[],
            now_us: 0,
        };
        let out = sitl_ins_noise_scheduler_tick(&mut hookup, &inp);
        assert!(!out.noise_applied);
        assert!(hookup.cluster.backend(0).unwrap().noise_config.is_none());
    }
}
