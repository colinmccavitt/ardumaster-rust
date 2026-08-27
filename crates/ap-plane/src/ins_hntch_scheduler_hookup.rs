//! INS harmonic notch scheduler hookup for the plane main loop.
//!
//! Upstream `AP_InertialSensor::update_gyro_filters` and
//! `AP_Vehicle::update_dynamic_notch` run in the vehicle loop before gyro
//! samples reach the AHRS.

use ap_ins::sitl::SitlInsCluster;
use ap_ins::{DEFAULT_GYRO_FILTER_HZ, InertialSensorFrontend, InsHntchParams, TrackingMode};

/// Bound INS_HNTCH_* parameters and gyro low-pass cutoff for one vehicle.
#[derive(Debug, Clone)]
pub struct InsHntchHookup {
    pub params: InsHntchParams,
    pub gyro_filter_hz: f32,
    filters_dirty: bool,
}

impl Default for InsHntchHookup {
    fn default() -> Self {
        Self {
            params: InsHntchParams::default(),
            gyro_filter_hz: DEFAULT_GYRO_FILTER_HZ,
            filters_dirty: true,
        }
    }
}

impl InsHntchHookup {
    /// Mark gyro filters for reconfiguration on the next scheduler tick.
    pub fn mark_filters_dirty(&mut self) {
        self.filters_dirty = true;
    }

    /// Replace INS_HNTCH_* binding and schedule filter reconfiguration.
    pub fn set_params(&mut self, params: InsHntchParams) {
        self.params = params;
        self.filters_dirty = true;
    }
}

/// Per-tick motor inputs for dynamic notch centre tracking.
#[derive(Debug, Clone, Copy, Default)]
pub struct InsHntchSchedulerInputs {
    pub throttle: f32,
    pub motor_rpm: Option<f32>,
}

/// Per-tick harmonic notch accounting.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct InsHntchSchedulerOutput {
    pub filters_configured: bool,
    pub dynamic_notch_updated: bool,
}

/// Configure gyro filters and retune dynamic notch centres for this tick.
#[must_use]
pub fn ins_hntch_scheduler_tick(
    ins: &mut InertialSensorFrontend,
    hookup: &mut InsHntchHookup,
    inp: &InsHntchSchedulerInputs,
) -> InsHntchSchedulerOutput {
    let mut out = InsHntchSchedulerOutput::default();

    if hookup.filters_dirty {
        ins.update_gyro_filters(&hookup.params, hookup.gyro_filter_hz);
        hookup.filters_dirty = false;
        out.filters_configured = true;
    }

    if hookup.params.enable && hookup.params.tracking_mode() != TrackingMode::Fixed {
        ins.update_dynamic_notch(&hookup.params, inp.throttle, inp.motor_rpm);
        out.dynamic_notch_updated = true;
    }

    out
}

/// Like [`ins_hntch_scheduler_tick`], but mirrors filter configuration onto
/// each SITL backend IMU where gyro samples are filtered before handoff.
#[must_use]
pub fn ins_hntch_scheduler_tick_cluster(
    cluster: &mut SitlInsCluster,
    hookup: &mut InsHntchHookup,
    inp: &InsHntchSchedulerInputs,
) -> InsHntchSchedulerOutput {
    let out = ins_hntch_scheduler_tick(&mut cluster.frontend, hookup, inp);
    for i in 0..cluster.instance_count() {
        let instance = i;
        let rate = f32::from(cluster.frontend.get_gyro_rate_hz(instance));
        if let Some(backend) = cluster.backend_mut(instance) {
            if out.filters_configured {
                hookup
                    .params
                    .apply_gyro_filters_to_imu(&mut backend.imu, rate, hookup.gyro_filter_hz);
            }
            if out.dynamic_notch_updated {
                hookup
                    .params
                    .update_notch_center(&mut backend.imu, inp.throttle, inp.motor_rpm);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_ins::sitl::SitlImuBackend;

    #[test]
    fn cluster_tick_mirrors_notch_onto_backend_imu() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        let mut hookup = InsHntchHookup {
            params: InsHntchParams {
                enable: true,
                freq_hz: 80.0,
                bandwidth_hz: 40.0,
                attenuation_db: 40.0,
                harmonics: 1,
                mode: 0,
                ..InsHntchParams::default()
            },
            ..InsHntchHookup::default()
        };
        ins_hntch_scheduler_tick_cluster(
            &mut cluster,
            &mut hookup,
            &InsHntchSchedulerInputs::default(),
        );
        assert!(cluster.backend(0).unwrap().imu.gyro_notch_is_initialised());
    }

    #[test]
    fn scheduler_tick_configures_notch_on_primary() {
        let mut ins = InertialSensorFrontend::new();
        ins.register_sitl_backend(8000, 1000).unwrap();
        let mut hookup = InsHntchHookup {
            params: InsHntchParams {
                enable: true,
                freq_hz: 80.0,
                bandwidth_hz: 40.0,
                attenuation_db: 40.0,
                harmonics: 1,
                mode: 0,
                ..InsHntchParams::default()
            },
            ..InsHntchHookup::default()
        };
        let out = ins_hntch_scheduler_tick(
            &mut ins,
            &mut hookup,
            &InsHntchSchedulerInputs::default(),
        );
        assert!(out.filters_configured);
        assert!(ins.imu(0).unwrap().gyro_notch_is_initialised());
    }

    #[test]
    fn throttle_tracking_updates_dynamic_notch() {
        let mut ins = InertialSensorFrontend::new();
        ins.register_sitl_backend(1000, 1000).unwrap();
        let mut hookup = InsHntchHookup {
            params: InsHntchParams {
                enable: true,
                freq_hz: 100.0,
                bandwidth_hz: 40.0,
                attenuation_db: 40.0,
                harmonics: 1,
                reference: 1.0,
                mode: 1,
                freq_min_ratio: 0.5,
                ..InsHntchParams::default()
            },
            ..InsHntchHookup::default()
        };
        ins_hntch_scheduler_tick(
            &mut ins,
            &mut hookup,
            &InsHntchSchedulerInputs {
                throttle: 1.0,
                motor_rpm: None,
            },
        );
        hookup.filters_dirty = false;
        for _ in 0..32 {
            ins_hntch_scheduler_tick(
                &mut ins,
                &mut hookup,
                &InsHntchSchedulerInputs {
                    throttle: 0.25,
                    motor_rpm: None,
                },
            );
        }
        let center = ins.imu(0).unwrap().gyro_notch_center(0).expect("notch");
        assert!(
            (center - 50.0).abs() < 1.0,
            "quarter throttle -> 50 Hz, got {center}"
        );
    }
}
