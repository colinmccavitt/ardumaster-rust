//! Baro parameter table stub, upstream AP_Baro var_info. FW-013.

use crate::frontend::{BaroFrontend, BaroFrontendParams};
use crate::sitl::{SitlBaroBackend, SitlBaroCluster, SitlBaroConfig, SITL_BARO_MAX_INSTANCES};

pub const SIM_BARO_RND_DEFAULT: f32 = 0.2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaroInstanceParams {
    pub disabled: bool,
    pub noise_scale: f32,
    pub delay_ms: u32,
    pub drift_rate_mps: f32,
}

impl Default for BaroInstanceParams {
    fn default() -> Self {
        Self { disabled: false, noise_scale: 0.0, delay_ms: 0, drift_rate_mps: 0.0 }
    }
}

impl BaroInstanceParams {
    pub fn apply_to_config(self) -> SitlBaroConfig {
        SitlBaroConfig {
            disabled: self.disabled,
            noise_scale: self.noise_scale,
            delay_ms: self.delay_ms,
            drift_rate_mps: self.drift_rate_mps,
            ..SitlBaroConfig::default()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaroParams {
    pub baro1: BaroInstanceParams,
    pub baro2: BaroInstanceParams,
    pub primary: u8,
    pub frontend: BaroFrontendParams,
}

impl Default for BaroParams {
    fn default() -> Self {
        Self {
            baro1: BaroInstanceParams::default(),
            baro2: BaroInstanceParams::default(),
            primary: 0,
            frontend: BaroFrontendParams::default(),
        }
    }
}

impl BaroParams {
    pub fn apply_instance(&self, instance: u8, backend: &mut SitlBaroBackend) {
        let inst = if instance == 0 { self.baro1 } else { self.baro2 };
        backend.set_config(inst.apply_to_config());
    }

    pub fn apply_to_cluster(&self, cluster: &mut SitlBaroCluster) {
        cluster.set_primary(self.primary.min((SITL_BARO_MAX_INSTANCES - 1) as u8));
        for i in 0..cluster.instance_count() {
            if let Some(backend) = cluster.backend_mut(i) {
                self.apply_instance(i, backend);
            }
        }
    }

    pub fn apply_to_frontend(&self, frontend: &mut BaroFrontend) {
        self.frontend.apply_to_frontend(frontend);
    }
}
