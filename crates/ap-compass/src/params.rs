//! Compass parameter table stub, upstream AP_Compass var_info. FW-014.

use ap_math::vector3::Vector3f;

use crate::offset::{COMPASS_LEARN_DEFAULT, COMPASS_OFFSETS_MAX_DEFAULT};
use crate::sitl::{
    SitlCompassBackend, SitlCompassCluster, SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES,
};

/// Upstream COMPASS_AUTODEC default.
pub const COMPASS_AUTODEC_DEFAULT: bool = true;

/// Upstream COMPASS_USE default.
pub const COMPASS_USE_DEFAULT: bool = true;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassInstanceParams {
    pub disabled: bool,
    pub use_for_yaw: bool,
    /// Hard-iron offset, upstream `COMPASS_OFS` / `OFS2` (field units).
    pub offset: Vector3f,
}

impl Default for CompassInstanceParams {
    fn default() -> Self {
        Self {
            disabled: false,
            use_for_yaw: COMPASS_USE_DEFAULT,
            offset: Vector3f::zero(),
        }
    }
}

impl CompassInstanceParams {
    pub fn apply_to_config(self) -> SitlCompassConfig {
        SitlCompassConfig {
            disabled: self.disabled,
            offset: self.offset,
            ..SitlCompassConfig::default()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassParams {
    pub compass1: CompassInstanceParams,
    pub compass2: CompassInstanceParams,
    pub primary: u8,
    /// Manual declination override, upstream COMPASS_DEC (radians).
    pub declination_rad: f32,
    /// Auto declination from GPS location, upstream COMPASS_AUTODEC.
    pub auto_declination: bool,
    /// Offset learn mode, upstream `COMPASS_LEARN`.
    pub learn: u8,
    /// Max allowed offset length, upstream `COMPASS_OFFS_MAX`.
    pub offsets_max: f32,
}

impl Default for CompassParams {
    fn default() -> Self {
        Self {
            compass1: CompassInstanceParams::default(),
            compass2: CompassInstanceParams::default(),
            primary: 0,
            declination_rad: 0.0,
            auto_declination: COMPASS_AUTODEC_DEFAULT,
            learn: COMPASS_LEARN_DEFAULT,
            offsets_max: COMPASS_OFFSETS_MAX_DEFAULT,
        }
    }
}

impl CompassParams {
    pub fn apply_instance(&self, instance: u8, backend: &mut SitlCompassBackend) {
        let inst = if instance == 0 { self.compass1 } else { self.compass2 };
        let mut cfg = *backend.config();
        cfg.disabled = inst.disabled;
        cfg.offset = inst.offset;
        backend.set_config(cfg);
    }

    pub fn apply_to_cluster(&self, cluster: &mut SitlCompassCluster) {
        cluster.set_primary(self.primary.min((SITL_COMPASS_MAX_INSTANCES - 1) as u8));
        for i in 0..cluster.instance_count() {
            if let Some(backend) = cluster.backend_mut(i) {
                self.apply_instance(i, backend);
            }
        }
    }

    /// Primary instance use-for-yaw flag, upstream COMPASS_USE / COMPASS_USE2.
    #[must_use]
    pub fn primary_use_for_yaw(&self) -> bool {
        if self.primary == 0 {
            self.compass1.use_for_yaw
        } else {
            self.compass2.use_for_yaw
        }
    }
}
