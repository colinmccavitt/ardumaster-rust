//! Compass parameter table stub, upstream AP_Compass var_info. FW-014.

use ap_math::vector3::Vector3f;

use crate::motor_comp::COMPASS_MOTCT_DEFAULT;
use crate::offset::{COMPASS_LEARN_DEFAULT, COMPASS_OFFSETS_MAX_DEFAULT};
use crate::orientation::{COMPASS_EXTERNAL_DEFAULT, COMPASS_ORIENT_DEFAULT};
use crate::scale::COMPASS_SCALE_DEFAULT;
use crate::sitl::{
    SitlCompassBackend, SitlCompassCluster, SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES,
};
use crate::soft_iron::{COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT};

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
    /// Motor compensation factors, upstream `COMPASS_MOT` / `MOT2`.
    pub motor_compensation: Vector3f,
    /// Instance orientation, upstream `COMPASS_ORIENT` / `ORIENT2`.
    pub orientation: u8,
    /// External mount, upstream `COMPASS_EXTERNAL` / `EXTERN2`.
    pub external: bool,
    /// Scale factor, upstream `COMPASS_SCALE` / `SCALE2`.
    pub scale: f32,
    /// Soft-iron diagonal, upstream `COMPASS_DIA` / `DIA2`.
    pub diagonals: Vector3f,
    /// Soft-iron off-diagonal, upstream `COMPASS_ODI` / `ODI2`.
    pub offdiagonals: Vector3f,
}

impl Default for CompassInstanceParams {
    fn default() -> Self {
        Self {
            disabled: false,
            use_for_yaw: COMPASS_USE_DEFAULT,
            offset: Vector3f::zero(),
            motor_compensation: Vector3f::zero(),
            orientation: COMPASS_ORIENT_DEFAULT,
            external: COMPASS_EXTERNAL_DEFAULT,
            scale: COMPASS_SCALE_DEFAULT,
            diagonals: COMPASS_DIA_DEFAULT,
            offdiagonals: COMPASS_ODI_DEFAULT,
        }
    }
}

impl CompassInstanceParams {
    pub fn apply_to_config(self) -> SitlCompassConfig {
        SitlCompassConfig {
            disabled: self.disabled,
            offset: self.offset,
            motor_compensation: self.motor_compensation,
            orientation: self.orientation,
            external: self.external,
            scale: self.scale,
            diagonals: self.diagonals,
            offdiagonals: self.offdiagonals,
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
    /// Motor compensation type, upstream `COMPASS_MOTCT`.
    pub motor_comp_type: u8,
    /// AHRS board orientation applied to internal compasses.
    pub board_orientation: u8,
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
            motor_comp_type: COMPASS_MOTCT_DEFAULT,
            board_orientation: COMPASS_ORIENT_DEFAULT,
        }
    }
}

impl CompassParams {
    pub fn apply_instance(&self, instance: u8, backend: &mut SitlCompassBackend) {
        let inst = if instance == 0 {
            self.compass1
        } else {
            self.compass2
        };
        let mut cfg = *backend.config();
        cfg.disabled = inst.disabled;
        cfg.offset = inst.offset;
        cfg.motor_compensation = inst.motor_compensation;
        cfg.motor_comp_type = self.motor_comp_type;
        cfg.orientation = inst.orientation;
        cfg.external = inst.external;
        cfg.board_orientation = self.board_orientation;
        cfg.scale = inst.scale;
        cfg.diagonals = inst.diagonals;
        cfg.offdiagonals = inst.offdiagonals;
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
