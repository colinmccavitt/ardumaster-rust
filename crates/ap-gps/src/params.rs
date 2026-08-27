//! GPS parameter table stub, upstream `AP_GPS` var_info / param table. FW-012.

use crate::blend::GpsAutoSwitch;
use crate::moving_baseline::GpsMovingBaseline;
use crate::dual::GpsDualStub;
use crate::health::GPS_MIN_NSATS;
use crate::sitl::{SitlGpsBackend, SITL_GPS_DEFAULT_LAG_SEC, SITL_GPS_UPDATE_MS};

pub const GPS_TYPE_NONE: u8 = 0;
pub const GPS_TYPE_SITL: u8 = 1;
pub const GPS_BLEND_MASK_PARAM_DEFAULT: u8 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsInstanceParams {
    pub gps_type: u8,
    pub delay_ms: u16,
    pub rate_ms: u16,
}

impl Default for GpsInstanceParams {
    fn default() -> Self {
        Self {
            gps_type: GPS_TYPE_SITL,
            delay_ms: 0,
            rate_ms: SITL_GPS_UPDATE_MS as u16,
        }
    }
}

impl GpsInstanceParams {
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        self.gps_type != GPS_TYPE_NONE
    }

    #[must_use]
    pub fn lag_sec(self) -> f32 {
        if self.delay_ms > 0 {
            self.delay_ms as f32 / 1000.0
        } else {
            SITL_GPS_DEFAULT_LAG_SEC
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsParams {
    pub gps1: GpsInstanceParams,
    pub gps2: GpsInstanceParams,
    pub auto_switch: GpsAutoSwitch,
    pub blend_mask: u8,
    pub primary: u8,
    pub min_nsats: u8,
}

impl Default for GpsParams {
    fn default() -> Self {
        Self {
            gps1: GpsInstanceParams::default(),
            gps2: GpsInstanceParams {
                gps_type: GPS_TYPE_NONE,
                ..GpsInstanceParams::default()
            },
            auto_switch: GpsAutoSwitch::UseBest,
            blend_mask: GPS_BLEND_MASK_PARAM_DEFAULT,
            primary: 0,
            min_nsats: GPS_MIN_NSATS,
        }
    }
}

impl GpsParams {
    #[must_use]
    pub fn dual_enabled(self) -> bool {
        self.gps1.is_enabled() && self.gps2.is_enabled()
    }

    pub fn apply_instance(&self, instance: u8, backend: &mut SitlGpsBackend) {
        let inst = if instance == 0 { self.gps1 } else { self.gps2 };
        backend.disabled = !inst.is_enabled();
        backend.set_lag_sec(inst.lag_sec());
    }

    #[must_use]
    pub fn configure_dual_stub(self) -> Option<GpsDualStub> {
        if !self.dual_enabled() {
            return None;
        }
        let mut stub = GpsDualStub::default();
        stub.apply_params(self);
        Some(stub)
    }

    pub fn apply_to_dual(self, dual: &mut GpsDualStub) {
        dual.apply_params(self);
    }

    #[must_use]
    pub const fn moving_baseline(self) -> GpsMovingBaseline {
        GpsMovingBaseline::from_types(self.gps1.gps_type, self.gps2.gps_type)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_plane_fixture_subset() {
        let p = GpsParams::default();
        assert_eq!(p.gps1.gps_type, GPS_TYPE_SITL);
        assert_eq!(p.gps2.gps_type, GPS_TYPE_NONE);
        assert!(!p.dual_enabled());
    }

    #[test]
    fn dual_enabled_when_both_receivers_configured() {
        let mut p = GpsParams::default();
        p.gps2.gps_type = GPS_TYPE_SITL;
        assert!(p.dual_enabled());
    }

    #[test]
    fn apply_instance_sets_backend_lag() {
        let mut backend = SitlGpsBackend::default();
        let mut p = GpsParams::default();
        p.gps1.delay_ms = 400;
        p.apply_instance(0, &mut backend);
        assert!((backend.lag_sec() - 0.4).abs() < 1e-6);
    }
}
