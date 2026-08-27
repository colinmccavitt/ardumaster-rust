//! Dual-GPS SITL stub, upstream two-receiver `AP_GPS` frontend. FW-012.
//!
//! Holds two [`SitlGpsBackend`] instances and optionally blends their lag-buffered
//! outputs when [`GpsAutoSwitch::Blend`] is selected.

use crate::blend::{
    GpsAutoSwitch, GpsBlendInstance, GpsBlender, GPS_BLEND_MASK_DEFAULT,
    GPS_BLENDED_INSTANCE,
};
use crate::health::GpsHealthFlags;
use crate::sitl::{GpsFixState, SitlGpsBackend, SITL_GPS_UPDATE_MS};
use crate::status::GpsStatus;
use crate::velocity::{GpsVelocityProducer, GpsVelocitySample};
use ap_math::vector3::Vector3f;

/// Ground-truth inputs for one SITL GPS instance.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsInstanceTruth {
    pub velocity_ned: Vector3f,
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
    pub now_ms: u32,
}

/// Dual-GPS stub with optional blending, upstream `AP_GPS` multi-instance.
#[derive(Debug, Clone, Copy)]
pub struct GpsDualStub {
    pub primary: SitlGpsBackend,
    pub secondary: SitlGpsBackend,
    pub primary_truth: GpsInstanceTruth,
    pub secondary_truth: GpsInstanceTruth,
    pub auto_switch: GpsAutoSwitch,
    pub primary_instance: u8,
    blender: GpsBlender,
    pub dual_enabled: bool,
}

impl Default for GpsDualStub {
    fn default() -> Self {
        Self {
            primary: SitlGpsBackend::default(),
            secondary: SitlGpsBackend::default(),
            primary_truth: GpsInstanceTruth {
                latitude_deg: 51.875,
                longitude_deg: -0.154,
                ..GpsInstanceTruth::default()
            },
            secondary_truth: GpsInstanceTruth {
                latitude_deg: 51.8751,
                longitude_deg: -0.1541,
                ..GpsInstanceTruth::default()
            },
            auto_switch: GpsAutoSwitch::UsePrimary,
            primary_instance: 0,
            blender: GpsBlender::new(GPS_BLEND_MASK_DEFAULT),
            dual_enabled: false,
        }
    }
}

impl GpsDualStub {
    fn read_instance(backend: &mut SitlGpsBackend, truth: &GpsInstanceTruth) -> GpsFixState {
        backend.read(
            truth.velocity_ned,
            truth.latitude_deg,
            truth.longitude_deg,
            truth.altitude_m,
            truth.now_ms,
        );
        backend.delayed_state(truth.now_ms)
    }

    #[must_use]
    pub fn instance_status(&mut self, instance: u8) -> GpsStatus {
        let (backend, truth) = if instance == 0 {
            (&mut self.primary, &self.primary_truth)
        } else {
            (&mut self.secondary, &self.secondary_truth)
        };
        let fix = Self::read_instance(backend, truth);
        GpsStatus::from_fix(&fix, backend.lag_sec())
    }

    #[must_use]
    pub fn primary_status(&mut self) -> GpsStatus {
        self.instance_status(0)
    }

    #[must_use]
    pub fn secondary_status(&mut self) -> GpsStatus {
        self.instance_status(1)
    }

    /// Re-select primary to the first healthy instance, upstream `AP_GPS` UsePrimary failover.
    pub fn select_primary_healthy(&mut self) {
        if !self.dual_enabled || self.auto_switch != GpsAutoSwitch::UsePrimary {
            return;
        }
        for i in 0..2u8 {
            let status = self.instance_status(i);
            if GpsHealthFlags::from_status(&status).is_healthy() {
                self.primary_instance = i;
                return;
            }
        }
    }

    /// Dual stub with disabled primary for UsePrimary failover tests.
    #[must_use]
    pub fn with_disabled_primary() -> Self {
        let mut stub = Self::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UsePrimary;
        stub.primary.disabled = true;
        stub
    }

    fn blend_instances(&mut self) -> [GpsBlendInstance; 2] {
        [
            GpsBlendInstance::from_status(self.primary_status()),
            GpsBlendInstance::from_status(self.secondary_status()),
        ]
    }

    /// Active output status, upstream primary/blended instance selection.
    #[must_use]
    pub fn output_status(&mut self) -> GpsStatus {
        if !self.dual_enabled {
            return self.primary_status();
        }
        match self.auto_switch {
            GpsAutoSwitch::UsePrimary => {
                if self.primary_instance == 0 {
                    self.primary_status()
                } else {
                    self.secondary_status()
                }
            }
            GpsAutoSwitch::UseBest => {
                let p = self.primary_status();
                let s = self.secondary_status();
                if s.num_sats > p.num_sats {
                    s
                } else {
                    p
                }
            }
            GpsAutoSwitch::Blend => {
                let instances = self.blend_instances();
                if self.blender.calc_weights(&instances) {
                    self.blender.calc_state(&instances)
                } else if self.primary_status().have_fix {
                    self.primary_status()
                } else {
                    self.secondary_status()
                }
            }
        }
    }

    #[must_use]
    pub fn output_velocity(&mut self) -> GpsVelocitySample {
        let status = self.output_status();
        GpsVelocityProducer::publish_status(&status)
    }

    #[must_use]
    pub fn output_health(&mut self) -> GpsHealthFlags {
        self.output_health_at(self.primary_truth.now_ms)
    }

    #[must_use]
    pub fn output_health_at(&mut self, now_ms: u32) -> GpsHealthFlags {
        let status = self.output_status();
        GpsHealthFlags::from_status_at(&status, now_ms)
    }


    /// Active output instance index, upstream `AP_GPS::primary_instance()` / blended.
    #[must_use]
    pub fn output_active_instance(&mut self) -> u8 {
        if !self.dual_enabled {
            return 0;
        }
        match self.auto_switch {
            GpsAutoSwitch::UsePrimary => self.primary_instance,
            GpsAutoSwitch::UseBest => {
                let p = self.primary_status();
                let s = self.secondary_status();
                if s.num_sats > p.num_sats {
                    1
                } else {
                    0
                }
            }
            GpsAutoSwitch::Blend => {
                let instances = self.blend_instances();
                if self.blender.calc_weights(&instances) {
                    GPS_BLENDED_INSTANCE
                } else if self.primary_status().have_fix {
                    self.primary_instance
                } else if self.secondary_status().have_fix {
                    1 - self.primary_instance
                } else {
                    self.primary_instance
                }
            }
        }
    }

    #[must_use]
    pub fn output_is_blended(&self) -> bool {
        self.dual_enabled && self.auto_switch == GpsAutoSwitch::Blend && self.blender.output_is_blended()
    }

    #[must_use]
    pub const fn gps_lag_sec(&self) -> f32 {
        crate::sitl::SITL_GPS_DEFAULT_LAG_SEC
    }

    #[must_use]
    pub const fn rate_ms(&self) -> u32 {
        SITL_GPS_UPDATE_MS
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FixType;

    #[test]
    fn dual_stub_single_instance_matches_primary() {
        let mut stub = GpsDualStub::default();
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        let status = stub.output_status();
        assert!(status.have_fix);
        assert!((status.velocity_ned.x - 5.0).abs() < 1e-3);
    }

    #[test]
    fn dual_stub_blend_averages_velocity() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::Blend;
        stub.primary_truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(6.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let status = stub.output_status();
        assert_eq!(status.fix_type, FixType::Fix3D);
        assert!((status.velocity_ned.x - 8.0).abs() < 0.5);
        assert!(stub.output_is_blended());
    }
    #[test]
    fn use_best_selects_higher_satellite_count() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UseBest;
        stub.primary.num_sats = 10;
        stub.secondary.num_sats = 18;
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        assert_eq!(stub.output_active_instance(), 1);
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 9.0).abs() < 1e-3);
    }

    #[test]
    fn use_primary_failover_selects_secondary_when_primary_disabled() {
        let mut stub = GpsDualStub::with_disabled_primary();
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        stub.select_primary_healthy();
        assert_eq!(stub.primary_instance, 1);
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 8.0).abs() < 1e-3);
        assert_eq!(stub.output_active_instance(), 1);
    }

}
