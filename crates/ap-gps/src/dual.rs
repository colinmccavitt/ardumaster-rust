//! Dual-GPS SITL stub, upstream two-receiver `AP_GPS` frontend. FW-012.
//!
//! Holds two [`SitlGpsBackend`] instances and optionally blends their lag-buffered
//! outputs when [`GpsAutoSwitch::Blend`] is selected.

use crate::blend::{
    GpsAutoSwitch, GpsBlendInstance, GpsBlender, GPS_BLEND_MASK_DEFAULT,
    GPS_BLENDED_INSTANCE,
};
use crate::params::GpsParams;
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
    pub fn apply_params(&mut self, params: GpsParams) {
        self.dual_enabled = params.dual_enabled();
        self.auto_switch = params.auto_switch;
        self.primary_instance = params.primary.min(1);
        self.blender = GpsBlender::new(params.blend_mask);
        params.apply_instance(0, &mut self.primary);
        params.apply_instance(1, &mut self.secondary);
    }

    #[must_use]
    pub const fn blend_mask(&self) -> u8 {
        self.blender.blend_mask()
    }

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

    #[must_use]
    fn instance_now_ms(&self, instance: u8) -> u32 {
        if instance == 0 {
            self.primary_truth.now_ms
        } else {
            self.secondary_truth.now_ms
        }
    }

    #[must_use]
    pub fn instance_health_at(&mut self, instance: u8, now_ms: u32) -> GpsHealthFlags {
        let (backend, truth) = if instance == 0 {
            (&mut self.primary, &self.primary_truth)
        } else {
            (&mut self.secondary, &self.secondary_truth)
        };
        let fix = backend.delayed_state(now_ms);
        let status = if fix.have_fix {
            GpsStatus::from_fix(&fix, backend.lag_sec())
        } else {
            let fix = Self::read_instance(backend, truth);
            GpsStatus::from_fix(&fix, backend.lag_sec())
        };
        GpsHealthFlags::from_status_at(&status, now_ms)
    }

    /// Pick the best healthy instance for UseBest, upstream `AP_GPS` auto-switch.
    fn use_best_instance(&mut self) -> u8 {
        let p_health = self.instance_health_at(0, self.primary_truth.now_ms);
        let s_health = self.instance_health_at(1, self.secondary_truth.now_ms);
        match (p_health.is_healthy(), s_health.is_healthy()) {
            (true, true) => {
                let p = self.primary_status();
                let s = self.secondary_status();
                if s.num_sats > p.num_sats {
                    1
                } else if p.num_sats > s.num_sats {
                    0
                } else {
                    self.primary_instance
                }
            }
            (true, false) => 0,
            (false, true) => 1,
            (false, false) => {
                let p = self.primary_status();
                let s = self.secondary_status();
                if s.num_sats > p.num_sats {
                    1
                } else if p.num_sats > s.num_sats {
                    0
                } else {
                    self.primary_instance
                }
            }
        }
    }

    /// Re-select primary to the first healthy instance, upstream `AP_GPS` UsePrimary failover.
    pub fn select_primary_healthy(&mut self) {
        if !self.dual_enabled || self.auto_switch != GpsAutoSwitch::UsePrimary {
            return;
        }
        let order = [self.primary_instance, 1 - self.primary_instance];
        for i in order {
            let now_ms = self.instance_now_ms(i);
            if self.instance_health_at(i, now_ms).is_healthy() {
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



    #[must_use]
    fn blend_freshness(&mut self) -> (bool, bool) {
        let p_health = self.instance_health_at(0, self.primary_truth.now_ms);
        let s_health = self.instance_health_at(1, self.secondary_truth.now_ms);
        (
            p_health.fix_fresh && p_health.is_healthy(),
            s_health.fix_fresh && s_health.is_healthy(),
        )
    }

    #[must_use]
    fn blend_fallback_instance(&mut self) -> u8 {
        if self.primary_status().have_fix {
            0
        } else {
            1
        }
    }

    fn clear_blend_output(&mut self) {
        self.blender = GpsBlender::new(GPS_BLEND_MASK_DEFAULT);
    }

    /// Blend output with freshness fallback, upstream `AP_GPS` blended instance.
    #[must_use]

    #[must_use]

    #[must_use]
    fn blend_output_active_instance(&mut self) -> u8 {
        let (p_fresh, s_fresh) = self.blend_freshness();
        match (p_fresh, s_fresh) {
            (true, true) => {
                let instances = self.blend_instances();
                if self.blender.calc_weights(&instances) {
                    GPS_BLENDED_INSTANCE
                } else {
                    self.blend_fallback_instance()
                }
            }
            (true, false) => 0,
            (false, true) => 1,
            (false, false) => self.blend_fallback_instance(),
        }
    }

    fn blend_output_health(&mut self) -> GpsHealthFlags {
        let (p_fresh, s_fresh) = self.blend_freshness();
        let p_health = self.instance_health_at(0, self.primary_truth.now_ms);
        let s_health = self.instance_health_at(1, self.secondary_truth.now_ms);
        match (p_fresh, s_fresh) {
            (true, true) => GpsHealthFlags {
                have_fix: p_health.have_fix && s_health.have_fix,
                has_3d_fix: p_health.has_3d_fix && s_health.has_3d_fix,
                num_sats_ok: p_health.num_sats_ok && s_health.num_sats_ok,
                velocity_valid: p_health.velocity_valid && s_health.velocity_valid,
                fix_fresh: p_health.fix_fresh && s_health.fix_fresh,
            },
            (true, false) => p_health,
            (false, true) => s_health,
            (false, false) => {
                if self.primary_status().have_fix {
                    p_health
                } else {
                    s_health
                }
            }
        }
    }

    fn blend_output_status(&mut self) -> GpsStatus {
        let (p_fresh, s_fresh) = self.blend_freshness();
        match (p_fresh, s_fresh) {
            (true, true) => {
                let instances = self.blend_instances();
                if self.blender.calc_weights(&instances) {
                    self.blender.calc_state(&instances)
                } else {
                    self.clear_blend_output();
                    if self.primary_status().have_fix {
                        self.primary_status()
                    } else {
                        self.secondary_status()
                    }
                }
            }
            (true, false) => {
                self.clear_blend_output();
                self.primary_status()
            }
            (false, true) => {
                self.clear_blend_output();
                self.secondary_status()
            }
            (false, false) => {
                self.clear_blend_output();
                if self.primary_status().have_fix {
                    self.primary_status()
                } else {
                    self.secondary_status()
                }
            }
        }
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
                if self.use_best_instance() == 1 {
                    self.secondary_status()
                } else {
                    self.primary_status()
                }
            }
            GpsAutoSwitch::Blend => self.blend_output_status(),
        }
    }

    #[must_use]
    pub fn output_velocity(&mut self) -> GpsVelocitySample {
        let status = self.output_status();
        GpsVelocityProducer::publish_status(&status)
    }

    #[must_use]
    pub fn output_health(&mut self) -> GpsHealthFlags {
        self.output_health_for_active_instance()
    }

    #[must_use]
    pub fn output_health_at(&mut self, _now_ms: u32) -> GpsHealthFlags {
        self.output_health_for_active_instance()
    }

    #[must_use]
    fn output_health_for_active_instance(&mut self) -> GpsHealthFlags {
        if !self.dual_enabled {
            return self.instance_health_at(0, self.primary_truth.now_ms);
        }
        match self.auto_switch {
            GpsAutoSwitch::Blend => self.blend_output_health(),
            GpsAutoSwitch::UsePrimary => {
                let inst = self.primary_instance;
                self.instance_health_at(inst, self.instance_now_ms(inst))
            }
            GpsAutoSwitch::UseBest => {
                let inst = self.use_best_instance();
                self.instance_health_at(inst, self.instance_now_ms(inst))
            }
        }
    }


    /// Active output instance index, upstream `AP_GPS::primary_instance()` / blended.
    #[must_use]
    pub fn output_active_instance(&mut self) -> u8 {
        if !self.dual_enabled {
            return 0;
        }
        match self.auto_switch {
            GpsAutoSwitch::UsePrimary => self.primary_instance,
            GpsAutoSwitch::UseBest => self.use_best_instance(),
            GpsAutoSwitch::Blend => self.blend_output_active_instance(),
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

    #[test]
    fn use_primary_failover_skips_stale_primary() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UsePrimary;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 10;
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let _ = stub.primary_status();
        let _ = stub.secondary_status();
        stub.primary_truth.now_ms = 5000;
        stub.select_primary_healthy();
        assert_eq!(stub.primary_instance, 1);
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 8.0).abs() < 1e-3);
    }

    #[test]
    fn use_best_prefers_fresh_secondary_over_stale_high_sat_primary() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UseBest;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 10;
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let _ = stub.primary_status();
        let _ = stub.secondary_status();
        stub.primary_truth.now_ms = 5000;
        assert_eq!(stub.output_active_instance(), 1);
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 9.0).abs() < 1e-3);
    }

    #[test]
    fn blend_falls_back_to_fresh_secondary_when_primary_stale() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::Blend;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 12;
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(7.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let _ = stub.primary_status();
        let _ = stub.secondary_status();
        stub.primary_truth.now_ms = 5000;
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 7.0).abs() < 1e-3);
        assert!(!stub.output_is_blended());
    }

}
    #[test]
    fn blend_health_follows_fresh_secondary_when_primary_stale() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::Blend;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 12;
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let _ = stub.primary_status();
        let _ = stub.secondary_status();
        stub.primary_truth.now_ms = 5000;
        let health = stub.output_health();
        assert!(health.is_healthy());
        assert_eq!(stub.output_active_instance(), 1);
    }

    #[test]
    fn blend_health_requires_both_when_both_fresh() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::Blend;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 12;
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        let _ = stub.primary_status();
        let _ = stub.secondary_status();
        let health = stub.output_health();
        assert!(health.is_healthy());
        assert_eq!(stub.output_active_instance(), GPS_BLENDED_INSTANCE);
    }

    #[test]
    fn use_best_tie_prefers_configured_primary() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UseBest;
        stub.primary_instance = 1;
        stub.primary.num_sats = 12;
        stub.secondary.num_sats = 12;
        stub.primary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        assert_eq!(stub.output_active_instance(), 1);
    }

    #[test]
    fn use_primary_failover_scans_configured_primary_first() {
        let mut stub = GpsDualStub::default();
        stub.dual_enabled = true;
        stub.auto_switch = GpsAutoSwitch::UsePrimary;
        stub.primary_instance = 1;
        stub.primary.num_sats = 18;
        stub.secondary.num_sats = 10;
        stub.primary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        stub.secondary_truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
        stub.primary_truth.now_ms = 200;
        stub.secondary_truth.now_ms = 200;
        stub.select_primary_healthy();
        assert_eq!(stub.primary_instance, 1);
        let status = stub.output_status();
        assert!((status.velocity_ned.x - 5.0).abs() < 1e-3);
    }

