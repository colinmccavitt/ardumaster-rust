//! SITL barometer producer wired into AHRS drift motion, upstream `AP_Baro_SITL` → `get_EAS2TAS()`.
//!
//! [`SitlBaroHookup`] runs the [`SitlBaroCluster`] timer/update path and publishes
//! pressure, altitude, EAS2TAS, and per-instance health before
//! [`PlaneMainLoop::ahrs_update`] builds [`DriftMotionInputs`](ap_ahrs::DriftMotionInputs).

use ap_baro::eas2tas_for_alt_amsl;
use ap_baro::sitl::{
    BaroClimbRate, BaroGroundPressure, BaroHealthFlags, BaroSampleState, SitlBaroBackend,
    SitlBaroCluster, SitlBaroConfig,
};
use ap_math::vector3::Vector3f;

/// Sim truth fed into the SITL baro backend each tick.
#[derive(Debug, Clone, Copy)]
pub struct SitlBaroTruth {
    pub sim_altitude_m: f32,
    pub airspeed_bf: Vector3f,
    pub now_ms: u32,
    pub noise_sample: f32,
}

impl Default for SitlBaroTruth {
    fn default() -> Self {
        Self {
            sim_altitude_m: 0.0,
            airspeed_bf: Vector3f::zero(),
            now_ms: 0,
            noise_sample: 0.0,
        }
    }
}

/// SITL baro cluster hookup for the vehicle main loop.
#[derive(Debug, Clone)]
pub struct SitlBaroHookup {
    cluster: SitlBaroCluster,
    climb_rate: BaroClimbRate,
    ground: BaroGroundPressure,
    pub truth: SitlBaroTruth,
}

impl Default for SitlBaroHookup {
    fn default() -> Self {
        Self {
            cluster: SitlBaroCluster::default(),
            climb_rate: BaroClimbRate::default(),
            ground: BaroGroundPressure::default(),
            truth: SitlBaroTruth::default(),
        }
    }
}

/// Baro sample, EAS2TAS, and health published before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SitlBaroPublish {
    pub sample: BaroSampleState,
    pub eas2tas: f32,
    /// Primary instance healthy, upstream `AP_Baro::healthy()`.
    pub healthy: bool,
    pub health: BaroHealthFlags,
    /// Filtered climb rate from primary altitude, upstream `get_climb_rate()`.
    pub climb_rate_mps: f32,
    /// Altitude above latched ground pressure when calibrated.
    pub relative_altitude_m: Option<f32>,
}

impl SitlBaroHookup {
    /// One primary plus one secondary SITL baro backend.
    #[must_use]
    pub fn with_dual_backends() -> Self {
        let mut cluster = SitlBaroCluster::default();
        let _ = cluster.register(SitlBaroBackend::default());
        Self {
            cluster,
            climb_rate: BaroClimbRate::default(),
            ground: BaroGroundPressure::default(),
            truth: SitlBaroTruth::default(),
        }
    }

    #[must_use]
    pub const fn cluster(&self) -> &SitlBaroCluster {
        &self.cluster
    }

    #[must_use]
    pub fn backend(&self) -> Option<&SitlBaroBackend> {
        self.cluster.backend(self.cluster.primary())
    }

    /// Run timer tick and frontend update, upstream `AP_Baro_SITL::_timer` + `update`.
    #[must_use]
    /// Latch current primary pressure as ground reference.
    pub fn latch_ground_pressure(&mut self) {
        if let Some(sample) = self.cluster.primary_sample() {
            if sample.have_sample {
                self.ground.latch(sample.pressure_pa, sample.temp_c);
            }
        }
    }

    #[must_use]
    pub const fn ground(&self) -> &BaroGroundPressure {
        &self.ground
    }

    pub fn publish(&mut self) -> SitlBaroPublish {
        self.cluster.timer_tick_all(
            self.truth.sim_altitude_m,
            self.truth.airspeed_bf,
            self.truth.now_ms,
            self.truth.noise_sample,
        );
        self.cluster.select_primary_healthy();
        let health = self.cluster.health_flags();
        let sample = self
            .cluster
            .primary_sample()
            .unwrap_or(*self.cluster.backend(self.cluster.primary()).unwrap().state());
        let eas2tas = if sample.have_sample {
            eas2tas_for_alt_amsl(sample.altitude_m)
        } else {
            1.0
        };
        let healthy = health.primary_healthy();
        self.climb_rate.update_primary(
            sample.altitude_m,
            sample.last_sample_time_ms,
            healthy,
            health.primary,
        );
        let relative_altitude_m = self.ground.relative_altitude_m(sample.pressure_pa);
        SitlBaroPublish {
            sample,
            eas2tas,
            healthy,
            health,
            climb_rate_mps: self.climb_rate.climb_rate_mps(healthy),
            relative_altitude_m,
        }
    }
}

/// Mark secondary instance unhealthy when disabled, for dual-baro tests.
#[must_use]
pub fn secondary_disabled_cluster() -> SitlBaroCluster {
    let mut cluster = SitlBaroCluster::default();
    let secondary = SitlBaroBackend::with_config(SitlBaroConfig {
        disabled: true,
        ..SitlBaroConfig::default()
    });
    let _ = cluster.register(secondary);
    cluster
}

/// Hookup with a disabled secondary instance for health-flag tests.
#[must_use]
pub fn hookup_with_disabled_secondary() -> SitlBaroHookup {
    SitlBaroHookup {
        cluster: secondary_disabled_cluster(),
        climb_rate: BaroClimbRate::default(),
        ground: BaroGroundPressure::default(),
        truth: SitlBaroTruth::default(),
    }
}

/// Hookup with a disabled primary instance for failover tests.
#[must_use]
pub fn hookup_with_disabled_primary() -> SitlBaroHookup {
    SitlBaroHookup {
        cluster: SitlBaroCluster::cluster_with_disabled_primary(),
        climb_rate: BaroClimbRate::default(),
        ground: BaroGroundPressure::default(),
        truth: SitlBaroTruth::default(),
    }
}
