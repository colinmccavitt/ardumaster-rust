//! SITL airspeed producer wired into AHRS drift motion and TECS, upstream `AP_Airspeed_SITL`.
//!
//! [`SitlAirspeedHookup`] runs the [`SitlAirspeedCluster`] timer/update path and
//! publishes pitot TAS/EAS samples and health before
//! [`PlaneMainLoop::ahrs_update`] builds [`DriftMotionInputs`](ap_ahrs::DriftMotionInputs).

use ap_airspeed::sitl::{
    AirspeedHealthFlags, AirspeedSampleState, SitlAirspeedBackend, SitlAirspeedCluster,
};
use ap_math::vector3::Vector3f;

/// Sim truth fed into the SITL airspeed backend each tick.
#[derive(Debug, Clone, Copy)]
pub struct SitlAirspeedTruth {
    pub airspeed_bf: Vector3f,
    pub now_ms: u32,
}

impl Default for SitlAirspeedTruth {
    fn default() -> Self {
        Self {
            airspeed_bf: Vector3f::zero(),
            now_ms: 0,
        }
    }
}

/// SITL airspeed cluster hookup for the vehicle main loop.
#[derive(Debug, Clone)]
pub struct SitlAirspeedHookup {
    cluster: SitlAirspeedCluster,
    pub truth: SitlAirspeedTruth,
}

impl Default for SitlAirspeedHookup {
    fn default() -> Self {
        Self {
            cluster: SitlAirspeedCluster::default(),
            truth: SitlAirspeedTruth::default(),
        }
    }
}

/// Pitot sample and health published before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SitlAirspeedPublish {
    pub sample: AirspeedSampleState,
    pub healthy: bool,
    pub health: AirspeedHealthFlags,
}

impl SitlAirspeedHookup {
    /// One primary plus one secondary SITL airspeed backend.
    #[must_use]
    pub fn with_dual_backends() -> Self {
        let mut cluster = SitlAirspeedCluster::default();
        let _ = cluster.register(SitlAirspeedBackend::default());
        Self {
            cluster,
            truth: SitlAirspeedTruth::default(),
        }
    }

    #[must_use]
    pub const fn cluster(&self) -> &SitlAirspeedCluster {
        &self.cluster
    }

    #[must_use]
    pub fn backend(&self) -> Option<&SitlAirspeedBackend> {
        self.cluster.backend(self.cluster.primary())
    }

    /// Latch pitot offsets on every enabled instance, upstream `calibrate()`.
    #[must_use]
    pub fn calibrate_offsets(&mut self) -> bool {
        self.cluster.calibrate_offsets()
    }

    /// Run timer tick and publish pitot TAS/EAS + health.
    #[must_use]
    pub fn publish(&mut self, eas2tas: f32) -> SitlAirspeedPublish {
        self.cluster.timer_tick_all(self.truth.airspeed_bf, eas2tas, self.truth.now_ms);
        self.cluster.select_primary_healthy();
        let health = self.cluster.health_flags();
        let sample = self
            .cluster
            .primary_sample()
            .unwrap_or(*self.cluster.backend(self.cluster.primary()).unwrap().state());
        let healthy = health.primary_healthy();
        SitlAirspeedPublish {
            sample,
            healthy,
            health,
        }
    }
}

/// Mark primary instance unhealthy when disabled, for dual-airspeed tests.
#[must_use]
pub fn hookup_with_disabled_primary() -> SitlAirspeedHookup {
    SitlAirspeedHookup {
        cluster: SitlAirspeedCluster::cluster_with_disabled_primary(),
        truth: SitlAirspeedTruth::default(),
    }
}
