//! Airspeed health scheduler hookup for the plane main loop.
//!
//! Upstream AP_Airspeed::update() timer path, primary failover, and health
//! flags before AHRS drift motion consumes pitot TAS.

use ap_airspeed::sitl::{AirspeedHealthFlags, AirspeedSampleState};

use crate::sitl_airspeed_hookup::SitlAirspeedHookup;

/// Per-tick inputs for the airspeed health scheduler.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedHealthSchedulerInputs {
    pub eas2tas: f32,
}

/// Per-tick airspeed health and pitot sample output.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AirspeedHealthSchedulerOutput {
    pub sample: AirspeedSampleState,
    pub healthy: bool,
    pub health: AirspeedHealthFlags,
    /// True when primary instance changed this tick, upstream failover.
    pub primary_switched: bool,
    /// Whether TAS is used for TECS/nav, upstream `ARSPD_USE`.
    pub use_airspeed: bool,
}

/// Run timer tick, primary selection, and health refresh.
#[must_use]
pub fn airspeed_health_scheduler_tick(
    hookup: &mut SitlAirspeedHookup,
    inp: &AirspeedHealthSchedulerInputs,
) -> AirspeedHealthSchedulerOutput {
    let prev_primary = hookup.cluster().primary();
    let published = hookup.publish(inp.eas2tas);
    AirspeedHealthSchedulerOutput {
        sample: published.sample,
        healthy: published.healthy,
        health: published.health,
        primary_switched: published.health.primary != prev_primary,
        use_airspeed: published.use_airspeed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;
    use crate::sitl_airspeed_hookup::{hookup_with_disabled_primary, SitlAirspeedTruth};

    #[test]
    fn scheduler_emits_healthy_primary_sample() {
        let mut hookup = SitlAirspeedHookup::default();
        hookup.truth = SitlAirspeedTruth {
            airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
            now_ms: 10,
        };
        let out = airspeed_health_scheduler_tick(
            &mut hookup,
            &AirspeedHealthSchedulerInputs { eas2tas: 1.0 },
        );
        assert!(out.sample.have_sample);
        assert!(out.healthy);
        assert!(out.health.primary_healthy());
        assert!(!out.primary_switched);
    }

    #[test]
    fn scheduler_failover_switches_primary_when_disabled() {
        let mut hookup = hookup_with_disabled_primary();
        hookup.truth = SitlAirspeedTruth {
            airspeed_bf: Vector3f::new(18.0, 0.0, 0.0),
            now_ms: 10,
        };
        assert_eq!(hookup.cluster().primary(), 0);
        let out = airspeed_health_scheduler_tick(
            &mut hookup,
            &AirspeedHealthSchedulerInputs { eas2tas: 1.0 },
        );
        assert!(out.primary_switched);
        assert_eq!(out.health.primary, 1);
        assert!(out.health.primary_healthy());
    }
}
