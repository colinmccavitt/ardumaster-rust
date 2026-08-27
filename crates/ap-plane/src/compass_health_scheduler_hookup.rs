//! Compass health scheduler hookup for the plane main loop.
//!
//! Upstream AP_Compass::read() timer path, primary failover, and health
//! flags before AHRS yaw drift consumes the body-frame mag sample.

use ap_ahrs::YawCompassSample;
use ap_compass::GpsDeclinationFix;
use ap_compass::sitl::{CompassHealthFlags, MagSampleState};
use ap_math::matrix3::Matrix3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Per-tick inputs for the compass health scheduler.
#[derive(Debug, Clone, Copy)]
pub struct CompassHealthSchedulerInputs {
    pub attitude: Matrix3f,
    pub loop_dt: f32,
    pub gps: Option<GpsDeclinationFix>,
}

/// Per-tick compass health and mag sample output.
#[derive(Debug, Clone, Copy, Default)]
pub struct CompassHealthSchedulerOutput {
    pub sample: MagSampleState,
    pub healthy: bool,
    pub health: CompassHealthFlags,
    pub yaw_compass: Option<YawCompassSample>,
    /// True when primary instance changed this tick, upstream failover.
    pub primary_switched: bool,
}

/// Run timer tick, primary selection, and health refresh.
#[must_use]
pub fn compass_health_scheduler_tick(
    hookup: &mut SitlCompassHookup,
    inp: &CompassHealthSchedulerInputs,
) -> CompassHealthSchedulerOutput {
    let prev_primary = hookup.cluster().primary();
    let published = hookup.publish(inp.attitude, inp.loop_dt, inp.gps);
    CompassHealthSchedulerOutput {
        sample: published.sample,
        healthy: published.healthy,
        health: published.health,
        yaw_compass: published.yaw_compass,
        primary_switched: published.health.primary != prev_primary,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::{hookup_with_disabled_primary, SitlCompassTruth};

    #[test]
    fn scheduler_emits_healthy_primary_sample() {
        let mut hookup = SitlCompassHookup::default();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let out = compass_health_scheduler_tick(
            &mut hookup,
            &CompassHealthSchedulerInputs {
                attitude: Matrix3f::identity(),
                loop_dt: 0.0025,
                gps: None,
            },
        );
        assert!(out.sample.have_sample);
        assert!(out.healthy);
        assert!(out.health.primary_healthy());
        assert!(out.yaw_compass.is_some());
        assert!(!out.primary_switched);
    }

    #[test]
    fn scheduler_failover_switches_primary_when_disabled() {
        let mut hookup = hookup_with_disabled_primary();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        assert_eq!(hookup.cluster().primary(), 0);
        let out = compass_health_scheduler_tick(
            &mut hookup,
            &CompassHealthSchedulerInputs {
                attitude: Matrix3f::identity(),
                loop_dt: 0.0025,
                gps: None,
            },
        );
        assert!(out.primary_switched);
        assert_eq!(out.health.primary, 1);
        assert!(out.health.primary_healthy());
        assert!(out.yaw_compass.is_some());
    }
}
