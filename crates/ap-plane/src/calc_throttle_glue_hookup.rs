//! calc_throttle helpers for the vehicle glue path.
//!
//! Upstream `Plane::calc_throttle` reads TECS throttle demand in auto-throttle
//! modes and falls back to the pilot stick elsewhere.

use ap_math::scalar::constrain_value;

use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::yaw_throttle_glue_hookup::{pilot_throttle_glue_tick, PilotThrottleGlueInputs};

/// HAL inputs for one calc_throttle glue tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalcThrottleGlueInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// TECS throttle demand 0..100, upstream `get_throttle_demand()`.
    pub tecs_throttle_demand: f32,
    /// Mission/GCS nudge, percent. Upstream `throttle_nudge`.
    pub throttle_nudge: i16,
    pub pilot_throttle: PilotThrottleGlueInputs,
}

fn does_auto_throttle(mode: ModeNumber) -> bool {
    !matches!(
        mode,
        ModeNumber::Manual
            | ModeNumber::Stabilize
            | ModeNumber::Training
            | ModeNumber::Acro
            | ModeNumber::FlyByWireA
            | ModeNumber::Autotune
            | ModeNumber::QAcro
            | ModeNumber::Initialising
            | ModeNumber::Circle
    )
}

/// Apply throttle nudge to a TECS demand, upstream calc_throttle nudge path.
#[must_use]
pub fn apply_throttle_nudge(throttle: f32, nudge: i16) -> f32 {
    constrain_value(throttle + f32::from(nudge), 0.0, 100.0)
}

/// Resolve throttle for the active mode: TECS+nudge or pilot stick glue.
#[must_use]
pub fn calc_throttle_glue_tick(inp: &CalcThrottleGlueInputs) -> f32 {
    let Some(mode) = ModeNumber::from_number(inp.control_mode, &inp.features) else {
        return pilot_throttle_glue_tick(&inp.pilot_throttle);
    };
    if does_auto_throttle(mode) {
        apply_throttle_nudge(inp.tecs_throttle_demand, inp.throttle_nudge)
    } else {
        pilot_throttle_glue_tick(&inp.pilot_throttle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mode_run::PilotThrottleSource;
    use crate::mode_table::ModeNumber;
    use crate::rc_failsafe_scheduler_hookup::RcChannelConfig;

    #[test]
    fn auto_mode_uses_tecs_with_nudge() {
        let thr = calc_throttle_glue_tick(&CalcThrottleGlueInputs {
            control_mode: ModeNumber::Auto.as_number(),
            features: BuildFeatures::default(),
            tecs_throttle_demand: 60.0,
            throttle_nudge: 5,
            pilot_throttle: PilotThrottleGlueInputs::default(),
        });
        assert!((thr - 65.0).abs() < 1e-6);
    }

    #[test]
    fn manual_mode_uses_pilot_glue() {
        let thr = calc_throttle_glue_tick(&CalcThrottleGlueInputs {
            control_mode: ModeNumber::Manual.as_number(),
            features: BuildFeatures::default(),
            tecs_throttle_demand: 60.0,
            throttle_nudge: 0,
            pilot_throttle: PilotThrottleGlueInputs {
                throttle_pwm: Some(2000),
                throttle_cfg: RcChannelConfig {
                    radio_min: 1000,
                    radio_max: 2000,
                    ..Default::default()
                },
                pilot_throttle_source: PilotThrottleSource::Direct,
                ..Default::default()
            },
        });
        assert!((thr - 100.0).abs() < 1e-6);
    }
}
