//! Cross-module calc_throttle glue for the set_servos scheduler tick.

use crate::calc_throttle_glue_hookup::{calc_throttle_glue_tick, CalcThrottleGlueInputs};
use crate::landing_hookup::ServoOutputState;
use crate::mode_table::BuildFeatures;
use crate::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetServosGlueInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub tecs_throttle_demand: f32,
    pub throttle_nudge: i16,
    pub landing_throttle_applied: bool,
    pub disarm_throttle_applied: bool,
    pub mode_entry_applied: bool,
    pub mode_glue_throttle_restored: bool,
    pub pilot_throttle: PilotThrottleGlueInputs,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetServosGlueOutput {
    pub servos: ServoOutputState,
    pub stabilize_throttle: f32,
    pub applied: bool,
}

#[must_use]
pub fn set_servos_calc_throttle_tick(
    servos: ServoOutputState,
    inp: &SetServosGlueInputs,
) -> SetServosGlueOutput {
    if inp.landing_throttle_applied
        || inp.disarm_throttle_applied
        || inp.mode_entry_applied
        || inp.mode_glue_throttle_restored
    {
        return SetServosGlueOutput {
            servos,
            stabilize_throttle: servos.throttle_scaled,
            applied: false,
        };
    }
    let throttle = calc_throttle_glue_tick(&CalcThrottleGlueInputs {
        control_mode: inp.control_mode,
        features: inp.features,
        tecs_throttle_demand: inp.tecs_throttle_demand,
        throttle_nudge: inp.throttle_nudge,
        pilot_throttle: inp.pilot_throttle,
    });
    if servos.throttle_scaled > 0.0 && throttle == 0.0 {
        return SetServosGlueOutput {
            servos,
            stabilize_throttle: servos.throttle_scaled,
            applied: false,
        };
    }
    let mut servos = servos;
    servos.throttle_scaled = throttle;
    SetServosGlueOutput {
        servos,
        stabilize_throttle: throttle,
        applied: true,
    }
}
