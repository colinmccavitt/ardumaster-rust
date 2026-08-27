//! SRV output mapping scheduler hookup for the `set_servos` tick.
//!
//! Upstream `Plane::set_servos` calls elevon/V-tail mixing, `set_servos_flaps`,
//! and `flaperon_update` after the attitude controllers publish demands.

use ap_servo::function::Function;
use ap_servo::registry::Registry;

use crate::landing_hookup::ServoOutputState;
use crate::srv_output_hookup::{
    apply_elevon_mixing, apply_vtail_mixing, set_servos_flaps, update_dspoilers,
    DspoilerHookupInputs, FlapDeployInputs, MixingParams,
};

/// Flap speed schedule parameters, upstream `FLAP_*` and takeoff/landing flaps.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FlapSpeedParams {
    pub flap_1_speed: i16,
    pub flap_1_percent: i8,
    pub flap_2_speed: i16,
    pub flap_2_percent: i8,
    pub takeoff_flap_percent: i8,
    pub landing_flap_percent: i8,
    pub flap_slewrate: f32,
}

/// HAL inputs for one SRV output scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SrvOutputSchedulerInputs {
    pub mixing: MixingParams,
    pub flap_params: FlapSpeedParams,
    pub manual_flap_percent: i8,
    /// Airspeed source for the flap schedule, metres per second.
    pub flap_speed_source_ms: f32,
    pub has_auto_flap_schedule: bool,
    pub flight_stage_is_takeoff: bool,
    pub flight_stage_is_land: bool,
    pub apply_elevon_mixing: bool,
    pub apply_vtail_mixing: bool,
    pub apply_dspoiler_mixing: bool,
    pub dspoiler: DspoilerHookupInputs,
    pub dt: f32,
    /// Elevator scaled centidegrees for registry seed (`servos` only stores PWM).
    pub elevator_scaled: f32,
}

/// Persistent vehicle-side SRV output hookup state.
#[derive(Debug, Clone)]
pub struct SrvOutputHookupState {
    pub registry: Registry,
    pub mixing: MixingParams,
    pub flap_params: FlapSpeedParams,
    pub manual_flap_percent: i8,
    pub apply_elevon_mixing: bool,
    pub apply_vtail_mixing: bool,
    pub has_auto_flap_schedule: bool,
    pub flight_stage_is_takeoff: bool,
    pub apply_dspoiler_mixing: bool,
    pub dspoiler: DspoilerHookupInputs,
    pub last_auto_flap_percent: i8,
}

impl Default for SrvOutputHookupState {
    fn default() -> Self {
        Self {
            registry: Registry::new(),
            mixing: MixingParams::default(),
            flap_params: FlapSpeedParams::default(),
            manual_flap_percent: 0,
            apply_elevon_mixing: false,
            apply_vtail_mixing: false,
            has_auto_flap_schedule: false,
            flight_stage_is_takeoff: false,
            apply_dspoiler_mixing: false,
            dspoiler: DspoilerHookupInputs::default(),
            last_auto_flap_percent: 0,
        }
    }
}

/// Result of one SRV output scheduler tick.
#[derive(Debug, Clone, PartialEq)]
pub struct SrvOutputSchedulerOutput {
    pub servos: ServoOutputState,
    pub auto_flap_percent: i8,
}

/// Compute auto flap percent from speed and flight stage, upstream
/// `Plane::set_servos_flaps` before the manual override merge.
#[must_use]
pub fn auto_flap_percent_from_speed(
    flap_speed_source_ms: f32,
    params: &FlapSpeedParams,
    flight_stage_is_takeoff: bool,
    flight_stage_is_land: bool,
) -> i8 {
    if flight_stage_is_takeoff && params.takeoff_flap_percent != 0 {
        return params.takeoff_flap_percent;
    }
    if flight_stage_is_land && params.landing_flap_percent != 0 {
        return params.landing_flap_percent;
    }

    let speed = flap_speed_source_ms as i16;
    if params.flap_2_speed != 0 && speed <= params.flap_2_speed {
        params.flap_2_percent
    } else if params.flap_1_speed != 0 && speed <= params.flap_1_speed {
        params.flap_1_percent
    } else {
        0
    }
}

/// Apply SRV output mapping during the `set_servos` scheduler tick.
#[must_use]
pub fn srv_output_scheduler_tick(
    servos: ServoOutputState,
    state: &mut SrvOutputHookupState,
    inp: &SrvOutputSchedulerInputs,
) -> SrvOutputSchedulerOutput {
    let reg = &mut state.registry;
    reg.set_output_scaled(Function::AILERON, servos.aileron_scaled);
    reg.set_output_scaled(Function::ELEVATOR, inp.elevator_scaled);
    reg.set_output_scaled(Function::RUDDER, servos.rudder_scaled);
    reg.set_output_scaled(Function::THROTTLE, servos.throttle_scaled);

    let auto_flap = if inp.has_auto_flap_schedule {
        auto_flap_percent_from_speed(
            inp.flap_speed_source_ms,
            &inp.flap_params,
            inp.flight_stage_is_takeoff,
            inp.flight_stage_is_land,
        )
    } else {
        0
    };

    set_servos_flaps(
        reg,
        FlapDeployInputs {
            manual_flap_percent: inp.manual_flap_percent,
            auto_flap_percent: auto_flap,
            flap_slewrate: inp.flap_params.flap_slewrate,
            dt: inp.dt,
        },
    );

    if inp.apply_elevon_mixing {
        apply_elevon_mixing(reg, inp.mixing);
    }
    if inp.apply_vtail_mixing {
        apply_vtail_mixing(reg, inp.mixing);
    }
    if inp.apply_dspoiler_mixing {
        update_dspoilers(reg, inp.dspoiler);
    }

    state.last_auto_flap_percent = auto_flap;

    SrvOutputSchedulerOutput {
        servos,
        auto_flap_percent: auto_flap,
    }
}
