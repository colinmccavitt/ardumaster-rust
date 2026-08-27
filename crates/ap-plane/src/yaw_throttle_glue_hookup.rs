//! Pilot throttle and VTOL yaw stick glue for the main vehicle loop.
//!
//! Upstream `Mode::output_pilot_throttle` maps the RC throttle stick into the
//! scaled output the stabilize path and SRV registry consume. When
//! `StickMixing::VtolYaw` is active, the yaw stick is mixed into the rudder
//! demand after the yaw controller runs.

use ap_math::scalar::constrain_value;

use crate::mode_run::{PilotThrottleSource, StickMixing};
use crate::rc_failsafe_scheduler_hookup::{percent_input, RcChannelConfig};
use crate::stabilize_hookup::AP_PLANE_TRIM_THROTTLE_DEFAULT;

/// Throttle glue inputs for one scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotThrottleGlueInputs {
    pub throttle_pwm: Option<u16>,
    pub throttle_cfg: RcChannelConfig,
    pub pilot_throttle_source: PilotThrottleSource,
    pub trim_throttle: f32,
    pub throttle_min: f32,
    pub throttle_max: f32,
    pub use_throttle_limits: bool,
    pub use_battery_compensation: bool,
    /// Pack voltage ratio vs nominal; 1.0 disables compensation effect.
    pub battery_voltage_ratio: f32,
}

impl Default for PilotThrottleGlueInputs {
    fn default() -> Self {
        Self {
            throttle_pwm: None,
            throttle_cfg: RcChannelConfig::default(),
            pilot_throttle_source: PilotThrottleSource::TrimAdjusted,
            trim_throttle: AP_PLANE_TRIM_THROTTLE_DEFAULT,
            throttle_min: 0.0,
            throttle_max: 100.0,
            use_throttle_limits: true,
            use_battery_compensation: true,
            battery_voltage_ratio: 1.0,
        }
    }
}

/// VTOL yaw stick glue inputs applied after the yaw controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VtolYawStickGlueInputs {
    pub stick_mixing: Option<StickMixing>,
    pub yaw_norm_dz: f32,
    /// Rudder demand limit, scaled centidegrees.
    pub rudder_limit_scaled: f32,
}

impl Default for VtolYawStickGlueInputs {
    fn default() -> Self {
        Self {
            stick_mixing: None,
            yaw_norm_dz: 0.0,
            rudder_limit_scaled: 4500.0,
        }
    }
}

/// Map RC throttle to scaled 0..100, upstream `Mode::output_pilot_throttle`.
#[must_use]
pub fn map_pilot_throttle(
    throttle_pwm: u16,
    cfg: &RcChannelConfig,
    source: PilotThrottleSource,
    trim_throttle: f32,
) -> f32 {
    let pct = f32::from(percent_input(throttle_pwm, cfg));
    match source {
        PilotThrottleSource::Direct => pct,
        PilotThrottleSource::TrimAdjusted => {
            if pct <= 50.0 {
                pct / 50.0 * trim_throttle
            } else {
                trim_throttle + (pct - 50.0) / 50.0 * (100.0 - trim_throttle)
            }
        }
    }
}

/// Apply configured throttle limits when the active mode allows them.
#[must_use]
pub fn apply_throttle_limits(throttle: f32, use_limits: bool, min: f32, max: f32) -> f32 {
    if use_limits {
        constrain_value(throttle, min, max)
    } else {
        throttle
    }
}

/// Scale throttle for sagging pack voltage when battery compensation is on.
#[must_use]
pub fn apply_battery_compensation(throttle: f32, use_comp: bool, voltage_ratio: f32) -> f32 {
    if use_comp && voltage_ratio > 0.01 {
        constrain_value(throttle / voltage_ratio, 0.0, 100.0)
    } else {
        throttle
    }
}

/// One pilot-throttle glue tick: RC stick to scaled output.
#[must_use]
pub fn pilot_throttle_glue_tick(inp: &PilotThrottleGlueInputs) -> f32 {
    let Some(pwm) = inp.throttle_pwm else {
        return 0.0;
    };
    let mapped = map_pilot_throttle(
        pwm,
        &inp.throttle_cfg,
        inp.pilot_throttle_source,
        inp.trim_throttle,
    );
    let limited = apply_throttle_limits(
        mapped,
        inp.use_throttle_limits,
        inp.throttle_min,
        inp.throttle_max,
    );
    apply_battery_compensation(
        limited,
        inp.use_battery_compensation,
        inp.battery_voltage_ratio,
    )
}

/// Mix VTOL yaw stick into rudder demand when `StickMixing::VtolYaw` is set.
#[must_use]
pub fn vtol_yaw_stick_glue_tick(rudder_scaled: f32, inp: &VtolYawStickGlueInputs) -> f32 {
    if !matches!(inp.stick_mixing, Some(StickMixing::VtolYaw)) {
        return rudder_scaled;
    }
    let mixed = rudder_scaled + inp.yaw_norm_dz * inp.rudder_limit_scaled;
    constrain_value(
        mixed,
        -inp.rudder_limit_scaled,
        inp.rudder_limit_scaled,
    )
}
