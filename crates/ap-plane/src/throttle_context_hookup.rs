//! Throttle rules context hookup for the scheduler tick.
//!
//! Upstream `Mode::use_throttle_limits`, `Mode::use_battery_compensation`, and
//! `Mode::output_pilot_throttle` are mode-dependent. This module builds the
//! [`ThrottleContext`](crate::throttle_rules::ThrottleContext) from the active
//! mode each `update_control_mode` tick.

use crate::mode_run::{pilot_throttle_source, PilotThrottleSource};
use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::throttle_rules::{
    manual_use_battery_compensation, manual_use_throttle_limits, use_battery_compensation,
    use_throttle_limits, ThrottleContext,
};

/// HAL inputs for one throttle-context tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleContextInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub nav_scripting_active: bool,
    pub throttle_passthru_stabilize: bool,
    pub guided_throttle_passthru: bool,
    pub allow_forward_throttle_in_vtol: bool,
    pub quadplane_available: bool,
    pub idle_gov_manual: bool,
}

impl Default for ThrottleContextInputs {
    fn default() -> Self {
        Self {
            control_mode: ModeNumber::FlyByWireB.as_number(),
            features: BuildFeatures::default(),
            nav_scripting_active: false,
            throttle_passthru_stabilize: false,
            guided_throttle_passthru: false,
            allow_forward_throttle_in_vtol: true,
            quadplane_available: false,
            idle_gov_manual: false,
        }
    }
}

/// Result of one throttle-context tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleContextOutput {
    pub use_throttle_limits: bool,
    pub use_battery_compensation: bool,
    pub pilot_throttle_source: PilotThrottleSource,
}

fn is_manual_throttle_mode(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::Stabilize
            | ModeNumber::Training
            | ModeNumber::Acro
            | ModeNumber::FlyByWireA
            | ModeNumber::Autotune
    )
}

fn is_vtol_mode(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::QStabilize
            | ModeNumber::QHover
            | ModeNumber::QLoiter
            | ModeNumber::QLand
            | ModeNumber::QRtl
            | ModeNumber::QAutotune
            | ModeNumber::QAcro
            | ModeNumber::LoiterAltQLand
    )
}

/// Resolve throttle limit/battery flags and pilot throttle source for the mode.
#[must_use]
pub fn throttle_context_tick(inp: &ThrottleContextInputs) -> ThrottleContextOutput {
    let pilot = pilot_throttle_source(inp.throttle_passthru_stabilize);

    let Some(mode) = ModeNumber::from_number(inp.control_mode, &inp.features) else {
        return ThrottleContextOutput {
            use_throttle_limits: true,
            use_battery_compensation: true,
            pilot_throttle_source: pilot,
        };
    };

    if mode == ModeNumber::Manual {
        return ThrottleContextOutput {
            use_throttle_limits: manual_use_throttle_limits(
                inp.quadplane_available,
                inp.idle_gov_manual,
            ),
            use_battery_compensation: manual_use_battery_compensation(),
            pilot_throttle_source: pilot,
        };
    }

    let context = ThrottleContext {
        nav_scripting_active: inp.nav_scripting_active,
        manual_throttle_mode: is_manual_throttle_mode(mode),
        throttle_passthru_stabilize: inp.throttle_passthru_stabilize,
        guided_throttle_passthru: inp.guided_throttle_passthru,
        in_vtol_mode: is_vtol_mode(mode),
        allow_forward_throttle_in_vtol: inp.allow_forward_throttle_in_vtol,
    };

    ThrottleContextOutput {
        use_throttle_limits: use_throttle_limits(&context),
        use_battery_compensation: use_battery_compensation(&context),
        pilot_throttle_source: pilot,
    }
}
