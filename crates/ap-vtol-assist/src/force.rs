//! Force-assist / Q_OPTIONS path for VTOL assistance.
//!
//! Upstream `VTOL_Assist::should_assist` force half (`force_assist =
//! state == FORCE_ENABLED`) plus the QuadPlane option bits that latch
//! it. This module does not rewrite [`crate::assist`] enable / check
//! or [`crate::speed_alt`].
//!
//! `Q_OPTIONS` bit 7 (`Q_ASSIST_FORCE_ENABLE`) forces assist on. That
//! path overrides the speed / alt gate: when `Q_ASSIST_SPEED <= 0`
//! every speed / alt / angle check is skipped and only force-enable
//! still returns true. Aux LOW (`ASSIST_DISABLED`) still wins.
//!
//! When force-assist is live and the aircraft is armed, the VTOL
//! motors should sit in the spin-when-armed warning spool (upstream
//! `DesiredSpoolState::GROUND_IDLE`) so the operator sees they can
//! become active. `Q_ASSIST_OPTIONS` `SPIN_DISABLED` is spin
//! *recovery* and is not this flag.
//!
//! `Q_OPTIONS` bit 12 (`DISABLE_SYNTHETIC_AIRSPEED_ASSIST`) does not
//! force assist; it only requires a real airspeed sensor for the
//! speed trigger.

use crate::assist::{
    q_assist_force_enable_set, AssistState, VtolAssist, DISABLE_SYNTHETIC_AIRSPEED_ASSIST,
};
use crate::speed_alt::SpeedAltDecision;

/// Inputs the force / option-bit half of `should_assist` needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ForceSample {
    /// Live `Q_OPTIONS` bitmask.
    pub q_options: u32,
    /// Armed with safety off. Upstream `arming.is_armed_and_safety_off`.
    pub armed: bool,
}

impl ForceSample {
    /// Build a sample from `Q_OPTIONS` and the arming latch.
    #[must_use]
    pub const fn new(q_options: u32, armed: bool) -> Self {
        Self { q_options, armed }
    }
}

/// Result of one force / option-bit evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ForceDecision {
    force_assist: bool,
    spin_while_armed: bool,
}

impl ForceDecision {
    /// No force request; motors stay idle. Upstream `reset` of the
    /// force half when assist is disabled.
    #[must_use]
    pub const fn idle() -> Self {
        Self {
            force_assist: false,
            spin_while_armed: false,
        }
    }

    /// Upstream `in_force_assist` / `force_assist`.
    #[must_use]
    pub const fn force_assist(&self) -> bool {
        self.force_assist
    }

    /// Motors should spin-when-armed (`GROUND_IDLE` warning spool).
    #[must_use]
    pub const fn spin_while_armed(&self) -> bool {
        self.spin_while_armed
    }

    /// Whether force-assist is requesting. Overrides a closed speed /
    /// alt gate.
    #[must_use]
    pub const fn requested(&self) -> bool {
        self.force_assist
    }

    /// True when this decision alone opens assist with the speed / alt
    /// checks skipped.
    #[must_use]
    pub const fn overrides_speed_alt(&self) -> bool {
        self.force_assist
    }
}

/// Whether `Q_OPTIONS` has `DISABLE_SYNTHETIC_AIRSPEED_ASSIST` set.
#[must_use]
pub const fn disable_synthetic_airspeed_assist_set(q_options: u32) -> bool {
    (q_options & DISABLE_SYNTHETIC_AIRSPEED_ASSIST) != 0
}

/// Speed-assist may use a synthetic airspeed estimate.
///
/// Upstream `should_assist` speed line: if bit 12 is set, require
/// `ahrs.using_airspeed_sensor()`. This bit never forces assist.
#[must_use]
pub const fn synthetic_airspeed_assist_allowed(
    q_options: u32,
    using_airspeed_sensor: bool,
) -> bool {
    using_airspeed_sensor || !disable_synthetic_airspeed_assist_set(q_options)
}

/// Whether the force latch is live: aux HIGH / `FORCE_ENABLED`, or
/// `Q_OPTIONS` bit 7. Aux LOW wins over the option bit.
#[must_use]
pub fn force_assist_latched(assist: &VtolAssist, q_options: u32) -> bool {
    if assist.state() == AssistState::AssistDisabled {
        return false;
    }
    assist.state() == AssistState::ForceEnabled || q_assist_force_enable_set(q_options)
}

/// Evaluate the force / option-bit half of `should_assist`.
///
/// - [`AssistState::AssistDisabled`]: flags cleared, no request.
/// - `FORCE_ENABLED` or `Q_ASSIST_FORCE_ENABLE`: force-assist on,
///   which overrides a closed speed / alt gate.
/// - [`ForceDecision::spin_while_armed`]: force-assist and armed.
#[must_use]
pub fn evaluate_force(assist: &VtolAssist, sample: ForceSample) -> ForceDecision {
    let force = force_assist_latched(assist, sample.q_options);
    if !force {
        return ForceDecision::idle();
    }
    ForceDecision {
        force_assist: true,
        spin_while_armed: sample.armed,
    }
}

/// Assist requested when force overrides a closed speed / alt gate.
///
/// Upstream `return force_assist || speed_assist || alt_error...`.
#[must_use]
pub const fn requested_overriding_speed_alt(
    force: ForceDecision,
    speed_alt: SpeedAltDecision,
) -> bool {
    force.requested() || speed_alt.requested()
}
