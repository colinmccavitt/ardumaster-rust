//! Landing-sequence gate for RC short/long failsafe.
//!
//! Upstream `Plane::failsafe_in_landing_sequence` in `ArduPlane/events.cpp`.
//! True when `flight_stage == LAND`, the mission in-landing-sequence flag
//! is set, or QuadPlane is in a VTOL land sequence. AUTO / AUTOLAND then
//! skip `rc_failsafe_short_on_event` and `failsafe_long_on_event` mode
//! changes (the FALLTHROUGH into the action table does not run).
//!
//! Landing-sequence / Q_OPTIONS gates were deferred from
//! [`crate::failsafe_action_hookup`]. This stub is that gate; it does not
//! rewrite the `FS_SHORT_ACTN` / `FS_LONG_ACTN` table.

use crate::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use crate::mode_table::ModeNumber;
use ap_tecs::params::FlightStage;

/// Inputs for `Plane::failsafe_in_landing_sequence`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandingSequenceInputs {
    /// `flight_stage`.
    pub flight_stage: FlightStage,
    /// `mission.get_in_landing_sequence_flag()`.
    pub mission_in_landing_sequence: bool,
    /// `quadplane.in_vtol_land_sequence()`.
    pub vtol_land_sequence: bool,
}

impl Default for LandingSequenceInputs {
    fn default() -> Self {
        Self {
            flight_stage: FlightStage::Normal,
            mission_in_landing_sequence: false,
            vtol_land_sequence: false,
        }
    }
}

impl LandingSequenceInputs {
    /// `flight_stage == LAND`.
    #[must_use]
    pub fn land_stage() -> Self {
        Self {
            flight_stage: FlightStage::Land,
            ..Self::default()
        }
    }

    /// `mission.get_in_landing_sequence_flag()`.
    #[must_use]
    pub fn mission_flag() -> Self {
        Self {
            mission_in_landing_sequence: true,
            ..Self::default()
        }
    }

    /// `quadplane.in_vtol_land_sequence()`.
    #[must_use]
    pub fn vtol_land() -> Self {
        Self {
            vtol_land_sequence: true,
            ..Self::default()
        }
    }
}

/// Upstream `Plane::failsafe_in_landing_sequence`.
///
/// Intended only for failsafe code. Any one of LAND stage, the mission
/// landing-sequence flag, or a QuadPlane VTOL land sequence is enough.
#[must_use]
pub fn failsafe_in_landing_sequence(inp: &LandingSequenceInputs) -> bool {
    inp.flight_stage == FlightStage::Land
        || inp.mission_in_landing_sequence
        || inp.vtol_land_sequence
}

/// AUTO / AUTOLAND skip the RC failsafe action table when in landing sequence.
///
/// Short: both AUTO and AUTOLAND `break` before FALLTHROUGH.
/// Long: AUTO `break`s; AUTOLAND is already a no-action mode, so this is
/// a no-op there but keeps the same gate the short path uses.
#[must_use]
pub fn skip_rc_failsafe_in_landing_sequence(mode: ModeNumber, inp: &LandingSequenceInputs) -> bool {
    matches!(mode, ModeNumber::Auto | ModeNumber::Autoland) && failsafe_in_landing_sequence(inp)
}

/// `rc_failsafe_short_on_event` after the landing-sequence gate.
///
/// Does not rewrite [`short_failsafe_action`]; AUTO/AUTOLAND in a landing
/// sequence stay put, otherwise the existing table decides.
#[must_use]
pub fn gated_short_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionShort,
    inp: &LandingSequenceInputs,
) -> FailsafeActionResult {
    if skip_rc_failsafe_in_landing_sequence(mode, inp) {
        return FailsafeActionResult::Continue;
    }
    short_failsafe_action(mode, action)
}

/// `failsafe_long_on_event` after the landing-sequence gate.
///
/// Does not rewrite [`long_failsafe_action`].
#[must_use]
pub fn gated_long_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionLong,
    autoland_available: bool,
    inp: &LandingSequenceInputs,
) -> FailsafeActionResult {
    if skip_rc_failsafe_in_landing_sequence(mode, inp) {
        return FailsafeActionResult::Continue;
    }
    long_failsafe_action(mode, action, autoland_available)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn land_stage_or_mission_flag_or_vtol_is_enough() {
        assert!(!failsafe_in_landing_sequence(
            &LandingSequenceInputs::default()
        ));
        assert!(failsafe_in_landing_sequence(&LandingSequenceInputs {
            flight_stage: FlightStage::Land,
            ..LandingSequenceInputs::default()
        }));
        assert!(failsafe_in_landing_sequence(&LandingSequenceInputs {
            mission_in_landing_sequence: true,
            ..LandingSequenceInputs::default()
        }));
        assert!(failsafe_in_landing_sequence(&LandingSequenceInputs {
            vtol_land_sequence: true,
            ..LandingSequenceInputs::default()
        }));
        assert!(!failsafe_in_landing_sequence(&LandingSequenceInputs {
            flight_stage: FlightStage::AbortLanding,
            ..LandingSequenceInputs::default()
        }));
    }
}
