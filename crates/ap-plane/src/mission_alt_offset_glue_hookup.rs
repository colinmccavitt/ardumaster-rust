//! Mission altitude offset glue from the mission scheduler tick.
//!
//! Upstream `Plane::mission_alt_offset()` and `target_altitude.offset_cm` feed
//! TECS height demand. The port carries scheduler `offset_cm` on the vehicle
//! for the altitude TECS feed path.

use crate::target_altitude::TargetAltitude;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionAltOffsetGlueInputs {
    pub offset_cm: i32,
    pub target: TargetAltitude,
}

impl Default for MissionAltOffsetGlueInputs {
    fn default() -> Self {
        Self {
            offset_cm: 0,
            target: TargetAltitude::FromNextWaypoint,
        }
    }
}

#[must_use]
pub fn mission_alt_offset_glue_tick(inp: MissionAltOffsetGlueInputs) -> i32 {
    if matches!(inp.target, TargetAltitude::HoldCurrentAndResetOffset) {
        return 0;
    }
    inp.offset_cm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passes_scheduler_offset_cm() {
        assert_eq!(
            mission_alt_offset_glue_tick(MissionAltOffsetGlueInputs {
                offset_cm: 500,
                target: TargetAltitude::ProportionalToNextWaypoint,
            }),
            500
        );
    }

    #[test]
    fn resets_on_hold_current_and_reset_offset() {
        assert_eq!(
            mission_alt_offset_glue_tick(MissionAltOffsetGlueInputs {
                offset_cm: 2500,
                target: TargetAltitude::HoldCurrentAndResetOffset,
            }),
            0
        );
    }
}
