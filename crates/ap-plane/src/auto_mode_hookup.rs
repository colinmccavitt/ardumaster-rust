//! AUTO mode glue for the main vehicle loop.
//!
//! Upstream ModeAuto::_enter calls mission.start_or_resume().
//! ModeAuto::navigate calls mission.update() once home is set, which
//! starts or advances the current nav command. Stabilization stays on the
//! default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_auto_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Auto)
}

/// Inputs for AUTO mission start / advance (ModeAuto enter plus navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoModeMissionInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream AP_Mission::state() == MISSION_RUNNING.
    pub mission_running: bool,
    /// Upstream AP::ahrs().home_is_set().
    pub home_is_set: bool,
    pub waypoint_count: u8,
    pub current_index: u16,
}

/// Result of the AUTO mission start / advance tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoModeMissionOutput {
    pub mission_running: bool,
    pub current_index: u16,
    /// start_or_resume armed the mission this tick.
    pub started: bool,
    /// navigate will call mission.update this tick.
    pub allow_advance: bool,
    pub applied: bool,
}

/// Start the mission on AUTO entry (or if AUTO is flying without one) and
/// gate mission.update() on home, matching ModeAuto enter and navigate.
#[must_use]
pub fn auto_mode_mission_tick(inp: &AutoModeMissionInputs) -> AutoModeMissionOutput {
    if !is_auto_mode(inp.control_mode, &inp.features) {
        return AutoModeMissionOutput {
            mission_running: inp.mission_running,
            current_index: inp.current_index,
            started: false,
            allow_advance: false,
            applied: false,
        };
    }

    let mut running = inp.mission_running;
    let mut index = inp.current_index;
    let mut started = false;

    if (inp.mode_just_entered || !running) && inp.waypoint_count > 0 {
        if !running {
            index = 0;
            started = true;
        }
        running = true;
    }

    AutoModeMissionOutput {
        mission_running: running,
        current_index: index,
        started,
        allow_advance: running && inp.home_is_set,
        applied: true,
    }
}
