//! AUTO mode glue for the main vehicle loop.
//!
//! Upstream ModeAuto::_enter calls mission.start_or_resume().
//! ModeAuto::navigate calls mission.update() once home is set, which
//! starts or advances the current nav command. When the mission ends,
//! `exit_mission_callback` switches to RTL (`ModeReason::MISSION_END`)
//! unless the current nav command is NAV_LAND, which stays in AUTO and
//! hands off to the landing controller. Stabilization stays on the
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

/// Upstream `MAV_CMD_NAV_LAND`.
pub const MAV_CMD_NAV_LAND: u16 = 21;
/// Upstream `ModeReason::MISSION_END`.
pub const MODE_REASON_MISSION_END: u8 = 8;

/// Inputs for AUTO mission-complete / landing handoff
/// (`Plane::exit_mission_callback` and the `ModeAuto::update` NAV_LAND path).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoModeCompleteInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// Upstream AP_Mission::state() == MISSION_RUNNING.
    pub mission_running: bool,
    /// Mission has no remaining nav items (or the last item just completed).
    pub mission_complete: bool,
    /// Current nav command is `MAV_CMD_NAV_LAND`.
    pub current_nav_is_land: bool,
}

/// Result of the AUTO mission-complete / landing handoff tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoModeCompleteOutput {
    /// `exit_mission_callback` / AUTO-without-mission switches to RTL.
    pub switch_to_rtl: bool,
    /// Stay in AUTO and hand the NAV_LAND command to the landing controller.
    pub allow_land: bool,
    /// `ModeReason::MISSION_END` when `switch_to_rtl`, otherwise 0.
    pub reason: u8,
    pub applied: bool,
}

/// Hand off a finished AUTO mission to RTL, or a current NAV_LAND command
/// to the landing controller, matching `exit_mission_callback` and
/// `ModeAuto::update`.
#[must_use]
pub fn auto_mode_complete_tick(inp: &AutoModeCompleteInputs) -> AutoModeCompleteOutput {
    if !is_auto_mode(inp.control_mode, &inp.features) {
        return AutoModeCompleteOutput {
            switch_to_rtl: false,
            allow_land: false,
            reason: 0,
            applied: false,
        };
    }

    let ended = inp.mission_complete || !inp.mission_running;
    let allow_land = inp.current_nav_is_land && (inp.mission_running || inp.mission_complete);
    let switch_to_rtl = ended && !allow_land;

    AutoModeCompleteOutput {
        switch_to_rtl,
        allow_land,
        reason: if switch_to_rtl {
            MODE_REASON_MISSION_END
        } else {
            0
        },
        applied: true,
    }
}
