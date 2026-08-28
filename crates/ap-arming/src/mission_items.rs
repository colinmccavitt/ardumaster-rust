//! `ARMING_MIS_ITEMS` / mission-item pre-arm check. FW-026.
//!
//! Upstream `AP_Arming::_required_mission_items` / `ARMING_MIS_ITEMS` is
//! the bitmask of items the plan must contain. This slice is the Plane
//! takeoff gate that fills `Check::Mission`: refuse when the mission is
//! empty (no flown items — seq 0 is home) or the first flown item is
//! not `MAV_CMD_NAV_TAKEOFF`. Land / rally / RTL bits are a later slice.

use crate::{Check, NamedCheck};

/// Default `ARMING_MIS_ITEMS`, upstream `_required_mission_items`.
pub const ARMING_MIS_ITEMS_DEFAULT: u32 = 0;

/// Bit 0 — `MAV_CMD_NAV_LAND` must be in the plan.
pub const MIS_ITEM_CHECK_LAND: u32 = 1 << 0;
/// Bit 1 — `MAV_CMD_NAV_VTOL_LAND`.
pub const MIS_ITEM_CHECK_VTOL_LAND: u32 = 1 << 1;
/// Bit 2 — `MAV_CMD_DO_LAND_START`.
pub const MIS_ITEM_CHECK_DO_LAND_START: u32 = 1 << 2;
/// Bit 3 — `MAV_CMD_NAV_TAKEOFF`.
pub const MIS_ITEM_CHECK_TAKEOFF: u32 = 1 << 3;
/// Bit 4 — `MAV_CMD_NAV_VTOL_TAKEOFF`.
pub const MIS_ITEM_CHECK_VTOL_TAKEOFF: u32 = 1 << 4;
/// Bit 5 — a sufficiently close rally point.
pub const MIS_ITEM_CHECK_RALLY: u32 = 1 << 5;
/// Bit 6 — `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
pub const MIS_ITEM_CHECK_RETURN_TO_LAUNCH: u32 = 1 << 6;

/// `MAV_CMD_NAV_TAKEOFF` — first flown item this gate accepts.
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;

/// `MAV_CMD_NAV_WAYPOINT` — a typical non-takeoff first item.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// Command #0 is home; first flown item is seq 1.
/// Upstream `AP_MISSION_FIRST_REAL_COMMAND`.
pub const FIRST_REAL_COMMAND: u16 = 1;

/// Registry name for the mission named check.
pub const MISSION_CHECK_NAME: &str = "MISSION";

/// Whether `ARMING_MIS_ITEMS` requires this item bit.
#[must_use]
pub const fn mis_item_required(required_items: u32, bit: u32) -> bool {
    (required_items & bit) != 0
}

/// Whether the mission has a flown item and it starts with takeoff.
///
/// `num_commands` includes home at seq 0, upstream `num_commands()`.
/// `<= FIRST_REAL_COMMAND` means empty of flown items.
#[must_use]
pub const fn mission_starts_with_takeoff(num_commands: u16, first_flown_command: u16) -> bool {
    if num_commands <= FIRST_REAL_COMMAND {
        return false;
    }
    first_flown_command == MAV_CMD_NAV_TAKEOFF
}

/// Fill `Check::Mission` from the mission list (empty / first-item takeoff).
#[must_use]
pub const fn mission_items_named_check(num_commands: u16, first_flown_command: u16) -> NamedCheck {
    NamedCheck {
        check: Check::Mission,
        name: MISSION_CHECK_NAME,
        ok: mission_starts_with_takeoff(num_commands, first_flown_command),
    }
}
