//! `in_vtol_mode` / `in_vtol_auto` stub, upstream `QuadPlane::in_vtol_mode`
//! / `QuadPlane::in_vtol_auto` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. These two methods decide whether the vehicle
//! is flying as a VTOL right now: a Q* mode (`Mode::is_vtol_mode`), or
//! AUTO flying a VTOL nav command.
//!
//! Upstream reads `plane.control_mode`, `plane.auto_state`, and
//! `plane.mission.get_current_nav_cmd()`. This crate does not own those
//! objects, so the caller passes a [`VtolModeView`].
//!
//! Land-sequence / `poscontrol` state (`QPOS_APPROACH` / `AIRBRAKE`)
//! and `Q_OPTIONS` `ALLOW_FW_TAKEOFF` / `ALLOW_FW_LAND` are later
//! slices. Unset options mean `NAV_TAKEOFF` / `NAV_LAND` count as VTOL
//! when the QuadPlane is available, matching the upstream default.

use crate::QuadPlane;

/// `MAV_CMD_NAV_LOITER_UNLIM`.
pub const MAV_CMD_NAV_LOITER_UNLIM: u16 = 17;
/// `MAV_CMD_NAV_LOITER_TURNS`.
pub const MAV_CMD_NAV_LOITER_TURNS: u16 = 18;
/// `MAV_CMD_NAV_LOITER_TIME`.
pub const MAV_CMD_NAV_LOITER_TIME: u16 = 19;
/// `MAV_CMD_NAV_LAND`.
pub const MAV_CMD_NAV_LAND: u16 = 21;
/// `MAV_CMD_NAV_TAKEOFF`.
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;
/// `MAV_CMD_NAV_LOITER_TO_ALT`.
pub const MAV_CMD_NAV_LOITER_TO_ALT: u16 = 31;
/// `MAV_CMD_NAV_VTOL_TAKEOFF`.
pub const MAV_CMD_NAV_VTOL_TAKEOFF: u16 = 84;
/// `MAV_CMD_NAV_VTOL_LAND`.
pub const MAV_CMD_NAV_VTOL_LAND: u16 = 85;
/// `MAV_CMD_NAV_PAYLOAD_PLACE`.
pub const MAV_CMD_NAV_PAYLOAD_PLACE: u16 = 94;
/// `MAV_CMD_NAV_WAYPOINT` — a non-VTOL AUTO command used in tests.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// The control-mode kind `in_vtol_mode` / `in_vtol_auto` branch on.
///
/// This is not Plane's `ModeNumber` table. QuadPlane only needs to
/// know whether the live mode is a Q* VTOL mode, AUTO, GUIDED, or
/// something else.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlKind {
    /// `Mode::is_vtol_mode()` — QStabilize, QHover, QLoiter, QLand,
    /// QRTL, QAcro, QAutotune.
    Vtol,
    /// `control_mode == &plane.mode_auto`.
    Auto,
    /// `control_mode->is_guided_mode()` (GUIDED / AVOID_ADSB).
    Guided,
    /// Any other fixed-wing mode (FBWA, MANUAL, RTL, …).
    Other,
}

/// What QuadPlane reads from Plane for the two VTOL-mode predicates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VtolModeView {
    /// Live control-mode kind.
    pub control: ControlKind,
    /// `mission.get_current_nav_cmd().id`.
    pub nav_cmd_id: u16,
    /// `auto_state.vtol_mode`.
    pub auto_vtol_mode: bool,
    /// `auto_state.vtol_loiter`.
    pub auto_vtol_loiter: bool,
    /// `guided_takeoff` — a GUIDED VTOL takeoff is in progress.
    pub guided_takeoff: bool,
}

impl Default for VtolModeView {
    fn default() -> Self {
        Self::new()
    }
}

impl VtolModeView {
    /// Fixed-wing, no AUTO nav command, no guided takeoff.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            control: ControlKind::Other,
            nav_cmd_id: 0,
            auto_vtol_mode: false,
            auto_vtol_loiter: false,
            guided_takeoff: false,
        }
    }

    /// A Q* VTOL mode (`Mode::is_vtol_mode()`).
    #[must_use]
    pub const fn q_mode() -> Self {
        Self {
            control: ControlKind::Vtol,
            nav_cmd_id: 0,
            auto_vtol_mode: false,
            auto_vtol_loiter: false,
            guided_takeoff: false,
        }
    }

    /// AUTO flying `nav_cmd_id`.
    #[must_use]
    pub const fn auto(nav_cmd_id: u16) -> Self {
        Self {
            control: ControlKind::Auto,
            nav_cmd_id,
            auto_vtol_mode: false,
            auto_vtol_loiter: false,
            guided_takeoff: false,
        }
    }

    /// GUIDED, optionally in a VTOL takeoff.
    #[must_use]
    pub const fn guided(guided_takeoff: bool) -> Self {
        Self {
            control: ControlKind::Guided,
            nav_cmd_id: 0,
            auto_vtol_mode: false,
            auto_vtol_loiter: false,
            guided_takeoff,
        }
    }
}

impl QuadPlane {
    /// Upstream `QuadPlane::is_vtol_takeoff`.
    ///
    /// `NAV_VTOL_TAKEOFF` is always a VTOL takeoff. `NAV_TAKEOFF` is
    /// treated as VTOL when the QuadPlane is available (`Q_OPTIONS`
    /// `ALLOW_FW_TAKEOFF` is a later slice; the default bit is unset).
    #[must_use]
    pub const fn is_vtol_takeoff(&self, id: u16) -> bool {
        if id == MAV_CMD_NAV_VTOL_TAKEOFF {
            return true;
        }
        id == MAV_CMD_NAV_TAKEOFF && self.available()
    }

    /// Upstream `QuadPlane::is_vtol_land`.
    ///
    /// `NAV_VTOL_LAND` / `NAV_PAYLOAD_PLACE` are VTOL landings. `NAV_LAND`
    /// is treated as VTOL when available (`ALLOW_FW_LAND` and the
    /// spiral-approach stage are later slices).
    #[must_use]
    pub const fn is_vtol_land(&self, id: u16) -> bool {
        if id == MAV_CMD_NAV_VTOL_LAND || id == MAV_CMD_NAV_PAYLOAD_PLACE {
            return true;
        }
        id == MAV_CMD_NAV_LAND && self.available()
    }

    /// Upstream `QuadPlane::in_vtol_auto`.
    ///
    /// False unless [`Self::available`] and the vehicle is in AUTO.
    /// Then true when `auto_state.vtol_mode` is set, or the current
    /// nav command is a VTOL takeoff / land / (optionally) loiter.
    #[must_use]
    pub fn in_vtol_auto(&self, view: &VtolModeView) -> bool {
        if !self.available() {
            return false;
        }
        if view.control != ControlKind::Auto {
            return false;
        }
        if view.auto_vtol_mode {
            return true;
        }
        match view.nav_cmd_id {
            MAV_CMD_NAV_VTOL_TAKEOFF => true,
            MAV_CMD_NAV_LOITER_UNLIM
            | MAV_CMD_NAV_LOITER_TIME
            | MAV_CMD_NAV_LOITER_TURNS
            | MAV_CMD_NAV_LOITER_TO_ALT => view.auto_vtol_loiter,
            MAV_CMD_NAV_TAKEOFF => self.is_vtol_takeoff(view.nav_cmd_id),
            MAV_CMD_NAV_VTOL_LAND | MAV_CMD_NAV_LAND | MAV_CMD_NAV_PAYLOAD_PLACE => {
                self.is_vtol_land(view.nav_cmd_id)
            }
            _ => false,
        }
    }

    /// Upstream `QuadPlane::in_vtol_mode`.
    ///
    /// True when available and either the live mode is a Q* VTOL mode,
    /// GUIDED is doing a VTOL takeoff, or [`Self::in_vtol_auto`] is
    /// true for a non-loiter VTOL command.
    ///
    /// A VTOL loiter still in the approach / airbrake poscontrol
    /// states is not `in_vtol_mode` upstream; without poscontrol this
    /// stub keeps that case false.
    #[must_use]
    pub fn in_vtol_mode(&self, view: &VtolModeView) -> bool {
        if !self.available() {
            return false;
        }
        if view.control == ControlKind::Vtol {
            return true;
        }
        if view.control == ControlKind::Guided && view.guided_takeoff {
            return true;
        }
        if self.in_vtol_auto(view) && !view.auto_vtol_loiter {
            return true;
        }
        false
    }
}
