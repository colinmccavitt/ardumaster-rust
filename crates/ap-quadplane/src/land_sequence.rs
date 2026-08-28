//! Leftover VTOL land-sequence predicates, upstream
//! `QuadPlane::in_vtol_land_approach` / `in_vtol_land_descent` /
//! `in_vtol_land_final` / `in_vtol_land_sequence` /
//! `in_vtol_land_poscontrol` / `in_vtol_airbrake` (Plane-4.7.0
//! `quadplane.cpp`).
//!
//! Tracked as **VT-001**. Plane owns `control_mode` and the current
//! mission nav command; the caller passes a [`LandSequenceView`].
//! Poscontrol state is read from [`crate::poscontrol`]. This is not a
//! rewrite of [`crate::landing`] detect / complete, [`crate::auto_vtol`]
//! mission dispatch, or [`crate::position_controller`].

use crate::quadplane_completeness::{
    airbrake_state, land_approach_state, land_descent_state, land_final_state,
    land_poscontrol_state, land_sequence, qrtl_approach_state,
};
use crate::QuadPlane;

/// What QuadPlane reads from Plane for the land-sequence predicates.
///
/// Upstream reads `plane.control_mode`, `in_vtol_auto()`, and
/// `is_vtol_land(mission.get_current_nav_cmd().id)`. This crate does
/// not own those objects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LandSequenceView {
    /// `plane.control_mode == &plane.mode_qrtl`.
    pub in_qrtl: bool,
    /// `plane.control_mode == &plane.mode_auto`.
    ///
    /// `in_vtol_airbrake` uses this, not [`Self::in_vtol_auto`].
    pub in_auto: bool,
    /// `QuadPlane::in_vtol_auto()`.
    pub in_vtol_auto: bool,
    /// `is_vtol_land(mission.get_current_nav_cmd().id)`.
    pub is_vtol_land: bool,
}

impl LandSequenceView {
    /// Fixed-wing / other mode, not a VTOL land command.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_qrtl: false,
            in_auto: false,
            in_vtol_auto: false,
            is_vtol_land: false,
        }
    }

    /// QRTL. Sequence is true regardless of poscontrol.
    #[must_use]
    pub const fn qrtl() -> Self {
        Self {
            in_qrtl: true,
            in_auto: false,
            in_vtol_auto: false,
            is_vtol_land: false,
        }
    }

    /// AUTO flying a VTOL land command (`in_vtol_auto` + `is_vtol_land`).
    #[must_use]
    pub const fn auto_vtol_land() -> Self {
        Self {
            in_qrtl: false,
            in_auto: true,
            in_vtol_auto: true,
            is_vtol_land: true,
        }
    }
}

impl Default for LandSequenceView {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadPlane {
    /// Upstream `QuadPlane::in_vtol_land_approach`.
    ///
    /// QRTL while `poscontrol.get_state() <= QPOS_POSITION2`, or AUTO
    /// VTOL-land in APPROACH / AIRBRAKE / POSITION1 / POSITION2.
    #[must_use]
    pub const fn in_vtol_land_approach(&self, view: LandSequenceView) -> bool {
        let state = self.poscontrol().state();
        if view.in_qrtl && qrtl_approach_state(state) {
            return true;
        }
        view.in_vtol_auto && view.is_vtol_land && land_approach_state(state)
    }

    /// Upstream `QuadPlane::in_vtol_land_descent`.
    ///
    /// QRTL or AUTO VTOL-land in `QPOS_LAND_DESCEND` / `LAND_FINAL` /
    /// `LAND_ABORT`.
    #[must_use]
    pub const fn in_vtol_land_descent(&self, view: LandSequenceView) -> bool {
        let state = self.poscontrol().state();
        (view.in_qrtl || (view.in_vtol_auto && view.is_vtol_land)) && land_descent_state(state)
    }

    /// Upstream `QuadPlane::in_vtol_land_final`.
    ///
    /// Descent and `QPOS_LAND_FINAL`.
    #[must_use]
    pub const fn in_vtol_land_final(&self, view: LandSequenceView) -> bool {
        land_final_state(self.in_vtol_land_descent(view), self.poscontrol().state())
    }

    /// Upstream `QuadPlane::in_vtol_land_sequence`.
    ///
    /// `qrtl || approach || descent || final`.
    #[must_use]
    pub const fn in_vtol_land_sequence(&self, view: LandSequenceView) -> bool {
        land_sequence(
            view.in_qrtl,
            self.in_vtol_land_approach(view),
            self.in_vtol_land_descent(view),
            self.in_vtol_land_final(view),
        )
    }

    /// Upstream `QuadPlane::in_vtol_land_poscontrol`.
    ///
    /// AUTO VTOL-land with `poscontrol.get_state() >= QPOS_POSITION1`.
    #[must_use]
    pub const fn in_vtol_land_poscontrol(&self, view: LandSequenceView) -> bool {
        view.in_vtol_auto && view.is_vtol_land && land_poscontrol_state(self.poscontrol().state())
    }

    /// Upstream `QuadPlane::in_vtol_airbrake`.
    ///
    /// QRTL or AUTO (`control_mode == mode_auto`, not `in_vtol_auto`)
    /// plus a VTOL land command, while `QPOS_AIRBRAKE`.
    #[must_use]
    pub const fn in_vtol_airbrake(&self, view: LandSequenceView) -> bool {
        if !airbrake_state(self.poscontrol().state()) {
            return false;
        }
        view.in_qrtl || (view.in_auto && view.is_vtol_land)
    }
}
