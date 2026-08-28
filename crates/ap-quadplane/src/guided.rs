//! Leftover GUIDED / `Q_RTL_MODE` stub, upstream
//! `QuadPlane::guided_start` / `guided_update` / `RTL_MODE` /
//! `guided_mode_enabled` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. Plane owns current / next WP altitudes and
//! the GUIDED / AUTO mode pointers; the caller passes a
//! [`GuidedStartView`] / [`GuidedUpdateView`]. This is not a rewrite of
//! [`crate::landing`] `do_user_takeoff`, [`crate::mode_qrtl`],
//! [`crate::auto_vtol`] approach init, or [`crate::position_controller`].

use crate::auto_vtol::ApproachInitView;
use crate::poscontrol::PositionControlState;
use crate::quadplane_completeness::{
    guided_mode_enabled as guided_mode_is_enabled, guided_slow_descent, guided_update_climbing,
    rtl_mode_qrtl_always, rtl_mode_vtol_landing, RtlMode,
};
use crate::QuadPlane;

/// Default `Q_RTL_MODE`, upstream `AP_GROUPINFO("RTL_MODE", ..., rtl_mode, 0)`.
pub const Q_RTL_MODE_DEFAULT: i8 = 0;

/// Default `Q_GUIDED_MODE`, upstream `AP_GROUPINFO("GUIDED_MODE", ..., guided_mode, 0)`.
pub const Q_GUIDED_MODE_DEFAULT: i8 = 0;

/// Inputs [`QuadPlane::guided_start`] reads from Plane locations.
///
/// Upstream prefers absolute `get_alt_cm`; when that fails it compares
/// raw `Location.alt`. Approach geometry is handed to
/// [`QuadPlane::poscontrol_init_approach`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuidedStartView {
    /// Both `get_alt_cm(ABSOLUTE)` calls succeeded.
    pub abs_ok: bool,
    /// `current_loc` absolute altitude, cm.
    pub abs_from_alt_cm: i32,
    /// `next_WP_loc` absolute altitude, cm.
    pub abs_to_alt_cm: i32,
    /// `current_loc.alt` fallback, cm.
    pub current_alt_cm: i32,
    /// `next_WP_loc.alt` fallback, cm.
    pub next_wp_alt_cm: i32,
    /// Snapshot for [`QuadPlane::poscontrol_init_approach`].
    pub approach: ApproachInitView,
}

impl GuidedStartView {
    /// Absolute altitudes available; descent when `from > to`.
    #[must_use]
    pub const fn abs(from_alt_cm: i32, to_alt_cm: i32) -> Self {
        Self {
            abs_ok: true,
            abs_from_alt_cm: from_alt_cm,
            abs_to_alt_cm: to_alt_cm,
            current_alt_cm: from_alt_cm,
            next_wp_alt_cm: to_alt_cm,
            approach: ApproachInitView::far(),
        }
    }

    /// `get_alt_cm` failed — fall back to `Location.alt`.
    #[must_use]
    pub const fn loc_alt(current_alt_cm: i32, next_wp_alt_cm: i32) -> Self {
        Self {
            abs_ok: false,
            abs_from_alt_cm: 0,
            abs_to_alt_cm: 0,
            current_alt_cm,
            next_wp_alt_cm,
            approach: ApproachInitView::far(),
        }
    }
}

/// Side-effects of [`QuadPlane::guided_start`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GuidedStartResult {
    /// `setup_target_position` ran.
    pub setup_target: bool,
    /// `poscontrol_init_approach` ran.
    pub approach_inited: bool,
    /// `poscontrol.slow_descent` after the start.
    pub slow_descent: bool,
}

/// Inputs [`QuadPlane::guided_update`] reads from Plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GuidedUpdateView {
    /// `plane.control_mode == &plane.mode_guided`.
    pub in_guided: bool,
    /// `plane.current_loc.alt` (cm).
    pub current_alt_cm: i32,
    /// `plane.next_WP_loc.alt` (cm).
    pub next_wp_alt_cm: i32,
}

impl GuidedUpdateView {
    /// GUIDED, still below the takeoff target.
    #[must_use]
    pub const fn climbing(current_alt_cm: i32, next_wp_alt_cm: i32) -> Self {
        Self {
            in_guided: true,
            current_alt_cm,
            next_wp_alt_cm,
        }
    }

    /// GUIDED, at or above the takeoff target (or no climb).
    #[must_use]
    pub const fn arrived(current_alt_cm: i32, next_wp_alt_cm: i32) -> Self {
        Self {
            in_guided: true,
            current_alt_cm,
            next_wp_alt_cm,
        }
    }
}

/// Which [`QuadPlane::guided_update`] path ran.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuidedUpdateAction {
    /// Climbing GUIDED takeoff: `takeoff_controller`.
    TakeoffClimb,
    /// Finished takeoff or not climbing: `vtol_position_controller`.
    PositionHold,
}

/// Side-effects of [`QuadPlane::guided_update`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GuidedUpdateResult {
    /// Which controller path ran.
    pub action: GuidedUpdateAction,
    /// `throttle_wait` after the tick.
    pub throttle_wait: bool,
    /// `set_desired_spool_state(THROTTLE_UNLIMITED)` on the climb path.
    pub spool_unlimited: bool,
    /// `guided_takeoff` caused `QPOS_POSITION2` this tick.
    pub entered_position2: bool,
}

/// Inputs [`QuadPlane::guided_mode_enabled`] reads from Plane / mission.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GuidedModeView {
    /// `plane.control_mode == &plane.mode_guided`.
    pub in_guided: bool,
    /// `plane.control_mode == &plane.mode_auto`.
    pub in_auto: bool,
    /// AUTO current nav cmd is `MAV_CMD_NAV_LOITER_TURNS`.
    pub auto_loiter_turns: bool,
}

impl GuidedModeView {
    /// GUIDED mode.
    #[must_use]
    pub const fn guided() -> Self {
        Self {
            in_guided: true,
            in_auto: false,
            auto_loiter_turns: false,
        }
    }

    /// AUTO, not loiter-turns.
    #[must_use]
    pub const fn auto() -> Self {
        Self {
            in_guided: false,
            in_auto: true,
            auto_loiter_turns: false,
        }
    }

    /// AUTO `NAV_LOITER_TURNS` — fixed-wing only.
    #[must_use]
    pub const fn auto_loiter_turns() -> Self {
        Self {
            in_guided: false,
            in_auto: true,
            auto_loiter_turns: true,
        }
    }
}

impl QuadPlane {
    /// Current `Q_RTL_MODE` value.
    #[must_use]
    pub const fn rtl_mode(&self) -> i8 {
        self.rtl_mode
    }

    /// Write `Q_RTL_MODE`.
    pub fn set_rtl_mode(&mut self, rtl_mode: i8) {
        self.rtl_mode = rtl_mode;
    }

    /// Inverse of the upstream `RTL_MODE` discriminant.
    #[must_use]
    pub const fn rtl_mode_enum(&self) -> Option<RtlMode> {
        RtlMode::from_i8(self.rtl_mode)
    }

    /// ModeRTL `_enter` switches to QRTL immediately.
    #[must_use]
    pub const fn rtl_qrtl_always(&self) -> bool {
        match self.rtl_mode_enum() {
            Some(mode) => rtl_mode_qrtl_always(mode),
            None => false,
        }
    }

    /// ModeRTL treats the return as a VTOL landing
    /// (`SWITCH_QRTL` or `VTOL_APPROACH_QRTL`).
    #[must_use]
    pub const fn rtl_vtol_landing(&self) -> bool {
        match self.rtl_mode_enum() {
            Some(mode) => rtl_mode_vtol_landing(mode),
            None => false,
        }
    }

    /// Current `Q_GUIDED_MODE` value.
    #[must_use]
    pub const fn guided_mode(&self) -> i8 {
        self.guided_mode
    }

    /// Write `Q_GUIDED_MODE`.
    pub fn set_guided_mode(&mut self, guided_mode: i8) {
        self.guided_mode = guided_mode;
    }

    /// `poscontrol.slow_descent` from leftover [`Self::guided_start`].
    #[must_use]
    pub const fn slow_descent(&self) -> bool {
        self.slow_descent
    }

    /// Upstream `QuadPlane::guided_mode_enabled`.
    ///
    /// Needs `available()`, GUIDED or AUTO (not `NAV_LOITER_TURNS`),
    /// and `Q_GUIDED_MODE != 0`.
    #[must_use]
    pub const fn guided_mode_enabled(&self, view: GuidedModeView) -> bool {
        guided_mode_is_enabled(
            self.available(),
            view.in_guided,
            view.in_auto,
            view.auto_loiter_turns,
            self.guided_mode,
        )
    }

    /// Upstream `QuadPlane::guided_start`.
    ///
    /// Clears `guided_takeoff`, records `setup_target_position`, inits
    /// the approach, and latches `slow_descent` from the altitude pair.
    pub fn guided_start(&mut self, view: GuidedStartView) -> GuidedStartResult {
        self.guided_takeoff = false;
        self.slow_descent = if view.abs_ok {
            guided_slow_descent(view.abs_from_alt_cm, view.abs_to_alt_cm)
        } else {
            guided_slow_descent(view.current_alt_cm, view.next_wp_alt_cm)
        };
        self.poscontrol_init_approach(view.approach);
        GuidedStartResult {
            setup_target: true,
            approach_inited: true,
            slow_descent: self.slow_descent,
        }
    }

    /// Upstream `QuadPlane::guided_update`.
    ///
    /// GUIDED takeoff still below the target clears `throttle_wait` and
    /// asks for unlimited spool + `takeoff_controller`. Otherwise a
    /// just-finished takeoff jumps to `QPOS_POSITION2` and the tick
    /// runs `vtol_position_controller`.
    pub fn guided_update(&mut self, view: GuidedUpdateView) -> GuidedUpdateResult {
        if guided_update_climbing(
            view.in_guided,
            self.guided_takeoff,
            view.current_alt_cm,
            view.next_wp_alt_cm,
        ) {
            self.throttle_wait = false;
            return GuidedUpdateResult {
                action: GuidedUpdateAction::TakeoffClimb,
                throttle_wait: false,
                spool_unlimited: true,
                entered_position2: false,
            };
        }
        let entered_position2 = self.guided_takeoff;
        if self.guided_takeoff {
            self.poscontrol.set_state(PositionControlState::Position2);
        }
        self.guided_takeoff = false;
        GuidedUpdateResult {
            action: GuidedUpdateAction::PositionHold,
            throttle_wait: self.throttle_wait,
            spool_unlimited: false,
            entered_position2,
        }
    }
}
