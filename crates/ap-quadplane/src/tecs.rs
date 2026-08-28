//! Leftover TECS / stick-mix / stopping-distance stub, upstream
//! `QuadPlane::should_disable_TECS` / `allow_stick_mixing` /
//! `stopping_distance_m` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. Plane owns `control_mode`, `auto_state.vtol_loiter`,
//! AHRS groundspeed, and `transition->allow_stick_mixing()`; the caller
//! passes a [`TecsView`] / [`StickMixView`]. This is not a rewrite of
//! [`crate::land_sequence`] descent predicates, [`crate::tailsitter`]
//! stick-mix, [`crate::transition_fsm`] decel math, [`crate::guided`]
//! start / update, or [`crate::thrust_loss`].

use crate::land_sequence::LandSequenceView;
use crate::quadplane_completeness::{
    accel_needed, allow_stick_mixing, leftover_stopping_distance_m,
    leftover_transition_threshold_m, should_disable_tecs, TRANSITION_THRESHOLD_SCALE,
};
use crate::transition_fsm::Q_TRANS_DECEL_DEFAULT;
use crate::QuadPlane;

/// Inputs [`QuadPlane::should_disable_tecs`] reads from Plane.
///
/// This crate does not own `control_mode` or `auto_state.vtol_loiter`.
/// Land-descent uses [`LandSequenceView`] plus `poscontrol` state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TecsView {
    /// Mode / mission bits [`QuadPlane::in_vtol_land_descent`] reads.
    pub land: LandSequenceView,
    /// `plane.control_mode == &plane.mode_guided`.
    pub in_guided: bool,
    /// `plane.auto_state.vtol_loiter`.
    pub vtol_loiter: bool,
}

impl TecsView {
    /// Not GUIDED, not loitering, not a VTOL land command.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            land: LandSequenceView::new(),
            in_guided: false,
            vtol_loiter: false,
        }
    }

    /// GUIDED with `auto_state.vtol_loiter`.
    #[must_use]
    pub const fn guided_vtol_loiter() -> Self {
        Self {
            land: LandSequenceView::new(),
            in_guided: true,
            vtol_loiter: true,
        }
    }

    /// QRTL land-descent (poscontrol still has to be a descent state).
    #[must_use]
    pub const fn qrtl() -> Self {
        Self {
            land: LandSequenceView::qrtl(),
            in_guided: false,
            vtol_loiter: false,
        }
    }

    /// AUTO flying a VTOL land command.
    #[must_use]
    pub const fn auto_vtol_land() -> Self {
        Self {
            land: LandSequenceView::auto_vtol_land(),
            in_guided: false,
            vtol_loiter: false,
        }
    }
}

impl Default for TecsView {
    fn default() -> Self {
        Self::new()
    }
}

/// Inputs [`QuadPlane::allow_stick_mixing`] reads from the transition.
///
/// SLT's base `Transition::allow_stick_mixing` is always true.
/// Tailsitter override is [`crate::tailsitter::PitchLimit::allow_stick_mixing`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StickMixView {
    /// `transition->allow_stick_mixing()`.
    pub transition_allows: bool,
}

impl StickMixView {
    /// SLT / base transition — always allows stick mix.
    #[must_use]
    pub const fn slt() -> Self {
        Self {
            transition_allows: true,
        }
    }

    /// Tailsitter pitching up into VTOL or levelling off in FW.
    #[must_use]
    pub const fn tailsitter_blocked() -> Self {
        Self {
            transition_allows: false,
        }
    }
}

impl Default for StickMixView {
    fn default() -> Self {
        Self::slt()
    }
}

/// Last leftover TECS / decel values this stub records.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TecsState {
    /// `Q_TRANS_DECEL`, upstream `transition_decel_mss`.
    transition_decel_mss: f32,
}

impl TecsState {
    /// Default `Q_TRANS_DECEL` (2 m/s²).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            transition_decel_mss: Q_TRANS_DECEL_DEFAULT,
        }
    }

    /// Stored `Q_TRANS_DECEL`.
    #[must_use]
    pub const fn transition_decel_mss(&self) -> f32 {
        self.transition_decel_mss
    }
}

impl Default for TecsState {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadPlane {
    /// Leftover TECS / stick-mix / stopping-distance latches.
    #[must_use]
    pub const fn tecs(&self) -> &TecsState {
        &self.tecs
    }

    /// Write `Q_TRANS_DECEL`.
    pub fn set_transition_decel_mss(&mut self, decel_mss: f32) {
        self.tecs.transition_decel_mss = decel_mss;
    }

    /// Upstream `QuadPlane::should_disable_TECS`.
    ///
    /// True during VTOL land descent, or GUIDED with `vtol_loiter`, so
    /// TECS resets when height is commanded in a VTOL mode.
    #[must_use]
    pub const fn should_disable_tecs(&self, view: TecsView) -> bool {
        should_disable_tecs(
            self.in_vtol_land_descent(view.land),
            view.in_guided && view.vtol_loiter,
        )
    }

    /// Upstream `QuadPlane::allow_stick_mixing`.
    ///
    /// Unavailable QuadPlane always allows mix (fixed-wing path).
    /// Otherwise the transition object decides.
    #[must_use]
    pub const fn allow_stick_mixing(&self, view: StickMixView) -> bool {
        allow_stick_mixing(self.available(), view.transition_allows)
    }

    /// Upstream `QuadPlane::stopping_distance_m(ground_speed_squared_m)`.
    ///
    /// `v² / (2 * Q_TRANS_DECEL)`.
    #[must_use]
    pub fn stopping_distance_m(&self, ground_speed_squared_m: f32) -> f32 {
        leftover_stopping_distance_m(ground_speed_squared_m, self.tecs.transition_decel_mss)
    }

    /// Upstream `QuadPlane::stopping_distance_m(void)`.
    ///
    /// Plane passes `ahrs.groundspeed_vector().length_squared()`.
    #[must_use]
    pub fn stopping_distance_from_groundspeed(&self, groundspeed_ms: f32) -> f32 {
        self.stopping_distance_m(groundspeed_ms * groundspeed_ms)
    }

    /// Upstream `QuadPlane::accel_needed`.
    ///
    /// `v² / (2 * MAX(1, stop_distance))`.
    #[must_use]
    pub fn accel_needed(&self, stop_distance: f32, ground_speed_squared: f32) -> f32 {
        accel_needed(stop_distance, ground_speed_squared)
    }

    /// Upstream `QuadPlane::transition_threshold_m`.
    ///
    /// `1.5 * stopping_distance_m(sq(airspeed_cruise))`.
    #[must_use]
    pub fn transition_threshold_m(&self, airspeed_cruise_ms: f32) -> f32 {
        leftover_transition_threshold_m(
            airspeed_cruise_ms,
            self.tecs.transition_decel_mss,
            TRANSITION_THRESHOLD_SCALE,
        )
    }
}
