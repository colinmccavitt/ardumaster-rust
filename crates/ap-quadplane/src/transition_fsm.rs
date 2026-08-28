//! QuadPlane SLT transition FSM stub, upstream `ArduPlane/transition.h`
//! `SLT_Transition` (Plane-4.7.0).
//!
//! Tracked as **VT-003**. This is the separate-lift-thrust state machine
//! (`AIRSPEED_WAIT` / `TIMER` / `DONE`) that `transition_state`,
//! `complete()`, `restart()`, and `QuadPlane::in_frwd_transition` /
//! `active_frwd()` read. The QuadPlane-side dispatch hooks stay in
//! [`crate::air_mode`]; the tailsitter pitch / throttle ramp stays in
//! [`crate::transition`].
//!
//! `get_mav_vtol_state` reports the high-level AIR / VTOL / TRANSITION
//! phase: VTOL mode is MC, `AIRSPEED_WAIT` / `TIMER` is a forward
//! transition, and `DONE` is FW. Forward-transition timing
//! (`Q_TRANSITION_MS`) and back-transition deceleration
//! (`Q_TRANS_DECEL`) are later slices.

use crate::air_mode::MavVtolState;
use crate::QuadPlane;

/// `SLT_Transition::State`, stored as `transition_state`.
///
/// Sequential values: `AIRSPEED_WAIT = 0`, `TIMER = 1`, `DONE = 2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TransitionState {
    /// Waiting for airspeed (or the no-sensor fallback) before the timer.
    AirspeedWait = 0,
    /// Post-airspeed dwell; `Q_TRANSITION_MS` lives on a later slice.
    Timer = 1,
    /// Forward transition finished (or forced complete / parked VTOL).
    Done = 2,
}

impl Default for TransitionState {
    fn default() -> Self {
        Self::AirspeedWait
    }
}

/// High-level AIR / VTOL / TRANSITION view of `get_mav_vtol_state`.
///
/// VTOL mode is [`Self::Vtol`]. `AIRSPEED_WAIT` / `TIMER` in a
/// fixed-wing mode is [`Self::Transition`]. `DONE` is [`Self::Air`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransitionPhase {
    /// Fixed-wing, `MAV_VTOL_STATE_FW`.
    Air,
    /// A Q* / VTOL-auto mode, `MAV_VTOL_STATE_MC`.
    Vtol,
    /// Forward transition, `MAV_VTOL_STATE_TRANSITION_TO_FW`.
    Transition,
}

/// Separate-lift-thrust transition object, upstream `SLT_Transition`.
#[derive(Clone, Copy, Debug)]
pub struct SltTransition {
    /// Upstream `SLT_Transition::transition_state`.
    transition_state: TransitionState,
    /// Upstream `bool in_forced_transition`.
    in_forced_transition: bool,
    /// Upstream `uint32_t transition_start_ms`.
    transition_start_ms: u32,
    /// Upstream `uint32_t transition_low_airspeed_ms`.
    transition_low_airspeed_ms: u32,
}

impl Default for SltTransition {
    fn default() -> Self {
        Self::new()
    }
}

impl SltTransition {
    /// Zero-init matches the first enumerator: `AIRSPEED_WAIT`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            transition_state: TransitionState::AirspeedWait,
            in_forced_transition: false,
            transition_start_ms: 0,
            transition_low_airspeed_ms: 0,
        }
    }

    /// Current `transition_state`.
    #[must_use]
    pub const fn transition_state(&self) -> TransitionState {
        self.transition_state
    }

    /// Upstream `SLT_Transition::complete` — `transition_state == DONE`.
    #[must_use]
    pub const fn complete(&self) -> bool {
        matches!(self.transition_state, TransitionState::Done)
    }

    /// `AIRSPEED_WAIT` or `TIMER` — the enum `QuadPlane::in_transition`
    /// / `transition_state` treats as an in-progress forward transition.
    #[must_use]
    pub const fn in_transition(&self) -> bool {
        matches!(
            self.transition_state,
            TransitionState::AirspeedWait | TransitionState::Timer
        )
    }

    /// Upstream `get_log_transition_state` — `static_cast<uint8_t>(transition_state)`.
    #[must_use]
    pub const fn get_log_transition_state(&self) -> u8 {
        self.transition_state as u8
    }

    /// Upstream `bool in_forced_transition`.
    #[must_use]
    pub const fn in_forced_transition(&self) -> bool {
        self.in_forced_transition
    }

    /// Upstream `transition_start_ms`.
    #[must_use]
    pub const fn transition_start_ms(&self) -> u32 {
        self.transition_start_ms
    }

    /// Upstream `transition_low_airspeed_ms`.
    #[must_use]
    pub const fn transition_low_airspeed_ms(&self) -> u32 {
        self.transition_low_airspeed_ms
    }

    /// Upstream `SLT_Transition::restart` — `transition_state = AIRSPEED_WAIT`.
    pub fn restart(&mut self) {
        self.transition_state = TransitionState::AirspeedWait;
    }

    /// Upstream `SLT_Transition::force_transition_complete`.
    ///
    /// Sets `DONE`, clears the forced-transition latch and both timers.
    /// Assist reset is a later slice (this crate does not own `VTOL_Assist`).
    pub fn force_transition_complete(&mut self) {
        self.transition_state = TransitionState::Done;
        self.in_forced_transition = false;
        self.transition_start_ms = 0;
        self.transition_low_airspeed_ms = 0;
    }

    /// Enter `TIMER` after the airspeed-wait stage.
    ///
    /// Upstream `SLT_Transition::update` assigns this when airspeed
    /// (or the no-sensor fallback) is reached. The `Q_TRANSITION_MS`
    /// dwell is a later slice.
    pub fn enter_timer(&mut self) {
        self.transition_state = TransitionState::Timer;
    }

    /// Upstream `SLT_Transition::VTOL_update` state reset.
    ///
    /// Clears the timers. Parked (`throttle_wait && !is_flying`) goes
    /// to `DONE`; otherwise the next FW mode starts at `AIRSPEED_WAIT`.
    pub fn vtol_update(&mut self, throttle_wait: bool, is_flying: bool) {
        self.transition_start_ms = 0;
        self.transition_low_airspeed_ms = 0;
        if throttle_wait && !is_flying {
            self.in_forced_transition = false;
            self.transition_state = TransitionState::Done;
        } else {
            self.transition_state = TransitionState::AirspeedWait;
        }
    }

    /// Upstream `SLT_Transition::active_frwd`.
    ///
    /// True when assist is on, the SLT state is `AIRSPEED_WAIT` or
    /// `TIMER`, and this is not a landing airbrake.
    #[must_use]
    pub const fn active_frwd(&self, assisted_flight: bool, in_vtol_airbrake: bool) -> bool {
        assisted_flight && self.in_transition() && !in_vtol_airbrake
    }

    /// High-level AIR / VTOL / TRANSITION phase.
    #[must_use]
    pub const fn phase(&self, in_vtol_mode: bool) -> TransitionPhase {
        if in_vtol_mode {
            TransitionPhase::Vtol
        } else if self.in_transition() {
            TransitionPhase::Transition
        } else {
            TransitionPhase::Air
        }
    }

    /// Upstream `SLT_Transition::get_mav_vtol_state` (land-approach later).
    ///
    /// VTOL mode → `MC`. `AIRSPEED_WAIT` / `TIMER` → `TRANSITION_TO_FW`.
    /// `DONE` → `FW`.
    #[must_use]
    pub const fn get_mav_vtol_state(&self, in_vtol_mode: bool) -> MavVtolState {
        if in_vtol_mode {
            MavVtolState::Mc
        } else {
            match self.transition_state {
                TransitionState::AirspeedWait | TransitionState::Timer => {
                    MavVtolState::TransitionToFw
                }
                TransitionState::Done => MavVtolState::Fw,
            }
        }
    }
}

impl QuadPlane {
    /// `available()` and the SLT FSM is not `DONE`.
    ///
    /// Upstream `in_frwd_transition` is `available() && active_frwd()`.
    /// This is the broader `transition_state` query the log / GCS
    /// treat as "in transition".
    #[must_use]
    pub const fn in_transition(&self, fsm: &SltTransition) -> bool {
        self.available() && fsm.in_transition()
    }
}
