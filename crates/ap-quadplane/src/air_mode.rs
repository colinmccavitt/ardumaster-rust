//! Air-mode and QuadPlane-side transition hooks, upstream
//! `QuadPlane::air_mode_active` / `QuadPlane::update` /
//! `QuadPlane::in_frwd_transition` / `QuadPlane::handle_do_vtol_transition`
//! (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. The `AirMode` latch (`OFF` / `ON` /
//! `ASSISTED_FLIGHT_ONLY`) plus `Q_OPTIONS` bit 9 (`AIRMODE_UNUSED`)
//! decide whether airmode is live. [`QuadPlane::update`] is the
//! QuadPlane-side entry into the transition FSM: it picks
//! `transition->update()`, `VTOL_update()`, or
//! `force_transition_complete()`. The FSM itself is **VT-003**.

use crate::QuadPlane;

/// Default `Q_OPTIONS`, upstream `AP_GROUPINFO("OPTIONS", ..., 0)`.
pub const Q_OPTIONS_DEFAULT: i32 = 0;

/// Upstream `QuadPlane::Option::AIRMODE_UNUSED` (`1<<9`).
///
/// The bit is unused at runtime. `system.cpp` still reads it once at
/// boot to convert `ARMDISARM_UNUSED` → `ARMDISARM_AIRMODE` when
/// QuadPlane is enabled and no dedicated AIRMODE aux channel exists.
pub const Q_OPTIONS_AIRMODE_UNUSED: i32 = 1 << 9;

/// `Q_OPTIONS` bits this slice cares about.
///
/// Named `QOption` so it does not collide with [`core::option::Option`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum QOption {
    /// Bit 9, upstream `QuadPlane::Option::AIRMODE_UNUSED`.
    AirmodeUnused = 1 << 9,
    /// Bit 13, upstream `QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP`.
    DisableGroundEffectComp = 1 << 13,
}

/// Air-mode latch, upstream `enum class AirMode` in `defines.h`.
///
/// Sequential values: `OFF = 0`, `ON = 1`, `ASSISTED_FLIGHT_ONLY = 2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AirMode {
    /// Motors idle at zero throttle in manual VTOL modes.
    Off = 0,
    /// Keep motors spinning at zero throttle (airmode on).
    On = 1,
    /// Airmode only while [`QuadPlane::assisted_flight`] is set.
    AssistedFlightOnly = 2,
}

impl Default for AirMode {
    fn default() -> Self {
        Self::Off
    }
}

/// `MAV_VTOL_STATE` values `handle_do_vtol_transition` switches on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavVtolState {
    /// `MAV_VTOL_STATE_UNDEFINED`.
    Undefined = 0,
    /// `MAV_VTOL_STATE_TRANSITION_TO_FW` — rejected (not MC/FW).
    TransitionToFw = 1,
    /// `MAV_VTOL_STATE_TRANSITION_TO_MC` — rejected (not MC/FW).
    TransitionToMc = 2,
    /// `MAV_VTOL_STATE_MC` — enter VTOL (`auto_state.vtol_mode = true`).
    Mc = 3,
    /// `MAV_VTOL_STATE_FW` — exit VTOL (`auto_state.vtol_mode = false`).
    Fw = 4,
}

/// Which `Transition` virtual [`QuadPlane::update`] would call.
///
/// The methods themselves live on the VT-003 FSM. This is only the
/// QuadPlane-side dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransitionHook {
    /// `setup()` failed — no FSM call.
    None,
    /// `transition->force_transition_complete()` (MANUAL / ACRO / TRAINING).
    ForceComplete,
    /// `transition->update()` (fixed-wing, not a stick-only mode).
    Update,
    /// `transition->VTOL_update()` (Q* / VTOL-auto / airbrake).
    VtolUpdate,
}

/// What [`QuadPlane::update`] reads from Plane to pick a transition hook.
///
/// Upstream reads `in_vtol_mode()`, `in_vtol_airbrake()`, and
/// `control_mode` (MANUAL / ACRO / TRAINING). This crate does not own
/// those objects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransitionUpdateView {
    /// `QuadPlane::in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `QuadPlane::in_vtol_airbrake()` — a later poscontrol slice.
    pub in_vtol_airbrake: bool,
    /// `control_mode` is MANUAL, ACRO, or TRAINING.
    pub fw_manual: bool,
}

impl Default for TransitionUpdateView {
    fn default() -> Self {
        Self::new()
    }
}

impl TransitionUpdateView {
    /// Fixed-wing, not a stick-only mode, not airbrake.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_vtol_mode: false,
            in_vtol_airbrake: false,
            fw_manual: false,
        }
    }

    /// MANUAL / ACRO / TRAINING — motors off, force transition complete.
    #[must_use]
    pub const fn fw_manual() -> Self {
        Self {
            in_vtol_mode: false,
            in_vtol_airbrake: false,
            fw_manual: true,
        }
    }

    /// A Q* / VTOL-auto mode (`in_vtol_mode`).
    #[must_use]
    pub const fn vtol() -> Self {
        Self {
            in_vtol_mode: true,
            in_vtol_airbrake: false,
            fw_manual: false,
        }
    }

    /// Airbrake only — still the VTOL `update` path, `assisted_flight` set.
    #[must_use]
    pub const fn vtol_airbrake() -> Self {
        Self {
            in_vtol_mode: false,
            in_vtol_airbrake: true,
            fw_manual: false,
        }
    }
}

impl QuadPlane {
    /// Current `Q_OPTIONS` bitmask.
    #[must_use]
    pub const fn options(&self) -> i32 {
        self.options
    }

    /// Write `Q_OPTIONS`.
    pub fn set_options(&mut self, options: i32) {
        self.options = options;
    }

    /// Upstream `QuadPlane::option_is_set`.
    #[must_use]
    pub const fn option_is_set(&self, option: QOption) -> bool {
        (self.options & (option as i32)) != 0
    }

    /// Current air-mode latch.
    #[must_use]
    pub const fn air_mode(&self) -> AirMode {
        self.air_mode
    }

    /// Write the air-mode latch (RC `AUX_FUNC::AIRMODE` / arm-disarm).
    pub fn set_air_mode(&mut self, air_mode: AirMode) {
        self.air_mode = air_mode;
    }

    /// Upstream `bool assisted_flight`.
    #[must_use]
    pub const fn assisted_flight(&self) -> bool {
        self.assisted_flight
    }

    /// Latch assist (the VT-003 FSM and VT-002 assist set this).
    pub fn set_assisted_flight(&mut self, assisted_flight: bool) {
        self.assisted_flight = assisted_flight;
    }

    /// Upstream `QuadPlane::in_assisted_flight` — `available() && assisted_flight`.
    #[must_use]
    pub const fn in_assisted_flight(&self) -> bool {
        self.available() && self.assisted_flight
    }

    /// Upstream `QuadPlane::air_mode_active`.
    ///
    /// True when the latch is `ON`, or `ASSISTED_FLIGHT_ONLY` while
    /// assist is running. Does not consult [`Self::available`].
    #[must_use]
    pub const fn air_mode_active(&self) -> bool {
        matches!(self.air_mode, AirMode::On)
            || (matches!(self.air_mode, AirMode::AssistedFlightOnly) && self.assisted_flight)
    }

    /// `system.cpp` RC conversion: `ARMDISARM_UNUSED` → `ARMDISARM_AIRMODE`.
    ///
    /// True when QuadPlane is enabled, `Q_OPTIONS` bit 9 is set, and no
    /// dedicated AIRMODE aux channel is configured.
    #[must_use]
    pub const fn armdisarm_converts_to_airmode(&self, airmode_aux_present: bool) -> bool {
        self.enabled() && self.option_is_set(QOption::AirmodeUnused) && !airmode_aux_present
    }

    /// Upstream `QuadPlane::update` — QuadPlane-side transition dispatch.
    ///
    /// When `setup()` fails, returns [`TransitionHook::None`]. In
    /// MANUAL / ACRO / TRAINING, clears assist and returns
    /// [`TransitionHook::ForceComplete`]. Other fixed-wing modes return
    /// [`TransitionHook::Update`]. VTOL / airbrake return
    /// [`TransitionHook::VtolUpdate`] and set `assisted_flight` from
    /// the airbrake flag.
    pub fn update(&mut self, view: &TransitionUpdateView) -> TransitionHook {
        if !self.setup() {
            return TransitionHook::None;
        }
        if !view.in_vtol_mode && !view.in_vtol_airbrake {
            if view.fw_manual {
                self.assisted_flight = false;
                return TransitionHook::ForceComplete;
            }
            return TransitionHook::Update;
        }
        self.assisted_flight = view.in_vtol_airbrake;
        TransitionHook::VtolUpdate
    }

    /// Upstream `QuadPlane::in_frwd_transition`.
    ///
    /// `available() && transition->active_frwd()`. `active_frwd` is a
    /// VT-003 FSM query, so the caller passes it.
    #[must_use]
    pub const fn in_frwd_transition(&self, active_frwd: bool) -> bool {
        self.available() && active_frwd
    }

    /// Upstream `QuadPlane::handle_do_vtol_transition`.
    ///
    /// Rejects when not [`Self::available`], not AUTO, or the state is
    /// not `MAV_VTOL_STATE_MC` / `MAV_VTOL_STATE_FW`. On accept returns
    /// the value to write to `auto_state.vtol_mode` (`true` for MC,
    /// `false` for FW). The C++ method returns `bool`; the `Option`
    /// carries that AUTO-flag write because this crate does not own
    /// `plane.auto_state`.
    #[must_use]
    pub const fn handle_do_vtol_transition(
        &self,
        state: MavVtolState,
        in_auto: bool,
    ) -> Option<bool> {
        if !self.available() || !in_auto {
            return None;
        }
        match state {
            MavVtolState::Mc => Some(true),
            MavVtolState::Fw => Some(false),
            MavVtolState::Undefined
            | MavVtolState::TransitionToFw
            | MavVtolState::TransitionToMc => None,
        }
    }
}
