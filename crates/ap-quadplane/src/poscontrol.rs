//! Control-mode / poscontrol init stub, upstream `QuadPlane::setup`
//! attitude_control / pos_control allocation and `QuadPlane::mode_enter`
//! (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. `setup()` constructs `AC_AttitudeControl_TS`
//! and `AC_PosControl` after motors; those controllers live in the COP
//! crates (`ap-control`). This module only records that they were
//! allocated, and resets the QuadPlane-side poscontrol / lean-angle
//! state when Plane enters a new mode (including a Q* mode).
//!
//! [`QuadPlane::init_throttle_wait`] is the QHover / QLoiter `_enter`
//! hook. QStabilize / QAcro / QLand force `throttle_wait = false`.

use crate::QuadPlane;

/// Upstream `QuadPlane::position_control_state`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PositionControlState {
    /// `QPOS_NONE` — `mode_enter` always returns here.
    None = 0,
    /// `QPOS_APPROACH`.
    Approach = 1,
    /// `QPOS_AIRBRAKE`.
    Airbrake = 2,
    /// `QPOS_POSITION1`.
    Position1 = 3,
    /// `QPOS_POSITION2`.
    Position2 = 4,
    /// `QPOS_LAND_DESCEND`.
    LandDescend = 5,
    /// `QPOS_LAND_ABORT`.
    LandAbort = 6,
    /// `QPOS_LAND_FINAL`.
    LandFinal = 7,
    /// `QPOS_LAND_COMPLETE`.
    LandComplete = 8,
}

/// Upstream `QuadPlane::PosControlState poscontrol`.
///
/// Only the fields `mode_enter` zeros / clears. Approach / land
/// transitions are a later slice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PosControlState {
    state: PositionControlState,
    correction_north_m: f32,
    correction_east_m: f32,
    velocity_match_north_ms: f32,
    velocity_match_east_ms: f32,
    last_velocity_match_ms: u32,
    pilot_correction_done: bool,
    pilot_correction_active: bool,
    target_vel_north_ms: f32,
    target_vel_east_ms: f32,
    target_vel_down_ms: f32,
}

impl Default for PosControlState {
    fn default() -> Self {
        Self::new()
    }
}

impl PosControlState {
    /// Empty poscontrol block, `QPOS_NONE`, all corrections zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: PositionControlState::None,
            correction_north_m: 0.0,
            correction_east_m: 0.0,
            velocity_match_north_ms: 0.0,
            velocity_match_east_ms: 0.0,
            last_velocity_match_ms: 0,
            pilot_correction_done: false,
            pilot_correction_active: false,
            target_vel_north_ms: 0.0,
            target_vel_east_ms: 0.0,
            target_vel_down_ms: 0.0,
        }
    }

    /// Current `poscontrol.get_state()`.
    #[must_use]
    pub const fn state(&self) -> PositionControlState {
        self.state
    }

    /// Write `poscontrol.set_state` without the transition side-effects.
    ///
    /// Upstream `set_state` resets yaw / integrators on some targets;
    /// those COP calls are a later slice.
    pub fn set_state(&mut self, state: PositionControlState) {
        self.state = state;
    }

    /// `correction_ne_m` north (metres).
    #[must_use]
    pub const fn correction_north_m(&self) -> f32 {
        self.correction_north_m
    }

    /// `correction_ne_m` east (metres).
    #[must_use]
    pub const fn correction_east_m(&self) -> f32 {
        self.correction_east_m
    }

    /// Write a NE correction so tests can prove `mode_enter` zeros it.
    pub fn set_correction_ne_m(&mut self, north_m: f32, east_m: f32) {
        self.correction_north_m = north_m;
        self.correction_east_m = east_m;
    }

    /// `velocity_match_ms` north (m/s).
    #[must_use]
    pub const fn velocity_match_north_ms(&self) -> f32 {
        self.velocity_match_north_ms
    }

    /// `velocity_match_ms` east (m/s).
    #[must_use]
    pub const fn velocity_match_east_ms(&self) -> f32 {
        self.velocity_match_east_ms
    }

    /// `last_velocity_match_ms`.
    #[must_use]
    pub const fn last_velocity_match_ms(&self) -> u32 {
        self.last_velocity_match_ms
    }

    /// Write velocity-match so tests can prove `mode_enter` zeros it.
    pub fn set_velocity_match_ms(&mut self, north_ms: f32, east_ms: f32, last_ms: u32) {
        self.velocity_match_north_ms = north_ms;
        self.velocity_match_east_ms = east_ms;
        self.last_velocity_match_ms = last_ms;
    }

    /// `pilot_correction_done`.
    #[must_use]
    pub const fn pilot_correction_done(&self) -> bool {
        self.pilot_correction_done
    }

    /// `pilot_correction_active`.
    #[must_use]
    pub const fn pilot_correction_active(&self) -> bool {
        self.pilot_correction_active
    }

    /// Latch a pilot correction so tests can prove `mode_enter` clears it.
    pub fn set_pilot_correction(&mut self, done: bool, active: bool) {
        self.pilot_correction_done = done;
        self.pilot_correction_active = active;
    }

    /// `target_vel_ms` north (m/s).
    #[must_use]
    pub const fn target_vel_north_ms(&self) -> f32 {
        self.target_vel_north_ms
    }

    /// `target_vel_ms` east (m/s).
    #[must_use]
    pub const fn target_vel_east_ms(&self) -> f32 {
        self.target_vel_east_ms
    }

    /// `target_vel_ms` down (m/s).
    #[must_use]
    pub const fn target_vel_down_ms(&self) -> f32 {
        self.target_vel_down_ms
    }

    /// Write `target_vel_ms` so tests can prove `mode_enter` zeros it.
    pub fn set_target_vel_ms(&mut self, north_ms: f32, east_ms: f32, down_ms: f32) {
        self.target_vel_north_ms = north_ms;
        self.target_vel_east_ms = east_ms;
        self.target_vel_down_ms = down_ms;
    }

    /// True when every `mode_enter` reset field is at its cleared value.
    #[must_use]
    pub const fn mode_enter_cleared(&self) -> bool {
        matches!(self.state, PositionControlState::None)
            && self.correction_north_m == 0.0
            && self.correction_east_m == 0.0
            && self.velocity_match_north_ms == 0.0
            && self.velocity_match_east_ms == 0.0
            && self.last_velocity_match_ms == 0
            && !self.pilot_correction_done
            && !self.pilot_correction_active
            && self.target_vel_north_ms == 0.0
            && self.target_vel_east_ms == 0.0
            && self.target_vel_down_ms == 0.0
    }

    /// Upstream `mode_enter` poscontrol resets (always, even if unavailable).
    pub fn reset_on_mode_enter(&mut self) {
        self.correction_north_m = 0.0;
        self.correction_east_m = 0.0;
        self.velocity_match_north_ms = 0.0;
        self.velocity_match_east_ms = 0.0;
        self.last_velocity_match_ms = 0;
        self.state = PositionControlState::None;
        self.pilot_correction_done = false;
        self.pilot_correction_active = false;
        self.target_vel_north_ms = 0.0;
        self.target_vel_east_ms = 0.0;
        self.target_vel_down_ms = 0.0;
    }
}

/// Throttle threshold for [`QuadPlane::init_throttle_wait`].
///
/// Upstream `get_throttle_input() >= 10`.
pub const THROTTLE_WAIT_INPUT_MIN: i16 = 10;

impl QuadPlane {
    /// Whether [`Self::setup`] constructed the attitude-control object.
    ///
    /// Upstream `attitude_control != nullptr` after a successful setup.
    /// The controller itself is COP `AC_AttitudeControl`.
    #[must_use]
    pub const fn attitude_control_inited(&self) -> bool {
        self.attitude_control_inited
    }

    /// Whether [`Self::setup`] constructed the position-control object.
    ///
    /// Upstream `pos_control != nullptr` after a successful setup.
    /// The controller itself is COP `AC_PosControl`.
    #[must_use]
    pub const fn pos_control_inited(&self) -> bool {
        self.pos_control_inited
    }

    /// Last `pos_control->set_lean_angle_max_cd` value this stub recorded.
    #[must_use]
    pub const fn lean_angle_max_cd(&self) -> i32 {
        self.lean_angle_max_cd
    }

    /// Stub of `pos_control->set_lean_angle_max_cd`.
    ///
    /// COP owns the real limiter; QuadPlane records the command so
    /// [`Self::mode_enter`] can reset it to 0 when available.
    pub fn set_lean_angle_max_cd(&mut self, lean_angle_max_cd: i32) {
        self.lean_angle_max_cd = lean_angle_max_cd;
    }

    /// Upstream `poscontrol` block.
    #[must_use]
    pub const fn poscontrol(&self) -> &PosControlState {
        &self.poscontrol
    }

    /// Mutable `poscontrol` (tests / later slices dirty then reset).
    pub fn poscontrol_mut(&mut self) -> &mut PosControlState {
        &mut self.poscontrol
    }

    /// Upstream `bool throttle_wait`.
    #[must_use]
    pub const fn throttle_wait(&self) -> bool {
        self.throttle_wait
    }

    /// Write `throttle_wait` (QStabilize / QAcro / QLand force false).
    pub fn set_throttle_wait(&mut self, throttle_wait: bool) {
        self.throttle_wait = throttle_wait;
    }

    /// Upstream `bool guided_wait_takeoff`.
    #[must_use]
    pub const fn guided_wait_takeoff(&self) -> bool {
        self.guided_wait_takeoff
    }

    /// Latch a guided takeoff wait (GUIDED VTOL takeoff sequence).
    pub fn set_guided_wait_takeoff(&mut self, guided_wait_takeoff: bool) {
        self.guided_wait_takeoff = guided_wait_takeoff;
    }

    /// Upstream `bool guided_wait_takeoff_on_mode_enter`.
    ///
    /// `mode_enter` copies `guided_wait_takeoff` here, then clears the
    /// live flag. QRTL uses the copy to divert to QLand.
    #[must_use]
    pub const fn guided_wait_takeoff_on_mode_enter(&self) -> bool {
        self.guided_wait_takeoff_on_mode_enter
    }

    /// Upstream `QuadPlane::init_throttle_wait`.
    ///
    /// QHover / QLoiter `_enter` call this. Stick throttle at or above
    /// [`THROTTLE_WAIT_INPUT_MIN`], or an already-flying airframe,
    /// clears the wait; otherwise the motors stay at ground idle.
    pub fn init_throttle_wait(&mut self, throttle_input: i16, is_flying: bool) {
        self.throttle_wait = !(throttle_input >= THROTTLE_WAIT_INPUT_MIN || is_flying);
    }

    /// Upstream `QuadPlane::mode_enter`.
    ///
    /// Plane calls this on every mode change (not only Q* modes),
    /// before the new mode's `_enter`. When [`Self::available`], resets
    /// `pos_control->set_lean_angle_max_cd(0)`. Always zeros the
    /// poscontrol corrections / velocity-match / pilot offsets and
    /// returns the state to [`PositionControlState::None`]. Guided
    /// takeoff wait is snapshotted then cleared.
    pub fn mode_enter(&mut self) {
        if self.available() {
            self.lean_angle_max_cd = 0;
        }
        self.poscontrol.reset_on_mode_enter();
        self.guided_wait_takeoff_on_mode_enter = self.guided_wait_takeoff;
        self.guided_wait_takeoff = false;
    }
}
