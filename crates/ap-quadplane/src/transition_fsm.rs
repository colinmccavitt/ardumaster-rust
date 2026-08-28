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
//! transition, and `DONE` is FW.
//!
//! Forward / assist-back timing lives on [`SltTransition::update_forward_timing`].
//! `AIRSPEED_WAIT` lasts until airspeed is above `AIRSPEED_MIN` without
//! assist — not `Q_TRANSITION_MS`. [`Q_TRANSITION_MS_DEFAULT`] is the
//! post-airspeed `TIMER` dwell (`constrain_float(..., 500, 30000)`).
//! Assist re-trigger returns the FSM to `AIRSPEED_WAIT`.
//! [`Q_TRANS_DECEL_DEFAULT`] is the FW → VTOL stopping deceleration
//! (`v² / (2a)`).
//!
//! This slice is transition completion / fail / Q* fallback.
//! [`Q_TRANS_FAIL_DEFAULT`] (`0`) is no time limit. When
//! [`SltTransition::apply_transition_fail`] sees
//! `now - transition_start_ms > Q_TRANS_FAIL * 1000` during
//! `AIRSPEED_WAIT`, it requests [`TransFailOutcome::FallbackQLand`]
//! (default `Q_TRANS_FAIL_ACT` 0), [`TransFailOutcome::FallbackQRtl`],
//! or [`TransFailOutcome::WarnOnly`] (`-1`). `Q_OPTIONS` bit 19
//! ([`Q_OPTIONS_TRANS_FAIL_TO_FW`]) plus tiltrotor ground-speed
//! completes into `TIMER` with `in_forced_transition`. QLAND / QRTL
//! are later tickets; this stub records the mode token
//! ([`MODE_QLAND`] / [`MODE_QRTL`]; QStabilize is [`MODE_QSTABILIZE`]).
//! Leftover `transition.h` attitude helpers are listed in
//! [`TRANSITION_H_LEFTOVER`].

use crate::air_mode::MavVtolState;
use crate::QuadPlane;

/// Default `Q_TRANSITION_MS`, upstream
/// `AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000)`.
pub const Q_TRANSITION_MS_DEFAULT: i16 = 5000;

/// Upstream `constrain_float(quadplane.transition_time_ms, 500, 30000)` floor.
pub const Q_TRANSITION_MS_MIN: i16 = 500;

/// Upstream `constrain_float(quadplane.transition_time_ms, 500, 30000)` ceiling.
pub const Q_TRANSITION_MS_MAX: i16 = 30000;

/// Default `Q_TRANS_DECEL`, upstream
/// `AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel_mss, 2.0)`.
pub const Q_TRANS_DECEL_DEFAULT: f32 = 2.0;

/// Default `Q_TRANS_FAIL` (seconds). `0` is no limit.
///
/// Upstream `AP_GROUPINFO("TRANS_FAIL", 8, QuadPlane, transition_failure.timeout, 0)`.
pub const Q_TRANS_FAIL_DEFAULT: i16 = 0;

/// Default `Q_TRANS_FAIL_ACT`. `0` is QLand.
///
/// Upstream `AP_GROUPINFO("TRANS_FAIL_ACT", 29, QuadPlane, transition_failure.action, 0)`.
pub const Q_TRANS_FAIL_ACT_DEFAULT: i16 = 0;

/// `Q_OPTIONS` bit 19, upstream `QuadPlane::Option::TRANS_FAIL_TO_FW`.
///
/// Completes the forward transition instead of the Q* fallback when
/// the fail timer elapses and the tiltrotor has ground speed.
pub const Q_OPTIONS_TRANS_FAIL_TO_FW: i32 = 1 << 19;

/// `Mode::Number::QSTABILIZE` — first Q* mode (VT-004).
pub const MODE_QSTABILIZE: u8 = 17;

/// `Mode::Number::QLAND` — default `Q_TRANS_FAIL_ACT` fallback (VT-005).
pub const MODE_QLAND: u8 = 20;

/// `Mode::Number::QRTL` — `Q_TRANS_FAIL_ACT` 1 (VT-006).
pub const MODE_QRTL: u8 = 21;

/// `ModeReason::VTOL_FAILED_TRANSITION`.
pub const MODE_REASON_VTOL_FAILED_TRANSITION: u8 = 23;

/// Leftover `transition.h` virtuals outside the SLT FSM (base-class
/// defaults or attitude / mix helpers). Not stubbed here.
pub const TRANSITION_H_LEFTOVER: &[&str] = &[
    "show_vtol_view",
    "set_FW_roll_pitch",
    "set_FW_roll_limit",
    "allow_update_throttle_mix",
    "update_yaw_target",
    "set_VTOL_roll_pitch_limit",
    "allow_weathervane",
    "allow_vfwd",
    "set_last_fw_pitch",
    "allow_stick_mixing",
    "use_multirotor_control_in_fwd_transition",
];

/// `Q_TRANS_FAIL_ACT`, upstream `QuadPlane::TRANS_FAIL::ACTION`.
///
/// Param values: `-1` warn only, `0` QLand, `1` QRTL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i16)]
pub enum TransFailAction {
    /// `-1` — GCS warning only; stay in `AIRSPEED_WAIT`.
    WarnOnly = -1,
    /// `0` — `plane.set_mode(mode_qland, VTOL_FAILED_TRANSITION)`.
    QLand = 0,
    /// `1` — `mode_qrtl` plus `poscontrol` `QPOS_POSITION1`.
    QRtl = 1,
}

impl TransFailAction {
    /// Decode the stored `Q_TRANS_FAIL_ACT` parameter.
    ///
    /// Unknown values take the upstream `default:` path (warn only).
    #[must_use]
    pub const fn from_param(v: i16) -> Self {
        match v {
            0 => Self::QLand,
            1 => Self::QRtl,
            _ => Self::WarnOnly,
        }
    }

    /// Stored parameter value.
    #[must_use]
    pub const fn as_i16(self) -> i16 {
        self as i16
    }
}

/// Outcome of one `Q_TRANS_FAIL` check during `AIRSPEED_WAIT`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransFailOutcome {
    /// Still inside the timeout, or `Q_TRANS_FAIL == 0`.
    Continue,
    /// Timeout; warned; stay in `AIRSPEED_WAIT` (`Q_TRANS_FAIL_ACT` `-1`).
    WarnOnly,
    /// Timeout; request QLand (`ModeReason::VTOL_FAILED_TRANSITION`).
    FallbackQLand,
    /// Timeout; request QRTL + `QPOS_POSITION1`.
    FallbackQRtl,
    /// Bit 19 + tiltrotor ground-speed: enter `TIMER` forced-complete.
    CompleteToFw,
}

impl TransFailOutcome {
    /// `Mode::Number` the fail path asks Plane to enter.
    ///
    /// QStabilize ([`MODE_QSTABILIZE`]) is the first Q* mode; the
    /// fail actions themselves are QLand / QRTL.
    #[must_use]
    pub const fn fallback_mode_number(self) -> Option<u8> {
        match self {
            Self::FallbackQLand => Some(MODE_QLAND),
            Self::FallbackQRtl => Some(MODE_QRTL),
            _ => None,
        }
    }

    /// True when Plane should leave the forward-transition mode.
    #[must_use]
    pub const fn requests_q_fallback(self) -> bool {
        matches!(self, Self::FallbackQLand | Self::FallbackQRtl)
    }
}

/// `option_is_set(TRANS_FAIL_TO_FW)`.
#[must_use]
pub const fn trans_fail_to_fw_set(q_options: i32) -> bool {
    (q_options & Q_OPTIONS_TRANS_FAIL_TO_FW) != 0
}

/// Constrain `Q_TRANSITION_MS` to the TIMER dwell range.
///
/// Upstream `const float trans_time_ms = constrain_float(quadplane.transition_time_ms, 500, 30000)`.
#[must_use]
pub const fn constrain_transition_time_ms(ms: i16) -> u32 {
    let v = if ms < Q_TRANSITION_MS_MIN {
        Q_TRANSITION_MS_MIN
    } else if ms > Q_TRANSITION_MS_MAX {
        Q_TRANSITION_MS_MAX
    } else {
        ms
    };
    v as u32
}

/// FW → VTOL stopping distance, upstream `QuadPlane::stopping_distance_m`.
///
/// `ground_speed_squared_m / (2 * Q_TRANS_DECEL)`.
#[must_use]
pub fn stopping_distance_m(ground_speed_squared_m: f32, transition_decel_mss: f32) -> f32 {
    ground_speed_squared_m / (2.0 * transition_decel_mss)
}

/// Seconds to shed `ground_speed_ms` at `Q_TRANS_DECEL` (`t = v / a`).
///
/// The time that belongs to [`stopping_distance_m`]: `v²/(2a)` over
/// constant decel.
#[must_use]
pub fn back_transition_time_s(ground_speed_ms: f32, transition_decel_mss: f32) -> f32 {
    ground_speed_ms / transition_decel_mss
}

/// `SLT_Transition::State`, stored as `transition_state`.
///
/// Sequential values: `AIRSPEED_WAIT = 0`, `TIMER = 1`, `DONE = 2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TransitionState {
    /// Waiting for airspeed (or the no-sensor fallback) before the timer.
    AirspeedWait = 0,
    /// Post-airspeed dwell of constrained `Q_TRANSITION_MS`.
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
    /// `Q_TRANSITION_MS`, upstream `QuadPlane::transition_time_ms`.
    transition_time_ms: i16,
    /// `Q_TRANS_DECEL`, upstream `QuadPlane::transition_decel_mss`.
    transition_decel_mss: f32,
    /// `Q_TRANS_FAIL`, upstream `transition_failure.timeout` (seconds).
    transition_fail_timeout_s: i16,
    /// `Q_TRANS_FAIL_ACT`, upstream `transition_failure.action`.
    transition_fail_action: i16,
    /// Upstream `transition_failure.warned`.
    transition_fail_warned: bool,
    /// Live `Q_OPTIONS` (bit 19 is [`Q_OPTIONS_TRANS_FAIL_TO_FW`]).
    q_options: i32,
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
            transition_time_ms: Q_TRANSITION_MS_DEFAULT,
            transition_decel_mss: Q_TRANS_DECEL_DEFAULT,
            transition_fail_timeout_s: Q_TRANS_FAIL_DEFAULT,
            transition_fail_action: Q_TRANS_FAIL_ACT_DEFAULT,
            transition_fail_warned: false,
            q_options: 0,
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

    /// `Q_TRANSITION_MS` as stored (before the 500..30000 constrain).
    #[must_use]
    pub const fn transition_time_ms(&self) -> i16 {
        self.transition_time_ms
    }

    /// Constrained TIMER dwell, upstream `trans_time_ms`.
    #[must_use]
    pub const fn timer_duration_ms(&self) -> u32 {
        constrain_transition_time_ms(self.transition_time_ms)
    }

    /// Write `Q_TRANSITION_MS`.
    pub fn set_transition_time_ms(&mut self, ms: i16) {
        self.transition_time_ms = ms;
    }

    /// `Q_TRANS_DECEL` as stored.
    #[must_use]
    pub const fn transition_decel_mss(&self) -> f32 {
        self.transition_decel_mss
    }

    /// Write `Q_TRANS_DECEL`.
    pub fn set_transition_decel_mss(&mut self, decel_mss: f32) {
        self.transition_decel_mss = decel_mss;
    }

    /// `Q_TRANS_FAIL` seconds (`0` = no limit).
    #[must_use]
    pub const fn transition_fail_timeout_s(&self) -> i16 {
        self.transition_fail_timeout_s
    }

    /// Write `Q_TRANS_FAIL`.
    pub fn set_transition_fail_timeout_s(&mut self, timeout_s: i16) {
        self.transition_fail_timeout_s = timeout_s;
    }

    /// Decoded `Q_TRANS_FAIL_ACT`.
    #[must_use]
    pub const fn transition_fail_action(&self) -> TransFailAction {
        TransFailAction::from_param(self.transition_fail_action)
    }

    /// Write `Q_TRANS_FAIL_ACT`.
    pub fn set_transition_fail_action(&mut self, action: TransFailAction) {
        self.transition_fail_action = action.as_i16();
    }

    /// Upstream `transition_failure.warned`.
    #[must_use]
    pub const fn transition_fail_warned(&self) -> bool {
        self.transition_fail_warned
    }

    /// Live `Q_OPTIONS` stored on this FSM.
    #[must_use]
    pub const fn q_options(&self) -> i32 {
        self.q_options
    }

    /// Write `Q_OPTIONS` (bit 19 selects complete-to-FW on fail).
    pub fn set_q_options(&mut self, q_options: i32) {
        self.q_options = q_options;
    }

    /// `option_is_set(TRANS_FAIL_TO_FW)` for the stored `Q_OPTIONS`.
    #[must_use]
    pub const fn trans_fail_to_fw(&self) -> bool {
        trans_fail_to_fw_set(self.q_options)
    }

    /// Reset the fail timer while disarmed.
    ///
    /// Upstream `SLT_Transition::update`: when not
    /// `arming.is_armed_and_safety_off()`, `transition_start_ms = now`.
    pub fn reset_fail_timer_if_disarmed(&mut self, now_ms: u32, armed_and_safety_off: bool) {
        if !armed_and_safety_off {
            self.transition_start_ms = now_ms;
        }
    }

    /// `Q_TRANS_FAIL` check during `AIRSPEED_WAIT`.
    ///
    /// Upstream `SLT_Transition::update` `AIRSPEED_WAIT` case: when
    /// `transition_start_ms != 0`, `timeout > 0`, and
    /// `now - start > timeout * 1000`, warn once then either force
    /// `TIMER` (`TRANS_FAIL_TO_FW` and tiltrotor ground-speed) or
    /// request the `Q_TRANS_FAIL_ACT` Q* fallback. Otherwise clear
    /// `warned`. Not consulted in `TIMER` / `DONE`.
    pub fn apply_transition_fail(
        &mut self,
        now_ms: u32,
        tiltrotor_with_ground_speed: bool,
    ) -> TransFailOutcome {
        if !matches!(self.transition_state, TransitionState::AirspeedWait) {
            return TransFailOutcome::Continue;
        }
        let timeout_s = self.transition_fail_timeout_s;
        let timed_out = self.transition_start_ms != 0
            && timeout_s > 0
            && now_ms.wrapping_sub(self.transition_start_ms)
                > (timeout_s as u32).saturating_mul(1000);
        if !timed_out {
            self.transition_fail_warned = false;
            return TransFailOutcome::Continue;
        }
        if !self.transition_fail_warned {
            self.transition_fail_warned = true;
        }
        if self.trans_fail_to_fw() && tiltrotor_with_ground_speed {
            self.transition_state = TransitionState::Timer;
            self.in_forced_transition = true;
            return TransFailOutcome::CompleteToFw;
        }
        match self.transition_fail_action() {
            TransFailAction::QLand => TransFailOutcome::FallbackQLand,
            TransFailAction::QRtl => TransFailOutcome::FallbackQRtl,
            TransFailAction::WarnOnly => TransFailOutcome::WarnOnly,
        }
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
        self.transition_fail_warned = false;
    }

    /// Enter `TIMER` after the airspeed-wait stage.
    ///
    /// Upstream `SLT_Transition::update` assigns this when airspeed
    /// (or the no-sensor fallback) is reached. Prefer
    /// [`Self::update_airspeed_wait`] so `transition_low_airspeed_ms`
    /// stamps the start of the `Q_TRANSITION_MS` dwell.
    pub fn enter_timer(&mut self) {
        self.transition_state = TransitionState::Timer;
    }

    /// Stamp AIRSPEED_WAIT and enter TIMER when airspeed is reached.
    ///
    /// Upstream `SLT_Transition::update` `AIRSPEED_WAIT` case:
    /// `transition_start_ms` is set on first entry,
    /// `transition_low_airspeed_ms` is refreshed every tick, and TIMER
    /// starts when `have_airspeed && aspeed > airspeed_min &&
    /// !assisted_flight`. `Q_TRANSITION_MS` does not bound this wait.
    pub fn update_airspeed_wait(
        &mut self,
        now_ms: u32,
        have_airspeed: bool,
        aspeed: f32,
        airspeed_min: f32,
        assisted_flight: bool,
    ) {
        if self.transition_start_ms == 0 {
            self.transition_start_ms = now_ms;
        }
        self.transition_low_airspeed_ms = now_ms;
        if have_airspeed && aspeed > airspeed_min && !assisted_flight {
            self.transition_state = TransitionState::Timer;
        }
    }

    /// Complete TIMER after the constrained `Q_TRANSITION_MS` dwell.
    ///
    /// Upstream: `transition_timer_ms = now - transition_low_airspeed_ms`
    /// and `transition_timer_ms > unsigned(trans_time_ms) && tilt_fwd_complete`.
    pub fn update_timer(&mut self, now_ms: u32, tilt_fwd_complete: bool) {
        let trans_time_ms = self.timer_duration_ms();
        let transition_timer_ms = now_ms.wrapping_sub(self.transition_low_airspeed_ms);
        if transition_timer_ms > trans_time_ms && tilt_fwd_complete {
            self.force_transition_complete();
        }
    }

    /// Assist re-trigger during a forward transition: back to AIRSPEED_WAIT.
    ///
    /// Upstream `update` assist block: when `should_assist` and not
    /// `in_forced_transition`, `transition_state = AIRSPEED_WAIT` and
    /// `transition_start_ms` is set if it was zero.
    pub fn apply_assist_back(&mut self, now_ms: u32, should_assist: bool) {
        if should_assist && !self.in_forced_transition {
            self.transition_state = TransitionState::AirspeedWait;
            if self.transition_start_ms == 0 {
                self.transition_start_ms = now_ms;
            }
        }
    }

    /// One forward-transition timing tick (assist-back, then AIRSPEED_WAIT / TIMER).
    ///
    /// Mirrors the order in `SLT_Transition::update`: assist can throw
    /// the FSM back to `AIRSPEED_WAIT` before the stage switch runs.
    pub fn update_forward_timing(
        &mut self,
        now_ms: u32,
        have_airspeed: bool,
        aspeed: f32,
        airspeed_min: f32,
        should_assist: bool,
        tilt_fwd_complete: bool,
    ) {
        self.apply_assist_back(now_ms, should_assist);
        match self.transition_state {
            TransitionState::AirspeedWait => {
                self.update_airspeed_wait(
                    now_ms,
                    have_airspeed,
                    aspeed,
                    airspeed_min,
                    should_assist,
                );
            }
            TransitionState::Timer => {
                self.update_timer(now_ms, tilt_fwd_complete);
            }
            TransitionState::Done => {}
        }
    }

    /// FW → VTOL stopping distance at this `Q_TRANS_DECEL`.
    #[must_use]
    pub fn stopping_distance_m(&self, ground_speed_squared_m: f32) -> f32 {
        stopping_distance_m(ground_speed_squared_m, self.transition_decel_mss)
    }

    /// Seconds to stop from `ground_speed_ms` at this `Q_TRANS_DECEL`.
    #[must_use]
    pub fn back_transition_time_s(&self, ground_speed_ms: f32) -> f32 {
        back_transition_time_s(ground_speed_ms, self.transition_decel_mss)
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
