//! The state a mode change clears, upstream `ArduPlane/mode.cpp:27`,
//! `Mode::enter`.
//!
//! # A reset list is not interesting, and that is exactly the risk
//!
//! `Mode::enter` is about twenty-five assignments and one virtual call. There
//! is almost no decision in it. What there is, is the opportunity to leave one
//! field out — and a field left out is state from the mode the pilot just left
//! quietly steering the mode they just entered. A locked course that survives
//! into a mode that does not lock courses, a crash flag that survives the
//! recovery, an `initial_pitch_cd` from the previous mode's attitude.
//!
//! So this is ported as one struct with one method rather than as loose
//! assignments at the call site, and the parity test fills every field with a
//! distinct non-default sentinel before calling the real firmware's `enter()`.
//! A field the port forgets shows up as a sentinel that survived.
//!
//! # What is not here
//!
//! `Mode::enter` also resets state belonging to subsystems this port has not
//! reached: `guided_state` (offboard guidance), `nav_scripting`, the quadplane
//! transition state, terrain following, and system identification. Those are
//! named rather than silently dropped, and each one is behind a feature gate
//! upstream too. A caller running any of those subsystems must reset them as
//! well; this struct does not pretend to be the whole list.
//!
//! `prev_WP_loc = current_loc` and `last_mode_change_ms = millis()` are also
//! absent: both read something the caller owns rather than resetting anything,
//! so they are the caller's to do.

/// The steering controller's course lock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteerState {
    /// Upstream `steer_state.locked_course`.
    pub locked_course: bool,
    /// Upstream `steer_state.locked_course_err`.
    pub locked_course_err: f32,
}

/// The crash detector's findings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CrashState {
    /// Upstream `crash_state.is_crashed`.
    pub is_crashed: bool,
    /// Upstream `crash_state.impact_detected`.
    pub impact_detected: bool,
}

/// The parts of `auto_state` a mode change clears.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoState {
    /// Flying inverted. Cancelled on any mode change, because a mode that
    /// does not know about inverted flight would fly the aircraft into the
    /// ground while believing it was climbing.
    pub inverted_flight: bool,
    /// Whether to cross-track to the next waypoint. Cleared so a mission
    /// started from a mode change flies to the first waypoint directly rather
    /// than to a track it was never on.
    pub next_wp_crosstrack: bool,
    /// Whether the automatic landing check has run for this approach.
    pub checked_for_autoland: bool,
    /// The highest airspeed seen. Zeroed so the new mode measures its own.
    pub highest_airspeed: f32,
    /// The pitch at the moment the mode was entered, centidegrees. This one is
    /// *seeded* rather than zeroed — see [`ModeEntryState::reset`].
    pub initial_pitch_cd: i16,
    /// Taildragger takeoff handling.
    pub fbwa_tdrag_takeoff_mode: bool,
    /// Whether a takeoff rotation has completed.
    pub rotation_complete: bool,
    /// Whether the entered mode is a VTOL mode. Also *seeded* rather than
    /// zeroed.
    pub vtol_mode: bool,
    /// Whether a VTOL loiter is running.
    pub vtol_loiter: bool,
    /// Servo idle during an altitude-wait command.
    pub idle_mode: bool,
}

/// Everything [`ModeEntryState::reset`] touches.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeEntryState {
    /// The `auto_state` fields.
    pub auto: AutoState,
    /// The steering course lock.
    pub steer: SteerState,
    /// The crash detector.
    pub crash: CrashState,
    /// Upstream `takeoff_state.waiting_for_rudder_neutral`.
    pub waiting_for_rudder_neutral: bool,
    /// Upstream `loiter.start_time_ms`.
    pub loiter_start_time_ms: u32,
    /// Upstream `new_airspeed_cm`, the `DO_CHANGE_SPEED` scratch value.
    /// Reset to −1, not zero: zero is a speed a mission could ask for, so the
    /// "nothing requested" value has to be one that cannot be confused with a
    /// request.
    pub new_airspeed_cm: i32,
    /// Upstream `long_failsafe_pending`. Cleared so a long failsafe queued
    /// before the mode change is not recalled into a mode that never saw it.
    pub long_failsafe_pending: bool,
    /// Upstream `throttle_suppressed`. Set *after* `_enter()` succeeds, from
    /// the entered mode's own answer.
    pub throttle_suppressed: bool,
}

impl ModeEntryState {
    /// Everything a mode change clears before the mode's own `_enter()` runs.
    ///
    /// # Two fields are seeded, not cleared
    ///
    /// `initial_pitch_cd` takes the current attitude and `vtol_mode` takes the
    /// entered mode's answer. They are in the same list as the twenty-odd
    /// fields being zeroed, and they are the two most likely to be
    /// "simplified" into a zero by a port that read the list as uniform.
    /// Zeroing `initial_pitch_cd` would tell a takeoff the aircraft started
    /// level when it started on a ramp.
    pub fn reset(&mut self, current_pitch_cd: i16, entering_vtol_mode: bool) {
        self.auto.inverted_flight = false;
        self.waiting_for_rudder_neutral = false;
        self.auto.next_wp_crosstrack = false;
        self.auto.checked_for_autoland = false;

        self.steer.locked_course_err = 0.0;
        self.steer.locked_course = false;

        self.crash.is_crashed = false;
        self.crash.impact_detected = false;

        self.auto.highest_airspeed = 0.0;
        self.auto.initial_pitch_cd = current_pitch_cd;

        self.auto.fbwa_tdrag_takeoff_mode = false;
        self.auto.rotation_complete = false;

        self.loiter_start_time_ms = 0;

        self.auto.vtol_mode = entering_vtol_mode;
        self.auto.vtol_loiter = false;

        self.new_airspeed_cm = -1;
        self.long_failsafe_pending = false;

        self.auto.idle_mode = false;
    }

    /// The part that runs only once the mode's own `_enter()` has succeeded.
    ///
    /// Upstream is explicit that this must come after: it uses the entered
    /// mode's results, and a mode that refused to start has no results to
    /// use. Suppressing the throttle in an auto-throttle mode is what stops
    /// the aircraft opening up before the mode has decided it should.
    pub fn after_enter(&mut self, does_auto_throttle: bool) {
        self.throttle_suppressed = does_auto_throttle;
    }
}
