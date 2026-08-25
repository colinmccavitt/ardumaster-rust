//! The ground-to-air state machine every altitude-holding mode runs through.
//!
//! Upstream `ArduCopter/mode.cpp:1012`, `Mode::get_alt_hold_state_D_ms`.
//!
//! # Why this returns two things
//!
//! Upstream's signature returns only the state, and commands the motors as a
//! side effect on the way — `motors->set_desired_spool_state(...)` appears in
//! three of its four branches. The command is not incidental to the decision,
//! it *is* half of it: the returned state tells the mode what to do this
//! iteration, and the spool command tells the motors where to be heading so
//! that a later iteration can return something else. A port that returned only
//! the state would drop the half that makes the machine advance.
//!
//! So both come back in an [`AltHoldDecision`] and the caller applies the
//! command. That also makes the machine a pure function of its inputs, which
//! is what lets it be swept exhaustively rather than sampled.
//!
//! # The spool state is read, not assumed
//!
//! Every branch that commands a spool state then *reads* the current one to
//! decide what to return, and the two disagree: the motors take time to spool,
//! so a freshly-commanded `ShutDown` still reads as `ThrottleUnlimited` for
//! as long as the ramp lasts. That is why [`AltHoldInputs::spool_state`] is an
//! input rather than something derived from the command — the aircraft's
//! answer to "where are the motors" is not the vehicle code's to give.

use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// Where an altitude-holding mode sits in the ground-to-air sequence.
///
/// Upstream `Mode::AltHoldModeState`.
///
/// The discriminants are upstream's declaration order, not a sequence anyone
/// would choose: `Takeoff` sits second, between `MotorStopped` and the two
/// landed states. They are pinned because the numbers leave the vehicle —
/// they are logged, and a renumbering would silently reinterpret every
/// existing log.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AltHoldModeState {
    /// Disarmed and spooled down. Nothing turns.
    #[default]
    MotorStopped = 0,
    /// Climbing away under the takeoff helper's control.
    Takeoff = 1,
    /// On the ground with the rotors at idle. Attitude control runs.
    LandedGroundIdle = 2,
    /// On the ground and free to leave it the moment the pilot asks.
    LandedPreTakeoff = 3,
    /// Airborne. The mode's own controller has the aircraft.
    Flying = 4,
}

/// What the machine decided, and what it wants the motors doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AltHoldDecision {
    /// What the calling mode should do this iteration.
    pub state: AltHoldModeState,
    /// Where the motors should be heading, or `None` where upstream issues no
    /// command and the previous one stands.
    ///
    /// The absence is represented rather than filled in. Upstream's takeoff
    /// branch commands nothing, and substituting the value it "obviously"
    /// wants would overwrite a command a previous iteration made — which is a
    /// different machine, not a tidier spelling of this one.
    pub desired_spool: Option<DesiredSpoolState>,
}

/// Everything the machine reads.
#[derive(Debug, Clone, Copy)]
pub struct AltHoldInputs {
    /// `motors->armed()`.
    pub armed: bool,
    /// `motors->get_spool_state()` — where the motors are, not where they were
    /// last told to go.
    pub spool_state: SpoolState,
    /// `takeoff.running()` — the takeoff helper is already flying the climb.
    pub takeoff_running: bool,
    /// `copter.ap.auto_armed`. Distinct from `armed`: the aircraft is armed but
    /// the throttle has not yet been raised, so it is not yet allowed to fly.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.using_interlock` — a motor interlock (a helicopter's rotor
    /// switch) is gating the output.
    pub using_interlock: bool,
    /// The climb rate the mode is asking for, m/s, up positive.
    pub target_climb_rate_ms: f32,
}

/// Whether a takeoff should begin, upstream `Mode::_TakeOff::triggered_ms`.
///
/// Three conditions, each rejecting for its own reason: already flying, not
/// actually asking to go up, or the rotors have not finished running up.
///
/// # The threshold is not symmetric with the machine's
///
/// This rejects `target_climb_rate_ms <= 0.0`, so a rate of exactly zero does
/// not trigger a takeoff. The landed branch of [`alt_hold_state`] tests
/// `< 0.0` instead, so a rate of exactly zero there means "prepare to fly"
/// rather than "settle to idle". Both readings of zero are deliberate: a stick
/// at centre is not a request to leave the ground, but neither is it a request
/// to spool down.
#[must_use]
pub fn takeoff_triggered(
    land_complete: bool,
    target_climb_rate_ms: f32,
    spool_state: SpoolState,
) -> bool {
    if !land_complete {
        // Already flying; there is nothing to take off from.
        return false;
    }
    if target_climb_rate_ms <= 0.0 {
        return false;
    }
    // Hold the aircraft down until the rotors have finished running up.
    spool_state == SpoolState::ThrottleUnlimited
}

/// The altitude-hold state machine.
///
/// # The order of the branches carries meaning
///
/// Disarmed is tested first and unconditionally, so nothing below it can keep
/// the motors alive. Takeoff is tested before landed, so an aircraft that has
/// begun its climb is not dragged back to a ground state by a `land_complete`
/// that has not yet cleared. Only what survives all three is flying.
#[must_use]
pub fn alt_hold_state(inputs: &AltHoldInputs) -> AltHoldDecision {
    if !inputs.armed {
        // Whatever the mode wanted, a disarmed aircraft shuts down. The state
        // returned is where the motors *are*, which is not yet where they have
        // just been told to go.
        let state = match inputs.spool_state {
            SpoolState::ShutDown => AltHoldModeState::MotorStopped,
            SpoolState::GroundIdle => AltHoldModeState::LandedGroundIdle,
            // Still spooling down, or not yet begun. Treat it as on the ground
            // and able to move, because the motors still have authority.
            _ => AltHoldModeState::LandedPreTakeoff,
        };
        return AltHoldDecision {
            state,
            desired_spool: Some(DesiredSpoolState::ShutDown),
        };
    }

    if inputs.takeoff_running
        || takeoff_triggered(
            inputs.land_complete,
            inputs.target_climb_rate_ms,
            inputs.spool_state,
        )
    {
        // Upstream issues no spool command here, and this port does not invent
        // one. `triggered_ms` only fires once the motors already read
        // THROTTLE_UNLIMITED, so on the common path there is nothing left to
        // ask for — but `takeoff_running` can hold across an iteration in
        // which the motors were commanded elsewhere, and there the standing
        // command is the one that should survive.
        return AltHoldDecision {
            state: AltHoldModeState::Takeoff,
            desired_spool: None,
        };
    }

    if !inputs.auto_armed || inputs.land_complete {
        let desired_spool = if inputs.target_climb_rate_ms < 0.0 && !inputs.using_interlock {
            // Asked to descend while already down: settle to idle. The
            // interlock exclusion is for helicopters, where the rotor is
            // commanded separately and spooling down behind the pilot's back
            // would be the wrong response to a downward stick.
            DesiredSpoolState::GroundIdle
        } else {
            DesiredSpoolState::ThrottleUnlimited
        };

        let state = if inputs.spool_state == SpoolState::GroundIdle {
            AltHoldModeState::LandedGroundIdle
        } else {
            AltHoldModeState::LandedPreTakeoff
        };

        return AltHoldDecision {
            state,
            desired_spool: Some(desired_spool),
        };
    }

    AltHoldDecision {
        state: AltHoldModeState::Flying,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
    }
}
