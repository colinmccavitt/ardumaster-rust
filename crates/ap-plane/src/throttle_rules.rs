//! Which corrections a mode's throttle is subject to.
//!
//! Upstream `ArduPlane/mode.cpp:322` and `:354`, `Mode::use_throttle_limits`
//! and `Mode::use_battery_compensation`.
//!
//! # Two functions that look like one
//!
//! They read the same four things in the same order and differ in exactly two
//! places. That similarity is a trap: deduplicating them into one predicate
//! with a flag would be an easy change to make and would silently alter
//! behaviour, so they are ported as two functions and the divergences have
//! their own tests.
//!
//! The divergences are both principled.
//!
//! In a manual-throttle mode, battery compensation is always off, while
//! throttle limits are off only when `THR_PASS_STAB` is set. A pilot flying
//! on the stick expects the stick position to mean a throttle position;
//! silently scaling it as the battery sags would make the aircraft respond
//! differently to the same stick as the flight went on. But the configured
//! limits still apply unless the pilot has explicitly asked for a direct
//! mapping.
//!
//! In a VTOL mode, battery compensation is off, while throttle limits defer
//! to the quadplane, which knows whether forward throttle is allowed at all.
//!
//! # MANUAL overrides both
//!
//! It is the only mode that does, and it is not in the base's five-mode
//! manual-throttle list — it does not need to be, because it replaces the
//! predicates outright. See [`manual_use_throttle_limits`].

/// What the vehicle is doing, as these two predicates see it.
#[derive(Debug, Clone, Copy)]
pub struct ThrottleContext {
    /// A Lua script is flying the aircraft. Nothing else applies while it is.
    pub nav_scripting_active: bool,
    /// The mode flies on the pilot's throttle stick: STABILIZE, TRAINING,
    /// ACRO, FBWA or AUTOTUNE.
    pub manual_throttle_mode: bool,
    /// `THR_PASS_STAB`.
    pub throttle_passthru_stabilize: bool,
    /// A guided mode with `guided_throttle_passthru` set — the ground station
    /// is passing a throttle straight through.
    pub guided_throttle_passthru: bool,
    /// The vehicle is in a VTOL mode.
    pub in_vtol_mode: bool,
    /// `quadplane.allow_forward_throttle_in_vtol_mode()`. Only consulted in a
    /// VTOL mode.
    pub allow_forward_throttle_in_vtol: bool,
}

/// Whether the configured throttle limits apply, upstream
/// `Mode::use_throttle_limits`.
#[must_use]
pub fn use_throttle_limits(context: &ThrottleContext) -> bool {
    if context.nav_scripting_active {
        return false;
    }

    if context.manual_throttle_mode {
        // The one place this differs from battery compensation: the limits
        // still apply unless the pilot asked for a direct mapping.
        return !context.throttle_passthru_stabilize;
    }

    if context.guided_throttle_passthru {
        return false;
    }

    if context.in_vtol_mode {
        // The quadplane decides, because it knows whether forward throttle is
        // wanted at all in this VTOL mode.
        return context.allow_forward_throttle_in_vtol;
    }

    true
}

/// Whether the throttle is corrected for battery voltage, upstream
/// `Mode::use_battery_compensation`.
///
/// The same four questions as [`use_throttle_limits`], with two different
/// answers. See the module documentation.
#[must_use]
pub fn use_battery_compensation(context: &ThrottleContext) -> bool {
    if context.nav_scripting_active {
        return false;
    }

    if context.manual_throttle_mode {
        // Unconditionally off, unlike the limits: the stick means a throttle
        // position, and rescaling it as the battery sags would change what
        // the same stick does over a flight.
        return false;
    }

    if context.guided_throttle_passthru {
        return false;
    }

    if context.in_vtol_mode {
        // Also unconditional, unlike the limits.
        return false;
    }

    true
}

/// What a fixed-wing system-identification run needs to be true.
#[derive(Debug, Clone, Copy)]
pub struct SystemIdContext {
    /// The mode supports fixed-wing system ID at all.
    pub mode_supports: bool,
    /// Taking off.
    pub taking_off: bool,
    /// Landing.
    pub landing: bool,
    /// A quadplane is available.
    pub quadplane_available: bool,
    /// VTOL motors are assisting forward flight.
    pub in_assisted_flight: bool,
    /// The transition to forward flight has completed.
    pub transition_complete: bool,
}

/// Whether a fixed-wing system identification may run, upstream
/// `Mode::allow_fw_systemid`.
///
/// System identification injects deliberate disturbances to measure how the
/// airframe responds. Every rejection here is a phase of flight where a
/// disturbance would be answered by something other than the fixed wing —
/// the ground, or the VTOL motors — so the measurement would describe
/// something that is not the aircraft's fixed-wing response.
#[must_use]
pub fn allow_fw_systemid(context: &SystemIdContext) -> bool {
    if !context.mode_supports {
        return false;
    }

    if context.taking_off || context.landing {
        return false;
    }

    if context.quadplane_available {
        if context.in_assisted_flight {
            // VTOL motors are contributing, so the response is not the wing's.
            return false;
        }
        if !context.transition_complete {
            return false;
        }
    }

    true
}

/// Whether the *vertical* throttle is under manual control, upstream
/// `Mode::is_vtol_man_throttle`.
///
/// # The inverted pair
///
/// Only true for a tailsitter that has fully transitioned to Q-assisted
/// forward flight, where the forward throttle directly drives the vertical
/// one. Upstream's own comment flags the confusion and it is worth repeating:
/// the forward throttle asks `does_auto_throttle`, the vertical asks
/// `is_vtol_man_throttle`, and the two booleans mean opposite things. So the
/// answer here is the *negation* of the forward throttle's automatic flag,
/// not a copy of it.
///
/// Everywhere else it is false, including on a tailsitter that has not
/// transitioned.
#[must_use]
pub fn is_vtol_man_throttle(
    tailsitter_in_fw_flight: bool,
    assisted_flight: bool,
    does_auto_throttle: bool,
) -> bool {
    if tailsitter_in_fw_flight && assisted_flight {
        return !does_auto_throttle;
    }
    false
}

/// MANUAL's override of [`use_throttle_limits`], upstream
/// `ModeManual::use_throttle_limits` in `mode_manual.cpp`.
///
/// # MANUAL is not in the base's manual-throttle list
///
/// The base implementation names five modes as manual-throttle — STABILIZE,
/// TRAINING, ACRO, FBWA, AUTOTUNE — and MANUAL is not among them. It does not
/// need to be, because it overrides the whole predicate: in MANUAL the stick
/// *is* the output, so the configured limits do not apply at all.
///
/// The exception is a quadplane with `IDLE_GOV_MANUAL`, where the idle
/// governor needs the limits in order to hold the motor at idle rather than
/// letting a closed stick stop it.
///
/// This caught a real gap. The port modelled only the base implementations,
/// and the first recorded row disagreed — reading `mode.cpp` alone does not
/// show the override, which is declared in `mode.h` and defined in
/// `mode_manual.cpp`.
#[must_use]
pub fn manual_use_throttle_limits(quadplane_available: bool, idle_gov_manual: bool) -> bool {
    quadplane_available && idle_gov_manual
}

/// MANUAL's override of [`use_battery_compensation`], which is unconditional.
///
/// Upstream writes it inline in `mode.h` as `{ return false; }`. In MANUAL the
/// stick is the output, and rescaling it for battery voltage would mean the
/// same stick commanded a different throttle as the flight went on — which is
/// the one thing a manual mode promises not to do.
#[must_use]
pub const fn manual_use_battery_compensation() -> bool {
    false
}
