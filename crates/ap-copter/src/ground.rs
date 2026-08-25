//! What a mode does while the aircraft is on the ground, and what happens on
//! the way out of one.
//!
//! Upstream `ArduCopter/mode.cpp`. These are small decisions attached to large
//! side effects — each one commands the motors, the attitude controller or
//! both. As elsewhere in this crate the decision is returned and the caller
//! performs it, so the choice can be swept exhaustively instead of inferred
//! from what a controller looked like afterwards.

use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// Whether the vehicle is not currently flying, upstream
/// `Mode::is_disarmed_or_landed`.
///
/// Three ways of not flying, and they are genuinely different states rather
/// than three spellings of one. Not armed: the motors cannot turn.
/// `auto_armed` false: armed, but the pilot has not yet raised the throttle,
/// so the vehicle is holding still on purpose. `land_complete`: the landing
/// detector believes it is on the ground.
///
/// Callers use this to decide whether to run a controller at all, and any of
/// the three is reason enough not to.
#[must_use]
pub fn is_disarmed_or_landed(armed: bool, auto_armed: bool, land_complete: bool) -> bool {
    !armed || !auto_armed || land_complete
}

/// The spool state `Mode::zero_throttle_and_relax_ac` commands.
///
/// The attitude and throttle it commands alongside are fixed — level, zero —
/// so the only thing that varies is where the motors are asked to be.
///
/// `spool_up` exists for the modes that must keep the rotors turning while
/// they sit at zero throttle. Spooling all the way down and back up costs
/// seconds a mode may not have.
#[must_use]
pub fn zero_throttle_spool(spool_up: bool) -> DesiredSpoolState {
    if spool_up {
        DesiredSpoolState::ThrottleUnlimited
    } else {
        DesiredSpoolState::GroundIdle
    }
}

/// What `Mode::make_safe_ground_handling` decides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroundHandling {
    /// Where the motors should be heading.
    pub desired_spool: DesiredSpoolState,
    /// Whether to reset the yaw target and rate.
    pub reset_yaw_target_and_rate: bool,
}

/// Holding an aircraft safely on the ground, upstream
/// `Mode::make_safe_ground_handling`.
///
/// # Why the yaw reset is conditional and the I-term reset is not
///
/// The rate controllers' integrators are reset every call, whatever the
/// motors are doing: an integrator winding up against the ground is what
/// produces the twitch on takeoff, and it accumulates whether or not the
/// rotors are turning.
///
/// The yaw *target* is different. Resetting it means "wherever the aircraft is
/// pointing now is what we are asking for", which is right while the motors
/// are idle or stopped and the airframe may be pushed around by hand or by
/// wind. Once the motors are spooling or unlimited the aircraft is holding a
/// heading on purpose, and snapping the target to the current heading each
/// iteration would throw away a demand a pilot or a mission is making.
///
/// # `force_throttle_unlimited` is for helicopters
///
/// A traditional helicopter's main rotor stops at ground idle, so Guided and
/// Auto keep it spooled up while waiting on the ground — which in turn makes
/// the motor interlock the thing that decides whether the rotor turns, rather
/// than the spool state.
#[must_use]
pub fn make_safe_ground_handling(
    force_throttle_unlimited: bool,
    spool_state: SpoolState,
) -> GroundHandling {
    GroundHandling {
        desired_spool: if force_throttle_unlimited {
            DesiredSpoolState::ThrottleUnlimited
        } else {
            DesiredSpoolState::GroundIdle
        },
        reset_yaw_target_and_rate: matches!(
            spool_state,
            SpoolState::ShutDown | SpoolState::GroundIdle
        ),
    }
}

/// Whether leaving one mode for another needs the throttle integrator seeded,
/// upstream the first branch of `Copter::exit_mode`.
///
/// Going from a mode where the pilot's stick *is* the throttle to one where a
/// controller decides it, mid-flight, means the controller starts from
/// nothing while the aircraft is already holding altitude on whatever the
/// pilot had set. Seeding its integrator from the pilot's throttle makes the
/// handover continuous instead of a drop followed by a catch.
///
/// The conditions are exactly the cases where that discontinuity could be
/// felt. Disarmed or landed, there is no altitude to lose.
#[must_use]
pub fn smooth_throttle_transition_on_exit(
    old_has_manual_throttle: bool,
    new_has_manual_throttle: bool,
    armed: bool,
    land_complete: bool,
) -> bool {
    old_has_manual_throttle && !new_has_manual_throttle && armed && !land_complete
}

/// How the position controller should absorb an EKF position reset.
///
/// Upstream `AC_PosControl::EKFResetMethod`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EkfResetMethod {
    /// Move the target with the estimate, so the aircraft holds still and the
    /// destination shifts under it.
    MoveTarget = 0,
    /// Move the vehicle's idea of itself, so the target stays put and the
    /// aircraft flies to it.
    MoveVehicle = 1,
}

/// Which reset method the running mode wants, upstream the line in
/// `Copter::update_flight_mode`.
///
/// An EKF reset means the estimate of where the aircraft is has jumped, while
/// the aircraft itself has not moved. Every mode has to decide what that jump
/// means for the thing it was trying to do.
///
/// A mode holding a position — loiter, or a mission leg — wants the target
/// moved with the estimate, so the aircraft stays where it physically is
/// rather than lurching to correct an error that was never real. A mode whose
/// target is defined relative to the vehicle wants the opposite.
#[must_use]
pub fn ekf_reset_method(move_vehicle_on_ekf_reset: bool) -> EkfResetMethod {
    if move_vehicle_on_ekf_reset {
        EkfResetMethod::MoveVehicle
    } else {
        EkfResetMethod::MoveTarget
    }
}
