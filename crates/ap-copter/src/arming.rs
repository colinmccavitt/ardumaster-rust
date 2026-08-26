//! ArduCopter's pre-arm checks, upstream `ArduCopter/AP_Arming_Copter.cpp`.
//!
//! Every one of these is the last thing between a parameter set someone got
//! wrong and a vehicle with spinning propellers, so each is ported as its own
//! decision returning its own reason rather than folded into a single
//! predicate. A pilot who is refused arming acts on the message; merging them
//! would leave the aircraft equally unarmed and the pilot with nothing to go
//! on.

/// Why a pre-arm check refused.
///
/// The strings are upstream's, because they are what reaches the pilot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmRefusal {
    /// The system had not finished booting.
    SystemNotInitialised,
    /// A motor interlock switch and an emergency-stop switch are both
    /// assigned.
    InterlockEstopConflict,
    /// The motor interlock switch is enabled.
    MotorInterlockEnabled,
    /// No RC receiver has ever been seen.
    RcNotFound,
    /// The throttle is below the failsafe threshold.
    ThrottleBelowFailsafe,
    /// A battery failsafe is active.
    BatteryFailsafe,
    /// The ground station failsafe is active.
    GcsFailsafeOn,
    /// No usable altitude estimate.
    NeedAltEstimate,
}

impl ArmRefusal {
    /// Upstream's message for this refusal.
    ///
    /// `ThrottleBelowFailsafe` is formatted from a frame-dependent noun —
    /// "Collective" on a traditional helicopter, "Throttle" otherwise — so it
    /// takes the multirotor form here. This port is multirotor; a helicopter
    /// build would need the other.
    #[must_use]
    pub fn message(self) -> &'static str {
        match self {
            Self::SystemNotInitialised => "System not initialised",
            Self::InterlockEstopConflict => "Interlock/E-Stop Conflict",
            Self::MotorInterlockEnabled => "Motor Interlock Enabled",
            Self::RcNotFound => "RC not found",
            Self::ThrottleBelowFailsafe => "Throttle below failsafe",
            Self::BatteryFailsafe => "Battery failsafe",
            Self::GcsFailsafeOn => "GCS failsafe on",
            Self::NeedAltEstimate => "Need Alt Estimate",
        }
    }
}

/// `FS_THR_DISABLED`, the value of `FS_THR_ENABLE` that turns the throttle
/// failsafe off.
pub const FS_THR_DISABLED: u8 = 0;

/// What the RC and throttle-failsafe state looks like to the check.
#[derive(Debug, Clone, Copy)]
pub struct RcFailsafeState {
    /// `check_enabled(Check::RC)` — the operator has not switched this check
    /// off with `ARMING_CHECK`.
    pub rc_check_enabled: bool,
    /// `FS_THR_ENABLE`.
    pub failsafe_throttle: u8,
    /// An RC receiver has been seen at some point.
    pub has_had_rc_receiver: bool,
    /// An RC override has been received from a ground station.
    pub has_had_rc_override: bool,
    /// The throttle channel's raw radio input, microseconds.
    pub throttle_radio_in: u16,
    /// `FS_THR_VALUE`, the microsecond threshold below which the throttle
    /// counts as failed.
    pub failsafe_throttle_value: u16,
}

/// The RC throttle-failsafe pre-arm check, upstream
/// `rc_throttle_failsafe_checks`.
///
/// # The failsafe parameter gates the no-pulses case too
///
/// Upstream's comment is worth keeping: `FS_THR_ENABLE` also gates the
/// no-RC-pulses failure, because a radio that has sent nothing leaves
/// `radio_in` at zero, which is below any threshold. An operator who turns
/// the throttle failsafe off is therefore also turning off the check that
/// would notice a receiver that never spoke.
///
/// The same comment notes the residual risk and why it is acceptable: if RC
/// was seen and then lost, these checks may pass — but arming is precluded
/// anyway by the vehicle being in RC failsafe, which is a different gate.
///
/// # An override counts as a receiver
///
/// A ground station sending RC overrides satisfies the receiver check, so a
/// vehicle flown entirely from a companion computer can arm without a radio.
#[must_use]
pub fn rc_throttle_failsafe_check(state: &RcFailsafeState) -> Option<ArmRefusal> {
    if !state.rc_check_enabled {
        return None;
    }
    if state.failsafe_throttle == FS_THR_DISABLED {
        return None;
    }
    if !state.has_had_rc_receiver && !state.has_had_rc_override {
        return Some(ArmRefusal::RcNotFound);
    }
    if state.throttle_radio_in < state.failsafe_throttle_value {
        return Some(ArmRefusal::ThrottleBelowFailsafe);
    }
    None
}

/// The battery half of `board_voltage_checks`, upstream's addition to the
/// shared `AP_Arming` implementation.
///
/// The base class's own voltage check runs first and is not duplicated here;
/// this is only the Copter-specific battery-failsafe test, and it is gated by
/// `ARMING_CHECK`'s voltage bit.
#[must_use]
pub fn battery_failsafe_check(
    voltage_check_enabled: bool,
    battery_has_failsafed: bool,
) -> Option<ArmRefusal> {
    if voltage_check_enabled && battery_has_failsafed {
        return Some(ArmRefusal::BatteryFailsafe);
    }
    None
}

/// The ground-station failsafe check, upstream `gcs_failsafe_check`.
///
/// Not gated by `ARMING_CHECK`: an operator cannot switch this one off. A
/// vehicle that has lost its ground station is one nobody is watching.
#[must_use]
pub fn gcs_failsafe_check(gcs_failsafe: bool) -> Option<ArmRefusal> {
    if gcs_failsafe {
        return Some(ArmRefusal::GcsFailsafeOn);
    }
    None
}

/// The altitude check, upstream `alt_checks`.
///
/// # Manual-throttle modes are exempt
///
/// A mode where the pilot's stick *is* the throttle needs no altitude
/// estimate, because nothing is trying to hold a height. Requiring one would
/// stop a pilot taking off in Stabilize on a day the barometer is unhappy,
/// which is exactly when they most want the mode that does not depend on it.
///
/// Also not gated by `ARMING_CHECK` — upstream's comment says "always EKF
/// altitude estimate".
#[must_use]
pub fn alt_check(mode_has_manual_throttle: bool, ekf_alt_ok: bool) -> Option<ArmRefusal> {
    if !mode_has_manual_throttle && !ekf_alt_ok {
        return Some(ArmRefusal::NeedAltEstimate);
    }
    None
}

/// The interlock and emergency-stop switch assignments.
#[derive(Debug, Clone, Copy)]
pub struct InterlockSwitches {
    /// A channel is assigned to `MOTOR_INTERLOCK`.
    pub motor_interlock_assigned: bool,
    /// A channel is assigned to `MOTOR_ESTOP`.
    pub motor_estop_assigned: bool,
    /// A channel is assigned to `ARM_EMERGENCY_STOP`.
    pub arm_emergency_stop_assigned: bool,
    /// `copter.ap.using_interlock`.
    pub using_interlock: bool,
    /// `copter.ap.motor_interlock_switch` — the switch is in the enabled
    /// position.
    pub motor_interlock_switch: bool,
}

/// The two interlock checks at the top of `run_pre_arm_checks`.
///
/// # Both switches at once is a configuration error, not a state
///
/// A motor interlock says "the motors may turn"; an emergency stop says "the
/// motors must not". Assigning both leaves two switches with authority over
/// the same thing and no defined precedence, so upstream refuses the
/// configuration outright rather than picking a winner.
///
/// # The interlock must be *disabled* to arm
///
/// It reads backwards until you see it from the aircraft's side: arming with
/// the interlock already enabled would mean the propellers are permitted to
/// turn the instant arming completes, with nothing between the operator's
/// button press and a spinning rotor.
///
/// Returns the refusals in upstream's order. Note that upstream does *not*
/// return early on either — it records the failure and carries on, so a
/// vehicle with both problems is told about both. That is why this returns a
/// list rather than the first refusal.
#[must_use]
pub fn interlock_checks(switches: &InterlockSwitches) -> [Option<ArmRefusal>; 2] {
    let conflict = switches.motor_interlock_assigned
        && (switches.motor_estop_assigned || switches.arm_emergency_stop_assigned);

    [
        conflict.then_some(ArmRefusal::InterlockEstopConflict),
        (switches.using_interlock && switches.motor_interlock_switch)
            .then_some(ArmRefusal::MotorInterlockEnabled),
    ]
}

/// The system-initialised check, upstream's second test in
/// `run_pre_arm_checks`.
///
/// Unlike the interlock checks below it this one returns immediately: nothing
/// else can be trusted before the scheduler says the system is up, so running
/// further checks would produce answers about uninitialised state.
#[must_use]
pub fn system_initialised_check(is_system_initialized: bool) -> Option<ArmRefusal> {
    if is_system_initialized {
        return None;
    }
    Some(ArmRefusal::SystemNotInitialised)
}

/// Whether the pre-arm checks run at all, upstream the first line of
/// `run_pre_arm_checks`.
///
/// An already-armed vehicle passes without any check running. The checks
/// exist to decide whether arming may begin; re-running them on an armed
/// aircraft could only produce a refusal for something already in progress.
#[must_use]
pub fn pre_arm_checks_apply(already_armed: bool) -> bool {
    !already_armed
}
