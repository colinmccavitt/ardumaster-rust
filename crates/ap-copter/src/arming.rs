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

/// `FS_GCS_ENABLED_CONTINUE_MISSION`, the removed value 2 of `FS_GCS_ENABLE`.
pub const FS_GCS_ENABLED_CONTINUE_MISSION: u8 = 2;

/// The lowest `FS_THR_VALUE` upstream will accept.
///
/// A PPM encoder signals loss of signal by outputting 900 microseconds, so a
/// failsafe threshold at or below that could never distinguish a lost link
/// from a legitimately low stick.
pub const MIN_FAILSAFE_THROTTLE_VALUE: u16 = 910;

/// The margin `RC3_MIN` must clear `FS_THR_VALUE` by.
///
/// Without it a throttle resting at its own minimum would sit on the failsafe
/// threshold, and the aircraft would failsafe the moment the stick was pulled
/// fully down.
pub const FAILSAFE_THROTTLE_MARGIN: u16 = 10;

/// Why a parameter check refused, or warned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterRefusal {
    /// `FS_THR_VALUE` is unusable against `RC3_MIN` or the encoder floor.
    CheckFsThrValue,
    /// `FS_GCS_ENABLE=2` was removed. **Does not block arming** — see
    /// [`parameter_checks`].
    FsGcsEnable2Removed,
    /// The acro balance gains are negative or exceed the angle-P gains.
    CheckAcroBalance,
    /// `PILOT_SPD_UP` is not positive.
    CheckPilotSpdUp,
    /// A helicopter frame class on a multirotor build.
    InvalidMulticopterFrameClass,
    /// `RTL_ALT_TYPE` is above-terrain but there is no terrain data.
    RtlTerrainNoData,
    /// `RTL_ALT_TYPE` is above-terrain but there is no downward rangefinder.
    RtlTerrainNoRangefinder,
    /// `RTL_ALT_M` exceeds the rangefinder's maximum range.
    RtlAltAboveRangefinderMax,
    /// An ADS-B threat is active.
    AdsbThreatDetected,
    /// The position controller refused its own parameters.
    BadPositionControllerParameter,
    /// The attitude controller refused its own parameters.
    BadAttitudeControllerParameter,
}

impl ParameterRefusal {
    /// Whether this refusal actually prevents arming.
    ///
    /// All but one do. `FsGcsEnable2Removed` is reported and then execution
    /// falls through to the next check — upstream calls `check_failed` without
    /// returning, so the operator is told their parameter is obsolete without
    /// being grounded by it. That is easy to miss reading the function, and
    /// easy to "tidy" into a return.
    #[must_use]
    pub fn blocks_arming(self) -> bool {
        self != Self::FsGcsEnable2Removed
    }
}

/// Where `RTL_ALT_TYPE`'s terrain data would come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainSource {
    /// No terrain source at all.
    Unavailable,
    /// A downward rangefinder.
    Rangefinder,
    /// The terrain database. Checked by the shared `AP_Arming`, not here.
    Database,
}

/// Everything `parameter_checks` reads.
#[derive(Debug, Clone, Copy)]
pub struct ParameterState {
    /// `check_enabled(Check::PARAMETERS)`.
    pub parameter_check_enabled: bool,
    /// `FS_THR_ENABLE`, non-zero meaning the throttle failsafe is on.
    pub failsafe_throttle: u8,
    /// `RC3_MIN`.
    pub throttle_radio_min: u16,
    /// `FS_THR_VALUE`.
    pub failsafe_throttle_value: u16,
    /// `FS_GCS_ENABLE`.
    pub failsafe_gcs: u8,
    /// `ACRO_BAL_ROLL`.
    pub acro_balance_roll: f32,
    /// `ACRO_BAL_PITCH`.
    pub acro_balance_pitch: f32,
    /// The attitude controller's roll angle-P gain.
    pub angle_roll_p: f32,
    /// The attitude controller's pitch angle-P gain.
    pub angle_pitch_p: f32,
    /// `PILOT_SPD_UP`, m/s.
    pub pilot_speed_up_ms: f32,
    /// The frame class is one of the helicopter ones.
    pub frame_class_is_heli: bool,
    /// `RTL_ALT_TYPE` is above-terrain.
    pub rtl_alt_type_is_terrain: bool,
    /// Where terrain data would come from.
    pub terrain_source: TerrainSource,
    /// A downward rangefinder is enabled and present.
    pub rangefinder_available: bool,
    /// `RTL_ALT_M`.
    pub rtl_altitude_m: f32,
    /// The rangefinder's maximum range, metres.
    pub rangefinder_max_distance_m: f32,
    /// An ADS-B failsafe is active.
    pub adsb_failsafe: bool,
    /// The position controller accepted its parameters.
    pub pos_control_ok: bool,
    /// The attitude controller accepted its parameters.
    pub attitude_control_ok: bool,
}

/// The parameter pre-arm checks, upstream `AP_Arming_Copter::parameter_checks`.
///
/// Returns every refusal raised, in upstream's order. Most stop the ladder;
/// [`ParameterRefusal::blocks_arming`] says which do not, and the list exists
/// because the one that does not is followed by checks that still run.
///
/// # The throttle failsafe parameters are checked against each other
///
/// `FS_THR_VALUE` has to sit above the PPM encoder's loss-of-signal output
/// *and* below `RC3_MIN` by a margin. Between them these say the threshold
/// must be distinguishable from a dead link at one end and from a pilot
/// holding the stick down at the other — a value satisfying neither would
/// either never fire or fire constantly.
///
/// # Two slots, because two is the most one call can raise
///
/// Only one refusal is non-blocking, and it is followed by checks that still
/// run — so a call yields at most the warning plus whichever blocking refusal
/// comes after it. The array says that in the type.
#[must_use]
pub fn parameter_checks(state: &ParameterState) -> [Option<ParameterRefusal>; 2] {
    if !state.parameter_check_enabled {
        return [None, None];
    }

    // Above the warning: this rung returns, so nothing below it runs.
    if state.failsafe_throttle != 0
        && (state.throttle_radio_min <= state.failsafe_throttle_value + FAILSAFE_THROTTLE_MARGIN
            || state.failsafe_throttle_value < MIN_FAILSAFE_THROTTLE_VALUE)
    {
        return [Some(ParameterRefusal::CheckFsThrValue), None];
    }

    // The one rung that reports and carries on, so it can be accompanied by
    // whatever fires below it.
    let warning = (state.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION)
        .then_some(ParameterRefusal::FsGcsEnable2Removed);

    // Dense and in order: a caller reading slot 0 gets the first refusal
    // raised, not a hole where the warning would have been. Only the warning
    // can be followed by anything, so two slots is the most that is needed.
    match (warning, first_blocking_parameter_refusal(state)) {
        (Some(w), blocking) => [Some(w), blocking],
        (None, blocking) => [blocking, None],
    }
}

/// The rungs below the `FS_GCS_ENABLE` warning, in upstream's order.
///
/// Each of these returns, so at most one can fire.
fn first_blocking_parameter_refusal(state: &ParameterState) -> Option<ParameterRefusal> {
    if state.acro_balance_roll < 0.0
        || state.acro_balance_pitch < 0.0
        || state.acro_balance_roll > state.angle_roll_p
        || state.acro_balance_pitch > state.angle_pitch_p
    {
        return Some(ParameterRefusal::CheckAcroBalance);
    }

    if state.pilot_speed_up_ms <= 0.0 {
        return Some(ParameterRefusal::CheckPilotSpdUp);
    }

    if state.frame_class_is_heli {
        return Some(ParameterRefusal::InvalidMulticopterFrameClass);
    }

    if state.rtl_alt_type_is_terrain {
        if let Some(refusal) = above_terrain_rtl_refusal(state) {
            return Some(refusal);
        }
    }

    if state.adsb_failsafe {
        return Some(ParameterRefusal::AdsbThreatDetected);
    }

    if !state.pos_control_ok {
        return Some(ParameterRefusal::BadPositionControllerParameter);
    }
    if !state.attitude_control_ok {
        return Some(ParameterRefusal::BadAttitudeControllerParameter);
    }
    None
}

/// The above-terrain RTL rungs, upstream's switch on the terrain source.
///
/// The database case is deliberately empty: upstream's comment says those
/// checks are done in the shared `AP_Arming`, so duplicating them here would
/// mean two places to keep in step.
fn above_terrain_rtl_refusal(state: &ParameterState) -> Option<ParameterRefusal> {
    match state.terrain_source {
        TerrainSource::Unavailable => Some(ParameterRefusal::RtlTerrainNoData),
        TerrainSource::Rangefinder => {
            if !state.rangefinder_available {
                return Some(ParameterRefusal::RtlTerrainNoRangefinder);
            }
            if state.rtl_altitude_m > state.rangefinder_max_distance_m {
                return Some(ParameterRefusal::RtlAltAboveRangefinderMax);
            }
            None
        }
        TerrainSource::Database => None,
    }
}

/// How far the barometer and inertial-nav altitudes may disagree, upstream
/// `PREARM_MAX_ALT_DISPARITY_M` (`config.h:97`).
pub const PREARM_MAX_ALT_DISPARITY_M: f32 = 1.0;

/// The highest `RCn_MIN` a calibrated channel may have, upstream
/// `RC_Channel::RC_CALIB_MIN_LIMIT_PWM`.
pub const RC_CALIB_MIN_LIMIT_PWM: u16 = 1300;

/// The lowest `RCn_MAX` a calibrated channel may have, upstream
/// `RC_Channel::RC_CALIB_MAX_LIMIT_PWM`.
pub const RC_CALIB_MAX_LIMIT_PWM: u16 = 1700;

/// The barometer pre-arm check, upstream the Copter half of
/// `barometer_checks`.
///
/// # Only when the EKF is using an absolute height reference
///
/// The comparison is skipped when the estimator is producing a *ground
/// relative* height, because that legitimately differs from the barometer as
/// the baro drifts. Upstream derives "using an absolute reference" from the
/// two prediction-status flags: absolute position predicted, relative not.
///
/// Checking regardless would refuse arming on any vehicle flying terrain-
/// relative, which is exactly the configuration where the disparity is
/// expected rather than a fault.
///
/// # It does not return early
///
/// Upstream sets a flag and falls through, so this check runs alongside the
/// shared `AP_Arming::barometer_checks` rather than pre-empting it.
#[must_use]
pub fn altitude_disparity_check(
    baro_check_enabled: bool,
    predicts_relative_position: bool,
    predicts_absolute_position: bool,
    inertial_height_m: f32,
    baro_altitude_m: f32,
) -> bool {
    if !baro_check_enabled {
        return true;
    }
    let using_baro_reference = !predicts_relative_position && predicts_absolute_position;
    if !using_baro_reference {
        return true;
    }
    libm::fabsf(inertial_height_m - baro_altitude_m) <= PREARM_MAX_ALT_DISPARITY_M
}

/// The Copter half of `ins_checks`, upstream's EKF attitude test.
///
/// Upstream's comment names the usual cause: a bad EKF attitude is normally
/// the gyro biases still settling. It is worth knowing because the message a
/// pilot sees says "EKF attitude is bad", which sounds like a fault rather
/// than like "wait a moment".
#[must_use]
pub fn ekf_attitude_check(ins_check_enabled: bool, ekf_attitude_ok: bool) -> bool {
    if !ins_check_enabled {
        return true;
    }
    ekf_attitude_ok
}

/// Which end of a channel's calibration is wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcCalibrationFault {
    /// `RCn_MIN` is above the limit — the stick cannot reach low enough.
    MinTooHigh,
    /// `RCn_MAX` is below the limit — the stick cannot reach high enough.
    MaxTooLow,
}

/// One channel's calibration limits.
#[derive(Debug, Clone, Copy)]
pub struct RcChannelCalibration {
    /// `RCn_MIN`.
    pub radio_min: u16,
    /// `RCn_MAX`.
    pub radio_max: u16,
}

/// Both faults a single channel can have, upstream `rc_checks_copter_sub`'s
/// loop body.
///
/// # A channel can be wrong at both ends
///
/// Upstream tests the two independently and sets its failure flag from each,
/// so a channel that was never calibrated at all is reported twice — once for
/// each end. Returning at the first would tell the operator to fix the
/// minimum and let them discover the maximum on the next attempt.
#[must_use]
pub fn rc_channel_calibration_faults(
    channel: &RcChannelCalibration,
) -> [Option<RcCalibrationFault>; 2] {
    [
        (channel.radio_min > RC_CALIB_MIN_LIMIT_PWM).then_some(RcCalibrationFault::MinTooHigh),
        (channel.radio_max < RC_CALIB_MAX_LIMIT_PWM).then_some(RcCalibrationFault::MaxTooLow),
    ]
}

/// The four channels the calibration check covers, in upstream's order.
///
/// The order is the order the messages arrive in, which is the order an
/// operator works through them.
pub const RC_CALIBRATION_CHANNEL_NAMES: [&str; 4] = ["Roll", "Pitch", "Throttle", "Yaw"];

/// Whether every channel's calibration passes, upstream
/// `rc_checks_copter_sub`.
#[must_use]
pub fn rc_calibration_passes(rc_check_enabled: bool, channels: &[RcChannelCalibration; 4]) -> bool {
    if !rc_check_enabled {
        return true;
    }
    channels
        .iter()
        .all(|c| rc_channel_calibration_faults(c).iter().all(Option::is_none))
}

/// Combine the two halves of `rc_calibration_checks` the way upstream does.
///
/// # Bitwise, not logical
///
/// Upstream writes `rc_checks_copter_sub(...) & AP_Arming::rc_calibration_checks(...)`
/// with a bitwise `&` and a comment saying it "ensures all checks are run".
/// A logical `&&` would short-circuit, and a vehicle failing the first half
/// would never run the second — so the operator would fix the reported
/// problem, try again, and be told about a different one.
///
/// The two spellings are one character apart and behave identically as far as
/// the returned bool is concerned. The difference is entirely in the messages
/// the pilot gets, which is why this is a named function rather than an
/// operator at a call site.
#[must_use]
pub fn combine_rc_calibration(copter_half: bool, shared_half: bool) -> bool {
    copter_half & shared_half
}
