//! Where an autonomous mode points the nose, upstream `ArduCopter/autoyaw.cpp`.
//!
//! Every autonomous Copter mode delegates yaw to this. It is a small state
//! machine with eleven states, and almost all of its content is in which
//! state a given situation selects and what each one contributes.

/// How the yaw target is being decided.
///
/// Upstream `Mode::AutoYaw::Mode`. The numbers reach `DO_CONDITIONAL_YAW`
/// handling and the logs, so they are pinned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawMode {
    /// 0 — hold zero yaw rate. The nose goes wherever the aircraft's momentum
    /// leaves it.
    Hold,
    /// 1 — point towards the next waypoint. No pilot input accepted.
    LookAtNextWp,
    /// 2 — point at a region of interest. No pilot input accepted.
    Roi,
    /// 3 — point at a particular angle. No pilot input accepted.
    Fixed,
    /// 4 — point the way the aircraft is moving.
    LookAhead,
    /// 5 — point where the aircraft was facing when it armed.
    ResetToArmedYaw,
    /// 6 — turn at a rate from a starting angle.
    AngleRate,
    /// 7 — turn at a rate.
    Rate,
    /// 8 — take the yaw AC_Circle provides, during Loiter-Turns.
    Circle,
    /// 9 — turn at the rate the pilot's stick asks for.
    PilotRate,
    /// 10 — yaw into wind.
    Weathervane,
}

impl YawMode {
    /// Upstream's number for this mode.
    #[must_use]
    pub fn as_number(self) -> u8 {
        match self {
            Self::Hold => 0,
            Self::LookAtNextWp => 1,
            Self::Roi => 2,
            Self::Fixed => 3,
            Self::LookAhead => 4,
            Self::ResetToArmedYaw => 5,
            Self::AngleRate => 6,
            Self::Rate => 7,
            Self::Circle => 8,
            Self::PilotRate => 9,
            Self::Weathervane => 10,
        }
    }

    /// The mode a number denotes, or `None` if none does.
    #[must_use]
    pub fn from_number(number: u8) -> Option<Self> {
        Some(match number {
            0 => Self::Hold,
            1 => Self::LookAtNextWp,
            2 => Self::Roi,
            3 => Self::Fixed,
            4 => Self::LookAhead,
            5 => Self::ResetToArmedYaw,
            6 => Self::AngleRate,
            7 => Self::Rate,
            8 => Self::Circle,
            9 => Self::PilotRate,
            10 => Self::Weathervane,
            _ => return None,
        })
    }

    /// Whether this mode accepts the pilot's yaw stick.
    ///
    /// Upstream expresses this as comments on the enum — "no pilot input
    /// accepted" against ROI, FIXED and LOOK_AT_NEXT_WP — rather than as a
    /// predicate, because the exclusion is enforced by `get_heading` choosing
    /// not to switch out of them rather than by any check here.
    #[must_use]
    pub fn is_pilot_excluded(self) -> bool {
        matches!(self, Self::LookAtNextWp | Self::Roi | Self::Fixed)
    }
}

/// `WP_YAW_BEHAVIOR`, the operator's standing preference for where the nose
/// points during a mission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WpYawBehaviour {
    /// 0 — never control yaw during missions or RTL, except when a
    /// `DO_CONDITIONAL_YAW` command asks.
    Never,
    /// 1 — face the next waypoint, and home during RTL.
    LookAtNextWp,
    /// 2 — face the next waypoint, except during RTL where the last heading
    /// is kept.
    LookAtNextWpExceptRtl,
    /// 3 — look the way the aircraft is going. Meant for traditional
    /// helicopters, which do not like being yawed while translating.
    LookAhead,
}

impl WpYawBehaviour {
    /// The behaviour a stored parameter value denotes.
    ///
    /// # Out of range means "look at the next waypoint"
    ///
    /// Upstream's switch has `WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP` and `default`
    /// sharing one arm, so any unrecognised value behaves as 1 rather than
    /// being refused. That is the safer default of the four — an aircraft
    /// facing where it is going is predictable — but it does mean a
    /// misconfigured parameter is silently interpreted rather than reported.
    #[must_use]
    pub fn from_number(number: u8) -> Self {
        match number {
            0 => Self::Never,
            2 => Self::LookAtNextWpExceptRtl,
            3 => Self::LookAhead,
            // 1 and everything else.
            _ => Self::LookAtNextWp,
        }
    }
}

/// The yaw mode a mission should start in, upstream
/// `Mode::AutoYaw::default_mode`.
///
/// `rtl` distinguishes a return from an ordinary mission leg, and only one
/// behaviour treats them differently: `LookAtNextWpExceptRtl` holds the
/// current heading on the way home. The reasoning is that on RTL the "next
/// waypoint" is home, and yawing to face it tells the operator nothing they
/// want while costing them the camera pointing wherever they left it.
#[must_use]
pub fn default_yaw_mode(behaviour: WpYawBehaviour, rtl: bool) -> YawMode {
    match behaviour {
        WpYawBehaviour::Never => YawMode::Hold,
        WpYawBehaviour::LookAtNextWpExceptRtl => {
            if rtl {
                YawMode::Hold
            } else {
                YawMode::LookAtNextWp
            }
        }
        WpYawBehaviour::LookAhead => YawMode::LookAhead,
        WpYawBehaviour::LookAtNextWp => YawMode::LookAtNextWp,
    }
}

/// What entering a yaw mode has to initialise.
///
/// Most modes need nothing: their target is computed fresh each iteration, or
/// set by whoever asked for the mode. Only two carry state into the mode from
/// the moment of entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawModeEntry {
    /// Nothing to do.
    Nothing,
    /// Seed the look-ahead heading from the current attitude, so the aircraft
    /// does not swing while the estimate settles.
    SeedLookAheadFromCurrentYaw,
    /// Zero the target rate. Entering a rate mode with a stale rate would
    /// start the aircraft turning at whatever the last rate command was.
    ZeroYawRate,
}

/// What a transition into `new_mode` requires, upstream the switch in
/// `Mode::AutoYaw::set_mode`.
///
/// Returns `None` when the mode is unchanged: upstream returns immediately in
/// that case, which matters because it means re-selecting the mode you are
/// already in does *not* re-run the initialisation. Asking for `Rate` twice
/// leaves the current rate alone rather than zeroing it.
#[must_use]
pub fn yaw_mode_entry(current: YawMode, new_mode: YawMode) -> Option<YawModeEntry> {
    if current == new_mode {
        return None;
    }

    Some(match new_mode {
        YawMode::LookAhead => YawModeEntry::SeedLookAheadFromCurrentYaw,
        YawMode::Rate => YawModeEntry::ZeroYawRate,
        // HOLD, LOOK_AT_NEXT_WP (wpnav initialises the heading when its
        // destination is set), ROI, FIXED (the caller sets the angle),
        // RESET_TO_ARMED_YAW (the bearing is captured at arming), ANGLE_RATE,
        // CIRCLE, PILOT_RATE and WEATHERVANE all need nothing.
        _ => YawModeEntry::Nothing,
    })
}

/// Where the yaw *rate* comes from in each mode, upstream
/// `Mode::AutoYaw::rate_rads`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawRateSource {
    /// The rate is forced to zero. These modes hold an angle, and a non-zero
    /// rate alongside it would fight the angle controller.
    Zero,
    /// Whatever the position controller is turning at, so the nose follows
    /// the track rather than leading or lagging it.
    PositionController,
    /// The pilot's stick.
    Pilot,
    /// Left as it is. The rate was set by whoever commanded the mode and
    /// nothing here should overwrite it.
    Unchanged,
}

/// Upstream `Mode::AutoYaw::rate_rads`' choice.
///
/// # `Unchanged` is not the same as zero
///
/// `ANGLE_RATE`, `RATE` and `WEATHERVANE` fall through the switch without
/// assigning, so the stored rate survives. That is the point: a
/// `DO_CONDITIONAL_YAW` command sets a rate and this function must not
/// discard it on the next iteration. Reading those three as "no case, so
/// zero" would stop every commanded yaw turn dead.
#[must_use]
pub fn yaw_rate_source(mode: YawMode) -> YawRateSource {
    match mode {
        YawMode::Hold
        | YawMode::Roi
        | YawMode::Fixed
        | YawMode::LookAhead
        | YawMode::ResetToArmedYaw
        | YawMode::Circle => YawRateSource::Zero,
        YawMode::LookAtNextWp => YawRateSource::PositionController,
        YawMode::PilotRate => YawRateSource::Pilot,
        YawMode::AngleRate | YawMode::Rate | YawMode::Weathervane => YawRateSource::Unchanged,
    }
}

/// How the attitude controller should read the heading command.
///
/// Upstream `AC_AttitudeControl::HeadingMode`. `Angle_Only` exists in the
/// enum but `get_heading` never selects it — every yaw mode is either a rate
/// or both — so it is here for completeness of the type rather than because
/// this module produces it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadingMode {
    /// 0 — an angle with no rate.
    AngleOnly,
    /// 1 — an angle and the rate to approach it at.
    AngleAndRate,
    /// 2 — a rate alone, with no angle to hold.
    RateOnly,
}

/// Which kind of heading command a yaw mode produces, upstream the switch at
/// the end of `Mode::AutoYaw::get_heading`.
///
/// The split is between modes that know where they want the nose pointed and
/// modes that only know how fast they want it moving. Handing an angle the
/// caller does not have would make the attitude controller chase a stale
/// target; handing none where there is one would throw away the only thing
/// that stops drift.
#[must_use]
pub fn heading_mode(mode: YawMode) -> HeadingMode {
    match mode {
        YawMode::Hold | YawMode::Rate | YawMode::PilotRate | YawMode::Weathervane => {
            HeadingMode::RateOnly
        }
        YawMode::LookAtNextWp
        | YawMode::Roi
        | YawMode::Fixed
        | YawMode::LookAhead
        | YawMode::ResetToArmedYaw
        | YawMode::AngleRate
        | YawMode::Circle => HeadingMode::AngleAndRate,
    }
}

/// What the pilot's yaw stick does to the yaw mode, upstream the top of
/// `Mode::AutoYaw::get_heading`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PilotYawOverride {
    /// Leave the mode alone.
    None,
    /// The pilot is asking for yaw, so hand them the axis.
    TakeControl,
    /// The pilot had the axis and can no longer have it. Upstream goes to
    /// `HOLD` rather than back to whatever preceded `PILOT_RATE`.
    ReleaseToHold,
}

/// Whether the pilot takes or loses the yaw axis this iteration.
///
/// # Losing it goes to HOLD, not back
///
/// When the radio fails or the mode stops accepting pilot yaw, upstream sets
/// `HOLD` rather than restoring the mode that was running before the pilot
/// took over. That is deliberate and worth not "improving": the previous mode
/// may have been pointing at something the aircraft can no longer see, and
/// holding zero rate is the one answer that is safe without knowing why the
/// pilot's input went away.
///
/// # Any non-zero stick takes control
///
/// The test is `!is_zero(rate)`, not a deadzone — the deadzone has already
/// been applied by `get_pilot_desired_yaw_rate_rads`, so a rate that arrives
/// here at all is one the pilot meant.
#[must_use]
pub fn pilot_yaw_override(
    current: YawMode,
    has_valid_input: bool,
    mode_uses_pilot_yaw: bool,
    pilot_yaw_rate_rads: f32,
) -> PilotYawOverride {
    if has_valid_input && mode_uses_pilot_yaw {
        if !ap_math::scalar::is_zero(pilot_yaw_rate_rads) {
            return PilotYawOverride::TakeControl;
        }
        return PilotYawOverride::None;
    }

    if current == YawMode::PilotRate {
        return PilotYawOverride::ReleaseToHold;
    }
    PilotYawOverride::None
}

/// What the weathervane controller decided this iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WeathervaneAction {
    /// Nothing to do: weathervaning is not active and was not active.
    None,
    /// Take the yaw axis and turn at the rate the controller asked for.
    Engage,
    /// Give the axis back to the mode that had it before.
    ReleaseTo(YawMode),
    /// Give the axis back, but to the default mode rather than to what was
    /// recorded. See [`weathervane_action`].
    ReleaseToDefault,
}

/// Whether weathervaning takes or releases the yaw axis, upstream
/// `Mode::AutoYaw::update_weathervane`.
///
/// # Releasing to `HOLD` means releasing to the default instead
///
/// Upstream restores `_last_mode` unless that was `HOLD`, in which case it
/// calls `set_mode_to_default(false)`. The asymmetry looks arbitrary and is
/// not: `HOLD` is what the pilot-override path leaves behind when it takes the
/// axis away, so a recorded `HOLD` usually means "nothing chose this", and
/// restoring it would strand the aircraft holding zero rate for the rest of
/// the mission. Consulting `WP_YAW_BEHAVIOR` instead gives the operator's
/// standing preference, which is the best available answer to "what should
/// the nose do now".
///
/// Note the `rtl` argument to that default is hard-coded false, even if the
/// aircraft is in fact returning. Reproduced as written.
#[must_use]
pub fn weathervane_action(
    current: YawMode,
    last_mode: YawMode,
    allows_weathervaning: bool,
    controller_wants_yaw: bool,
) -> WeathervaneAction {
    if allows_weathervaning && controller_wants_yaw {
        return WeathervaneAction::Engage;
    }

    if current == YawMode::Weathervane {
        if last_mode == YawMode::Hold {
            return WeathervaneAction::ReleaseToDefault;
        }
        return WeathervaneAction::ReleaseTo(last_mode);
    }

    WeathervaneAction::None
}

/// The fixed-yaw slew, upstream the `FIXED` arm of `Mode::AutoYaw::yaw_rad`.
///
/// Returns the new `(yaw_angle_rad, remaining_offset_rad)`.
///
/// # The offset is consumed, not tracked
///
/// A fixed-yaw command arrives as an *offset* to fly through, and each
/// iteration takes as much of it as the slew rate allows and subtracts that
/// from what remains. So the target walks towards the commanded heading at a
/// bounded rate and stops when the offset reaches zero — no separate
/// "finished" flag is needed, and an interrupted slew simply resumes from
/// wherever it got to.
///
/// The step is constrained symmetrically, so a negative offset slews the
/// other way at the same rate.
#[must_use]
pub fn fixed_yaw_step(
    yaw_angle_rad: f32,
    offset_rad: f32,
    slew_rate_rads: f32,
    dt_s: f32,
) -> (f32, f32) {
    let limit = dt_s * slew_rate_rads;
    let step = ap_math::scalar::constrain_value(offset_rad, -limit, limit);
    (yaw_angle_rad + step, offset_rad - step)
}

/// The angle-rate integration, upstream the `ANGLE_RATE` arm of
/// `Mode::AutoYaw::yaw_rad`.
///
/// Plain rectangular integration of the commanded rate. Unlike the fixed
/// slew there is no target to converge on: the caller commanded a rate from a
/// starting angle and this advances the angle for as long as the mode lasts.
#[must_use]
pub fn angle_rate_step(yaw_angle_rad: f32, yaw_rate_rads: f32, dt_s: f32) -> f32 {
    yaw_angle_rad + yaw_rate_rads * dt_s
}
