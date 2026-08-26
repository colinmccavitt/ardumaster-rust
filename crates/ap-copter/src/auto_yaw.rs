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
