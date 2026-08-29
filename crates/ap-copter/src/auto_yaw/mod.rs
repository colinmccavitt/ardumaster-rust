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

/// Minimum ground speed before the aircraft is aimed at its ground course,
/// upstream `YAW_LOOK_AHEAD_MIN_SPEED_MS` (`config.h:459`).
///
/// Below it the velocity vector's direction is noise — a hovering aircraft
/// drifting a few centimetres a second has a well-defined heading only in the
/// arithmetic sense — so the last good heading is kept instead.
pub const YAW_LOOK_AHEAD_MIN_SPEED_MS: f32 = 1.0;

/// The look-ahead heading, upstream `Mode::AutoYaw::look_ahead_yaw_rad`.
///
/// Returns the updated heading. `held` is the previous value, which is what
/// comes back unchanged when the aircraft is too slow or has no position —
/// note it is *held*, not zeroed, so a brief slowdown does not swing the nose
/// to north and back.
///
/// The threshold is compared on squared speed against the squared constant,
/// avoiding a square root, so a port comparing `length()` against the
/// constant would agree except where the square root rounds across the
/// boundary.
///
/// # A mutant that cannot be killed
///
/// The mutation gate reports `MIN_SPEED * MIN_SPEED` replaced by
/// `MIN_SPEED / MIN_SPEED` as untested. The constant is 1.0, so the two
/// expressions are the same number and no input distinguishes them. Written
/// as the square anyway, because that is what upstream means and what would
/// still be right if the constant ever moved.
#[must_use]
pub fn look_ahead_yaw_rad(held: f32, position_ok: bool, vel_n_ms: f32, vel_e_ms: f32) -> f32 {
    if !position_ok {
        return held;
    }
    let speed_sq = vel_n_ms * vel_n_ms + vel_e_ms * vel_e_ms;
    if speed_sq > YAW_LOOK_AHEAD_MIN_SPEED_MS * YAW_LOOK_AHEAD_MIN_SPEED_MS {
        return libm::atan2f(vel_e_ms, vel_n_ms);
    }
    held
}

/// How a fixed-yaw command should be interpreted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedYawDirection {
    /// Turn whichever way is shorter.
    Shortest,
    /// Turn clockwise, going the long way if necessary.
    Clockwise,
    /// Turn counter-clockwise, going the long way if necessary.
    CounterClockwise,
}

impl FixedYawDirection {
    /// Upstream passes this as an `int8_t` where the sign is what matters.
    #[must_use]
    pub fn from_sign(direction: i8) -> Self {
        if direction < 0 {
            Self::CounterClockwise
        } else if direction > 0 {
            Self::Clockwise
        } else {
            Self::Shortest
        }
    }
}

/// The offset a fixed-yaw command should slew through, upstream the first
/// half of `Mode::AutoYaw::set_fixed_yaw_rad`.
///
/// # Relative and absolute are different commands
///
/// A relative command is already an offset, so the direction only chooses its
/// sign — and note that `direction >= 0` takes the positive branch, so
/// "shortest" and "clockwise" are the same thing for a relative command.
/// There is nothing to be shortest about when the caller has said how far to
/// turn.
///
/// An absolute command names a heading, so the offset is the difference,
/// wrapped to the shorter way round. The direction then *overrides* that:
/// asking for counter-clockwise when the short way is clockwise subtracts a
/// full turn, so the aircraft goes the long way deliberately. That is what
/// makes `CONDITION_YAW` able to command three quarters of a turn one way
/// rather than a quarter the other, which matters when a camera is tracking
/// something on the way round.
///
/// `current_yaw_rad` is the *target* angle the machine is holding, not the
/// aircraft's measured heading — except that a relative command entering from
/// `HOLD` seeds it from the measurement first, because in `HOLD` there is no
/// meaningful target to be relative to.
#[must_use]
pub fn fixed_yaw_offset_rad(
    angle_rad: f32,
    current_yaw_rad: f32,
    direction: FixedYawDirection,
    relative_angle: bool,
) -> f32 {
    if relative_angle {
        // `direction >= 0 ? 1.0 : -1.0` upstream: zero counts as positive.
        return if direction == FixedYawDirection::CounterClockwise {
            -angle_rad
        } else {
            angle_rad
        };
    }

    let mut offset = ap_math::scalar::wrap_pi(angle_rad - current_yaw_rad);
    match direction {
        FixedYawDirection::CounterClockwise if offset > 0.0 => {
            offset -= core::f32::consts::TAU;
        }
        FixedYawDirection::Clockwise if offset < 0.0 => {
            offset += core::f32::consts::TAU;
        }
        _ => {}
    }
    offset
}

/// The slew rate a fixed-yaw command should use, upstream the second half of
/// `set_fixed_yaw_rad`.
///
/// A non-positive request means "no preference", which takes the controller's
/// maximum. A positive one is capped by that maximum, so a caller cannot
/// command a turn faster than the attitude controller will actually fly.
#[must_use]
pub fn fixed_yaw_slew_rate_rads(requested_rads: f32, controller_max_rads: f32) -> f32 {
    if requested_rads <= 0.0 {
        return controller_max_rads;
    }
    libm::fminf(controller_max_rads, requested_rads)
}

/// Whether a fixed-yaw slew has arrived, upstream
/// `Mode::AutoYaw::reached_fixed_yaw_target`.
///
/// # Not being in FIXED reports arrival
///
/// Upstream returns true when the mode is not `FIXED`, with a comment saying
/// it should not happen. Reporting "arrived" for a question that does not
/// apply is the safe direction: a caller waiting on this is usually a mission
/// command waiting to advance, and blocking forever on a mode that will never
/// arrive would stall the mission.
///
/// # Two conditions, not one
///
/// The offset must be fully consumed *and* the aircraft must be within two
/// degrees. The first says the target has finished moving; the second says
/// the aircraft has caught up with it. A slew can be complete while the
/// airframe is still swinging towards the final heading.
#[must_use]
pub fn reached_fixed_yaw_target(
    mode: YawMode,
    fixed_yaw_offset_rad: f32,
    yaw_angle_rad: f32,
    measured_yaw_rad: f32,
) -> bool {
    if mode != YawMode::Fixed {
        return true;
    }
    if !ap_math::scalar::is_zero(fixed_yaw_offset_rad) {
        return false;
    }
    let error = ap_math::scalar::wrap_pi(yaw_angle_rad - measured_yaw_rad);
    libm::fabsf(error) <= 2.0_f32.to_radians()
}

/// The order `set_rate_rad` does its two things in, upstream
/// `Mode::AutoYaw::set_rate_rad`.
///
/// # The order is the whole function
///
/// Upstream calls `set_mode(RATE)` *first* and assigns the rate *second*.
/// That is not stylistic: entering `RATE` zeroes the stored rate (see
/// [`yaw_mode_entry`]), so assigning first and switching second would discard
/// the rate that was just commanded and leave the aircraft turning at zero.
///
/// This function exists to make the ordering something a caller cannot get
/// wrong.
///
/// # The mutation gate cannot see the point of it
///
/// It reports the `== Some(ZeroYawRate)` test as untested, and it is
/// untestable: the assignment below runs unconditionally, so whether the
/// zeroing happened is unobservable from outside. That is equally true of
/// upstream, where `set_mode` zeroes the rate and `set_rate_rad` immediately
/// overwrites it.
///
/// The defect this guards against is not a mutation of this code — it is a
/// caller writing the two statements the other way round, which no mutation
/// of the correct version can express. That is a limit of the gate rather
/// than a hole in the tests.
pub fn set_rate(current: YawMode, turn_rate_rads: f32, stored_rate: &mut f32) -> YawMode {
    // The mode change first, because it may zero the rate.
    if yaw_mode_entry(current, YawMode::Rate) == Some(YawModeEntry::ZeroYawRate) {
        *stored_rate = 0.0;
    }
    // Then the commanded rate, which must survive.
    *stored_rate = turn_rate_rads;
    YawMode::Rate
}

/// What a `set_roi` command does, upstream `Mode::AutoYaw::set_roi`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoiAction {
    /// The location was empty, so the region of interest is cancelled and the
    /// yaw mode returns to the operator's default.
    Cancel,
    /// Point the airframe at the location, because the mount cannot pan.
    PointAirframe,
    /// Leave the airframe alone: the mount can pan, so it tracks the target
    /// on its own and yawing the aircraft as well would be redundant.
    MountOnly,
}

/// Upstream `set_roi`'s decision.
///
/// # An uninitialised location means "stop", not "point at the equator"
///
/// A `Location` of all zeros is a real point off the coast of Africa, so
/// upstream tests `initialised()` rather than the coordinates. A mission that
/// clears its region of interest sends zeros, and reading that literally
/// would swing the aircraft to face a point thousands of kilometres away.
///
/// # A panning mount does not need the airframe
///
/// If the mount can pan, it tracks the target itself and the aircraft is left
/// to fly. Only a fixed mount makes the whole airframe the pointing
/// mechanism.
#[must_use]
pub fn roi_action(location_initialised: bool, mount_has_pan_control: bool) -> RoiAction {
    if !location_initialised {
        return RoiAction::Cancel;
    }
    if mount_has_pan_control {
        return RoiAction::MountOnly;
    }
    RoiAction::PointAirframe
}

/// The heading towards the region of interest, upstream
/// `Mode::AutoYaw::roi_yaw_rad`.
///
/// `position_ne_m` is the vehicle's position relative to the EKF origin, and
/// `roi_ne_m` the target's in the same frame — both north-east, so the
/// bearing is a plain `atan2` wrapped to a full turn by
/// [`get_bearing_rad`](ap_math::location::get_bearing_rad).
///
/// # No position means hold the target, not point north
///
/// When the position estimate is unavailable upstream returns the attitude
/// controller's *current target* yaw rather than zero or the measured
/// heading. Returning zero would swing the aircraft to north; returning the
/// measurement would let the target drift with every gust the airframe took.
/// Returning the standing target leaves the demand exactly where it was, so
/// a momentary loss of position produces no yaw at all.
#[must_use]
pub fn roi_yaw_rad(
    position_ne_m: Option<(f32, f32)>,
    roi_ne_m: (f32, f32),
    attitude_target_yaw_rad: f32,
) -> f32 {
    match position_ne_m {
        Some((n, e)) => ap_math::location::get_bearing_rad(
            ap_math::vector2::Vector2f::new(n, e),
            ap_math::vector2::Vector2f::new(roi_ne_m.0, roi_ne_m.1),
        ),
        None => attitude_target_yaw_rad,
    }
}

/// The state a yaw command leaves behind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawCommand {
    /// The target angle, radians.
    pub yaw_angle_rad: f32,
    /// The target rate, radians per second.
    pub yaw_rate_rads: f32,
    /// The mode to enter.
    pub mode: YawMode,
}

/// An absolute angle and rate, upstream
/// `Mode::AutoYaw::set_yaw_angle_and_rate_rad`.
///
/// Both are taken as given — this is the `SET_POSITION_TARGET` path, where a
/// companion computer has said exactly what it wants and there is nothing to
/// derive.
///
/// Note the mode is `ANGLE_RATE`, whose entry initialises nothing, so unlike
/// [`set_rate`] the order of the assignments does not matter here. That is
/// worth knowing precisely because the two functions look parallel and only
/// one of them is order-sensitive.
#[must_use]
pub fn set_yaw_angle_and_rate(yaw_angle_rad: f32, yaw_rate_rads: f32) -> YawCommand {
    YawCommand {
        yaw_angle_rad,
        yaw_rate_rads,
        mode: YawMode::AngleRate,
    }
}

/// A relative angle change, upstream `Mode::AutoYaw::set_yaw_angle_offset_deg`.
///
/// # Wrapped to a full turn, not to a half
///
/// The new angle is `wrap_2PI(current + offset)`, so it lands in 0..2π rather
/// than −π..π. That differs from the fixed-yaw path, which wraps to ±π — and
/// the difference is not cosmetic, because these two produce the same
/// physical heading with different numbers, and anything comparing the target
/// against a −π..π measurement has to account for it.
///
/// # The rate is zeroed
///
/// An offset command says where to end up, not how fast. Leaving a previously
/// commanded rate in place would have the aircraft sail past the new target,
/// since `ANGLE_RATE` integrates the rate into the angle every iteration.
#[must_use]
pub fn set_yaw_angle_offset(current_yaw_angle_rad: f32, offset_deg: f32) -> YawCommand {
    YawCommand {
        yaw_angle_rad: ap_math::scalar::wrap_2pi(current_yaw_angle_rad + offset_deg.to_radians()),
        yaw_rate_rads: 0.0,
        mode: YawMode::AngleRate,
    }
}

mod leftover;
pub use leftover::{
    get_heading, set_mode, set_mode_to_default, GetHeadingContext, GetHeadingLeftover,
    SetModeLeftover, YawAngleSource,
};
