//! Land-detector leftover, upstream `ArduCopter/land_detector.cpp`.
//!
//! Tracked as **COP-021**. This is `Copter::update_land_detector` only — the
//! counter that decides whether the aircraft has been still, level, and at
//! minimum throttle long enough to raise `ap.land_complete`. The crash
//! check that shares `update_land_and_crash_detectors` is COP-019.
//! `set_land_complete`'s disarm-on-land side effects, LDET logging, and the
//! heli-frame collective path stay later leftovers.
//!
//! # The multi-rotor path, not the heli one
//!
//! Upstream compiles a different "at the lower limit" test for
//! `FRAME_CONFIG == HELI_FRAME`. This leftover is the `#else`: throttle
//! lower-limit plus mix-min, with the airmode override that lengthens the
//! trigger to three seconds and forces mix-min true. A heli build must not
//! use this result.
//!
//! # `set_land_complete` zeroes the counter only on a real change
//!
//! The flag assignment is not a write. If the new value equals the old one
//! the function returns immediately and `land_detector_count` is left
//! alone. Only a *transition* — flying to landed, or the unexpected
//! takeoff the other way — resets the counter. A port that zeroed on every
//! "set true" would wipe a count the disarmed path is entitled to keep
//! when the vehicle was already down.
//!
//! # Missing vertical velocity is treated as zero, not as "unknown"
//!
//! `ahrs.get_velocity_D` failing leaves `vel_d_ms` at the 0.0 it was
//! initialised to (`UNUSED_RESULT`). That *passes* the descent-rate check.
//! Inventing a veto on a failed read would refuse to declare landed just
//! when the EKF is most confused by ground effect — the opposite of what
//! the comment at the top of the function is willing to do.
//!
//! Logging, `INTERNAL_ERROR(flow_of_control)` on the unexpected-takeoff
//! arm, and `set_likely_flying` belong to the caller. This returns the
//! two flags and the counter.

use ap_motors::spool::SpoolState;

/// Seconds of settled criteria before `land_complete`, upstream
/// `LAND_DETECTOR_TRIGGER_SEC`.
pub const LAND_DETECTOR_TRIGGER_SEC: f32 = 1.0;

/// Seconds of settled criteria in airmode, upstream
/// `LAND_AIRMODE_DETECTOR_TRIGGER_SEC`.
pub const LAND_AIRMODE_DETECTOR_TRIGGER_SEC: f32 = 3.0;

/// Seconds that raise `land_complete_maybe`, upstream
/// `LAND_DETECTOR_MAYBE_TRIGGER_SEC`.
pub const LAND_DETECTOR_MAYBE_TRIGGER_SEC: f32 = 0.2;

/// Filtered earth-frame acceleration that still counts as stationary, m/s².
///
/// Upstream `LAND_DETECTOR_ACCEL_MAX`. Multiplied by the WoW scalar.
pub const LAND_DETECTOR_ACCEL_MAX: f32 = 1.0;

/// Vertical speed that still counts as settled, m/s.
///
/// Upstream `LAND_DETECTOR_VEL_Z_MAX`. Multiplied by the WoW scalar.
pub const LAND_DETECTOR_VEL_Z_MAX: f32 = 1.0;

/// Rangefinder altitude below which landing is allowed, metres.
///
/// Upstream `LAND_RANGEFINDER_MIN_ALT_M`. A healthy rangefinder above this
/// refuses the detection; an unhealthy one is ignored.
pub const LAND_RANGEFINDER_MIN_ALT_M: f32 = 2.0;

/// Attitude-error threshold, degrees. Above this we are not landing.
///
/// Upstream `LAND_CHECK_ANGLE_ERROR_DEG`.
pub const LAND_CHECK_ANGLE_ERROR_DEG: f32 = 30.0;

/// Target lean that counts as an aggressive request, radians.
///
/// Upstream `LAND_CHECK_LARGE_ANGLE_RAD` (`radians(15)`). Compared on
/// squared length of the roll/pitch target, so a 15° roll with zero pitch
/// is exactly the threshold and is *not* large (`>` not `>=`).
pub const LAND_CHECK_LARGE_ANGLE_RAD: f32 = 15.0_f32.to_radians();

/// Weight-on-wheels state the detector reads.
///
/// Upstream `AP_LandingGear::get_wow_state`, plus the `#else` when the
/// library is compiled out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WowState {
    /// `AP_LANDINGGEAR_ENABLED` off. The check is skipped (passes) and the
    /// scalar stays 1.
    Disabled,
    /// Sensor present but `LG_WOW_UNKNOWN`. Scalar stays 1; the check
    /// passes — unknown is treated as "never no WoW".
    Unknown,
    /// Weight on wheels. Scalar 2 (looser accel/speed); the check passes.
    Wow,
    /// Weight off wheels. Scalar 2; the check *fails* — we cannot be
    /// landed with the wheels in the air.
    NoWow,
}

impl WowState {
    /// Whether a real sensor reading is available, so the accel/speed
    /// thresholds are doubled.
    #[must_use]
    pub const fn sensor_known(self) -> bool {
        matches!(self, Self::Wow | Self::NoWow)
    }

    /// Upstream `WoW_check`. Disabled and unknown both pass.
    #[must_use]
    pub const fn check(self) -> bool {
        !matches!(self, Self::NoWow)
    }
}

/// Inputs for `Copter::update_land_detector`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandDetectorInputs {
    /// `motors->armed()`.
    pub armed: bool,
    /// `ap.land_complete` from the previous iteration.
    pub land_complete: bool,
    /// `flightmode->is_taking_off()`.
    pub is_taking_off: bool,
    /// `standby_active`.
    pub standby_active: bool,
    /// `motors->get_throttle_out()`.
    pub throttle_out: f32,
    /// `motors->get_throttle_hover()`, for [`non_takeoff_throttle`].
    pub throttle_hover: f32,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `flightmode->has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `air_mode == AirMode::AIRMODE_ENABLED`.
    pub airmode_enabled: bool,
    /// `motors->limit.throttle_lower`.
    pub motor_at_lower_limit: bool,
    /// `attitude_control->is_throttle_mix_min()`.
    ///
    /// Forced true when airmode is on in a manual-throttle mode.
    pub throttle_mix_at_min: bool,
    /// `attitude_control->get_att_target_euler_rad().x`.
    pub target_roll_rad: f32,
    /// `attitude_control->get_att_target_euler_rad().y`.
    pub target_pitch_rad: f32,
    /// `attitude_control->get_att_error_angle_deg()`.
    pub att_error_angle_deg: f32,
    /// `land_accel_ef_filter.get().length()`, m/s².
    pub filtered_accel_ms2: f32,
    /// Downward velocity, m/s. Pass `0.0` when `get_velocity_D` failed.
    pub vel_d_ms: f32,
    /// `rangefinder_alt_ok()`.
    pub rangefinder_alt_ok: bool,
    /// `rangefinder_state.alt_m_filt.get()`. Ignored when not ok.
    pub rangefinder_alt_m: f32,
    /// Landing-gear WoW. See [`WowState`].
    pub wow: WowState,
    /// Static `land_detector_count` from the previous iteration.
    pub land_detector_count: u32,
    /// `scheduler.get_loop_rate_hz()`.
    pub loop_rate_hz: u16,
}

impl Default for LandDetectorInputs {
    fn default() -> Self {
        // Armed, airborne, every landing criterion met — the case that
        // *would* accumulate toward `land_complete`.
        Self {
            armed: true,
            land_complete: false,
            is_taking_off: false,
            standby_active: false,
            throttle_out: 0.0,
            throttle_hover: 0.5,
            spool_state: SpoolState::GroundIdle,
            has_manual_throttle: false,
            airmode_enabled: false,
            motor_at_lower_limit: true,
            throttle_mix_at_min: true,
            target_roll_rad: 0.0,
            target_pitch_rad: 0.0,
            att_error_angle_deg: 0.0,
            filtered_accel_ms2: 0.0,
            vel_d_ms: 0.0,
            rangefinder_alt_ok: false,
            rangefinder_alt_m: 10.0,
            wow: WowState::Disabled,
            land_detector_count: 0,
            loop_rate_hz: 400,
        }
    }
}

/// What `Copter::update_land_detector` stored this iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandDetectorUpdate {
    /// New `ap.land_complete`.
    pub land_complete: bool,
    /// New `ap.land_complete_maybe`.
    pub land_complete_maybe: bool,
    /// New `land_detector_count`.
    pub count: u32,
    /// The landed-plus-high-throttle arm fired. Caller logs
    /// `INTERNAL_ERROR(flow_of_control)`.
    pub unexpected_takeoff: bool,
}

/// Throttle that should not lift off, upstream
/// `Copter::get_non_takeoff_throttle` in `Attitude.cpp`.
///
/// Half hover, floored at zero. The land detector uses this as the
/// "throttle is high, so we cannot still be landed" threshold — strictly
/// greater than, so sitting exactly on the value stays landed.
#[must_use]
pub fn non_takeoff_throttle(throttle_hover: f32) -> f32 {
    libm::fmaxf(0.0, throttle_hover / 2.0)
}

/// Whether the roll/pitch *target* is an aggressive request, upstream
/// `large_angle_request`.
///
/// Compared on squared length against [`LAND_CHECK_LARGE_ANGLE_RAD`]
/// squared. Equality is not large.
#[must_use]
pub fn large_angle_request(target_roll_rad: f32, target_pitch_rad: f32) -> bool {
    let len_sq = target_roll_rad * target_roll_rad + target_pitch_rad * target_pitch_rad;
    len_sq > LAND_CHECK_LARGE_ANGLE_RAD * LAND_CHECK_LARGE_ANGLE_RAD
}

/// Iterations that raise `land_complete_maybe`.
///
/// Upstream `LAND_DETECTOR_MAYBE_TRIGGER_SEC * scheduler.get_loop_rate_hz()`.
/// The uint32 count is compared against this float (`>=`).
#[must_use]
pub fn maybe_trigger_iterations(loop_rate_hz: u16) -> f32 {
    LAND_DETECTOR_MAYBE_TRIGGER_SEC * f32::from(loop_rate_hz)
}

/// Iterations that raise `land_complete`.
///
/// `airmode` here means the leftover's own condition: manual throttle
/// *and* airmode enabled. Either false keeps the one-second trigger.
#[must_use]
pub fn land_trigger_iterations(airmode: bool, loop_rate_hz: u16) -> f32 {
    let sec = if airmode {
        LAND_AIRMODE_DETECTOR_TRIGGER_SEC
    } else {
        LAND_DETECTOR_TRIGGER_SEC
    };
    sec * f32::from(loop_rate_hz)
}

/// `set_land_complete`: write the flag, and zero the count only when it
/// actually changed.
#[must_use]
const fn apply_land_complete(current: bool, next: bool, count: u32) -> (bool, u32) {
    if current == next {
        (current, count)
    } else {
        (next, 0)
    }
}

#[must_use]
fn land_complete_maybe(land_complete: bool, count: u32, loop_rate_hz: u16) -> bool {
    if land_complete {
        return true;
    }
    // Upstream: `uint32_t >= float`. The same promotion.
    #[allow(
        clippy::cast_precision_loss,
        reason = "reproduces land_detector_count >= MAYBE_TRIGGER_SEC * rate_hz"
    )]
    let count_f = count as f32;
    count_f >= maybe_trigger_iterations(loop_rate_hz)
}

/// Land-detector leftover, upstream `Copter::update_land_detector`.
#[must_use]
pub fn update_land_detector(inp: &LandDetectorInputs) -> LandDetectorUpdate {
    if !inp.armed {
        let (land_complete, count) =
            apply_land_complete(inp.land_complete, true, inp.land_detector_count);
        return finish(land_complete, count, inp.loop_rate_hz, false);
    }

    if inp.land_complete {
        let high_throttle = inp.throttle_out > non_takeoff_throttle(inp.throttle_hover);
        if !inp.is_taking_off && high_throttle && inp.spool_state == SpoolState::ThrottleUnlimited {
            let (land_complete, count) = apply_land_complete(true, false, inp.land_detector_count);
            return finish(land_complete, count, inp.loop_rate_hz, true);
        }
        return finish(true, inp.land_detector_count, inp.loop_rate_hz, false);
    }

    if inp.standby_active {
        return finish(false, 0, inp.loop_rate_hz, false);
    }

    let airmode = inp.has_manual_throttle && inp.airmode_enabled;
    let throttle_mix_at_min = airmode || inp.throttle_mix_at_min;
    let scalar = if inp.wow.sensor_known() { 2.0 } else { 1.0 };

    let accel_stationary = inp.filtered_accel_ms2 <= LAND_DETECTOR_ACCEL_MAX * scalar;
    let descent_rate_low = libm::fabsf(inp.vel_d_ms) < LAND_DETECTOR_VEL_Z_MAX * scalar;
    let rangefinder_check =
        !inp.rangefinder_alt_ok || inp.rangefinder_alt_m < LAND_RANGEFINDER_MIN_ALT_M;
    let large_error = inp.att_error_angle_deg > LAND_CHECK_ANGLE_ERROR_DEG;

    let settled = inp.motor_at_lower_limit
        && throttle_mix_at_min
        && !large_angle_request(inp.target_roll_rad, inp.target_pitch_rad)
        && !large_error
        && accel_stationary
        && descent_rate_low
        && rangefinder_check
        && inp.wow.check();

    if !settled {
        return finish(false, 0, inp.loop_rate_hz, false);
    }

    let trigger = land_trigger_iterations(airmode, inp.loop_rate_hz);
    #[allow(
        clippy::cast_precision_loss,
        reason = "reproduces land_detector_count < land_trigger_sec * rate_hz"
    )]
    let count_f = inp.land_detector_count as f32;
    if count_f < trigger {
        finish(
            false,
            inp.land_detector_count.wrapping_add(1),
            inp.loop_rate_hz,
            false,
        )
    } else {
        let (land_complete, count) = apply_land_complete(false, true, inp.land_detector_count);
        finish(land_complete, count, inp.loop_rate_hz, false)
    }
}

fn finish(
    land_complete: bool,
    count: u32,
    loop_rate_hz: u16,
    unexpected_takeoff: bool,
) -> LandDetectorUpdate {
    LandDetectorUpdate {
        land_complete,
        land_complete_maybe: land_complete_maybe(land_complete, count, loop_rate_hz),
        count,
        unexpected_takeoff,
    }
}
