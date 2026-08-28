//! Crash-detection leftover, upstream `ArduCopter/crash_check.cpp`.
//!
//! Tracked as **COP-019**. This is `Copter::crash_check` only — the counter
//! that decides whether the aircraft has been inverted and out of control
//! long enough to disarm. Thrust-loss, yaw-imbalance, and parachute checks
//! in the same file stay later leftovers.
//!
//! # Every gate resets the counter
//!
//! The function is a ladder of "not crashed" excuses. Each one that fires
//! zeroes `crash_counter` and returns. That reset *is* the content: a
//! vehicle that leans past 15° for a second, then levels, then leans again
//! does not inherit the first second. A port that only skipped the increment
//! would let two separate wobbles add up to a disarm.
//!
//! # Missing velocity is not a veto
//!
//! `ahrs.get_velocity_NED` failing does **not** clear the counter. Only a
//! successful read whose length is at or above
//! [`CRASH_CHECK_SPEED_MAX_MS`] does. An EKF that has no velocity is treated
//! as still possibly crashed; inventing "no speed, so not crashed" would
//! invert that.
//!
//! # Force-flying is not a blanket exemption
//!
//! `get_force_flying()` clears the counter only while the current mode is
//! *not* landing. A landing under force-flying still runs the check, because
//! that is when an inverted, out-of-control airframe is most likely to be
//! a crash rather than a commanded attitude.
//!
//! Logging, the GCS emergency string, and `arming.disarm(Method::CRASH)`
//! belong to the caller. This returns whether that disarm should happen.

/// Seconds of continuous crash conditions before disarm, upstream
/// `CRASH_CHECK_TRIGGER_SEC`.
pub const CRASH_CHECK_TRIGGER_SEC: u16 = 2;

/// Attitude-error threshold, degrees. Above this we may be out of control.
///
/// Upstream `CRASH_CHECK_ANGLE_DEVIATION_DEG`.
pub const CRASH_CHECK_ANGLE_DEVIATION_DEG: f32 = 30.0;

/// Minimum lean angle, degrees. Below this the vehicle is not "inverted
/// enough" to count as a crash.
///
/// Upstream `CRASH_CHECK_ANGLE_MIN_DEG`.
pub const CRASH_CHECK_ANGLE_MIN_DEG: f32 = 15.0;

/// Speed at or above which the vehicle is still flying, m/s.
///
/// Upstream `CRASH_CHECK_SPEED_MAX`.
pub const CRASH_CHECK_SPEED_MAX_MS: f32 = 10.0;

/// Filtered earth-frame acceleration at or above which we are not crashed.
///
/// Upstream `CRASH_CHECK_ACCEL_MAX`. The 1 G on the Z-axis has already been
/// subtracted by `land_accel_ef_filter`.
pub const CRASH_CHECK_ACCEL_MAX_MS2: f32 = 3.0;

/// Default `FS_CRASH_CHECK` — enabled.
pub const FS_CRASH_CHECK_DEFAULT: u8 = 1;

/// Lean angle used by the crash check.
///
/// Upstream `degrees(acosf(ahrs.cos_roll() * ahrs.cos_pitch()))`. The
/// product is the Z-component of the body-to-earth DCM, so the angle is
/// how far the thrust axis is from vertical — not roll or pitch alone.
#[must_use]
pub fn lean_angle_deg(cos_roll: f32, cos_pitch: f32) -> f32 {
    libm::acosf(cos_roll * cos_pitch).to_degrees()
}

/// Iterations of crash conditions needed to disarm.
///
/// Upstream `CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz()`.
#[must_use]
pub const fn crash_trigger_count(loop_rate_hz: u16) -> u32 {
    CRASH_CHECK_TRIGGER_SEC as u32 * loop_rate_hz as u32
}

/// Inputs for `Copter::crash_check`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrashCheckInputs {
    /// `motors->armed()`.
    pub armed: bool,
    /// `ap.land_complete`.
    pub land_complete: bool,
    /// `g.fs_crash_check`. Zero disables the check.
    pub fs_crash_check: u8,
    /// `standby_active`.
    pub standby_active: bool,
    /// `get_force_flying()`.
    pub force_flying: bool,
    /// `flightmode->is_landing()`.
    pub is_landing: bool,
    /// `flightmode->crash_check_enabled()`.
    ///
    /// False in Acro and Flip. The default on `Mode` is true.
    pub crash_check_enabled: bool,
    /// `flightmode->mode_number() == AUTOROTATE`.
    ///
    /// Upstream compiles this gate only when `MODE_AUTOROTATE_ENABLED`.
    /// A build without that mode passes `false`.
    pub in_autorotate: bool,
    /// `land_accel_ef_filter.get().length()`, m/s².
    pub filtered_accel_ms2: f32,
    /// Lean from vertical, degrees. See [`lean_angle_deg`].
    pub lean_angle_deg: f32,
    /// `attitude_control->get_att_error_angle_deg()`.
    pub att_error_angle_deg: f32,
    /// NED speed when `ahrs.get_velocity_NED` succeeded.
    ///
    /// `None` is not a reset. See the module docs.
    pub vel_ned_ms: Option<f32>,
    /// Static `crash_counter` from the previous iteration.
    pub crash_counter: u16,
    /// `scheduler.get_loop_rate_hz()`.
    pub loop_rate_hz: u16,
}

impl Default for CrashCheckInputs {
    fn default() -> Self {
        // Armed, airborne, crash-check on, leaning and out of control —
        // the case that *would* accumulate toward a disarm.
        Self {
            armed: true,
            land_complete: false,
            fs_crash_check: FS_CRASH_CHECK_DEFAULT,
            standby_active: false,
            force_flying: false,
            is_landing: false,
            crash_check_enabled: true,
            in_autorotate: false,
            filtered_accel_ms2: 0.0,
            lean_angle_deg: 20.0,
            att_error_angle_deg: 35.0,
            vel_ned_ms: None,
            crash_counter: 0,
            loop_rate_hz: 400,
        }
    }
}

/// What `Copter::crash_check` did to the counter, and whether to disarm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrashCheck {
    /// A gate fired. `crash_counter` is zero.
    Clear,
    /// Incremented, still under the 2-second trigger.
    Accumulating {
        /// New `crash_counter`.
        counter: u16,
    },
    /// Counter reached the trigger. Caller disarms with `Method::CRASH`.
    ///
    /// Upstream does **not** reset the counter on disarm.
    Disarm {
        /// New `crash_counter` (the triggered value, not zero).
        counter: u16,
    },
}

impl CrashCheck {
    /// The counter to store for the next iteration.
    #[must_use]
    pub const fn counter(self) -> u16 {
        match self {
            Self::Clear => 0,
            Self::Accumulating { counter } | Self::Disarm { counter } => counter,
        }
    }

    /// Whether the caller should `arming.disarm(Method::CRASH)`.
    #[must_use]
    pub const fn should_disarm(self) -> bool {
        matches!(self, Self::Disarm { .. })
    }
}

/// Crash-check leftover, upstream `Copter::crash_check`.
#[must_use]
pub fn crash_check(inp: &CrashCheckInputs) -> CrashCheck {
    if !inp.armed || inp.land_complete || inp.fs_crash_check == 0 {
        return CrashCheck::Clear;
    }
    if inp.standby_active {
        return CrashCheck::Clear;
    }
    if inp.force_flying && !inp.is_landing {
        return CrashCheck::Clear;
    }
    if !inp.crash_check_enabled {
        return CrashCheck::Clear;
    }
    if inp.in_autorotate {
        return CrashCheck::Clear;
    }
    if inp.filtered_accel_ms2 >= CRASH_CHECK_ACCEL_MAX_MS2 {
        return CrashCheck::Clear;
    }
    if inp.lean_angle_deg <= CRASH_CHECK_ANGLE_MIN_DEG {
        return CrashCheck::Clear;
    }
    if inp.att_error_angle_deg <= CRASH_CHECK_ANGLE_DEVIATION_DEG {
        return CrashCheck::Clear;
    }
    if let Some(speed) = inp.vel_ned_ms {
        if speed >= CRASH_CHECK_SPEED_MAX_MS {
            return CrashCheck::Clear;
        }
    }

    let counter = inp.crash_counter.wrapping_add(1);
    if u32::from(counter) >= crash_trigger_count(inp.loop_rate_hz) {
        CrashCheck::Disarm { counter }
    } else {
        CrashCheck::Accumulating { counter }
    }
}
