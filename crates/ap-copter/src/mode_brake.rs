//! `ModeBrake` init / run leftover, upstream `ArduCopter/mode_brake.cpp`.
//!
//! Tracked as **COP-019**. Brake is the emergency stop: hold heading, drive
//! NE velocity to zero at [`BRAKE_MODE_DECEL_RATE_MSS`], and hold altitude.
//! There is no pilot stick. The leftover this file owns is the init that
//! sizes the position controller to the *current* ground speed, the run that
//! either parks on the ground or commands that stop, and the Solo
//! pause-button timeout that leaves Brake for Loiter (or AltHold).
//!
//! Upstream names the enter `init`, not `_enter`. Plane modes use `_enter`;
//! Copter modes use `init`. This is that enter.
//!
//! # Init sizes the stop to the speed we already have
//!
//! `NE_set_max_speed_accel_m` and the correction twin both take
//! `get_vel_estimate_NED_ms().xy().length()`, not a fixed cruise. A vehicle
//! that entered Brake at 12 m/s is allowed 12 m/s while it decelerates at
//! 7.5 m/s²; inventing a smaller max speed would clip the stop. Vertical
//! limits are the constants. `D_init_controller` runs only when the D
//! controller is inactive — an already-running altitude loop is left
//! alone so a failsafe that dropped into Brake mid-climb does not snap
//! the throttle.
//!
//! `ignore_checks` is unused. Init always succeeds and always clears
//! `_timeout_ms`. A leftover timeout from a previous Solo pause must not
//! fire the moment Brake is re-entered.
//!
//! # Ground handling returns before the timeout
//!
//! `is_disarmed_or_landed` (`!armed || !auto_armed || land_complete`)
//! calls `make_safe_ground_handling`, relaxes D to zero, and returns.
//! The timeout is not consulted on that path. A Solo pause that expires
//! while the aircraft is on the ground does not switch modes from here.
//!
//! # The flying path is a zero-velocity stop
//!
//! Motors go `THROTTLE_UNLIMITED`. `land_complete_maybe` softens the NE
//! target so a maybe-landed airframe does not fight the ground. Then
//! `input_vel_accel_NE_m(0, 0)` — stop, no extra accel — and a heading
//! rate of zero on the thrust vector. Climb rate is zero. That is the
//! whole attitude / position leftover; the numbers are constants.
//!
//! # Timeout tries Loiter, then AltHold
//!
//! `timeout_to_loiter_ms` arms `_timeout_start = millis()`. When
//! `_timeout_ms != 0` and `millis() - _timeout_start >= _timeout_ms`
//! (unsigned wrap, C++ `uint32_t`), the caller tries Loiter with
//! `ModeReason::BRAKE_TIMEOUT` and falls back to AltHold if Loiter
//! refuses. Equality fires — a port that used `>` would wait one extra
//! millisecond.

use ap_motors::spool::DesiredSpoolState;

/// z-axis speed in Brake, m/s. Upstream `BRAKE_MODE_SPEED_Z_MS`.
pub const BRAKE_MODE_SPEED_Z_MS: f32 = 2.50;

/// Deceleration in Brake, m/s². Upstream `BRAKE_MODE_DECEL_RATE_MSS`.
pub const BRAKE_MODE_DECEL_RATE_MSS: f32 = 7.50;

/// `Mode::Number::BRAKE`.
pub const MODE_NUMBER_BRAKE: u8 = 17;

/// `Mode::Number::LOITER` — first timeout exit.
pub const MODE_NUMBER_LOITER: u8 = 5;

/// `Mode::Number::ALT_HOLD` — timeout fallback if Loiter refuses.
pub const MODE_NUMBER_ALT_HOLD: u8 = 2;

/// `ModeReason::BRAKE_TIMEOUT`.
pub const MODE_REASON_BRAKE_TIMEOUT: u8 = 12;

/// `ModeBrake` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BrakeModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`.
    pub requires_position: bool,
    /// `has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
}

/// Upstream `ModeBrake` flags.
#[must_use]
pub const fn brake_mode_flags() -> BrakeModeFlags {
    BrakeModeFlags {
        mode_number: MODE_NUMBER_BRAKE,
        requires_position: true,
        has_manual_throttle: false,
        allows_arming: false,
        is_autopilot: true,
    }
}

/// Vehicle view `ModeBrake::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct BrakeInitView {
    /// `pos_control->get_vel_estimate_NED_ms().xy().length()`, m/s.
    pub vel_ne_ms: f32,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
}

impl BrakeInitView {
    /// Hovering, D controller already running — the failsafe-entry path.
    #[must_use]
    pub const fn hovering() -> Self {
        Self {
            vel_ne_ms: 0.0,
            d_is_active: true,
        }
    }
}

/// Leftover of one `ModeBrake::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakeInit {
    /// Horizontal max / correction speed handed to the NE controller.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel, always [`BRAKE_MODE_DECEL_RATE_MSS`].
    pub ne_accel_mss: f32,
    /// Always true: `NE_init_controller`.
    pub init_ne: bool,
    /// Vertical max / correction speed up and down.
    pub d_speed_ms: f32,
    /// Vertical max / correction accel, always [`BRAKE_MODE_DECEL_RATE_MSS`].
    pub d_accel_mss: f32,
    /// `D_init_controller` — only when D was inactive.
    pub init_d: bool,
    /// `_timeout_ms` after init. Always zero.
    pub timeout_ms: u32,
    /// Always true. `ignore_checks` is unused.
    pub ok: bool,
}

/// Upstream `ModeBrake::init`.
///
/// `ignore_checks` is accepted and ignored, matching the unused parameter.
#[must_use]
pub fn brake_init(view: &BrakeInitView, _ignore_checks: bool) -> BrakeInit {
    BrakeInit {
        ne_speed_ms: view.vel_ne_ms,
        ne_accel_mss: BRAKE_MODE_DECEL_RATE_MSS,
        init_ne: true,
        d_speed_ms: BRAKE_MODE_SPEED_Z_MS,
        d_accel_mss: BRAKE_MODE_DECEL_RATE_MSS,
        init_d: !view.d_is_active,
        timeout_ms: 0,
        ok: true,
    }
}

/// Vehicle view `ModeBrake::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct BrakeRunView {
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.land_complete_maybe`.
    pub land_complete_maybe: bool,
    /// `_timeout_ms`. Zero disables the Solo pause exit.
    pub timeout_ms: u32,
    /// `_timeout_start`, milliseconds.
    pub timeout_start_ms: u32,
    /// `millis()`.
    pub now_ms: u32,
}

impl BrakeRunView {
    /// Armed, auto-armed, airborne, no timeout.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            armed: true,
            auto_armed: true,
            land_complete: false,
            land_complete_maybe: false,
            timeout_ms: 0,
            timeout_start_ms: 0,
            now_ms: 0,
        }
    }
}

/// Where the timeout, if any, wants the vehicle to go.
///
/// The leftover cannot know whether `set_mode(LOITER)` will succeed, so
/// it reports the pair: try Loiter, then AltHold. The reason is always
/// [`MODE_REASON_BRAKE_TIMEOUT`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrakeTimeoutExit {
    /// Timeout disabled, or not yet elapsed. Stay in Brake.
    None,
    /// Elapsed. Try Loiter, fall back to AltHold.
    LoiterThenAltHold,
}

/// Attitude / position leftover of one `ModeBrake::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakeRun {
    /// `is_disarmed_or_landed()` fired; `make_safe_ground_handling` plus
    /// `D_relax_controller(0)` and return. The flying leftovers below
    /// are then unused.
    pub safe_ground: bool,
    /// `D_relax_controller(0)` — always with [`Self::safe_ground`].
    pub relax_d: bool,
    /// Spool ask on the flying path. `None` on the ground path.
    pub desired_spool: Option<DesiredSpoolState>,
    /// `NE_soften_for_landing` when `land_complete_maybe`.
    pub soften_ne: bool,
    /// `input_vel_accel_NE_m` north/east velocity, always zero.
    pub vel_ne_ms: f32,
    /// `input_vel_accel_NE_m` north/east accel, always zero.
    pub accel_ne_mss: f32,
    /// `NE_update_controller` on the flying path.
    pub update_ne: bool,
    /// Heading rate handed to `input_thrust_vector_rate_heading_rads`.
    pub heading_rate_rads: f32,
    /// `D_set_pos_target_from_climb_rate_ms`, always zero.
    pub climb_rate_ms: f32,
    /// `D_update_controller` on the flying path.
    pub update_d: bool,
    /// Timeout leftover. Always [`BrakeTimeoutExit::None`] on the ground.
    pub timeout_exit: BrakeTimeoutExit,
}

/// Upstream `Mode::is_disarmed_or_landed`.
#[must_use]
pub const fn is_disarmed_or_landed(armed: bool, auto_armed: bool, land_complete: bool) -> bool {
    !armed || !auto_armed || land_complete
}

/// Whether the Solo pause timeout has elapsed.
///
/// `_timeout_ms != 0 && millis() - _timeout_start >= _timeout_ms`.
/// Subtraction wraps the way C++ `uint32_t` does.
#[must_use]
pub const fn brake_timeout_elapsed(timeout_ms: u32, start_ms: u32, now_ms: u32) -> bool {
    timeout_ms != 0 && now_ms.wrapping_sub(start_ms) >= timeout_ms
}

/// Upstream `ModeBrake::timeout_to_loiter_ms`.
///
/// Returns the new `_timeout_start` / `_timeout_ms` pair. `timeout_ms == 0`
/// disables the exit; the start is still written, matching upstream.
#[must_use]
pub const fn timeout_to_loiter_ms(now_ms: u32, timeout_ms: u32) -> (u32, u32) {
    (now_ms, timeout_ms)
}

/// Upstream `ModeBrake::run`.
#[must_use]
pub fn brake_run(view: &BrakeRunView) -> BrakeRun {
    if is_disarmed_or_landed(view.armed, view.auto_armed, view.land_complete) {
        return BrakeRun {
            safe_ground: true,
            relax_d: true,
            desired_spool: None,
            soften_ne: false,
            vel_ne_ms: 0.0,
            accel_ne_mss: 0.0,
            update_ne: false,
            heading_rate_rads: 0.0,
            climb_rate_ms: 0.0,
            update_d: false,
            timeout_exit: BrakeTimeoutExit::None,
        };
    }

    let timeout_exit = if brake_timeout_elapsed(view.timeout_ms, view.timeout_start_ms, view.now_ms)
    {
        BrakeTimeoutExit::LoiterThenAltHold
    } else {
        BrakeTimeoutExit::None
    };

    BrakeRun {
        safe_ground: false,
        relax_d: false,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
        soften_ne: view.land_complete_maybe,
        vel_ne_ms: 0.0,
        accel_ne_mss: 0.0,
        update_ne: true,
        heading_rate_rads: 0.0,
        climb_rate_ms: 0.0,
        update_d: true,
        timeout_exit,
    }
}
