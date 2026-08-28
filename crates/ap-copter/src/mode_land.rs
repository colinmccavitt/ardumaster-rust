//! `ModeLand` init / run leftover, upstream `ArduCopter/mode_land.cpp`.
//!
//! Tracked as **COP-018**. Land is the dedicated landing mode: hold heading,
//! descend, and (when a position estimate is available) hold the landing
//! spot. This file owns the init that sizes the position controller and
//! parks the pause timer, and the run that either flies the GPS landing
//! or hands roll/pitch back to the pilot. The descent demand itself is
//! [`crate::land::land_descent`]; the horizontal leftover is
//! [`crate::land_horizontal`]. What is here is *which of those runs* and
//! *whether we are even descending yet*.
//!
//! Upstream names the enter `init`, not `_enter`. Plane modes use `_enter`;
//! Copter modes use `init`. This is that enter.
//!
//! # Init always succeeds
//!
//! `ignore_checks` is unused. Land does not require a home or a position
//! estimate — [`LandModeFlags::requires_position`] is false — so a GPS
//! failsafe can drop into this mode and still descend. `control_position`
//! is latched from `position_ok()` at init (and later cleared by
//! [`land_do_not_use_gps`] if the estimate dies mid-landing). The NE
//! controller is initialised only when that latch is true *and* NE is
//! inactive; D is initialised whenever it is inactive. Both limit sets
//! are always written from the waypoint navigator's defaults.
//!
//! Init also starts the pause clock (`land_start_time = millis()`),
//! clears `land_pause`, and clears the land-reposition / precland flags.
//! Yaw is HOLD. A failsafe that wants the four-second hold calls
//! [`land_with_pause`] *after* init, which is why init itself always
//! leaves pause false.
//!
//! # `run` is two leftover machines
//!
//! `control_position` picks [`LandRunner::Gps`] or [`LandRunner::NoGps`].
//! Both disarm on `land_complete && GROUND_IDLE`, both park on
//! [`crate::mode_brake::is_disarmed_or_landed`], and both clear
//! `land_pause` once [`LAND_WITH_DELAY_MS`] has elapsed — but only on
//! the flying path. The GPS flying path then asks
//! `land_run_normal_or_precland(pause)` and does not run the attitude
//! controller itself. The no-GPS flying path asks
//! `land_run_vertical_control(pause)` and then *always* runs the
//! attitude controller, even on the ground path, with the pilot's lean
//! (or zero) and the HOLD yaw rate.
//!
//! # No-GPS throttle cancel goes to AltHold, not Loiter
//!
//! That is not [`crate::land_horizontal::land_cancel_destination`]. The
//! no-GPS runner has no position estimate to hold, so a raised stick
//! asks AltHold directly with [`MODE_REASON_THROTTLE_LAND_ESCAPE`]. The
//! leftover still continues the land tick after recording the request —
//! `set_mode` is a request, not a return.

use crate::auto_yaw::YawMode;
use crate::land::land_pause_expired;
use crate::land_horizontal::land_cancelled_by_throttle;
use crate::mode_brake::is_disarmed_or_landed;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `LAND_SPD_MS_DEFAULT`, metres per second. Final-stage descent.
pub const LAND_SPD_MS_DEFAULT: f32 = 0.5;

/// `LAND_SPD_HIGH_MS` parameter default. Zero means "use WP_SPD_DN".
pub const LAND_SPD_HIGH_MS_DEFAULT: f32 = 0.0;

/// `LAND_ALT_LOW_M` parameter default, metres.
pub const LAND_ALT_LOW_M_DEFAULT: f32 = 10.0;

/// `LAND_WITH_DELAY_MS`, re-exported so Land callers do not have to
/// reach into [`crate::land`] for the failsafe pause they just armed.
pub use crate::land::LAND_WITH_DELAY_MS;

/// `Mode::Number::LAND`.
pub const MODE_NUMBER_LAND: u8 = 9;

/// `Mode::Number::ALT_HOLD` — no-GPS throttle-cancel exit.
pub const MODE_NUMBER_ALT_HOLD: u8 = 2;

/// `ModeReason::THROTTLE_LAND_ESCAPE`.
pub const MODE_REASON_THROTTLE_LAND_ESCAPE: u8 = 9;

/// `ModeLand` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. False: Land can descend without GPS.
    pub requires_position: bool,
    /// `has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
    /// `is_landing()`. Always true in this mode.
    pub is_landing: bool,
}

/// Upstream `ModeLand` flags.
#[must_use]
pub const fn land_mode_flags() -> LandModeFlags {
    LandModeFlags {
        mode_number: MODE_NUMBER_LAND,
        requires_position: false,
        has_manual_throttle: false,
        allows_arming: false,
        is_autopilot: true,
        is_landing: true,
    }
}

/// Vehicle view `ModeLand::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct LandInitView {
    /// `copter.position_ok()`.
    pub position_ok: bool,
    /// `pos_control->NE_is_active()`.
    pub ne_is_active: bool,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_NE_ms()`.
    pub speed_ne_ms: f32,
    /// `wp_nav->get_wp_acceleration_mss()`.
    pub wp_accel_mss: f32,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `millis()`.
    pub now_ms: u32,
}

impl LandInitView {
    /// Position ok, both controllers already running.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            position_ok: true,
            ne_is_active: true,
            d_is_active: true,
            speed_ne_ms: 5.0,
            wp_accel_mss: 1.0,
            speed_down_ms: 1.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            now_ms: 1_000,
        }
    }
}

/// Leftover of one `ModeLand::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandInit {
    /// Always true. `ignore_checks` is unused.
    pub ok: bool,
    /// `control_position` after init. Latched from `position_ok`.
    pub control_position: bool,
    /// Horizontal max / correction speed handed to the NE controller.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel, from `wp_nav`.
    pub ne_accel_mss: f32,
    /// `NE_init_controller` — only when position-ok and NE was inactive.
    pub init_ne: bool,
    /// Vertical max / correction speed down.
    pub d_speed_down_ms: f32,
    /// Vertical max / correction speed up.
    pub d_speed_up_ms: f32,
    /// Vertical max / correction accel, from `wp_nav`.
    pub d_accel_mss: f32,
    /// `D_init_controller` — only when D was inactive.
    pub init_d: bool,
    /// `land_start_time` after init. `millis()`.
    pub land_start_ms: u32,
    /// `land_pause` after init. Always false.
    pub land_pause: bool,
    /// `copter.ap.land_repo_active` after init. Always false.
    pub land_repo_active: bool,
    /// `copter.ap.prec_land_active` after init. Always false.
    pub prec_land_active: bool,
    /// Yaw mode init asked for. Always [`YawMode::Hold`].
    pub yaw: YawMode,
}

/// Upstream `ModeLand::init`.
///
/// `ignore_checks` is accepted and ignored, matching the unused parameter.
#[must_use]
pub fn land_init(view: &LandInitView, _ignore_checks: bool) -> LandInit {
    let control_position = view.position_ok;
    LandInit {
        ok: true,
        control_position,
        ne_speed_ms: view.speed_ne_ms,
        ne_accel_mss: view.wp_accel_mss,
        init_ne: control_position && !view.ne_is_active,
        d_speed_down_ms: view.speed_down_ms,
        d_speed_up_ms: view.speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        init_d: !view.d_is_active,
        land_start_ms: view.now_ms,
        land_pause: false,
        land_repo_active: false,
        prec_land_active: false,
        yaw: YawMode::Hold,
    }
}

/// Which runner `ModeLand::run` dispatched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandRunner {
    /// `gps_run` — `control_position` was true.
    Gps,
    /// `nogps_run` — `control_position` was false.
    NoGps,
}

/// Vehicle view `ModeLand::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct LandRunView {
    /// `control_position`. Picks the runner.
    pub control_position: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `land_pause` on entry.
    pub land_pause: bool,
    /// `land_start_time`.
    pub land_start_ms: u32,
    /// `millis()`.
    pub now_ms: u32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `g.throttle_behavior`.
    pub throttle_behavior: i32,
    /// `copter.rc_throttle_control_in_filter.get()`.
    pub filtered_throttle_control_in: f32,
    /// `g.land_repositioning`.
    pub land_repositioning: bool,
}

impl LandRunView {
    /// Armed, auto-armed, airborne, GPS landing, no pause.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            control_position: true,
            armed: true,
            auto_armed: true,
            land_complete: false,
            spool_state: SpoolState::ThrottleUnlimited,
            land_pause: false,
            land_start_ms: 0,
            now_ms: 0,
            has_valid_input: true,
            throttle_behavior: 0,
            filtered_throttle_control_in: 0.0,
            land_repositioning: false,
        }
    }
}

/// Leftover of one `ModeLand::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandRun {
    /// Which C++ runner fired.
    pub runner: LandRunner,
    /// `land_complete && spool == GROUND_IDLE` asked `disarm(LANDED)`.
    pub disarm_landed: bool,
    /// `is_disarmed_or_landed` fired; `make_safe_ground_handling`.
    pub safe_ground: bool,
    /// Spool ask on the flying path. `None` on the ground path.
    pub desired_spool: Option<DesiredSpoolState>,
    /// `land_pause` after the flying-path timeout check.
    pub land_pause: bool,
    /// The flying path cleared a still-set pause this tick.
    pub pause_cleared: bool,
    /// GPS flying path asked `land_run_normal_or_precland(pause)`.
    pub land_normal_or_precland: bool,
    /// No-GPS flying path asked `land_run_vertical_control(pause)`.
    pub land_vertical: bool,
    /// No-GPS throttle cancel asked `set_mode(ALT_HOLD, THROTTLE_LAND_ESCAPE)`.
    pub cancel_to_althold: bool,
    /// No-GPS path will read pilot lean (`valid input && land_repositioning`).
    pub use_pilot_lean: bool,
    /// No-GPS path always runs the attitude controller, ground included.
    pub attitude: bool,
}

/// Shared disarm / pause / ground leftovers of `gps_run` / `nogps_run`.
struct LandStage {
    disarm_landed: bool,
    safe_ground: bool,
    desired_spool: Option<DesiredSpoolState>,
    land_pause: bool,
    pause_cleared: bool,
    flying: bool,
}

fn land_stage(view: &LandRunView) -> LandStage {
    let disarm_landed = view.land_complete && view.spool_state == SpoolState::GroundIdle;
    if is_disarmed_or_landed(view.armed, view.auto_armed, view.land_complete) {
        return LandStage {
            disarm_landed,
            safe_ground: true,
            desired_spool: None,
            land_pause: view.land_pause,
            pause_cleared: false,
            flying: false,
        };
    }

    let pause_cleared = land_pause_expired(view.land_pause, view.now_ms, view.land_start_ms);
    LandStage {
        disarm_landed,
        safe_ground: false,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
        land_pause: view.land_pause && !pause_cleared,
        pause_cleared,
        flying: true,
    }
}

/// Upstream `ModeLand::run`.
#[must_use]
pub fn land_run(view: &LandRunView) -> LandRun {
    let stage = land_stage(view);
    if view.control_position {
        LandRun {
            runner: LandRunner::Gps,
            disarm_landed: stage.disarm_landed,
            safe_ground: stage.safe_ground,
            desired_spool: stage.desired_spool,
            land_pause: stage.land_pause,
            pause_cleared: stage.pause_cleared,
            land_normal_or_precland: stage.flying,
            land_vertical: false,
            cancel_to_althold: false,
            use_pilot_lean: false,
            attitude: false,
        }
    } else {
        LandRun {
            runner: LandRunner::NoGps,
            disarm_landed: stage.disarm_landed,
            safe_ground: stage.safe_ground,
            desired_spool: stage.desired_spool,
            land_pause: stage.land_pause,
            pause_cleared: stage.pause_cleared,
            land_normal_or_precland: false,
            land_vertical: stage.flying,
            cancel_to_althold: land_cancelled_by_throttle(
                view.throttle_behavior,
                view.filtered_throttle_control_in,
                view.has_valid_input,
            ),
            use_pilot_lean: view.has_valid_input && view.land_repositioning,
            attitude: true,
        }
    }
}

/// `ModeLand::do_not_use_GPS`. The new `control_position` value.
///
/// A GPS failsafe that finds the vehicle already in Land must not keep
/// using the estimate. The leftover is just the latch: false.
#[must_use]
pub const fn land_do_not_use_gps() -> bool {
    false
}

/// `ModeLand::use_pilot_yaw`. Only when repositioning is enabled.
#[must_use]
pub const fn land_use_pilot_yaw(land_repositioning: bool) -> bool {
    land_repositioning
}

/// Leftover of `Copter::set_mode_land_with_pause`.
///
/// Init has already run (pause false). This leftover then sets pause
/// and raises the failsafe-mode-change notify. The mode-number is
/// always Land; the reason is the caller's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandWithPause {
    /// `set_mode` target. Always [`MODE_NUMBER_LAND`].
    pub mode_number: u8,
    /// `mode_land.set_land_pause(true)`.
    pub land_pause: bool,
    /// `AP_Notify::events.failsafe_mode_change`.
    pub failsafe_mode_change: bool,
}

/// Upstream `Copter::set_mode_land_with_pause` after `set_mode` succeeds.
#[must_use]
pub const fn land_with_pause() -> LandWithPause {
    LandWithPause {
        mode_number: MODE_NUMBER_LAND,
        land_pause: true,
        failsafe_mode_change: true,
    }
}

/// `Copter::landing_with_GPS`.
#[must_use]
pub const fn landing_with_gps(mode_number: u8, control_position: bool) -> bool {
    mode_number == MODE_NUMBER_LAND && control_position
}
