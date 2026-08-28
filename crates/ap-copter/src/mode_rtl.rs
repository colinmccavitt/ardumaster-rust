//! `ModeRTL` init / run leftover, upstream `ArduCopter/mode_rtl.cpp`.
//!
//! Tracked as **COP-018**. RTL is the climb-return-loiter-descent machine:
//! start at the current stopping point, climb to the return altitude, fly
//! home, loiter, then either descend to `RTL_ALT_FINAL` or land. This file
//! owns the init that parks the machine on [`RtlSubMode::Starting`] and the
//! run that advances that machine. The path geometry (`build_path` /
//! `compute_return_target`) and the LAND / FINAL_DESCENT controllers are
//! not here — they are later leftovers. What is here is *which state we
//! are in* and *which runner that state calls*.
//!
//! Upstream names the enter `init`, not `_enter`. Plane modes use `_enter`;
//! Copter modes use `init`. This is that enter.
//!
//! # Init refuses without a home
//!
//! Unless `ignore_checks`, a missing home is a hard refuse and nothing else
//! runs — no waypoint init, no state write. With a home (or with checks
//! ignored) the leftover always succeeds: `wp_and_spline_init_m(speed_ms)`,
//! `_state = STARTING`, `_state_complete = true` so the first `run()` will
//! build the path, and the two land-reposition / precland flags cleared.
//! Terrain following is allowed only when the terrain failsafe is not
//! already latched.
//!
//! # `run` is a two-switch leftover
//!
//! Disarmed returns immediately. Nothing advances, no runner fires. That is
//! not [`crate::mode_brake::is_disarmed_or_landed`] — `ModeRTL::run` tests
//! `motors->armed()` alone. The three-gate ground test lives inside the
//! climb / loiter runners.
//!
//! Armed and `_state_complete`, the first switch walks
//! STARTING → climb, INITIAL_CLIMB → return, RETURN_HOME → loiter,
//! LOITER_AT_HOME → land (if `rtl_path.land` or radio failsafe) else
//! descent. FINAL_DESCENT and LAND stay put.
//!
//! The second switch then runs the controller for the *new* state.
//! STARTING is not supposed to reach it; if it does, the leftover coerces
//! to INITIAL_CLIMB and falls through into [`rtl_climb_return_run`] — the
//! C++ `FALLTHROUGH`. A return-destination failure uses that path: it
//! restarts to STARTING with `_state_complete` still true, and the same
//! tick then fallthrough-climbs.
//!
//! # Climb and return share one runner
//!
//! [`rtl_climb_return_run`] is `climb_return_run`: ground handling, then
//! unlimited spool, `update_wpnav`, the D controller, and
//! `_state_complete = reached_wp_destination()`. Loiter uses the same
//! flying leftovers and then the timer / 2-degree armed-yaw gate.

use crate::auto_yaw::YawMode;
use crate::mode_brake::is_disarmed_or_landed;
use ap_math::scalar::{radians, wrap_pi};
use ap_motors::spool::DesiredSpoolState;

/// `RTL_ALT_M_DEFAULT`, metres.
pub const RTL_ALT_M_DEFAULT: f32 = 15.0;

/// `RTL_ALT_FINAL_M_DEFAULT`, metres. Zero means land.
pub const RTL_ALT_FINAL_M_DEFAULT: f32 = 0.0;

/// `RTL_CLIMB_MIN_M_DEFAULT`, metres.
pub const RTL_CLIMB_MIN_M_DEFAULT: f32 = 0.0;

/// `RTL_ALT_MIN_M`, metres.
pub const RTL_ALT_MIN_M: f32 = 0.30;

/// `RTL_LOITER_TIME`, milliseconds.
pub const RTL_LOITER_TIME_MS: u32 = 5_000;

/// Heading window for the RESET_TO_ARMED_YAW loiter gate, degrees.
pub const RTL_LOITER_YAW_ALIGN_DEG: f32 = 2.0;

/// `Mode::Number::RTL`.
pub const MODE_NUMBER_RTL: u8 = 6;

/// `Mode::Number::LAND` — climb-destination failure exit.
pub const MODE_NUMBER_LAND: u8 = 9;

/// `ModeReason::TERRAIN_FAILSAFE`.
pub const MODE_REASON_TERRAIN_FAILSAFE: u8 = 11;

/// RTL sub-mode, upstream `ModeRTL::SubMode`.
///
/// The numbers are declaration order. They are not logged as a MAVLink
/// enum, but the leftover treats them as pinned so a later recording can
/// compare them to `_state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlSubMode {
    /// Just entered. First complete `run()` builds the path and climbs.
    Starting,
    /// Climb to the return altitude at the origin.
    InitialClimb,
    /// Fly the return target (home or rally).
    ReturnHome,
    /// Loiter above home for `RTL_LOIT_TIME`.
    LoiterAtHome,
    /// Descend to `RTL_ALT_FINAL` when that is above zero.
    FinalDescent,
    /// Land. `is_landing()` is true only here.
    Land,
}

impl RtlSubMode {
    /// Declaration-order number.
    #[must_use]
    pub const fn as_number(self) -> u8 {
        match self {
            Self::Starting => 0,
            Self::InitialClimb => 1,
            Self::ReturnHome => 2,
            Self::LoiterAtHome => 3,
            Self::FinalDescent => 4,
            Self::Land => 5,
        }
    }
}

/// `ModeRTL::RTLAltType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlAltType {
    /// Altitude above home.
    Relative,
    /// Altitude above terrain.
    Terrain,
}

/// Which controller `run`'s second switch calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlRunner {
    /// `climb_return_run` — INITIAL_CLIMB and RETURN_HOME, and the
    /// STARTING fallthrough.
    ClimbReturn,
    /// `loiterathome_run`.
    LoiterAtHome,
    /// `descent_run`. Not expanded in this leftover.
    FinalDescent,
    /// `land_run(disarm_on_land)`. Not expanded in this leftover.
    Land {
        /// The `disarm_on_land` argument. Bare `run()` passes `true`.
        disarm_on_land: bool,
    },
}

/// `ModeRTL` capability flags from `mode.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeFlags {
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
    /// `requires_terrain_failsafe()`.
    pub requires_terrain_failsafe: bool,
}

/// Upstream `ModeRTL` flags.
#[must_use]
pub const fn rtl_mode_flags() -> RtlModeFlags {
    RtlModeFlags {
        mode_number: MODE_NUMBER_RTL,
        requires_position: true,
        has_manual_throttle: false,
        allows_arming: false,
        is_autopilot: true,
        requires_terrain_failsafe: true,
    }
}

/// `ModeRTL::get_alt_type`.
///
/// Only `RELATIVE` and `TERRAIN` are accepted. Any other parameter value
/// falls back to relative, matching the C++ range switch.
#[must_use]
pub const fn rtl_alt_type(rtl_alt_type: i8) -> RtlAltType {
    match rtl_alt_type {
        1 => RtlAltType::Terrain,
        _ => RtlAltType::Relative,
    }
}

/// Vehicle view `ModeRTL::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct RtlInitView {
    /// `AP::ahrs().home_is_set()`.
    pub home_is_set: bool,
    /// `copter.failsafe.terrain`.
    pub terrain_failsafe: bool,
    /// `speed_ms.get()`, handed to `wp_and_spline_init_m`. Zero means
    /// "use WP_SPD".
    pub speed_ms: f32,
}

impl RtlInitView {
    /// Home set, no terrain failsafe, WP_SPD (speed parameter zero).
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            home_is_set: true,
            terrain_failsafe: false,
            speed_ms: 0.0,
        }
    }
}

/// Leftover of one `ModeRTL::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlInit {
    /// What `init` returned.
    pub ok: bool,
    /// `_state` after a successful init. [`RtlSubMode::Starting`].
    pub state: RtlSubMode,
    /// `_state_complete` after a successful init. Always true then.
    pub state_complete: bool,
    /// `terrain_following_allowed = !failsafe.terrain`.
    pub terrain_following_allowed: bool,
    /// `copter.ap.land_repo_active` after init. Always false.
    pub land_repo_active: bool,
    /// `copter.ap.prec_land_active` after init. Always false.
    pub prec_land_active: bool,
    /// Speed handed to `wp_and_spline_init_m`.
    pub wp_speed_ms: f32,
}

/// Upstream `ModeRTL::init`.
#[must_use]
pub fn rtl_init(view: &RtlInitView, ignore_checks: bool) -> RtlInit {
    if !ignore_checks && !view.home_is_set {
        return RtlInit {
            ok: false,
            state: RtlSubMode::Starting,
            state_complete: false,
            terrain_following_allowed: false,
            land_repo_active: false,
            prec_land_active: false,
            wp_speed_ms: view.speed_ms,
        };
    }

    RtlInit {
        ok: true,
        state: RtlSubMode::Starting,
        state_complete: true,
        terrain_following_allowed: !view.terrain_failsafe,
        land_repo_active: false,
        prec_land_active: false,
        wp_speed_ms: view.speed_ms,
    }
}

/// Vehicle view `ModeRTL::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct RtlRunView {
    /// `motors->armed()`. The outer `run` gate.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `_state` on entry.
    pub state: RtlSubMode,
    /// `_state_complete` on entry.
    pub state_complete: bool,
    /// `rtl_path.land` — `alt_final_m <= 0`.
    pub path_land: bool,
    /// `copter.failsafe.radio`.
    pub radio_failsafe: bool,
    /// `set_wp_destination_loc(climb) && set_wp_destination_next_loc(return)`.
    pub climb_dest_ok: bool,
    /// `set_wp_destination_loc(return_target)`.
    pub return_dest_ok: bool,
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp: bool,
    /// `auto_yaw.default_mode(true)` — RTL's default yaw.
    pub default_yaw: YawMode,
    /// `auto_yaw.mode()` during loiter, for the armed-yaw gate.
    pub yaw_mode: YawMode,
    /// `ahrs.get_yaw_rad()`.
    pub yaw_rad: f32,
    /// `copter.initial_armed_bearing_rad`.
    pub initial_armed_bearing_rad: f32,
    /// `_loiter_start_time` already latched from a previous tick.
    pub loiter_start_ms: u32,
    /// `millis()`.
    pub now_ms: u32,
    /// `g.rtl_loiter_time`, milliseconds.
    pub rtl_loiter_time_ms: u32,
    /// The `disarm_on_land` argument. Bare `run()` passes `true`.
    pub disarm_on_land: bool,
}

impl RtlRunView {
    /// Armed, auto-armed, airborne, mid initial climb, destination ok.
    #[must_use]
    pub const fn climbing() -> Self {
        Self {
            armed: true,
            auto_armed: true,
            land_complete: false,
            state: RtlSubMode::InitialClimb,
            state_complete: false,
            path_land: true,
            radio_failsafe: false,
            climb_dest_ok: true,
            return_dest_ok: true,
            reached_wp: false,
            default_yaw: YawMode::Hold,
            yaw_mode: YawMode::Hold,
            yaw_rad: 0.0,
            initial_armed_bearing_rad: 0.0,
            loiter_start_ms: 0,
            now_ms: 0,
            rtl_loiter_time_ms: RTL_LOITER_TIME_MS,
            disarm_on_land: true,
        }
    }
}

/// Attitude / position leftover of the climb-return / loiter WP runners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlWpRun {
    /// `is_disarmed_or_landed` fired; `make_safe_ground_handling` and return.
    pub safe_ground: bool,
    /// Spool ask on the flying path. `None` on the ground path.
    pub desired_spool: Option<DesiredSpoolState>,
    /// `wp_nav->update_wpnav` on the flying path.
    pub update_wpnav: bool,
    /// `D_update_controller` on the flying path.
    pub update_d: bool,
}

/// Leftover of one `ModeRTL::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlRun {
    /// `!motors->armed()` returned before either switch.
    pub early_return_disarmed: bool,
    /// The first switch ran because `_state_complete` was true.
    pub advanced: bool,
    /// STARTING complete called `build_path`.
    pub built_path: bool,
    /// `_state` after both switches (including STARTING fallthrough).
    pub state: RtlSubMode,
    /// `_state_complete` after the runner, if one fired.
    pub state_complete: bool,
    /// Yaw mode the start leftover asked for. `None` if no start ran.
    pub yaw: Option<YawMode>,
    /// Climb or return destination setter failed.
    pub dest_failed: bool,
    /// Climb dest fail asked `set_mode(LAND, TERRAIN_FAILSAFE)`.
    pub switch_to_land: bool,
    /// Return dest fail called `restart_without_terrain`.
    pub restart_without_terrain: bool,
    /// `terrain_following_allowed` after a restart. `None` if unchanged.
    pub terrain_following_allowed: Option<bool>,
    /// `_loiter_start_time` after a loiter start. `None` if not started.
    pub loiter_start_ms: Option<u32>,
    /// The second-switch runner. `None` on the disarmed early return.
    pub runner: Option<RtlRunner>,
    /// Climb-return / loiter WP leftover. `None` for descent / land / early.
    pub wp: Option<RtlWpRun>,
}

/// `ModeRTL::is_landing`.
#[must_use]
pub const fn rtl_is_landing(state: RtlSubMode) -> bool {
    matches!(state, RtlSubMode::Land)
}

/// The RESET_TO_ARMED_YAW loiter heading gate.
///
/// `fabsf(wrap_PI(yaw - armed_bearing)) <= radians(2)`.
#[must_use]
pub fn rtl_loiter_yaw_aligned(yaw_rad: f32, armed_bearing_rad: f32) -> bool {
    libm::fabsf(wrap_pi(yaw_rad - armed_bearing_rad)) <= radians(RTL_LOITER_YAW_ALIGN_DEG)
}

/// Whether the loiter stage is finished.
///
/// Time uses unsigned wrap, C++ `uint32_t`. Equality fires.
#[must_use]
pub fn rtl_loiter_complete(
    now_ms: u32,
    loiter_start_ms: u32,
    rtl_loiter_time_ms: u32,
    yaw_mode: YawMode,
    yaw_rad: f32,
    armed_bearing_rad: f32,
) -> bool {
    if now_ms.wrapping_sub(loiter_start_ms) < rtl_loiter_time_ms {
        return false;
    }
    if yaw_mode == YawMode::ResetToArmedYaw {
        rtl_loiter_yaw_aligned(yaw_rad, armed_bearing_rad)
    } else {
        true
    }
}

/// Shared flying leftovers of `climb_return_run` / `loiterathome_run`.
#[must_use]
pub fn rtl_wp_run(armed: bool, auto_armed: bool, land_complete: bool) -> RtlWpRun {
    if is_disarmed_or_landed(armed, auto_armed, land_complete) {
        return RtlWpRun {
            safe_ground: true,
            desired_spool: None,
            update_wpnav: false,
            update_d: false,
        };
    }
    RtlWpRun {
        safe_ground: false,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
        update_wpnav: true,
        update_d: true,
    }
}

fn yaw_for_loiter_start(default_yaw: YawMode) -> YawMode {
    if default_yaw == YawMode::Hold {
        YawMode::Hold
    } else {
        YawMode::ResetToArmedYaw
    }
}

/// Upstream `ModeRTL::run`.
///
/// Bare C++ `run()` is `run(true)`.
#[must_use]
pub fn rtl_run(view: &RtlRunView) -> RtlRun {
    if !view.armed {
        return RtlRun {
            early_return_disarmed: true,
            advanced: false,
            built_path: false,
            state: view.state,
            state_complete: view.state_complete,
            yaw: None,
            dest_failed: false,
            switch_to_land: false,
            restart_without_terrain: false,
            terrain_following_allowed: None,
            loiter_start_ms: None,
            runner: None,
            wp: None,
        };
    }

    let mut state = view.state;
    let mut state_complete = view.state_complete;
    let mut built_path = false;
    let mut yaw = None;
    let mut dest_failed = false;
    let mut switch_to_land = false;
    let mut restart_without_terrain = false;
    let mut terrain_following_allowed = None;
    let mut loiter_start_ms = None;
    let advanced = view.state_complete;

    if view.state_complete {
        match view.state {
            RtlSubMode::Starting => {
                built_path = true;
                state = RtlSubMode::InitialClimb;
                state_complete = false;
                if view.climb_dest_ok {
                    yaw = Some(YawMode::Hold);
                } else {
                    dest_failed = true;
                    switch_to_land = true;
                }
            }
            RtlSubMode::InitialClimb => {
                state = RtlSubMode::ReturnHome;
                state_complete = false;
                yaw = Some(view.default_yaw);
                if !view.return_dest_ok {
                    dest_failed = true;
                    restart_without_terrain = true;
                    state = RtlSubMode::Starting;
                    state_complete = true;
                    terrain_following_allowed = Some(false);
                }
            }
            RtlSubMode::ReturnHome => {
                state = RtlSubMode::LoiterAtHome;
                state_complete = false;
                loiter_start_ms = Some(view.now_ms);
                yaw = Some(yaw_for_loiter_start(view.default_yaw));
            }
            RtlSubMode::LoiterAtHome => {
                if view.path_land || view.radio_failsafe {
                    state = RtlSubMode::Land;
                } else {
                    state = RtlSubMode::FinalDescent;
                }
                state_complete = false;
                yaw = Some(YawMode::Hold);
            }
            RtlSubMode::FinalDescent | RtlSubMode::Land => {}
        }
    }

    // STARTING fallthrough: coerce and run the climb leftover.
    if state == RtlSubMode::Starting {
        state = RtlSubMode::InitialClimb;
    }

    let (runner, wp, runner_complete) = match state {
        RtlSubMode::Starting | RtlSubMode::InitialClimb | RtlSubMode::ReturnHome => {
            let wp = rtl_wp_run(view.armed, view.auto_armed, view.land_complete);
            let complete = if wp.safe_ground {
                state_complete
            } else {
                view.reached_wp
            };
            (RtlRunner::ClimbReturn, Some(wp), complete)
        }
        RtlSubMode::LoiterAtHome => {
            let wp = rtl_wp_run(view.armed, view.auto_armed, view.land_complete);
            let start = loiter_start_ms.unwrap_or(view.loiter_start_ms);
            let complete = if wp.safe_ground {
                state_complete
            } else {
                rtl_loiter_complete(
                    view.now_ms,
                    start,
                    view.rtl_loiter_time_ms,
                    yaw.unwrap_or(view.yaw_mode),
                    view.yaw_rad,
                    view.initial_armed_bearing_rad,
                )
            };
            (RtlRunner::LoiterAtHome, Some(wp), complete)
        }
        RtlSubMode::FinalDescent => (RtlRunner::FinalDescent, None, state_complete),
        RtlSubMode::Land => (
            RtlRunner::Land {
                disarm_on_land: view.disarm_on_land,
            },
            None,
            state_complete,
        ),
    };

    RtlRun {
        early_return_disarmed: false,
        advanced,
        built_path,
        state,
        state_complete: runner_complete,
        yaw,
        dest_failed,
        switch_to_land,
        restart_without_terrain,
        terrain_following_allowed,
        loiter_start_ms,
        runner: Some(runner),
        wp,
    }
}
