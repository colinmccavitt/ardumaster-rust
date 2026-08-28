//! `ModeRTL` init / run leftover, upstream `ArduCopter/mode_rtl.cpp`.
//!
//! Tracked as **COP-018**. RTL is the climb-return-loiter-descent machine:
//! start at the current stopping point, climb to the return altitude, fly
//! home, loiter, then either descend to `RTL_ALT_FINAL` or land. This file
//! owns the init that parks the machine on [`RtlSubMode::Starting`] and the
//! run that advances that machine. The path geometry (`build_path` /
//! `compute_return_target`) and the LAND controller (`land_start` /
//! `land_run`) are here, as are descent start/run and
//! restart-without-terrain. What is here is *which state we are in*,
//! *which runner that state calls*, and the leftover those runners
//! still needed.
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
use crate::land_horizontal::land_cancelled_by_throttle;
use crate::mode_brake::is_disarmed_or_landed;
use ap_math::scalar::{radians, wrap_pi};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

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
    /// `descent_run`. See [`rtl_descent_run`].
    FinalDescent,
    /// `land_run(disarm_on_land)`. See [`rtl_land_run`].
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

/// Within this many metres of `RTL_ALT_FINAL` the descent stage is done.
///
/// Upstream compares centimetres converted with `* 0.01` against a
/// 20 cm window. The leftover takes metres on both sides.
pub const RTL_DESCENT_COMPLETE_M: f32 = 0.2;

/// Leftover of `ModeRTL::restart_without_terrain`.
///
/// The climb-return runner already applies this when the return
/// destination setter fails. The function is the leftover itself: terrain
/// following is forbidden, the machine is parked back on
/// [`RtlSubMode::Starting`] with `_state_complete` still true so the same
/// tick fallthrough-climbs a no-terrain path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlRestart {
    /// Always false. Terrain following is done for this RTL.
    pub terrain_following_allowed: bool,
    /// Always [`RtlSubMode::Starting`].
    pub state: RtlSubMode,
    /// Always true, so `run`'s first switch rebuilds the path immediately.
    pub state_complete: bool,
}

/// Upstream `ModeRTL::restart_without_terrain`.
#[must_use]
pub const fn rtl_restart_without_terrain() -> RtlRestart {
    RtlRestart {
        terrain_following_allowed: false,
        state: RtlSubMode::Starting,
        state_complete: true,
    }
}

/// Leftover of `ModeRTL::descent_start`.
///
/// The first-switch walk in [`rtl_run`] already parks `_state` on
/// [`RtlSubMode::FinalDescent`]. This leftover is the controller seed
/// that walk does not own: initialise D at the current stopping point
/// and hold yaw. Landing-gear deploy is not here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlDescentStart {
    /// Always [`RtlSubMode::FinalDescent`].
    pub state: RtlSubMode,
    /// Always false.
    pub state_complete: bool,
    /// `D_init_controller_stopping_point`.
    pub d_init_stopping_point: bool,
    /// Always [`YawMode::Hold`].
    pub yaw: YawMode,
}

/// Upstream `ModeRTL::descent_start`.
#[must_use]
pub const fn rtl_descent_start() -> RtlDescentStart {
    RtlDescentStart {
        state: RtlSubMode::FinalDescent,
        state_complete: false,
        d_init_stopping_point: true,
        yaw: YawMode::Hold,
    }
}

/// Vehicle view `ModeRTL::descent_run` reads.
#[derive(Debug, Clone, Copy)]
pub struct RtlDescentView {
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `g.throttle_behavior`.
    pub throttle_behavior: i32,
    /// `copter.rc_throttle_control_in_filter.get()`.
    pub filtered_throttle_control_in: f32,
    /// `g.land_repositioning`.
    pub land_repositioning: bool,
    /// `copter.ap.land_repo_active` on entry.
    pub land_repo_active: bool,
    /// Pilot reposition velocity is zero.
    pub pilot_velocity_is_zero: bool,
    /// `rtl_path.descent_target.alt * 0.01`, metres.
    pub descent_target_alt_m: f32,
    /// `pos_control->get_pos_estimate_U_m()`.
    pub pos_u_m: f32,
}

impl RtlDescentView {
    /// Armed, auto-armed, airborne, no pilot intervention, 10 m still to go.
    #[must_use]
    pub const fn descending() -> Self {
        Self {
            armed: true,
            auto_armed: true,
            land_complete: false,
            has_valid_input: true,
            throttle_behavior: 0,
            filtered_throttle_control_in: 0.0,
            land_repositioning: false,
            land_repo_active: false,
            pilot_velocity_is_zero: true,
            descent_target_alt_m: 10.0,
            pos_u_m: 20.0,
        }
    }
}

/// Leftover of one `ModeRTL::descent_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlDescentRun {
    /// `is_disarmed_or_landed` fired; `make_safe_ground_handling` and return.
    pub safe_ground: bool,
    /// Raised stick asked Loiter, then AltHold, with `THROTTLE_LAND_ESCAPE`.
    pub cancel_escape: bool,
    /// `land_repo_active` after this tick.
    pub land_repo_active: bool,
    /// Spool ask on the flying path. `None` on the ground path.
    pub desired_spool: Option<DesiredSpoolState>,
    /// `input_vel_accel_NE_m` plus `NE_update_controller` on the flying path.
    pub input_vel_ne: bool,
    /// `D_set_alt_target_with_slew_m` plus `D_update_controller`.
    pub d_slew: bool,
    /// Within [`RTL_DESCENT_COMPLETE_M`] of the target. False on the ground
    /// path — the 20 cm check does not run before the early return.
    pub state_complete: bool,
}

/// The 20 cm FINAL_DESCENT arrival gate.
#[must_use]
pub fn rtl_descent_complete(descent_target_alt_m: f32, pos_u_m: f32) -> bool {
    libm::fabsf(descent_target_alt_m - pos_u_m) < RTL_DESCENT_COMPLETE_M
}

/// Upstream `ModeRTL::descent_run`.
#[must_use]
pub fn rtl_descent_run(view: &RtlDescentView) -> RtlDescentRun {
    if is_disarmed_or_landed(view.armed, view.auto_armed, view.land_complete) {
        return RtlDescentRun {
            safe_ground: true,
            cancel_escape: false,
            land_repo_active: view.land_repo_active,
            desired_spool: None,
            input_vel_ne: false,
            d_slew: false,
            state_complete: false,
        };
    }

    let cancel_escape = land_cancelled_by_throttle(
        view.throttle_behavior,
        view.filtered_throttle_control_in,
        view.has_valid_input,
    );

    let land_repo_active = view.land_repo_active
        || (view.has_valid_input && view.land_repositioning && !view.pilot_velocity_is_zero);

    RtlDescentRun {
        safe_ground: false,
        cancel_escape,
        land_repo_active,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
        input_vel_ne: true,
        d_slew: true,
        state_complete: rtl_descent_complete(view.descent_target_alt_m, view.pos_u_m),
    }
}

/// `RTL_CONE_SLOPE_DEFAULT`. Height / distance of the return cone.
pub const RTL_CONE_SLOPE_DEFAULT: f32 = 3.0;

/// `RTL_MIN_CONE_SLOPE`. Shallower slopes are ignored so the return
/// cannot crawl home at a few centimetres of climb.
pub const RTL_MIN_CONE_SLOPE: f32 = 0.5;

/// `ModeRTL::Option::IgnorePilotYaw`, bit 2 of `RTL_OPTIONS`.
pub const RTL_OPTION_IGNORE_PILOT_YAW: u32 = 1 << 2;

/// `ModeRTL::ReturnTargetAltType`.
///
/// This is not [`RtlAltType`]. That is the parameter. This is the leftover
/// of what `compute_return_target` actually flies after the terrain-source
/// switch and its fallbacks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlReturnAltType {
    /// Altitude above home.
    Relative,
    /// Altitude above terrain from the rangefinder.
    Rangefinder,
    /// Altitude above terrain from the terrain database.
    TerrainDatabase,
}

/// `AC_WPNav::TerrainSource` as `compute_return_target` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlTerrainSource {
    /// `TERRAIN_UNAVAILABLE`.
    Unavailable,
    /// `TERRAIN_FROM_RANGEFINDER`.
    Rangefinder,
    /// `TERRAIN_FROM_TERRAINDATABASE`.
    TerrainDatabase,
}

/// Leftover GCS / logger warn of `compute_return_target`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlPathWarn {
    /// No warning. The chosen alt type held.
    None,
    /// Terrain source unavailable, or the rangefinder was unhealthy.
    /// Both paths log `RTL_MISSING_RNGFND`.
    MissingRangefinder,
    /// Terrain-database frame change failed. Logs `MISSING_TERRAIN_DATA`.
    MissingTerrainData,
    /// Relative `ABOVE_HOME` conversion failed. Should never happen.
    UnexpectedTargetAlt,
}

/// Altitude frame the leftover parked a path point on.
///
/// This is not `Location::AltFrame`. The leftover records the *ask*, not
/// a converted Location: `change_alt_frame` needs origin / home / terrain
/// that this leftover does not own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlPathFrame {
    /// `Location::AltFrame::ABOVE_HOME`.
    AboveHome,
    /// `Location::AltFrame::ABOVE_TERRAIN`.
    AboveTerrain,
    /// `Location::AltFrame::ABOVE_ORIGIN`. Descent after the leftover change.
    AboveOrigin,
}

/// One RTL path point. Lat/lng are 1e-7 degrees; alt is metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlPathPoint {
    /// Latitude, 1e-7 degrees.
    pub lat: i32,
    /// Longitude, 1e-7 degrees.
    pub lng: i32,
    /// Altitude, metres, in [`RtlPathPoint::frame`].
    pub alt_m: f32,
    /// Frame the leftover wrote.
    pub frame: RtlPathFrame,
}

/// Vehicle view `ModeRTL::build_path` / `compute_return_target` read.
#[derive(Debug, Clone, Copy)]
pub struct RtlPathView {
    /// Stopping-point latitude after `get_stopping_point`.
    pub origin_lat: i32,
    /// Stopping-point longitude.
    pub origin_lng: i32,
    /// Rally-or-home return latitude.
    pub return_lat: i32,
    /// Rally-or-home return longitude.
    pub return_lng: i32,
    /// `return_target.alt` after a successful `ABOVE_HOME` change, centimetres.
    /// Used on the relative path. Home's stored alt is typically zero.
    pub relative_return_alt_cm: i32,
    /// `return_target.change_alt_frame(ABOVE_HOME)` succeeded.
    pub relative_frame_ok: bool,
    /// `copter.current_loc.alt * 0.01`, metres, before the pos-offset subtract.
    pub current_alt_m: f32,
    /// `pos_control->get_pos_offset_U_m()`.
    pub pos_offset_u_m: f32,
    /// `terrain_following_allowed` from init / restart.
    pub terrain_following_allowed: bool,
    /// `get_alt_type()`.
    pub rtl_alt_type: RtlAltType,
    /// `wp_nav->get_terrain_source()`.
    pub terrain_source: RtlTerrainSource,
    /// `get_rangefinder_height_interpolated_m` succeeded.
    pub rangefinder_ok: bool,
    /// Height that call wrote, metres. Only read when [`Self::rangefinder_ok`].
    pub rangefinder_height_m: f32,
    /// Both `current_loc.get_alt_m(ABOVE_TERRAIN)` and
    /// `return_target.change_alt_frame(ABOVE_TERRAIN)` succeeded.
    pub terrain_db_ok: bool,
    /// Current altitude above terrain, metres. Only read when db ok.
    pub terrain_db_current_alt_m: f32,
    /// `return_target.alt` after the ABOVE_TERRAIN change, centimetres.
    pub terrain_db_return_alt_cm: i32,
    /// `climb_min_m.get()`.
    pub climb_min_m: f32,
    /// `altitude_m.get()` — `RTL_ALT`.
    pub altitude_m: f32,
    /// `return_target.get_distance(origin_point)`, metres.
    pub return_dist_m: f32,
    /// `g.rtl_cone_slope`.
    pub cone_slope: f32,
    /// Alt-max fence is enabled.
    pub fence_alt_max: bool,
    /// `return_target.get_alt_m(fence_alt_max_frame, ...)` succeeded.
    /// The leftover then compares the just-computed target against
    /// [`Self::fence_alt_m`] — the usual case where the fence frame matches
    /// the return frame. A mismatched fence frame must not use this leftover
    /// unmodified.
    pub fence_alt_ok: bool,
    /// `fence.get_safe_alt_max_m()`.
    pub fence_alt_m: f32,
    /// `alt_final_m.get()`. Zero or below means land.
    pub alt_final_m: f32,
}

impl RtlPathView {
    /// Home return, 20 m current, default RTL_ALT / cone, land at the end.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            origin_lat: 0,
            origin_lng: 0,
            return_lat: 0,
            return_lng: 0,
            relative_return_alt_cm: 0,
            relative_frame_ok: true,
            current_alt_m: 20.0,
            pos_offset_u_m: 0.0,
            terrain_following_allowed: true,
            rtl_alt_type: RtlAltType::Relative,
            terrain_source: RtlTerrainSource::Unavailable,
            rangefinder_ok: false,
            rangefinder_height_m: 0.0,
            terrain_db_ok: false,
            terrain_db_current_alt_m: 0.0,
            terrain_db_return_alt_cm: 0,
            climb_min_m: RTL_CLIMB_MIN_M_DEFAULT,
            altitude_m: RTL_ALT_M_DEFAULT,
            return_dist_m: 100.0,
            cone_slope: RTL_CONE_SLOPE_DEFAULT,
            fence_alt_max: false,
            fence_alt_ok: false,
            fence_alt_m: 0.0,
            alt_final_m: RTL_ALT_FINAL_M_DEFAULT,
        }
    }
}

/// Leftover of `ModeRTL::build_path` / `compute_return_target`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlPath {
    /// Origin after `change_alt_frame(ABOVE_HOME)`.
    pub origin: RtlPathPoint,
    /// Climb target: origin lat/lng, return alt and frame.
    pub climb: RtlPathPoint,
    /// Return target after the altitude leftover.
    pub return_target: RtlPathPoint,
    /// Descent target after `change_alt_frame(ABOVE_ORIGIN)`.
    pub descent: RtlPathPoint,
    /// `alt_final_m <= 0`.
    pub land: bool,
    /// Alt type actually flown after fallbacks.
    pub alt_type: RtlReturnAltType,
    /// GCS / logger leftover. [`RtlPathWarn::None`] if the type held.
    pub warn: RtlPathWarn,
    /// The cone-slope trim ran (`cone_slope >= RTL_MIN_CONE_SLOPE`).
    pub cone_applied: bool,
    /// Fence leftover reduced the target.
    pub fence_reduced: bool,
    /// The no-descend clamp raised the target to current altitude.
    pub no_descend_raised: bool,
}

/// Pick `ReturnTargetAltType` and the current / seed altitudes.
fn rtl_return_alt_seed(view: &RtlPathView) -> (RtlReturnAltType, RtlPathWarn, f32, f32) {
    let mut curr_alt_m = view.current_alt_m - view.pos_offset_u_m;
    let mut alt_type = RtlReturnAltType::Relative;
    let mut warn = RtlPathWarn::None;

    if view.terrain_following_allowed && view.rtl_alt_type == RtlAltType::Terrain {
        match view.terrain_source {
            RtlTerrainSource::Unavailable => {
                alt_type = RtlReturnAltType::Relative;
                warn = RtlPathWarn::MissingRangefinder;
            }
            RtlTerrainSource::Rangefinder => {
                alt_type = RtlReturnAltType::Rangefinder;
            }
            RtlTerrainSource::TerrainDatabase => {
                alt_type = RtlReturnAltType::TerrainDatabase;
            }
        }
    }

    let seed_alt_m;

    if alt_type == RtlReturnAltType::Rangefinder {
        if view.rangefinder_ok {
            curr_alt_m = view.rangefinder_height_m - view.pos_offset_u_m;
            seed_alt_m = libm::fmaxf(
                curr_alt_m + libm::fmaxf(0.0, view.climb_min_m),
                libm::fmaxf(view.altitude_m, RTL_ALT_MIN_M),
            );
        } else {
            alt_type = RtlReturnAltType::Relative;
            warn = RtlPathWarn::MissingRangefinder;
            seed_alt_m = relative_seed(view, &mut warn);
        }
    } else if alt_type == RtlReturnAltType::TerrainDatabase {
        if view.terrain_db_ok {
            curr_alt_m = view.terrain_db_current_alt_m - view.pos_offset_u_m;
            seed_alt_m = libm::fmaxf(view.terrain_db_return_alt_cm as f32, 0.0) * 0.01;
        } else {
            alt_type = RtlReturnAltType::Relative;
            warn = RtlPathWarn::MissingTerrainData;
            seed_alt_m = relative_seed(view, &mut warn);
        }
    } else {
        seed_alt_m = relative_seed(view, &mut warn);
    }

    (alt_type, warn, curr_alt_m, seed_alt_m)
}

fn relative_seed(view: &RtlPathView, warn: &mut RtlPathWarn) -> f32 {
    if view.relative_frame_ok {
        libm::fmaxf(view.relative_return_alt_cm as f32, 0.0) * 0.01
    } else {
        if *warn == RtlPathWarn::None {
            *warn = RtlPathWarn::UnexpectedTargetAlt;
        }
        0.0
    }
}

/// The return-altitude leftover shared by every alt type.
///
/// Upstream is the block after the RELATIVE conversion: raise to
/// `max(RTL_ALT, min_rtl)`, trim by the cone, clamp to the fence, then
/// refuse to descend below current.
fn rtl_raise_return_alt(
    seed_alt_m: f32,
    curr_alt_m: f32,
    view: &RtlPathView,
) -> (f32, bool, bool, bool) {
    let min_rtl_alt_m = libm::fmaxf(
        RTL_ALT_MIN_M,
        curr_alt_m + libm::fmaxf(0.0, view.climb_min_m),
    );
    let mut target_alt_m = libm::fmaxf(seed_alt_m, libm::fmaxf(view.altitude_m, min_rtl_alt_m));

    let cone_applied = view.cone_slope >= RTL_MIN_CONE_SLOPE;
    if cone_applied {
        target_alt_m = libm::fminf(
            target_alt_m,
            libm::fmaxf(view.return_dist_m * view.cone_slope, min_rtl_alt_m),
        );
    }

    let mut fence_reduced = false;
    if view.fence_alt_max && view.fence_alt_ok && target_alt_m > view.fence_alt_m {
        target_alt_m = view.fence_alt_m;
        fence_reduced = true;
    }

    let no_descend_raised = target_alt_m < curr_alt_m;
    if no_descend_raised {
        target_alt_m = curr_alt_m;
    }

    (target_alt_m, cone_applied, fence_reduced, no_descend_raised)
}

/// Upstream `ModeRTL::build_path` including `compute_return_target`.
///
/// Origin / climb / descent geometry is the leftover of `build_path`.
/// The altitude machine is the leftover of `compute_return_target`. Rally
/// vs home is not here — the view already holds the return lat/lng.
#[must_use]
pub fn rtl_build_path(view: &RtlPathView) -> RtlPath {
    let (alt_type, warn, curr_alt_m, seed_alt_m) = rtl_return_alt_seed(view);
    let (return_alt_m, cone_applied, fence_reduced, no_descend_raised) =
        rtl_raise_return_alt(seed_alt_m, curr_alt_m, view);

    let return_frame = if alt_type == RtlReturnAltType::Relative {
        RtlPathFrame::AboveHome
    } else {
        RtlPathFrame::AboveTerrain
    };

    let origin = RtlPathPoint {
        lat: view.origin_lat,
        lng: view.origin_lng,
        alt_m: 0.0,
        frame: RtlPathFrame::AboveHome,
    };
    let return_target = RtlPathPoint {
        lat: view.return_lat,
        lng: view.return_lng,
        alt_m: return_alt_m,
        frame: return_frame,
    };
    let climb = RtlPathPoint {
        lat: view.origin_lat,
        lng: view.origin_lng,
        alt_m: return_alt_m,
        frame: return_frame,
    };
    let descent = RtlPathPoint {
        lat: view.return_lat,
        lng: view.return_lng,
        alt_m: view.alt_final_m,
        frame: RtlPathFrame::AboveOrigin,
    };

    RtlPath {
        origin,
        climb,
        return_target,
        descent,
        land: view.alt_final_m <= 0.0,
        alt_type,
        warn,
        cone_applied,
        fence_reduced,
        no_descend_raised,
    }
}

/// Leftover of `ModeRTL::land_start`.
///
/// The first-switch walk in [`rtl_run`] already parks `_state` on
/// [`RtlSubMode::Land`]. This leftover is the controller seed that walk
/// does not own: NE / D limits from the waypoint navigator, init if
/// inactive, hold yaw. Landing-gear deploy is not here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlLandStart {
    /// Always [`RtlSubMode::Land`].
    pub state: RtlSubMode,
    /// Always false.
    pub state_complete: bool,
    /// Horizontal max / correction speed from `wp_nav`.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel from `wp_nav`.
    pub ne_accel_mss: f32,
    /// `NE_init_controller` — only when NE was inactive.
    pub init_ne: bool,
    /// `D_init_controller` — only when D was inactive.
    pub init_d: bool,
    /// Always [`YawMode::Hold`].
    pub yaw: YawMode,
}

/// Vehicle view `ModeRTL::land_start` reads.
#[derive(Debug, Clone, Copy)]
pub struct RtlLandStartView {
    /// `pos_control->NE_is_active()`.
    pub ne_is_active: bool,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_NE_ms()`.
    pub speed_ne_ms: f32,
    /// `wp_nav->get_wp_acceleration_mss()`.
    pub wp_accel_mss: f32,
}

impl RtlLandStartView {
    /// Both controllers already running, default WP speeds.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            ne_is_active: true,
            d_is_active: true,
            speed_ne_ms: 5.0,
            wp_accel_mss: 1.0,
        }
    }
}

/// Upstream `ModeRTL::land_start`.
#[must_use]
pub const fn rtl_land_start(view: &RtlLandStartView) -> RtlLandStart {
    RtlLandStart {
        state: RtlSubMode::Land,
        state_complete: false,
        ne_speed_ms: view.speed_ne_ms,
        ne_accel_mss: view.wp_accel_mss,
        init_ne: !view.ne_is_active,
        init_d: !view.d_is_active,
        yaw: YawMode::Hold,
    }
}

/// Vehicle view `ModeRTL::land_run` reads.
#[derive(Debug, Clone, Copy)]
pub struct RtlLandView {
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// The `disarm_on_land` argument. Bare `run()` passes `true`.
    pub disarm_on_land: bool,
}

impl RtlLandView {
    /// Armed, auto-armed, airborne, disarm-on-land.
    #[must_use]
    pub const fn landing() -> Self {
        Self {
            armed: true,
            auto_armed: true,
            land_complete: false,
            spool_state: SpoolState::ThrottleUnlimited,
            disarm_on_land: true,
        }
    }
}

/// Leftover of one `ModeRTL::land_run` tick.
///
/// RTL land is not [`crate::mode_land::land_run`]. There is no pause, no
/// no-GPS runner, no throttle-cancel. `_state_complete` is
/// `land_complete`, then the GPS landing leftover
/// `land_run_normal_or_precland()` with no pause argument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlLandRun {
    /// `_state_complete` after this tick. Always `land_complete`.
    pub state_complete: bool,
    /// `disarm_on_land && land_complete && GROUND_IDLE` asked `disarm(LANDED)`.
    pub disarm_landed: bool,
    /// `is_disarmed_or_landed` fired; `make_safe_ground_handling` and return.
    pub safe_ground: bool,
    /// Spool ask on the flying path. `None` on the ground path.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Flying path asked `land_run_normal_or_precland()` (pause default false).
    pub land_normal_or_precland: bool,
}

/// Upstream `ModeRTL::land_run`.
#[must_use]
pub fn rtl_land_run(view: &RtlLandView) -> RtlLandRun {
    let disarm_landed =
        view.disarm_on_land && view.land_complete && view.spool_state == SpoolState::GroundIdle;
    if is_disarmed_or_landed(view.armed, view.auto_armed, view.land_complete) {
        return RtlLandRun {
            state_complete: view.land_complete,
            disarm_landed,
            safe_ground: true,
            desired_spool: None,
            land_normal_or_precland: false,
        };
    }
    RtlLandRun {
        state_complete: view.land_complete,
        disarm_landed,
        safe_ground: false,
        desired_spool: Some(DesiredSpoolState::ThrottleUnlimited),
        land_normal_or_precland: true,
    }
}

/// `ModeRTL::option_is_enabled`.
#[must_use]
pub const fn rtl_option_is_enabled(rtl_options: u32, option: u32) -> bool {
    (rtl_options & option) != 0
}

/// `ModeRTL::use_pilot_yaw`.
///
/// Descent and land use Land's leftover ([`crate::mode_land::land_use_pilot_yaw`]).
/// Every earlier stage uses the RTL option bit.
#[must_use]
pub const fn rtl_use_pilot_yaw(
    state: RtlSubMode,
    land_repositioning: bool,
    rtl_options: u32,
) -> bool {
    if matches!(state, RtlSubMode::FinalDescent | RtlSubMode::Land) {
        land_repositioning
    } else {
        !rtl_option_is_enabled(rtl_options, RTL_OPTION_IGNORE_PILOT_YAW)
    }
}

/// `ModeRTL::get_wp`. Whether the leftover asks `wp_nav` for the OA dest.
///
/// LAND has no waypoint destination. Every other submode does.
#[must_use]
pub const fn rtl_get_wp(state: RtlSubMode) -> bool {
    !matches!(state, RtlSubMode::Land)
}
