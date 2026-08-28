//! QRTL `_enter` plus `run()` climb-then-return, upstream
//! `ArduPlane/mode_qrtl.cpp` (Plane-4.7.0).
//!
//! Tracked as **VT-006**. `Mode::enter` always calls
//! [`QuadPlane::mode_enter`] then [`qrtl_enter`]. Guided-wait takeoff
//! on mode enter is treated as QLAND (`QLAND_INSTEAD_OF_RTL`) so a
//! failsafe during GUIDED→AUTO takeoff does not try to fly home.
//!
//! Otherwise `_enter` starts in [`QrtlSubMode::Rtl`], picks
//! home vs nearest rally (`calc_best_rally_or_home_location`), and —
//! when VTOL motors are already `THROTTLE_UNLIMITED` — climbs the
//! QRTL cone (`Q_RTL_ALT` / `Q_RTL_ALT_MIN` / `Q_LAND_FINAL_ALT`)
//! before returning. Close-in and already above the cone jumps to
//! `QPOS_POSITION1` and `do_RTL` at the lower of QRTL alt and the
//! current absolute altitude.
//!
//! This slice is [`qrtl_run`] (`ModeQRTL::run`): tailsitter FW pull-up
//! runs `Mode::run()`; [`QrtlSubMode::Climb`] holds XY, climbs at
//! `Q_WP_SPD_UP`, and when the stopping point reaches the climb
//! waypoint switches to [`QrtlSubMode::Rtl`] (`do_RTL`, maybe
//! `QPOS_POSITION1` if already inside the VTOL return radius).
//! Already-in-RTL ticks run `vtol_position_controller` plus FW
//! stabilize. This slice is the QRTL land handoff: once
//! `poscontrol` is at or past `QPOS_POSITION2`, `verify_vtol_land`
//! starts the descent and states past POSITION2 copy home altitude
//! onto `next_WP_loc`. Approach / airbrake allow FBW stick mixing.
//! Also `update()` (QStabilize), `update_target_altitude()`,
//! `allows_throttle_nudging()`, and the `mode_qrtl.cpp` completeness
//! table. This module does not rewrite [`crate::mode_q`] or
//! [`crate::landing`].

use crate::auto_vtol::{VerifyLandResult, VerifyLandView};
use crate::poscontrol::PositionControlState;
use crate::QuadPlane;

/// `Mode::Number::QRTL`.
pub const MODE_QRTL: u8 = 21;

/// Default `Q_RTL_ALT`, upstream `AP_GROUPINFO("RTL_ALT", ..., qrtl_alt_m, 15)`.
pub const Q_RTL_ALT_DEFAULT_M: f32 = 15.0;

/// Default `Q_RTL_ALT_MIN`, upstream `AP_GROUPINFO("RTL_ALT_MIN", ..., qrtl_alt_min_m, 10)`.
pub const Q_RTL_ALT_MIN_DEFAULT_M: f32 = 10.0;

/// Default `WP_LOITER_RAD`, upstream `LOITER_RADIUS_DEFAULT`.
pub const WP_LOITER_RAD_DEFAULT_M: f32 = 60.0;

/// Default `RTL_RADIUS` (metres). Zero means "use `WP_LOITER_RAD`".
pub const RTL_RADIUS_DEFAULT_M: f32 = 0.0;

/// Default `Q_WP_SPD_UP` / `wp_nav->get_default_speed_up_ms()`.
///
/// Upstream `AC_WPNav` `WP_SPD_UP_DEFAULT` is 2.5 m/s.
pub const Q_WP_SPD_UP_DEFAULT_MS: f32 = 2.5;

/// `ModeQRTL::SubMode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QrtlSubMode {
    /// Climb the QRTL cone before heading home / rally.
    Climb,
    /// `do_RTL` + VTOL position controller.
    Rtl,
}

/// Which destination `calc_best_rally_or_home_location` selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QrtlDestination {
    /// AHRS home at `Q_RTL_ALT` AMSL.
    Home,
    /// Nearest rally, closer than home (or home excluded).
    Rally,
}

/// Side-effect path `_enter` took.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QrtlEnterAction {
    /// `guided_wait_takeoff_on_mode_enter` → `set_mode(QLAND)`.
    QLandInstead,
    /// `submode = climb`; `next_WP_loc` is current loc plus climb.
    Climb,
    /// `do_RTL` + `poscontrol_init_approach`.
    Rtl,
}

/// Plane-side inputs `ModeQRTL::_enter` reads.
///
/// This crate does not own `plane.current_loc` / `plane.rally` /
/// motors spool, so the caller passes them here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlEnterView {
    /// `motors->get_desired_spool_state() == THROTTLE_UNLIMITED`.
    pub throttle_unlimited: bool,
    /// Horizontal distance from `current_loc` to home, metres.
    pub home_dist_m: f32,
    /// Distance to the nearest valid rally, metres. `None` if none.
    pub rally_dist_m: Option<f32>,
    /// `RALLY_INCL_HOME` — when false a rally always wins over home.
    pub rally_incl_home: bool,
    /// `WP_LOITER_RAD`, metres (signed; radius uses `fabsf`).
    pub loiter_radius_m: f32,
    /// `RTL_RADIUS`, metres (signed; radius uses `fabsf`).
    pub rtl_radius_m: f32,
    /// `Q_RTL_ALT`, metres.
    pub qrtl_alt_m: f32,
    /// `Q_RTL_ALT_MIN`, metres.
    pub qrtl_alt_min_m: f32,
    /// `relative_ground_altitude(RangeFinderUse::CLIMB)`, metres.
    pub relative_ground_alt_m: f32,
    /// `current_loc` absolute altitude, centimetres.
    pub current_alt_abs_cm: i32,
    /// `plane.home.alt`, centimetres AMSL.
    pub home_alt_abs_cm: i32,
}

impl Default for QrtlEnterView {
    fn default() -> Self {
        Self::new()
    }
}

impl QrtlEnterView {
    /// Far from home, VTOL motors up, below the QRTL cone.
    ///
    /// `WP_LOITER_RAD` 60 → return radius 90 m; `home_dist` 200 m;
    /// AGL 5 m vs `Q_RTL_ALT` 15 m → climb.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            throttle_unlimited: true,
            home_dist_m: 200.0,
            rally_dist_m: None,
            rally_incl_home: true,
            loiter_radius_m: WP_LOITER_RAD_DEFAULT_M,
            rtl_radius_m: RTL_RADIUS_DEFAULT_M,
            qrtl_alt_m: Q_RTL_ALT_DEFAULT_M,
            qrtl_alt_min_m: Q_RTL_ALT_MIN_DEFAULT_M,
            relative_ground_alt_m: 5.0,
            current_alt_abs_cm: 500,
            home_alt_abs_cm: 0,
        }
    }

    /// Far from home, already above the cone — RTL without climb.
    #[must_use]
    pub const fn far_above_cone() -> Self {
        let mut view = Self::new();
        view.relative_ground_alt_m = 20.0;
        view.current_alt_abs_cm = 2000;
        view
    }

    /// Inside the VTOL return radius and above the cone — `QPOS_POSITION1`.
    #[must_use]
    pub const fn close_above_cone() -> Self {
        let mut view = Self::new();
        view.home_dist_m = 50.0;
        view.relative_ground_alt_m = 12.0;
        view.current_alt_abs_cm = 1200;
        view
    }

    /// Fixed-wing (motors not unlimited) — skip the climb cone.
    #[must_use]
    pub const fn forward_flight() -> Self {
        let mut view = Self::new();
        view.throttle_unlimited = false;
        view
    }
}

/// Outcome of combined `Mode::enter` for QRTL (`mode_enter` then `_enter`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlEnter {
    /// Upstream `_enter` always returns true.
    pub accepted: bool,
    /// Path taken after the guided-wait / cone tests.
    pub action: QrtlEnterAction,
    /// Latched `ModeQRTL::submode` (`Climb` unused on the QLAND path).
    pub submode: QrtlSubMode,
    /// Home vs rally selected for the return.
    pub dest: QrtlDestination,
    /// Horizontal distance to that destination, metres.
    pub dist_m: f32,
    /// [`qrtl_vtol_return_radius_m`].
    pub radius_m: f32,
    /// `home.alt + Q_RTL_ALT * 100`, maybe lowered when close-in.
    pub rtl_alt_abs_cm: i32,
    /// Climb-cone target AGL, metres (`MAX(cone, min_climb)`).
    pub climb_target_alt_m: f32,
    /// `target_alt - relative_ground_altitude`.
    pub dist_to_climb_m: f32,
    /// `next_WP_loc.alt` on the climb path (`current + dist_to_climb`).
    pub climb_next_wp_alt_cm: i32,
    /// `poscontrol.set_state(QPOS_POSITION1)` this enter.
    pub position1: bool,
    /// `plane.do_RTL(RTL_alt_abs_cm)` this enter.
    pub do_rtl: bool,
    /// `quadplane.poscontrol_init_approach()` this enter.
    pub poscontrol_init_approach: bool,
    /// `poscontrol.slow_descent` (`from_alt > to_alt`).
    pub slow_descent: bool,
}

impl QrtlEnter {
    fn qland_instead() -> Self {
        Self {
            accepted: true,
            action: QrtlEnterAction::QLandInstead,
            submode: QrtlSubMode::Rtl,
            dest: QrtlDestination::Home,
            dist_m: 0.0,
            radius_m: 0.0,
            rtl_alt_abs_cm: 0,
            climb_target_alt_m: 0.0,
            dist_to_climb_m: 0.0,
            climb_next_wp_alt_cm: 0,
            position1: false,
            do_rtl: false,
            poscontrol_init_approach: false,
            slow_descent: false,
        }
    }
}

/// `ModeQRTL` class predicates from `mode.h`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModeQrtl;

impl ModeQrtl {
    /// Upstream `mode_number()` — `Number::QRTL`.
    #[must_use]
    pub const fn mode_number() -> u8 {
        MODE_QRTL
    }

    /// Upstream `is_vtol_mode()` — true.
    #[must_use]
    pub const fn is_vtol_mode() -> bool {
        true
    }

    /// Upstream does not override `is_vtol_man_mode` (base `false`).
    #[must_use]
    pub const fn is_vtol_man_mode() -> bool {
        false
    }

    /// Upstream `does_auto_throttle()` — true.
    #[must_use]
    pub const fn does_auto_throttle() -> bool {
        true
    }

    /// Upstream `ModeQRTL::_pre_arm_checks` — always `false`.
    #[must_use]
    pub const fn pre_arm_checks() -> bool {
        false
    }
}

/// Upstream `ModeQRTL::get_VTOL_return_radius`.
///
/// `MAX(fabsf(WP_LOITER_RAD), fabsf(RTL_RADIUS)) * 1.5`.
#[must_use]
pub fn qrtl_vtol_return_radius_m(loiter_radius_m: f32, rtl_radius_m: f32) -> f32 {
    let loiter = if loiter_radius_m < 0.0 {
        -loiter_radius_m
    } else {
        loiter_radius_m
    };
    let rtl = if rtl_radius_m < 0.0 {
        -rtl_radius_m
    } else {
        rtl_radius_m
    };
    let larger = if loiter > rtl { loiter } else { rtl };
    larger * 1.5
}

/// `constrain_float(Q_RTL_ALT_MIN, Q_LAND_FINAL_ALT, Q_RTL_ALT)`.
#[must_use]
pub fn qrtl_min_climb_m(qrtl_alt_min_m: f32, land_final_alt_m: f32, qrtl_alt_m: f32) -> f32 {
    constrain_f32(qrtl_alt_min_m, land_final_alt_m, qrtl_alt_m)
}

/// Climb-cone target AGL: `MAX(Q_RTL_ALT * (dist / MAX(radius, dist)), min_climb)`.
#[must_use]
pub fn qrtl_climb_cone_target_alt_m(
    qrtl_alt_m: f32,
    dist_m: f32,
    radius_m: f32,
    min_climb_m: f32,
) -> f32 {
    let denom = if radius_m > dist_m { radius_m } else { dist_m };
    let cone = if denom > 0.0 {
        qrtl_alt_m * (dist_m / denom)
    } else {
        0.0
    };
    if cone > min_climb_m {
        cone
    } else {
        min_climb_m
    }
}

/// Upstream `Plane::calc_best_rally_or_home_location` distance pick.
///
/// No rally → home. A rally wins when it is closer than home, or when
/// `RALLY_INCL_HOME` is false. Equal distances keep home (`<`, not `<=`).
#[must_use]
pub fn calc_best_rally_or_home(
    home_dist_m: f32,
    rally_dist_m: Option<f32>,
    rally_incl_home: bool,
) -> (QrtlDestination, f32) {
    match rally_dist_m {
        Some(rally_m) if !rally_incl_home || rally_m < home_dist_m => {
            (QrtlDestination::Rally, rally_m)
        }
        _ => (QrtlDestination::Home, home_dist_m),
    }
}

/// Combined `Mode::enter` for QRTL: `mode_enter` then `_enter`.
///
/// Always returns [`QrtlEnter::accepted`] true, matching upstream.
pub fn qrtl_enter(qp: &mut QuadPlane, view: QrtlEnterView) -> QrtlEnter {
    qp.mode_enter();
    if qp.guided_wait_takeoff_on_mode_enter() {
        return QrtlEnter::qland_instead();
    }

    let (dest, dist_m) =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home);
    let radius_m = qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m);
    let min_climb_m = qrtl_min_climb_m(view.qrtl_alt_min_m, qp.land_final_alt_m(), view.qrtl_alt_m);
    let climb_target_alt_m =
        qrtl_climb_cone_target_alt_m(view.qrtl_alt_m, dist_m, radius_m, min_climb_m);
    let dist_to_climb_m = climb_target_alt_m - view.relative_ground_alt_m;
    let mut rtl_alt_abs_cm = view.home_alt_abs_cm + (view.qrtl_alt_m * 100.0) as i32;
    let climb_next_wp_alt_cm = view.current_alt_abs_cm + (dist_to_climb_m * 100.0) as i32;

    if view.throttle_unlimited && is_positive(dist_to_climb_m) {
        return QrtlEnter {
            accepted: true,
            action: QrtlEnterAction::Climb,
            submode: QrtlSubMode::Climb,
            dest,
            dist_m,
            radius_m,
            rtl_alt_abs_cm,
            climb_target_alt_m,
            dist_to_climb_m,
            climb_next_wp_alt_cm,
            position1: false,
            do_rtl: false,
            poscontrol_init_approach: false,
            slow_descent: false,
        };
    }

    let mut position1 = false;
    if view.throttle_unlimited && dist_m < radius_m {
        if view.current_alt_abs_cm < rtl_alt_abs_cm {
            rtl_alt_abs_cm = view.current_alt_abs_cm;
        }
        qp.poscontrol_mut()
            .set_state(PositionControlState::Position1);
        position1 = true;
    }

    QrtlEnter {
        accepted: true,
        action: QrtlEnterAction::Rtl,
        submode: QrtlSubMode::Rtl,
        dest,
        dist_m,
        radius_m,
        rtl_alt_abs_cm,
        climb_target_alt_m,
        dist_to_climb_m,
        climb_next_wp_alt_cm,
        position1,
        do_rtl: true,
        poscontrol_init_approach: true,
        slow_descent: view.current_alt_abs_cm > rtl_alt_abs_cm,
    }
}

/// Which path `ModeQRTL::run` took this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QrtlRunAction {
    /// Tailsitter FW pull-up: `Mode::run()`.
    FwControllers,
    /// `SubMode::climb` — hold XY, climb at WP speed-up.
    Climb,
    /// Climb finished this tick — switched to RTL and `do_RTL`.
    ClimbThenReturn,
    /// Already in `SubMode::RTL` — `vtol_position_controller`.
    Return,
}

/// Outcome of one [`qrtl_run`] tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlRun {
    /// FW pull-up / climb / climb-then-return / already returning.
    pub action: QrtlRunAction,
    /// Latched `ModeQRTL::submode` after this tick.
    pub submode: QrtlSubMode,
    /// Home vs rally selected when heading home this tick.
    pub dest: QrtlDestination,
    /// Horizontal distance to that destination, metres.
    pub dist_m: f32,
    /// [`qrtl_vtol_return_radius_m`].
    pub radius_m: f32,
    /// `set_climb_rate_ms(wp_nav->get_default_speed_up_ms())`.
    pub climb_rate_ms: f32,
    /// `input_vel_accel_NE_m(0, 0)` + `run_xy_controller()`.
    pub xy_hold: bool,
    /// `assign_tilt_to_fwd_thr()` on the climb branch.
    pub tilt_assigned: bool,
    /// `set_VTOL_roll_pitch_limit` → `NE_set_externally_limited()`.
    pub ne_externally_limited: bool,
    /// Weathervane yaw (no pilot input) on the climb branch.
    pub weathervane: bool,
    /// `run_z_controller()` on the climb branch.
    pub z_controller: bool,
    /// `poscontrol.set_state(QPOS_POSITION1)` this tick.
    pub position1: bool,
    /// `plane.do_RTL(RTL_alt_abs_cm)` this tick.
    pub do_rtl: bool,
    /// `quadplane.poscontrol_init_approach()` this tick.
    pub poscontrol_init_approach: bool,
    /// `poscontrol.slow_descent` after the climb→RTL switch.
    pub slow_descent: bool,
    /// `home.alt + Q_RTL_ALT * 100`, maybe lowered when close-in.
    pub rtl_alt_abs_cm: i32,
    /// `vtol_position_controller()` on the already-RTL branch.
    pub vtol_position_controller: bool,
    /// `stabilize_roll/pitch/yaw` after the submode switch.
    pub fw_stabilize: bool,
    /// `next_WP_loc.copy_alt_from(home)` when `poscontrol > QPOS_POSITION2`.
    pub copy_home_alt: bool,
    /// `verify_vtol_land` when `poscontrol >= QPOS_POSITION2`.
    pub verify_vtol_land: bool,
    /// Approach / airbrake `stabilize_stick_mixing_fbw`.
    pub stick_mixing_fbw: bool,
}

impl QrtlRun {
    const fn fw_controllers() -> Self {
        Self {
            action: QrtlRunAction::FwControllers,
            submode: QrtlSubMode::Climb,
            dest: QrtlDestination::Home,
            dist_m: 0.0,
            radius_m: 0.0,
            climb_rate_ms: 0.0,
            xy_hold: false,
            tilt_assigned: false,
            ne_externally_limited: false,
            weathervane: false,
            z_controller: false,
            position1: false,
            do_rtl: false,
            poscontrol_init_approach: false,
            slow_descent: false,
            rtl_alt_abs_cm: 0,
            vtol_position_controller: false,
            fw_stabilize: false,
            copy_home_alt: false,
            verify_vtol_land: false,
            stick_mixing_fbw: false,
        }
    }
}

/// Plane / nav view [`qrtl_run`] reads.
///
/// This crate does not own `plane.current_loc` / pos-control stopping
/// points, so the caller passes them here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlRunView {
    /// `tailsitter.in_vtol_transition(now)` — FW pull-up early-out.
    pub tailsitter_in_vtol_transition: bool,
    /// Latched `ModeQRTL::submode` at the start of this tick.
    pub submode: QrtlSubMode,
    /// Horizontal distance from `current_loc` to home, metres.
    pub home_dist_m: f32,
    /// Distance to the nearest valid rally, metres. `None` if none.
    pub rally_dist_m: Option<f32>,
    /// `RALLY_INCL_HOME` — when false a rally always wins over home.
    pub rally_incl_home: bool,
    /// `WP_LOITER_RAD`, metres (signed; radius uses `fabsf`).
    pub loiter_radius_m: f32,
    /// `RTL_RADIUS`, metres (signed; radius uses `fabsf`).
    pub rtl_radius_m: f32,
    /// `Q_RTL_ALT`, metres.
    pub qrtl_alt_m: f32,
    /// `current_loc` absolute altitude, centimetres.
    pub current_alt_abs_cm: i32,
    /// `plane.home.alt`, centimetres AMSL.
    pub home_alt_abs_cm: i32,
    /// Climb-waypoint / `next_WP_loc` absolute altitude, centimetres.
    pub next_wp_alt_abs_cm: i32,
    /// `stopping_loc.get_height_above(next_WP_loc)`. `None` if the
    /// height lookup fails (upstream treats that as climb finished).
    pub stopping_height_above_next_wp_m: Option<f32>,
    /// `current_loc.get_height_above(next_WP_loc)` after `do_RTL`.
    /// `None` falls back to comparing absolute altitudes.
    pub current_height_above_next_wp_m: Option<f32>,
    /// `transition->set_VTOL_roll_pitch_limit` returned true.
    pub vtol_roll_pitch_limited: bool,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub wp_speed_up_ms: f32,
    /// Snapshot for [`QuadPlane::verify_vtol_land`] on the land-handoff path.
    pub land: Option<VerifyLandView>,
}

impl QrtlRunView {
    /// Mid-climb tick: still below the climb waypoint, far from home.
    #[must_use]
    pub const fn climbing() -> Self {
        Self {
            tailsitter_in_vtol_transition: false,
            submode: QrtlSubMode::Climb,
            home_dist_m: 200.0,
            rally_dist_m: None,
            rally_incl_home: true,
            loiter_radius_m: WP_LOITER_RAD_DEFAULT_M,
            rtl_radius_m: RTL_RADIUS_DEFAULT_M,
            qrtl_alt_m: Q_RTL_ALT_DEFAULT_M,
            current_alt_abs_cm: 1000,
            home_alt_abs_cm: 0,
            next_wp_alt_abs_cm: 1500,
            stopping_height_above_next_wp_m: Some(-5.0),
            current_height_above_next_wp_m: None,
            vtol_roll_pitch_limited: false,
            wp_speed_up_ms: Q_WP_SPD_UP_DEFAULT_MS,
            land: None,
        }
    }

    /// Stopping point has reached the climb waypoint, still far from home.
    #[must_use]
    pub const fn climb_done_far() -> Self {
        let mut view = Self::climbing();
        view.current_alt_abs_cm = 1500;
        view.stopping_height_above_next_wp_m = Some(1.0);
        view.current_height_above_next_wp_m = Some(0.0);
        view
    }

    /// Climb finished inside the VTOL return radius — `QPOS_POSITION1`.
    #[must_use]
    pub const fn climb_done_close() -> Self {
        let mut view = Self::climb_done_far();
        view.home_dist_m = 50.0;
        view.next_wp_alt_abs_cm = 1200;
        view.current_alt_abs_cm = 1200;
        view
    }

    /// Already in `SubMode::RTL` — position-controller return.
    #[must_use]
    pub const fn returning() -> Self {
        let mut view = Self::climbing();
        view.submode = QrtlSubMode::Rtl;
        view.stopping_height_above_next_wp_m = None;
        view
    }

    /// Tailsitter FW pull-up phase of VTOL transition.
    #[must_use]
    pub const fn tailsitter_fw_transition() -> Self {
        let mut view = Self::climbing();
        view.tailsitter_in_vtol_transition = true;
        view
    }
}

/// Upstream climb-done test: `!get_height_above(...) || is_positive(alt_diff)`.
///
/// A failed height lookup (`None`) heads home. A positive difference
/// means the stopping point is already above the climb waypoint.
#[must_use]
pub const fn qrtl_climb_finished(stopping_height_above_next_wp_m: Option<f32>) -> bool {
    match stopping_height_above_next_wp_m {
        None => true,
        Some(alt_diff) => alt_diff > 0.0,
    }
}

/// Combined `ModeQRTL::run`: tailsitter FW pull-up, climb-then-return,
/// or the already-RTL position-controller path (including land handoff).
///
/// FW stabilize runs after the submode switch, matching upstream,
/// except the tailsitter early-out which `return`s from `Mode::run()`.
pub fn qrtl_run(qp: &mut QuadPlane, view: QrtlRunView) -> QrtlRun {
    if view.tailsitter_in_vtol_transition {
        return QrtlRun::fw_controllers();
    }

    match view.submode {
        QrtlSubMode::Climb => run_climb(qp, view),
        QrtlSubMode::Rtl => run_return(qp, view),
    }
}

fn run_climb(qp: &mut QuadPlane, view: QrtlRunView) -> QrtlRun {
    let ne_externally_limited = view.vtol_roll_pitch_limited;
    if !qrtl_climb_finished(view.stopping_height_above_next_wp_m) {
        return QrtlRun {
            action: QrtlRunAction::Climb,
            submode: QrtlSubMode::Climb,
            dest: QrtlDestination::Home,
            dist_m: view.home_dist_m,
            radius_m: qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m),
            climb_rate_ms: view.wp_speed_up_ms,
            xy_hold: true,
            tilt_assigned: true,
            ne_externally_limited,
            weathervane: true,
            z_controller: true,
            position1: false,
            do_rtl: false,
            poscontrol_init_approach: false,
            slow_descent: false,
            rtl_alt_abs_cm: view.home_alt_abs_cm + (view.qrtl_alt_m * 100.0) as i32,
            vtol_position_controller: false,
            fw_stabilize: true,
            copy_home_alt: false,
            verify_vtol_land: false,
            stick_mixing_fbw: false,
        };
    }

    let (dest, dist_m) =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home);
    let radius_m = qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m);
    let mut rtl_alt_abs_cm = view.home_alt_abs_cm + (view.qrtl_alt_m * 100.0) as i32;
    let mut position1 = false;
    if dist_m < radius_m {
        if view.next_wp_alt_abs_cm < rtl_alt_abs_cm {
            rtl_alt_abs_cm = view.next_wp_alt_abs_cm;
        }
        qp.poscontrol_mut()
            .set_state(PositionControlState::Position1);
        position1 = true;
    }

    let slow_descent = match view.current_height_above_next_wp_m {
        Some(alt_diff) => is_positive(alt_diff),
        None => view.current_alt_abs_cm > rtl_alt_abs_cm,
    };

    QrtlRun {
        action: QrtlRunAction::ClimbThenReturn,
        submode: QrtlSubMode::Rtl,
        dest,
        dist_m,
        radius_m,
        climb_rate_ms: view.wp_speed_up_ms,
        xy_hold: true,
        tilt_assigned: true,
        ne_externally_limited,
        weathervane: true,
        z_controller: true,
        position1,
        do_rtl: true,
        poscontrol_init_approach: true,
        slow_descent,
        rtl_alt_abs_cm,
        vtol_position_controller: false,
        fw_stabilize: true,
        copy_home_alt: false,
        verify_vtol_land: false,
        stick_mixing_fbw: false,
    }
}

fn run_return(qp: &mut QuadPlane, view: QrtlRunView) -> QrtlRun {
    let (dest, dist_m) =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home);
    let handoff = qrtl_land_handoff(qp, view.land);
    QrtlRun {
        action: QrtlRunAction::Return,
        submode: QrtlSubMode::Rtl,
        dest,
        dist_m,
        radius_m: qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m),
        climb_rate_ms: 0.0,
        xy_hold: false,
        tilt_assigned: false,
        ne_externally_limited: false,
        weathervane: false,
        z_controller: false,
        position1: false,
        do_rtl: false,
        poscontrol_init_approach: false,
        slow_descent: false,
        rtl_alt_abs_cm: view.home_alt_abs_cm + (view.qrtl_alt_m * 100.0) as i32,
        vtol_position_controller: true,
        fw_stabilize: true,
        copy_home_alt: handoff.copy_home_alt,
        verify_vtol_land: handoff.verify_vtol_land,
        stick_mixing_fbw: handoff.stick_mixing_fbw,
    }
}


/// Default `RTL_ALTITUDE`, upstream `AP_GROUPINFO("RTL_ALTITUDE", ..., 100)`.
pub const RTL_ALTITUDE_DEFAULT_M: f32 = 100.0;

/// Default `AIRSPEED_CRUISE` used by the QRTL approach profile stub.
pub const AIRSPEED_CRUISE_DEFAULT_MS: f32 = 12.0;

/// Default TECS `_maxSinkRate` used by the QRTL approach profile stub.
pub const TECS_MAX_SINKRATE_DEFAULT_MS: f32 = 5.0;

/// Outcome of [`qrtl_land_handoff`] (`ModeQRTL::run` RTL land path).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QrtlLandHandoff {
    /// `next_WP_loc.copy_alt_from(home)` when `poscontrol > QPOS_POSITION2`.
    pub copy_home_alt: bool,
    /// `quadplane.verify_vtol_land()` when `poscontrol >= QPOS_POSITION2`.
    pub verify_vtol_land: bool,
    /// Approach / airbrake `stabilize_stick_mixing_fbw`.
    pub stick_mixing_fbw: bool,
    /// Nested [`QuadPlane::verify_vtol_land`] result when it ran.
    pub land: VerifyLandResult,
}

impl QrtlLandHandoff {
    const fn idle() -> Self {
        Self {
            copy_home_alt: false,
            verify_vtol_land: false,
            stick_mixing_fbw: false,
            land: VerifyLandResult::incomplete(),
        }
    }
}

/// `poscontrol.get_state() > QPOS_POSITION2` (discriminant order).
#[must_use]
pub const fn qrtl_copy_home_alt(state: PositionControlState) -> bool {
    (state as u8) > (PositionControlState::Position2 as u8)
}

/// `poscontrol.get_state() >= QPOS_POSITION2` (discriminant order).
#[must_use]
pub const fn qrtl_should_verify_land(state: PositionControlState) -> bool {
    (state as u8) >= (PositionControlState::Position2 as u8)
}

/// Approach / airbrake stick mixing on the QRTL RTL branch.
#[must_use]
pub const fn qrtl_stick_mixing_fbw(state: PositionControlState) -> bool {
    matches!(
        state,
        PositionControlState::Airbrake | PositionControlState::Approach
    )
}

/// Upstream `ModeQRTL::allows_throttle_nudging`.
///
/// Only during [`QrtlSubMode::Rtl`] while `poscontrol` is `QPOS_APPROACH`.
#[must_use]
pub const fn qrtl_allows_throttle_nudging(
    submode: QrtlSubMode,
    state: PositionControlState,
) -> bool {
    matches!(submode, QrtlSubMode::Rtl) && matches!(state, PositionControlState::Approach)
}

/// QRTL RTL-branch land handoff, upstream `ModeQRTL::run` `SubMode::RTL`.
///
/// Reads `qp.poscontrol().state()`. When `>= QPOS_POSITION2` and `land`
/// is `Some`, calls [`QuadPlane::verify_vtol_land`]. Does not rewrite
/// [`crate::landing`] or [`crate::auto_vtol`].
pub fn qrtl_land_handoff(
    qp: &mut QuadPlane,
    land: Option<VerifyLandView>,
) -> QrtlLandHandoff {
    let state = qp.poscontrol().state();
    let mut out = QrtlLandHandoff::idle();
    out.copy_home_alt = qrtl_copy_home_alt(state);
    out.verify_vtol_land = qrtl_should_verify_land(state);
    out.stick_mixing_fbw = qrtl_stick_mixing_fbw(state);
    if out.verify_vtol_land {
        if let Some(view) = land {
            out.land = qp.verify_vtol_land(view);
        }
    }
    out
}

/// Outcome of [`qrtl_update`] (`ModeQRTL::update`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QrtlUpdate {
    /// Upstream always calls `plane.mode_qstabilize.update()`.
    pub used_qstabilize: bool,
}

/// Upstream `ModeQRTL::update`.
#[must_use]
pub const fn qrtl_update() -> QrtlUpdate {
    QrtlUpdate {
        used_qstabilize: true,
    }
}

/// Plane / TECS view [`qrtl_update_target_altitude`] reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlTargetAltView {
    /// Latched `ModeQRTL::submode`.
    pub submode: QrtlSubMode,
    /// `quadplane.poscontrol.get_state()`.
    pub poscontrol: PositionControlState,
    /// `WP_LOITER_RAD`, metres (signed; radius uses `fabsf`).
    pub loiter_radius_m: f32,
    /// `RTL_RADIUS`, metres (signed; radius uses `fabsf`).
    pub rtl_radius_m: f32,
    /// `RTL_ALTITUDE`, metres.
    pub rtl_altitude_m: f32,
    /// `Q_RTL_ALT`, metres.
    pub qrtl_alt_m: f32,
    /// `TECS_controller.get_max_sinkrate()`.
    pub tecs_max_sinkrate_ms: f32,
    /// `aparm.airspeed_cruise`.
    pub airspeed_cruise_ms: f32,
    /// `auto_state.wp_distance`.
    pub wp_distance_m: f32,
}

impl QrtlTargetAltView {
    /// RTL + `QPOS_APPROACH`, far enough that the profile is still at `RTL_ALTITUDE`.
    #[must_use]
    pub const fn approach_far() -> Self {
        Self {
            submode: QrtlSubMode::Rtl,
            poscontrol: PositionControlState::Approach,
            loiter_radius_m: WP_LOITER_RAD_DEFAULT_M,
            rtl_radius_m: RTL_RADIUS_DEFAULT_M,
            rtl_altitude_m: RTL_ALTITUDE_DEFAULT_M,
            qrtl_alt_m: Q_RTL_ALT_DEFAULT_M,
            tecs_max_sinkrate_ms: TECS_MAX_SINKRATE_DEFAULT_MS,
            airspeed_cruise_ms: AIRSPEED_CRUISE_DEFAULT_MS,
            wp_distance_m: 2000.0,
        }
    }

    /// Not in approach — base `Mode::update_target_altitude`.
    #[must_use]
    pub const fn not_approach() -> Self {
        let mut view = Self::approach_far();
        view.poscontrol = PositionControlState::Position1;
        view
    }
}

/// Outcome of [`qrtl_update_target_altitude`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QrtlTargetAlt {
    /// Fell through to `Mode::update_target_altitude`.
    pub used_base_mode: bool,
    /// Height added onto `next_WP_loc` (`loc.offset_up_m`).
    pub offset_up_m: f32,
}

/// Upstream `ModeQRTL::update_target_altitude`.
///
/// Outside RTL+APPROACH this is the base mode helper. In approach the
/// target drops from `RTL_ALTITUDE` toward `Q_RTL_ALT` using TECS max
/// sink and cruise airspeed, matching the C++ linear interpolate.
#[must_use]
pub fn qrtl_update_target_altitude(view: QrtlTargetAltView) -> QrtlTargetAlt {
    if view.submode != QrtlSubMode::Rtl || view.poscontrol != PositionControlState::Approach {
        return QrtlTargetAlt {
            used_base_mode: true,
            offset_up_m: 0.0,
        };
    }
    let loiter = abs_f32(view.loiter_radius_m);
    let rtl = abs_f32(view.rtl_radius_m);
    let radius = if loiter > rtl { loiter } else { rtl };
    let rtl_alt_delta = if view.rtl_altitude_m > view.qrtl_alt_m {
        view.rtl_altitude_m - view.qrtl_alt_m
    } else {
        0.0
    };
    let sink_den = 0.6 * view.tecs_max_sinkrate_ms;
    let sink_den = if sink_den > 1.0 { sink_den } else { 1.0 };
    let sink_time = rtl_alt_delta / sink_den;
    let sink_dist = view.airspeed_cruise_ms * sink_time;
    let rad_min = 2.0 * radius;
    let rad_max = 20.0 * radius;
    let upper = rad_min + sink_dist;
    let var_high = if rad_max < upper { rad_max } else { upper };
    let var_high = if rad_min > var_high { rad_min } else { var_high };
    let offset_up_m = linear_interpolate(0.0, rtl_alt_delta, view.wp_distance_m, rad_min, var_high);
    QrtlTargetAlt {
        used_base_mode: false,
        offset_up_m,
    }
}

/// Whether a catalog row is already hooked up or added by this closer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QrtlPortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by this slice (land handoff + leftover `mode_qrtl.cpp`).
    ThisSlice,
}

/// One `mode_qrtl.cpp` surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QrtlSurface {
    /// Function name (or `run` land-handoff leftover).
    pub name: &'static str,
    /// Hooked up on main or this slice.
    pub status: QrtlPortStatus,
    /// Short note (Rust symbol).
    pub note: &'static str,
}

/// Completeness closer: every function in `ArduPlane/mode_qrtl.cpp`.
pub const MODE_QRTL_CPP_SURFACES: &[QrtlSurface] = &[
    QrtlSurface {
        name: "_enter",
        status: QrtlPortStatus::OnMain,
        note: "qrtl_enter / home-rally climb cone / QLAND_INSTEAD_OF_RTL",
    },
    QrtlSurface {
        name: "update",
        status: QrtlPortStatus::ThisSlice,
        note: "qrtl_update delegates to mode_qstabilize.update",
    },
    QrtlSurface {
        name: "run",
        status: QrtlPortStatus::OnMain,
        note: "qrtl_run climb-then-return / vtol_position_controller",
    },
    QrtlSurface {
        name: "run land handoff",
        status: QrtlPortStatus::ThisSlice,
        note: "qrtl_land_handoff / verify_vtol_land past QPOS_POSITION2",
    },
    QrtlSurface {
        name: "update_target_altitude",
        status: QrtlPortStatus::ThisSlice,
        note: "qrtl_update_target_altitude RTL+APPROACH profile",
    },
    QrtlSurface {
        name: "allows_throttle_nudging",
        status: QrtlPortStatus::ThisSlice,
        note: "qrtl_allows_throttle_nudging RTL + QPOS_APPROACH",
    },
    QrtlSurface {
        name: "get_VTOL_return_radius",
        status: QrtlPortStatus::OnMain,
        note: "qrtl_vtol_return_radius_m MAX(abs radii)*1.5",
    },
];

/// True when every listed `mode_qrtl.cpp` surface is `OnMain` or `ThisSlice`.
#[must_use]
pub const fn mode_qrtl_surfaces_complete() -> bool {
    let mut i = 0;
    while i < MODE_QRTL_CPP_SURFACES.len() {
        match MODE_QRTL_CPP_SURFACES[i].status {
            QrtlPortStatus::OnMain | QrtlPortStatus::ThisSlice => {}
        }
        i += 1;
    }
    MODE_QRTL_CPP_SURFACES.len() == 7
}

fn abs_f32(value: f32) -> f32 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}

/// Upstream `AP_Math::linear_interpolate` (low/high output vs var).
#[must_use]
pub const fn linear_interpolate(
    low_out: f32,
    high_out: f32,
    var: f32,
    var_low: f32,
    var_high: f32,
) -> f32 {
    if var <= var_low {
        return low_out;
    }
    if var >= var_high {
        return high_out;
    }
    let span = var_high - var_low;
    low_out + (var - var_low) * (high_out - low_out) / span
}

fn constrain_f32(amt: f32, low: f32, high: f32) -> f32 {
    if amt < low {
        low
    } else if amt > high {
        high
    } else {
        amt
    }
}

fn is_positive(value: f32) -> bool {
    value > 0.0
}
