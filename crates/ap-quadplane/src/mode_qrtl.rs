//! QRTL `_enter` stub, upstream `ArduPlane/mode_qrtl.cpp` (Plane-4.7.0).
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
//! `run()` / land handoff are later slices. This module does not
//! rewrite [`crate::mode_q`] or [`crate::landing`].

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
