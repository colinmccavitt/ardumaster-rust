//! AUTO mission VTOL leftover, upstream `QuadPlane::control_auto` /
//! `do_vtol_takeoff` / `do_vtol_land` / `verify_vtol_takeoff` /
//! `verify_vtol_land` / `poscontrol_init_approach` (Plane-4.7.0
//! `quadplane.cpp`).
//!
//! Tracked as **VT-001**. This is the leftover AUTO-mission surface:
//! start / verify a VTOL takeoff or land, dispatch `control_auto` to
//! the takeoff / position / waypoint controllers, and seed the land
//! approach (`QPOS_APPROACH` / `AIRBRAKE` / `POSITION1`). It does not
//! rewrite [`crate::landing`] detect, [`crate::vtol_mode`] predicates,
//! or the leftover catalog in [`crate::quadplane_completeness`].

use crate::air_mode::QOption;
use crate::landing::{LandCompleteView, LandFinalView};
use crate::poscontrol::PositionControlState;
use crate::quadplane_completeness::{leftover_option_is_set, LeftoverQOption};
use crate::vtol_mode::{
    MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_LOITER_TURNS,
    MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_PAYLOAD_PLACE, MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_VTOL_LAND, MAV_CMD_NAV_VTOL_TAKEOFF,
};
use crate::QuadPlane;

/// Default `Q_PILOT_SPD_UP`, upstream `AP_GROUPINFO("PILOT_SPD_UP", ..., 2.50)`.
pub const PILOT_SPEED_Z_MAX_UP_DEFAULT_MS: f32 = 2.5;

/// Default `Q_PILOT_ACCEL_Z`, upstream `AP_GROUPINFO("PILOT_ACCEL_Z", ..., 2.5)`.
pub const PILOT_ACCEL_Z_DEFAULT_MSS: f32 = 2.5;

/// Default `Q_TKOFF_FAIL_SCL`, upstream `AP_GROUPINFO("TKOFF_FAIL_SCL", ..., 0)`.
pub const TAKEOFF_FAILURE_SCALAR_DEFAULT: f32 = 0.0;

/// Default `Q_TKOFF_ARSP_LIM`, upstream `AP_GROUPINFO("TKOFF_ARSP_LIM", ..., 0)`.
pub const MAX_TAKEOFF_AIRSPEED_DEFAULT_MS: f32 = 0.0;

/// Default `Q_APPROACH_DIST`, upstream `AP_GROUPINFO("APPROACH_DIST", ..., 0)`.
pub const APPROACH_DISTANCE_DEFAULT_M: f32 = 0.0;

/// Floor on `takeoff_time_limit_ms`, upstream `MAX(..., 5000)`.
pub const TAKEOFF_TIME_LIMIT_MIN_MS: u32 = 5000;

/// Ground-effect window after takeoff start, upstream `< 3000` ms.
pub const TAKEOFF_GND_EFFECT_MS: u32 = 3000;

/// `verify_vtol_land` POSITION2 distance gate, metres.
pub const DESCEND_DIST_THRESHOLD_M: f32 = 2.0;

/// `verify_vtol_land` POSITION2 speed gate, m/s.
pub const DESCEND_SPEED_THRESHOLD_MS: f32 = 3.0;

/// Stale-poscontrol reset on AUTO loiter, upstream `> 100` ms.
pub const LOITER_POSCONTROL_RESET_MS: u32 = 100;

/// Velocity-match freshness, upstream `< 1000` ms.
pub const VELOCITY_MATCH_FRESH_MS: u32 = 1000;

/// Minimum climb accel / speed used in the takeoff-time estimate.
pub const TAKEOFF_KIN_MIN: f32 = 0.1;

/// Which AUTO leftover controller `control_auto` would run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutoController {
    /// `setup()` failed — early return.
    None,
    /// `takeoff_controller()` (`NAV_VTOL_TAKEOFF` / VTOL `NAV_TAKEOFF`).
    Takeoff,
    /// `vtol_position_controller()` (VTOL land / loiter).
    Position,
    /// `waypoint_controller()` (any other AUTO nav command).
    Waypoint,
}

/// Inputs [`QuadPlane::control_auto`] reads from Plane / motors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlAutoView {
    /// `mission.get_current_nav_cmd().id`.
    pub nav_cmd_id: u16,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `arming.get_delay_arming()`.
    pub delay_arming: bool,
    /// `motors->get_desired_spool_state() == SHUT_DOWN`.
    pub spool_shutdown: bool,
    /// `plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)`.
    pub payload_place: bool,
}

impl ControlAutoView {
    /// AUTO flying `nav_cmd_id` at `now_ms`, motors not shut down.
    #[must_use]
    pub const fn nav(nav_cmd_id: u16, now_ms: u32) -> Self {
        Self {
            nav_cmd_id,
            now_ms,
            delay_arming: false,
            spool_shutdown: false,
            payload_place: false,
        }
    }
}

/// Side-effects of [`QuadPlane::control_auto`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlAutoResult {
    /// Controller leftover the C++ switch would call.
    pub controller: AutoController,
    /// `set_desired_spool_state(THROTTLE_UNLIMITED)` — dead in 4.7.0
    /// (`should_run_motors` is never set true).
    pub spool_unlimited: bool,
    /// Loiter path reset `poscontrol` to `QPOS_POSITION1`.
    pub loiter_reset_position1: bool,
}

/// Mission / kinematics snapshot for [`QuadPlane::do_vtol_takeoff`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VtolTakeoffCmd {
    /// `millis()` / `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `plane.current_loc.alt` (cm, absolute).
    pub current_alt_cm: i32,
    /// `cmd.content.location.alt` (cm). Relative climb unless
    /// `RESPECT_TAKEOFF_FRAME`.
    pub cmd_alt_cm: i32,
    /// `inertial_nav.get_velocity_z_up_cms() * 0.01`.
    pub vel_u_ms: f32,
}

impl VtolTakeoffCmd {
    /// Climb `cmd_alt_cm` from `current_alt_cm`, hover, at `now_ms`.
    #[must_use]
    pub const fn climb(now_ms: u32, current_alt_cm: i32, cmd_alt_cm: i32) -> Self {
        Self {
            now_ms,
            current_alt_cm,
            cmd_alt_cm,
            vel_u_ms: 0.0,
        }
    }
}

/// Outcome of [`QuadPlane::do_vtol_takeoff`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DoVtolTakeoffResult {
    /// The C++ method returned `true`.
    pub accepted: bool,
    /// Absolute `next_WP_loc.alt` written on accept (cm).
    pub next_wp_alt_cm: i32,
    /// `takeoff_time_limit_ms` latched on accept.
    pub takeoff_time_limit_ms: u32,
}

impl DoVtolTakeoffResult {
    /// `setup()` failed or already above a framed takeoff.
    #[must_use]
    pub const fn rejected() -> Self {
        Self {
            accepted: false,
            next_wp_alt_cm: 0,
            takeoff_time_limit_ms: 0,
        }
    }
}

/// Inputs [`QuadPlane::verify_vtol_takeoff`] reads each tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerifyTakeoffView {
    /// Command used to restart takeoff when disarmed.
    pub cmd: VtolTakeoffCmd,
    /// `arming.is_armed_and_safety_off()`.
    pub armed_and_safety_off: bool,
    /// `plane.current_loc.alt` (cm).
    pub current_alt_cm: i32,
    /// `plane.airspeed.get_airspeed()` (m/s).
    pub airspeed_ms: f32,
    /// `control_mode == mode_auto`.
    pub in_auto: bool,
}

impl VerifyTakeoffView {
    /// Armed AUTO climb check at `current_alt_cm`.
    #[must_use]
    pub const fn armed_auto(cmd: VtolTakeoffCmd, current_alt_cm: i32) -> Self {
        Self {
            cmd,
            armed_and_safety_off: true,
            current_alt_cm,
            airspeed_ms: 0.0,
            in_auto: true,
        }
    }
}

/// Side-effects of [`QuadPlane::verify_vtol_takeoff`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifyTakeoffResult {
    /// The C++ method returned `true` (item complete).
    pub done: bool,
    /// Switch to QLAND (`ModeReason::VTOL_FAILED_TAKEOFF`).
    pub failed_qland: bool,
    /// `ahrs.set_takeoff_expected(true)` in the first 3 s.
    pub takeoff_expected: bool,
    /// `transition->restart()` on a successful climb-out.
    pub transition_restart: bool,
    /// `TECS_controller.reset()` when completing in AUTO.
    pub tecs_reset: bool,
}

impl VerifyTakeoffResult {
    /// Still climbing / restarted / failed — item not complete.
    #[must_use]
    pub const fn incomplete() -> Self {
        Self {
            done: false,
            failed_qland: false,
            takeoff_expected: false,
            transition_restart: false,
            tecs_reset: false,
        }
    }
}

/// Geometry / spool snapshot for [`QuadPlane::poscontrol_init_approach`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApproachInitView {
    /// `current_loc.get_distance(next_WP_loc)` (metres).
    pub dist_m: f32,
    /// `transition_threshold_m()` (1.5 × cruise stopping distance).
    pub transition_threshold_m: f32,
    /// `tailsitter.enabled()`.
    pub tailsitter_enabled: bool,
    /// Motors already `THROTTLE_UNLIMITED`.
    pub spool_unlimited: bool,
}

impl ApproachInitView {
    /// Far from the land WP — full approach.
    #[must_use]
    pub const fn far() -> Self {
        Self {
            dist_m: 1000.0,
            transition_threshold_m: 50.0,
            tailsitter_enabled: false,
            spool_unlimited: false,
        }
    }

    /// Inside the transition threshold, motors not spooling.
    #[must_use]
    pub const fn close() -> Self {
        Self {
            dist_m: 20.0,
            transition_threshold_m: 50.0,
            tailsitter_enabled: false,
            spool_unlimited: false,
        }
    }
}

/// Inputs [`QuadPlane::verify_vtol_land`] reads each tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerifyLandView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `plane.current_loc.alt` (cm).
    pub current_alt_cm: i32,
    /// Horizontal distance to the land target (metres).
    pub dist_m: f32,
    /// `vel_ned_ms.x` (north, m/s).
    pub vel_north_ms: f32,
    /// `vel_ned_ms.y` (east, m/s).
    pub vel_east_ms: f32,
    /// `control_mode == mode_auto`.
    pub in_auto: bool,
    /// `in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)`.
    pub payload_place: bool,
    /// Current nav-cmd `p1` (payload-place abort depth, cm).
    pub payload_p1: u16,
    /// `mission.continue_after_land()`.
    pub continue_after_land: bool,
    /// Snapshot for [`QuadPlane::check_land_complete`].
    pub land_complete: LandCompleteView,
    /// Snapshot for [`QuadPlane::check_land_final`].
    pub land_final: LandFinalView,
}

impl VerifyLandView {
    /// POSITION2 / descend tick, no payload-place, not AUTO-continue.
    #[must_use]
    pub const fn hover_over(now_ms: u32, current_alt_cm: i32, dist_m: f32) -> Self {
        Self {
            now_ms,
            current_alt_cm,
            dist_m,
            vel_north_ms: 0.0,
            vel_east_ms: 0.0,
            in_auto: true,
            payload_place: false,
            payload_p1: 0,
            continue_after_land: false,
            land_complete: LandCompleteView::qland(now_ms, (current_alt_cm as f32) * 0.01),
            land_final: LandFinalView {
                detect: crate::landing::LandDetectView::settled(
                    now_ms,
                    (current_alt_cm as f32) * 0.01,
                ),
                height_above_ground_m: (current_alt_cm as f32) * 0.01,
            },
        }
    }
}

/// Side-effects of [`QuadPlane::verify_vtol_land`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifyLandResult {
    /// The C++ method returned `true` (advance the mission).
    pub done: bool,
    /// Moved `poscontrol` to `QPOS_LAND_DESCEND`.
    pub entered_descend: bool,
    /// Moved `poscontrol` to `QPOS_LAND_FINAL`.
    pub entered_final: bool,
    /// Moved `poscontrol` to `QPOS_LAND_ABORT`.
    pub entered_abort: bool,
    /// `check_land_complete && continue_after_land`.
    pub mission_continue: bool,
}

impl VerifyLandResult {
    /// Still in the land sequence.
    #[must_use]
    pub const fn incomplete() -> Self {
        Self {
            done: false,
            entered_descend: false,
            entered_final: false,
            entered_abort: false,
            mission_continue: false,
        }
    }
}

/// AUTO takeoff / land leftover state stored on [`QuadPlane`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutoVtol {
    takeoff_start_time_ms: u32,
    takeoff_time_limit_ms: u32,
    takeoff_failure_scalar: f32,
    maximum_takeoff_airspeed_ms: f32,
    next_wp_alt_cm: i32,
    land_descend_start_alt_m: f32,
    approach_distance_m: f32,
    last_run_ms: u32,
    pilot_speed_z_max_up_ms: f32,
    pilot_accel_z_mss: f32,
}

impl Default for AutoVtol {
    fn default() -> Self {
        Self::new()
    }
}

impl AutoVtol {
    /// Parameter defaults, timers zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            takeoff_start_time_ms: 0,
            takeoff_time_limit_ms: 0,
            takeoff_failure_scalar: TAKEOFF_FAILURE_SCALAR_DEFAULT,
            maximum_takeoff_airspeed_ms: MAX_TAKEOFF_AIRSPEED_DEFAULT_MS,
            next_wp_alt_cm: 0,
            land_descend_start_alt_m: 0.0,
            approach_distance_m: APPROACH_DISTANCE_DEFAULT_M,
            last_run_ms: 0,
            pilot_speed_z_max_up_ms: PILOT_SPEED_Z_MAX_UP_DEFAULT_MS,
            pilot_accel_z_mss: PILOT_ACCEL_Z_DEFAULT_MSS,
        }
    }

    /// `takeoff_start_time_ms`.
    #[must_use]
    pub const fn takeoff_start_time_ms(&self) -> u32 {
        self.takeoff_start_time_ms
    }

    /// `takeoff_time_limit_ms`.
    #[must_use]
    pub const fn takeoff_time_limit_ms(&self) -> u32 {
        self.takeoff_time_limit_ms
    }

    /// `Q_TKOFF_FAIL_SCL`.
    #[must_use]
    pub const fn takeoff_failure_scalar(&self) -> f32 {
        self.takeoff_failure_scalar
    }

    /// Write `Q_TKOFF_FAIL_SCL`.
    pub fn set_takeoff_failure_scalar(&mut self, scalar: f32) {
        self.takeoff_failure_scalar = scalar;
    }

    /// `Q_TKOFF_ARSP_LIM`.
    #[must_use]
    pub const fn maximum_takeoff_airspeed_ms(&self) -> f32 {
        self.maximum_takeoff_airspeed_ms
    }

    /// Write `Q_TKOFF_ARSP_LIM`.
    pub fn set_maximum_takeoff_airspeed_ms(&mut self, airspeed_ms: f32) {
        self.maximum_takeoff_airspeed_ms = airspeed_ms;
    }

    /// Absolute `next_WP_loc.alt` latched by [`QuadPlane::do_vtol_takeoff`].
    #[must_use]
    pub const fn next_wp_alt_cm(&self) -> i32 {
        self.next_wp_alt_cm
    }

    /// `land_descend_start_alt_m` (set on enter `QPOS_LAND_DESCEND`).
    #[must_use]
    pub const fn land_descend_start_alt_m(&self) -> f32 {
        self.land_descend_start_alt_m
    }

    /// Write `land_descend_start_alt_m` (tests / `set_state` side-effect).
    pub fn set_land_descend_start_alt_m(&mut self, alt_m: f32) {
        self.land_descend_start_alt_m = alt_m;
    }

    /// `Q_APPROACH_DIST`.
    #[must_use]
    pub const fn approach_distance_m(&self) -> f32 {
        self.approach_distance_m
    }

    /// Write `Q_APPROACH_DIST`.
    pub fn set_approach_distance_m(&mut self, approach_distance_m: f32) {
        self.approach_distance_m = approach_distance_m;
    }

    /// `poscontrol.last_run_ms` used by the AUTO loiter reset.
    #[must_use]
    pub const fn last_run_ms(&self) -> u32 {
        self.last_run_ms
    }

    /// Write `poscontrol.last_run_ms` (tests / later position slice).
    pub fn set_last_run_ms(&mut self, last_run_ms: u32) {
        self.last_run_ms = last_run_ms;
    }
}

/// Upstream takeoff-time estimate, then `MAX(..., 5000)`.
///
/// `t_accel = (V_max - V_z) / a`, `d_accel = V_z t + 1/2 a t^2`,
/// `t_constant = (d_total - d_accel) / V_max`.
#[must_use]
pub fn takeoff_time_limit_ms(
    d_total_m: f32,
    vel_u_ms: f32,
    vel_max_ms: f32,
    accel_mss: f32,
    failure_scalar: f32,
) -> u32 {
    let accel = if accel_mss > TAKEOFF_KIN_MIN {
        accel_mss
    } else {
        TAKEOFF_KIN_MIN
    };
    let vel_max = if vel_max_ms > TAKEOFF_KIN_MIN {
        vel_max_ms
    } else {
        TAKEOFF_KIN_MIN
    };
    let t_accel = (vel_max - vel_u_ms) / accel;
    let d_accel = vel_u_ms * t_accel + 0.5 * accel * t_accel * t_accel;
    let d_remaining = d_total_m - d_accel;
    let t_constant = d_remaining / vel_max;
    let travel_s = max_f32(t_accel, 0.0) + max_f32(t_constant, 0.0);
    let ms = travel_s * failure_scalar * 1000.0;
    let ms_u = if ms <= 0.0 {
        0
    } else if ms >= 4_294_967_295.0 {
        u32::MAX
    } else {
        ms as u32
    };
    if ms_u < TAKEOFF_TIME_LIMIT_MIN_MS {
        TAKEOFF_TIME_LIMIT_MIN_MS
    } else {
        ms_u
    }
}

const fn max_f32(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

const fn is_positive(v: f32) -> bool {
    v > 0.0
}

fn horiz_speed_ms(north_ms: f32, east_ms: f32) -> f32 {
    libm::hypotf(north_ms, east_ms)
}

impl QuadPlane {
    /// AUTO VTOL leftover block.
    #[must_use]
    pub const fn auto_vtol(&self) -> &AutoVtol {
        &self.auto_vtol
    }

    /// Mutable AUTO VTOL leftover block (parameter poke / tests).
    pub fn auto_vtol_mut(&mut self) -> &mut AutoVtol {
        &mut self.auto_vtol
    }

    /// Upstream `QuadPlane::poscontrol_init_approach`.
    ///
    /// `DISABLE_APPROACH` or a short `Q_APPROACH_DIST` skips to
    /// `QPOS_POSITION1`. Otherwise a far WP is `QPOS_APPROACH`; inside
    /// `transition_threshold_m` a tailsitter / already-spooling airframe
    /// goes to `POSITION1` and a still-fw airframe goes to `AIRBRAKE`.
    pub fn poscontrol_init_approach(&mut self, view: ApproachInitView) {
        let disable = leftover_option_is_set(self.options, LeftoverQOption::DisableApproach);
        let short_approach = is_positive(self.auto_vtol.approach_distance_m)
            && view.dist_m < self.auto_vtol.approach_distance_m;
        if disable || short_approach {
            self.poscontrol.set_state(PositionControlState::Position1);
        } else if self.poscontrol.state() != PositionControlState::Approach {
            if view.dist_m < view.transition_threshold_m {
                if view.tailsitter_enabled || view.spool_unlimited {
                    self.poscontrol.set_state(PositionControlState::Position1);
                } else {
                    self.poscontrol.set_state(PositionControlState::Airbrake);
                }
            } else {
                self.poscontrol.set_state(PositionControlState::Approach);
            }
        }
        self.poscontrol
            .set_pilot_correction(false, self.poscontrol.pilot_correction_active());
        self.poscontrol.set_correction_ne_m(0.0, 0.0);
    }

    /// Upstream `QuadPlane::do_vtol_takeoff`.
    ///
    /// XY is always the current location. Height is current + cmd
    /// unless `RESPECT_TAKEOFF_FRAME`, which uses the cmd altitude as
    /// absolute and rejects a start already at or above that height.
    /// Clears `throttle_wait` and latches the takeoff time limit.
    pub fn do_vtol_takeoff(&mut self, cmd: VtolTakeoffCmd) -> DoVtolTakeoffResult {
        if !self.setup() {
            return DoVtolTakeoffResult::rejected();
        }
        let next_alt_cm =
            if leftover_option_is_set(self.options, LeftoverQOption::RespectTakeoffFrame) {
                if cmd.current_alt_cm >= cmd.cmd_alt_cm {
                    return DoVtolTakeoffResult::rejected();
                }
                cmd.cmd_alt_cm
            } else {
                cmd.current_alt_cm.saturating_add(cmd.cmd_alt_cm)
            };
        self.throttle_wait = false;
        let d_total_m = ((next_alt_cm - cmd.current_alt_cm) as f32) * 0.01;
        let limit = takeoff_time_limit_ms(
            d_total_m,
            cmd.vel_u_ms,
            self.auto_vtol.pilot_speed_z_max_up_ms,
            self.auto_vtol.pilot_accel_z_mss,
            self.auto_vtol.takeoff_failure_scalar,
        );
        self.auto_vtol.takeoff_start_time_ms = cmd.now_ms;
        self.auto_vtol.takeoff_time_limit_ms = limit;
        self.auto_vtol.next_wp_alt_cm = next_alt_cm;
        DoVtolTakeoffResult {
            accepted: true,
            next_wp_alt_cm: next_alt_cm,
            takeoff_time_limit_ms: limit,
        }
    }

    /// Upstream `QuadPlane::do_vtol_land`.
    ///
    /// Clears `throttle_wait` and the land-detect timers, then
    /// [`Self::poscontrol_init_approach`].
    pub fn do_vtol_land(&mut self, approach: ApproachInitView) -> bool {
        if !self.setup() {
            return false;
        }
        self.throttle_wait = false;
        self.landing_detect.clear_land_timers();
        self.poscontrol_init_approach(approach);
        true
    }

    /// Upstream `QuadPlane::verify_vtol_takeoff`.
    ///
    /// Unavailable QuadPlane completes the item. Disarmed restarts
    /// takeoff. A positive `Q_TKOFF_FAIL_SCL` past the time limit, or
    /// airspeed above `Q_TKOFF_ARSP_LIM`, fails to QLAND. Reaching
    /// `next_WP` altitude completes the item and restarts transition.
    pub fn verify_vtol_takeoff(&mut self, view: VerifyTakeoffView) -> VerifyTakeoffResult {
        if !self.available() {
            return VerifyTakeoffResult {
                done: true,
                failed_qland: false,
                takeoff_expected: false,
                transition_restart: false,
                tecs_reset: false,
            };
        }
        if !view.armed_and_safety_off {
            let _ = self.do_vtol_takeoff(view.cmd);
            return VerifyTakeoffResult::incomplete();
        }
        let mut result = VerifyTakeoffResult::incomplete();
        let elapsed = view
            .cmd
            .now_ms
            .wrapping_sub(self.auto_vtol.takeoff_start_time_ms);
        if elapsed < TAKEOFF_GND_EFFECT_MS && !self.option_is_set(QOption::DisableGroundEffectComp)
        {
            result.takeoff_expected = true;
        }
        if is_positive(self.auto_vtol.takeoff_failure_scalar)
            && elapsed > self.auto_vtol.takeoff_time_limit_ms
        {
            result.failed_qland = true;
            return result;
        }
        if is_positive(self.auto_vtol.maximum_takeoff_airspeed_ms)
            && view.airspeed_ms > self.auto_vtol.maximum_takeoff_airspeed_ms
        {
            result.failed_qland = true;
            return result;
        }
        if view.current_alt_cm < self.auto_vtol.next_wp_alt_cm {
            return result;
        }
        result.done = true;
        result.transition_restart = true;
        result.tecs_reset = view.in_auto;
        result
    }

    /// Upstream `QuadPlane::verify_vtol_land`.
    ///
    /// POSITION2 plus the distance / speed gates enter descend. Descend
    /// plus [`Self::check_land_final`] enters final. `LAND_ABORT` at or
    /// above the descend-start altitude advances the mission. Payload
    /// place too deep aborts. Land-complete + `continue_after_land`
    /// advances the mission.
    pub fn verify_vtol_land(&mut self, view: VerifyLandView) -> VerifyLandResult {
        if !self.available() {
            return VerifyLandResult {
                done: true,
                entered_descend: false,
                entered_final: false,
                entered_abort: false,
                mission_continue: false,
            };
        }
        let mut result = VerifyLandResult::incomplete();
        if self.poscontrol.state() == PositionControlState::Position2 {
            let reached = if self.poscontrol.pilot_correction_done() {
                !self.poscontrol.pilot_correction_active()
            } else {
                view.dist_m < DESCEND_DIST_THRESHOLD_M
            };
            let match_age = view
                .now_ms
                .wrapping_sub(self.poscontrol.last_velocity_match_ms());
            let (app_n, app_e) = if match_age < VELOCITY_MATCH_FRESH_MS {
                (
                    self.poscontrol.velocity_match_north_ms(),
                    self.poscontrol.velocity_match_east_ms(),
                )
            } else {
                (0.0, 0.0)
            };
            let rel_speed = horiz_speed_ms(view.vel_north_ms - app_n, view.vel_east_ms - app_e);
            if reached && rel_speed < DESCEND_SPEED_THRESHOLD_MS {
                self.poscontrol.set_state(PositionControlState::LandDescend);
                self.poscontrol.set_pilot_correction(false, false);
                self.lean_angle_max_cd = 0;
                self.poscontrol.set_correction_ne_m(0.0, 0.0);
                self.auto_vtol.land_descend_start_alt_m = (view.current_alt_cm as f32) * 0.01;
                result.entered_descend = true;
            }
        }
        if self.poscontrol.state() == PositionControlState::LandDescend
            && self.check_land_final(view.land_final)
        {
            self.poscontrol.set_state(PositionControlState::LandFinal);
            self.landing_detect.clear_land_timers();
            result.entered_final = true;
        }
        if self.poscontrol.state() == PositionControlState::LandAbort {
            let current_alt_m = (view.current_alt_cm as f32) * 0.01;
            if current_alt_m >= self.auto_vtol.land_descend_start_alt_m {
                result.done = true;
                return result;
            }
        }
        if view.payload_place
            && matches!(
                self.poscontrol.state(),
                PositionControlState::LandDescend | PositionControlState::LandFinal
            )
            && view.payload_p1 > 0
        {
            let current_alt_m = (view.current_alt_cm as f32) * 0.01;
            let abort_alt =
                self.auto_vtol.land_descend_start_alt_m - (view.payload_p1 as f32) * 0.01;
            if current_alt_m < abort_alt {
                self.poscontrol.set_state(PositionControlState::LandAbort);
                result.entered_abort = true;
            }
        }
        let complete = self.check_land_complete(view.land_complete);
        if complete.complete && view.continue_after_land {
            result.done = true;
            result.mission_continue = true;
        }
        result
    }

    /// Upstream `QuadPlane::control_auto`.
    ///
    /// `setup()` first. The post-approach spool block is reproduced
    /// as-is (the 4.7.0 `should_run_motors` latch is never true). The
    /// nav-cmd switch picks takeoff / position / waypoint; a stale
    /// loiter resets poscontrol to `QPOS_POSITION1`.
    pub fn control_auto(&mut self, view: ControlAutoView) -> ControlAutoResult {
        if !self.setup() {
            return ControlAutoResult {
                controller: AutoController::None,
                spool_unlimited: false,
                loiter_reset_position1: false,
            };
        }
        let mut spool_unlimited = false;
        if (self.poscontrol.state() as u8) > (PositionControlState::Approach as u8) {
            let mut should_run_motors = false;
            if view.delay_arming {
                should_run_motors = false;
            }
            if view.spool_shutdown
                && view.payload_place
                && self.poscontrol.state() == PositionControlState::LandComplete
            {
                should_run_motors = false;
            }
            if should_run_motors {
                spool_unlimited = true;
            }
        }
        let id = view.nav_cmd_id;
        match id {
            MAV_CMD_NAV_VTOL_TAKEOFF | MAV_CMD_NAV_TAKEOFF => ControlAutoResult {
                controller: if self.is_vtol_takeoff(id) {
                    AutoController::Takeoff
                } else {
                    AutoController::None
                },
                spool_unlimited,
                loiter_reset_position1: false,
            },
            MAV_CMD_NAV_VTOL_LAND | MAV_CMD_NAV_PAYLOAD_PLACE | MAV_CMD_NAV_LAND => {
                ControlAutoResult {
                    controller: if self.is_vtol_land(id) {
                        AutoController::Position
                    } else {
                        AutoController::None
                    },
                    spool_unlimited,
                    loiter_reset_position1: false,
                }
            }
            MAV_CMD_NAV_LOITER_UNLIM
            | MAV_CMD_NAV_LOITER_TIME
            | MAV_CMD_NAV_LOITER_TURNS
            | MAV_CMD_NAV_LOITER_TO_ALT => {
                let stale = view.now_ms.wrapping_sub(self.auto_vtol.last_run_ms)
                    > LOITER_POSCONTROL_RESET_MS;
                if stale {
                    self.poscontrol.set_state(PositionControlState::Position1);
                }
                ControlAutoResult {
                    controller: AutoController::Position,
                    spool_unlimited,
                    loiter_reset_position1: stale,
                }
            }
            _ => ControlAutoResult {
                controller: AutoController::Waypoint,
                spool_unlimited,
                loiter_reset_position1: false,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::landing::RelaxView;
    use crate::vtol_mode::MAV_CMD_NAV_WAYPOINT;

    fn enabled() -> QuadPlane {
        let mut qp = QuadPlane::with_enable(1);
        assert!(qp.setup());
        qp
    }

    #[test]
    fn control_auto_dispatches_takeoff_land_and_waypoint() {
        let mut qp = QuadPlane::new();
        let idle = qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_VTOL_TAKEOFF, 0));
        assert_eq!(idle.controller, AutoController::None);

        let mut qp = enabled();
        assert_eq!(
            qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_VTOL_TAKEOFF, 0))
                .controller,
            AutoController::Takeoff
        );
        assert_eq!(
            qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_TAKEOFF, 0))
                .controller,
            AutoController::Takeoff
        );
        assert_eq!(
            qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_VTOL_LAND, 0))
                .controller,
            AutoController::Position
        );
        assert_eq!(
            qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_WAYPOINT, 0))
                .controller,
            AutoController::Waypoint
        );
    }

    #[test]
    fn control_auto_loiter_resets_stale_poscontrol() {
        let mut qp = enabled();
        qp.auto_vtol_mut().set_last_run_ms(0);
        qp.poscontrol_mut()
            .set_state(PositionControlState::Approach);
        let r = qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_LOITER_UNLIM, 150));
        assert_eq!(r.controller, AutoController::Position);
        assert!(r.loiter_reset_position1);
        assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);

        qp.auto_vtol_mut().set_last_run_ms(200);
        qp.poscontrol_mut()
            .set_state(PositionControlState::Approach);
        let r = qp.control_auto(ControlAutoView::nav(MAV_CMD_NAV_LOITER_TIME, 250));
        assert!(!r.loiter_reset_position1);
        assert_eq!(qp.poscontrol().state(), PositionControlState::Approach);
        assert!(!r.spool_unlimited);
    }

    #[test]
    fn do_vtol_takeoff_sets_alt_and_min_time_limit() {
        let mut qp = QuadPlane::new();
        assert!(
            !qp.do_vtol_takeoff(VtolTakeoffCmd::climb(10, 10000, 2000))
                .accepted
        );

        let mut qp = enabled();
        qp.set_throttle_wait(true);
        let r = qp.do_vtol_takeoff(VtolTakeoffCmd::climb(10, 10000, 2000));
        assert!(r.accepted);
        assert_eq!(r.next_wp_alt_cm, 12000);
        assert_eq!(r.takeoff_time_limit_ms, TAKEOFF_TIME_LIMIT_MIN_MS);
        assert!(!qp.throttle_wait());
        assert_eq!(qp.auto_vtol().takeoff_start_time_ms(), 10);
        assert_eq!(qp.auto_vtol().next_wp_alt_cm(), 12000);
    }

    #[test]
    fn do_vtol_takeoff_respect_frame_rejects_above_target() {
        let mut qp = enabled();
        qp.set_options(LeftoverQOption::RespectTakeoffFrame.as_i32());
        assert!(
            !qp.do_vtol_takeoff(VtolTakeoffCmd::climb(0, 15000, 12000))
                .accepted
        );
        let r = qp.do_vtol_takeoff(VtolTakeoffCmd::climb(0, 10000, 15000));
        assert!(r.accepted);
        assert_eq!(r.next_wp_alt_cm, 15000);
    }

    #[test]
    fn takeoff_time_limit_uses_kinematics_then_floor() {
        let limit = takeoff_time_limit_ms(20.0, 0.0, 2.5, 2.5, 0.0);
        assert_eq!(limit, TAKEOFF_TIME_LIMIT_MIN_MS);
        let limit = takeoff_time_limit_ms(20.0, 0.0, 2.5, 2.5, 10.0);
        assert!(limit > TAKEOFF_TIME_LIMIT_MIN_MS);
    }

    #[test]
    fn verify_vtol_takeoff_complete_fail_and_disarm_restart() {
        let mut qp = QuadPlane::new();
        let cmd = VtolTakeoffCmd::climb(0, 10000, 2000);
        assert!(
            qp.verify_vtol_takeoff(VerifyTakeoffView::armed_auto(cmd, 10000))
                .done
        );

        let mut qp = enabled();
        assert!(qp.do_vtol_takeoff(cmd).accepted);
        let mut view =
            VerifyTakeoffView::armed_auto(VtolTakeoffCmd::climb(100, 10000, 2000), 11000);
        let r = qp.verify_vtol_takeoff(view);
        assert!(!r.done);
        assert!(r.takeoff_expected);

        view.current_alt_cm = 12000;
        let r = qp.verify_vtol_takeoff(view);
        assert!(r.done);
        assert!(r.transition_restart);
        assert!(r.tecs_reset);

        let mut qp = enabled();
        assert!(qp.do_vtol_takeoff(cmd).accepted);
        let mut disarmed =
            VerifyTakeoffView::armed_auto(VtolTakeoffCmd::climb(50, 10000, 2000), 10000);
        disarmed.armed_and_safety_off = false;
        let r = qp.verify_vtol_takeoff(disarmed);
        assert!(!r.done);
        assert_eq!(qp.auto_vtol().takeoff_start_time_ms(), 50);

        let mut qp = enabled();
        qp.auto_vtol_mut().set_takeoff_failure_scalar(1.0);
        assert!(qp.do_vtol_takeoff(cmd).accepted);
        let late = VerifyTakeoffView::armed_auto(
            VtolTakeoffCmd::climb(20_000, 10000, 2000),
            10000,
        );
        let r = qp.verify_vtol_takeoff(late);
        assert!(r.failed_qland);
        assert!(!r.done);

        let mut qp = enabled();
        qp.auto_vtol_mut().set_maximum_takeoff_airspeed_ms(8.0);
        assert!(qp.do_vtol_takeoff(cmd).accepted);
        let mut windy =
            VerifyTakeoffView::armed_auto(VtolTakeoffCmd::climb(100, 10000, 2000), 10000);
        windy.airspeed_ms = 9.0;
        assert!(qp.verify_vtol_takeoff(windy).failed_qland);
    }

    #[test]
    fn do_vtol_land_inits_approach_and_clears_detect() {
        let mut qp = QuadPlane::new();
        assert!(!qp.do_vtol_land(ApproachInitView::far()));

        let mut qp = enabled();
        qp.set_throttle_wait(true);
        let _ = qp.should_relax(RelaxView::lower_limit(5_000));
        assert!(qp.do_vtol_land(ApproachInitView::far()));
        assert!(!qp.throttle_wait());
        assert_eq!(qp.landing_detect().lower_limit_start_ms(), 0);
        assert_eq!(qp.landing_detect().land_start_ms(), 0);
        assert_eq!(qp.poscontrol().state(), PositionControlState::Approach);
    }

    #[test]
    fn poscontrol_init_approach_picks_approach_airbrake_position1() {
        let mut qp = enabled();
        qp.poscontrol_init_approach(ApproachInitView::far());
        assert_eq!(qp.poscontrol().state(), PositionControlState::Approach);

        qp.poscontrol_init_approach(ApproachInitView::close());
        assert_eq!(qp.poscontrol().state(), PositionControlState::Approach);

        qp.poscontrol_mut().set_state(PositionControlState::None);
        qp.poscontrol_init_approach(ApproachInitView::close());
        assert_eq!(qp.poscontrol().state(), PositionControlState::Airbrake);

        qp.poscontrol_mut().set_state(PositionControlState::None);
        let mut close_ts = ApproachInitView::close();
        close_ts.tailsitter_enabled = true;
        qp.poscontrol_init_approach(close_ts);
        assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);

        qp.set_options(LeftoverQOption::DisableApproach.as_i32());
        qp.poscontrol_mut().set_state(PositionControlState::None);
        qp.poscontrol_init_approach(ApproachInitView::far());
        assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);

        qp.set_options(0);
        qp.auto_vtol_mut().set_approach_distance_m(100.0);
        qp.poscontrol_mut().set_state(PositionControlState::None);
        let mut mid = ApproachInitView::far();
        mid.dist_m = 40.0;
        qp.poscontrol_init_approach(mid);
        assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);
    }

    #[test]
    fn verify_vtol_land_descend_abort_and_unavailable() {
        let mut qp = QuadPlane::new();
        assert!(
            qp.verify_vtol_land(VerifyLandView::hover_over(0, 10000, 0.5))
                .done
        );

        let mut qp = enabled();
        qp.poscontrol_mut()
            .set_state(PositionControlState::Position2);
        let r = qp.verify_vtol_land(VerifyLandView::hover_over(1_000, 8000, 0.5));
        assert!(r.entered_descend);
        assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
        assert!(!r.done);

        qp.poscontrol_mut()
            .set_state(PositionControlState::LandAbort);
        let r = qp.verify_vtol_land(VerifyLandView::hover_over(2_000, 8000, 0.5));
        assert!(r.done);

        let mut qp = enabled();
        qp.poscontrol_mut()
            .set_state(PositionControlState::LandDescend);
        qp.auto_vtol_mut().set_land_descend_start_alt_m(80.0);
        let mut place = VerifyLandView::hover_over(3_000, 5000, 0.5);
        place.payload_place = true;
        place.payload_p1 = 2000;
        let r = qp.verify_vtol_land(place);
        assert!(r.entered_abort);
        assert_eq!(qp.poscontrol().state(), PositionControlState::LandAbort);
    }
}
