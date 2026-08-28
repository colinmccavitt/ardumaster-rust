//! Leftover VTOL position / takeoff / waypoint controllers, upstream
//! `QuadPlane::vtol_position_controller` / `takeoff_controller` /
//! `waypoint_controller` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. This is the leftover controller surface: the
//! `QPOS_*` horizontal / height dispatch, takeoff wait-then-climb, and
//! AUTO waypoint-nav refresh. It does not rewrite
//! [`crate::auto_vtol`] `control_auto` dispatch, [`crate::poscontrol`]
//! `mode_enter`, [`crate::logging`] QTUN / AttRate, or the leftover
//! `hold_hover` / `stopping_distance_m` rows.

use crate::logging::{qpos_period_elapsed, QPosView};
use crate::poscontrol::PositionControlState;
use crate::QuadPlane;

/// Distance that switches POSITION1 → POSITION2, metres.
pub const POSITION2_DIST_THRESHOLD_M: f32 = 10.0;

/// Target speed when entering POSITION2, m/s.
pub const POSITION2_TARGET_SPEED_MS: f32 = 3.0;

/// Minimum time in AIRBRAKE before POSITION1 is allowed, ms.
pub const MIN_AIRBRAKE_MS: u32 = 1000;

/// Attitude-error exit from AIRBRAKE, centidegrees.
pub const ATTITUDE_ERROR_THRESHOLD_CD: i32 = 1000;

/// Fresh `velocity_match_ms` window, ms.
pub const VELOCITY_MATCH_FRESH_MS: u32 = 1000;

/// `waypoint_controller` dest-refresh period, ms.
pub const WP_DEST_REFRESH_MS: u32 = 500;

/// Stale `takeoff_last_run_ms` that re-arms the rudder wait, ms.
pub const TAKEOFF_RUDDER_STALE_MS: u32 = 1000;

/// Extra seconds of closing speed added to the approach stop distance.
pub const APPROACH_AIRBRAKE_MARGIN_S: f32 = 2.0;

/// POSITION1 → POSITION2 groundspeed gate: `3 * position2_target_speed`.
pub const POSITION2_SPEED_MULT: f32 = 3.0;

/// Tiltrotor airbrake Z-suppress after `last_pidz_active_ms`, ms.
pub const TILT_AIRBRAKE_PIDZ_MS: u32 = 2000;

/// Heading-error exit from AIRBRAKE, degrees.
pub const AIRBRAKE_HEADING_ERR_DEG: f32 = 60.0;

/// Which leftover controller produced a result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControllerKind {
    /// `setup()` failed — `vtol_position_controller` returned.
    None,
    /// `vtol_position_controller`.
    Position,
    /// `takeoff_controller` climbed (armed, not waiting).
    Takeoff,
    /// `takeoff_controller` returned in a wait / disarmed gate.
    TakeoffWait,
    /// `waypoint_controller`.
    Waypoint,
}

/// Why `takeoff_controller` returned before `setup_target_position`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TakeoffWait {
    /// Not waiting — climb path ran.
    None,
    /// `!arming.is_armed_and_safety_off()`.
    Disarmed,
    /// GUIDED takeoff, tiltrotor not `fully_up()`.
    Tilt,
    /// ESC RPM check failed (`motors_takeoff_check`).
    Rpm,
    /// Rudder-arm, stick not recentered.
    Rudder,
}

/// Horizontal `QPOS_*` action this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HorizontalAction {
    /// `QPOS_NONE` / approach failsafe / airbrake exit / tailsitter stop.
    EnterPosition1,
    /// Approach inside stop distance, motors not yet unlimited.
    EnterAirbrake,
    /// POSITION1 close / slow / tilt done.
    EnterPosition2,
    /// Stay in the current state and run XY.
    Hold,
    /// Tailsitter still in VTOL transition — skip XY.
    Skip,
    /// `QPOS_LAND_COMPLETE` — nothing to do.
    Idle,
}

/// Height-control branch of `vtol_position_controller`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeightControl {
    /// Approach / airbrake with transition complete, or tailsitter trans.
    RelaxZ,
    /// GUIDED / AUTO loiter / QRTL — `input_pos_vel_accel_D`.
    HoldAlt,
    /// Other POSITION1/2 — `set_climb_rate_ms(0)`.
    ClimbZero,
    /// LAND_DESCEND / LAND_FINAL — `landing_descent_rate_ms`.
    LandDescent,
    /// LAND_ABORT — climb at WP-nav up speed.
    LandAbortClimb,
    /// LAND_COMPLETE / not reached.
    None,
}

/// Inputs [`QuadPlane::vtol_position_controller`] reads from Plane / COP.
///
/// `stopping_distance_m` is a later leftover row — pass the value in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `arming.is_armed_and_safety_off()`.
    pub armed: bool,
    /// `in_vtol_mode()` — approach failsafe into POSITION1.
    pub in_vtol_mode: bool,
    /// `tailsitter.enabled()`.
    pub tailsitter_enabled: bool,
    /// `tailsitter.in_vtol_transition(now_ms)`.
    pub tailsitter_in_vtol_transition: bool,
    /// `tiltrotor.enabled()`.
    pub tiltrotor_enabled: bool,
    /// `tiltrotor.tilt_angle_achieved()`.
    pub tilt_angle_achieved: bool,
    /// `tiltrotor.tilt_over_max_angle()`.
    pub tilt_over_max_angle: bool,
    /// `tiltrotor.current_tilt >= get_fully_forward_tilt()`.
    pub fully_forward_tilt: bool,
    /// Motors already `THROTTLE_UNLIMITED`.
    pub spool_unlimited: bool,
    /// `transition->complete()`.
    pub transition_complete: bool,
    /// `plane.auto_state.wp_distance` (metres).
    pub wp_distance_m: f32,
    /// `stopping_distance_m()` (later leftover) — metres.
    pub stopping_distance_m: f32,
    /// `landing_closing_velocity_NE_ms().length()`.
    pub closing_speed_ms: f32,
    /// `landing_desired_closing_velocity_NE_ms().length()`.
    pub desired_closing_speed_ms: f32,
    /// EAS, or groundspeed when airspeed is unavailable.
    pub aspeed_ms: f32,
    /// `MAX(airspeed_min - 2, assist.speed)`.
    pub aspeed_threshold_ms: f32,
    /// Closing vs desired-closing heading error, degrees.
    pub heading_err_deg: f32,
    /// `labs(ahrs.roll_sensor - nav_roll_cd)`.
    pub roll_err_cd: i32,
    /// `labs(ahrs.pitch_sensor - nav_pitch_cd)`.
    pub pitch_err_cd: i32,
    /// `landing_closing_velocity_NE_ms().length_squared()`.
    pub rel_groundspeed_sq: f32,
    /// `mode_guided` or AUTO loiter nav-cmd.
    pub guided_or_loiter: bool,
    /// `control_mode == mode_qrtl`.
    pub qrtl: bool,
    /// `poscontrol.last_pidz_active_ms`.
    pub last_pidz_active_ms: u32,
    /// `poscontrol.target_speed_ms` for the QPOS row.
    pub target_speed_ms: f32,
    /// `poscontrol.target_accel_mss` for the QPOS row.
    pub target_accel_mss: f32,
    /// `poscontrol.overshoot` for the QPOS row.
    pub overshoot: bool,
}

impl PositionControllerView {
    /// Armed SLT approach, far from the land WP.
    #[must_use]
    pub const fn approach_far(now_ms: u32) -> Self {
        Self {
            now_ms,
            armed: true,
            in_vtol_mode: false,
            tailsitter_enabled: false,
            tailsitter_in_vtol_transition: false,
            tiltrotor_enabled: false,
            tilt_angle_achieved: true,
            tilt_over_max_angle: false,
            fully_forward_tilt: false,
            spool_unlimited: false,
            transition_complete: true,
            wp_distance_m: 200.0,
            stopping_distance_m: 40.0,
            closing_speed_ms: 20.0,
            desired_closing_speed_ms: 15.0,
            aspeed_ms: 20.0,
            aspeed_threshold_ms: 10.0,
            heading_err_deg: 0.0,
            roll_err_cd: 0,
            pitch_err_cd: 0,
            rel_groundspeed_sq: 400.0,
            guided_or_loiter: false,
            qrtl: false,
            last_pidz_active_ms: 0,
            target_speed_ms: 0.0,
            target_accel_mss: 0.0,
            overshoot: false,
        }
    }

    /// Armed SLT approach inside the stop distance.
    #[must_use]
    pub const fn approach_close(now_ms: u32) -> Self {
        let mut view = Self::approach_far(now_ms);
        view.wp_distance_m = 20.0;
        view.closing_speed_ms = 8.0;
        view.stopping_distance_m = 5.0;
        view
    }
}

/// Side-effects of [`QuadPlane::vtol_position_controller`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PositionControllerResult {
    /// Controller kind that ran.
    pub kind: ControllerKind,
    /// `poscontrol.last_run_ms` was written (armed).
    pub last_run_updated: bool,
    /// `INTERNAL_ERROR(flow_of_control)` path.
    pub flow_of_control: bool,
    /// Horizontal switch action.
    pub horizontal: HorizontalAction,
    /// Height-control branch after the horizontal switch.
    pub height: HeightControl,
    /// Skip `run_z_controller` this tick.
    pub suppress_z: bool,
    /// Would have called leftover `hold_hover(0)` (later row).
    pub would_hold_hover: bool,
    /// Would have called leftover `hold_stabilize(0.01)` (later row).
    pub would_hold_stabilize: bool,
    /// `log_QPOS` wrote this tick.
    pub logged_qpos: bool,
    /// `poscontrol.get_state()` after the tick.
    pub state: PositionControlState,
}

impl PositionControllerResult {
    /// `setup()` failed.
    #[must_use]
    pub const fn setup_failed() -> Self {
        Self {
            kind: ControllerKind::None,
            last_run_updated: false,
            flow_of_control: false,
            horizontal: HorizontalAction::Idle,
            height: HeightControl::None,
            suppress_z: false,
            would_hold_hover: false,
            would_hold_stabilize: false,
            logged_qpos: false,
            state: PositionControlState::None,
        }
    }
}

/// Inputs [`QuadPlane::takeoff_controller`] reads from Plane / motors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffControllerView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `arming.is_armed_and_safety_off()`.
    pub armed: bool,
    /// `motors->get_desired_spool_state() == THROTTLE_UNLIMITED`.
    pub spool_unlimited: bool,
    /// `control_mode == mode_guided`.
    pub in_guided: bool,
    /// `tiltrotor.enabled()`.
    pub tiltrotor_enabled: bool,
    /// `tiltrotor.fully_up()`.
    pub tiltrotor_fully_up: bool,
    /// `motors_takeoff_check` (true when ESC-telem is off).
    pub motor_check_passed: bool,
    /// Rudder-arm and stick not recentered.
    pub rudder_arm_wait: bool,
    /// `current_loc.alt * 0.01` (metres).
    pub alt_m: f32,
    /// `Q_TKOFF_NAVALT_MIN`.
    pub navalt_min_m: f32,
}

impl TakeoffControllerView {
    /// Armed, spool unlimited, no wait gates.
    #[must_use]
    pub const fn climbing(now_ms: u32, alt_m: f32) -> Self {
        Self {
            now_ms,
            armed: true,
            spool_unlimited: true,
            in_guided: false,
            tiltrotor_enabled: false,
            tiltrotor_fully_up: true,
            motor_check_passed: true,
            rudder_arm_wait: false,
            alt_m,
            navalt_min_m: 0.0,
        }
    }
}

/// Side-effects of [`QuadPlane::takeoff_controller`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffControllerResult {
    /// Controller kind.
    pub kind: ControllerKind,
    /// Wait / disarmed gate (if any).
    pub wait: TakeoffWait,
    /// `setup_target_position` ran.
    pub setup_target: bool,
    /// Below `Q_TKOFF_NAVALT_MIN` — XY nav relaxed.
    pub no_navigation: bool,
    /// `takeoff_start_time_ms` after the tick.
    pub takeoff_start_time_ms: u32,
}

/// Inputs [`QuadPlane::waypoint_controller`] reads from Plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WaypointControllerView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `next_WP_loc.same_loc_as(last_auto_target)`.
    pub same_loc_as_last: bool,
}

/// Side-effects of [`QuadPlane::waypoint_controller`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WaypointControllerResult {
    /// Controller kind.
    pub kind: ControllerKind,
    /// `wp_nav->set_wp_destination_NED_m` ran.
    pub refreshed_destination: bool,
    /// `setup_target_position` ran.
    pub setup_target: bool,
}

/// Leftover controller timers stored on [`QuadPlane`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllers {
    last_run_ms: u32,
    state_start_ms: u32,
    takeoff_last_run_ms: u32,
    takeoff_start_time_ms: u32,
    takeoff_start_alt_m: f32,
    last_loiter_ms: u32,
}

impl Default for PositionControllers {
    fn default() -> Self {
        Self::new()
    }
}

impl PositionControllers {
    /// Zeroed leftover controller timers.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            last_run_ms: 0,
            state_start_ms: 0,
            takeoff_last_run_ms: 0,
            takeoff_start_time_ms: 0,
            takeoff_start_alt_m: 0.0,
            last_loiter_ms: 0,
        }
    }

    /// `poscontrol.last_run_ms`.
    #[must_use]
    pub const fn last_run_ms(&self) -> u32 {
        self.last_run_ms
    }

    /// `poscontrol.time_since_state_start_ms` origin.
    #[must_use]
    pub const fn state_start_ms(&self) -> u32 {
        self.state_start_ms
    }

    /// Test poke for the AIRBRAKE minimum-time gate.
    pub fn set_state_start_ms(&mut self, state_start_ms: u32) {
        self.state_start_ms = state_start_ms;
    }

    /// `takeoff_last_run_ms`.
    #[must_use]
    pub const fn takeoff_last_run_ms(&self) -> u32 {
        self.takeoff_last_run_ms
    }

    /// `takeoff_start_time_ms` written while waiting.
    #[must_use]
    pub const fn takeoff_start_time_ms(&self) -> u32 {
        self.takeoff_start_time_ms
    }

    /// `takeoff_start_alt_m` for the nav-alt-min gate.
    #[must_use]
    pub const fn takeoff_start_alt_m(&self) -> f32 {
        self.takeoff_start_alt_m
    }

    /// `last_loiter_ms`.
    #[must_use]
    pub const fn last_loiter_ms(&self) -> u32 {
        self.last_loiter_ms
    }

    /// Test poke so a later waypoint tick can be stale or fresh.
    pub fn set_last_loiter_ms(&mut self, last_loiter_ms: u32) {
        self.last_loiter_ms = last_loiter_ms;
    }
}

/// Approach stop distance: leftover `stopping_distance_m()` plus 2 s.
#[must_use]
pub const fn approach_stop_distance_m(stopping_distance_m: f32, closing_speed_ms: f32) -> f32 {
    stopping_distance_m + APPROACH_AIRBRAKE_MARGIN_S * closing_speed_ms
}

/// Tailsitters / already-unlimited motors skip AIRBRAKE.
#[must_use]
pub const fn approach_uses_airbrake(tailsitter_enabled: bool, spool_unlimited: bool) -> bool {
    !(tailsitter_enabled || spool_unlimited)
}

/// `poscontrol.time_since_state_start_ms() > 1000`.
#[must_use]
pub const fn airbrake_min_time_elapsed(time_since_state_ms: u32) -> bool {
    time_since_state_ms > MIN_AIRBRAKE_MS
}

/// AIRBRAKE → POSITION1 speed / attitude / heading exits.
#[must_use]
pub fn airbrake_exit_to_position1(
    aspeed_ms: f32,
    aspeed_threshold_ms: f32,
    heading_err_deg: f32,
    closing_speed_ms: f32,
    desired_closing_speed_ms: f32,
    roll_err_cd: i32,
    pitch_err_cd: i32,
) -> bool {
    let fast_a = desired_closing_speed_ms * 1.2;
    let fast_b = desired_closing_speed_ms + 2.0;
    let too_fast = if fast_a > fast_b { fast_a } else { fast_b };
    aspeed_ms < aspeed_threshold_ms
        || heading_err_deg.abs() > AIRBRAKE_HEADING_ERR_DEG
        || closing_speed_ms > too_fast
        || closing_speed_ms < desired_closing_speed_ms * 0.5
        || roll_err_cd.unsigned_abs() > ATTITUDE_ERROR_THRESHOLD_CD as u32
        || pitch_err_cd.unsigned_abs() > ATTITUDE_ERROR_THRESHOLD_CD as u32
}

/// POSITION1 → POSITION2 distance / tilt / speed gate.
#[must_use]
pub fn position1_enters_position2(
    wp_distance_m: f32,
    tilt_angle_achieved: bool,
    rel_groundspeed_sq: f32,
) -> bool {
    let speed_gate = POSITION2_SPEED_MULT * POSITION2_TARGET_SPEED_MS;
    wp_distance_m < POSITION2_DIST_THRESHOLD_M
        && tilt_angle_achieved
        && rel_groundspeed_sq.abs() < speed_gate * speed_gate
}

/// `!same_loc || now - last_loiter_ms > 500`.
#[must_use]
pub const fn waypoint_refresh_destination(
    same_loc_as_last: bool,
    now_ms: u32,
    last_loiter_ms: u32,
) -> bool {
    !same_loc_as_last || now_ms.wrapping_sub(last_loiter_ms) > WP_DEST_REFRESH_MS
}

/// GUIDED takeoff waiting for tilt-up.
#[must_use]
pub const fn takeoff_wait_tilt(
    spool_unlimited: bool,
    in_guided: bool,
    guided_takeoff: bool,
    tiltrotor_enabled: bool,
    tiltrotor_fully_up: bool,
) -> bool {
    !spool_unlimited && in_guided && guided_takeoff && tiltrotor_enabled && !tiltrotor_fully_up
}

/// Height-control branch from the current `QPOS_*` state.
#[must_use]
pub const fn height_control_for(
    state: PositionControlState,
    tailsitter_in_vtol_transition: bool,
    transition_complete: bool,
    guided_or_loiter: bool,
    qrtl: bool,
) -> HeightControl {
    match state {
        PositionControlState::Approach | PositionControlState::Airbrake => {
            if transition_complete {
                HeightControl::RelaxZ
            } else {
                HeightControl::None
            }
        }
        PositionControlState::Position1 => {
            if tailsitter_in_vtol_transition {
                HeightControl::RelaxZ
            } else if guided_or_loiter || qrtl {
                HeightControl::HoldAlt
            } else {
                HeightControl::ClimbZero
            }
        }
        PositionControlState::Position2 => {
            if guided_or_loiter || qrtl {
                HeightControl::HoldAlt
            } else {
                HeightControl::ClimbZero
            }
        }
        PositionControlState::LandAbort => HeightControl::LandAbortClimb,
        PositionControlState::LandDescend | PositionControlState::LandFinal => {
            HeightControl::LandDescent
        }
        PositionControlState::LandComplete | PositionControlState::None => HeightControl::None,
    }
}

impl QuadPlane {
    /// Leftover position / takeoff / waypoint controller timers.
    #[must_use]
    pub const fn position_controllers(&self) -> &PositionControllers {
        &self.position_controller
    }

    /// Mutable leftover controller timers (tests poke AIRBRAKE / loiter).
    pub fn position_controllers_mut(&mut self) -> &mut PositionControllers {
        &mut self.position_controller
    }

    /// Upstream `QuadPlane::vtol_position_controller`.
    ///
    /// `setup()` early-return, `last_run_ms` when armed, then the
    /// `QPOS_*` horizontal switch + height branch. `hold_hover` /
    /// `hold_stabilize` stay on a later leftover row — this stub
    /// only records that they would run. QPOS streams through
    /// [`QuadPlane::maybe_log_qpos`].
    pub fn vtol_position_controller(
        &mut self,
        view: PositionControllerView,
    ) -> PositionControllerResult {
        if !self.setup() {
            return PositionControllerResult::setup_failed();
        }

        let mut last_run_updated = false;
        if view.armed {
            self.position_controller.last_run_ms = view.now_ms;
            last_run_updated = true;
        }

        let state = self.poscontrol.state();
        let time_since = view
            .now_ms
            .wrapping_sub(self.position_controller.state_start_ms);
        let stop_m = approach_stop_distance_m(view.stopping_distance_m, view.closing_speed_ms);

        let mut flow_of_control = false;
        let mut suppress_z = false;
        let mut would_hold_hover = false;
        let mut would_hold_stabilize = false;

        let horizontal = match state {
            PositionControlState::None => {
                self.enter_pos_state(PositionControlState::Position1, view.now_ms);
                flow_of_control = true;
                HorizontalAction::EnterPosition1
            }
            PositionControlState::Approach if view.in_vtol_mode => {
                self.enter_pos_state(PositionControlState::Position1, view.now_ms);
                flow_of_control = true;
                HorizontalAction::EnterPosition1
            }
            PositionControlState::Approach | PositionControlState::Airbrake => {
                if view.tiltrotor_enabled && state == PositionControlState::Airbrake {
                    let pidz_stale =
                        view.now_ms.wrapping_sub(view.last_pidz_active_ms) > TILT_AIRBRAKE_PIDZ_MS;
                    if (pidz_stale && view.tilt_over_max_angle) || view.fully_forward_tilt {
                        suppress_z = true;
                        would_hold_stabilize = true;
                    }
                }
                if !suppress_z && state == PositionControlState::Airbrake {
                    would_hold_hover = true;
                    suppress_z = true;
                }
                if state == PositionControlState::Approach && view.wp_distance_m < stop_m {
                    if approach_uses_airbrake(view.tailsitter_enabled, view.spool_unlimited) {
                        self.enter_pos_state(PositionControlState::Airbrake, view.now_ms);
                        HorizontalAction::EnterAirbrake
                    } else {
                        self.enter_pos_state(PositionControlState::Position1, view.now_ms);
                        HorizontalAction::EnterPosition1
                    }
                } else if state == PositionControlState::Airbrake
                    && airbrake_min_time_elapsed(time_since)
                    && airbrake_exit_to_position1(
                        view.aspeed_ms,
                        view.aspeed_threshold_ms,
                        view.heading_err_deg,
                        view.closing_speed_ms,
                        view.desired_closing_speed_ms,
                        view.roll_err_cd,
                        view.pitch_err_cd,
                    )
                {
                    self.enter_pos_state(PositionControlState::Position1, view.now_ms);
                    HorizontalAction::EnterPosition1
                } else {
                    HorizontalAction::Hold
                }
            }
            PositionControlState::Position1 => {
                if view.tailsitter_in_vtol_transition {
                    HorizontalAction::Skip
                } else if position1_enters_position2(
                    view.wp_distance_m,
                    view.tilt_angle_achieved,
                    view.rel_groundspeed_sq,
                ) {
                    self.enter_pos_state(PositionControlState::Position2, view.now_ms);
                    self.poscontrol_mut().set_pilot_correction(false, false);
                    HorizontalAction::EnterPosition2
                } else {
                    HorizontalAction::Hold
                }
            }
            PositionControlState::Position2
            | PositionControlState::LandAbort
            | PositionControlState::LandDescend
            | PositionControlState::LandFinal => HorizontalAction::Hold,
            PositionControlState::LandComplete => HorizontalAction::Idle,
        };

        let new_state = self.poscontrol.state();
        let height = height_control_for(
            new_state,
            view.tailsitter_in_vtol_transition,
            view.transition_complete,
            view.guided_or_loiter,
            view.qrtl,
        );

        let logged_qpos = self.maybe_log_qpos(
            view.now_ms,
            QPosView {
                wp_distance: view.wp_distance_m,
                target_speed_ms: view.target_speed_ms,
                target_accel_mss: view.target_accel_mss,
                overshoot: view.overshoot,
            },
        );

        PositionControllerResult {
            kind: ControllerKind::Position,
            last_run_updated,
            flow_of_control,
            horizontal,
            height,
            suppress_z,
            would_hold_hover,
            would_hold_stabilize,
            logged_qpos,
            state: new_state,
        }
    }

    /// Upstream `QuadPlane::takeoff_controller`.
    ///
    /// Disarmed and spool-wait gates return early. The climb path
    /// records `setup_target_position` and the `Q_TKOFF_NAVALT_MIN`
    /// no-navigation window. XY / Z COP calls stay in COP.
    pub fn takeoff_controller(&mut self, view: TakeoffControllerView) -> TakeoffControllerResult {
        if !view.armed {
            return TakeoffControllerResult {
                kind: ControllerKind::TakeoffWait,
                wait: TakeoffWait::Disarmed,
                setup_target: false,
                no_navigation: false,
                takeoff_start_time_ms: self.position_controller.takeoff_start_time_ms,
            };
        }

        if !view.spool_unlimited {
            if takeoff_wait_tilt(
                view.spool_unlimited,
                view.in_guided,
                self.guided_takeoff(),
                view.tiltrotor_enabled,
                view.tiltrotor_fully_up,
            ) {
                self.position_controller.takeoff_start_time_ms = view.now_ms;
                return TakeoffControllerResult {
                    kind: ControllerKind::TakeoffWait,
                    wait: TakeoffWait::Tilt,
                    setup_target: false,
                    no_navigation: false,
                    takeoff_start_time_ms: view.now_ms,
                };
            }
            if !view.motor_check_passed {
                self.position_controller.takeoff_start_time_ms = view.now_ms;
                return TakeoffControllerResult {
                    kind: ControllerKind::TakeoffWait,
                    wait: TakeoffWait::Rpm,
                    setup_target: false,
                    no_navigation: false,
                    takeoff_start_time_ms: view.now_ms,
                };
            }
            let rudder_stale = self.position_controller.takeoff_last_run_ms == 0
                || view
                    .now_ms
                    .wrapping_sub(self.position_controller.takeoff_last_run_ms)
                    > TAKEOFF_RUDDER_STALE_MS;
            if view.rudder_arm_wait && rudder_stale {
                self.position_controller.takeoff_start_time_ms = view.now_ms;
                return TakeoffControllerResult {
                    kind: ControllerKind::TakeoffWait,
                    wait: TakeoffWait::Rudder,
                    setup_target: false,
                    no_navigation: false,
                    takeoff_start_time_ms: view.now_ms,
                };
            }
        }

        let mut no_navigation = false;
        if view.navalt_min_m > 0.0 {
            if self.position_controller.takeoff_last_run_ms == 0
                || view
                    .now_ms
                    .wrapping_sub(self.position_controller.takeoff_last_run_ms)
                    > TAKEOFF_RUDDER_STALE_MS
            {
                self.position_controller.takeoff_start_alt_m = view.alt_m;
            }
            if view.alt_m - self.position_controller.takeoff_start_alt_m < view.navalt_min_m {
                no_navigation = true;
            }
        }
        self.position_controller.takeoff_last_run_ms = view.now_ms;

        TakeoffControllerResult {
            kind: ControllerKind::Takeoff,
            wait: TakeoffWait::None,
            setup_target: true,
            no_navigation,
            takeoff_start_time_ms: self.position_controller.takeoff_start_time_ms,
        }
    }

    /// Upstream `QuadPlane::waypoint_controller`.
    ///
    /// Refreshes the WP-nav destination when the AUTO target moved or
    /// `last_loiter_ms` is older than 500 ms. Attitude / Z COP calls
    /// stay in COP.
    pub fn waypoint_controller(
        &mut self,
        view: WaypointControllerView,
    ) -> WaypointControllerResult {
        let refreshed = waypoint_refresh_destination(
            view.same_loc_as_last,
            view.now_ms,
            self.position_controller.last_loiter_ms,
        );
        self.position_controller.last_loiter_ms = view.now_ms;
        WaypointControllerResult {
            kind: ControllerKind::Waypoint,
            refreshed_destination: refreshed,
            setup_target: true,
        }
    }

    fn enter_pos_state(&mut self, state: PositionControlState, now_ms: u32) {
        self.poscontrol.set_state(state);
        self.position_controller.state_start_ms = now_ms;
    }
}

/// `qpos_period_elapsed` re-export so tests can name the leftover gate.
#[must_use]
pub const fn position_qpos_period_elapsed(now_ms: u32, last_qpos_log_ms: u32) -> bool {
    qpos_period_elapsed(now_ms, last_qpos_log_ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn approach_stop_and_airbrake_gates() {
        let stop = approach_stop_distance_m(10.0, 5.0);
        assert!(stop > 19.0 && stop < 21.0);
        assert!(approach_uses_airbrake(false, false));
        assert!(!approach_uses_airbrake(true, false));
        assert!(!approach_uses_airbrake(false, true));
        assert!(!airbrake_min_time_elapsed(1000));
        assert!(airbrake_min_time_elapsed(1001));
        assert!(airbrake_exit_to_position1(9.0, 10.0, 0.0, 5.0, 8.0, 0, 0));
        assert!(!airbrake_exit_to_position1(15.0, 10.0, 0.0, 8.0, 8.0, 0, 0));
        assert!(airbrake_exit_to_position1(15.0, 10.0, 61.0, 8.0, 8.0, 0, 0));
        assert!(position1_enters_position2(9.0, true, 0.0));
        assert!(!position1_enters_position2(10.0, true, 0.0));
        assert!(!position1_enters_position2(9.0, false, 0.0));
        assert!(!position1_enters_position2(9.0, true, 82.0));
    }

    #[test]
    fn waypoint_refresh_and_height_branches() {
        assert!(waypoint_refresh_destination(false, 10, 10));
        assert!(waypoint_refresh_destination(true, 501, 0));
        assert!(!waypoint_refresh_destination(true, 500, 0));
        assert_eq!(
            height_control_for(PositionControlState::Approach, false, true, false, false),
            HeightControl::RelaxZ
        );
        assert_eq!(
            height_control_for(PositionControlState::Position1, true, true, false, false),
            HeightControl::RelaxZ
        );
        assert_eq!(
            height_control_for(PositionControlState::Position2, false, true, true, false),
            HeightControl::HoldAlt
        );
        assert_eq!(
            height_control_for(PositionControlState::LandAbort, false, true, false, false),
            HeightControl::LandAbortClimb
        );
        assert!(takeoff_wait_tilt(false, true, true, true, false));
        assert!(!takeoff_wait_tilt(true, true, true, true, false));
        assert!(position_qpos_period_elapsed(40, 0));
    }
}
