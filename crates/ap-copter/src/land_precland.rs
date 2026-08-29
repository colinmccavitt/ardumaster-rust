//! Mode-layer precision-landing leftovers, upstream `ArduCopter/mode.cpp`
//! `Mode::land_run_normal_or_precland`, `Mode::precland_run`,
//! `Mode::precland_retry_position`, and the `AC_PRECLAND_ENABLED` override
//! inside `Mode::land_run_vertical_control`.
//!
//! Tracked as **COP-013** last 6%. The `AC_PrecLand` crate ([`ap_precland`])
//! is already on main (COP-028). This module is the Mode consumer those
//! leftovers were blocked on: which landing runner fires, how the retry
//! state machine is interpreted, how a retry position is flown, and how a
//! live target may hold or slow the descent.
//!
//! Controller calls stay leftovers. A decision that is also an action is
//! easier to test when the two are separated — the same split
//! [`crate::land::land_descent`] already uses.

use crate::land::LandDescent;
use crate::land_horizontal::land_cancelled_by_throttle;
use ap_math::scalar::is_zero;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_precland::{FailSafeAction, StateMachine, StateMachineFrontend, StateMachineWorld, Status};

/// Horizontal error at which the near-ground slowdown is fully applied,
/// upstream the local `precland_acceptable_error_m` in
/// `land_run_vertical_control`.
pub const PRECLAND_ACCEPTABLE_ERROR_M: f32 = 0.15;

/// Floor on the near-ground precland descent, m/s, upstream
/// `precland_min_descent_speed_ms`.
pub const PRECLAND_MIN_DESCENT_SPEED_MS: f32 = 0.1;

/// Measurement-z floor of the near-ground slowdown, metres (NED down).
/// Upstream `target_pos_meas_ned_m.z > 0.35`.
pub const PRECLAND_SLOWDOWN_MEAS_Z_MIN_M: f32 = 0.35;

/// Measurement-z ceiling of the near-ground slowdown, metres (NED down).
/// Upstream `target_pos_meas_ned_m.z < 2.0`.
pub const PRECLAND_SLOWDOWN_MEAS_Z_MAX_M: f32 = 2.0;

/// Speed handed to `input_pos_NED_m` during a retry. Upstream `0.0f`.
pub const RETRY_POS_SPEED_MS: f32 = 0.0;

/// Accel handed to `input_pos_NED_m` during a retry. Upstream `10.0`.
pub const RETRY_POS_ACCEL_MSS: f32 = 10.0;

/// COP-013 PrecLand mode leftovers this module closes.
///
/// Empty: the four Mode consumers of `AC_PrecLand` are here. Precision
/// loiter (`ModeLoiter::do_precision_loiter`) stays COP-015.
pub const REMAINING: &[&str] = &[];

/// Which landing runner `land_run_normal_or_precland` asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandOrPrecland {
    /// `land_run_horiz_and_vert_control(pause_descent)`.
    Normal {
        /// The pause argument forwarded to the normal landing pair.
        pause_descent: bool,
    },
    /// `precland_run()`. The state machine owns pause from here.
    PrecLand,
}

/// Upstream `Mode::land_run_normal_or_precland`.
///
/// # Pause and disabled look the same
///
/// A paused descent must not start the retry machine: a failsafe that
/// asked for four seconds of hover would otherwise spend them climbing
/// to a retry altitude. A disabled `AC_PrecLand` is the same runner
/// with no target to chase. Only an enabled, unpaused landing hands
/// the tick to [`precland_run`].
#[must_use]
pub fn land_run_normal_or_precland(pause_descent: bool, precland_enabled: bool) -> LandOrPrecland {
    if pause_descent || !precland_enabled {
        LandOrPrecland::Normal { pause_descent }
    } else {
        LandOrPrecland::PrecLand
    }
}

/// What `precland_run` asked the vehicle to do this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrecLandRunAction {
    /// `precland_retry_position(retry_pos)`.
    RetryPosition,
    /// `land_run_horiz_and_vert_control(pause_descent)`.
    HorizAndVert {
        /// `true` holds the descent (`HOLD_POS` failsafe).
        pause_descent: bool,
    },
}

/// Leftover of one `Mode::precland_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrecLandRun {
    /// Which C++ runner fired.
    pub action: PrecLandRunAction,
    /// Position written by the state machine this tick, if any.
    ///
    /// `None` is the leftover of C++ leaving the caller's `Vector3p`
    /// unchanged. A freshly declared retry vector is zero, so
    /// [`Status::Retrying`] with `None` still flies the zero vector.
    pub retry_pos_m: Option<Vector3f>,
    /// `Status::ERROR` asked `INTERNAL_ERROR(flow_of_control)` and then
    /// fell through to descend.
    pub need_internal_error: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Retrying")`.
    pub need_gcs_retrying: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Retry Completed")`.
    pub need_gcs_retry_completed: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Failsafe Measures")`.
    pub need_gcs_failsafe: bool,
}

fn horiz_and_vert(pause_descent: bool) -> PrecLandRun {
    PrecLandRun {
        action: PrecLandRunAction::HorizAndVert { pause_descent },
        retry_pos_m: None,
        need_internal_error: false,
        need_gcs_retrying: false,
        need_gcs_retry_completed: false,
        need_gcs_failsafe: false,
    }
}

/// Upstream `Mode::precland_run`.
///
/// # A repositioning pilot owns the landing
///
/// Once `land_repo_active` is set the state machine is not consulted.
/// Retries and failsafe holds would fight a pilot who has already taken
/// the aircraft; the leftover is a normal descend. The machine is not
/// even stepped — a later release must not inherit a retry that was
/// never flown.
///
/// # `ERROR` reports and then descends
///
/// Upstream `INTERNAL_ERROR` then `FALLTHROUGH` into `DESCEND`. The
/// leftover records the report; the action is still the normal landing
/// pair with pause false.
#[must_use]
pub fn precland_run(
    land_repo_active: bool,
    machine: &mut StateMachine,
    frontend: &StateMachineFrontend,
    world: &StateMachineWorld,
) -> PrecLandRun {
    if land_repo_active {
        return horiz_and_vert(false);
    }

    let update = machine.update(frontend, world);
    match update.status {
        Status::Retrying => PrecLandRun {
            action: PrecLandRunAction::RetryPosition,
            retry_pos_m: update.retry_pos_m,
            need_internal_error: false,
            need_gcs_retrying: update.need_gcs_retrying,
            need_gcs_retry_completed: update.need_gcs_retry_completed,
            need_gcs_failsafe: false,
        },
        Status::Failsafe => {
            let fs = machine.get_failsafe_actions(frontend, world);
            let pause_descent = match fs.action {
                FailSafeAction::Descend => false,
                FailSafeAction::HoldPos => true,
            };
            PrecLandRun {
                action: PrecLandRunAction::HorizAndVert { pause_descent },
                retry_pos_m: update.retry_pos_m,
                need_internal_error: false,
                need_gcs_retrying: update.need_gcs_retrying,
                need_gcs_retry_completed: update.need_gcs_retry_completed,
                need_gcs_failsafe: fs.need_gcs_failsafe,
            }
        }
        Status::Error => PrecLandRun {
            action: PrecLandRunAction::HorizAndVert {
                pause_descent: false,
            },
            retry_pos_m: update.retry_pos_m,
            need_internal_error: true,
            need_gcs_retrying: update.need_gcs_retrying,
            need_gcs_retry_completed: update.need_gcs_retry_completed,
            need_gcs_failsafe: false,
        },
        Status::Descend => PrecLandRun {
            action: PrecLandRunAction::HorizAndVert {
                pause_descent: false,
            },
            retry_pos_m: update.retry_pos_m,
            need_internal_error: false,
            need_gcs_retrying: update.need_gcs_retrying,
            need_gcs_retry_completed: update.need_gcs_retry_completed,
            need_gcs_failsafe: false,
        },
    }
}

/// Vehicle view `Mode::precland_retry_position` reads.
#[derive(Debug, Clone, Copy)]
pub struct PrecLandRetryView {
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `g.throttle_behavior`.
    pub throttle_behavior: i32,
    /// `copter.rc_throttle_control_in_filter.get()`.
    pub filtered_throttle_control_in: f32,
    /// `g.land_repositioning`.
    pub land_repositioning: bool,
    /// Pilot roll after `get_pilot_desired_lean_angles_rad`.
    pub target_roll_rad: f32,
    /// Pilot pitch after `get_pilot_desired_lean_angles_rad`.
    pub target_pitch_rad: f32,
    /// `copter.ap.land_repo_active` on entry.
    pub land_repo_active: bool,
    /// The NED retry location the state machine commanded, metres.
    pub retry_pos_ned_m: Vector3f,
}

/// Leftover of one `Mode::precland_retry_position` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrecLandRetry {
    /// High-throttle cancel asked `set_mode(LOITER then ALT_HOLD)`.
    pub cancel: bool,
    /// `copter.ap.land_repo_active` after this tick.
    pub land_repo_active: bool,
    /// First tick the pilot overrode lean. `LogEvent::LAND_REPO_ACTIVE`.
    pub need_log_repo_active: bool,
    /// High-throttle cancel. `LogEvent::LAND_CANCELLED_BY_PILOT`.
    pub need_log_cancel: bool,
    /// Argument of `pos_control->input_pos_NED_m`.
    pub retry_pos_ned_m: Vector3f,
    /// Speed argument of `input_pos_NED_m`. Always [`RETRY_POS_SPEED_MS`].
    pub retry_speed_ms: f32,
    /// Accel argument of `input_pos_NED_m`. Always [`RETRY_POS_ACCEL_MSS`].
    pub retry_accel_mss: f32,
    /// `NE_update_controller` always runs.
    pub update_ne: bool,
    /// `D_update_controller` always runs.
    pub update_d: bool,
    /// `input_thrust_vector_heading` always runs.
    pub attitude: bool,
}

/// Upstream `Mode::precland_retry_position`.
///
/// # Lean angles, not reposition velocity
///
/// The same pilot-takes-over idea lives in
/// [`crate::land_horizontal::reposition_state`], but that leftover
/// reads a velocity. This one reads lean. Upstream copied the check
/// and then let the two drift; matching the copy is the leftover.
///
/// Letting go does **not** clear `land_repo_active` here. There is no
/// `allow_precland_after_reposition` arm on the retry path.
#[must_use]
pub fn precland_retry_position(view: &PrecLandRetryView) -> PrecLandRetry {
    let cancel = land_cancelled_by_throttle(
        view.throttle_behavior,
        view.filtered_throttle_control_in,
        view.has_valid_input,
    );

    let mut land_repo_active = view.land_repo_active;
    let mut need_log_repo_active = false;
    if view.has_valid_input && view.land_repositioning {
        if !is_zero(view.target_roll_rad) || !is_zero(view.target_pitch_rad) {
            if !land_repo_active {
                need_log_repo_active = true;
            }
            land_repo_active = true;
        }
    }

    PrecLandRetry {
        cancel,
        land_repo_active,
        need_log_repo_active,
        need_log_cancel: cancel,
        retry_pos_ned_m: view.retry_pos_ned_m,
        retry_speed_ms: RETRY_POS_SPEED_MS,
        retry_accel_mss: RETRY_POS_ACCEL_MSS,
        update_ne: true,
        update_d: true,
        attitude: true,
    }
}

/// Whether the vertical precland override is live, upstream
/// `doing_precision_landing` in `land_run_vertical_control`.
///
/// All three are required. A repositioning pilot beats an acquired
/// target; a target without a live NE controller has nowhere to hold.
#[must_use]
pub fn doing_precision_landing(
    land_repo_active: bool,
    target_acquired: bool,
    navigating: bool,
) -> bool {
    !land_repo_active && target_acquired && navigating
}

/// Inputs the vertical precland override reads.
#[derive(Debug, Clone, Copy)]
pub struct PrecLandVerticalView {
    /// `pause_descent` of `land_run_vertical_control`.
    pub pause_descent: bool,
    /// [`doing_precision_landing`].
    pub doing_precision_landing: bool,
    /// `precland.get_target_position_m`. `None` leaves the error at zero.
    pub target_pos_ne_m: Option<Vector2f>,
    /// `pos_control->get_pos_estimate_NED_m().xy()`.
    pub current_pos_ne_m: Vector2f,
    /// `precland.get_max_xy_error_before_descending_m()`.
    pub max_horiz_pos_error_m: f32,
    /// `precland.get_target_position_measurement_NED_m().z`.
    pub target_pos_meas_ned_z_m: f32,
    /// `precland.do_fast_descend()`.
    pub do_fast_descend: bool,
    /// `mode_land.get_land_speed_ms()`. Sign is not trusted.
    pub land_speed_ms: f32,
}

/// Apply the `AC_PRECLAND` override to a [`LandDescent`].
///
/// [`crate::land::land_descent`] computes the demand *before* this
/// override. A caller that has precision landing active must run this
/// after it; one that does not can use the descent unmodified.
///
/// # Too far holds; too close slows
///
/// Horizontal error above `PLND_XY_DIST_MAX` (and that limit not zero)
/// zeros the climb rate so the aircraft slides onto the target before
/// it drops. Near the ground, and without `PLND_OPTION_FAST_DESCEND`,
/// the demand is replaced by a crawl that grows with horizontal error
/// and never goes above [`PRECLAND_MIN_DESCENT_SPEED_MS`] upward
/// (always a descent). The descent-limit lift from the base demand is
/// left alone — a hold is not an arrival.
#[must_use]
pub fn land_descent_precland_override(
    base: LandDescent,
    view: &PrecLandVerticalView,
) -> LandDescent {
    if view.pause_descent || !view.doing_precision_landing {
        return base;
    }

    let target_error_m = match view.target_pos_ne_m {
        Some(target) => (target - view.current_pos_ne_m).length(),
        None => 0.0,
    };

    let mut climb_rate_ms = base.climb_rate_ms;
    if target_error_m > view.max_horiz_pos_error_m && !is_zero(view.max_horiz_pos_error_m) {
        climb_rate_ms = 0.0;
    } else if view.target_pos_meas_ned_z_m > PRECLAND_SLOWDOWN_MEAS_Z_MIN_M
        && view.target_pos_meas_ned_z_m < PRECLAND_SLOWDOWN_MEAS_Z_MAX_M
        && !view.do_fast_descend
    {
        let max_descent_speed_ms = libm::fabsf(view.land_speed_ms) * 0.5;
        let land_slowdown_ms = libm::fmaxf(
            0.0,
            target_error_m * (max_descent_speed_ms / PRECLAND_ACCEPTABLE_ERROR_M),
        );
        climb_rate_ms = libm::fminf(
            -PRECLAND_MIN_DESCENT_SPEED_MS,
            -max_descent_speed_ms + land_slowdown_ms,
        );
    }

    LandDescent {
        climb_rate_ms,
        ignore_descent_limit: base.ignore_descent_limit,
    }
}
