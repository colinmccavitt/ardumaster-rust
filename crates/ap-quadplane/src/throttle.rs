//! QuadPlane throttle mix and tilt-wait-before-forward-flight,
//! upstream `QuadPlane::update_throttle_mix` and the TIMER
//! `tilt_fwd_complete` gate (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. During `AIRSPEED_WAIT` / `TIMER` with
//! assist on, the transition owns attitude-control mix
//! (`SLT_Transition::allow_update_throttle_mix`). Otherwise mix
//! follows the land-check / manual-throttle table. Completing
//! TIMER into forward flight also waits for tiltrotors:
//! `!tiltrotor.enabled() || tiltrotor.tilt_angle_achieved()`.
//!
//! This is not a rewrite of ap-motors mixing, `setup()` frame-class
//! selection, or the VT-003 TIMER dwell itself.

use crate::QuadPlane;

/// Maximum attitude error (deg) still treated as "landing" mix.
///
/// Upstream `#define LAND_CHECK_ANGLE_ERROR_DEG 30.0f`.
pub const LAND_CHECK_ANGLE_ERROR_DEG: f32 = 30.0;

/// Maximum roll/pitch target length (cd) still treated as "landing" mix.
///
/// Upstream `#define LAND_CHECK_LARGE_ANGLE_CD 1500.0f`.
pub const LAND_CHECK_LARGE_ANGLE_CD: f32 = 1500.0;

/// Maximum earth-frame accel length (m/s², gravity subtracted) for landing mix.
///
/// Upstream `#define LAND_CHECK_ACCEL_MOVING 3.0f`.
pub const LAND_CHECK_ACCEL_MOVING: f32 = 3.0;

/// Attitude-control throttle-mix demand from `update_throttle_mix`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThrottleMix {
    /// Transition is managing mix — `update_throttle_mix` returns early.
    Hold,
    /// `attitude_control->set_throttle_mix_min()`.
    Min,
    /// `attitude_control->set_throttle_mix_man()`.
    Man,
    /// `attitude_control->set_throttle_mix_max(1.0)`.
    Max,
}

/// Inputs `QuadPlane::update_throttle_mix` reads from the vehicle.
#[derive(Clone, Copy, Debug)]
pub struct ThrottleMixView {
    /// `SLT_Transition::allow_update_throttle_mix`.
    pub allow_update: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `control_mode->is_vtol_man_throttle()`.
    pub vtol_man_throttle: bool,
    /// `get_throttle_input()` (pilot throttle; `is_positive` is `> 0`).
    pub throttle_input: f32,
    /// `QuadPlane::air_mode_active()`.
    pub air_mode_active: bool,
    /// Attitude target roll, centidegrees.
    pub roll_target_cd: f32,
    /// Attitude target pitch, centidegrees.
    pub pitch_target_cd: f32,
    /// `attitude_control->get_att_error_angle_deg()`.
    pub att_error_deg: f32,
    /// `throttle_mix_accel_ef_filter.get().length()`.
    pub accel_ef_filt_len: f32,
    /// `pos_control->get_vel_desired_U_ms()` (up positive).
    pub vel_desired_u_ms: f32,
    /// `in_vtol_land_sequence()`.
    pub in_vtol_land_sequence: bool,
    /// `in_vtol_land_final()`.
    pub in_vtol_land_final: bool,
}

impl ThrottleMixView {
    /// Armed auto-throttle hover (small angles, no descent demand).
    ///
    /// `vel_desired_u_ms == 0` is `descent_not_demanded`, so
    /// [`QuadPlane::update_throttle_mix`] yields [`ThrottleMix::Max`]
    /// unless a landing-final override applies.
    #[must_use]
    pub const fn hover() -> Self {
        Self {
            allow_update: true,
            armed: true,
            vtol_man_throttle: false,
            throttle_input: 0.0,
            air_mode_active: false,
            roll_target_cd: 0.0,
            pitch_target_cd: 0.0,
            att_error_deg: 0.0,
            accel_ef_filt_len: 0.0,
            vel_desired_u_ms: 0.0,
            in_vtol_land_sequence: false,
            in_vtol_land_final: false,
        }
    }
}

/// Upstream `SLT_Transition::allow_update_throttle_mix`.
///
/// Transition owns mix while `assisted_flight` and the SLT state is
/// `AIRSPEED_WAIT` or `TIMER` (`in_transition`).
#[must_use]
pub const fn allow_update_throttle_mix(assisted_flight: bool, in_transition: bool) -> bool {
    !(assisted_flight && in_transition)
}

/// Upstream TIMER `tilt_fwd_complete`.
///
/// `!tiltrotor.enabled() || tiltrotor.tilt_angle_achieved()`, and
/// `tilt_angle_achieved` is `!enabled() || type != CONTINUOUS ||
/// angle_achieved`. Composed: wait only for an enabled continuous
/// tilt that has not reached the commanded angle.
#[must_use]
pub const fn tilt_fwd_complete(
    tiltrotor_enabled: bool,
    continuous_tilt: bool,
    angle_achieved: bool,
) -> bool {
    !tiltrotor_enabled || !continuous_tilt || angle_achieved
}

/// TIMER finishes only after the dwell *and* tilt-wait.
///
/// Upstream `transition_timer_ms > trans_time_ms && tilt_fwd_complete`.
#[must_use]
pub const fn timer_may_complete(timer_expired: bool, tilt_complete: bool) -> bool {
    timer_expired && tilt_complete
}

fn is_positive(v: f32) -> bool {
    v > 0.0
}

fn target_xy_length_cd(roll_cd: f32, pitch_cd: f32) -> f32 {
    libm::sqrtf(roll_cd * roll_cd + pitch_cd * pitch_cd)
}

impl QuadPlane {
    /// Upstream `QuadPlane::update_throttle_mix` mix selection.
    ///
    /// Does not write the COP attitude-control object; returns the
    /// `set_throttle_mix_*` call that would be made.
    #[must_use]
    pub fn update_throttle_mix(&self, view: &ThrottleMixView) -> ThrottleMix {
        if !view.allow_update {
            return ThrottleMix::Hold;
        }
        if !view.armed {
            return ThrottleMix::Min;
        }
        if view.vtol_man_throttle {
            if !is_positive(view.throttle_input) && !view.air_mode_active {
                ThrottleMix::Min
            } else {
                ThrottleMix::Man
            }
        } else {
            let large_angle_request =
                target_xy_length_cd(view.roll_target_cd, view.pitch_target_cd)
                    > LAND_CHECK_LARGE_ANGLE_CD;
            let large_angle_error = view.att_error_deg > LAND_CHECK_ANGLE_ERROR_DEG;
            let accel_moving = view.accel_ef_filt_len > LAND_CHECK_ACCEL_MOVING;
            let descent_not_demanded = view.vel_desired_u_ms >= 0.0;
            let mut use_mix_max =
                large_angle_request || large_angle_error || accel_moving || descent_not_demanded;
            if view.in_vtol_land_sequence {
                use_mix_max = !view.in_vtol_land_final;
            }
            if use_mix_max {
                ThrottleMix::Max
            } else {
                ThrottleMix::Min
            }
        }
    }

    /// TIMER tilt-wait gate, [`tilt_fwd_complete`].
    #[must_use]
    pub const fn tilt_fwd_complete(
        tiltrotor_enabled: bool,
        continuous_tilt: bool,
        angle_achieved: bool,
    ) -> bool {
        crate::throttle::tilt_fwd_complete(tiltrotor_enabled, continuous_tilt, angle_achieved)
    }
}
