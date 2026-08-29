//! AutoTune PosHold lean leftover, upstream `AC_AutoTune::get_poshold_attitude_rad`.
//!
//! Tracked as **COP-027**. When the pilot leaves roll / pitch centered,
//! AutoTune optionally leans back toward the start position so a Loiter
//! or PosHold from-mode does not drift while twitching. The Copter
//! wrapper already catalogs the call and the `have_position` latch.
//! This leftover is the lean math: 10° at 20 m, a 10 cm deadzone,
//! body-frame rotation, and the 5 m yaw-across-the-wind turn.
//!
//! GCS leftover lives in [`crate::autotune_gcs`]. Logging leftover lives
//! in [`crate::autotune_log`]. Heli is out of scope.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::AxisType;
use ap_math::scalar::{constrain_value, radians, wrap_pi};
use ap_math::vector2::Vector2f;

/// Don't go past 10 degrees — autotune result would deteriorate too much.
pub const AUTOTUNE_POSHOLD_ANGLE_MAX_DEG: f32 = 10.0;

/// Hit the 10 degree limit at 20 meters position error.
pub const AUTOTUNE_POSHOLD_DIST_LIMIT_M: f32 = 20.0;

/// Yaw only starts at 5 m from the start (2.5° lean at the 10° / 20 m scale).
pub const AUTOTUNE_POSHOLD_YAW_DIST_LIMIT_M: f32 = 5.0;

/// Don't do anything within 10 cm.
pub const AUTOTUNE_POSHOLD_DEADZONE_M: f32 = 0.10;

/// 5 degree slop past 90 so the nearest 180 mark does not oscillate.
pub const AUTOTUNE_POSHOLD_YAW_SLOP_DEG: f32 = 95.0;

/// What `get_poshold_attitude_rad` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHoldAttitudeView {
    /// `use_poshold` from `init_internals`.
    pub use_poshold: bool,
    /// `position_ok()` — Copter `copter.position_ok()`.
    pub position_ok: bool,
    /// `have_position` before this call.
    pub have_position: bool,
    /// Latched `start_position_ned_m.x` (north), metres.
    pub start_n_m: f32,
    /// Latched `start_position_ned_m.y` (east), metres.
    pub start_e_m: f32,
    /// `pos_control->get_pos_estimate_NED_m().x` (north), metres.
    pub pos_n_m: f32,
    /// `pos_control->get_pos_estimate_NED_m().y` (east), metres.
    pub pos_e_m: f32,
    /// `ahrs_view->cos_yaw()`.
    pub cos_yaw: f32,
    /// `ahrs_view->sin_yaw()`.
    pub sin_yaw: f32,
    /// Current held yaw target, rad. Overwritten past the 5 m gate.
    pub desired_yaw_rad: f32,
    /// Current Multi axis — pitch points across the wind.
    pub axis: AxisType,
}

impl PosHoldAttitudeView {
    /// Centered, position-ok, already latched at the origin, facing north.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            use_poshold: true,
            position_ok: true,
            have_position: true,
            start_n_m: 0.0,
            start_e_m: 0.0,
            pos_n_m: 0.0,
            pos_e_m: 0.0,
            cos_yaw: 1.0,
            sin_yaw: 0.0,
            desired_yaw_rad: 0.0,
            axis: AxisType::Roll,
        }
    }
}

/// Leftover of one `AC_AutoTune::get_poshold_attitude_rad`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosHoldAttitude {
    /// Body-frame roll lean, rad.
    pub roll_out_rad: f32,
    /// Body-frame pitch lean, rad.
    pub pitch_out_rad: f32,
    /// Held yaw after the optional 5 m turn, rad.
    pub yaw_out_rad: f32,
    /// `have_position` after this call.
    pub have_position: bool,
    /// First tick that captured `start_position_ned_m`.
    pub latched_start: bool,
    /// Horizontal error cleared the 10 cm deadzone.
    pub applied: bool,
    /// Start north after this call (current estimate when latching).
    pub start_n_m: f32,
    /// Start east after this call (current estimate when latching).
    pub start_e_m: f32,
}

/// `radians(10)` — the lean cap.
#[must_use]
pub fn poshold_angle_max_rad() -> f32 {
    radians(AUTOTUNE_POSHOLD_ANGLE_MAX_DEG)
}

/// Upstream `AC_AutoTune::get_poshold_attitude_rad`.
///
/// Roll and pitch start at zero. Disabled / no-fix returns immediately.
/// The first good fix latches `start_position_ned_m` to the current
/// estimate (so this tick is always inside the deadzone). Past 10 cm a
/// linear controller hits 10° at 20 m, rotates NED error into body
/// frame, and past 5 m yaws along (or, on pitch, across) the wind.
#[must_use]
pub fn get_poshold_attitude_rad(view: &PosHoldAttitudeView) -> PosHoldAttitude {
    let mut out = PosHoldAttitude {
        roll_out_rad: 0.0,
        pitch_out_rad: 0.0,
        yaw_out_rad: view.desired_yaw_rad,
        have_position: view.have_position,
        latched_start: false,
        applied: false,
        start_n_m: view.start_n_m,
        start_e_m: view.start_e_m,
    };

    if !view.use_poshold || !view.position_ok {
        return out;
    }

    if !out.have_position {
        out.have_position = true;
        out.latched_start = true;
        out.start_n_m = view.pos_n_m;
        out.start_e_m = view.pos_e_m;
    }

    let error_ne = Vector2f::new(view.pos_n_m - out.start_n_m, view.pos_e_m - out.start_e_m);
    let dist_m = error_ne.length();
    if dist_m < AUTOTUNE_POSHOLD_DEADZONE_M {
        return out;
    }

    let angle_max_rad = poshold_angle_max_rad();
    let scaling = constrain_value(
        angle_max_rad * dist_m / AUTOTUNE_POSHOLD_DIST_LIMIT_M,
        0.0,
        angle_max_rad,
    );
    let angle_ne = error_ne * (scaling / dist_m);

    out.pitch_out_rad = angle_ne.x * view.cos_yaw + angle_ne.y * view.sin_yaw;
    out.roll_out_rad = angle_ne.x * view.sin_yaw - angle_ne.y * view.cos_yaw;
    out.applied = true;

    if dist_m < AUTOTUNE_POSHOLD_YAW_DIST_LIMIT_M {
        return out;
    }

    let mut target_yaw_rad = libm::atan2f(error_ne.y, error_ne.x);
    if view.axis == AxisType::Pitch {
        target_yaw_rad += radians(90.0);
    }
    if wrap_pi(out.yaw_out_rad - target_yaw_rad).abs() > radians(AUTOTUNE_POSHOLD_YAW_SLOP_DEG) {
        target_yaw_rad += radians(180.0);
    }
    out.yaw_out_rad = target_yaw_rad;
    out
}
