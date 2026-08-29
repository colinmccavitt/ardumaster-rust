//! `AC_PrecLand::run_output_prediction` leftovers, upstream
//! `libraries/AC_PrecLand/AC_PrecLand.cpp`.
//!
//! Tracked as **COP-028**. This slice owns the lag-compensate walk from
//! the delayed estimate to the current horizon, the IMU / camera / land
//! offsets, and the getters that read `_target_pos_rel_out_ne_m` /
//! `_target_vel_rel_out_ne_ms`. The inertial ring is
//! [`crate::InertialHistory`]. `Write_Precland` packs the PL packet.
//! Driver `init` and `AC_PrecLand_StateMachine` stay later.

use ap_math::matrix3::Matrix3f;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

/// Last-known location is "lost" after this many ms.
/// Upstream `LANDING_TARGET_LOST_TIMEOUT_MS`.
pub const LANDING_TARGET_LOST_TIMEOUT_MS: u32 = 180_000;
/// Last-known location / vehicle pose farther than this (metres) is
/// treated as never seen. Upstream `LANDING_TARGET_LOST_DIST_THRESH_M`.
pub const LANDING_TARGET_LOST_DIST_THRESH_M: f32 = 30.0;
/// Default `PLND_ALT_MIN`, metres.
pub const SENSOR_MIN_ALT_M_DEFAULT: f32 = 0.75;
/// Default `PLND_ALT_MAX`, metres.
pub const SENSOR_MAX_ALT_M_DEFAULT: f32 = 8.0;

/// AHRS / INS leftovers `run_output_prediction` still needs.
///
/// The ring walk itself is passed as a slice of later
/// [`crate::InertialSample`]s (upstream `(*_inertial_history)[1..]`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutputPredictionWorld {
    /// Leftover of `(*_inertial_history)[available()-1]->Tbn`.
    pub newest_tbn: Matrix3f,
    /// Leftover of `AP::ins().get_imu_pos_offset(...)`.
    pub imu_pos_offset: Vector3f,
    /// Leftover of `AP::ahrs().get_gyro()`.
    pub gyro: Vector3f,
    /// Leftover of `AP::ahrs().get_velocity_NED`. `None` leaves
    /// `_last_veh_velocity_NED_ms` unchanged (`UNUSED_RESULT`).
    pub velocity_ned: Option<Vector3f>,
    /// Leftover of `AP::ahrs().get_rotation_body_to_ned()` for land offset.
    pub rotation_body_to_ned: Matrix3f,
    /// Leftover of `AP::ahrs().get_relative_position_NE_origin` used by
    /// [`crate::PrecLand::get_target_position_m`] at the end of prediction.
    pub relative_pos_ne_origin: Option<Vector2f>,
    /// Leftover of `AP_HAL::millis()` written to `_last_valid_target_ms`.
    pub now_ms: u32,
}

impl Default for OutputPredictionWorld {
    fn default() -> Self {
        Self {
            newest_tbn: Matrix3f::identity(),
            imu_pos_offset: Vector3f::zero(),
            gyro: Vector3f::zero(),
            velocity_ned: None,
            rotation_body_to_ned: Matrix3f::identity(),
            relative_pos_ne_origin: None,
            now_ms: 0,
        }
    }
}

/// What `AC_PrecLand::run_output_prediction` wrote.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutputPredictionLeftover {
    /// `get_target_position_m` succeeded and updated the last-known
    /// origin-relative NE. Upstream always assigns the (possibly
    /// uninitialised) out-arg; this port only writes on success.
    pub stored_last_target_pos: bool,
    /// `_ahrs.get_velocity_NED` returned a value.
    pub stored_vehicle_velocity: bool,
}

impl Default for OutputPredictionLeftover {
    fn default() -> Self {
        Self {
            stored_last_target_pos: false,
            stored_vehicle_velocity: false,
        }
    }
}
