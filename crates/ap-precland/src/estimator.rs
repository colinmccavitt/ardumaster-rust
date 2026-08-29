//! `AC_PrecLand::run_estimator` leftovers, upstream
//! `libraries/AC_PrecLand/AC_PrecLand.cpp`.
//!
//! Tracked as **COP-028**. This slice owns the estimator switch,
//! `check_ekf_init_timeout`, `construct_pos_meas_using_rangefinder`, and
//! `retrieve_los_meas`. [`PosVelEKF`](crate::PosVelEKF) predict / init /
//! fuse / NIS run with the Kalman path. `run_output_prediction` is a
//! separate leftover. The inertial ring is [`crate::InertialHistory`].

use ap_math::matrix3::Matrix3f;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

use crate::precland::VectorFrame;

/// EKF must see this many milliseconds of good sensor data before
/// `_target_acquired` is set. Upstream `EKF_INIT_TIME_MS`.
pub const EKF_INIT_TIME_MS: u32 = 2_000;
/// Sensor must update within this many ms during EKF init, else init
/// fails. Upstream `EKF_INIT_SENSOR_MIN_UPDATE_MS`.
pub const EKF_INIT_SENSOR_MIN_UPDATE_MS: u32 = 500;
/// `target_acquired()` timeout. Upstream `LANDING_TARGET_TIMEOUT_MS`.
pub const LANDING_TARGET_TIMEOUT_MS: u32 = 2_000;
/// Default `PLND_ACC_P_NSE`. Upstream `AP_GROUPINFO` default.
pub const ACCEL_NOISE_DEFAULT: f32 = 2.5;
/// NIS gate. Upstream `MAX(NIS_x, NIS_y) < 3.0f`.
pub const EKF_NIS_REJECT_THRESHOLD: f32 = 3.0;
/// Consecutive outliers before the EKF accepts the update anyway.
/// Upstream `_outlier_reject_count >= 3`.
pub const EKF_OUTLIER_REJECT_LIMIT: u32 = 3;
/// Velocity variance when delayed inertial-nav velocity is valid.
/// Upstream `sq(2.0f)`.
pub const EKF_INIT_VEL_VAR_NAV_VALID: f32 = 4.0;
/// Velocity variance when delayed inertial-nav velocity is invalid.
/// Upstream `sq(10.0f)`.
pub const EKF_INIT_VEL_VAR_NAV_INVALID: f32 = 100.0;

/// Delayed inertial snapshot `run_estimator` reads from
/// `(*_inertial_history)[0]`. The ring is [`crate::InertialHistory`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialSample {
    /// `inertial_data_frame_s::Tbn`.
    pub tbn: Matrix3f,
    /// `inertial_data_frame_s::correctedVehicleDeltaVelocityNED`.
    pub corrected_vehicle_delta_velocity_ned: Vector3f,
    /// `inertial_data_frame_s::inertialNavVelocity`.
    pub inertial_nav_velocity: Vector3f,
    /// `inertial_data_frame_s::inertialNavVelocityValid`.
    pub inertial_nav_velocity_valid: bool,
    /// `inertial_data_frame_s::dt`.
    pub dt: f32,
    /// `inertial_data_frame_s::time_usec`.
    pub time_usec: u64,
}

impl Default for InertialSample {
    fn default() -> Self {
        Self {
            tbn: Matrix3f::identity(),
            corrected_vehicle_delta_velocity_ned: Vector3f::zero(),
            inertial_nav_velocity: Vector3f::zero(),
            inertial_nav_velocity_valid: true,
            dt: 0.002_5,
            time_usec: 0,
        }
    }
}

/// Backend LOS snapshot `retrieve_los_meas` reads.
/// MAVLink fills this via [`crate::MavlinkBackend::los_sample`].
/// IRLock / SITL `update` stay later leftovers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LosSample {
    /// `_backend->los_meas_time_ms()`.
    pub time_ms: u32,
    /// Unit vector from `_backend->get_los_meas`.
    pub vec_unit: Vector3f,
    /// Frame from `_backend->get_los_meas`.
    pub frame: VectorFrame,
    /// `_backend->distance_to_target()`, metres. `0` means unknown.
    pub distance_to_target_m: f32,
}

/// Vehicle-world leftovers Kalman init / `construct_pos_meas` still need.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorWorld {
    /// Leftover of `AP::ahrs().get_gyro().length()`.
    pub gyro_length: f32,
    /// Leftover of `AP::ins().get_imu_pos_offset(...)`.
    pub imu_pos_offset: Vector3f,
    /// Leftover of `AP::ahrs().get_relative_position_NED_origin`.
    pub relative_pos_ned: Option<Vector3f>,
}

impl Default for EstimatorWorld {
    fn default() -> Self {
        Self {
            gyro_length: 0.0,
            imu_pos_offset: Vector3f::zero(),
            relative_pos_ned: None,
        }
    }
}

/// Arguments `run_estimator` takes instead of the inertial ring, backend,
/// AHRS, and `PosVelEKF` singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorInput {
    /// Altitude already converted to metres by `update`.
    pub rangefinder_alt_m: f32,
    /// Caller `rangefinder_alt_valid`.
    pub rangefinder_alt_valid: bool,
    /// Leftover of `AP_HAL::millis()`.
    pub now_ms: u32,
    /// Delayed history slot. Upstream `(*_inertial_history)[0]`.
    pub delayed: InertialSample,
    /// Leftover of walking the ring for `!inertialNavVelocityValid`.
    pub any_inertial_nav_invalid: bool,
    /// Current backend LOS, if `get_los_meas` would succeed.
    pub los: Option<LosSample>,
    /// AHRS / INS leftovers still needed for `xy_pos_var` and construct.
    pub world: EstimatorWorld,
}

impl Default for EstimatorInput {
    fn default() -> Self {
        Self {
            rangefinder_alt_m: 0.0,
            rangefinder_alt_valid: false,
            now_ms: 0,
            delayed: InertialSample::default(),
            any_inertial_nav_invalid: false,
            los: None,
            world: EstimatorWorld::default(),
        }
    }
}

/// What `AC_PrecLand::check_ekf_init_timeout` decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EkfInitTimeoutLeftover {
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Init Failed")`.
    pub need_gcs_init_failed: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Init Complete")`.
    pub need_gcs_init_complete: bool,
}

/// What `AC_PrecLand::run_estimator` ran and asked the vehicle for.
///
/// RAW_SENSOR writes `_target_pos_rel_est_ne_m` here. Kalman predict /
/// init / fuse / NIS run on the two [`PosVelEKF`](crate::PosVelEKF)s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunEstimatorLeftover {
    /// `construct_pos_meas_using_rangefinder` returned true.
    pub constructed_pos_meas: bool,
    /// RAW_SENSOR early-return: some history slot had invalid velocity.
    pub raw_sensor_invalid_velocity: bool,
    /// `run_output_prediction` should run this tick (`target_acquired`).
    pub need_output_prediction: bool,
    /// `_ekf_x/_ekf_y.predict` ran this tick.
    pub need_ekf_predict: bool,
    /// `_ekf_x/_ekf_y.init` ran on first measurement.
    pub need_ekf_init: bool,
    /// `_ekf_x/_ekf_y.fusePos` ran this tick.
    pub need_ekf_fuse: bool,
    /// `MAX(NIS_x, NIS_y)` when a fuse-or-reject decision ran.
    pub ekf_max_nis: f32,
    /// `max_nis` was at or above the gate and the reject counter was below 3.
    pub outlier_rejected: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Target Found")`.
    pub need_gcs_target_found: bool,
    /// Leftover of the `target_acquired()` timeout text.
    pub need_gcs_target_lost: bool,
    /// Forwarded from `check_ekf_init_timeout`.
    pub need_gcs_init_failed: bool,
    /// Forwarded from `check_ekf_init_timeout`.
    pub need_gcs_init_complete: bool,
    /// `-vehicleDelVel` passed to `PosVelEKF::predict`. Zero when not predicting.
    pub ekf_predict_del_vel_ne: Vector2f,
    /// `dt` passed to `PosVelEKF::predict`.
    pub ekf_predict_dt: f32,
    /// `_accel_noise * dt` passed to `PosVelEKF::predict`.
    pub ekf_predict_accel_noise: f32,
    /// `xy_pos_var` that would be passed to `PosVelEKF::init` / `fusePos`.
    pub ekf_pos_var: f32,
    /// Velocity variance that would be passed to `PosVelEKF::init`.
    pub ekf_init_vel_var: f32,
}

impl Default for RunEstimatorLeftover {
    fn default() -> Self {
        Self {
            constructed_pos_meas: false,
            raw_sensor_invalid_velocity: false,
            need_output_prediction: false,
            need_ekf_predict: false,
            need_ekf_init: false,
            need_ekf_fuse: false,
            ekf_max_nis: 0.0,
            outlier_rejected: false,
            need_gcs_target_found: false,
            need_gcs_target_lost: false,
            need_gcs_init_failed: false,
            need_gcs_init_complete: false,
            ekf_predict_del_vel_ne: Vector2f::zero(),
            ekf_predict_dt: 0.0,
            ekf_predict_accel_noise: 0.0,
            ekf_pos_var: 0.0,
            ekf_init_vel_var: 0.0,
        }
    }
}
