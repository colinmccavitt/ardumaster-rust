//! `AC_PrecLand::init` / `update` / `handle_msg` / estimator leftovers,
//! upstream `libraries/AC_PrecLand/AC_PrecLand.cpp`.
//!
//! Tracked as **COP-028**. Sensor `update`, `PosVelEKF`, output
//! prediction, getters, and the retry state machine stay in
//! [`crate::leftover`].

use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::{cd_to_rad, constrain_value, is_zero, sq};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

use crate::estimator::{
    EkfInitTimeoutLeftover, EstimatorInput, EstimatorWorld, InertialSample, LosSample,
    RunEstimatorLeftover, ACCEL_NOISE_DEFAULT, EKF_INIT_SENSOR_MIN_UPDATE_MS, EKF_INIT_TIME_MS,
    EKF_INIT_VEL_VAR_NAV_INVALID, EKF_INIT_VEL_VAR_NAV_VALID, EKF_NIS_REJECT_THRESHOLD,
    EKF_OUTLIER_REJECT_LIMIT, LANDING_TARGET_TIMEOUT_MS,
};

/// Default `PLND_LAG`, seconds. Upstream `AP_GROUPINFO` default.
pub const LAG_S_DEFAULT: f32 = 0.02;
/// Lower bound `init` writes back onto `PLND_LAG`.
/// Upstream `constrain_float(_lag_s, 0.02f, 0.25f)`.
pub const LAG_S_MIN: f32 = 0.02;
/// Upper bound `init` writes back onto `PLND_LAG`.
/// Comment says 0.250; the constrain call uses 0.25f.
pub const LAG_S_MAX: f32 = 0.25;
/// Copter / Plane default `PLND_ORIENT`. Upstream
/// `AC_PRECLAND_ORIENT_DEFAULT` when not Rover: `ROTATION_PITCH_270`.
pub const ORIENT_DEFAULT_COPTER: Rotation = Rotation::Pitch270;
/// Default `PLND_XY_DIST_MAX`, metres.
pub const XY_MAX_DIST_DESC_M_DEFAULT: f32 = 2.5;
/// `Write_Precland` cadence inside `update`.
/// Upstream `now - _last_log_ms > 40` (25 Hz).
pub const LOG_INTERVAL_MS: u32 = 40;

/// Precision landing sensor type, upstream `AC_PrecLand::Type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Type {
    /// Upstream `Type::NONE`.
    None = 0,
    /// Companion computer. Upstream `Type::MAVLINK`.
    Mavlink = 1,
    /// IR-Lock. Upstream `Type::IRLOCK`.
    Irlock = 2,
    /// Gazebo IR-Lock. Upstream `Type::SITL_GAZEBO`.
    SitlGazebo = 3,
    /// SITL precland sim. Upstream `Type::SITL`.
    Sitl = 4,
}

/// Estimator selection, upstream `AC_PrecLand::EstimatorType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EstimatorType {
    /// Upstream `RAW_SENSOR`.
    RawSensor = 0,
    /// Upstream `KALMAN_FILTER`. Default `PLND_EST_TYPE`.
    KalmanFilter = 1,
}

/// Landing-target sighting, upstream `AC_PrecLand::TargetState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetState {
    /// Upstream `TARGET_NEVER_SEEN`.
    NeverSeen = 0,
    /// Upstream `TARGET_OUT_OF_RANGE`.
    OutOfRange = 1,
    /// Upstream `TARGET_RECENTLY_LOST`.
    RecentlyLost = 2,
    /// Upstream `TARGET_FOUND`.
    Found = 3,
}

/// Frame of a vehicle-to-target unit vector, upstream `VectorFrame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VectorFrame {
    /// Body forward-right-down. Upstream `BODY_FRD`.
    BodyFrd = 0,
    /// Horizontal-plane forward aligned with the vehicle. Upstream `LOCAL_FRD`.
    LocalFrd = 1,
}

/// Upstream `PLND_OPTION_DISABLED`.
pub const OPTION_DISABLED: u16 = 0;
/// Bit 0. Upstream `PLND_OPTION_MOVING_TARGET`.
pub const OPTION_MOVING_TARGET: u16 = 1 << 0;
/// Bit 1. Upstream `PLND_OPTION_PRECLAND_AFTER_REPOSITION`.
pub const OPTION_PRECLAND_AFTER_REPOSITION: u16 = 1 << 1;
/// Bit 2. Upstream `PLND_OPTION_FAST_DESCEND`.
pub const OPTION_FAST_DESCEND: u16 = 1 << 2;

/// Parameters `init` reads. Upstream the `AP_Param` group on `AC_PrecLand`.
#[derive(Debug, Clone, Copy)]
pub struct PrecLandParams {
    /// `PLND_ENABLED`.
    pub enabled: bool,
    /// `PLND_TYPE`.
    pub sensor_type: Type,
    /// `PLND_LAG`, seconds. Constrained on `init`.
    pub lag_s: f32,
    /// `PLND_ORIENT`. Copter default is [`ORIENT_DEFAULT_COPTER`].
    pub orient: Rotation,
    /// `PLND_BUS`. `-1` is the default bus.
    pub bus: i8,
    /// `PLND_EST_TYPE`. Default is Kalman.
    pub estimator_type: EstimatorType,
    /// `PLND_YAW_ALIGN`, centidegrees.
    pub yaw_align_cd: f32,
    /// `PLND_CAM_POS`, metres, camera relative to CG.
    pub cam_offset_m: Vector3f,
    /// `PLND_ACC_P_NSE`.
    pub accel_noise: f32,
}

impl Default for PrecLandParams {
    fn default() -> Self {
        Self {
            enabled: false,
            sensor_type: Type::None,
            lag_s: LAG_S_DEFAULT,
            orient: ORIENT_DEFAULT_COPTER,
            bus: -1,
            estimator_type: EstimatorType::KalmanFilter,
            yaw_align_cd: 0.0,
            cam_offset_m: Vector3f::zero(),
            accel_noise: ACCEL_NOISE_DEFAULT,
        }
    }
}

/// What `AC_PrecLand::init` stored and asked the vehicle for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitLeftover {
    /// `true` when `_backend != nullptr` already, so `init` returned at the top.
    pub skipped: bool,
    /// Chosen backend after this call. `None` for `Type::None`.
    pub backend: Option<Type>,
    /// Inertial ring length, `max(roundf(lag * update_rate_hz), 1)`.
    pub inertial_buffer_size: u16,
    /// IRLock / SITL-Gazebo leftover of `irlock.init(get_bus())`.
    pub irlock_bus: Option<i8>,
    /// SITL leftover of `AP::sitl()`.
    pub need_sitl: bool,
}

/// What `AC_PrecLand::update` ran and what it asked the vehicle for.
///
/// The 400 Hz body is a dispatcher. AHRS history, `_backend->update()`,
/// `run_estimator`, `check_target_status`, and `Write_Precland` stay
/// later leftovers. This slice owns the early-return, the cm→m convert,
/// the `_enabled` gate, and the 25 Hz log cadence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateLeftover {
    /// `true` when `_backend == nullptr || _inertial_history == nullptr`.
    pub skipped: bool,
    /// Altitude after `rangefinder_alt_cm * 0.01`. Zero on skip (the
    /// convert lives after the early return).
    pub rangefinder_alt_m: f32,
    /// Caller `rangefinder_alt_valid`, forwarded to later leftovers.
    pub rangefinder_alt_valid: bool,
    /// Leftover of the AHRS snapshot + `_inertial_history->push_force`.
    pub need_inertial_push: bool,
    /// Leftover of `_backend->update()` when `_backend && _enabled`.
    pub need_backend_update: bool,
    /// Leftover of `run_estimator` (same gate as backend update).
    pub need_run_estimator: bool,
    /// Leftover of `check_target_status`. Always true when not skipped.
    pub need_check_target_status: bool,
    /// Leftover of `Write_Precland` when `now - _last_log_ms > 40`.
    pub need_write_precland: bool,
}

/// LANDING_TARGET fields `handle_msg` forwards to the backend.
///
/// Frontend `AC_PrecLand::handle_msg` does not inspect the packet.
/// [`AC_PrecLand_MAVLink::handle_msg`] is the leftover that will.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingTargetMsg {
    /// `mavlink_landing_target_t.frame`.
    pub frame: u8,
    /// `mavlink_landing_target_t.position_valid`.
    pub position_valid: u8,
    /// `mavlink_landing_target_t.distance`, metres.
    pub distance: f32,
    /// `mavlink_landing_target_t.x`.
    pub x: f32,
    /// `mavlink_landing_target_t.y`.
    pub y: f32,
    /// `mavlink_landing_target_t.z`.
    pub z: f32,
    /// `mavlink_landing_target_t.angle_x`.
    pub angle_x: f32,
    /// `mavlink_landing_target_t.angle_y`.
    pub angle_y: f32,
}

impl Default for LandingTargetMsg {
    fn default() -> Self {
        Self {
            frame: 0,
            position_valid: 0,
            distance: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            angle_x: 0.0,
            angle_y: 0.0,
        }
    }
}

/// What `AC_PrecLand::handle_msg` dispatched.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandleMsgLeftover {
    /// `true` when `_backend == nullptr`.
    pub skipped: bool,
    /// Leftover of `_backend->handle_msg(packet, timestamp_ms)`.
    pub need_backend_handle_msg: bool,
    /// Caller timestamp. Upstream `timestamp_ms`.
    pub timestamp_ms: u32,
    /// Packet forwarded to the backend leftover.
    pub packet: LandingTargetMsg,
}

/// Precision landing frontend, upstream `AC_PrecLand`.
///
/// ADR-0004 forbids the singleton. The vehicle holds this.
#[derive(Debug, Clone)]
pub struct PrecLand {
    enabled: bool,
    sensor_type: Type,
    lag_s: f32,
    orient: Rotation,
    bus: i8,
    estimator_type: EstimatorType,
    yaw_align_cd: f32,
    cam_offset_m: Vector3f,
    accel_noise: f32,
    backend: Option<Type>,
    backend_healthy: bool,
    current_target_state: TargetState,
    inertial_buffer_size: u16,
    inertial_history_ready: bool,
    approach_vector_body: Vector3f,
    last_log_ms: u32,
    target_acquired: bool,
    estimator_initialized: bool,
    estimator_init_ms: u32,
    last_update_ms: u32,
    last_backend_los_meas_ms: u32,
    outlier_reject_count: u32,
    target_pos_rel_meas_ned_m: Vector3f,
    target_pos_rel_est_ne_m: Vector2f,
    target_vel_rel_est_ne_ms: Vector2f,
    last_target_pos_rel_origin_ned_m: Vector3f,
    last_vehicle_pos_ned_m: Vector3f,
}

impl PrecLand {
    /// Construct with defaults. Upstream `AC_PrecLand::AC_PrecLand`
    /// plus `AP_Param::setup_object_defaults`.
    #[must_use]
    pub fn new() -> Self {
        Self::from_params(PrecLandParams::default())
    }

    /// Construct from parameters, without running `init`.
    #[must_use]
    pub fn from_params(params: PrecLandParams) -> Self {
        Self {
            enabled: params.enabled,
            sensor_type: params.sensor_type,
            lag_s: params.lag_s,
            orient: params.orient,
            bus: params.bus,
            estimator_type: params.estimator_type,
            yaw_align_cd: params.yaw_align_cd,
            cam_offset_m: params.cam_offset_m,
            accel_noise: params.accel_noise,
            backend: None,
            backend_healthy: false,
            current_target_state: TargetState::NeverSeen,
            inertial_buffer_size: 0,
            inertial_history_ready: false,
            approach_vector_body: Vector3f::zero(),
            last_log_ms: 0,
            target_acquired: false,
            estimator_initialized: false,
            estimator_init_ms: 0,
            last_update_ms: 0,
            last_backend_los_meas_ms: 0,
            outlier_reject_count: 0,
            target_pos_rel_meas_ned_m: Vector3f::zero(),
            target_pos_rel_est_ne_m: Vector2f::zero(),
            target_vel_rel_est_ne_ms: Vector2f::zero(),
            last_target_pos_rel_origin_ned_m: Vector3f::zero(),
            last_vehicle_pos_ned_m: Vector3f::zero(),
        }
    }

    /// `AC_PrecLand::init`. `update_rate_hz` is the rate `update` will be
    /// called, and sizes the inertial history from `PLND_LAG`.
    #[must_use]
    pub fn init(&mut self, update_rate_hz: u16) -> InitLeftover {
        // exit immediately if init has already been run
        if self.backend.is_some() {
            return InitLeftover {
                skipped: true,
                backend: self.backend,
                inertial_buffer_size: self.inertial_buffer_size,
                irlock_bus: None,
                need_sitl: false,
            };
        }

        self.current_target_state = TargetState::NeverSeen;
        self.backend = None;
        self.backend_healthy = false;

        self.lag_s = constrain_value(self.lag_s, LAG_S_MIN, LAG_S_MAX);

        // Upstream `roundf(_lag_s * update_rate_hz)`.
        let rounded = libm::roundf(self.lag_s * f32::from(update_rate_hz));
        let inertial_buffer_size = (rounded as u16).max(1);
        self.inertial_buffer_size = inertial_buffer_size;
        self.inertial_history_ready = true;

        let mut leftover = InitLeftover {
            skipped: false,
            backend: None,
            inertial_buffer_size,
            irlock_bus: None,
            need_sitl: false,
        };

        match self.sensor_type {
            Type::None => return leftover,
            Type::Mavlink => {
                self.backend = Some(Type::Mavlink);
                // AC_PrecLand_MAVLink::init
                self.backend_healthy = true;
            }
            Type::Irlock => {
                self.backend = Some(Type::Irlock);
                leftover.irlock_bus = Some(self.bus);
            }
            Type::SitlGazebo => {
                self.backend = Some(Type::SitlGazebo);
                leftover.irlock_bus = Some(self.bus);
            }
            Type::Sitl => {
                self.backend = Some(Type::Sitl);
                leftover.need_sitl = true;
            }
        }

        leftover.backend = self.backend;
        // `_backend->init()` already applied for MAVLink above; the
        // IRLock / SITL / Gazebo driver calls are the leftover fields.

        self.approach_vector_body = Vector3f::new(1.0, 0.0, 0.0);
        let _ = rotate(&mut self.approach_vector_body, self.orient);

        leftover
    }

    /// `AC_PrecLand::update`. `rangefinder_alt_cm` is centimetres despite
    /// the header name `rangefinder_alt_m`; `now_ms` is the leftover of
    /// `AP_HAL::millis()`.
    #[must_use]
    pub fn update(
        &mut self,
        rangefinder_alt_cm: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
    ) -> UpdateLeftover {
        // exit immediately if not enabled
        if self.backend.is_none() || !self.inertial_history_ready {
            return UpdateLeftover {
                skipped: true,
                rangefinder_alt_m: 0.0,
                rangefinder_alt_valid,
                need_inertial_push: false,
                need_backend_update: false,
                need_run_estimator: false,
                need_check_target_status: false,
                need_write_precland: false,
            };
        }

        let rangefinder_alt_m = rangefinder_alt_cm * 0.01;

        let need_backend_update = self.enabled;
        let need_write_precland = now_ms.wrapping_sub(self.last_log_ms) > LOG_INTERVAL_MS;
        if need_write_precland {
            self.last_log_ms = now_ms;
        }

        UpdateLeftover {
            skipped: false,
            rangefinder_alt_m,
            rangefinder_alt_valid,
            need_inertial_push: true,
            need_backend_update,
            need_run_estimator: need_backend_update,
            need_check_target_status: true,
            need_write_precland,
        }
    }

    /// `AC_PrecLand::handle_msg`. Forwards the packet when a backend
    /// exists. `AC_PrecLand_MAVLink::handle_msg` stays later.
    #[must_use]
    pub fn handle_msg(&self, packet: LandingTargetMsg, timestamp_ms: u32) -> HandleMsgLeftover {
        if self.backend.is_none() {
            return HandleMsgLeftover {
                skipped: true,
                need_backend_handle_msg: false,
                timestamp_ms,
                packet,
            };
        }
        HandleMsgLeftover {
            skipped: false,
            need_backend_handle_msg: true,
            timestamp_ms,
            packet,
        }
    }

    /// `AC_PrecLand::run_estimator`.
    ///
    /// RAW_SENSOR writes the relative NE estimate here. Kalman predict /
    /// init / fuse are recorded as [`RunEstimatorLeftover`] so
    /// `PosVelEKF` can stay a later leftover.
    #[must_use]
    pub fn run_estimator(&mut self, input: EstimatorInput) -> RunEstimatorLeftover {
        let mut leftover = RunEstimatorLeftover::default();
        leftover.need_gcs_target_lost = self.refresh_target_acquired(input.now_ms);

        match self.estimator_type {
            EstimatorType::RawSensor => self.run_raw_sensor(input, &mut leftover),
            EstimatorType::KalmanFilter => self.run_kalman_filter(input, &mut leftover),
        }
        leftover
    }

    /// `AC_PrecLand::check_ekf_init_timeout`.
    ///
    /// Expects the sensor to update within
    /// [`EKF_INIT_SENSOR_MIN_UPDATE_MS`] until [`EKF_INIT_TIME_MS`] have
    /// passed. After that the vehicle may consume the estimates.
    #[must_use]
    pub fn check_ekf_init_timeout(&mut self, now_ms: u32) -> EkfInitTimeoutLeftover {
        let mut leftover = EkfInitTimeoutLeftover {
            need_gcs_init_failed: false,
            need_gcs_init_complete: false,
        };
        let _lost = self.refresh_target_acquired(now_ms);
        if !self.target_acquired && self.estimator_initialized {
            if now_ms.wrapping_sub(self.last_update_ms) > EKF_INIT_SENSOR_MIN_UPDATE_MS {
                self.estimator_initialized = false;
                leftover.need_gcs_init_failed = true;
            } else if now_ms.wrapping_sub(self.estimator_init_ms) > EKF_INIT_TIME_MS {
                self.target_acquired = true;
                leftover.need_gcs_init_complete = true;
            }
        }
        leftover
    }

    /// `AC_PrecLand::retrieve_los_meas`.
    ///
    /// Returns the (possibly yaw-aligned and orientation-rotated) unit
    /// vector and frame when a *new* backend measurement is available.
    pub fn retrieve_los_meas(&mut self, los: Option<LosSample>) -> Option<(Vector3f, VectorFrame)> {
        let sample = los?;
        if sample.time_ms == self.last_backend_los_meas_ms {
            return None;
        }
        self.last_backend_los_meas_ms = sample.time_ms;

        let mut target_vec_unit = sample.vec_unit;
        if !is_zero(self.yaw_align_cd) {
            target_vec_unit.rotate_xy(cd_to_rad(self.yaw_align_cd));
        }

        // Default construction is downwards in body frame. Pitch270 is
        // that default, so it skips the extra rotations.
        if self.orient != Rotation::Pitch270 {
            let _ = rotate(&mut target_vec_unit, Rotation::Pitch90);
            let _ = rotate(&mut target_vec_unit, self.orient);
        }
        Some((target_vec_unit, sample.frame))
    }

    /// `AC_PrecLand::construct_pos_meas_using_rangefinder`.
    ///
    /// On success writes [`Self::target_pos_rel_meas_ned_m`]. `Tbn`,
    /// IMU offset, and origin position are leftovers of the inertial
    /// ring / INS / AHRS.
    #[must_use]
    pub fn construct_pos_meas_using_rangefinder(
        &mut self,
        rangefinder_alt_m: f32,
        rangefinder_alt_valid: bool,
        delayed: &InertialSample,
        los: Option<LosSample>,
        world: &EstimatorWorld,
    ) -> bool {
        let distance_to_target = los.map(|s| s.distance_to_target_m).unwrap_or(0.0);
        let Some((target_vec_unit, target_vec_frame)) = self.retrieve_los_meas(los) else {
            return false;
        };

        let target_vec_valid = target_vec_unit
            .projected(self.approach_vector_body)
            .dot(self.approach_vector_body)
            > 0.0;

        let target_vec_unit_ned = match target_vec_frame {
            VectorFrame::BodyFrd => delayed.tbn * target_vec_unit,
            VectorFrame::LocalFrd => {
                let (_roll, _pitch, yaw) = delayed.tbn.to_euler();
                let mut ned = target_vec_unit;
                ned.rotate_xy(yaw);
                ned
            }
        };

        let approach_vector_ned = delayed.tbn * self.approach_vector_body;
        let alt_valid =
            (rangefinder_alt_valid && rangefinder_alt_m > 0.0) || distance_to_target > 0.0;
        if !(target_vec_valid && alt_valid) {
            return false;
        }

        let cam_pos_ned = if self.cam_offset_m.is_zero() {
            Vector3f::zero()
        } else {
            delayed.tbn * self.cam_offset_m
        };

        let dist_to_target_m = if distance_to_target > 0.0 {
            distance_to_target
        } else {
            let dist_to_target_along_av_m =
                (rangefinder_alt_m - cam_pos_ned.projected(approach_vector_ned).length()).max(0.0);
            dist_to_target_along_av_m / target_vec_unit_ned.projected(approach_vector_ned).length()
        };

        let accel_pos_ned = delayed.tbn * world.imu_pos_offset;
        let cam_pos_ned_rel_imu = cam_pos_ned - accel_pos_ned;
        self.target_pos_rel_meas_ned_m =
            target_vec_unit_ned * dist_to_target_m + cam_pos_ned_rel_imu;

        if let Some(pos_ned) = world.relative_pos_ned {
            self.last_target_pos_rel_origin_ned_m.z = pos_ned.z;
            self.last_vehicle_pos_ned_m = pos_ned;
        }
        true
    }

    /// Upstream `enabled()`.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Upstream `healthy()`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.backend_healthy
    }

    /// Upstream `get_target_state()`.
    #[must_use]
    pub fn target_state(&self) -> TargetState {
        self.current_target_state
    }

    /// Constrained `PLND_LAG` after `init`.
    #[must_use]
    pub fn lag_s(&self) -> f32 {
        self.lag_s
    }

    /// Chosen backend, or `None` when `PLND_TYPE` is `NONE`.
    #[must_use]
    pub fn backend(&self) -> Option<Type> {
        self.backend
    }

    /// Inertial history length after `init`.
    #[must_use]
    pub fn inertial_buffer_size(&self) -> u16 {
        self.inertial_buffer_size
    }

    /// Whether the inertial ring was allocated. Upstream
    /// `_inertial_history != nullptr`.
    #[must_use]
    pub fn inertial_history_ready(&self) -> bool {
        self.inertial_history_ready
    }

    /// Body-frame approach unit vector after `init`.
    #[must_use]
    pub fn approach_vector_body(&self) -> Vector3f {
        self.approach_vector_body
    }

    /// `PLND_TYPE` this instance was constructed with.
    #[must_use]
    pub fn sensor_type(&self) -> Type {
        self.sensor_type
    }

    /// Last `Write_Precland` tick, leftover of `_last_log_ms`.
    #[must_use]
    pub fn last_log_ms(&self) -> u32 {
        self.last_log_ms
    }

    /// `PLND_EST_TYPE`.
    #[must_use]
    pub fn estimator_type(&self) -> EstimatorType {
        self.estimator_type
    }

    /// `_estimator_initialized` after the last estimator tick.
    #[must_use]
    pub fn estimator_initialized(&self) -> bool {
        self.estimator_initialized
    }

    /// `_target_acquired` flag without the public-getter timeout.
    ///
    /// [`AC_PrecLand::target_acquired`](crate::leftover::REMAINING) stays
    /// a leftover; this is the field `run_estimator` writes.
    #[must_use]
    pub fn estimator_target_acquired(&self) -> bool {
        self.target_acquired
    }

    /// `_last_update_ms` after the last accepted measurement.
    #[must_use]
    pub fn last_update_ms(&self) -> u32 {
        self.last_update_ms
    }

    /// `_last_backend_los_meas_ms`.
    #[must_use]
    pub fn last_backend_los_meas_ms(&self) -> u32 {
        self.last_backend_los_meas_ms
    }

    /// `_outlier_reject_count`.
    #[must_use]
    pub fn outlier_reject_count(&self) -> u32 {
        self.outlier_reject_count
    }

    /// `_target_pos_rel_meas_ned_m` after a successful construct.
    #[must_use]
    pub fn target_pos_rel_meas_ned_m(&self) -> Vector3f {
        self.target_pos_rel_meas_ned_m
    }

    /// RAW_SENSOR estimate, IMU-relative, not lag-compensated.
    #[must_use]
    pub fn target_pos_rel_est_ne_m(&self) -> Vector2f {
        self.target_pos_rel_est_ne_m
    }

    /// RAW_SENSOR relative velocity estimate.
    #[must_use]
    pub fn target_vel_rel_est_ne_ms(&self) -> Vector2f {
        self.target_vel_rel_est_ne_ms
    }

    /// `_last_target_pos_rel_origin_ned_m` down component lives in `.z`.
    #[must_use]
    pub fn last_target_pos_rel_origin_ned_m(&self) -> Vector3f {
        self.last_target_pos_rel_origin_ned_m
    }

    /// Vehicle NED when the last construct stored an AHRS origin.
    #[must_use]
    pub fn last_vehicle_pos_ned_m(&self) -> Vector3f {
        self.last_vehicle_pos_ned_m
    }

    /// Change `PLND_TYPE` the same way a param write would.
    ///
    /// Does not re-run `init`. A `NONE` instance can still `init` after
    /// this because no backend pointer exists yet.
    pub fn set_sensor_type(&mut self, sensor_type: Type) {
        self.sensor_type = sensor_type;
    }

    /// Change `PLND_LAG` the same way a param write would.
    pub fn set_lag_s(&mut self, lag_s: f32) {
        self.lag_s = lag_s;
    }

    /// Change `PLND_ENABLED` the same way a param write would.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Change `PLND_EST_TYPE` the same way a param write would.
    pub fn set_estimator_type(&mut self, estimator_type: EstimatorType) {
        self.estimator_type = estimator_type;
    }

    /// Change `PLND_YAW_ALIGN` the same way a param write would.
    pub fn set_yaw_align_cd(&mut self, yaw_align_cd: f32) {
        self.yaw_align_cd = yaw_align_cd;
    }

    /// Change `PLND_CAM_POS` the same way a param write would.
    pub fn set_cam_offset_m(&mut self, cam_offset_m: Vector3f) {
        self.cam_offset_m = cam_offset_m;
    }

    /// `target_acquired()` side-effect used by the estimator.
    ///
    /// Returns `true` when this call just lost a previously acquired
    /// target (the GCS "Target Lost" leftover).
    fn refresh_target_acquired(&mut self, now_ms: u32) -> bool {
        if now_ms.wrapping_sub(self.last_update_ms) > LANDING_TARGET_TIMEOUT_MS {
            let lost = self.target_acquired;
            self.estimator_initialized = false;
            self.target_acquired = false;
            return lost;
        }
        false
    }

    fn run_raw_sensor(&mut self, input: EstimatorInput, leftover: &mut RunEstimatorLeftover) {
        if input.any_inertial_nav_invalid {
            self.target_acquired = false;
            leftover.raw_sensor_invalid_velocity = true;
            return;
        }

        if self.target_acquired {
            self.target_pos_rel_est_ne_m.x -=
                input.delayed.inertial_nav_velocity.x * input.delayed.dt;
            self.target_pos_rel_est_ne_m.y -=
                input.delayed.inertial_nav_velocity.y * input.delayed.dt;
            self.target_vel_rel_est_ne_ms.x = -input.delayed.inertial_nav_velocity.x;
            self.target_vel_rel_est_ne_ms.y = -input.delayed.inertial_nav_velocity.y;
        }

        leftover.constructed_pos_meas = self.construct_pos_meas_using_rangefinder(
            input.rangefinder_alt_m,
            input.rangefinder_alt_valid,
            &input.delayed,
            input.los,
            &input.world,
        );
        if leftover.constructed_pos_meas {
            if !self.estimator_initialized {
                leftover.need_gcs_target_found = true;
                self.estimator_initialized = true;
            }
            self.target_pos_rel_est_ne_m.x = self.target_pos_rel_meas_ned_m.x;
            self.target_pos_rel_est_ne_m.y = self.target_pos_rel_meas_ned_m.y;
            self.target_vel_rel_est_ne_ms.x = -input.delayed.inertial_nav_velocity.x;
            self.target_vel_rel_est_ne_ms.y = -input.delayed.inertial_nav_velocity.y;
            self.last_update_ms = input.now_ms;
            self.target_acquired = true;
        }

        leftover.need_output_prediction = self.target_acquired;
    }

    fn run_kalman_filter(&mut self, input: EstimatorInput, leftover: &mut RunEstimatorLeftover) {
        if self.target_acquired || self.estimator_initialized {
            leftover.need_ekf_predict = true;
            leftover.ekf_predict_dt = input.delayed.dt;
            leftover.ekf_predict_del_vel_ne = Vector2f::new(
                -input.delayed.corrected_vehicle_delta_velocity_ned.x,
                -input.delayed.corrected_vehicle_delta_velocity_ned.y,
            );
            leftover.ekf_predict_accel_noise = self.accel_noise * input.delayed.dt;
        }

        leftover.constructed_pos_meas = self.construct_pos_meas_using_rangefinder(
            input.rangefinder_alt_m,
            input.rangefinder_alt_valid,
            &input.delayed,
            input.los,
            &input.world,
        );
        if leftover.constructed_pos_meas {
            leftover.ekf_pos_var = sq(self.target_pos_rel_meas_ned_m.z
                * (0.01 + 0.01 * input.world.gyro_length)
                + 0.02);
            if !self.estimator_initialized {
                leftover.need_gcs_target_found = true;
                leftover.need_ekf_init = true;
                leftover.ekf_init_vel_var = if input.delayed.inertial_nav_velocity_valid {
                    EKF_INIT_VEL_VAR_NAV_VALID
                } else {
                    EKF_INIT_VEL_VAR_NAV_INVALID
                };
                self.last_update_ms = input.now_ms;
                self.estimator_init_ms = input.now_ms;
                self.estimator_initialized = true;
            } else if let Some(max_nis) = input.world.max_nis {
                if max_nis < EKF_NIS_REJECT_THRESHOLD
                    || self.outlier_reject_count >= EKF_OUTLIER_REJECT_LIMIT
                {
                    self.outlier_reject_count = 0;
                    leftover.need_ekf_fuse = true;
                    self.last_update_ms = input.now_ms;
                } else {
                    self.outlier_reject_count += 1;
                    leftover.outlier_rejected = true;
                }
            } else {
                leftover.need_ekf_nis = true;
            }
        }

        let timeout = self.check_ekf_init_timeout(input.now_ms);
        leftover.need_gcs_init_failed = timeout.need_gcs_init_failed;
        leftover.need_gcs_init_complete = timeout.need_gcs_init_complete;

        leftover.need_output_prediction = self.target_acquired;
    }
}

impl Default for PrecLand {
    fn default() -> Self {
        Self::new()
    }
}
