//! `AC_PrecLand::init` / `update` / `handle_msg` / estimator /
//! output-prediction leftovers, upstream
//! `libraries/AC_PrecLand/AC_PrecLand.cpp`.
//!
//! Tracked as **COP-028**. `Write_Precland` packs the PL packet.
//! The inertial ring is [`crate::InertialHistory`]. Driver `init` and
//! the retry state machine stay in [`crate::leftover`].
//! [`crate::MavlinkBackend`], [`crate::IrlockBackend`], and
//! [`crate::SitlBackend`] are the sensor paths. Both
//! [`PosVelEKF`](crate::PosVelEKF)s run with the Kalman path.
//! `run_output_prediction` writes the lag-compensated output the
//! getters read.

use ap_math::location::Location;
use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::{cd_to_rad, constrain_value, is_zero, sq};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_math::Ftype;

use crate::inertial::InertialHistory;
use crate::state_machine::{RetryAction, RetryStrictness, StateMachineFrontend};
use crate::estimator::{
    EkfInitTimeoutLeftover, EstimatorInput, EstimatorWorld, InertialSample, LosSample,
    RunEstimatorLeftover, ACCEL_NOISE_DEFAULT, EKF_INIT_SENSOR_MIN_UPDATE_MS, EKF_INIT_TIME_MS,
    EKF_INIT_VEL_VAR_NAV_INVALID, EKF_INIT_VEL_VAR_NAV_VALID, EKF_NIS_REJECT_THRESHOLD,
    EKF_OUTLIER_REJECT_LIMIT, LANDING_TARGET_TIMEOUT_MS,
};
use crate::backend::{
    IrlockBackend, IrlockSample, MavlinkBackend, MavlinkHandleMsgLeftover, SitlBackend, SitlSample,
};
use crate::pos_vel_ekf::PosVelEKF;
use crate::prediction::{
    OutputPredictionLeftover, OutputPredictionWorld, LANDING_TARGET_LOST_DIST_THRESH_M,
    LANDING_TARGET_LOST_TIMEOUT_MS, SENSOR_MAX_ALT_M_DEFAULT, SENSOR_MIN_ALT_M_DEFAULT,
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
    /// `PLND_LAND_OFS_X`, centimetres, body-forward of the target.
    pub land_ofs_cm_x: f32,
    /// `PLND_LAND_OFS_Y`, centimetres, body-right of the target.
    pub land_ofs_cm_y: f32,
    /// `PLND_OPTIONS` bitfield.
    pub options: u16,
    /// `PLND_ALT_MIN`, metres. Zero means no floor.
    pub sensor_min_alt_m: f32,
    /// `PLND_ALT_MAX`, metres. Zero means no ceiling.
    pub sensor_max_alt_m: f32,
    /// `PLND_STRICT`.
    pub retry_strictness: RetryStrictness,
    /// `PLND_RET_MAX`.
    pub retry_max: u8,
    /// `PLND_TIMEOUT`, seconds.
    pub retry_timeout_s: f32,
    /// `PLND_RET_BEHAVE`.
    pub retry_behave: RetryAction,
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
            land_ofs_cm_x: 0.0,
            land_ofs_cm_y: 0.0,
            options: OPTION_DISABLED,
            sensor_min_alt_m: SENSOR_MIN_ALT_M_DEFAULT,
            sensor_max_alt_m: SENSOR_MAX_ALT_M_DEFAULT,
            retry_strictness: crate::STRICT_DEFAULT,
            retry_max: crate::RETRY_MAX_DEFAULT,
            retry_timeout_s: crate::RETRY_TIMEOUT_S_DEFAULT,
            retry_behave: crate::RETRY_BEHAVE_DEFAULT,
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
/// The 400 Hz body is a dispatcher. `run_estimator` and
/// `check_target_status` stay later leftovers of this dispatcher.
/// MAVLink `update` (stale-LOS expiry), IRLock / SITL-Gazebo `update`
/// (when an [`IrlockSample`] is supplied), and SITL `update` (when a
/// [`SitlSample`] is supplied) run here. This slice also owns the
/// early-return, the cm→m convert, the `_enabled` gate, and the 25 Hz
/// log cadence.
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
    /// `false` when a frame was supplied and pushed.
    pub need_inertial_push: bool,
    /// Leftover of a backend `update` that did not run here.
    /// `false` for MAVLink, for IRLock / SITL-Gazebo when an
    /// [`IrlockSample`] was supplied, and for SITL when a
    /// [`SitlSample`] was supplied.
    pub need_backend_update: bool,
    /// `true` when a backend `update` ran (`_backend && _enabled`).
    pub backend_updated: bool,
    /// Leftover of `run_estimator` (same gate as backend update).
    pub need_run_estimator: bool,
    /// Leftover of `check_target_status`. Always true when not skipped.
    pub need_check_target_status: bool,
    /// `true` when `now - _last_log_ms > 40`.
    pub need_write_precland: bool,
    /// Packed `log_Precland` when the cadence fired. `None` when the
    /// cadence did not fire or `update` skipped. `WriteBlock` stays a
    /// logger leftover.
    pub write_precland: Option<WritePreclandLeftover>,
}

/// Packed `log_Precland` payload. Upstream `LogStructure.h`.
/// `LOG_PACKET_HEADER_INIT` and `AP::logger().WriteBlock` stay logger leftovers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogPrecland {
    /// `AP_HAL::micros64()` leftover.
    pub time_us: u64,
    /// `healthy()`.
    pub healthy: u8,
    /// `target_acquired()`.
    pub target_acquired: u8,
    /// `get_target_position_relative_NE_m` X. Zero when not acquired.
    pub pos_x: f32,
    /// `get_target_position_relative_NE_m` Y. Zero when not acquired.
    pub pos_y: f32,
    /// `get_target_velocity_relative_NE_ms` X. Zero when not acquired.
    pub vel_x: f32,
    /// `get_target_velocity_relative_NE_ms` Y. Zero when not acquired.
    pub vel_y: f32,
    /// `get_target_position_measurement_NED_m` X.
    pub meas_x: f32,
    /// `get_target_position_measurement_NED_m` Y.
    pub meas_y: f32,
    /// `get_target_position_measurement_NED_m` Z.
    pub meas_z: f32,
    /// `last_backend_los_meas_ms()`.
    pub last_meas: u32,
    /// `ekf_outlier_count()` / `_outlier_reject_count`.
    pub ekf_outcount: u32,
    /// `(uint8_t)_estimator_type`.
    pub estimator: u8,
}

/// What `AC_PrecLand::Write_Precland` packed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WritePreclandLeftover {
    /// `true` when `!enabled()`.
    pub skipped: bool,
    /// Packed payload when not skipped.
    pub packet: Option<LogPrecland>,
    /// Leftover of `AP::logger().WriteBlock(&pkt, sizeof(pkt))`.
    pub need_write_block: bool,
}

/// LANDING_TARGET fields `handle_msg` forwards to the backend.
///
/// Frontend `AC_PrecLand::handle_msg` does not inspect the packet.
/// [`crate::MavlinkBackend::handle_msg`] consumes it when the type is
/// MAVLink. Other backends inherit the empty default.
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
    /// Leftover of a non-MAVLink `_backend->handle_msg`. Always
    /// `false` now: MAVLink overrides it and the base default is empty.
    pub need_backend_handle_msg: bool,
    /// Caller timestamp. Upstream `timestamp_ms`.
    pub timestamp_ms: u32,
    /// Packet forwarded to the backend.
    pub packet: LandingTargetMsg,
    /// MAVLink leftover when the chosen backend is MAVLink.
    pub mavlink: Option<MavlinkHandleMsgLeftover>,
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
    mavlink: MavlinkBackend,
    irlock: IrlockBackend,
    sitl: SitlBackend,
    current_target_state: TargetState,
    inertial_buffer_size: u16,
    inertial_history_ready: bool,
    inertial_history: InertialHistory,
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
    target_pos_rel_out_ne_m: Vector2f,
    target_vel_rel_out_ne_ms: Vector2f,
    last_target_pos_rel_origin_ned_m: Vector3f,
    last_vehicle_pos_ned_m: Vector3f,
    last_veh_velocity_ned_ms: Vector3f,
    last_valid_target_ms: u32,
    land_ofs_cm_x: f32,
    land_ofs_cm_y: f32,
    options: u16,
    sensor_min_alt_m: f32,
    sensor_max_alt_m: f32,
    retry_strictness: RetryStrictness,
    retry_max: u8,
    retry_timeout_s: f32,
    retry_behave: RetryAction,
    ekf_x: PosVelEKF,
    ekf_y: PosVelEKF,
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
            mavlink: MavlinkBackend::new(),
            irlock: IrlockBackend::new(),
            sitl: SitlBackend::new(),
            current_target_state: TargetState::NeverSeen,
            inertial_buffer_size: 0,
            inertial_history_ready: false,
            inertial_history: InertialHistory::default(),
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
            target_pos_rel_out_ne_m: Vector2f::zero(),
            target_vel_rel_out_ne_ms: Vector2f::zero(),
            last_target_pos_rel_origin_ned_m: Vector3f::zero(),
            last_vehicle_pos_ned_m: Vector3f::zero(),
            last_veh_velocity_ned_ms: Vector3f::zero(),
            last_valid_target_ms: 0,
            land_ofs_cm_x: params.land_ofs_cm_x,
            land_ofs_cm_y: params.land_ofs_cm_y,
            options: params.options,
            sensor_min_alt_m: params.sensor_min_alt_m,
            sensor_max_alt_m: params.sensor_max_alt_m,
            retry_strictness: params.retry_strictness,
            retry_max: params.retry_max,
            retry_timeout_s: params.retry_timeout_s,
            retry_behave: params.retry_behave,
            ekf_x: PosVelEKF::new(),
            ekf_y: PosVelEKF::new(),
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
        self.inertial_history = InertialHistory::new(inertial_buffer_size);
        self.inertial_history_ready = self.inertial_history.size() > 0;

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
                self.mavlink.init();
                self.backend_healthy = self.mavlink.healthy();
            }
            Type::Irlock => {
                self.backend = Some(Type::Irlock);
                let drv = self.irlock.init();
                if drv.need_irlock_init {
                    leftover.irlock_bus = Some(self.bus);
                }
            }
            Type::SitlGazebo => {
                self.backend = Some(Type::SitlGazebo);
                let drv = self.irlock.init();
                if drv.need_irlock_init {
                    leftover.irlock_bus = Some(self.bus);
                }
            }
            Type::Sitl => {
                self.backend = Some(Type::Sitl);
                leftover.need_sitl = self.sitl.init().need_sitl;
            }
        }

        leftover.backend = self.backend;
        // `_backend->init()` already applied for MAVLink / IRLock /
        // Gazebo above; the IRLock / SITL / Gazebo *driver* calls are
        // the leftover fields.

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
        self.update_inner(rangefinder_alt_cm, rangefinder_alt_valid, now_ms, None, None, None)
    }

    /// `AC_PrecLand::update` with an IRLock / SITL-Gazebo driver
    /// snapshot. SITL still leaves [`UpdateLeftover::need_backend_update`]
    /// unless [`Self::update_with_sitl`] is used.
    #[must_use]
    pub fn update_with_irlock(
        &mut self,
        rangefinder_alt_cm: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
        sample: IrlockSample,
    ) -> UpdateLeftover {
        self.update_inner(
            rangefinder_alt_cm,
            rangefinder_alt_valid,
            now_ms,
            Some(sample),
            None,
            None,
        )
    }

    /// `AC_PrecLand::update` with a SITL precland-sim snapshot.
    ///
    /// IRLock / SITL-Gazebo still leave
    /// [`UpdateLeftover::need_backend_update`] unless
    /// [`Self::update_with_irlock`] is used.
    #[must_use]
    pub fn update_with_sitl(
        &mut self,
        rangefinder_alt_cm: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
        sample: SitlSample,
    ) -> UpdateLeftover {
        self.update_inner(
            rangefinder_alt_cm,
            rangefinder_alt_valid,
            now_ms,
            None,
            Some(sample),
            None,
        )
    }

    /// `AC_PrecLand::update` with an AHRS inertial snapshot. Pushes the
    /// frame onto the history ring (`push_force`). Without a snapshot
    /// [`UpdateLeftover::need_inertial_push`] stays the AHRS leftover.
    #[must_use]
    pub fn update_with_inertial(
        &mut self,
        rangefinder_alt_cm: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
        frame: InertialSample,
    ) -> UpdateLeftover {
        self.update_inner(
            rangefinder_alt_cm,
            rangefinder_alt_valid,
            now_ms,
            None,
            None,
            Some(frame),
        )
    }

    fn update_inner(
        &mut self,
        rangefinder_alt_cm: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
        irlock: Option<IrlockSample>,
        sitl: Option<SitlSample>,
        inertial: Option<InertialSample>,
    ) -> UpdateLeftover {
        // exit immediately if not enabled
        if self.backend.is_none() || !self.inertial_history_ready {
            return UpdateLeftover {
                skipped: true,
                rangefinder_alt_m: 0.0,
                rangefinder_alt_valid,
                need_inertial_push: false,
                need_backend_update: false,
                backend_updated: false,
                need_run_estimator: false,
                need_check_target_status: false,
                need_write_precland: false,
                write_precland: None,
            };
        }

        if let Some(frame) = inertial {
            self.inertial_history.push_force(frame);
        }
        let need_inertial_push = inertial.is_none();

        let rangefinder_alt_m = rangefinder_alt_cm * 0.01;

        let mut need_backend_update = false;
        let mut backend_updated = false;
        if self.enabled {
            match self.backend {
                Some(Type::Mavlink) => {
                    self.mavlink.update(now_ms);
                    backend_updated = true;
                }
                Some(Type::Irlock | Type::SitlGazebo) => {
                    if let Some(sample) = irlock {
                        self.irlock.update(sample, now_ms);
                        self.backend_healthy = self.irlock.healthy();
                        backend_updated = true;
                    } else {
                        need_backend_update = true;
                    }
                }
                Some(Type::Sitl) => {
                    if let Some(sample) = sitl {
                        self.sitl.update(sample, now_ms, self.orient);
                        self.backend_healthy = self.sitl.healthy();
                        backend_updated = true;
                    } else {
                        need_backend_update = true;
                    }
                }
                Some(Type::None) | None => {}
            }
        }
        let need_write_precland = now_ms.wrapping_sub(self.last_log_ms) > LOG_INTERVAL_MS;
        let write_precland = if need_write_precland {
            self.last_log_ms = now_ms;
            Some(self.write_precland(now_ms, u64::from(now_ms).saturating_mul(1_000)))
        } else {
            None
        };

        UpdateLeftover {
            skipped: false,
            rangefinder_alt_m,
            rangefinder_alt_valid,
            need_inertial_push,
            need_backend_update,
            backend_updated,
            need_run_estimator: self.enabled,
            need_check_target_status: true,
            need_write_precland,
            write_precland,
        }
    }

    /// `AC_PrecLand::handle_msg`. Forwards the packet when a backend
    /// exists. MAVLink consumes it; other backends inherit the empty
    /// default.
    #[must_use]
    pub fn handle_msg(&mut self, packet: LandingTargetMsg, timestamp_ms: u32) -> HandleMsgLeftover {
        if self.backend.is_none() {
            return HandleMsgLeftover {
                skipped: true,
                need_backend_handle_msg: false,
                timestamp_ms,
                packet,
                mavlink: None,
            };
        }
        let mavlink = if self.backend == Some(Type::Mavlink) {
            Some(self.mavlink.handle_msg(packet, timestamp_ms))
        } else {
            // AC_PrecLand_Backend::handle_msg default is empty.
            None
        };
        HandleMsgLeftover {
            skipped: false,
            need_backend_handle_msg: false,
            timestamp_ms,
            packet,
            mavlink,
        }
    }

    /// `AC_PrecLand::run_estimator`.
    ///
    /// RAW_SENSOR writes the relative NE estimate here. Kalman predict /
    /// init / fuse / NIS run on `_ekf_x` / `_ekf_y`. Call
    /// [`Self::run_output_prediction`] when
    /// [`RunEstimatorLeftover::need_output_prediction`] is set.
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

    /// Upstream `_backend->get_los_meas` for MAVLink / IRLock /
    /// SITL-Gazebo / SITL.
    #[must_use]
    pub fn backend_los_meas(&self) -> Option<(Vector3f, VectorFrame)> {
        match self.backend {
            Some(Type::Mavlink) => self.mavlink.get_los_meas(),
            Some(Type::Irlock | Type::SitlGazebo) => self.irlock.get_los_meas(),
            Some(Type::Sitl) => self.sitl.get_los_meas(),
            _ => None,
        }
    }

    /// Snapshot [`crate::estimator::LosSample`] from the active backend.
    #[must_use]
    pub fn backend_los_sample(&self) -> Option<LosSample> {
        match self.backend {
            Some(Type::Mavlink) => self.mavlink.los_sample(),
            Some(Type::Irlock | Type::SitlGazebo) => self.irlock.los_sample(),
            Some(Type::Sitl) => self.sitl.los_sample(),
            _ => None,
        }
    }

    /// Upstream `_backend->distance_to_target()`. `0` when unknown or
    /// when there is no backend.
    #[must_use]
    pub fn distance_to_target(&self) -> f32 {
        match self.backend {
            Some(Type::Mavlink) => self.mavlink.distance_to_target(),
            Some(Type::Irlock | Type::SitlGazebo) => self.irlock.distance_to_target(),
            Some(Type::Sitl) => self.sitl.distance_to_target(),
            _ => 0.0,
        }
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

    /// The inertial history ring. Upstream `_inertial_history`.
    #[must_use]
    pub fn inertial_history(&self) -> &InertialHistory {
        &self.inertial_history
    }

    /// Delayed horizon. Upstream `(*_inertial_history)[0]`.
    #[must_use]
    pub fn inertial_delayed(&self) -> Option<InertialSample> {
        self.inertial_history.delayed()
    }

    /// Newest frame. Upstream `(*_inertial_history)[available()-1]`.
    #[must_use]
    pub fn inertial_newest(&self) -> Option<InertialSample> {
        self.inertial_history.newest()
    }

    /// Walk the ring for `!inertialNavVelocityValid`.
    #[must_use]
    pub fn any_inertial_nav_invalid(&self) -> bool {
        self.inertial_history.any_inertial_nav_invalid()
    }

    /// `AC_PrecLand::Write_Precland`.
    ///
    /// Packs the `log_Precland` payload. `now_ms` is the leftover of
    /// `AP_HAL::millis()` the getters use; `time_us` is the leftover of
    /// `AP_HAL::micros64()`. `AP::logger().WriteBlock` stays a leftover.
    #[must_use]
    pub fn write_precland(&mut self, now_ms: u32, time_us: u64) -> WritePreclandLeftover {
        if !self.enabled() {
            return WritePreclandLeftover {
                skipped: true,
                packet: None,
                need_write_block: false,
            };
        }

        let pos = self
            .get_target_position_relative_ne_m(now_ms)
            .unwrap_or_else(Vector2f::zero);
        let vel = self
            .get_target_velocity_relative_ne_ms(now_ms)
            .unwrap_or_else(Vector2f::zero);
        let meas = self.get_target_position_measurement_ned_m();

        let packet = LogPrecland {
            time_us,
            healthy: u8::from(self.healthy()),
            target_acquired: u8::from(self.target_acquired(now_ms)),
            pos_x: pos.x,
            pos_y: pos.y,
            vel_x: vel.x,
            vel_y: vel.y,
            meas_x: meas.x,
            meas_y: meas.y,
            meas_z: meas.z,
            last_meas: self.last_backend_los_meas_ms(),
            ekf_outcount: self.outlier_reject_count(),
            estimator: self.estimator_type as u8,
        };
        WritePreclandLeftover {
            skipped: false,
            packet: Some(packet),
            need_write_block: true,
        }
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
    /// [`Self::target_acquired`] applies [`LANDING_TARGET_TIMEOUT_MS`].
    #[must_use]
    pub fn estimator_target_acquired(&self) -> bool {
        self.target_acquired
    }

    /// `_ekf_x` after the last Kalman tick.
    #[must_use]
    pub fn ekf_x(&self) -> &PosVelEKF {
        &self.ekf_x
    }

    /// `_ekf_y` after the last Kalman tick.
    #[must_use]
    pub fn ekf_y(&self) -> &PosVelEKF {
        &self.ekf_y
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

    /// Lag-compensated relative position after `run_output_prediction`.
    #[must_use]
    pub fn target_pos_rel_out_ne_m(&self) -> Vector2f {
        self.target_pos_rel_out_ne_m
    }

    /// Lag-compensated relative velocity after `run_output_prediction`.
    #[must_use]
    pub fn target_vel_rel_out_ne_ms(&self) -> Vector2f {
        self.target_vel_rel_out_ne_ms
    }

    /// `_last_valid_target_ms`.
    #[must_use]
    pub fn last_valid_target_ms(&self) -> u32 {
        self.last_valid_target_ms
    }

    /// `_last_veh_velocity_NED_ms`.
    #[must_use]
    pub fn last_veh_velocity_ned_ms(&self) -> Vector3f {
        self.last_veh_velocity_ned_ms
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

    /// `get_retry_strictness()` / `PLND_STRICT`.
    #[must_use]
    pub fn retry_strictness(&self) -> RetryStrictness {
        self.retry_strictness
    }

    /// `get_max_retry_allowed()` / `PLND_RET_MAX`.
    #[must_use]
    pub fn max_retry_allowed(&self) -> u8 {
        self.retry_max
    }

    /// `get_min_retry_time_sec()` / `PLND_TIMEOUT`.
    #[must_use]
    pub fn min_retry_time_sec(&self) -> f32 {
        self.retry_timeout_s
    }

    /// `get_retry_behaviour()` / `PLND_RET_BEHAVE`.
    #[must_use]
    pub fn retry_behaviour(&self) -> RetryAction {
        self.retry_behave
    }

    /// Snapshot for [`crate::StateMachine`]. Leftover of
    /// `AP::ac_precland()` plus the retry getters.
    #[must_use]
    pub fn state_machine_frontend(&self) -> StateMachineFrontend {
        StateMachineFrontend {
            enabled: self.enabled,
            target_state: self.current_target_state,
            retry_strictness: self.retry_strictness,
            last_valid_target_ms: self.last_valid_target_ms,
            min_retry_time_sec: self.retry_timeout_s,
            max_retry_allowed: self.retry_max,
            retry_behaviour: self.retry_behave,
            last_detected_landing_pos_ned_m: self.last_target_pos_rel_origin_ned_m,
            last_vehicle_pos_when_target_detected_ned_m: self.last_vehicle_pos_ned_m,
        }
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

    /// Change `PLND_OPTIONS` the same way a param write would.
    pub fn set_options(&mut self, options: u16) {
        self.options = options;
    }

    /// Change `PLND_LAND_OFS_*` the same way a param write would.
    pub fn set_land_ofs_cm(&mut self, x: f32, y: f32) {
        self.land_ofs_cm_x = x;
        self.land_ofs_cm_y = y;
    }

    /// Change `PLND_ALT_MIN` / `PLND_ALT_MAX` the same way a param write would.
    pub fn set_sensor_alt_limits_m(&mut self, min_m: f32, max_m: f32) {
        self.sensor_min_alt_m = min_m;
        self.sensor_max_alt_m = max_m;
    }

    /// Change `PLND_STRICT` the same way a param write would.
    pub fn set_retry_strictness(&mut self, retry_strictness: RetryStrictness) {
        self.retry_strictness = retry_strictness;
    }

    /// Change `PLND_RET_MAX` the same way a param write would.
    pub fn set_max_retry_allowed(&mut self, retry_max: u8) {
        self.retry_max = retry_max;
    }

    /// Change `PLND_TIMEOUT` the same way a param write would.
    pub fn set_min_retry_time_sec(&mut self, retry_timeout_s: f32) {
        self.retry_timeout_s = retry_timeout_s;
    }

    /// Change `PLND_RET_BEHAVE` the same way a param write would.
    pub fn set_retry_behaviour(&mut self, retry_behave: RetryAction) {
        self.retry_behave = retry_behave;
    }

    /// `AC_PrecLand::run_output_prediction`.
    ///
    /// `later` is leftover of walking `(*_inertial_history)[1..available())`.
    /// Index 0 is the delayed slot `run_estimator` already consumed.
    #[must_use]
    pub fn run_output_prediction(
        &mut self,
        later: &[InertialSample],
        world: &OutputPredictionWorld,
    ) -> OutputPredictionLeftover {
        self.target_pos_rel_out_ne_m = self.target_pos_rel_est_ne_m;
        self.target_vel_rel_out_ne_ms = self.target_vel_rel_est_ne_ms;

        for inertial in later {
            self.target_vel_rel_out_ne_ms.x -= inertial.corrected_vehicle_delta_velocity_ned.x;
            self.target_vel_rel_out_ne_ms.y -= inertial.corrected_vehicle_delta_velocity_ned.y;
            self.target_pos_rel_out_ne_m.x += self.target_vel_rel_out_ne_ms.x * inertial.dt;
            self.target_pos_rel_out_ne_m.y += self.target_vel_rel_out_ne_ms.y * inertial.dt;
        }

        let tbn = world.newest_tbn;
        let imu_pos_ned = tbn * world.imu_pos_offset;
        self.target_pos_rel_out_ne_m.x += imu_pos_ned.x;
        self.target_pos_rel_out_ne_m.y += imu_pos_ned.y;

        let cam_horizontal = Vector3f::new(self.cam_offset_m.x, self.cam_offset_m.y, 0.0);
        let cam_pos_horizontal_ned = tbn * cam_horizontal;
        self.target_pos_rel_out_ne_m.x -= cam_pos_horizontal_ned.x;
        self.target_pos_rel_out_ne_m.y -= cam_pos_horizontal_ned.y;

        let vel_ned_rel_imu = tbn * world.gyro.cross(-world.imu_pos_offset);
        self.target_vel_rel_out_ne_ms.x -= vel_ned_rel_imu.x;
        self.target_vel_rel_out_ne_ms.y -= vel_ned_rel_imu.y;

        let mut leftover = OutputPredictionLeftover::default();
        if let Some(vel) = world.velocity_ned {
            self.last_veh_velocity_ned_ms = vel;
            leftover.stored_vehicle_velocity = true;
        }

        let land_ofs_body = Vector3f::new(self.land_ofs_cm_x, self.land_ofs_cm_y, 0.0) * 0.01;
        let land_ofs_ned = world.rotation_body_to_ned * land_ofs_body;
        self.target_pos_rel_out_ne_m.x += land_ofs_ned.x;
        self.target_pos_rel_out_ne_m.y += land_ofs_ned.y;

        if let Some(pos) = self.get_target_position_m(world.now_ms, world.relative_pos_ne_origin) {
            self.last_target_pos_rel_origin_ned_m.x = pos.x;
            self.last_target_pos_rel_origin_ned_m.y = pos.y;
            leftover.stored_last_target_pos = true;
        }

        self.last_valid_target_ms = world.now_ms;
        leftover
    }

    /// `AC_PrecLand::target_acquired`.
    ///
    /// Applies the [`LANDING_TARGET_TIMEOUT_MS`] side-effect. The GCS
    /// "Target Lost" text is the leftover of
    /// [`Self::refresh_target_acquired`] returning true.
    pub fn target_acquired(&mut self, now_ms: u32) -> bool {
        let _lost = self.refresh_target_acquired(now_ms);
        self.target_acquired
    }

    /// `AC_PrecLand::get_target_position_measurement_NED_m`.
    #[must_use]
    pub fn get_target_position_measurement_ned_m(&self) -> Vector3f {
        self.target_pos_rel_meas_ned_m
    }

    /// `AC_PrecLand::get_target_position_relative_NE_m`.
    pub fn get_target_position_relative_ne_m(&mut self, now_ms: u32) -> Option<Vector2f> {
        if !self.target_acquired(now_ms) {
            return None;
        }
        Some(self.target_pos_rel_out_ne_m)
    }

    /// `AC_PrecLand::get_target_velocity_relative_NE_ms`.
    pub fn get_target_velocity_relative_ne_ms(&mut self, now_ms: u32) -> Option<Vector2f> {
        if !self.target_acquired(now_ms) {
            return None;
        }
        Some(self.target_vel_rel_out_ne_ms)
    }

    /// `AC_PrecLand::get_target_position_m`.
    ///
    /// `curr_pos_ne` is leftover of `AP::ahrs().get_relative_position_NE_origin`.
    pub fn get_target_position_m(
        &mut self,
        now_ms: u32,
        curr_pos_ne: Option<Vector2f>,
    ) -> Option<Vector2f> {
        if !self.target_acquired(now_ms) {
            return None;
        }
        let curr = curr_pos_ne?;
        Some(Vector2f::new(
            self.target_pos_rel_out_ne_m.x + curr.x,
            self.target_pos_rel_out_ne_m.y + curr.y,
        ))
    }

    /// `AC_PrecLand::get_target_velocity_ms`.
    ///
    /// Returns zero when the target is not moving, RAW_SENSOR, or unknown.
    pub fn get_target_velocity_ms(
        &mut self,
        vehicle_velocity_ne_ms: Vector2f,
        now_ms: u32,
    ) -> Vector2f {
        if self.options & OPTION_MOVING_TARGET == 0 {
            return Vector2f::zero();
        }
        if self.estimator_type == EstimatorType::RawSensor {
            return Vector2f::zero();
        }
        match self.get_target_velocity_relative_ne_ms(now_ms) {
            Some(rel) => Vector2f::new(
                rel.x + vehicle_velocity_ne_ms.x,
                rel.y + vehicle_velocity_ne_ms.y,
            ),
            None => Vector2f::zero(),
        }
    }

    /// `AC_PrecLand::get_target_velocity`.
    pub fn get_target_velocity(&mut self, now_ms: u32) -> Option<Vector2f> {
        if self.options & OPTION_MOVING_TARGET == 0 {
            return None;
        }
        if self.estimator_type == EstimatorType::RawSensor {
            return None;
        }
        let rel = self.get_target_velocity_relative_ne_ms(now_ms)?;
        Some(Vector2f::new(
            rel.x + self.last_veh_velocity_ned_ms.x,
            rel.y + self.last_veh_velocity_ned_ms.y,
        ))
    }

    /// `AC_PrecLand::get_target_location`.
    ///
    /// `origin` is leftover of `AP::ahrs().get_origin`. Altitude in the
    /// returned location is not reliable (upstream comment).
    pub fn get_target_location(
        &mut self,
        now_ms: u32,
        origin: Option<Location>,
    ) -> Option<Location> {
        if !self.target_acquired(now_ms) {
            return None;
        }
        let mut loc = origin?;
        loc.offset(
            Ftype::from(self.last_target_pos_rel_origin_ned_m.x),
            Ftype::from(self.last_target_pos_rel_origin_ned_m.y),
        );
        loc.offset_up_m(-self.last_target_pos_rel_origin_ned_m.z);
        Some(loc)
    }

    /// `AC_PrecLand::check_if_sensor_in_range`.
    #[must_use]
    pub fn check_if_sensor_in_range(
        &self,
        rangefinder_alt_m: f32,
        rangefinder_alt_valid: bool,
    ) -> bool {
        if is_zero(self.sensor_max_alt_m) && is_zero(self.sensor_min_alt_m) {
            return true;
        }
        if !rangefinder_alt_valid {
            return false;
        }
        if rangefinder_alt_m > self.sensor_max_alt_m && !is_zero(self.sensor_max_alt_m) {
            return false;
        }
        if rangefinder_alt_m < self.sensor_min_alt_m && !is_zero(self.sensor_min_alt_m) {
            return false;
        }
        true
    }

    /// `AC_PrecLand::check_target_status`.
    ///
    /// `curr_pos_ne` is leftover of `AP::ahrs().get_relative_position_NE_origin`
    /// on the recently-lost path.
    pub fn check_target_status(
        &mut self,
        rangefinder_alt_m: f32,
        rangefinder_alt_valid: bool,
        now_ms: u32,
        curr_pos_ne: Option<Vector2f>,
    ) {
        if self.target_acquired(now_ms) {
            self.current_target_state = TargetState::Found;
            return;
        }

        if self.current_target_state == TargetState::Found
            || self.current_target_state == TargetState::RecentlyLost
        {
            self.current_target_state = TargetState::RecentlyLost;
        } else {
            self.current_target_state = TargetState::NeverSeen;
        }

        if !self.check_if_sensor_in_range(rangefinder_alt_m, rangefinder_alt_valid) {
            self.current_target_state = TargetState::OutOfRange;
            return;
        }

        if self.current_target_state == TargetState::RecentlyLost {
            if let Some(curr) = curr_pos_ne {
                let last_xy = self.last_target_pos_rel_origin_ned_m.xy();
                let last_veh_xy = self.last_vehicle_pos_ned_m.xy();
                let dist_to_last_target = (curr - last_xy).length();
                let dist_to_last_veh = (curr - last_veh_xy).length();
                if now_ms.wrapping_sub(self.last_valid_target_ms) > LANDING_TARGET_LOST_TIMEOUT_MS {
                    self.current_target_state = TargetState::NeverSeen;
                    return;
                }
                if dist_to_last_target > LANDING_TARGET_LOST_DIST_THRESH_M
                    || dist_to_last_veh > LANDING_TARGET_LOST_DIST_THRESH_M
                {
                    self.current_target_state = TargetState::NeverSeen;
                }
            }
        }
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
            self.ekf_x.predict(
                leftover.ekf_predict_dt,
                leftover.ekf_predict_del_vel_ne.x,
                leftover.ekf_predict_accel_noise,
            );
            self.ekf_y.predict(
                leftover.ekf_predict_dt,
                leftover.ekf_predict_del_vel_ne.y,
                leftover.ekf_predict_accel_noise,
            );
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
                let (vel_x, vel_y) = if input.delayed.inertial_nav_velocity_valid {
                    (
                        -input.delayed.inertial_nav_velocity.x,
                        -input.delayed.inertial_nav_velocity.y,
                    )
                } else {
                    (0.0, 0.0)
                };
                self.ekf_x.init(
                    self.target_pos_rel_meas_ned_m.x,
                    leftover.ekf_pos_var,
                    vel_x,
                    leftover.ekf_init_vel_var,
                );
                self.ekf_y.init(
                    self.target_pos_rel_meas_ned_m.y,
                    leftover.ekf_pos_var,
                    vel_y,
                    leftover.ekf_init_vel_var,
                );
                self.last_update_ms = input.now_ms;
                self.estimator_init_ms = input.now_ms;
                self.estimator_initialized = true;
            } else {
                let nis_x = self
                    .ekf_x
                    .pos_nis(self.target_pos_rel_meas_ned_m.x, leftover.ekf_pos_var);
                let nis_y = self
                    .ekf_y
                    .pos_nis(self.target_pos_rel_meas_ned_m.y, leftover.ekf_pos_var);
                leftover.ekf_max_nis = nis_x.max(nis_y);
                if leftover.ekf_max_nis < EKF_NIS_REJECT_THRESHOLD
                    || self.outlier_reject_count >= EKF_OUTLIER_REJECT_LIMIT
                {
                    self.outlier_reject_count = 0;
                    leftover.need_ekf_fuse = true;
                    self.ekf_x
                        .fuse_pos(self.target_pos_rel_meas_ned_m.x, leftover.ekf_pos_var);
                    self.ekf_y
                        .fuse_pos(self.target_pos_rel_meas_ned_m.y, leftover.ekf_pos_var);
                    self.last_update_ms = input.now_ms;
                } else {
                    self.outlier_reject_count += 1;
                    leftover.outlier_rejected = true;
                }
            }
        }

        let timeout = self.check_ekf_init_timeout(input.now_ms);
        leftover.need_gcs_init_failed = timeout.need_gcs_init_failed;
        leftover.need_gcs_init_complete = timeout.need_gcs_init_complete;

        leftover.need_output_prediction = self.target_acquired;
        if self.target_acquired {
            self.target_pos_rel_est_ne_m.x = self.ekf_x.pos();
            self.target_pos_rel_est_ne_m.y = self.ekf_y.pos();
            self.target_vel_rel_est_ne_ms.x = self.ekf_x.vel();
            self.target_vel_rel_est_ne_ms.y = self.ekf_y.vel();
        }
    }
}

impl Default for PrecLand {
    fn default() -> Self {
        Self::new()
    }
}
