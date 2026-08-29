//! `AC_PrecLand::init`, upstream `libraries/AC_PrecLand/AC_PrecLand.cpp`.
//!
//! Tracked as **COP-028**. Sensor `update`, the estimator, and the retry
//! state machine stay in [`crate::leftover`].

use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::constrain_value;
use ap_math::vector3::Vector3f;

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
}

impl Default for PrecLandParams {
    fn default() -> Self {
        Self {
            enabled: false,
            sensor_type: Type::None,
            lag_s: LAG_S_DEFAULT,
            orient: ORIENT_DEFAULT_COPTER,
            bus: -1,
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
    backend: Option<Type>,
    backend_healthy: bool,
    current_target_state: TargetState,
    inertial_buffer_size: u16,
    inertial_history_ready: bool,
    approach_vector_body: Vector3f,
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
            backend: None,
            backend_healthy: false,
            current_target_state: TargetState::NeverSeen,
            inertial_buffer_size: 0,
            inertial_history_ready: false,
            approach_vector_body: Vector3f::zero(),
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
}

impl Default for PrecLand {
    fn default() -> Self {
        Self::new()
    }
}
