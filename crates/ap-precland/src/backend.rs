//! `AC_PrecLand_Backend` + `AC_PrecLand_MAVLink` + IRLock leftovers +
//! SITL leftover, upstream `libraries/AC_PrecLand/AC_PrecLand_Backend.h`,
//! `AC_PrecLand_MAVLink.{h,cpp}`, `AC_PrecLand_IRLock.{h,cpp}`,
//! `AC_PrecLand_SITL_Gazebo.{h,cpp}`, and `AC_PrecLand_SITL.{h,cpp}`.
//!
//! Tracked as **COP-028**. This slice owns the shared LOS getters,
//! the companion-computer `LANDING_TARGET` path, the IRLock /
//! SITL-Gazebo `update` body, and the SITL sim `update` body. IRLock
//! and SITL-Gazebo share one algorithm; ADR-0004 forbids `AP_IRLock`,
//! so the vehicle injects an [`IrlockSample`]. SITL talks to
//! `AP::sitl()`; ADR-0004 forbids that singleton, so the vehicle
//! injects a [`SitlSample`]. `AC_PrecLand_Backend::handle_msg` is the
//! empty default; MAVLink overrides it.

use ap_math::matrix3::Matrix3f;
use ap_math::rotations_gen::{rotate_inverse, Rotation};
use ap_math::vector3::Vector3f;

use crate::estimator::LosSample;
use crate::precland::{LandingTargetMsg, VectorFrame};

/// Stale-LOS timeout used by every backend `update`.
/// Upstream `AP_HAL::millis() - _los_meas.time_ms <= 1000`.
pub const LOS_MEAS_TIMEOUT_MS: u32 = 1_000;

/// `MAV_FRAME_BODY_FRD`. Upstream `common.h`.
pub const MAV_FRAME_BODY_FRD: u8 = 12;
/// `MAV_FRAME_LOCAL_FRD`. Upstream `common.h`.
pub const MAV_FRAME_LOCAL_FRD: u8 = 20;

/// Shared backend LOS + distance state, upstream `AC_PrecLand_Backend`.
///
/// The C++ class is abstract (`init` / `update` pure virtual). The
/// getters and the empty `handle_msg` live here. MAVLink owns the
/// first concrete `update` / `handle_msg`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Backend {
    los_meas: LosMeas,
    distance_to_target: f32,
}

/// `_los_meas` on `AC_PrecLand_Backend`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LosMeas {
    vec_unit: Vector3f,
    frame: VectorFrame,
    time_ms: u32,
    valid: bool,
}

impl Default for LosMeas {
    fn default() -> Self {
        Self {
            vec_unit: Vector3f::zero(),
            frame: VectorFrame::BodyFrd,
            time_ms: 0,
            valid: false,
        }
    }
}

impl Default for Backend {
    fn default() -> Self {
        Self {
            los_meas: LosMeas::default(),
            distance_to_target: 0.0,
        }
    }
}

impl Backend {
    /// Construct an empty backend. Upstream constructor stores frontend
    /// / state references; those live on [`crate::PrecLand`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `get_los_meas`. Returns the unit vector and frame when
    /// `_los_meas.valid`.
    #[must_use]
    pub fn get_los_meas(&self) -> Option<(Vector3f, VectorFrame)> {
        if !self.los_meas.valid {
            return None;
        }
        Some((self.los_meas.vec_unit, self.los_meas.frame))
    }

    /// Upstream `los_meas_time_ms()`.
    #[must_use]
    pub fn los_meas_time_ms(&self) -> u32 {
        self.los_meas.time_ms
    }

    /// Upstream `distance_to_target()`. Metres. `0` means unknown.
    #[must_use]
    pub fn distance_to_target(&self) -> f32 {
        self.distance_to_target
    }

    /// Snapshot [`crate::estimator::LosSample`] for `retrieve_los_meas`.
    ///
    /// `None` when `_los_meas.valid` is false (same gate as
    /// `get_los_meas`).
    #[must_use]
    pub fn los_sample(&self) -> Option<LosSample> {
        let (vec_unit, frame) = self.get_los_meas()?;
        Some(LosSample {
            time_ms: self.los_meas.time_ms,
            vec_unit,
            frame,
            distance_to_target_m: self.distance_to_target,
        })
    }

    /// Upstream `AC_PrecLand_Backend::handle_msg`. Empty default.
    pub fn handle_msg(&mut self, _packet: LandingTargetMsg, _timestamp_ms: u32) {}

    /// Expire a stale LOS the way every backend `update` ends.
    ///
    /// Upstream `_los_meas.valid = _los_meas.valid && now - time_ms <= 1000`.
    pub fn expire_stale_los(&mut self, now_ms: u32) {
        self.los_meas.valid =
            self.los_meas.valid && now_ms.wrapping_sub(self.los_meas.time_ms) <= LOS_MEAS_TIMEOUT_MS;
    }
}

/// What `AC_PrecLand_MAVLink::handle_msg` did with one packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MavlinkHandleMsgLeftover {
    /// `true` when the packet wrote `_los_meas`.
    pub accepted: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "Plnd: Frame not supported")`.
    /// Set only the first time an unsupported frame arrives.
    pub need_gcs_wrong_frame: bool,
    /// `position_valid == 1` but `distance` was not positive, so the
    /// packet was dropped. Upstream early `return`.
    pub rejected_non_positive_distance: bool,
}

impl Default for MavlinkHandleMsgLeftover {
    fn default() -> Self {
        Self {
            accepted: false,
            need_gcs_wrong_frame: false,
            rejected_non_positive_distance: false,
        }
    }
}

/// Companion-computer backend, upstream `AC_PrecLand_MAVLink`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MavlinkBackend {
    inner: Backend,
    wrong_frame_msg_sent: bool,
    healthy: bool,
}

impl Default for MavlinkBackend {
    fn default() -> Self {
        Self {
            inner: Backend::new(),
            wrong_frame_msg_sent: false,
            healthy: false,
        }
    }
}

impl MavlinkBackend {
    /// Construct. Upstream `using AC_PrecLand_Backend::AC_PrecLand_Backend`.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `AC_PrecLand_MAVLink::init`. Sets healthy.
    pub fn init(&mut self) {
        self.healthy = true;
    }

    /// Upstream `_state.healthy` after `init`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.healthy
    }

    /// Upstream `AC_PrecLand_MAVLink::update`.
    ///
    /// Drops `_los_meas.valid` once the last accepted packet is older
    /// than [`LOS_MEAS_TIMEOUT_MS`].
    pub fn update(&mut self, now_ms: u32) {
        self.inner.expire_stale_los(now_ms);
    }

    /// Upstream `AC_PrecLand_MAVLink::handle_msg`.
    ///
    /// Supported frames are [`MAV_FRAME_BODY_FRD`] and
    /// [`MAV_FRAME_LOCAL_FRD`]. `position_valid == 1` takes the
    /// `(x, y, z) / distance` path; otherwise the angle path
    /// `(-tan(angle_y), tan(angle_x), 1)` is normalised.
    pub fn handle_msg(
        &mut self,
        packet: LandingTargetMsg,
        timestamp_ms: u32,
    ) -> MavlinkHandleMsgLeftover {
        let mut leftover = MavlinkHandleMsgLeftover::default();

        if packet.frame != MAV_FRAME_BODY_FRD && packet.frame != MAV_FRAME_LOCAL_FRD {
            if !self.wrong_frame_msg_sent {
                self.wrong_frame_msg_sent = true;
                leftover.need_gcs_wrong_frame = true;
            }
            return leftover;
        }

        let frame = if packet.frame == MAV_FRAME_BODY_FRD {
            VectorFrame::BodyFrd
        } else {
            VectorFrame::LocalFrd
        };

        if packet.position_valid == 1 {
            if packet.distance > 0.0 {
                self.inner.los_meas.vec_unit = Vector3f::new(packet.x, packet.y, packet.z);
                self.inner.los_meas.vec_unit /= packet.distance;
                self.inner.los_meas.frame = frame;
            } else {
                leftover.rejected_non_positive_distance = true;
                return leftover;
            }
        } else {
            // compute unit vector towards target
            // Upstream: Vector3f{-tanf(angle_y), tanf(angle_x), 1.0f}
            // then `/= length()` unguarded.
            let mut vec = Vector3f::new(
                -libm::tanf(packet.angle_y),
                libm::tanf(packet.angle_x),
                1.0,
            );
            let length = vec.length();
            vec /= length;
            self.inner.los_meas.vec_unit = vec;
            self.inner.los_meas.frame = frame;
        }

        self.inner.distance_to_target = packet.distance.max(0.0);
        self.inner.los_meas.time_ms = timestamp_ms;
        self.inner.los_meas.valid = true;
        leftover.accepted = true;
        leftover
    }

    /// Upstream `get_los_meas`.
    #[must_use]
    pub fn get_los_meas(&self) -> Option<(Vector3f, VectorFrame)> {
        self.inner.get_los_meas()
    }

    /// Upstream `los_meas_time_ms()`.
    #[must_use]
    pub fn los_meas_time_ms(&self) -> u32 {
        self.inner.los_meas_time_ms()
    }

    /// Upstream `distance_to_target()`.
    #[must_use]
    pub fn distance_to_target(&self) -> f32 {
        self.inner.distance_to_target()
    }

    /// Snapshot for [`crate::PrecLand::retrieve_los_meas`].
    #[must_use]
    pub fn los_sample(&self) -> Option<LosSample> {
        self.inner.los_sample()
    }

    /// Whether the GCS "Frame not supported" text has already been asked.
    #[must_use]
    pub fn wrong_frame_msg_sent(&self) -> bool {
        self.wrong_frame_msg_sent
    }
}

/// Snapshot of `AP_IRLock` state the vehicle feeds into
/// [`IrlockBackend::update`].
///
/// ADR-0004 forbids `AP_IRLock` / I2C / the SITL-Gazebo socket. This is
/// the leftover of `irlock.update()` plus the getters
/// `healthy()`, `num_targets()`, `last_update_ms()`, and
/// `get_unit_vector_body()`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IrlockSample {
    /// `AP_IRLock::healthy()`.
    pub healthy: bool,
    /// `AP_IRLock::last_update_ms()`.
    pub last_update_ms: u32,
    /// `_target_info.pos_x` — tan(theta) right of image centre.
    pub pos_x: f32,
    /// `_target_info.pos_y` — tan(theta) down of image centre.
    pub pos_y: f32,
    /// `_target_info.pos_z`.
    pub pos_z: f32,
}

impl IrlockSample {
    /// Upstream `AP_IRLock::num_targets`. `1` when healthy, else `0`.
    #[must_use]
    pub fn num_targets(&self) -> usize {
        if self.healthy {
            1
        } else {
            0
        }
    }

    /// Upstream `AP_IRLock::get_unit_vector_body`.
    ///
    /// `None` when not healthy. The divide by length is unguarded
    /// (a zero-length target vector becomes NaN), matching upstream.
    #[must_use]
    pub fn unit_vector_body(&self) -> Option<Vector3f> {
        if !self.healthy {
            return None;
        }
        let mut ret = Vector3f::new(-self.pos_y, self.pos_x, self.pos_z);
        ret /= ret.length();
        Some(ret)
    }
}

/// IR-Lock / SITL-Gazebo backend, upstream `AC_PrecLand_IRLock` and
/// `AC_PrecLand_SITL_Gazebo`.
///
/// Both classes share the same `update` body. `init` only calls
/// `irlock.init(get_bus())`, which stays a leftover on
/// [`crate::InitLeftover::irlock_bus`]. Healthy stays false until
/// the first driver snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IrlockBackend {
    inner: Backend,
    healthy: bool,
}

impl Default for IrlockBackend {
    fn default() -> Self {
        Self {
            inner: Backend::new(),
            healthy: false,
        }
    }
}

impl IrlockBackend {
    /// Construct. Upstream `using AC_PrecLand_Backend::AC_PrecLand_Backend`.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `AC_PrecLand_IRLock::init` /
    /// `AC_PrecLand_SITL_Gazebo::init`.
    ///
    /// Does not set healthy. The `irlock.init(get_bus())` driver call
    /// is the leftover on [`IrlockInitLeftover`].
    pub fn init(&mut self) -> IrlockInitLeftover {
        IrlockInitLeftover {
            need_irlock_init: true,
        }
    }

    /// Upstream `_state.healthy` after `update`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.healthy
    }

    /// Upstream `AC_PrecLand_IRLock::update` /
    /// `AC_PrecLand_SITL_Gazebo::update`.
    ///
    /// Writes `_los_meas` when `num_targets() > 0` and
    /// `last_update_ms` is new, then expires a stale LOS the same
    /// way every backend `update` ends.
    pub fn update(&mut self, sample: IrlockSample, now_ms: u32) {
        self.healthy = sample.healthy;

        if sample.num_targets() > 0 && sample.last_update_ms != self.inner.los_meas.time_ms {
            if let Some(vec) = sample.unit_vector_body() {
                self.inner.los_meas.vec_unit = vec;
                self.inner.los_meas.frame = VectorFrame::BodyFrd;
                self.inner.los_meas.valid = true;
                self.inner.los_meas.time_ms = sample.last_update_ms;
            }
        }
        self.inner.expire_stale_los(now_ms);
    }

    /// Upstream `get_los_meas`.
    #[must_use]
    pub fn get_los_meas(&self) -> Option<(Vector3f, VectorFrame)> {
        self.inner.get_los_meas()
    }

    /// Upstream `los_meas_time_ms()`.
    #[must_use]
    pub fn los_meas_time_ms(&self) -> u32 {
        self.inner.los_meas_time_ms()
    }

    /// Upstream `distance_to_target()`. IRLock never writes this;
    /// stays `0` (unknown).
    #[must_use]
    pub fn distance_to_target(&self) -> f32 {
        self.inner.distance_to_target()
    }

    /// Snapshot for [`crate::PrecLand::retrieve_los_meas`].
    #[must_use]
    pub fn los_sample(&self) -> Option<LosSample> {
        self.inner.los_sample()
    }
}

/// Snapshot of `SITL::SIM_Precland` + AHRS the vehicle feeds into
/// [`SitlBackend::update`].
///
/// ADR-0004 forbids `AP::sitl()` and the SITL singleton. This is the
/// leftover of `precland_sim.healthy()`, `last_update_ms()`,
/// `get_target_position()`, `option_enabled(ENABLE_TARGET_DISTANCE)`,
/// and `AP::ahrs().get_rotation_body_to_ned()`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlSample {
    /// `SIM_Precland::healthy()`.
    pub healthy: bool,
    /// `SIM_Precland::last_update_ms()`.
    pub last_update_ms: u32,
    /// `SIM_Precland::get_target_position()`. NED metres.
    ///
    /// Upstream is `Vector3d`; the backend immediately `tofloat()`s
    /// after `mul_transpose`, so this leftover stores [`Vector3f`].
    pub target_position: Vector3f,
    /// `SIM_Precland::option_enabled(Option::ENABLE_TARGET_DISTANCE)`.
    pub enable_target_distance: bool,
    /// `AP::ahrs().get_rotation_body_to_ned()`.
    ///
    /// Upstream `todouble()` then `mul_transpose`; leftover is
    /// [`Matrix3f`].
    pub body_to_ned: Matrix3f,
}

/// SITL precland-sim backend, upstream `AC_PrecLand_SITL`.
///
/// `init` stores `AP::sitl()`, which stays a leftover on
/// [`crate::InitLeftover::need_sitl`]. Healthy stays false until
/// the first sim snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlBackend {
    inner: Backend,
    healthy: bool,
}

impl Default for SitlBackend {
    fn default() -> Self {
        Self {
            inner: Backend::new(),
            healthy: false,
        }
    }
}

/// Leftover of `AC_PrecLand_IRLock::init` / `SITL_Gazebo::init`.
///
/// ADR-0004 forbids `AP_IRLock`. The vehicle owns `irlock.init(bus)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrlockInitLeftover {
    /// `true` when `irlock.init(get_bus())` still needs to run.
    pub need_irlock_init: bool,
}

/// Leftover of `AC_PrecLand_SITL::init`.
///
/// ADR-0004 forbids `AP::sitl()`. The vehicle owns the singleton fetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SitlInitLeftover {
    /// `true` when `AP::sitl()` still needs to run.
    pub need_sitl: bool,
}

impl SitlBackend {
    /// Construct. Upstream `using AC_PrecLand_Backend::AC_PrecLand_Backend`.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `AC_PrecLand_SITL::init`.
    ///
    /// Does not set healthy. The `AP::sitl()` singleton fetch is the
    /// leftover on [`SitlInitLeftover`].
    pub fn init(&mut self) -> SitlInitLeftover {
        SitlInitLeftover { need_sitl: true }
    }

    /// Upstream `_state.healthy` after `update`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.healthy
    }

    /// Upstream `AC_PrecLand_SITL::update`.
    ///
    /// Writes `_los_meas` when the sim is healthy and
    /// `last_update_ms` is new. Unlike IRLock, a repeated timestamp
    /// or an unhealthy snapshot *clears* `_los_meas.valid` before
    /// the shared stale-LOS expiry.
    pub fn update(&mut self, sample: SitlSample, now_ms: u32, orient: Rotation) {
        self.healthy = sample.healthy;

        if sample.healthy && sample.last_update_ms != self.inner.los_meas.time_ms {
            let mut vec = sample.body_to_ned.mul_transpose(-sample.target_position);
            self.inner.distance_to_target = if sample.enable_target_distance {
                vec.length()
            } else {
                0.0
            };
            vec /= vec.length();
            self.inner.los_meas.vec_unit = vec;
            self.inner.los_meas.frame = VectorFrame::BodyFrd;

            if orient != Rotation::Pitch270 {
                // rotate body frame vector based on orientation
                // this is done to have homogeneity among backends
                // frontend rotates it back to get correct body frame vector
                let _ = rotate_inverse(&mut self.inner.los_meas.vec_unit, orient);
                let _ = rotate_inverse(&mut self.inner.los_meas.vec_unit, Rotation::Pitch90);
            }

            self.inner.los_meas.valid = true;
            self.inner.los_meas.time_ms = sample.last_update_ms;
        } else {
            self.inner.los_meas.valid = false;
        }

        self.inner.expire_stale_los(now_ms);
    }

    /// Upstream `get_los_meas`.
    #[must_use]
    pub fn get_los_meas(&self) -> Option<(Vector3f, VectorFrame)> {
        self.inner.get_los_meas()
    }

    /// Upstream `los_meas_time_ms()`.
    #[must_use]
    pub fn los_meas_time_ms(&self) -> u32 {
        self.inner.los_meas_time_ms()
    }

    /// Upstream `distance_to_target()`. Metres. `0` means unknown.
    #[must_use]
    pub fn distance_to_target(&self) -> f32 {
        self.inner.distance_to_target()
    }

    /// Snapshot for [`crate::PrecLand::retrieve_los_meas`].
    #[must_use]
    pub fn los_sample(&self) -> Option<LosSample> {
        self.inner.los_sample()
    }
}
