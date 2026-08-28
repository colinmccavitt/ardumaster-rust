//! 24-state NavEKF3, upstream `libraries/AP_NavEKF3`. FW-009.
//!
//! This slice is the skeleton: the state index map, a 24-element state vector,
//! and the frontend `InitialiseFilter` / `UpdateFilter` dispatch that walks
//! the cores. The IMU sample ring that downsamples gyro/accel into the
//! fusion-horizon FIFO lives in [`measurements`]. Filter-mode control
//! (`setInhibitGPS`, `inFlight` / `onGround`, `controlFilterModes`)
//! lives in [`control`]. Gyro-bias reset / constrain / inactive-IMU
//! learn lives in [`gyro_bias`]. Mag-fusion enable / yaw-reset
//! (`SelectMagFusion`, `magFieldEarth`) lives in [`mag_fusion`].
//! Pos/vel fusion enable (`SelectVelPosFusion`, GPS inhibit / quality)
//! lives in [`pos_vel_fusion`]. Air-data / TAS fusion enable
//! (`SelectTasFusion`, innovation gate) lives in [`air_data_fusion`].
//! Height / baro fusion enable (`selectHeightForFusion`, `FuseBaro`,
//! baro offset) lives in [`height_fusion`]. Range-finder / optical-flow
//! fusion enable (`SelectFlowFusion`, `EstimateTerrainOffset` quality
//! gates) lives in [`rng_flow_fusion`]. Covariance prediction and
//! the 3-axis Kalman mag update are not here.
//!
//! # Twenty-four states, one vector
//!
//! Upstream keeps the filter state as a `Vector24` overlaid on a
//! `state_elements` struct (quaternion, velocity, position, gyro and accel
//! bias, earth and body magnetic field, wind). The overlay is a C union; the
//! port stores a `[Ftype; 24]` and names the slots with [`StateIndex`] so the
//! later covariance work can index the same way `P[i][i]` does, without
//! `unsafe`.
//!
//! # Precision is a feature, not a generic
//!
//! ADR-0004 decision 3: `ftype` is a global compile-time choice. This crate
//! uses [`ap_math::Ftype`] — `f32` by default, `f64` behind `ekf-double` —
//! and does not parameterise [`NavEkf3`] or [`NavEkf3Core`] over a scalar.
//! Mixed-precision builds have no upstream counterpart for sitl-diff.
//!
//! # What this slice does not include
//!
//! Strapdown prediction, covariance growth, GPS/baro/mag fusion, and the
//! AHRS `ekf3_loop` DCM fallback. That loop stays in `ap-ahrs`; this crate
//! is the estimator, not the AHRS glue. The IMU ring is [`measurements`];
//! the flight-mode latch is [`control`]; gyro bias is [`gyro_bias`];
//! mag-fusion enable / yaw-reset is [`mag_fusion`]; pos/vel fusion
//! enable is [`pos_vel_fusion`]; TAS / air-data enable is
//! [`air_data_fusion`]; height / baro enable is [`height_fusion`];
//! range-finder / optical-flow enable is [`rng_flow_fusion`].

#![no_std]

pub mod air_data_fusion;
pub mod control;
pub mod gyro_bias;
pub mod height_fusion;
pub mod mag_fusion;
pub mod measurements;
pub mod pos_vel_fusion;
pub mod rng_flow_fusion;

pub use air_data_fusion::{AirDataFusion, TasFuseSel, TAS_INNOV_GATE_DEFAULT, TAS_RETRY_TIME_MS};
pub use control::{AidingMode, FilterControl};
pub use gyro_bias::{GyroBias, GYRO_BIAS_INIT_DPS, GYRO_BIAS_LIMIT_RAD_S};
pub use height_fusion::{
    HeightFuseSel, HeightFusion, HeightSource, HGT_INNOV_GATE_DEFAULT, HGT_RETRY_TIME_MODE12_MS,
};
pub use mag_fusion::{MagFuseSel, MagFusion};
pub use measurements::{
    ImuBuffer, ImuElements, ImuRawSample, ImuSampleRing, EKF_TARGET_DT, EKF_TARGET_DT_MS,
    IMU_BUFFER_CAPACITY,
};
pub use pos_vel_fusion::{PosVelFuseSel, PosVelFusion};
pub use rng_flow_fusion::{
    FlowFuseSel, FlowUse, RngFlowFusion, RngFuseSel, DCM33_FLOW_MIN, FLOW_INNOV_GATE_DEFAULT,
    FLOW_USE_DEFAULT, MAX_FLOW_RATE_DEFAULT, RNG_INNOV_GATE_DEFAULT,
};

use ap_math::Ftype;

/// Length of the EKF3 state vector, upstream `Vector24` / `statesArray`.
pub const STATE_VECTOR_LEN: usize = 24;

/// Maximum number of EKF3 cores, upstream `MAX_EKF_CORES` in `AP_Nav_Common.h`.
pub const MAX_EKF_CORES: usize = 3;

/// One element of the 24-state vector, upstream `statesArray[i]`.
pub type StateVector = [Ftype; STATE_VECTOR_LEN];

/// Index into the 24-element EKF3 state vector, upstream `statesArray`.
///
/// Comments on `NavEKF3_core::state_elements` give the ranges: quaternion
/// 0..3, velocity 4..6, position 7..9, gyro bias 10..12, accel bias 13..15,
/// earth magnetic field 16..18, body magnetic field 19..21, wind 22..23.
#[repr(usize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateIndex {
    /// Quaternion w, upstream `stateStruct.quat` element 0.
    Quat0 = 0,
    /// Quaternion x, upstream `stateStruct.quat` element 1.
    Quat1 = 1,
    /// Quaternion y, upstream `stateStruct.quat` element 2.
    Quat2 = 2,
    /// Quaternion z, upstream `stateStruct.quat` element 3.
    Quat3 = 3,
    /// North velocity (m/s), upstream `stateStruct.velocity` x.
    VelN = 4,
    /// East velocity (m/s), upstream `stateStruct.velocity` y.
    VelE = 5,
    /// Down velocity (m/s), upstream `stateStruct.velocity` z.
    VelD = 6,
    /// North position (m), upstream `stateStruct.position` x.
    PosN = 7,
    /// East position (m), upstream `stateStruct.position` y.
    PosE = 8,
    /// Down position (m), upstream `stateStruct.position` z.
    PosD = 9,
    /// Body-X gyro delta-angle bias (rad), upstream `stateStruct.gyro_bias` x.
    GyroBiasX = 10,
    /// Body-Y gyro delta-angle bias (rad), upstream `stateStruct.gyro_bias` y.
    GyroBiasY = 11,
    /// Body-Z gyro delta-angle bias (rad), upstream `stateStruct.gyro_bias` z.
    GyroBiasZ = 12,
    /// Body-X accel delta-velocity bias (m/s), upstream `stateStruct.accel_bias` x.
    AccelBiasX = 13,
    /// Body-Y accel delta-velocity bias (m/s), upstream `stateStruct.accel_bias` y.
    AccelBiasY = 14,
    /// Body-Z accel delta-velocity bias (m/s), upstream `stateStruct.accel_bias` z.
    AccelBiasZ = 15,
    /// Earth-frame magnetic field North (Gauss), upstream `stateStruct.earth_magfield` x.
    EarthMagN = 16,
    /// Earth-frame magnetic field East (Gauss), upstream `stateStruct.earth_magfield` y.
    EarthMagE = 17,
    /// Earth-frame magnetic field Down (Gauss), upstream `stateStruct.earth_magfield` z.
    EarthMagD = 18,
    /// Body-X magnetic field (Gauss), upstream `stateStruct.body_magfield` x.
    BodyMagX = 19,
    /// Body-Y magnetic field (Gauss), upstream `stateStruct.body_magfield` y.
    BodyMagY = 20,
    /// Body-Z magnetic field (Gauss), upstream `stateStruct.body_magfield` z.
    BodyMagZ = 21,
    /// North wind velocity (m/s), upstream `stateStruct.wind_vel` x.
    WindVelN = 22,
    /// East wind velocity (m/s), upstream `stateStruct.wind_vel` y.
    WindVelE = 23,
}

impl StateIndex {
    /// Every state index, in vector order.
    pub const ALL: [Self; STATE_VECTOR_LEN] = [
        Self::Quat0,
        Self::Quat1,
        Self::Quat2,
        Self::Quat3,
        Self::VelN,
        Self::VelE,
        Self::VelD,
        Self::PosN,
        Self::PosE,
        Self::PosD,
        Self::GyroBiasX,
        Self::GyroBiasY,
        Self::GyroBiasZ,
        Self::AccelBiasX,
        Self::AccelBiasY,
        Self::AccelBiasZ,
        Self::EarthMagN,
        Self::EarthMagE,
        Self::EarthMagD,
        Self::BodyMagX,
        Self::BodyMagY,
        Self::BodyMagZ,
        Self::WindVelN,
        Self::WindVelE,
    ];

    /// The slot as a `usize` into [`StateVector`].
    #[must_use]
    pub const fn as_usize(self) -> usize {
        self as usize
    }
}

/// One EKF3 core, upstream `NavEKF3_core`.
///
/// The state vector is `[Ftype; 24]`, not `VectorN<T, 24>`: the scalar is the
/// crate-wide `Ftype` choice, not a type parameter.
#[derive(Debug, Clone)]
pub struct NavEkf3Core {
    /// Filter state, upstream `statesArray`.
    states: StateVector,
    /// Whether bootstrap has latched, upstream `statesInitialised`.
    states_initialised: bool,
    /// IMU frames since the last prediction, upstream `_framesSincePredict`.
    frames_since_predict: u32,
    /// Filter-mode latch, upstream `controlFilterModes` / `detectFlight`.
    control: FilterControl,
    /// Body-axis gyro bias, upstream `AP_NavEKF3_GyroBias.cpp`.
    gyro_bias: GyroBias,
    /// Mag-fusion enable / yaw-reset, upstream `AP_NavEKF3_MagFusion.cpp`.
    mag_fusion: MagFusion,
    /// Pos/vel fusion enable, upstream `AP_NavEKF3_PosVelFusion.cpp`.
    pos_vel: PosVelFusion,
    /// TAS / air-data fusion enable, upstream `AP_NavEKF3_AirDataFusion.cpp`.
    air_data: AirDataFusion,
    /// Height / baro fusion enable, upstream `selectHeightForFusion`.
    height: HeightFusion,
    /// Range-finder / optical-flow fusion enable, upstream `SelectFlowFusion`.
    rng_flow: RngFlowFusion,
}

impl Default for NavEkf3Core {
    fn default() -> Self {
        Self::new()
    }
}

impl NavEkf3Core {
    /// An uninitialised core with a zero state vector.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            states: [0.0 as Ftype; STATE_VECTOR_LEN],
            states_initialised: false,
            frames_since_predict: 0,
            control: FilterControl::new(),
            gyro_bias: GyroBias::new(),
            mag_fusion: MagFusion::new(),
            pos_vel: PosVelFusion::new(),
            air_data: AirDataFusion::new(),
            height: HeightFusion::new(),
            rng_flow: RngFlowFusion::new(),
        }
    }

    /// Whether bootstrap has completed, upstream `statesInitialised`.
    #[must_use]
    pub const fn states_initialised(&self) -> bool {
        self.states_initialised
    }

    /// IMU frames since the last prediction, upstream `getFramesSincePredict`.
    #[must_use]
    pub const fn frames_since_predict(&self) -> u32 {
        self.frames_since_predict
    }

    /// The 24-element state vector, upstream `statesArray`.
    #[must_use]
    pub const fn states(&self) -> &StateVector {
        &self.states
    }

    /// One state element, addressed the way `P[i][i]` is.
    #[must_use]
    pub fn state(&self, index: StateIndex) -> Ftype {
        // `StateIndex` is `repr(usize)` over `0..24`, so the slot is in range.
        match self.states.get(index.as_usize()) {
            Some(&value) => value,
            None => 0.0 as Ftype,
        }
    }

    /// Zero the state vector and latch initialisation, upstream
    /// `InitialiseFilterBootstrap`.
    ///
    /// The real bootstrap reads IMU, mag, GPS and baro, waits a second, then
    /// fills the IMU buffer before returning true. This stub has no sensors,
    /// so it reports success as soon as the vector is zeroed.
    pub fn initialise_filter_bootstrap(&mut self) -> bool {
        self.states = [0.0 as Ftype; STATE_VECTOR_LEN];
        self.states_initialised = true;
        self.frames_since_predict = 0;
        self.control.reset();
        self.gyro_bias.reset();
        self.gyro_bias.write_into_states(&mut self.states);
        self.mag_fusion.reset();
        self.mag_fusion.write_earth_into_states(&mut self.states);
        self.pos_vel.reset();
        self.air_data.reset();
        self.height.reset();
        self.rng_flow.reset();
        true
    }

    /// One filter cycle, upstream `NavEKF3_core::UpdateFilter(bool predict)`.
    ///
    /// Returns immediately when states have not been initialised. The predict
    /// flag is the only behaviour this stub keeps: a prediction resets the
    /// frame counter, a skipped prediction increments it (the frontend uses
    /// that to shed load across cores).
    pub fn update_filter(&mut self, predict: bool) {
        if !self.states_initialised {
            return;
        }
        // Upstream `UpdateFilter` runs `controlFilterModes` before IMU read
        // and `SelectMagFusion` after prediction.
        self.control.control_filter_modes();
        if predict {
            self.frames_since_predict = 0;
        } else {
            self.frames_since_predict = self.frames_since_predict.saturating_add(1);
        }
        self.mag_fusion.select_mag_fusion(self.states_initialised);
        self.mag_fusion.write_earth_into_states(&mut self.states);
        // Upstream `UpdateFilter` runs `SelectVelPosFusion` after mag.
        self.pos_vel.set_gps_inhibit(self.control.gps_inhibit());
        self.pos_vel.set_aiding_mode(self.control.aiding_mode());
        self.pos_vel
            .set_mag_fuse_timing(self.mag_fusion.mag_fuse_performed(), crate::EKF_TARGET_DT);
        self.pos_vel.select_vel_pos_fusion();
        // Upstream `SelectVelPosFusion` calls `selectHeightForFusion`.
        self.height.select_height_fusion(self.states_initialised);
        // Upstream `UpdateFilter` runs `SelectFlowFusion` before TAS.
        self.rng_flow
            .set_mag_fuse_timing(self.mag_fusion.mag_fuse_performed(), crate::EKF_TARGET_DT);
        self.rng_flow.select_rng_flow_fusion(self.states_initialised);
        // Upstream `UpdateFilter` runs `SelectTasFusion` after pos/vel.
        self.air_data
            .set_mag_fuse_timing(self.mag_fusion.mag_fuse_performed(), crate::EKF_TARGET_DT);
        self.air_data.select_tas_fusion(self.states_initialised);
    }

    /// Filter-mode latch, upstream `onGround` / `inFlight` / `gpsInhibit`.
    #[must_use]
    pub const fn control(&self) -> &FilterControl {
        &self.control
    }

    /// Mutable latch so callers can poke arm / GPS cues before an update.
    pub fn control_mut(&mut self) -> &mut FilterControl {
        &mut self.control
    }

    /// High certainty we are not flying, upstream `onGround`.
    #[must_use]
    pub const fn on_ground(&self) -> bool {
        self.control.on_ground()
    }

    /// High certainty we are flying, upstream `inFlight`.
    #[must_use]
    pub const fn in_flight(&self) -> bool {
        self.control.in_flight()
    }

    /// Historical `NavEKF3_core::setInhibitGPS`. See [`FilterControl::set_inhibit_gps`].
    pub fn set_inhibit_gps(&mut self) -> u8 {
        self.control.set_inhibit_gps()
    }

    /// Upstream `controlFilterModes`.
    pub fn control_filter_modes(&mut self) {
        self.control.control_filter_modes()
    }

    /// Gyro-bias helper, upstream `stateStruct.gyro_bias` / `resetGyroBias`.
    #[must_use]
    pub const fn gyro_bias(&self) -> &GyroBias {
        &self.gyro_bias
    }

    /// Mutable gyro-bias so tests can poke a learned offset.
    pub fn gyro_bias_mut(&mut self) -> &mut GyroBias {
        &mut self.gyro_bias
    }

    /// Upstream `NavEKF3_core::resetGyroBias`.
    pub fn reset_gyro_bias(&mut self) {
        self.gyro_bias.reset();
        self.gyro_bias.write_into_states(&mut self.states);
    }

    /// Gyro-bias half of upstream `ConstrainStates`.
    pub fn constrain_gyro_bias(&mut self) {
        self.gyro_bias.read_from_states(&self.states);
        self.gyro_bias.constrain();
        self.gyro_bias.write_into_states(&mut self.states);
    }

    /// Mag-fusion helper, upstream `SelectMagFusion` / `earth_magfield`.
    #[must_use]
    pub const fn mag_fusion(&self) -> &MagFusion {
        &self.mag_fusion
    }

    /// Mutable mag-fusion so tests can poke compass / sample flags.
    pub fn mag_fusion_mut(&mut self) -> &mut MagFusion {
        &mut self.mag_fusion
    }

    /// Upstream `NavEKF3_core::SelectMagFusion`.
    pub fn select_mag_fusion(&mut self) {
        self.mag_fusion.select_mag_fusion(self.states_initialised);
        self.mag_fusion.write_earth_into_states(&mut self.states);
    }

    /// External yaw-reset request, upstream `magYawResetRequest`.
    pub fn request_mag_yaw_reset(&mut self) {
        self.mag_fusion.request_yaw_reset();
    }

    /// Pos/vel fusion helper, upstream `SelectVelPosFusion`.
    #[must_use]
    pub const fn pos_vel(&self) -> &PosVelFusion {
        &self.pos_vel
    }

    /// Mutable pos/vel fusion so tests can poke GPS / quality flags.
    pub fn pos_vel_mut(&mut self) -> &mut PosVelFusion {
        &mut self.pos_vel
    }

    /// Upstream `NavEKF3_core::SelectVelPosFusion`.
    pub fn select_vel_pos_fusion(&mut self) {
        self.pos_vel.set_gps_inhibit(self.control.gps_inhibit());
        self.pos_vel.set_aiding_mode(self.control.aiding_mode());
        self.pos_vel.select_vel_pos_fusion();
    }

    /// TAS / air-data helper, upstream `SelectTasFusion`.
    #[must_use]
    pub const fn air_data(&self) -> &AirDataFusion {
        &self.air_data
    }

    /// Mutable TAS latch so tests can poke sample / wind / quality flags.
    pub fn air_data_mut(&mut self) -> &mut AirDataFusion {
        &mut self.air_data
    }

    /// Upstream `NavEKF3_core::SelectTasFusion`.
    pub fn select_tas_fusion(&mut self) {
        self.air_data
            .set_mag_fuse_timing(self.mag_fusion.mag_fuse_performed(), crate::EKF_TARGET_DT);
        self.air_data.select_tas_fusion(self.states_initialised);
    }

    /// Height / baro helper, upstream `selectHeightForFusion`.
    #[must_use]
    pub const fn height(&self) -> &HeightFusion {
        &self.height
    }

    /// Mutable height latch so tests can poke baro / source / quality flags.
    pub fn height_mut(&mut self) -> &mut HeightFusion {
        &mut self.height
    }

    /// Upstream `NavEKF3_core::selectHeightForFusion`.
    pub fn select_height_fusion(&mut self) {
        self.height.select_height_fusion(self.states_initialised);
    }

    /// Range-finder / optical-flow helper, upstream `SelectFlowFusion`.
    #[must_use]
    pub const fn rng_flow(&self) -> &RngFlowFusion {
        &self.rng_flow
    }

    /// Mutable rng/flow latch so tests can poke sample / tilt / quality flags.
    pub fn rng_flow_mut(&mut self) -> &mut RngFlowFusion {
        &mut self.rng_flow
    }

    /// Upstream `NavEKF3_core::SelectFlowFusion`.
    pub fn select_rng_flow_fusion(&mut self) {
        self.rng_flow
            .set_mag_fuse_timing(self.mag_fusion.mag_fuse_performed(), crate::EKF_TARGET_DT);
        self.rng_flow.select_rng_flow_fusion(self.states_initialised);
    }
}

/// Frontend that owns the cores, upstream `NavEKF3`.
///
/// `InitialiseFilter` counts IMUs from the mask and bootstraps one core each.
/// `UpdateFilter` returns immediately when no cores exist, otherwise walks
/// them. Lane switching and CPU-budget prediction suppression are not here.
#[derive(Debug, Clone)]
pub struct NavEkf3 {
    /// Parameter `EK3_ENABLE`, upstream `_enable`.
    enable: i8,
    /// Parameter `EK3_IMU_MASK`, upstream `_imuMask`.
    imu_mask: u8,
    /// Instantiated core count, upstream `num_cores`.
    num_cores: u8,
    /// Core storage, upstream `core[MAX_EKF_CORES]`.
    cores: [NavEkf3Core; MAX_EKF_CORES],
}

impl Default for NavEkf3 {
    fn default() -> Self {
        Self::new()
    }
}

impl NavEkf3 {
    /// Enabled frontend with IMU 0 selected, the usual Plane default mask.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: 1,
            imu_mask: 1,
            num_cores: 0,
            cores: [NavEkf3Core::new(), NavEkf3Core::new(), NavEkf3Core::new()],
        }
    }

    /// Construct with explicit enable and IMU mask, upstream `_enable` /
    /// `_imuMask`.
    #[must_use]
    pub const fn with_mask(enable: bool, imu_mask: u8) -> Self {
        Self {
            enable: if enable { 1 } else { 0 },
            imu_mask,
            num_cores: 0,
            cores: [NavEkf3Core::new(), NavEkf3Core::new(), NavEkf3Core::new()],
        }
    }

    /// Instantiated core count, upstream `num_cores`.
    #[must_use]
    pub const fn num_cores(&self) -> u8 {
        self.num_cores
    }

    /// Core at `index`, if that slot was instantiated.
    #[must_use]
    pub fn core(&self, index: usize) -> Option<&NavEkf3Core> {
        if index < self.num_cores as usize {
            self.cores.get(index)
        } else {
            None
        }
    }

    /// Allocate and bootstrap cores, upstream `NavEKF3::InitialiseFilter`.
    ///
    /// Returns false when the estimator is disabled or the IMU mask is empty,
    /// matching the `_enable == 0 || _imuMask == 0` guard. On success every
    /// selected IMU gets a core and [`NavEkf3Core::initialise_filter_bootstrap`].
    pub fn initialise_filter(&mut self) -> bool {
        if self.enable == 0 || self.imu_mask == 0 {
            self.num_cores = 0;
            return false;
        }

        let mut n = 0usize;
        for bit in 0..8u8 {
            if n >= MAX_EKF_CORES {
                break;
            }
            if self.imu_mask & (1 << bit) == 0 {
                continue;
            }
            let Some(core) = self.cores.get_mut(n) else {
                break;
            };
            if !core.initialise_filter_bootstrap() {
                self.num_cores = 0;
                return false;
            }
            n = n.saturating_add(1);
        }

        #[allow(
            clippy::cast_possible_truncation,
            reason = "n is capped at MAX_EKF_CORES (3)"
        )]
        {
            self.num_cores = n as u8;
        }
        self.num_cores > 0
    }

    /// Dispatch one cycle to every core, upstream `NavEKF3::UpdateFilter`.
    ///
    /// Mirrors `if (!core) return;` — a frontend that has not initialised
    /// does nothing. Each live core is given `predict = true`; the CPU-budget
    /// suppression that can set that false is not in this slice.
    pub fn update_filter(&mut self) {
        if self.num_cores == 0 {
            return;
        }
        for core in self.cores.iter_mut().take(self.num_cores as usize) {
            core.update_filter(true);
        }
    }

    /// Historical `NavEKF3::setInhibitGPS`.
    ///
    /// Returns 0 when no cores exist (upstream `if (!core) return 0`).
    /// Otherwise forwards to the primary core (index 0).
    pub fn set_inhibit_gps(&mut self) -> u8 {
        if self.num_cores == 0 {
            return 0;
        }
        match self.cores.get_mut(0) {
            Some(core) => core.set_inhibit_gps(),
            None => 0,
        }
    }

    /// Upstream `NavEKF3::resetGyroBias`: walk every live core.
    ///
    /// No-ops when no cores exist (`if (!core) return`).
    pub fn reset_gyro_bias(&mut self) {
        if self.num_cores == 0 {
            return;
        }
        for core in self.cores.iter_mut().take(self.num_cores as usize) {
            core.reset_gyro_bias();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::EKF_DOUBLE;

    #[test]
    fn state_vector_is_twenty_four() {
        assert_eq!(STATE_VECTOR_LEN, 24);
        assert_eq!(StateIndex::ALL.len(), 24);
        assert_eq!(StateIndex::Quat0.as_usize(), 0);
        assert_eq!(StateIndex::VelN.as_usize(), 4);
        assert_eq!(StateIndex::PosN.as_usize(), 7);
        assert_eq!(StateIndex::GyroBiasX.as_usize(), 10);
        assert_eq!(StateIndex::AccelBiasX.as_usize(), 13);
        assert_eq!(StateIndex::EarthMagN.as_usize(), 16);
        assert_eq!(StateIndex::BodyMagX.as_usize(), 19);
        assert_eq!(StateIndex::WindVelN.as_usize(), 22);
        assert_eq!(StateIndex::WindVelE.as_usize(), 23);
        assert_eq!(StateIndex::WindVelE.as_usize() + 1, STATE_VECTOR_LEN);
        for (i, index) in StateIndex::ALL.iter().enumerate() {
            assert_eq!(index.as_usize(), i);
        }
    }

    #[test]
    fn state_scalar_follows_ekf_double_feature() {
        // ADR-0004: the estimator is not generic; Ftype is the only scalar.
        let width = core::mem::size_of::<Ftype>();
        if EKF_DOUBLE {
            assert_eq!(width, 8);
        } else {
            assert_eq!(width, 4);
        }
        assert_eq!(
            core::mem::size_of::<StateVector>(),
            STATE_VECTOR_LEN * width
        );
        assert!(core::mem::size_of::<NavEkf3Core>() >= core::mem::size_of::<StateVector>());
    }

    #[test]
    fn initialise_filter_rejects_disabled_or_empty_mask() {
        let mut off = NavEkf3::with_mask(false, 1);
        assert!(!off.initialise_filter());
        assert_eq!(off.num_cores(), 0);

        let mut empty = NavEkf3::with_mask(true, 0);
        assert!(!empty.initialise_filter());
        assert_eq!(empty.num_cores(), 0);
    }

    #[test]
    fn initialise_and_update_dispatch_to_cores() {
        let mut ekf = NavEkf3::with_mask(true, 0b011);
        assert!(ekf.initialise_filter());
        assert_eq!(ekf.num_cores(), 2);
        assert_eq!(MAX_EKF_CORES, 3);

        let core0 = ekf.core(0).expect("core 0");
        assert!(core0.states_initialised());
        assert_eq!(core0.states().len(), STATE_VECTOR_LEN);
        assert_eq!(core0.state(StateIndex::Quat0), 0.0 as Ftype);
        assert_eq!(core0.state(StateIndex::WindVelE), 0.0 as Ftype);
        assert!(ekf.core(2).is_none());

        ekf.update_filter();
        let core0 = ekf.core(0).expect("core 0 after update");
        assert_eq!(core0.frames_since_predict(), 0);
        let core1 = ekf.core(1).expect("core 1 after update");
        assert!(core1.states_initialised());
        assert_eq!(core1.frames_since_predict(), 0);
    }

    #[test]
    fn update_filter_is_noop_before_init() {
        let mut ekf = NavEkf3::new();
        ekf.update_filter();
        assert_eq!(ekf.num_cores(), 0);
        assert!(ekf.core(0).is_none());
    }

    #[test]
    fn core_update_skips_until_bootstrap() {
        let mut core = NavEkf3Core::new();
        core.update_filter(true);
        assert!(!core.states_initialised());
        assert_eq!(core.frames_since_predict(), 0);

        assert!(core.initialise_filter_bootstrap());
        core.update_filter(false);
        assert_eq!(core.frames_since_predict(), 1);
        core.update_filter(true);
        assert_eq!(core.frames_since_predict(), 0);
    }

    #[test]
    fn frontend_set_inhibit_gps_needs_cores_then_latches() {
        let mut ekf = NavEkf3::new();
        assert_eq!(ekf.set_inhibit_gps(), 0);
        assert!(ekf.initialise_filter());
        assert_eq!(ekf.set_inhibit_gps(), 1);
        let core = ekf.core(0).expect("primary");
        assert!(core.control().gps_inhibit());
        assert!(core.on_ground());
        assert!(!core.in_flight());
    }

    #[test]
    fn frontend_reset_gyro_bias_is_noop_then_walks_cores() {
        let mut ekf = NavEkf3::new();
        ekf.reset_gyro_bias();
        assert_eq!(ekf.num_cores(), 0);

        assert!(ekf.initialise_filter());
        ekf.reset_gyro_bias();
        let core = ekf.core(0).expect("primary");
        assert_eq!(core.state(StateIndex::GyroBiasX), 0.0 as Ftype);
        assert_eq!(core.state(StateIndex::GyroBiasY), 0.0 as Ftype);
        assert_eq!(core.state(StateIndex::GyroBiasZ), 0.0 as Ftype);
        let expected = core.gyro_bias().variance().x;
        assert!(expected > 0.0 as Ftype);
    }

    #[test]
    fn core_mag_yaw_reset_writes_earth_field_states() {
        let mut core = NavEkf3Core::new();
        assert!(core.initialise_filter_bootstrap());
        core.mag_fusion_mut().set_use_compass(true);
        core.mag_fusion_mut().set_tilt_align_complete(true);
        core.mag_fusion_mut().set_mag_data(
            true,
            ap_math::vector3::Vector3::new(0.22 as Ftype, 0.05 as Ftype, 0.41 as Ftype),
        );
        core.request_mag_yaw_reset();
        core.select_mag_fusion();
        assert!(core.mag_fusion().yaw_align_complete());
        assert_eq!(core.mag_fusion().mag_fusion_sel(), MagFuseSel::FuseYaw);
        let n = core.state(StateIndex::EarthMagN) - (0.22 as Ftype);
        let e = core.state(StateIndex::EarthMagE) - (0.05 as Ftype);
        let d = core.state(StateIndex::EarthMagD) - (0.41 as Ftype);
        assert!(n * n + e * e + d * d < 1.0e-12 as Ftype);
    }

    #[test]
    fn core_pos_vel_gate_needs_absolute_gps_and_quality() {
        let mut core = NavEkf3Core::new();
        assert!(core.initialise_filter_bootstrap());
        core.select_vel_pos_fusion();
        assert_eq!(core.pos_vel().fuse_sel(), PosVelFuseSel::NotFusing);

        core.control_mut()
            .set_aiding_mode_for_test(AidingMode::Absolute);
        core.pos_vel_mut().set_gps_data_to_fuse(true);
        core.pos_vel_mut().set_gps_accuracy_good(true);
        core.select_vel_pos_fusion();
        assert_eq!(core.pos_vel().fuse_sel(), PosVelFuseSel::FuseVelPos);
        assert!(core.pos_vel().fuse_performed());

        // `setInhibitGPS` is accepted only before AID_ABSOLUTE.
        let mut inhibited = NavEkf3Core::new();
        assert!(inhibited.initialise_filter_bootstrap());
        assert_eq!(inhibited.set_inhibit_gps(), 1);
        inhibited
            .control_mut()
            .set_aiding_mode_for_test(AidingMode::Absolute);
        inhibited.pos_vel_mut().set_gps_data_to_fuse(true);
        inhibited.pos_vel_mut().set_gps_accuracy_good(true);
        inhibited.select_vel_pos_fusion();
        assert!(inhibited.control().gps_inhibit());
        assert_eq!(inhibited.pos_vel().fuse_sel(), PosVelFuseSel::NotFusing);
    }

    #[test]
    fn core_tas_gate_needs_wind_and_airspeed() {
        let mut core = NavEkf3Core::new();
        assert!(core.initialise_filter_bootstrap());
        core.select_tas_fusion();
        assert_eq!(core.air_data().fuse_sel(), TasFuseSel::NotFusing);

        core.air_data_mut().set_inhibit_wind_states(false);
        core.air_data_mut().set_tas_data(true, 20.0 as Ftype, true);
        core.air_data_mut()
            .set_velocity(20.0 as Ftype, 0.0 as Ftype, 0.0 as Ftype);
        core.select_tas_fusion();
        assert_eq!(core.air_data().fuse_sel(), TasFuseSel::FuseTas);
        assert!(core.air_data().fuse_performed());

        // Wind inhibit keeps SelectTasFusion closed even with a sample.
        let mut inhibited = NavEkf3Core::new();
        assert!(inhibited.initialise_filter_bootstrap());
        inhibited
            .air_data_mut()
            .set_tas_data(true, 20.0 as Ftype, true);
        inhibited
            .air_data_mut()
            .set_velocity(20.0 as Ftype, 0.0 as Ftype, 0.0 as Ftype);
        inhibited.select_tas_fusion();
        assert!(inhibited.air_data().inhibit_wind_states());
        assert_eq!(inhibited.air_data().fuse_sel(), TasFuseSel::NotFusing);
    }

    #[test]
    fn core_height_gate_needs_baro_sample() {
        let mut core = NavEkf3Core::new();
        assert!(core.initialise_filter_bootstrap());
        core.select_height_fusion();
        assert_eq!(core.height().fuse_sel(), HeightFuseSel::NotFusing);

        core.height_mut().set_quality_overrides(false, false, false);
        core.height_mut().set_baro_data(true, 10.0 as Ftype);
        core.height_mut().set_position_d(-(10.0 as Ftype));
        core.height_mut().set_imu_sample_time_ms(1000);
        core.select_height_fusion();
        assert_eq!(core.height().fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(core.height().fuse_performed());

        // Lost GPS height falls back to baro.
        let mut fallback = NavEkf3Core::new();
        assert!(fallback.initialise_filter_bootstrap());
        fallback
            .height_mut()
            .set_quality_overrides(false, false, false);
        fallback
            .height_mut()
            .set_configured_source(HeightSource::Gps);
        fallback
            .height_mut()
            .set_active_hgt_source(HeightSource::Gps);
        fallback.height_mut().set_gps_height(false, false);
        fallback.height_mut().set_baro_data(true, 10.0 as Ftype);
        fallback.height_mut().set_position_d(-(10.0 as Ftype));
        fallback.height_mut().set_imu_sample_time_ms(1000);
        fallback.select_height_fusion();
        assert_eq!(fallback.height().active_hgt_source(), HeightSource::Baro);
        assert_eq!(fallback.height().fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(fallback.height().fuse_performed());
    }

    #[test]
    fn core_rng_flow_gate_needs_sample_tilt_and_nav_source() {
        let mut core = NavEkf3Core::new();
        assert!(core.initialise_filter_bootstrap());
        core.select_rng_flow_fusion();
        assert_eq!(core.rng_flow().rng_fuse_sel(), RngFuseSel::NotFusing);
        assert_eq!(core.rng_flow().flow_fuse_sel(), FlowFuseSel::NotFusing);

        core.rng_flow_mut().set_range_data(true, 3.0 as Ftype);
        core.rng_flow_mut().set_rng_innov(0.0 as Ftype, 1.0 as Ftype);
        core.rng_flow_mut().set_tilt_ok(true);
        core.select_rng_flow_fusion();
        assert_eq!(core.rng_flow().rng_fuse_sel(), RngFuseSel::FuseRng);
        assert!(core.rng_flow().rng_fuse_performed());

        // Plane FLOW_USE default is TERRAIN: main-filter flow stays closed
        // until NAV + OPTFLOW XY are selected.
        let mut flow = NavEkf3Core::new();
        assert!(flow.initialise_filter_bootstrap());
        flow.rng_flow_mut().set_flow_use(FlowUse::Nav);
        flow.rng_flow_mut().set_use_optflow_xy(true);
        flow.rng_flow_mut()
            .set_flow_data(true, 0.2 as Ftype, 0.1 as Ftype);
        flow.rng_flow_mut().set_flow_innov(0.0 as Ftype, 1.0 as Ftype);
        flow.rng_flow_mut().set_tilt_ok(true);
        flow.rng_flow_mut().set_takeoff_detected(true);
        flow.select_rng_flow_fusion();
        assert_eq!(flow.rng_flow().flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(flow.rng_flow().flow_fuse_performed());
    }
}
