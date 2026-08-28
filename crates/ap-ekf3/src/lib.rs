//! 24-state NavEKF3, upstream `libraries/AP_NavEKF3`. FW-009.
//!
//! This slice is the skeleton: the state index map, a 24-element state vector,
//! and the frontend `InitialiseFilter` / `UpdateFilter` dispatch that walks
//! the cores. The covariance prediction, fusion, and IMU buffer are not here.
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
//! Strapdown prediction, covariance growth, GPS/baro/mag fusion, the IMU
//! sample buffer, and the AHRS `ekf3_loop` DCM fallback. That loop stays in
//! `ap-ahrs`; this crate is the estimator, not the AHRS glue.

#![no_std]

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
        if predict {
            self.frames_since_predict = 0;
        } else {
            self.frames_since_predict = self.frames_since_predict.saturating_add(1);
        }
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
}
