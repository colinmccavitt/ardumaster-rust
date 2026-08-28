//! Covariance prediction, upstream `NavEKF3_core::CovariancePrediction`.
//!
//! This slice is the process-noise inject and the symmetrising copy from
//! `nextP` back onto `P`. Upstream's auto-generated 24×24 strapdown
//! `F·P·Fᵀ` growth is not here: `nextP` starts as a copy of `P` (identity
//! propagation) so the diagonal `Q` inject and the `P[row][col] =
//! P[col][row] = nextP[col][row]` copy-back can be tested without the
//! symbolic Jacobian.
//!
//! After the copy-back, [`Covariance::constrain_variances`] clamps the
//! diagonals the way `ConstrainVariances` does so an ill-conditioned `P`
//! cannot grow without bound. Quaternion / velocity / position IMU-noise
//! growth stays with the later Jacobian port.

use ap_math::scalar::{constrain_value, radians, sq};
use ap_math::Ftype;

use crate::gyro_bias::GYRO_BIAS_INIT_DPS;
use crate::measurements::EKF_TARGET_DT;
use crate::{StateIndex, STATE_VECTOR_LEN};

/// Length of the process-noise vector applied to states 10..23.
///
/// Upstream `Vector14 processNoiseVariance`: gyro bias, accel bias, earth
/// mag, body mag, wind.
pub const PROCESS_NOISE_LEN: usize = 14;

/// Horizontal position variance sum that freezes further NE growth.
///
/// Upstream `(P[7][7] + P[8][8]) > 1e4f` (100 m).
pub const POS_VAR_GROWTH_LIMIT: Ftype = 1.0e4;

/// Floor on NE / D velocity variance, upstream `VEL_STATE_MIN_VARIANCE`.
pub const VEL_STATE_MIN_VARIANCE: Ftype = 1.0e-4;

/// Floor on NED position variance, upstream `POS_STATE_MIN_VARIANCE`.
pub const POS_STATE_MIN_VARIANCE: Ftype = 1.0e-4;

/// Wind-velocity variance ceiling, upstream `WIND_VEL_VARIANCE_MAX`.
pub const WIND_VEL_VARIANCE_MAX: Ftype = 400.0;

/// Plane `EK3_GBIAS_P_NSE` default (rad/s).
pub const GBIAS_P_NSE_DEFAULT: Ftype = 1.0e-3;

/// Plane `EK3_ABIAS_P_NSE` default (m/s²).
pub const ABIAS_P_NSE_DEFAULT: Ftype = 2.0e-2;

/// Plane `EK3_MAGE_P_NSE` default (gauss/s).
pub const MAGE_P_NSE_DEFAULT: Ftype = 1.0e-3;

/// Plane `EK3_MAGB_P_NSE` default (gauss/s).
pub const MAGB_P_NSE_DEFAULT: Ftype = 1.0e-4;

/// Plane `EK3_WIND_P_NSE` default (m/s²).
pub const WIND_P_NSE_DEFAULT: Ftype = 0.1;

/// Plane `EK3_WIND_PSCALE` default.
pub const WND_VAR_HGT_RATE_SCALE_DEFAULT: Ftype = 1.0;

/// Plane `EK3_VELNE_M_NSE` default (m/s).
pub const VELNE_M_NSE_DEFAULT: Ftype = 0.5;

/// Plane `EK3_VELD_M_NSE` default (m/s).
pub const VELD_M_NSE_DEFAULT: Ftype = 0.7;

/// Plane `EK3_POSNE_M_NSE` default (m).
pub const POSNE_M_NSE_DEFAULT: Ftype = 0.5;

/// Plane `EK3_ALT_M_NSE` default (m).
pub const ALT_M_NSE_DEFAULT: Ftype = 3.0;

/// Plane `EK3_MAG_M_NSE` default (gauss).
pub const MAG_M_NSE_DEFAULT: Ftype = 0.05;

/// Plane `EK3_ACC_BIAS_LIM` default (m/s²).
pub const ACC_BIAS_LIM_DEFAULT: Ftype = 1.0;

/// Upstream `ACCEL_BIAS_LIM_SCALER` used to seed `P[13..15]`.
pub const ACCEL_BIAS_LIM_SCALER: Ftype = 0.2;

/// Initial quaternion rotation-vector 1-sigma (rad), `sq(0.1f)` in
/// `CovarianceInit`.
const QUAT_INIT_ROT_SIGMA: Ftype = 0.1;

/// Gyro-bias variance clamp rate (rad/s), `sq(0.175 * dtEkfAvg)`.
const GYRO_BIAS_VAR_CLAMP_RATE: Ftype = 0.175;

/// Accel-bias variance clamp rate scale, `sq(10.0 * dtEkfAvg)`.
const ACCEL_BIAS_VAR_CLAMP_RATE: Ftype = 10.0;

/// Minimum safe accel-bias variance, upstream `minSafeStateVar`.
const ACCEL_BIAS_MIN_SAFE_VAR: Ftype = 5.0e-9;

/// Earth / body mag variance ceiling.
const MAG_VAR_MAX: Ftype = 0.01;

/// Attitude-error variance ceiling.
const ATT_VAR_MAX: Ftype = 1.0;

/// NE velocity variance ceiling.
const VEL_NE_VAR_MAX: Ftype = 1.0e3;

/// NED position variance ceiling.
const POS_VAR_MAX: Ftype = 1.0e6;

/// 24×24 state covariance, upstream `P[24][24]`.
pub type CovMatrix = [[Ftype; STATE_VECTOR_LEN]; STATE_VECTOR_LEN];

/// Process-noise diagonals for states 10..23, upstream `Vector14`.
pub type ProcessNoise = [Ftype; PROCESS_NOISE_LEN];

/// Predicted state covariance and the `Q` inject that grows it.
///
/// Upstream overlays `P` on the core. The port keeps the matrix here so
/// reset / predict / constrain can run without the strapdown Jacobian.
#[derive(Debug, Clone)]
pub struct Covariance {
    /// State covariance, upstream `P`.
    p: CovMatrix,
    /// Last process-noise vector, upstream `processNoiseVariance`.
    process_noise: ProcessNoise,
    /// Constrained prediction dt (s), upstream `dt`.
    dt: Ftype,
    /// Expected EKF update interval (s), upstream `dtEkfAvg`.
    dt_ekf_avg: Ftype,
    /// Delayed IMU delta-angle integration period (s).
    del_ang_dt: Ftype,
    /// Delayed IMU delta-velocity integration period (s).
    del_vel_dt: Ftype,
    /// Filtered height rate (m/s), upstream `hgtRate`.
    hgt_rate: Ftype,
    /// Down velocity used to update [`hgt_rate`](Self::hgt_rate) (m/s).
    vel_d: Ftype,
    /// Highest active state index, upstream `stateIndexLim`.
    state_index_lim: u8,
    /// Gyro-bias process noise (rad/s), `EK3_GBIAS_P_NSE`.
    gyro_bias_process_noise: Ftype,
    /// Accel-bias process noise (m/s²), `EK3_ABIAS_P_NSE`.
    accel_bias_process_noise: Ftype,
    /// Earth-mag process noise (gauss/s), `EK3_MAGE_P_NSE`.
    mag_earth_process_noise: Ftype,
    /// Body-mag process noise (gauss/s), `EK3_MAGB_P_NSE`.
    mag_body_process_noise: Ftype,
    /// Wind-velocity process noise (m/s²), `EK3_WIND_P_NSE`.
    wind_vel_process_noise: Ftype,
    /// Height-rate scale on wind `Q`, `EK3_WIND_PSCALE`.
    wnd_var_hgt_rate_scale: Ftype,
    /// GPS / baro / mag measurement-noise seeds for [`covariance_init`].
    gps_horiz_vel_noise: Ftype,
    gps_vert_vel_noise: Ftype,
    gps_horiz_pos_noise: Ftype,
    baro_alt_noise: Ftype,
    mag_noise: Ftype,
    acc_bias_lim: Ftype,
    /// Upstream `inhibitDelAngBiasStates`.
    inhibit_del_ang_bias: bool,
    /// Upstream `inhibitDelVelBiasStates`.
    inhibit_del_vel_bias: bool,
    /// Upstream `inhibitMagStates`.
    inhibit_mag_states: bool,
    /// Previous `inhibitMagStates`, used to request a mag-variance reset.
    last_inhibit_mag_states: bool,
    /// Upstream `inhibitWindStates`.
    inhibit_wind_states: bool,
    /// Upstream `treatWindStatesAsTruth`.
    treat_wind_as_truth: bool,
    /// Upstream `tasDataDelayed.allowFusion`.
    tas_allow_fusion: bool,
    /// Upstream `needMagBodyVarReset`.
    need_mag_body_var_reset: bool,
    /// Upstream `needEarthBodyVarReset`.
    need_earth_var_reset: bool,
}

impl Default for Covariance {
    fn default() -> Self {
        Self::new()
    }
}

impl Covariance {
    /// Zero `P`, Plane process-noise defaults, all 24 states active.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            p: [[0.0 as Ftype; STATE_VECTOR_LEN]; STATE_VECTOR_LEN],
            process_noise: [0.0 as Ftype; PROCESS_NOISE_LEN],
            dt: EKF_TARGET_DT,
            dt_ekf_avg: EKF_TARGET_DT,
            del_ang_dt: EKF_TARGET_DT,
            del_vel_dt: EKF_TARGET_DT,
            hgt_rate: 0.0 as Ftype,
            vel_d: 0.0 as Ftype,
            state_index_lim: 23,
            gyro_bias_process_noise: GBIAS_P_NSE_DEFAULT,
            accel_bias_process_noise: ABIAS_P_NSE_DEFAULT,
            mag_earth_process_noise: MAGE_P_NSE_DEFAULT,
            mag_body_process_noise: MAGB_P_NSE_DEFAULT,
            wind_vel_process_noise: WIND_P_NSE_DEFAULT,
            wnd_var_hgt_rate_scale: WND_VAR_HGT_RATE_SCALE_DEFAULT,
            gps_horiz_vel_noise: VELNE_M_NSE_DEFAULT,
            gps_vert_vel_noise: VELD_M_NSE_DEFAULT,
            gps_horiz_pos_noise: POSNE_M_NSE_DEFAULT,
            baro_alt_noise: ALT_M_NSE_DEFAULT,
            mag_noise: MAG_M_NSE_DEFAULT,
            acc_bias_lim: ACC_BIAS_LIM_DEFAULT,
            inhibit_del_ang_bias: false,
            inhibit_del_vel_bias: false,
            inhibit_mag_states: false,
            last_inhibit_mag_states: false,
            inhibit_wind_states: false,
            treat_wind_as_truth: false,
            tas_allow_fusion: true,
            need_mag_body_var_reset: false,
            need_earth_var_reset: false,
        }
    }

    /// The covariance matrix, upstream `P`.
    #[must_use]
    pub const fn matrix(&self) -> &CovMatrix {
        &self.p
    }

    /// One element of `P`, or zero when the index is out of range.
    #[must_use]
    pub fn p(&self, row: usize, col: usize) -> Ftype {
        p_get(&self.p, row, col)
    }

    /// Diagonal variance for `index`, the way `P[i][i]` is read.
    #[must_use]
    pub fn variance(&self, index: StateIndex) -> Ftype {
        self.p(index.as_usize(), index.as_usize())
    }

    /// Last process-noise vector written by [`covariance_prediction`].
    #[must_use]
    pub const fn process_noise(&self) -> &ProcessNoise {
        &self.process_noise
    }

    /// Constrained prediction dt (s).
    #[must_use]
    pub const fn dt(&self) -> Ftype {
        self.dt
    }

    /// Expected EKF update interval (s).
    #[must_use]
    pub const fn dt_ekf_avg(&self) -> Ftype {
        self.dt_ekf_avg
    }

    /// Filtered height rate (m/s).
    #[must_use]
    pub const fn hgt_rate(&self) -> Ftype {
        self.hgt_rate
    }

    /// Highest active state index.
    #[must_use]
    pub const fn state_index_lim(&self) -> u8 {
        self.state_index_lim
    }

    /// Poke a single `P[row][col]`. Tests use this to reach the
    /// symmetry / freeze / constrain paths without a Jacobian.
    pub fn set_p(&mut self, row: usize, col: usize, value: Ftype) {
        p_set(&mut self.p, row, col, value);
    }

    /// Override `dtEkfAvg` and the IMU integration periods together.
    pub fn set_dt_ekf_avg(&mut self, dt_ekf_avg: Ftype) {
        self.dt_ekf_avg = dt_ekf_avg;
        self.del_ang_dt = dt_ekf_avg;
        self.del_vel_dt = dt_ekf_avg;
        self.dt = dt_ekf_avg;
    }

    /// Delayed IMU integration periods, upstream `imuDataDelayed.delAngDT`
    /// / `delVelDT`.
    pub fn set_imu_dt(&mut self, del_ang_dt: Ftype, del_vel_dt: Ftype) {
        self.del_ang_dt = del_ang_dt;
        self.del_vel_dt = del_vel_dt;
    }

    /// Down velocity that drives the height-rate filter (m/s).
    pub fn set_velocity_d(&mut self, vel_d: Ftype) {
        self.vel_d = vel_d;
    }

    /// Override `EK3_GBIAS_P_NSE`.
    pub fn set_gyro_bias_process_noise(&mut self, noise: Ftype) {
        self.gyro_bias_process_noise = noise;
    }

    /// Override `EK3_ABIAS_P_NSE`.
    pub fn set_accel_bias_process_noise(&mut self, noise: Ftype) {
        self.accel_bias_process_noise = noise;
    }

    /// Override `EK3_WIND_P_NSE`.
    pub fn set_wind_vel_process_noise(&mut self, noise: Ftype) {
        self.wind_vel_process_noise = noise;
    }

    /// Override `EK3_WIND_PSCALE`.
    pub fn set_wnd_var_hgt_rate_scale(&mut self, scale: Ftype) {
        self.wnd_var_hgt_rate_scale = scale;
    }

    /// Upstream `inhibitDelAngBiasStates`.
    pub fn set_inhibit_del_ang_bias(&mut self, inhibit: bool) {
        self.inhibit_del_ang_bias = inhibit;
        self.update_state_index_lim();
    }

    /// Upstream `inhibitDelVelBiasStates`.
    pub fn set_inhibit_del_vel_bias(&mut self, inhibit: bool) {
        self.inhibit_del_vel_bias = inhibit;
        self.update_state_index_lim();
    }

    /// Upstream `inhibitMagStates`.
    pub fn set_inhibit_mag_states(&mut self, inhibit: bool) {
        self.inhibit_mag_states = inhibit;
        self.update_state_index_lim();
    }

    /// Upstream `inhibitWindStates`.
    pub fn set_inhibit_wind_states(&mut self, inhibit: bool) {
        self.inhibit_wind_states = inhibit;
        self.update_state_index_lim();
    }

    /// Upstream `tasDataDelayed.allowFusion`.
    pub fn set_tas_allow_fusion(&mut self, allow: bool) {
        self.tas_allow_fusion = allow;
    }

    /// Upstream `treatWindStatesAsTruth`.
    pub fn set_treat_wind_as_truth(&mut self, treat: bool) {
        self.treat_wind_as_truth = treat;
    }

    /// Whether `P` is symmetric through `state_index_lim`.
    #[must_use]
    pub fn is_symmetric(&self) -> bool {
        let lim = usize::from(self.state_index_lim);
        for row in 0..=lim {
            for col in 0..row {
                let lower = p_get(&self.p, row, col);
                let upper = p_get(&self.p, col, row);
                if abs_ftype(lower - upper) > 0.0 as Ftype {
                    return false;
                }
            }
        }
        true
    }

    /// Seed `P` diagonals, upstream `CovarianceInit`.
    ///
    /// Zeros the matrix, writes the Plane measurement-noise variances onto
    /// the kinematic / bias / mag diagonals, and leaves wind at zero
    /// (`update_sensor_selection` is not here). The quaternion reset
    /// `CovariancePrediction(&rot_vec_var)` path is stubbed as
    /// `P[0..3] = sq(0.1)`.
    pub fn covariance_init(&mut self) {
        self.p = [[0.0 as Ftype; STATE_VECTOR_LEN]; STATE_VECTOR_LEN];
        self.process_noise = [0.0 as Ftype; PROCESS_NOISE_LEN];
        self.hgt_rate = 0.0 as Ftype;
        self.need_mag_body_var_reset = false;
        self.need_earth_var_reset = false;
        self.last_inhibit_mag_states = self.inhibit_mag_states;

        let quat_var = sq(QUAT_INIT_ROT_SIGMA);
        set_diag_range(&mut self.p, 0, 3, quat_var);

        let vel_ne = sq(self.gps_horiz_vel_noise);
        p_set(&mut self.p, 4, 4, vel_ne);
        p_set(&mut self.p, 5, 5, vel_ne);
        p_set(&mut self.p, 6, 6, sq(self.gps_vert_vel_noise));

        let pos_ne = sq(self.gps_horiz_pos_noise);
        p_set(&mut self.p, 7, 7, pos_ne);
        p_set(&mut self.p, 8, 8, pos_ne);
        p_set(&mut self.p, 9, 9, sq(self.baro_alt_noise));

        let gyro_var = sq(radians(GYRO_BIAS_INIT_DPS * self.dt_ekf_avg));
        set_diag_range(&mut self.p, 10, 12, gyro_var);

        let accel_var = sq(ACCEL_BIAS_LIM_SCALER * self.acc_bias_lim * self.dt_ekf_avg);
        set_diag_range(&mut self.p, 13, 15, accel_var);

        let mag_var = sq(self.mag_noise);
        set_diag_range(&mut self.p, 16, 18, mag_var);
        set_diag_range(&mut self.p, 19, 21, mag_var);

        self.update_state_index_lim();
    }

    /// Predict `P`, upstream `NavEKF3_core::CovariancePrediction(nullptr)`.
    ///
    /// Builds `processNoiseVariance`, adds it to `nextP[i][i]` for
    /// `i = 10..=stateIndexLim`, freezes NE position rows when the
    /// horizontal variance already exceeds 100 m, copies `nextP` onto `P`
    /// with enforced symmetry, then [`constrain_variances`].
    pub fn covariance_prediction(&mut self) {
        self.update_dt();
        self.update_hgt_rate();
        self.update_state_index_lim();
        self.reset_mag_variances_if_requested();
        self.process_noise = self.build_process_noise();

        let mut next_p = self.p;
        let lim = usize::from(self.state_index_lim);
        if self.state_index_lim > 9 {
            for i in 10..=lim {
                let q = match self.process_noise.get(i.saturating_sub(10)) {
                    Some(&value) => value,
                    None => 0.0 as Ftype,
                };
                let cur = p_get(&next_p, i, i);
                p_set(&mut next_p, i, i, cur + q);
            }
        }

        let horiz = p_get(&self.p, 7, 7) + p_get(&self.p, 8, 8);
        if horiz > POS_VAR_GROWTH_LIMIT {
            for i in 7..=8 {
                for j in 0..=lim {
                    p_set(&mut next_p, i, j, p_get(&self.p, i, j));
                    p_set(&mut next_p, j, i, p_get(&self.p, j, i));
                }
            }
        }

        self.copy_symmetric(&next_p);
        self.constrain_variances();
    }

    /// Clamp diagonals, upstream `ConstrainVariances`.
    ///
    /// Inactive-state off-diagonal zeroing is the inhibit / truth path
    /// already applied by [`zero_states_var_cov`](Self::zero_states_var_cov);
    /// this method keeps the March-2025 variance floors and ceilings.
    pub fn constrain_variances(&mut self) {
        for i in 0..=3 {
            let v = constrain_value(p_get(&self.p, i, i), 0.0 as Ftype, ATT_VAR_MAX);
            p_set(&mut self.p, i, i, v);
        }
        for i in 4..=5 {
            let v = constrain_value(p_get(&self.p, i, i), VEL_STATE_MIN_VARIANCE, VEL_NE_VAR_MAX);
            p_set(&mut self.p, i, i, v);
        }
        if p_get(&self.p, 6, 6) < VEL_STATE_MIN_VARIANCE {
            p_set(&mut self.p, 6, 6, VEL_STATE_MIN_VARIANCE);
        }
        for i in 7..=9 {
            let v = constrain_value(p_get(&self.p, i, i), POS_STATE_MIN_VARIANCE, POS_VAR_MAX);
            p_set(&mut self.p, i, i, v);
        }

        if self.inhibit_del_ang_bias {
            self.zero_states_var_cov(10, 12);
        } else {
            let gyro_max = sq(GYRO_BIAS_VAR_CLAMP_RATE * self.dt_ekf_avg);
            for i in 10..=12 {
                let v = constrain_value(p_get(&self.p, i, i), 0.0 as Ftype, gyro_max);
                p_set(&mut self.p, i, i, v);
            }
        }

        if self.inhibit_del_vel_bias {
            self.zero_states_var_cov(13, 15);
            for i in 13..=15 {
                let floor = ACCEL_BIAS_MIN_SAFE_VAR * 10.0 as Ftype;
                let cur = p_get(&self.p, i, i);
                if cur < floor {
                    p_set(&mut self.p, i, i, floor);
                }
            }
        } else {
            let mut max_state_var = 0.0 as Ftype;
            for i in 13..=15 {
                let v = p_get(&self.p, i, i);
                if v > max_state_var {
                    max_state_var = v;
                }
            }
            let min_allowed = max_ftype(0.01 as Ftype * max_state_var, ACCEL_BIAS_MIN_SAFE_VAR);
            let accel_max = sq(ACCEL_BIAS_VAR_CLAMP_RATE * self.dt_ekf_avg);
            for i in 13..=15 {
                let v = constrain_value(p_get(&self.p, i, i), min_allowed, accel_max);
                p_set(&mut self.p, i, i, v);
            }
        }

        if self.inhibit_mag_states {
            self.zero_states_var_cov(16, 21);
        } else {
            for i in 16..=21 {
                let v = constrain_value(p_get(&self.p, i, i), 0.0 as Ftype, MAG_VAR_MAX);
                p_set(&mut self.p, i, i, v);
            }
        }

        if self.inhibit_wind_states || self.treat_wind_as_truth {
            self.zero_states_var_cov(22, 23);
        } else {
            for i in 22..=23 {
                let v = constrain_value(p_get(&self.p, i, i), 0.0 as Ftype, WIND_VEL_VARIANCE_MAX);
                p_set(&mut self.p, i, i, v);
            }
        }
    }

    /// Zero a closed interval of rows and columns, upstream `zeroStatesVarCov`.
    pub fn zero_states_var_cov(&mut self, first: usize, last: usize) {
        let last = if last >= STATE_VECTOR_LEN {
            STATE_VECTOR_LEN.saturating_sub(1)
        } else {
            last
        };
        if first > last {
            return;
        }
        for i in first..=last {
            for j in 0..STATE_VECTOR_LEN {
                p_set(&mut self.p, i, j, 0.0 as Ftype);
                p_set(&mut self.p, j, i, 0.0 as Ftype);
            }
        }
    }

    fn update_dt(&mut self) {
        let raw = 0.5 as Ftype * (self.del_ang_dt + self.del_vel_dt);
        self.dt = constrain_value(
            raw,
            0.5 as Ftype * self.dt_ekf_avg,
            2.0 as Ftype * self.dt_ekf_avg,
        );
    }

    fn update_hgt_rate(&mut self) {
        let alpha = 0.1 as Ftype * self.dt;
        self.hgt_rate = self.hgt_rate * (1.0 as Ftype - alpha) - self.vel_d * alpha;
    }

    fn update_state_index_lim(&mut self) {
        // Upstream `updateStateIndexLim`.
        self.state_index_lim = if self.inhibit_wind_states {
            if self.inhibit_mag_states {
                if self.inhibit_del_vel_bias {
                    if self.inhibit_del_ang_bias {
                        9
                    } else {
                        12
                    }
                } else {
                    15
                }
            } else {
                21
            }
        } else {
            23
        };
    }

    fn reset_mag_variances_if_requested(&mut self) {
        if !self.inhibit_mag_states && self.last_inhibit_mag_states {
            self.need_mag_body_var_reset = true;
            self.need_earth_var_reset = true;
        }
        let mag_var = sq(self.mag_noise);
        if self.need_mag_body_var_reset {
            self.need_mag_body_var_reset = false;
            self.zero_states_var_cov(19, 21);
            set_diag_range(&mut self.p, 19, 21, mag_var);
        }
        if self.need_earth_var_reset {
            self.need_earth_var_reset = false;
            self.zero_states_var_cov(16, 18);
            set_diag_range(&mut self.p, 16, 18, mag_var);
        }
        self.last_inhibit_mag_states = self.inhibit_mag_states;
    }

    fn build_process_noise(&self) -> ProcessNoise {
        let mut q = [0.0 as Ftype; PROCESS_NOISE_LEN];
        if !self.inhibit_del_ang_bias {
            let d_ang = sq(sq(self.dt)
                * constrain_value(self.gyro_bias_process_noise, 0.0 as Ftype, 1.0 as Ftype));
            write_q_range(&mut q, 0, 2, d_ang);
        }
        if !self.inhibit_del_vel_bias {
            let d_vel = sq(sq(self.dt)
                * constrain_value(self.accel_bias_process_noise, 0.0 as Ftype, 1.0 as Ftype));
            write_q_range(&mut q, 3, 5, d_vel);
        }
        if !self.inhibit_mag_states {
            let earth =
                sq(self.dt
                    * constrain_value(self.mag_earth_process_noise, 0.0 as Ftype, 1.0 as Ftype));
            let body =
                sq(self.dt
                    * constrain_value(self.mag_body_process_noise, 0.0 as Ftype, 1.0 as Ftype));
            write_q_range(&mut q, 6, 8, earth);
            write_q_range(&mut q, 9, 11, body);
        }
        if !self.inhibit_wind_states && !self.treat_wind_as_truth {
            let scale = 1.0 as Ftype
                + constrain_value(self.wnd_var_hgt_rate_scale, 0.0 as Ftype, 1.0 as Ftype)
                    * abs_ftype(self.hgt_rate);
            let mut wind = sq(self.dt
                * constrain_value(self.wind_vel_process_noise, 0.0 as Ftype, 1.0 as Ftype)
                * scale);
            if !self.tas_allow_fusion {
                wind *= 10.0 as Ftype;
            }
            write_q_range(&mut q, 12, 13, wind);
        }
        q
    }

    fn copy_symmetric(&mut self, next_p: &CovMatrix) {
        let lim = usize::from(self.state_index_lim);
        for row in 0..=lim {
            p_set(&mut self.p, row, row, p_get(next_p, row, row));
            for col in 0..row {
                let upper = p_get(next_p, col, row);
                p_set(&mut self.p, row, col, upper);
                p_set(&mut self.p, col, row, upper);
            }
        }
    }
}

fn p_get(p: &CovMatrix, row: usize, col: usize) -> Ftype {
    match p.get(row).and_then(|r| r.get(col)) {
        Some(&value) => value,
        None => 0.0 as Ftype,
    }
}

fn p_set(p: &mut CovMatrix, row: usize, col: usize, value: Ftype) {
    if let Some(slot) = p.get_mut(row).and_then(|r| r.get_mut(col)) {
        *slot = value;
    }
}

fn set_diag_range(p: &mut CovMatrix, first: usize, last: usize, value: Ftype) {
    for i in first..=last {
        p_set(p, i, i, value);
    }
}

fn write_q_range(q: &mut ProcessNoise, first: usize, last: usize, value: Ftype) {
    for i in first..=last {
        if let Some(slot) = q.get_mut(i) {
            *slot = value;
        }
    }
}

fn abs_ftype(v: Ftype) -> Ftype {
    if v < 0.0 as Ftype {
        -v
    } else {
        v
    }
}

fn max_ftype(a: Ftype, b: Ftype) -> Ftype {
    if a > b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Ftype, b: Ftype) {
        let err = abs_ftype(a - b);
        assert!(err < 1.0e-12 as Ftype, "{a} !~= {b}");
    }

    fn d_ang_bias_var(dt: Ftype, noise: Ftype) -> Ftype {
        sq(sq(dt) * constrain_value(noise, 0.0 as Ftype, 1.0 as Ftype))
    }

    #[test]
    fn covariance_init_seeds_plane_diagonals() {
        let mut cov = Covariance::new();
        cov.covariance_init();

        near(cov.variance(StateIndex::Quat0), sq(QUAT_INIT_ROT_SIGMA));
        near(cov.variance(StateIndex::VelN), sq(VELNE_M_NSE_DEFAULT));
        near(cov.variance(StateIndex::VelD), sq(VELD_M_NSE_DEFAULT));
        near(cov.variance(StateIndex::PosN), sq(POSNE_M_NSE_DEFAULT));
        near(cov.variance(StateIndex::PosD), sq(ALT_M_NSE_DEFAULT));
        let gyro = sq(radians(GYRO_BIAS_INIT_DPS * EKF_TARGET_DT));
        near(cov.variance(StateIndex::GyroBiasX), gyro);
        let accel = sq(ACCEL_BIAS_LIM_SCALER * ACC_BIAS_LIM_DEFAULT * EKF_TARGET_DT);
        near(cov.variance(StateIndex::AccelBiasX), accel);
        near(cov.variance(StateIndex::EarthMagN), sq(MAG_M_NSE_DEFAULT));
        near(cov.variance(StateIndex::WindVelN), 0.0 as Ftype);
        assert_eq!(cov.state_index_lim(), 23);
        assert!(cov.is_symmetric());
    }

    #[test]
    fn predict_injects_process_noise_on_bias_diagonals() {
        let mut cov = Covariance::new();
        cov.covariance_init();
        let before_gyro = cov.variance(StateIndex::GyroBiasX);
        let before_accel = cov.variance(StateIndex::AccelBiasX);
        let before_earth = cov.variance(StateIndex::EarthMagN);
        let before_wind = cov.variance(StateIndex::WindVelN);

        cov.covariance_prediction();

        let q = cov.process_noise();
        let gyro_q = match q.get(0) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        let accel_q = match q.get(3) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        let earth_q = match q.get(6) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        let wind_q = match q.get(12) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        near(gyro_q, d_ang_bias_var(EKF_TARGET_DT, GBIAS_P_NSE_DEFAULT));
        assert!(gyro_q > 0.0 as Ftype);
        near(cov.variance(StateIndex::GyroBiasX), before_gyro + gyro_q);
        near(cov.variance(StateIndex::GyroBiasY), before_gyro + gyro_q);
        near(cov.variance(StateIndex::GyroBiasZ), before_gyro + gyro_q);
        near(cov.variance(StateIndex::AccelBiasX), before_accel + accel_q);
        near(cov.variance(StateIndex::EarthMagN), before_earth + earth_q);
        near(cov.variance(StateIndex::WindVelN), before_wind + wind_q);
        // Kinematic states have no Q in this stub (Jacobian is later).
        near(cov.variance(StateIndex::VelN), sq(VELNE_M_NSE_DEFAULT));
        near(cov.dt(), EKF_TARGET_DT);
    }

    #[test]
    fn predict_restores_p_symmetry_from_nextp_upper() {
        let mut cov = Covariance::new();
        cov.covariance_init();
        // Asymmetric off-diagonal: upper (col < row in the copy-back
        // source `nextP[col][row]`) wins.
        cov.set_p(10, 12, 0.001 as Ftype);
        cov.set_p(12, 10, 0.002 as Ftype);
        cov.set_p(16, 18, -0.004 as Ftype);
        cov.set_p(18, 16, 0.007 as Ftype);

        cov.covariance_prediction();

        near(cov.p(10, 12), 0.001 as Ftype);
        near(cov.p(12, 10), 0.001 as Ftype);
        near(cov.p(16, 18), -0.004 as Ftype);
        near(cov.p(18, 16), -0.004 as Ftype);
        assert!(cov.is_symmetric());
    }

    #[test]
    fn inhibit_skips_matching_process_noise() {
        let mut cov = Covariance::new();
        cov.covariance_init();
        cov.set_inhibit_del_ang_bias(true);
        cov.set_inhibit_wind_states(true);
        assert_eq!(cov.state_index_lim(), 21);

        let before_gyro = cov.variance(StateIndex::GyroBiasX);
        cov.covariance_prediction();

        let q = cov.process_noise();
        near(
            match q.get(0) {
                Some(&v) => v,
                None => 1.0 as Ftype,
            },
            0.0 as Ftype,
        );
        near(
            match q.get(12) {
                Some(&v) => v,
                None => 1.0 as Ftype,
            },
            0.0 as Ftype,
        );
        // Gyro Q is zero, but ConstrainVariances zeros the whole 10..12
        // block when the states are inhibited.
        near(cov.variance(StateIndex::GyroBiasX), 0.0 as Ftype);
        let _ = before_gyro;
        assert_eq!(cov.state_index_lim(), 21);
    }

    #[test]
    fn wind_q_grows_with_height_rate_and_failed_tas() {
        let mut level = Covariance::new();
        level.covariance_init();
        level.covariance_prediction();
        let q_level = match level.process_noise().get(12) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };

        let mut climb = Covariance::new();
        climb.covariance_init();
        climb.set_velocity_d(-(10.0 as Ftype));
        climb.covariance_prediction();
        let q_climb = match climb.process_noise().get(12) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        assert!(q_climb > q_level);
        assert!(abs_ftype(climb.hgt_rate()) > 0.0 as Ftype);

        let mut no_tas = Covariance::new();
        no_tas.covariance_init();
        no_tas.set_tas_allow_fusion(false);
        no_tas.covariance_prediction();
        let q_no_tas = match no_tas.process_noise().get(12) {
            Some(&v) => v,
            None => 0.0 as Ftype,
        };
        near(q_no_tas, q_level * 10.0 as Ftype);
    }

    #[test]
    fn horiz_pos_variance_freeze_keeps_ne_rows() {
        let mut cov = Covariance::new();
        cov.covariance_init();
        cov.set_p(7, 7, 6000.0 as Ftype);
        cov.set_p(8, 8, 6000.0 as Ftype);
        cov.set_p(7, 10, 0.5 as Ftype);
        cov.set_p(10, 7, 0.5 as Ftype);
        cov.covariance_prediction();
        // Sum is 1.2e4 > 1e4, so NE rows are restored from P (no Q on
        // those states anyway). ConstrainVariances still applies the
        // 1e6 ceiling, which these values are under.
        near(cov.variance(StateIndex::PosN), 6000.0 as Ftype);
        near(cov.variance(StateIndex::PosE), 6000.0 as Ftype);
        near(cov.p(7, 10), 0.5 as Ftype);
        near(cov.p(10, 7), 0.5 as Ftype);
    }

    #[test]
    fn constrain_clamps_attitude_and_gyro_bias_variance() {
        let mut cov = Covariance::new();
        cov.covariance_init();
        cov.set_p(0, 0, 4.0 as Ftype);
        cov.set_p(10, 10, 1.0 as Ftype);
        cov.constrain_variances();
        near(cov.variance(StateIndex::Quat0), ATT_VAR_MAX);
        let gyro_max = sq(GYRO_BIAS_VAR_CLAMP_RATE * EKF_TARGET_DT);
        near(cov.variance(StateIndex::GyroBiasX), gyro_max);
    }

    #[test]
    fn dt_is_constrained_to_half_to_double_ekf_avg() {
        let mut cov = Covariance::new();
        cov.set_imu_dt(0.0 as Ftype, 0.0 as Ftype);
        cov.covariance_prediction();
        near(cov.dt(), 0.5 as Ftype * EKF_TARGET_DT);

        cov.set_imu_dt(1.0 as Ftype, 1.0 as Ftype);
        cov.covariance_prediction();
        near(cov.dt(), 2.0 as Ftype * EKF_TARGET_DT);
    }
}
