//! True-airspeed fusion enable / innovation gate, upstream
//! `AP_NavEKF3_AirDataFusion.cpp`.
//!
//! This slice is the gate that `SelectTasFusion` evaluates before
//! `FuseAirspeed`. The algebraic Kalman update that consumes TAS and
//! writes wind / velocity is not here.
//!
//! # Enable gate
//!
//! Upstream starts a TAS cycle only when a delayed sample is at the
//! fusion horizon (`tasDataToFuse`), bootstrap has latched
//! (`statesInitialised`), and wind states are active
//! (`!inhibitWindStates`). A 200 Hz magnetometer step
//! (`magFusePerformed && dtIMUavg < 0.005`) delays the selector by one
//! IMU frame (`airSpdFusionDelayed`).
//!
//! # Innovation gate
//!
//! `FuseAirspeed` predicts TAS from NED velocity minus wind
//! (`norm(ve-vwe, vn-vwn, vd)`) and refuses the sample when that
//! prediction is at or below 1 m/s. Otherwise it forms
//! `tasTestRatio = sq(innov) / (sq(MAX(0.01 * tasInnovGate, 1)) * varInnov)`
//! and fuses when `allowFusion` and the ratio is below 1 (or `badIMUdata`),
//! or when both TAS and position have timed out
//! (`tasTimeout && posTimeout`).

use ap_math::scalar::{norm3, sq};
use ap_math::Ftype;

use crate::measurements::EKF_TARGET_DT;
use crate::{StateIndex, StateVector};

/// 200 Hz: `dtIMUavg < 0.005` skips TAS when mag already fused.
const MAG_DELAY_DT_S: Ftype = 0.005;

/// Predicted TAS must exceed this (m/s) before `FuseAirspeed` runs.
const MIN_VTAS_PRED: Ftype = 1.0;

/// Default `EK3_EAS_I_GATE` (`_tasInnovGate`).
pub const TAS_INNOV_GATE_DEFAULT: i16 = 400;

/// Upstream `tasRetryTime_ms`: TAS timeout / retry interval.
pub const TAS_RETRY_TIME_MS: u32 = 5000;

/// TAS fusion selection after [`AirDataFusion::select_tas_fusion`].
///
/// Discriminant values are local to the port so a sitl-diff dump can
/// compare the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TasFuseSel {
    /// Selector did not call `FuseAirspeed` this step.
    NotFusing = 0,
    /// Enable gate opened; `FuseAirspeed` ran (gate may still reject).
    FuseTas = 1,
}

/// Air-data / TAS fusion latch, the `NavEKF3_core` fields
/// `SelectTasFusion` and `FuseAirspeed` read and write.
///
/// Covariance and the TAS Jacobians are not here: tests (and later
/// cores) poke the sample / wind-inhibit / innovation flags that the
/// selector would have read from DAL and the covariance diagonals.
#[derive(Debug, Clone)]
pub struct AirDataFusion {
    tas_data_to_fuse: bool,
    inhibit_wind_states: bool,
    allow_fusion: bool,
    mag_fuse_performed: bool,
    dt_imu_avg: Ftype,
    air_spd_fusion_delayed: bool,
    tas_meas: Ftype,
    vel_n: Ftype,
    vel_e: Ftype,
    vel_d: Ftype,
    wind_n: Ftype,
    wind_e: Ftype,
    vtas_pred: Ftype,
    innov_vtas: Ftype,
    var_innov_vtas: Ftype,
    tas_test_ratio: Ftype,
    tas_innov_gate: i16,
    bad_imu_data: bool,
    bad_airspeed: bool,
    tas_timeout: bool,
    pos_timeout: bool,
    imu_sample_time_ms: u32,
    last_tas_pass_time_ms: u32,
    last_tas_fail_time_ms: u32,
    prev_tas_step_ms: u32,
    fuse_performed: bool,
    fuse_sel: TasFuseSel,
}

impl Default for AirDataFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl AirDataFusion {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// Wind states inhibited, TAS timed out, no delayed sample,
    /// `dtIMUavg = EKF_TARGET_DT`, `EK3_EAS_I_GATE = 400`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tas_data_to_fuse: false,
            inhibit_wind_states: true,
            allow_fusion: false,
            mag_fuse_performed: false,
            dt_imu_avg: EKF_TARGET_DT,
            air_spd_fusion_delayed: false,
            tas_meas: 0.0 as Ftype,
            vel_n: 0.0 as Ftype,
            vel_e: 0.0 as Ftype,
            vel_d: 0.0 as Ftype,
            wind_n: 0.0 as Ftype,
            wind_e: 0.0 as Ftype,
            vtas_pred: 0.0 as Ftype,
            innov_vtas: 0.0 as Ftype,
            var_innov_vtas: 1.0 as Ftype,
            tas_test_ratio: 0.0 as Ftype,
            tas_innov_gate: TAS_INNOV_GATE_DEFAULT,
            bad_imu_data: false,
            bad_airspeed: false,
            tas_timeout: true,
            pos_timeout: false,
            imu_sample_time_ms: 0,
            last_tas_pass_time_ms: 0,
            last_tas_fail_time_ms: 0,
            prev_tas_step_ms: 0,
            fuse_performed: false,
            fuse_sel: TasFuseSel::NotFusing,
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Delayed TAS sample is at the fusion horizon, upstream `tasDataToFuse`.
    #[must_use]
    pub const fn tas_data_to_fuse(&self) -> bool {
        self.tas_data_to_fuse
    }

    /// Wind states are inactive, upstream `inhibitWindStates`.
    #[must_use]
    pub const fn inhibit_wind_states(&self) -> bool {
        self.inhibit_wind_states
    }

    /// Measurement may modify states, upstream `tasDataDelayed.allowFusion`.
    #[must_use]
    pub const fn allow_fusion(&self) -> bool {
        self.allow_fusion
    }

    /// Mag-step delay is holding TAS off this IMU frame.
    #[must_use]
    pub const fn air_spd_fusion_delayed(&self) -> bool {
        self.air_spd_fusion_delayed
    }

    /// Predicted TAS (m/s), upstream `VtasPred`.
    #[must_use]
    pub const fn vtas_pred(&self) -> Ftype {
        self.vtas_pred
    }

    /// TAS innovation (m/s), upstream `innovVtas`.
    #[must_use]
    pub const fn innov_vtas(&self) -> Ftype {
        self.innov_vtas
    }

    /// Innovation consistency ratio, upstream `tasTestRatio`.
    #[must_use]
    pub const fn tas_test_ratio(&self) -> Ftype {
        self.tas_test_ratio
    }

    /// TAS measurements have timed out, upstream `tasTimeout`.
    #[must_use]
    pub const fn tas_timeout(&self) -> bool {
        self.tas_timeout
    }

    /// Badly-conditioned observation variance, upstream `faultStatus.bad_airspeed`.
    #[must_use]
    pub const fn bad_airspeed(&self) -> bool {
        self.bad_airspeed
    }

    /// Kalman TAS update ran this step (enable-side stub).
    #[must_use]
    pub const fn fuse_performed(&self) -> bool {
        self.fuse_performed
    }

    /// Combined selection after the enable gate.
    #[must_use]
    pub const fn fuse_sel(&self) -> TasFuseSel {
        self.fuse_sel
    }

    /// Last successful TAS pass time (ms), upstream `lastTasPassTime_ms`.
    #[must_use]
    pub const fn last_tas_pass_time_ms(&self) -> u32 {
        self.last_tas_pass_time_ms
    }

    /// Last TAS fusion-step time (ms), upstream `prevTasStep_ms`.
    #[must_use]
    pub const fn prev_tas_step_ms(&self) -> u32 {
        self.prev_tas_step_ms
    }

    /// Poke `tasDataToFuse`.
    pub fn set_tas_data_to_fuse(&mut self, ready: bool) {
        self.tas_data_to_fuse = ready;
    }

    /// Poke `inhibitWindStates`.
    pub fn set_inhibit_wind_states(&mut self, inhibit: bool) {
        self.inhibit_wind_states = inhibit;
    }

    /// Poke `tasDataDelayed.allowFusion`.
    pub fn set_allow_fusion(&mut self, allow: bool) {
        self.allow_fusion = allow;
    }

    /// Poke delayed TAS (m/s) and `allowFusion`.
    pub fn set_tas_data(&mut self, ready: bool, tas_mps: Ftype, allow_fusion: bool) {
        self.tas_data_to_fuse = ready;
        self.tas_meas = tas_mps;
        self.allow_fusion = allow_fusion;
    }

    /// Poke NED velocity used by `VtasPred`.
    pub fn set_velocity(&mut self, north: Ftype, east: Ftype, down: Ftype) {
        self.vel_n = north;
        self.vel_e = east;
        self.vel_d = down;
    }

    /// Poke NE wind used by `VtasPred`.
    pub fn set_wind(&mut self, north: Ftype, east: Ftype) {
        self.wind_n = north;
        self.wind_e = east;
    }

    /// Poke the stub innovation variance (no `P` matrix in this slice).
    pub fn set_var_innov_vtas(&mut self, var: Ftype) {
        self.var_innov_vtas = var;
    }

    /// Poke `EK3_EAS_I_GATE`.
    pub fn set_tas_innov_gate(&mut self, gate: i16) {
        self.tas_innov_gate = gate;
    }

    /// Poke `magFusePerformed` and `dtIMUavg` for the 200 Hz delay.
    pub fn set_mag_fuse_timing(&mut self, mag_fuse_performed: bool, dt_imu_avg: Ftype) {
        self.mag_fuse_performed = mag_fuse_performed;
        self.dt_imu_avg = dt_imu_avg;
    }

    /// Poke `badIMUdata` / `posTimeout` for the innovation override.
    pub fn set_quality_overrides(&mut self, bad_imu_data: bool, pos_timeout: bool) {
        self.bad_imu_data = bad_imu_data;
        self.pos_timeout = pos_timeout;
    }

    /// Poke `imuSampleTime_ms`.
    pub fn set_imu_sample_time_ms(&mut self, time_ms: u32) {
        self.imu_sample_time_ms = time_ms;
    }

    /// Copy velocity / wind from the 24-vector, the slots `FuseAirspeed` reads.
    pub fn read_vel_wind_from_states(&mut self, states: &StateVector) {
        self.vel_n = get_state(states, StateIndex::VelN);
        self.vel_e = get_state(states, StateIndex::VelE);
        self.vel_d = get_state(states, StateIndex::VelD);
        self.wind_n = get_state(states, StateIndex::WindVelN);
        self.wind_e = get_state(states, StateIndex::WindVelE);
    }

    /// Enable half of `SelectTasFusion`: sample, bootstrap, wind not inhibited.
    #[must_use]
    pub const fn tas_enable_ok(&self, states_initialised: bool) -> bool {
        self.tas_data_to_fuse && states_initialised && !self.inhibit_wind_states
    }

    /// Innovation consistency, upstream `tasTestRatio < 1 || badIMUdata`.
    #[must_use]
    pub fn innovation_consistent(&self) -> bool {
        self.tas_test_ratio < (1.0 as Ftype) || self.bad_imu_data
    }

    /// Upstream `NavEKF3_core::SelectTasFusion` enable gate.
    ///
    /// Applies the 200 Hz mag delay, then `FuseAirspeed` when
    /// [`tas_enable_ok`](Self::tas_enable_ok). Jacobians are not here.
    pub fn select_tas_fusion(&mut self, states_initialised: bool) {
        self.fuse_performed = false;
        self.fuse_sel = TasFuseSel::NotFusing;

        if self.mag_fuse_performed
            && self.dt_imu_avg < MAG_DELAY_DT_S
            && !self.air_spd_fusion_delayed
        {
            self.air_spd_fusion_delayed = true;
            return;
        }
        self.air_spd_fusion_delayed = false;

        if !self.tas_enable_ok(states_initialised) {
            return;
        }

        self.fuse_sel = TasFuseSel::FuseTas;
        self.fuse_airspeed();
        self.tas_data_to_fuse = false;
        self.prev_tas_step_ms = self.imu_sample_time_ms;
    }

    /// Upstream `NavEKF3_core::FuseAirspeed` enable-side stub.
    ///
    /// Predicts TAS, forms `tasTestRatio`, and sets [`fuse_performed`]
    /// when the innovation gate accepts. The Kalman gain and covariance
    /// update are not here.
    pub fn fuse_airspeed(&mut self) {
        self.fuse_performed = false;
        self.bad_airspeed = false;
        self.vtas_pred = norm3(
            self.vel_e - self.wind_e,
            self.vel_n - self.wind_n,
            self.vel_d,
        );
        if self.vtas_pred <= MIN_VTAS_PRED {
            return;
        }

        self.innov_vtas = self.vtas_pred - self.tas_meas;
        if self.var_innov_vtas <= (0.0 as Ftype) {
            self.bad_airspeed = true;
            return;
        }

        let scaled = (0.01 as Ftype) * (self.tas_innov_gate as Ftype);
        let gate = if scaled > (1.0 as Ftype) {
            scaled
        } else {
            1.0 as Ftype
        };
        self.tas_test_ratio = sq(self.innov_vtas) / (sq(gate) * self.var_innov_vtas);

        let is_consistent = self.innovation_consistent();
        self.tas_timeout = self
            .imu_sample_time_ms
            .wrapping_sub(self.last_tas_pass_time_ms)
            > TAS_RETRY_TIME_MS;
        if is_consistent {
            self.last_tas_fail_time_ms = 0;
        } else {
            self.last_tas_fail_time_ms = self.imu_sample_time_ms;
        }

        if self.allow_fusion && (is_consistent || (self.tas_timeout && self.pos_timeout)) {
            self.last_tas_pass_time_ms = self.imu_sample_time_ms;
            self.fuse_performed = true;
        }
    }
}

fn get_state(states: &StateVector, index: StateIndex) -> Ftype {
    match states.get(index.as_usize()) {
        Some(&value) => value,
        None => 0.0 as Ftype,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_tas(ad: &mut AirDataFusion) {
        ad.set_inhibit_wind_states(false);
        ad.set_tas_data(true, 20.0 as Ftype, true);
        ad.set_velocity(20.0 as Ftype, 0.0 as Ftype, 0.0 as Ftype);
        ad.set_wind(0.0 as Ftype, 0.0 as Ftype);
        ad.set_var_innov_vtas(1.0 as Ftype);
        ad.set_imu_sample_time_ms(1000);
    }

    #[test]
    fn enable_gate_refuses_until_sample_init_and_wind() {
        let mut ad = AirDataFusion::new();
        ad.set_tas_data(true, 20.0 as Ftype, true);
        ad.set_velocity(20.0 as Ftype, 0.0 as Ftype, 0.0 as Ftype);
        ad.select_tas_fusion(true);
        // Wind states start inhibited (`InitialiseVariables`).
        assert!(ad.inhibit_wind_states());
        assert_eq!(ad.fuse_sel(), TasFuseSel::NotFusing);
        assert!(!ad.fuse_performed());

        ad.set_inhibit_wind_states(false);
        ad.select_tas_fusion(false);
        assert_eq!(ad.fuse_sel(), TasFuseSel::NotFusing);
        assert!(!ad.fuse_performed());

        ad.select_tas_fusion(true);
        assert_eq!(ad.fuse_sel(), TasFuseSel::FuseTas);
        assert!(ad.fuse_performed());
        assert!(!ad.tas_data_to_fuse());
        assert_eq!(ad.prev_tas_step_ms(), 0);
    }

    #[test]
    fn innovation_gate_refuses_large_innov_unless_timeout() {
        let mut ad = AirDataFusion::new();
        ready_tas(&mut ad);
        // Predicted 20 m/s vs measured 4 m/s: innov = 16, gate sigma = 4,
        // var = 1 → ratio = 256 / 16 = 16 > 1.
        ad.set_tas_data(true, 4.0 as Ftype, true);
        ad.select_tas_fusion(true);
        assert_eq!(ad.fuse_sel(), TasFuseSel::FuseTas);
        assert!(!ad.fuse_performed());
        assert!(ad.tas_test_ratio() > (1.0 as Ftype));
        assert!(!ad.innovation_consistent());

        // Timeout override needs both TAS and position timed out.
        ad.set_tas_data(true, 4.0 as Ftype, true);
        ad.set_imu_sample_time_ms(TAS_RETRY_TIME_MS + 1);
        ad.set_quality_overrides(false, false);
        ad.select_tas_fusion(true);
        assert!(!ad.fuse_performed());

        ad.set_tas_data(true, 4.0 as Ftype, true);
        ad.set_quality_overrides(false, true);
        ad.select_tas_fusion(true);
        assert!(ad.tas_timeout());
        assert!(ad.fuse_performed());
        assert_eq!(ad.last_tas_pass_time_ms(), TAS_RETRY_TIME_MS + 1);
    }

    #[test]
    fn innovation_gate_accepts_when_ratio_below_one_or_bad_imu() {
        let mut ad = AirDataFusion::new();
        ready_tas(&mut ad);
        ad.select_tas_fusion(true);
        assert_eq!(ad.fuse_sel(), TasFuseSel::FuseTas);
        assert!(ad.fuse_performed());
        assert!(ad.tas_test_ratio() < (1.0 as Ftype));
        let innov = ad.innov_vtas();
        assert!(innov * innov < (1.0e-12 as Ftype));

        let mut bad = AirDataFusion::new();
        ready_tas(&mut bad);
        bad.set_tas_data(true, 4.0 as Ftype, true);
        bad.set_quality_overrides(true, false);
        bad.select_tas_fusion(true);
        assert!(bad.tas_test_ratio() > (1.0 as Ftype));
        assert!(bad.innovation_consistent());
        assert!(bad.fuse_performed());
    }

    #[test]
    fn predicted_tas_below_one_skips_fusion() {
        let mut ad = AirDataFusion::new();
        ready_tas(&mut ad);
        ad.set_velocity(0.4 as Ftype, 0.0 as Ftype, 0.0 as Ftype);
        ad.select_tas_fusion(true);
        assert_eq!(ad.fuse_sel(), TasFuseSel::FuseTas);
        assert!(!ad.fuse_performed());
        assert!(ad.vtas_pred() < MIN_VTAS_PRED);
        assert!(!ad.tas_data_to_fuse());
    }

    #[test]
    fn mag_fuse_at_high_rate_delays_once() {
        let mut ad = AirDataFusion::new();
        ready_tas(&mut ad);
        ad.set_mag_fuse_timing(true, 0.004 as Ftype);
        ad.select_tas_fusion(true);
        assert!(ad.air_spd_fusion_delayed());
        assert_eq!(ad.fuse_sel(), TasFuseSel::NotFusing);
        assert!(!ad.fuse_performed());
        assert!(ad.tas_data_to_fuse());

        ad.select_tas_fusion(true);
        assert!(!ad.air_spd_fusion_delayed());
        assert_eq!(ad.fuse_sel(), TasFuseSel::FuseTas);
        assert!(ad.fuse_performed());
    }

    #[test]
    fn read_vel_wind_from_states_feeds_prediction() {
        let mut ad = AirDataFusion::new();
        let mut states = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        if let Some(slot) = states.get_mut(StateIndex::VelN.as_usize()) {
            *slot = 15.0 as Ftype;
        }
        if let Some(slot) = states.get_mut(StateIndex::WindVelN.as_usize()) {
            *slot = 3.0 as Ftype;
        }
        ad.read_vel_wind_from_states(&states);
        ad.set_inhibit_wind_states(false);
        ad.set_tas_data(true, 12.0 as Ftype, true);
        ad.select_tas_fusion(true);
        let pred = ad.vtas_pred() - (12.0 as Ftype);
        assert!(pred * pred < (1.0e-12 as Ftype));
        assert!(ad.fuse_performed());
    }
}
