//! Range-finder / optical-flow fusion enable / quality gates, upstream
//! `AP_NavEKF3_OptFlowFusion.cpp` (`SelectFlowFusion`,
//! `EstimateTerrainOffset`, `FuseOptFlow`).
//!
//! This slice is the gate that `SelectFlowFusion` evaluates before the
//! 1-state terrain EKF or the sequential LOS Kalman update. The
//! Jacobians that write `terrainState` / velocity are not here.
//!
//! # Range-finder enable (`SelectRngFusion`)
//!
//! Terrain-offset rangefinder starts only when a delayed sample is at
//! the fusion horizon (`rangeDataToFuse`), tilt is acceptable
//! (`prevTnb.c.z > DCM33FlowMin`), bootstrap has latched, and the
//! main-filter height source is *not* RANGEFINDER (that path already
//! owns height; `EstimateTerrainOffset` then sets `inhibitGndState`).
//!
//! # Optical-flow enable (`SelectFlowFusion`)
//!
//! Main-filter flow starts only when a delayed sample is at the
//! fusion horizon (`flowDataToFuse`), tilt is OK, `EK3_FLOW_USE` is
//! NAV, and the XY source list includes OPTFLOW. Plane defaults
//! `FLOW_USE` to TERRAIN, so the main-filter path stays closed until
//! a test (or later param) selects NAV. A 200 Hz magnetometer step
//! (`magFusePerformed && dtIMUavg < 0.005`) delays the selector by
//! one IMU frame (`optFlowFusionDelayed`).
//!
//! # Quality gates
//!
//! Flow: `flowTestRatio = sq(innov) / (sq(MAX(0.01 * flowInnovGate, 1))
//! * varInnov)` and both body rates below `EK3_FLOW_MAX`. Range:
//! the same ratio on `auxRngTestRatio` / `EK3_RNG_I_GATE`. Freshness
//! windows are 1 s (`flowDataValid`) and 5 s (`gndOffsetValid`).

use ap_math::scalar::sq;
use ap_math::Ftype;

use crate::measurements::EKF_TARGET_DT;

/// 200 Hz: `dtIMUavg < 0.005` skips flow when mag already fused.
const MAG_DELAY_DT_S: Ftype = 0.005;

/// Upstream `DCM33FlowMin`: `Tbn(3,3)` must exceed this to fuse.
pub const DCM33_FLOW_MIN: Ftype = 0.71;

/// Plane `EK3_FLOW_I_GATE` (`_flowInnovGate`).
pub const FLOW_INNOV_GATE_DEFAULT: i16 = 500;

/// Upstream `EK3_RNG_I_GATE` (`_rngInnovGate`).
pub const RNG_INNOV_GATE_DEFAULT: i16 = 500;

/// Upstream `EK3_FLOW_MAX` (`_maxFlowRate`), rad/s.
pub const MAX_FLOW_RATE_DEFAULT: Ftype = 2.5;

/// `flowDataValid` window (ms), upstream `flowValidMeaTime_ms`.
pub const FLOW_VALID_MS: u32 = 1000;

/// `gndOffsetValid` window (ms), upstream `gndHgtValidTime_ms`.
pub const GND_OFFSET_VALID_MS: u32 = 5000;

/// Plane `EK3_FLOW_USE` default, upstream `FLOW_USE_DEFAULT`.
pub const FLOW_USE_DEFAULT: FlowUse = FlowUse::Terrain;

/// `EK3_FLOW_USE`, upstream `FLOW_USE_NONE` / `NAV` / `TERRAIN`.
///
/// Discriminant values match the upstream macros so a sitl-diff dump
/// can compare the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowUse {
    /// Optical flow ignored, upstream `FLOW_USE_NONE`.
    None = 0,
    /// Fuse into the main filter, upstream `FLOW_USE_NAV`.
    Nav = 1,
    /// Terrain-offset estimator only, upstream `FLOW_USE_TERRAIN`.
    Terrain = 2,
}

/// Range-finder fusion selection after [`RngFlowFusion::select_rng_fusion`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RngFuseSel {
    /// Selector did not call the terrain-offset rangefinder path.
    NotFusing = 0,
    /// Enable gate opened; quality gate may still reject.
    FuseRng = 1,
}

/// Optical-flow fusion selection after [`RngFlowFusion::select_flow_fusion`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowFuseSel {
    /// Selector did not call `FuseOptFlow` this step.
    NotFusing = 0,
    /// Enable gate opened; quality gate may still reject.
    FuseFlow = 1,
}

/// Range-finder / optical-flow fusion latch, the `NavEKF3_core`
/// fields `SelectFlowFusion` / `EstimateTerrainOffset` / `FuseOptFlow`
/// read and write.
///
/// Covariance and the LOS / terrain Jacobians are not here: tests
/// (and later cores) poke the delayed-sample / tilt / source flags
/// that the selector would have read from DAL.
#[derive(Debug, Clone)]
pub struct RngFlowFusion {
    range_data_to_fuse: bool,
    flow_data_to_fuse: bool,
    flow_use: FlowUse,
    use_optflow_xy: bool,
    tilt_ok: bool,
    takeoff_detected: bool,
    active_hgt_is_rangefinder: bool,
    mag_fuse_performed: bool,
    dt_imu_avg: Ftype,
    opt_flow_fusion_delayed: bool,
    rng_meas: Ftype,
    innov_rng: Ftype,
    var_innov_rng: Ftype,
    rng_test_ratio: Ftype,
    rng_innov_gate: i16,
    flow_rad_x: Ftype,
    flow_rad_y: Ftype,
    innov_flow: Ftype,
    var_innov_flow: Ftype,
    flow_test_ratio: Ftype,
    flow_innov_gate: i16,
    max_flow_rate: Ftype,
    flow_data_valid: bool,
    gnd_offset_valid: bool,
    imu_sample_time_ms: u32,
    flow_valid_mea_time_ms: u32,
    gnd_hgt_valid_time_ms: u32,
    prev_flow_fuse_time_ms: u32,
    rng_fuse_performed: bool,
    flow_fuse_performed: bool,
    rng_fuse_sel: RngFuseSel,
    flow_fuse_sel: FlowFuseSel,
}

impl Default for RngFlowFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl RngFlowFusion {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// Plane `FLOW_USE = TERRAIN`, no delayed samples, tilt OK,
    /// `EK3_FLOW_I_GATE = 500`, `EK3_RNG_I_GATE = 500`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            range_data_to_fuse: false,
            flow_data_to_fuse: false,
            flow_use: FLOW_USE_DEFAULT,
            use_optflow_xy: false,
            tilt_ok: true,
            takeoff_detected: false,
            active_hgt_is_rangefinder: false,
            mag_fuse_performed: false,
            dt_imu_avg: EKF_TARGET_DT,
            opt_flow_fusion_delayed: false,
            rng_meas: 0.0 as Ftype,
            innov_rng: 0.0 as Ftype,
            var_innov_rng: 1.0 as Ftype,
            rng_test_ratio: 0.0 as Ftype,
            rng_innov_gate: RNG_INNOV_GATE_DEFAULT,
            flow_rad_x: 0.0 as Ftype,
            flow_rad_y: 0.0 as Ftype,
            innov_flow: 0.0 as Ftype,
            var_innov_flow: 1.0 as Ftype,
            flow_test_ratio: 0.0 as Ftype,
            flow_innov_gate: FLOW_INNOV_GATE_DEFAULT,
            max_flow_rate: MAX_FLOW_RATE_DEFAULT,
            flow_data_valid: false,
            gnd_offset_valid: false,
            imu_sample_time_ms: 0,
            flow_valid_mea_time_ms: 0,
            gnd_hgt_valid_time_ms: 0,
            prev_flow_fuse_time_ms: 0,
            rng_fuse_performed: false,
            flow_fuse_performed: false,
            rng_fuse_sel: RngFuseSel::NotFusing,
            flow_fuse_sel: FlowFuseSel::NotFusing,
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Delayed rangefinder sample is at the fusion horizon.
    #[must_use]
    pub const fn range_data_to_fuse(&self) -> bool {
        self.range_data_to_fuse
    }

    /// Delayed optical-flow sample is at the fusion horizon.
    #[must_use]
    pub const fn flow_data_to_fuse(&self) -> bool {
        self.flow_data_to_fuse
    }

    /// `EK3_FLOW_USE`, upstream `_flowUse`.
    #[must_use]
    pub const fn flow_use(&self) -> FlowUse {
        self.flow_use
    }

    /// Flow observations are still fresh, upstream `flowDataValid`.
    #[must_use]
    pub const fn flow_data_valid(&self) -> bool {
        self.flow_data_valid
    }

    /// Terrain-offset estimate is still fresh, upstream `gndOffsetValid`.
    #[must_use]
    pub const fn gnd_offset_valid(&self) -> bool {
        self.gnd_offset_valid
    }

    /// Mag-step delay is holding flow off this IMU frame.
    #[must_use]
    pub const fn opt_flow_fusion_delayed(&self) -> bool {
        self.opt_flow_fusion_delayed
    }

    /// Range innovation (m), upstream `innovRng`.
    #[must_use]
    pub const fn innov_rng(&self) -> Ftype {
        self.innov_rng
    }

    /// Range innovation consistency ratio, upstream `auxRngTestRatio`.
    #[must_use]
    pub const fn rng_test_ratio(&self) -> Ftype {
        self.rng_test_ratio
    }

    /// Flow innovation (rad/s), upstream `flowInnov[0]`.
    #[must_use]
    pub const fn innov_flow(&self) -> Ftype {
        self.innov_flow
    }

    /// Flow innovation consistency ratio, upstream `flowTestRatio`.
    #[must_use]
    pub const fn flow_test_ratio(&self) -> Ftype {
        self.flow_test_ratio
    }

    /// Terrain-offset rangefinder Kalman update ran this step.
    #[must_use]
    pub const fn rng_fuse_performed(&self) -> bool {
        self.rng_fuse_performed
    }

    /// Main-filter optical-flow Kalman update ran this step.
    #[must_use]
    pub const fn flow_fuse_performed(&self) -> bool {
        self.flow_fuse_performed
    }

    /// Combined rangefinder selection after the enable gate.
    #[must_use]
    pub const fn rng_fuse_sel(&self) -> RngFuseSel {
        self.rng_fuse_sel
    }

    /// Combined optical-flow selection after the enable gate.
    #[must_use]
    pub const fn flow_fuse_sel(&self) -> FlowFuseSel {
        self.flow_fuse_sel
    }

    /// Last successful flow pass time (ms), upstream `prevFlowFuseTime_ms`.
    #[must_use]
    pub const fn prev_flow_fuse_time_ms(&self) -> u32 {
        self.prev_flow_fuse_time_ms
    }

    /// Delayed rangefinder measurement (m), upstream `rangeDataDelayed.rng`.
    #[must_use]
    pub const fn rng_meas(&self) -> Ftype {
        self.rng_meas
    }

    /// Poke delayed rangefinder range (m) and `rangeDataToFuse`.
    pub fn set_range_data(&mut self, ready: bool, rng_m: Ftype) {
        self.range_data_to_fuse = ready;
        self.rng_meas = rng_m;
    }

    /// Poke delayed optical-flow rates (rad/s) and `flowDataToFuse`.
    pub fn set_flow_data(&mut self, ready: bool, flow_rad_x: Ftype, flow_rad_y: Ftype) {
        self.flow_data_to_fuse = ready;
        self.flow_rad_x = flow_rad_x;
        self.flow_rad_y = flow_rad_y;
    }

    /// Poke `EK3_FLOW_USE`.
    pub fn set_flow_use(&mut self, flow_use: FlowUse) {
        self.flow_use = flow_use;
    }

    /// Poke `sources.useVelXYSource(OPTFLOW)`.
    pub fn set_use_optflow_xy(&mut self, use_xy: bool) {
        self.use_optflow_xy = use_xy;
    }

    /// Poke the tilt check, upstream `prevTnb.c.z > DCM33FlowMin`.
    pub fn set_tilt_ok(&mut self, tilt_ok: bool) {
        self.tilt_ok = tilt_ok;
    }

    /// Poke `takeOffDetected`.
    pub fn set_takeoff_detected(&mut self, detected: bool) {
        self.takeoff_detected = detected;
    }

    /// Poke `activeHgtSource == RANGEFINDER`.
    pub fn set_active_hgt_is_rangefinder(&mut self, is_rangefinder: bool) {
        self.active_hgt_is_rangefinder = is_rangefinder;
    }

    /// Poke `magFusePerformed` and `dtIMUavg` for the 200 Hz delay.
    pub fn set_mag_fuse_timing(&mut self, mag_fuse_performed: bool, dt_imu_avg: Ftype) {
        self.mag_fuse_performed = mag_fuse_performed;
        self.dt_imu_avg = dt_imu_avg;
    }

    /// Poke the stub range innovation / variance (no `P` in this slice).
    pub fn set_rng_innov(&mut self, innov_m: Ftype, var: Ftype) {
        self.innov_rng = innov_m;
        self.var_innov_rng = var;
    }

    /// Poke the stub flow innovation / variance (no LOS Jacobian here).
    pub fn set_flow_innov(&mut self, innov_rad_s: Ftype, var: Ftype) {
        self.innov_flow = innov_rad_s;
        self.var_innov_flow = var;
    }

    /// Poke `imuSampleTime_ms`.
    pub fn set_imu_sample_time_ms(&mut self, time_ms: u32) {
        self.imu_sample_time_ms = time_ms;
    }

    /// Poke `flowValidMeaTime_ms`.
    pub fn set_flow_valid_mea_time_ms(&mut self, time_ms: u32) {
        self.flow_valid_mea_time_ms = time_ms;
    }

    /// Poke `gndHgtValidTime_ms`.
    pub fn set_gnd_hgt_valid_time_ms(&mut self, time_ms: u32) {
        self.gnd_hgt_valid_time_ms = time_ms;
    }

    /// Range innovation is inside the consistency gate.
    #[must_use]
    pub fn rng_innovation_consistent(&self) -> bool {
        self.rng_test_ratio < (1.0 as Ftype)
    }

    /// Flow innovation is inside the consistency gate and below rate limit.
    #[must_use]
    pub fn flow_innovation_consistent(&self) -> bool {
        self.flow_test_ratio < (1.0 as Ftype)
            && self.flow_rad_x < self.max_flow_rate
            && self.flow_rad_y < self.max_flow_rate
    }

    /// Terrain-offset rangefinder enable, upstream `EstimateTerrainOffset`
    /// entry (`SelectRngFusion` in the port).
    #[must_use]
    pub const fn rng_enable_ok(&self, states_initialised: bool) -> bool {
        states_initialised
            && self.range_data_to_fuse
            && self.tilt_ok
            && !self.active_hgt_is_rangefinder
    }

    /// Main-filter optical-flow enable, upstream `SelectFlowFusion`
    /// `fuse_optflow` conjunction.
    #[must_use]
    pub const fn flow_enable_ok(&self, states_initialised: bool) -> bool {
        states_initialised
            && self.flow_data_to_fuse
            && self.tilt_ok
            && matches!(self.flow_use, FlowUse::Nav)
            && self.use_optflow_xy
    }

    /// Refresh `flowDataValid` / `gndOffsetValid` freshness windows.
    pub fn update_freshness(&mut self) {
        self.flow_data_valid = self
            .imu_sample_time_ms
            .wrapping_sub(self.flow_valid_mea_time_ms)
            < FLOW_VALID_MS;
        self.gnd_offset_valid = self
            .imu_sample_time_ms
            .wrapping_sub(self.gnd_hgt_valid_time_ms)
            < GND_OFFSET_VALID_MS
            || self.active_hgt_is_rangefinder;
    }

    /// Combined selector, upstream `SelectFlowFusion`.
    ///
    /// Applies the 200 Hz mag delay, then the rangefinder terrain
    /// path and the main-filter flow path. Jacobians are not here.
    pub fn select_rng_flow_fusion(&mut self, states_initialised: bool) {
        self.rng_fuse_performed = false;
        self.flow_fuse_performed = false;
        self.rng_fuse_sel = RngFuseSel::NotFusing;
        self.flow_fuse_sel = FlowFuseSel::NotFusing;

        if self.mag_fuse_performed
            && self.dt_imu_avg < MAG_DELAY_DT_S
            && !self.opt_flow_fusion_delayed
        {
            self.opt_flow_fusion_delayed = true;
            return;
        }
        self.opt_flow_fusion_delayed = false;

        self.update_freshness();

        if !self.takeoff_detected {
            // Upstream zeros flow when AGL is below 0.5 m so a carry
            // test still marks `flowDataValid`. The AGL number is not
            // in this slice; pre-takeoff keeps validity latched.
            self.flow_data_valid = true;
        }

        self.select_rng_fusion(states_initialised);
        self.select_flow_fusion(states_initialised);
    }

    /// Rangefinder half of `SelectFlowFusion` / `EstimateTerrainOffset`.
    pub fn select_rng_fusion(&mut self, states_initialised: bool) {
        self.rng_fuse_performed = false;
        self.rng_fuse_sel = RngFuseSel::NotFusing;
        if !self.rng_enable_ok(states_initialised) {
            return;
        }
        self.rng_fuse_sel = RngFuseSel::FuseRng;
        self.fuse_rng();
        self.range_data_to_fuse = false;
    }

    /// Optical-flow half of `SelectFlowFusion` / `FuseOptFlow`.
    pub fn select_flow_fusion(&mut self, states_initialised: bool) {
        self.flow_fuse_performed = false;
        self.flow_fuse_sel = FlowFuseSel::NotFusing;
        if !self.flow_enable_ok(states_initialised) {
            return;
        }
        self.flow_fuse_sel = FlowFuseSel::FuseFlow;
        self.fuse_opt_flow();
        self.flow_data_to_fuse = false;
    }

    /// Upstream `EstimateTerrainOffset` rangefinder innovation check.
    ///
    /// Sets [`rng_fuse_performed`](Self::rng_fuse_performed) when
    /// `auxRngTestRatio < 1`. The 1-state Kalman gain is not here.
    pub fn fuse_rng(&mut self) {
        self.rng_fuse_performed = false;
        if self.var_innov_rng <= (0.0 as Ftype) {
            return;
        }
        self.rng_test_ratio = innov_test_ratio(
            self.innov_rng,
            self.rng_innov_gate,
            self.var_innov_rng,
        );
        if self.rng_innovation_consistent() {
            self.rng_fuse_performed = true;
            self.gnd_hgt_valid_time_ms = self.imu_sample_time_ms;
            self.gnd_offset_valid = true;
        }
    }

    /// Upstream `FuseOptFlow` innovation / rate-limit check.
    ///
    /// Sets [`flow_fuse_performed`](Self::flow_fuse_performed) when
    /// the ratio is below 1 and both axes are slower than
    /// `EK3_FLOW_MAX`. The sequential LOS update is not here.
    pub fn fuse_opt_flow(&mut self) {
        self.flow_fuse_performed = false;
        if self.var_innov_flow <= (0.0 as Ftype) {
            return;
        }
        self.flow_test_ratio = innov_test_ratio(
            self.innov_flow,
            self.flow_innov_gate,
            self.var_innov_flow,
        );
        if self.flow_innovation_consistent() {
            self.flow_fuse_performed = true;
            self.prev_flow_fuse_time_ms = self.imu_sample_time_ms;
            self.flow_valid_mea_time_ms = self.imu_sample_time_ms;
            self.flow_data_valid = true;
        }
    }
}

fn innov_test_ratio(innov: Ftype, gate: i16, var: Ftype) -> Ftype {
    let scaled = (0.01 as Ftype) * (gate as Ftype);
    let sigma = if scaled > (1.0 as Ftype) {
        scaled
    } else {
        1.0 as Ftype
    };
    sq(innov) / (sq(sigma) * var)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_rng(r: &mut RngFlowFusion) {
        r.set_range_data(true, 3.0 as Ftype);
        r.set_rng_innov(0.0 as Ftype, 1.0 as Ftype);
        r.set_tilt_ok(true);
        r.set_active_hgt_is_rangefinder(false);
        r.set_imu_sample_time_ms(1000);
    }

    fn ready_flow(r: &mut RngFlowFusion) {
        r.set_flow_use(FlowUse::Nav);
        r.set_use_optflow_xy(true);
        r.set_flow_data(true, 0.2 as Ftype, 0.1 as Ftype);
        r.set_flow_innov(0.0 as Ftype, 1.0 as Ftype);
        r.set_tilt_ok(true);
        r.set_takeoff_detected(true);
        r.set_imu_sample_time_ms(1000);
        r.set_flow_valid_mea_time_ms(1000);
    }

    #[test]
    fn rng_enable_gate_refuses_until_sample_init_tilt_and_not_hgt_source() {
        let mut r = RngFlowFusion::new();
        r.set_range_data(true, 3.0 as Ftype);
        r.set_rng_innov(0.0 as Ftype, 1.0 as Ftype);
        r.select_rng_flow_fusion(false);
        assert_eq!(r.rng_fuse_sel(), RngFuseSel::NotFusing);
        assert!(!r.rng_fuse_performed());

        r.select_rng_flow_fusion(true);
        assert_eq!(r.rng_fuse_sel(), RngFuseSel::FuseRng);
        assert!(r.rng_fuse_performed());
        assert!(!r.range_data_to_fuse());

        let mut hgt = RngFlowFusion::new();
        ready_rng(&mut hgt);
        hgt.set_active_hgt_is_rangefinder(true);
        hgt.select_rng_flow_fusion(true);
        assert_eq!(hgt.rng_fuse_sel(), RngFuseSel::NotFusing);
        assert!(!hgt.rng_fuse_performed());
    }

    #[test]
    fn flow_enable_gate_refuses_until_nav_source_sample_and_tilt() {
        let mut r = RngFlowFusion::new();
        // Plane default is TERRAIN: main-filter flow stays closed.
        assert_eq!(r.flow_use(), FlowUse::Terrain);
        r.set_flow_data(true, 0.2 as Ftype, 0.1 as Ftype);
        r.set_flow_innov(0.0 as Ftype, 1.0 as Ftype);
        r.set_use_optflow_xy(true);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::NotFusing);
        assert!(!r.flow_fuse_performed());

        r.set_flow_use(FlowUse::Nav);
        r.set_flow_data(true, 0.2 as Ftype, 0.1 as Ftype);
        r.set_tilt_ok(false);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::NotFusing);

        r.set_tilt_ok(true);
        r.set_flow_data(true, 0.2 as Ftype, 0.1 as Ftype);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(r.flow_fuse_performed());
        assert!(!r.flow_data_to_fuse());
        assert_eq!(r.prev_flow_fuse_time_ms(), 0);
    }

    #[test]
    fn rng_innovation_gate_refuses_large_innov() {
        let mut r = RngFlowFusion::new();
        ready_rng(&mut r);
        // innov = 10, gate sigma = 5, var = 1 → ratio = 100 / 25 = 4.
        r.set_rng_innov(10.0 as Ftype, 1.0 as Ftype);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.rng_fuse_sel(), RngFuseSel::FuseRng);
        assert!(!r.rng_fuse_performed());
        assert!(r.rng_test_ratio() > (1.0 as Ftype));
        assert!(!r.rng_innovation_consistent());
    }

    #[test]
    fn flow_quality_gate_refuses_large_innov_or_fast_rate() {
        let mut r = RngFlowFusion::new();
        ready_flow(&mut r);
        // innov = 10, gate sigma = 5, var = 1 → ratio = 4.
        r.set_flow_innov(10.0 as Ftype, 1.0 as Ftype);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(!r.flow_fuse_performed());
        assert!(r.flow_test_ratio() > (1.0 as Ftype));

        let mut fast = RngFlowFusion::new();
        ready_flow(&mut fast);
        fast.set_flow_data(true, 3.0 as Ftype, 0.1 as Ftype);
        fast.select_rng_flow_fusion(true);
        assert_eq!(fast.flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(!fast.flow_fuse_performed());
        assert!(!fast.flow_innovation_consistent());
    }

    #[test]
    fn quality_gates_accept_when_ratio_below_one() {
        let mut r = RngFlowFusion::new();
        ready_rng(&mut r);
        ready_flow(&mut r);
        r.select_rng_flow_fusion(true);
        assert_eq!(r.rng_fuse_sel(), RngFuseSel::FuseRng);
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(r.rng_fuse_performed());
        assert!(r.flow_fuse_performed());
        assert!(r.rng_test_ratio() < (1.0 as Ftype));
        assert!(r.flow_test_ratio() < (1.0 as Ftype));
        assert_eq!(r.prev_flow_fuse_time_ms(), 1000);
    }

    #[test]
    fn mag_fuse_at_high_rate_delays_once() {
        let mut r = RngFlowFusion::new();
        ready_flow(&mut r);
        r.set_mag_fuse_timing(true, 0.004 as Ftype);
        r.select_rng_flow_fusion(true);
        assert!(r.opt_flow_fusion_delayed());
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::NotFusing);
        assert!(!r.flow_fuse_performed());
        assert!(r.flow_data_to_fuse());

        r.select_rng_flow_fusion(true);
        assert!(!r.opt_flow_fusion_delayed());
        assert_eq!(r.flow_fuse_sel(), FlowFuseSel::FuseFlow);
        assert!(r.flow_fuse_performed());
    }

    #[test]
    fn freshness_windows_match_upstream() {
        let mut r = RngFlowFusion::new();
        r.set_imu_sample_time_ms(6000);
        r.set_flow_valid_mea_time_ms(5001);
        r.set_gnd_hgt_valid_time_ms(500);
        r.update_freshness();
        assert!(r.flow_data_valid());
        assert!(!r.gnd_offset_valid());

        r.set_gnd_hgt_valid_time_ms(2000);
        r.update_freshness();
        assert!(r.gnd_offset_valid());

        r.set_active_hgt_is_rangefinder(true);
        r.set_gnd_hgt_valid_time_ms(0);
        r.update_freshness();
        assert!(r.gnd_offset_valid());
    }
}
