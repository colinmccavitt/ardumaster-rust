//! Position / velocity fusion enable gate, upstream `AP_NavEKF3_PosVelFusion.cpp`.
//!
//! This slice is the gate that `SelectVelPosFusion` evaluates before
//! `FuseVelPosNED`. Upstream names the combined selector; the two
//! halves are [`PosVelFusion::select_vel_fusion`] and
//! [`PosVelFusion::select_pos_fusion`]. The sequential Kalman update
//! that consumes NED velocity and position is not here.
//!
//! # GPS enable gate
//!
//! Horizontal GPS fusion starts only when a delayed sample is at the
//! fusion horizon (`gpsDataToFuse && !waitingForGpsChecks`), the core
//! is in `AID_ABSOLUTE`, the XY source is GPS, and `gpsInhibit` is
//! clear. Velocity then also needs `useVelXYSource(GPS)`; vertical
//! velocity needs `useVelZSource(GPS) && useGpsVertVel`.
//!
//! # Quality gates
//!
//! `FuseVelPosNED` drops an axis when the innovation consistency
//! check fails and there is no timeout or bad-IMU override. This stub
//! keeps those three flags plus the `gpsAccuracyGood` latch from
//! `AP_NavEKF3_VehicleStatus.cpp`. A high-rate magnetometer step
//! (`magFusePerformed && dtIMUavg < 0.005`) delays the whole selector
//! by one IMU frame.

use crate::control::AidingMode;
use crate::measurements::EKF_TARGET_DT;
use crate::Ftype;

/// 200 Hz: `dtIMUavg < 0.005` skips pos/vel when mag already fused.
const MAG_DELAY_DT_S: Ftype = 0.005;

/// Position / velocity fusion selection, derived from `fuseVelData` /
/// `fusePosData` after the quality gates.
///
/// Discriminant values are local to the port so a sitl-diff dump can
/// compare the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PosVelFuseSel {
    /// Neither axis this step. Upstream both flags false.
    NotFusing = 0,
    /// Velocity only. Upstream `fuseVelData` (and maybe `fuseVelVertData`).
    FuseVel = 1,
    /// Horizontal position only. Upstream `fusePosData`.
    FusePos = 2,
    /// Velocity and horizontal position. Upstream both flags true.
    FuseVelPos = 3,
}

/// Pos/vel fusion latch, the `NavEKF3_core` fields `SelectVelPosFusion`
/// reads and writes.
///
/// Covariance and the sequential `FuseVelPosNED` Jacobians are not
/// here: tests (and later cores) poke the GPS / aiding / quality flags
/// that the selector would have read from DAL and VehicleStatus.
#[derive(Debug, Clone)]
pub struct PosVelFusion {
    gps_data_to_fuse: bool,
    waiting_for_gps_checks: bool,
    gps_inhibit: bool,
    gps_good_to_align: bool,
    gps_accuracy_good: bool,
    valid_origin: bool,
    tilt_align_complete: bool,
    yaw_align_complete: bool,
    del_ang_bias_learned: bool,
    assume_zero_sideslip: bool,
    aiding_mode: AidingMode,
    posxy_source_is_gps: bool,
    use_vel_xy_gps: bool,
    use_vel_z_gps: bool,
    use_gps_vert_vel: bool,
    mag_fuse_performed: bool,
    dt_imu_avg: Ftype,
    pos_vel_fusion_delayed: bool,
    pos_check_passed: bool,
    vel_check_passed: bool,
    pos_timeout: bool,
    vel_timeout: bool,
    bad_imu_data: bool,
    fuse_pos_data: bool,
    fuse_vel_data: bool,
    fuse_vel_vert_data: bool,
    fuse_performed: bool,
    fuse_sel: PosVelFuseSel,
}

impl Default for PosVelFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl PosVelFusion {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// `PV_AidingMode = AID_NONE`, GPS source selected, no delayed
    /// sample, quality latches false, `dtIMUavg = EKF_TARGET_DT`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            gps_data_to_fuse: false,
            waiting_for_gps_checks: false,
            gps_inhibit: false,
            gps_good_to_align: false,
            gps_accuracy_good: false,
            valid_origin: false,
            tilt_align_complete: false,
            yaw_align_complete: false,
            del_ang_bias_learned: false,
            assume_zero_sideslip: true,
            aiding_mode: AidingMode::None,
            posxy_source_is_gps: true,
            use_vel_xy_gps: true,
            use_vel_z_gps: false,
            use_gps_vert_vel: false,
            mag_fuse_performed: false,
            dt_imu_avg: EKF_TARGET_DT,
            pos_vel_fusion_delayed: false,
            pos_check_passed: true,
            vel_check_passed: true,
            pos_timeout: false,
            vel_timeout: false,
            bad_imu_data: false,
            fuse_pos_data: false,
            fuse_vel_data: false,
            fuse_vel_vert_data: false,
            fuse_performed: false,
            fuse_sel: PosVelFuseSel::NotFusing,
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Delayed GPS sample is at the fusion horizon, upstream `gpsDataToFuse`.
    #[must_use]
    pub const fn gps_data_to_fuse(&self) -> bool {
        self.gps_data_to_fuse
    }

    /// External GPS inhibit flag, upstream `gpsInhibit`.
    #[must_use]
    pub const fn gps_inhibit(&self) -> bool {
        self.gps_inhibit
    }

    /// GPS quality can initialise navigation, upstream `gpsGoodToAlign`.
    #[must_use]
    pub const fn gps_good_to_align(&self) -> bool {
        self.gps_good_to_align
    }

    /// GPS accuracy is good enough for flight, upstream `gpsAccuracyGood`.
    #[must_use]
    pub const fn gps_accuracy_good(&self) -> bool {
        self.gps_accuracy_good
    }

    /// Current aiding mode used by the selector, upstream `PV_AidingMode`.
    #[must_use]
    pub const fn aiding_mode(&self) -> AidingMode {
        self.aiding_mode
    }

    /// Horizontal GPS fusion selected this step, upstream `fusePosData`.
    #[must_use]
    pub const fn fuse_pos_data(&self) -> bool {
        self.fuse_pos_data
    }

    /// Horizontal GPS velocity fusion selected this step, upstream `fuseVelData`.
    #[must_use]
    pub const fn fuse_vel_data(&self) -> bool {
        self.fuse_vel_data
    }

    /// Vertical GPS velocity fusion selected this step, upstream `fuseVelVertData`.
    #[must_use]
    pub const fn fuse_vel_vert_data(&self) -> bool {
        self.fuse_vel_vert_data
    }

    /// `FuseVelPosNED` ran this step (enable-side stub).
    #[must_use]
    pub const fn fuse_performed(&self) -> bool {
        self.fuse_performed
    }

    /// Combined selection after the quality gates.
    #[must_use]
    pub const fn fuse_sel(&self) -> PosVelFuseSel {
        self.fuse_sel
    }

    /// Mag-step delay is holding pos/vel off this IMU frame.
    #[must_use]
    pub const fn pos_vel_fusion_delayed(&self) -> bool {
        self.pos_vel_fusion_delayed
    }

    /// Poke `gpsDataToFuse`.
    pub fn set_gps_data_to_fuse(&mut self, ready: bool) {
        self.gps_data_to_fuse = ready;
    }

    /// Poke `waitingForGpsChecks`.
    pub fn set_waiting_for_gps_checks(&mut self, waiting: bool) {
        self.waiting_for_gps_checks = waiting;
    }

    /// Poke `gpsInhibit`.
    pub fn set_gps_inhibit(&mut self, inhibit: bool) {
        self.gps_inhibit = inhibit;
    }

    /// Poke `gpsGoodToAlign`.
    pub fn set_gps_good_to_align(&mut self, good: bool) {
        self.gps_good_to_align = good;
    }

    /// Poke `gpsAccuracyGood`.
    pub fn set_gps_accuracy_good(&mut self, good: bool) {
        self.gps_accuracy_good = good;
    }

    /// Poke origin / tilt / yaw / gyro-bias bits of `readyToUseGPS`.
    pub fn set_alignment(
        &mut self,
        valid_origin: bool,
        tilt_align_complete: bool,
        yaw_align_complete: bool,
        del_ang_bias_learned: bool,
    ) {
        self.valid_origin = valid_origin;
        self.tilt_align_complete = tilt_align_complete;
        self.yaw_align_complete = yaw_align_complete;
        self.del_ang_bias_learned = del_ang_bias_learned;
    }

    /// Plane vs copter `readyToUseGPS` gyro-bias exception.
    pub fn set_assume_zero_sideslip(&mut self, assume: bool) {
        self.assume_zero_sideslip = assume;
    }

    /// Poke `PV_AidingMode`.
    pub fn set_aiding_mode(&mut self, mode: AidingMode) {
        self.aiding_mode = mode;
    }

    /// Poke `getPosXYSource == GPS`.
    pub fn set_posxy_source_is_gps(&mut self, is_gps: bool) {
        self.posxy_source_is_gps = is_gps;
    }

    /// Poke `useVelXYSource(GPS)` / `useVelZSource(GPS)` / `useGpsVertVel`.
    pub fn set_vel_sources(&mut self, xy: bool, z: bool, use_gps_vert_vel: bool) {
        self.use_vel_xy_gps = xy;
        self.use_vel_z_gps = z;
        self.use_gps_vert_vel = use_gps_vert_vel;
    }

    /// Poke `magFusePerformed` and `dtIMUavg` for the 200 Hz delay.
    pub fn set_mag_fuse_timing(&mut self, mag_fuse_performed: bool, dt_imu_avg: Ftype) {
        self.mag_fuse_performed = mag_fuse_performed;
        self.dt_imu_avg = dt_imu_avg;
    }

    /// Poke the `FuseVelPosNED` innovation / timeout / bad-IMU flags.
    pub fn set_quality_overrides(
        &mut self,
        pos_check_passed: bool,
        vel_check_passed: bool,
        pos_timeout: bool,
        vel_timeout: bool,
        bad_imu_data: bool,
    ) {
        self.pos_check_passed = pos_check_passed;
        self.vel_check_passed = vel_check_passed;
        self.pos_timeout = pos_timeout;
        self.vel_timeout = vel_timeout;
        self.bad_imu_data = bad_imu_data;
    }

    /// Delayed GPS sample is usable, upstream `gpsDataToFuse && !waitingForGpsChecks`
    /// plus the historical `gpsInhibit` handshake.
    #[must_use]
    pub const fn gps_sample_usable(&self) -> bool {
        self.gps_data_to_fuse && !self.waiting_for_gps_checks && !self.gps_inhibit
    }

    /// GPS is the absolute XY source, upstream `AID_ABSOLUTE` and `SourceXY::GPS`.
    #[must_use]
    pub const fn absolute_gps_source(&self) -> bool {
        matches!(self.aiding_mode, AidingMode::Absolute) && self.posxy_source_is_gps
    }

    /// Upstream `NavEKF3_core::readyToUseGPS` plus `gpsInhibit`.
    ///
    /// `setAidingMode` promotes `AID_NONE` only when this is true. The
    /// selector itself still requires [`absolute_gps_source`].
    #[must_use]
    pub const fn ready_to_use_gps(&self) -> bool {
        self.posxy_source_is_gps
            && self.valid_origin
            && self.tilt_align_complete
            && self.yaw_align_complete
            && (self.del_ang_bias_learned || self.assume_zero_sideslip)
            && self.gps_good_to_align
            && self.gps_data_to_fuse
            && !self.gps_inhibit
    }

    /// Position half of `FuseVelPosNED` quality: innovation pass, or
    /// timeout / bad IMU, and `gpsAccuracyGood` unless overridden.
    #[must_use]
    pub const fn pos_quality_ok(&self) -> bool {
        self.pos_timeout
            || self.bad_imu_data
            || (self.gps_accuracy_good && self.pos_check_passed)
    }

    /// Velocity half of `FuseVelPosNED` quality.
    #[must_use]
    pub const fn vel_quality_ok(&self) -> bool {
        self.vel_timeout
            || self.bad_imu_data
            || (self.gps_accuracy_good && self.vel_check_passed)
    }

    /// Upstream velocity half of `SelectVelPosFusion`.
    ///
    /// Sets `fuseVelData` / `fuseVelVertData` from the GPS source flags,
    /// then drops them when the velocity quality gate fails.
    pub fn select_vel_fusion(&mut self) {
        self.fuse_vel_data = false;
        self.fuse_vel_vert_data = false;

        if !self.gps_sample_usable() || !self.absolute_gps_source() {
            return;
        }

        self.fuse_vel_data = self.use_vel_xy_gps;
        self.fuse_vel_vert_data = self.use_vel_z_gps && self.use_gps_vert_vel;

        if (self.fuse_vel_data || self.fuse_vel_vert_data) && !self.vel_quality_ok() {
            self.fuse_vel_data = false;
            self.fuse_vel_vert_data = false;
        }
    }

    /// Upstream position half of `SelectVelPosFusion`.
    ///
    /// Sets `fusePosData` when a delayed GPS sample is the absolute XY
    /// source, then drops it when the position quality gate fails.
    pub fn select_pos_fusion(&mut self) {
        self.fuse_pos_data = false;

        if !self.gps_sample_usable() || !self.absolute_gps_source() {
            return;
        }

        self.fuse_pos_data = true;
        if !self.pos_quality_ok() {
            self.fuse_pos_data = false;
        }
    }

    /// Upstream `NavEKF3_core::SelectVelPosFusion` enable gate.
    ///
    /// Applies the 200 Hz mag delay, runs both halves, records
    /// [`fuse_sel`](Self::fuse_sel), then the [`fuse_vel_pos_ned`]
    /// performed-flag stub. Jacobians are not here.
    pub fn select_vel_pos_fusion(&mut self) {
        self.fuse_performed = false;
        self.fuse_sel = PosVelFuseSel::NotFusing;
        self.fuse_pos_data = false;
        self.fuse_vel_data = false;
        self.fuse_vel_vert_data = false;

        if self.mag_fuse_performed && self.dt_imu_avg < MAG_DELAY_DT_S && !self.pos_vel_fusion_delayed
        {
            self.pos_vel_fusion_delayed = true;
            return;
        }
        self.pos_vel_fusion_delayed = false;

        self.select_vel_fusion();
        self.select_pos_fusion();
        self.record_sel();

        if self.fuse_vel_data || self.fuse_vel_vert_data || self.fuse_pos_data {
            self.fuse_vel_pos_ned();
        }
    }

    /// Upstream `NavEKF3_core::FuseVelPosNED` enable-side stub.
    ///
    /// Sets `fusePerformed` and clears the per-axis flags so the same
    /// sample is not fused twice. The sequential-axis Jacobians and
    /// Kalman gain are not here.
    pub fn fuse_vel_pos_ned(&mut self) {
        self.fuse_performed = self.fuse_vel_data || self.fuse_vel_vert_data || self.fuse_pos_data;
        self.fuse_vel_data = false;
        self.fuse_vel_vert_data = false;
        self.fuse_pos_data = false;
    }

    fn record_sel(&mut self) {
        self.fuse_sel = match (self.fuse_vel_data, self.fuse_pos_data) {
            (false, false) => PosVelFuseSel::NotFusing,
            (true, false) => PosVelFuseSel::FuseVel,
            (false, true) => PosVelFuseSel::FusePos,
            (true, true) => PosVelFuseSel::FuseVelPos,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_absolute_gps(pv: &mut PosVelFusion) {
        pv.set_gps_data_to_fuse(true);
        pv.set_aiding_mode(AidingMode::Absolute);
        pv.set_gps_accuracy_good(true);
        pv.set_gps_good_to_align(true);
        pv.set_alignment(true, true, true, true);
    }

    #[test]
    fn enable_gate_refuses_until_absolute_gps_sample() {
        let mut pv = PosVelFusion::new();
        pv.select_vel_pos_fusion();
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
        assert!(!pv.fuse_performed());

        pv.set_gps_data_to_fuse(true);
        pv.set_gps_accuracy_good(true);
        pv.select_vel_pos_fusion();
        // Sample present but still AID_NONE: GPS pos/vel stays closed.
        assert_eq!(pv.aiding_mode(), AidingMode::None);
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
        assert!(!pv.fuse_performed());

        pv.set_aiding_mode(AidingMode::Absolute);
        pv.set_posxy_source_is_gps(false);
        pv.select_vel_pos_fusion();
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);

        pv.set_posxy_source_is_gps(true);
        pv.set_waiting_for_gps_checks(true);
        pv.select_vel_pos_fusion();
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
    }

    #[test]
    fn gps_inhibit_blocks_pos_and_vel() {
        let mut pv = PosVelFusion::new();
        ready_absolute_gps(&mut pv);
        pv.set_gps_inhibit(true);
        pv.select_vel_pos_fusion();
        assert!(!pv.gps_sample_usable());
        assert!(!pv.ready_to_use_gps());
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
        assert!(!pv.fuse_performed());
    }

    #[test]
    fn quality_gate_refuses_bad_accuracy_unless_timeout() {
        let mut pv = PosVelFusion::new();
        ready_absolute_gps(&mut pv);
        pv.set_gps_accuracy_good(false);
        pv.select_vel_pos_fusion();
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
        assert!(!pv.fuse_performed());

        pv.set_quality_overrides(false, false, true, true, false);
        pv.select_vel_fusion();
        pv.select_pos_fusion();
        assert!(pv.fuse_vel_data());
        assert!(pv.fuse_pos_data());
    }

    #[test]
    fn absolute_gps_selects_vel_and_pos() {
        let mut pv = PosVelFusion::new();
        ready_absolute_gps(&mut pv);
        pv.select_vel_pos_fusion();
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::FuseVelPos);
        assert!(pv.fuse_performed());
        // Upstream clears the per-axis flags after FuseVelPosNED.
        assert!(!pv.fuse_vel_data());
        assert!(!pv.fuse_pos_data());
    }

    #[test]
    fn select_vel_skips_when_xy_source_unused() {
        let mut pv = PosVelFusion::new();
        ready_absolute_gps(&mut pv);
        pv.set_vel_sources(false, false, false);
        pv.select_vel_fusion();
        pv.select_pos_fusion();
        assert!(!pv.fuse_vel_data());
        assert!(!pv.fuse_vel_vert_data());
        assert!(pv.fuse_pos_data());
    }

    #[test]
    fn mag_fuse_at_high_rate_delays_once() {
        let mut pv = PosVelFusion::new();
        ready_absolute_gps(&mut pv);
        pv.set_mag_fuse_timing(true, 0.004 as Ftype);
        pv.select_vel_pos_fusion();
        assert!(pv.pos_vel_fusion_delayed());
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::NotFusing);
        assert!(!pv.fuse_performed());

        pv.select_vel_pos_fusion();
        assert!(!pv.pos_vel_fusion_delayed());
        assert_eq!(pv.fuse_sel(), PosVelFuseSel::FuseVelPos);
        assert!(pv.fuse_performed());
    }

    #[test]
    fn ready_to_use_gps_needs_align_quality_and_no_inhibit() {
        let mut pv = PosVelFusion::new();
        pv.set_gps_data_to_fuse(true);
        assert!(!pv.ready_to_use_gps());

        pv.set_alignment(true, true, true, false);
        pv.set_gps_good_to_align(true);
        // Plane (`assume_zero_sideslip`) does not wait for gyro-bias learn.
        assert!(pv.ready_to_use_gps());

        pv.set_assume_zero_sideslip(false);
        assert!(!pv.ready_to_use_gps());
        pv.set_alignment(true, true, true, true);
        assert!(pv.ready_to_use_gps());

        pv.set_gps_inhibit(true);
        assert!(!pv.ready_to_use_gps());
    }
}
