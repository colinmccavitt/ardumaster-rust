//! Height / baro fusion enable / offset / innovation gate, upstream
//! `AP_NavEKF3_PosVelFusion.cpp` (`selectHeightForFusion`) and
//! `AP_NavEKF3_Measurements.cpp` (`calcFiltBaroOffset`).
//!
//! This slice is the gate that `selectHeightForFusion` evaluates before
//! `FuseVelPosNED` consumes `hgtMea`. The sequential Kalman update
//! that writes `position.z` is not here.
//!
//! # Enable gate
//!
//! Baro height starts only when a delayed sample is at the fusion
//! horizon (`baroDataToFuse`), bootstrap has latched
//! (`statesInitialised`), and the active Z source is BARO. Plane
//! defaults `activeHgtSource` to BARO. Lost GPS / rangefinder height
//! falls back to baro (`fallback_to_baro`).
//!
//! # Baro offset
//!
//! `calcFiltBaroOffset` is a first-order LPF with ±5 m spike
//! protection. It runs on new baro when the active source is *not*
//! BARO, so a later revert can subtract `baroHgtOffset` from the raw
//! baro and match the filter height:
//! `hgtMea = baroDataDelayed.hgt - baroHgtOffset`.
//!
//! # Innovation gate
//!
//! `FuseVelPosNED` forms
//! `hgtTestRatio = sq(innov) / (sq(MAX(0.01 * hgtInnovGate, 1)) * varInnov)`
//! with `innov = position.z - (-hgtMea)`. It fuses when the ratio is
//! below 1 (or 3 when `AID_NONE && onGround`), or when `hgtTimeout` /
//! `badIMUdata` override. A timeout resets height instead of fusing.

use ap_math::scalar::sq;
use ap_math::Ftype;

use crate::{StateIndex, StateVector};

/// Default `EK3_HGT_I_GATE` (`_hgtInnovGate`).
pub const HGT_INNOV_GATE_DEFAULT: i16 = 500;

/// Upstream `hgtRetryTimeMode12_ms`: no vertical GPS velocity.
pub const HGT_RETRY_TIME_MODE12_MS: u32 = 5000;

/// Upstream `hgtRetryTimeMode0_ms`: vertical GPS / ext-nav velocity.
pub const HGT_RETRY_TIME_MODE0_MS: u32 = 10000;

/// LPF gain on `calcFiltBaroOffset`.
const BARO_OFFSET_ALPHA: Ftype = 0.1;

/// Spike clamp (m) on the baro-offset residual.
const BARO_OFFSET_SPIKE_M: Ftype = 5.0;

/// GPS height is "lost" after this many ms without a fix.
const GPS_HGT_LOST_MS: u32 = 2000;

/// Height source, upstream `AP_NavEKF_Source::SourceZ`.
///
/// Discriminant values match the upstream enum so a sitl-diff dump can
/// compare the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightSource {
    /// No height sensor, upstream `SourceZ::NONE`.
    None = 0,
    /// Barometer, upstream `SourceZ::BARO`.
    Baro = 1,
    /// Range finder, upstream `SourceZ::RANGEFINDER`.
    Rangefinder = 2,
    /// GPS altitude, upstream `SourceZ::GPS`.
    Gps = 3,
}

/// Height fusion selection after [`HeightFusion::select_height_fusion`].
///
/// Discriminant values are local to the port so a sitl-diff dump can
/// compare the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightFuseSel {
    /// Selector did not call the baro path this step.
    NotFusing = 0,
    /// Enable gate opened; `FuseBaro` ran (gate may still reject).
    FuseBaro = 1,
}

/// Height / baro fusion latch, the `NavEKF3_core` fields
/// `selectHeightForFusion` and the `FuseVelPosNED` height check read
/// and write.
///
/// Covariance and the sequential `FuseVelPosNED` Jacobians are not
/// here: tests (and later cores) poke the baro / source / innovation
/// flags that the selector would have read from DAL.
#[derive(Debug, Clone)]
pub struct HeightFusion {
    baro_data_to_fuse: bool,
    baro_hgt: Ftype,
    baro_hgt_offset: Ftype,
    pos_d: Ftype,
    configured_source: HeightSource,
    active_hgt_source: HeightSource,
    prev_hgt_source: HeightSource,
    gps_data_fresh: bool,
    gps_accuracy_good_for_altitude: bool,
    last_time_gps_received_ms: u32,
    range_finder_data_fresh: bool,
    fuse_hgt_data: bool,
    hgt_mea: Ftype,
    innov_hgt: Ftype,
    var_innov_hgt: Ftype,
    hgt_test_ratio: Ftype,
    hgt_innov_gate: i16,
    bad_imu_data: bool,
    hgt_timeout: bool,
    on_ground: bool,
    aiding_none: bool,
    use_gps_vert_vel: bool,
    vel_timeout: bool,
    imu_sample_time_ms: u32,
    last_hgt_pass_time_ms: u32,
    source_reset: bool,
    fuse_performed: bool,
    fuse_sel: HeightFuseSel,
}

impl Default for HeightFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl HeightFusion {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// Baro source, zero offset, height timed out, no delayed sample,
    /// `EK3_HGT_I_GATE = 500`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            baro_data_to_fuse: false,
            baro_hgt: 0.0 as Ftype,
            baro_hgt_offset: 0.0 as Ftype,
            pos_d: 0.0 as Ftype,
            configured_source: HeightSource::Baro,
            active_hgt_source: HeightSource::Baro,
            prev_hgt_source: HeightSource::Baro,
            gps_data_fresh: false,
            gps_accuracy_good_for_altitude: false,
            last_time_gps_received_ms: 0,
            range_finder_data_fresh: false,
            fuse_hgt_data: false,
            hgt_mea: 0.0 as Ftype,
            innov_hgt: 0.0 as Ftype,
            var_innov_hgt: 1.0 as Ftype,
            hgt_test_ratio: 0.0 as Ftype,
            hgt_innov_gate: HGT_INNOV_GATE_DEFAULT,
            bad_imu_data: false,
            hgt_timeout: true,
            on_ground: true,
            aiding_none: true,
            use_gps_vert_vel: false,
            vel_timeout: false,
            imu_sample_time_ms: 0,
            last_hgt_pass_time_ms: 0,
            source_reset: false,
            fuse_performed: false,
            fuse_sel: HeightFuseSel::NotFusing,
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Delayed baro sample is at the fusion horizon, upstream `baroDataToFuse`.
    #[must_use]
    pub const fn baro_data_to_fuse(&self) -> bool {
        self.baro_data_to_fuse
    }

    /// Filtered baro offset (m), upstream `baroHgtOffset`.
    #[must_use]
    pub const fn baro_hgt_offset(&self) -> Ftype {
        self.baro_hgt_offset
    }

    /// Configured Z source, upstream `sources.getPosZSource`.
    #[must_use]
    pub const fn configured_source(&self) -> HeightSource {
        self.configured_source
    }

    /// Active Z source after fallback, upstream `activeHgtSource`.
    #[must_use]
    pub const fn active_hgt_source(&self) -> HeightSource {
        self.active_hgt_source
    }

    /// Height observation (m, up-positive), upstream `hgtMea`.
    #[must_use]
    pub const fn hgt_mea(&self) -> Ftype {
        self.hgt_mea
    }

    /// Height innovation (m), upstream `innovVelPos[5]`.
    #[must_use]
    pub const fn innov_hgt(&self) -> Ftype {
        self.innov_hgt
    }

    /// Innovation consistency ratio, upstream `hgtTestRatio`.
    #[must_use]
    pub const fn hgt_test_ratio(&self) -> Ftype {
        self.hgt_test_ratio
    }

    /// Height measurements have timed out, upstream `hgtTimeout`.
    #[must_use]
    pub const fn hgt_timeout(&self) -> bool {
        self.hgt_timeout
    }

    /// Selector armed `fuseHgtData` this step.
    #[must_use]
    pub const fn fuse_hgt_data(&self) -> bool {
        self.fuse_hgt_data
    }

    /// Kalman height update ran this step (enable-side stub).
    #[must_use]
    pub const fn fuse_performed(&self) -> bool {
        self.fuse_performed
    }

    /// Combined selection after the enable gate.
    #[must_use]
    pub const fn fuse_sel(&self) -> HeightFuseSel {
        self.fuse_sel
    }

    /// Last successful height pass time (ms), upstream `lastHgtPassTime_ms`.
    #[must_use]
    pub const fn last_hgt_pass_time_ms(&self) -> u32 {
        self.last_hgt_pass_time_ms
    }

    /// Source change latched a D-axis reset, upstream `ResetPositionD`.
    #[must_use]
    pub const fn source_reset(&self) -> bool {
        self.source_reset
    }

    /// Poke delayed baro height (m, up-positive) and `baroDataToFuse`.
    pub fn set_baro_data(&mut self, ready: bool, baro_hgt_m: Ftype) {
        self.baro_data_to_fuse = ready;
        self.baro_hgt = baro_hgt_m;
    }

    /// Poke `baroHgtOffset` (m).
    pub fn set_baro_hgt_offset(&mut self, offset_m: Ftype) {
        self.baro_hgt_offset = offset_m;
    }

    /// Poke filter down-position (m), upstream `stateStruct.position.z`.
    pub fn set_position_d(&mut self, pos_d_m: Ftype) {
        self.pos_d = pos_d_m;
    }

    /// Poke configured Z source, upstream `getPosZSource`.
    pub fn set_configured_source(&mut self, source: HeightSource) {
        self.configured_source = source;
    }

    /// Poke `activeHgtSource` (tests that start already on GPS / rangefinder).
    pub fn set_active_hgt_source(&mut self, source: HeightSource) {
        self.active_hgt_source = source;
        self.prev_hgt_source = source;
    }

    /// Poke GPS height freshness / altitude quality.
    ///
    /// `fresh` is the 500 ms `lastTimeGpsReceived` window used to
    /// *select* GPS. A stale sample is placed 3 s behind `imuSampleTime`
    /// so the 2 s lost-GPS fallback fires.
    pub fn set_gps_height(&mut self, fresh: bool, accuracy_good_for_altitude: bool) {
        self.gps_data_fresh = fresh;
        self.gps_accuracy_good_for_altitude = accuracy_good_for_altitude;
        if fresh {
            self.last_time_gps_received_ms = self.imu_sample_time_ms;
        } else {
            self.last_time_gps_received_ms = self.imu_sample_time_ms.saturating_sub(3000);
        }
    }

    /// Poke range-finder freshness (`imuSampleTime - rngValidMeaTime < 500`).
    pub fn set_range_finder_data_fresh(&mut self, fresh: bool) {
        self.range_finder_data_fresh = fresh;
    }

    /// Poke the stub innovation variance (no `P` matrix in this slice).
    pub fn set_var_innov_hgt(&mut self, var: Ftype) {
        self.var_innov_hgt = var;
    }

    /// Poke `EK3_HGT_I_GATE`.
    pub fn set_hgt_innov_gate(&mut self, gate: i16) {
        self.hgt_innov_gate = gate;
    }

    /// Poke `badIMUdata` / `onGround` / `AID_NONE` for the innovation override.
    pub fn set_quality_overrides(
        &mut self,
        bad_imu_data: bool,
        on_ground: bool,
        aiding_none: bool,
    ) {
        self.bad_imu_data = bad_imu_data;
        self.on_ground = on_ground;
        self.aiding_none = aiding_none;
    }

    /// Poke vertical-velocity aiding used by `hgtRetryTime_ms`.
    pub fn set_vert_vel_aiding(&mut self, use_gps_vert_vel: bool, vel_timeout: bool) {
        self.use_gps_vert_vel = use_gps_vert_vel;
        self.vel_timeout = vel_timeout;
    }

    /// Poke `imuSampleTime_ms`.
    pub fn set_imu_sample_time_ms(&mut self, time_ms: u32) {
        self.imu_sample_time_ms = time_ms;
    }

    /// Copy down-position from the 24-vector, the slot `hgtMea` compares to.
    pub fn read_pos_d_from_states(&mut self, states: &StateVector) {
        self.pos_d = match states.get(StateIndex::PosD.as_usize()) {
            Some(&value) => value,
            None => 0.0 as Ftype,
        };
    }

    /// Enable half of `selectHeightForFusion`: sample, bootstrap, BARO active.
    #[must_use]
    pub const fn baro_enable_ok(&self, states_initialised: bool) -> bool {
        self.baro_data_to_fuse
            && states_initialised
            && matches!(self.active_hgt_source, HeightSource::Baro)
    }

    /// Innovation consistency, upstream `hgtTestRatio < maxTestRatio`.
    ///
    /// `maxTestRatio` is 3 when `AID_NONE && onGround`, else 1.
    #[must_use]
    pub fn innovation_consistent(&self) -> bool {
        self.hgt_test_ratio < self.max_test_ratio()
    }

    /// Upstream `NavEKF3_core::selectHeightForFusion` enable gate.
    ///
    /// Picks the Z source (with baro fallback), updates
    /// [`baro_hgt_offset`](Self::baro_hgt_offset) when not on baro,
    /// then `FuseBaro` when [`baro_enable_ok`](Self::baro_enable_ok).
    /// Jacobians are not here.
    pub fn select_height_fusion(&mut self, states_initialised: bool) {
        self.fuse_performed = false;
        self.fuse_sel = HeightFuseSel::NotFusing;
        self.fuse_hgt_data = false;
        self.source_reset = false;

        self.update_active_source();

        if self.baro_data_to_fuse && !matches!(self.active_hgt_source, HeightSource::Baro) {
            self.calc_filt_baro_offset();
        }

        self.update_hgt_timeout();

        if !self.baro_enable_ok(states_initialised) {
            return;
        }

        self.fuse_sel = HeightFuseSel::FuseBaro;
        self.fuse_baro();
        self.baro_data_to_fuse = false;
        if self.fuse_hgt_data && self.active_hgt_source != self.prev_hgt_source {
            self.prev_hgt_source = self.active_hgt_source;
            self.source_reset = true;
        }
    }

    /// Upstream `calcFiltBaroOffset`: LPF with ±5 m spike protection.
    ///
    /// `baroHgtOffset += 0.1 * constrain(baro.hgt + pos.z - offset, -5, 5)`.
    pub fn calc_filt_baro_offset(&mut self) {
        let residual = self.baro_hgt + self.pos_d - self.baro_hgt_offset;
        let clipped = constrain_spike(residual);
        self.baro_hgt_offset += BARO_OFFSET_ALPHA * clipped;
    }

    /// Upstream baro path of `selectHeightForFusion` plus the
    /// `FuseVelPosNED` height innovation check.
    ///
    /// Sets `hgtMea = baro.hgt - baroHgtOffset` and [`fuse_performed`]
    /// when the innovation gate accepts. The Kalman gain and covariance
    /// update are not here.
    pub fn fuse_baro(&mut self) {
        self.fuse_performed = false;
        self.hgt_mea = self.baro_hgt - self.baro_hgt_offset;
        self.fuse_hgt_data = true;

        // `innovVelPos[5] = stateStruct.position.z - velPosObs[5]`
        // and `velPosObs[5] = -hgtMea`.
        self.innov_hgt = self.pos_d + self.hgt_mea;
        if self.var_innov_hgt <= (0.0 as Ftype) {
            self.fuse_hgt_data = false;
            return;
        }

        let scaled = (0.01 as Ftype) * (self.hgt_innov_gate as Ftype);
        let gate = if scaled > (1.0 as Ftype) {
            scaled
        } else {
            1.0 as Ftype
        };
        self.hgt_test_ratio = sq(self.innov_hgt) / (sq(gate) * self.var_innov_hgt);

        let is_consistent = self.innovation_consistent();
        if is_consistent {
            self.last_hgt_pass_time_ms = self.imu_sample_time_ms;
        }

        // Timeout resets height (`ResetHeight`) and skips the fuse.
        if is_consistent || self.bad_imu_data {
            self.fuse_performed = true;
        } else if self.hgt_timeout {
            self.fuse_hgt_data = false;
        } else {
            self.fuse_hgt_data = false;
        }
    }

    fn update_active_source(&mut self) {
        match self.configured_source {
            HeightSource::None => {
                self.active_hgt_source = HeightSource::None;
            }
            HeightSource::Rangefinder if self.range_finder_data_fresh => {
                self.active_hgt_source = HeightSource::Rangefinder;
            }
            HeightSource::Baro => {
                self.active_hgt_source = HeightSource::Baro;
            }
            HeightSource::Gps if self.gps_data_fresh && self.gps_accuracy_good_for_altitude => {
                self.active_hgt_source = HeightSource::Gps;
            }
            HeightSource::Gps => {
                self.active_hgt_source = HeightSource::Gps;
            }
            HeightSource::Rangefinder => {
                self.active_hgt_source = HeightSource::Rangefinder;
            }
        }

        let lost_rng = matches!(self.active_hgt_source, HeightSource::Rangefinder)
            && !self.range_finder_data_fresh;
        let gps_stale = self
            .imu_sample_time_ms
            .wrapping_sub(self.last_time_gps_received_ms)
            > GPS_HGT_LOST_MS;
        let lost_gps = matches!(self.active_hgt_source, HeightSource::Gps)
            && (gps_stale || !self.gps_accuracy_good_for_altitude);
        if lost_rng || lost_gps {
            self.active_hgt_source = HeightSource::Baro;
        }
    }

    fn update_hgt_timeout(&mut self) {
        let retry_ms = if self.use_gps_vert_vel && !self.vel_timeout {
            HGT_RETRY_TIME_MODE0_MS
        } else {
            HGT_RETRY_TIME_MODE12_MS
        };
        self.hgt_timeout = self
            .imu_sample_time_ms
            .wrapping_sub(self.last_hgt_pass_time_ms)
            > retry_ms;
    }

    fn max_test_ratio(&self) -> Ftype {
        if self.aiding_none && self.on_ground {
            3.0 as Ftype
        } else {
            1.0 as Ftype
        }
    }
}

fn constrain_spike(amt: Ftype) -> Ftype {
    if amt > BARO_OFFSET_SPIKE_M {
        BARO_OFFSET_SPIKE_M
    } else if amt < -BARO_OFFSET_SPIKE_M {
        -BARO_OFFSET_SPIKE_M
    } else {
        amt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_baro(h: &mut HeightFusion) {
        h.set_quality_overrides(false, false, false);
        h.set_baro_data(true, 10.0 as Ftype);
        h.set_position_d(-(10.0 as Ftype));
        h.set_var_innov_hgt(1.0 as Ftype);
        h.set_imu_sample_time_ms(1000);
    }

    #[test]
    fn enable_gate_refuses_until_sample_init_and_baro_source() {
        let mut h = HeightFusion::new();
        h.set_baro_data(true, 10.0 as Ftype);
        h.set_position_d(-(10.0 as Ftype));
        h.select_height_fusion(false);
        assert_eq!(h.fuse_sel(), HeightFuseSel::NotFusing);
        assert!(!h.fuse_performed());

        h.select_height_fusion(true);
        assert_eq!(h.active_hgt_source(), HeightSource::Baro);
        assert_eq!(h.fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(h.fuse_performed());
        assert!(!h.baro_data_to_fuse());
        let mea = h.hgt_mea() - (10.0 as Ftype);
        assert!(mea * mea < (1.0e-12 as Ftype));
    }

    #[test]
    fn innovation_gate_refuses_large_innov_unless_timeout_or_bad_imu() {
        let mut h = HeightFusion::new();
        ready_baro(&mut h);
        // pos.z = 0, hgtMea = 10 → innov = 10. Gate sigma = 5, var = 1
        // → ratio = 100 / 25 = 4 > 1.
        h.set_position_d(0.0 as Ftype);
        h.select_height_fusion(true);
        assert_eq!(h.fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(!h.fuse_performed());
        assert!(h.hgt_test_ratio() > (1.0 as Ftype));
        assert!(!h.innovation_consistent());
        assert!(!h.fuse_hgt_data());

        // Timeout (last pass still 0, imu far past 5 s) skips the fuse
        // (`ResetHeight` path) rather than accepting the sample.
        h.set_baro_data(true, 10.0 as Ftype);
        h.set_imu_sample_time_ms(HGT_RETRY_TIME_MODE12_MS + 1);
        h.select_height_fusion(true);
        assert!(h.hgt_timeout());
        assert!(!h.fuse_performed());
        assert!(!h.fuse_hgt_data());

        h.set_baro_data(true, 10.0 as Ftype);
        h.set_quality_overrides(true, false, false);
        h.select_height_fusion(true);
        assert!(h.fuse_performed());
    }

    #[test]
    fn innovation_gate_accepts_when_ratio_below_one() {
        let mut h = HeightFusion::new();
        ready_baro(&mut h);
        h.select_height_fusion(true);
        assert_eq!(h.fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(h.fuse_performed());
        assert!(h.hgt_test_ratio() < (1.0 as Ftype));
        let innov = h.innov_hgt();
        assert!(innov * innov < (1.0e-12 as Ftype));
        assert_eq!(h.last_hgt_pass_time_ms(), 1000);
    }

    #[test]
    fn lost_gps_height_falls_back_to_baro() {
        let mut h = HeightFusion::new();
        ready_baro(&mut h);
        h.set_configured_source(HeightSource::Gps);
        h.set_active_hgt_source(HeightSource::Gps);
        h.set_gps_height(false, false);
        h.select_height_fusion(true);
        assert_eq!(h.active_hgt_source(), HeightSource::Baro);
        assert_eq!(h.fuse_sel(), HeightFuseSel::FuseBaro);
        assert!(h.fuse_performed());
        assert!(h.source_reset());
    }

    #[test]
    fn non_baro_source_learns_offset_and_skips_fuse() {
        let mut h = HeightFusion::new();
        h.set_quality_overrides(false, false, false);
        h.set_configured_source(HeightSource::Gps);
        h.set_gps_height(true, true);
        h.set_baro_data(true, 12.0 as Ftype);
        h.set_position_d(-(10.0 as Ftype));
        h.set_imu_sample_time_ms(1000);
        h.select_height_fusion(true);
        assert_eq!(h.active_hgt_source(), HeightSource::Gps);
        assert_eq!(h.fuse_sel(), HeightFuseSel::NotFusing);
        assert!(!h.fuse_performed());
        // residual = 12 + (-10) - 0 = 2 → offset += 0.1 * 2 = 0.2
        let err = h.baro_hgt_offset() - (0.2 as Ftype);
        assert!(err * err < (1.0e-12 as Ftype));
        assert!(h.baro_data_to_fuse());
    }

    #[test]
    fn baro_offset_spike_is_clamped() {
        let mut h = HeightFusion::new();
        h.set_baro_data(true, 40.0 as Ftype);
        h.set_position_d(0.0 as Ftype);
        h.calc_filt_baro_offset();
        let err = h.baro_hgt_offset() - (BARO_OFFSET_ALPHA * BARO_OFFSET_SPIKE_M);
        assert!(err * err < (1.0e-12 as Ftype));
    }

    #[test]
    fn read_pos_d_from_states_feeds_innovation() {
        let mut h = HeightFusion::new();
        let mut states = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        if let Some(slot) = states.get_mut(StateIndex::PosD.as_usize()) {
            *slot = -(15.0 as Ftype);
        }
        h.read_pos_d_from_states(&states);
        h.set_quality_overrides(false, false, false);
        h.set_baro_data(true, 15.0 as Ftype);
        h.set_imu_sample_time_ms(1000);
        h.select_height_fusion(true);
        let innov = h.innov_hgt();
        assert!(innov * innov < (1.0e-12 as Ftype));
        assert!(h.fuse_performed());
    }
}
