//! Magnetometer fusion enable / yaw-reset, upstream `AP_NavEKF3_MagFusion.cpp`.
//!
//! This slice is the gate that `SelectMagFusion` evaluates before
//! `FuseMagnetometer`, plus the yaw-reset request that writes
//! `stateStruct.earth_magfield` (`magFieldEarth`). The Kalman update
//! that consumes the three magnetometer axes is not here.
//!
//! # Enable gate
//!
//! Upstream starts a fusion cycle only when
//! `magDataToFuse && statesInitialised && use_compass() && yawAlignComplete`.
//! A ready sample still does not run 3-axis fusion if magnetic field
//! states are inhibited, a field-state reset is pending, or the field
//! has not been initialised — those cases take `MagFuseSel::FUSE_YAW`
//! (`fuseEulerYaw`) instead of `FUSE_MAG`.
//!
//! # Yaw reset / magFieldEarth
//!
//! `controlMagYawReset` ORs an external `magYawResetRequest` with the
//! initial-alignment request (`tiltAlignComplete && !yawAlignComplete`).
//! When the request is live and the compass is in use it calls
//! `setYawFromMag` and, if the earth field has not been learned,
//! `resetMagFieldStates`. The field comes from the WMM table when
//! `have_table_earth_field && _mag_ef_limit > 0`, otherwise from the
//! delayed mag sample rotated into NED (identity DCM in this stub).

use ap_math::vector3::Vector3;
use ap_math::Ftype;

use crate::{StateIndex, StateVector};

/// Magnetometer fusion selection, upstream `NavEKF3_core::MagFuseSel`.
///
/// Discriminant values match the C++ enum so a sitl-diff dump can compare
/// the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagFuseSel {
    /// No magnetometer fusion this step. Upstream `NOT_FUSING`.
    NotFusing = 0,
    /// Simple declination / Euler yaw. Upstream `FUSE_YAW`.
    FuseYaw = 1,
    /// Sequential 3-axis `FuseMagnetometer`. Upstream `FUSE_MAG`.
    FuseMag = 2,
}

/// Earth-field clamp used by `ConstrainStates` when no WMM table is active, Gauss.
const EARTH_FIELD_LIMIT_GA: Ftype = 1.0;

/// Mag-fusion latch, the `NavEKF3_core` fields `SelectMagFusion` and
/// `controlMagYawReset` read and write.
///
/// Covariance and the algebraic `FuseMagnetometer` Jacobians are not here:
/// tests (and later cores) poke the compass / alignment / sample flags
/// that `SelectMagFusion` would have read from DAL.
#[derive(Debug, Clone)]
pub struct MagFusion {
    use_compass: bool,
    tilt_align_complete: bool,
    yaw_align_complete: bool,
    mag_data_to_fuse: bool,
    inhibit_mag_states: bool,
    mag_state_reset_request: bool,
    mag_state_init_complete: bool,
    mag_yaw_reset_request: bool,
    mag_field_learned: bool,
    mag_fuse_performed: bool,
    have_table_earth_field: bool,
    mag_ef_limit: i16,
    mag_fusion_sel: MagFuseSel,
    /// NED earth field (Gauss), upstream `stateStruct.earth_magfield`.
    mag_field_earth: Vector3<Ftype>,
    /// WMM table field (Gauss), upstream `table_earth_field_ga`.
    table_earth_field: Vector3<Ftype>,
    /// Delayed body-frame mag sample (Gauss), upstream `magDataDelayed.mag`.
    mag_body: Vector3<Ftype>,
}

impl Default for MagFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl MagFusion {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// Compass assumed available (Plane default), mag states inhibited,
    /// yaw not aligned, `magFusionSel = NOT_FUSING`, earth field zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            use_compass: true,
            tilt_align_complete: false,
            yaw_align_complete: false,
            mag_data_to_fuse: false,
            inhibit_mag_states: true,
            mag_state_reset_request: false,
            mag_state_init_complete: false,
            mag_yaw_reset_request: false,
            mag_field_learned: false,
            mag_fuse_performed: false,
            have_table_earth_field: false,
            mag_ef_limit: 0,
            mag_fusion_sel: MagFuseSel::NotFusing,
            mag_field_earth: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            table_earth_field: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            mag_body: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Whether the compass is a yaw source, upstream `use_compass()`.
    #[must_use]
    pub const fn use_compass(&self) -> bool {
        self.use_compass
    }

    /// Whether tilt alignment has finished, upstream `tiltAlignComplete`.
    #[must_use]
    pub const fn tilt_align_complete(&self) -> bool {
        self.tilt_align_complete
    }

    /// Whether yaw alignment has finished, upstream `yawAlignComplete`.
    #[must_use]
    pub const fn yaw_align_complete(&self) -> bool {
        self.yaw_align_complete
    }

    /// Delayed mag sample is at the fusion horizon, upstream `magDataToFuse`.
    #[must_use]
    pub const fn mag_data_to_fuse(&self) -> bool {
        self.mag_data_to_fuse
    }

    /// Magnetic field states are inactive, upstream `inhibitMagStates`.
    #[must_use]
    pub const fn inhibit_mag_states(&self) -> bool {
        self.inhibit_mag_states
    }

    /// Pending field-state reset, upstream `magStateResetRequest`.
    #[must_use]
    pub const fn mag_state_reset_request(&self) -> bool {
        self.mag_state_reset_request
    }

    /// Field states have been initialised, upstream `magStateInitComplete`.
    #[must_use]
    pub const fn mag_state_init_complete(&self) -> bool {
        self.mag_state_init_complete
    }

    /// Combined yaw / field reset request, upstream `magYawResetRequest`.
    #[must_use]
    pub const fn mag_yaw_reset_request(&self) -> bool {
        self.mag_yaw_reset_request
    }

    /// Earth field has been learned in flight, upstream `magFieldLearned`.
    #[must_use]
    pub const fn mag_field_learned(&self) -> bool {
        self.mag_field_learned
    }

    /// Expensive 3-axis fusion ran this step, upstream `magFusePerformed`.
    #[must_use]
    pub const fn mag_fuse_performed(&self) -> bool {
        self.mag_fuse_performed
    }

    /// Fusion selection for this step, upstream `magFusionSel`.
    #[must_use]
    pub const fn mag_fusion_sel(&self) -> MagFuseSel {
        self.mag_fusion_sel
    }

    /// NED earth field (Gauss), upstream `stateStruct.earth_magfield`.
    #[must_use]
    pub const fn mag_field_earth(&self) -> Vector3<Ftype> {
        self.mag_field_earth
    }

    /// WMM table is populated, upstream `have_table_earth_field`.
    #[must_use]
    pub const fn have_table_earth_field(&self) -> bool {
        self.have_table_earth_field
    }

    /// Enable or disable the compass yaw source.
    pub fn set_use_compass(&mut self, use_compass: bool) {
        self.use_compass = use_compass;
    }

    /// Poke `tiltAlignComplete`.
    pub fn set_tilt_align_complete(&mut self, complete: bool) {
        self.tilt_align_complete = complete;
    }

    /// Poke `yawAlignComplete`.
    pub fn set_yaw_align_complete(&mut self, complete: bool) {
        self.yaw_align_complete = complete;
    }

    /// Poke `magDataToFuse` and the delayed body-frame sample.
    pub fn set_mag_data(&mut self, ready: bool, mag_body: Vector3<Ftype>) {
        self.mag_data_to_fuse = ready;
        self.mag_body = mag_body;
    }

    /// Poke `inhibitMagStates`.
    pub fn set_inhibit_mag_states(&mut self, inhibit: bool) {
        self.inhibit_mag_states = inhibit;
    }

    /// Poke `magStateInitComplete`.
    pub fn set_mag_state_init_complete(&mut self, complete: bool) {
        self.mag_state_init_complete = complete;
    }

    /// Install a WMM table field and `_mag_ef_limit`, upstream
    /// `table_earth_field_ga` / `have_table_earth_field`.
    pub fn set_table_earth_field(&mut self, field: Vector3<Ftype>, mag_ef_limit: i16) {
        self.table_earth_field = field;
        self.have_table_earth_field = true;
        self.mag_ef_limit = mag_ef_limit;
    }

    /// External yaw-reset request, upstream `magYawResetRequest = true`.
    pub fn request_yaw_reset(&mut self) {
        self.mag_yaw_reset_request = true;
    }

    /// Request a field-state-only reset, upstream `magStateResetRequest`.
    pub fn request_mag_state_reset(&mut self) {
        self.mag_state_reset_request = true;
    }

    /// `SelectMagFusion` data-ready predicate.
    ///
    /// `magDataToFuse && statesInitialised && use_compass() && yawAlignComplete`.
    #[must_use]
    pub const fn mag_data_ready(&self, states_initialised: bool) -> bool {
        self.mag_data_to_fuse && states_initialised && self.use_compass && self.yaw_align_complete
    }

    /// Upstream `NavEKF3_core::SelectMagFusion` enable gate.
    ///
    /// Clears `magFusePerformed`, runs [`control_mag_yaw_reset`] when a
    /// delayed sample is present, then picks `FUSE_YAW` / `FUSE_MAG` or
    /// stays `NOT_FUSING`. The 3-axis Jacobian update is a performed-flag
    /// stub — see [`fuse_magnetometer`](Self::fuse_magnetometer).
    pub fn select_mag_fusion(&mut self, states_initialised: bool) {
        self.mag_fuse_performed = false;
        self.mag_fusion_sel = MagFuseSel::NotFusing;

        if self.mag_data_to_fuse {
            self.control_mag_yaw_reset();
        }

        if !self.mag_data_ready(states_initialised) {
            return;
        }

        if self.inhibit_mag_states || self.mag_state_reset_request || !self.mag_state_init_complete
        {
            self.mag_fusion_sel = MagFuseSel::FuseYaw;
            return;
        }

        self.mag_fusion_sel = MagFuseSel::FuseMag;
        self.fuse_magnetometer();
    }

    /// Upstream `NavEKF3_core::controlMagYawReset` (request OR + compass gate).
    ///
    /// The Plane GPS-velocity recovery path (`assume_zero_sideslip` /
    /// `gpsYawResetRequest`) and the climb-away anomaly interim reset
    /// are not here. This stub keeps the external request, the initial
    /// alignment request, and the `setYawFromMag` / `resetMagFieldStates`
    /// pair that writes `magFieldEarth`.
    pub fn control_mag_yaw_reset(&mut self) {
        let initial_reset_request = self.tilt_align_complete && !self.yaw_align_complete;
        self.mag_yaw_reset_request = self.mag_yaw_reset_request || initial_reset_request;

        if !(self.mag_yaw_reset_request && self.use_compass) {
            return;
        }

        self.set_yaw_from_mag();
        if !self.mag_field_learned {
            self.reset_mag_field_states();
        }
        self.record_yaw_resets_completed();
    }

    /// Upstream `NavEKF3_core::setYawFromMag`.
    ///
    /// The quaternion yaw rewrite (`resetQuatStateYawOnly`) is not here:
    /// this stub refuses when the compass is unused, otherwise the
    /// subsequent [`record_yaw_resets_completed`] latch is what tests
    /// observe.
    pub fn set_yaw_from_mag(&mut self) {
        if !self.use_compass {
            return;
        }
        // Quaternion yaw from `magDataDelayed` / declination is a later slice.
    }

    /// Upstream `NavEKF3_core::resetMagFieldStates`.
    ///
    /// Writes `earth_magfield` from the WMM table when the table is
    /// armed (`have_table_earth_field && _mag_ef_limit > 0`), otherwise
    /// from the delayed body sample (identity body→NED, the bootstrap
    /// quaternion). Then [`record_mag_reset`].
    pub fn reset_mag_field_states(&mut self) {
        if self.have_table_earth_field && self.mag_ef_limit > 0 {
            self.mag_field_earth = self.table_earth_field;
        } else {
            self.mag_field_earth = self.mag_body;
        }
        self.constrain_earth_field();
        self.record_mag_reset();
    }

    /// Earth-field half of `ConstrainStates` when no table clamp is active.
    fn constrain_earth_field(&mut self) {
        if self.have_table_earth_field && self.mag_ef_limit > 0 {
            return;
        }
        self.mag_field_earth.x = constrain_ga(self.mag_field_earth.x);
        self.mag_field_earth.y = constrain_ga(self.mag_field_earth.y);
        self.mag_field_earth.z = constrain_ga(self.mag_field_earth.z);
    }

    /// Upstream `NavEKF3_core::recordMagReset`.
    pub fn record_mag_reset(&mut self) {
        self.mag_state_reset_request = false;
        self.mag_state_init_complete = true;
    }

    /// Upstream `NavEKF3_core::recordYawResetsCompleted`.
    pub fn record_yaw_resets_completed(&mut self) {
        self.mag_yaw_reset_request = false;
        self.yaw_align_complete = true;
    }

    /// Upstream `NavEKF3_core::FuseMagnetometer` enable-side stub.
    ///
    /// Sets `magFusePerformed`. The sequential-axis Jacobians and Kalman
    /// gain are not here.
    pub fn fuse_magnetometer(&mut self) {
        self.mag_fuse_performed = true;
    }

    /// Copy `earth_magfield` onto `statesArray[16..18]`.
    pub fn write_earth_into_states(&self, states: &mut StateVector) {
        write_axis(states, StateIndex::EarthMagN, self.mag_field_earth.x);
        write_axis(states, StateIndex::EarthMagE, self.mag_field_earth.y);
        write_axis(states, StateIndex::EarthMagD, self.mag_field_earth.z);
    }

    /// Read `statesArray[16..18]` into the stored earth field.
    pub fn read_earth_from_states(&mut self, states: &StateVector) {
        self.mag_field_earth.x = read_axis(states, StateIndex::EarthMagN);
        self.mag_field_earth.y = read_axis(states, StateIndex::EarthMagE);
        self.mag_field_earth.z = read_axis(states, StateIndex::EarthMagD);
    }
}

fn constrain_ga(value: Ftype) -> Ftype {
    if value > EARTH_FIELD_LIMIT_GA {
        EARTH_FIELD_LIMIT_GA
    } else if value < -EARTH_FIELD_LIMIT_GA {
        -EARTH_FIELD_LIMIT_GA
    } else {
        value
    }
}

fn write_axis(states: &mut StateVector, index: StateIndex, value: Ftype) {
    if let Some(slot) = states.get_mut(index.as_usize()) {
        *slot = value;
    }
}

fn read_axis(states: &StateVector, index: StateIndex) -> Ftype {
    match states.get(index.as_usize()) {
        Some(&value) => value,
        None => 0.0 as Ftype,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Ftype, b: Ftype) {
        let err = if a > b { a - b } else { b - a };
        assert!(err < 1.0e-6 as Ftype, "{a} !~= {b}");
    }

    fn sample() -> Vector3<Ftype> {
        Vector3::new(0.22 as Ftype, 0.05 as Ftype, 0.41 as Ftype)
    }

    fn ready_for_yaw(mag: &mut MagFusion) {
        mag.set_use_compass(true);
        mag.set_tilt_align_complete(true);
        mag.set_yaw_align_complete(true);
        mag.set_mag_data(true, sample());
    }

    #[test]
    fn enable_gate_refuses_until_compass_aligned_and_sample_ready() {
        let mut mag = MagFusion::new();
        mag.select_mag_fusion(true);
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::NotFusing);
        assert!(!mag.mag_fuse_performed());

        mag.set_mag_data(true, sample());
        mag.select_mag_fusion(true);
        // Sample present but yaw not aligned and tilt still false: the
        // enable gate stays closed (no initial-alignment request).
        assert!(!mag.yaw_align_complete());
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::NotFusing);

        mag.set_tilt_align_complete(true);
        mag.set_use_compass(false);
        mag.select_mag_fusion(true);
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::NotFusing);

        mag.set_use_compass(true);
        mag.select_mag_fusion(false);
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::NotFusing);
    }

    #[test]
    fn ready_sample_with_inhibited_field_selects_fuse_yaw() {
        let mut mag = MagFusion::new();
        ready_for_yaw(&mut mag);
        // Bootstrap leaves inhibitMagStates true and magStateInitComplete false.
        mag.select_mag_fusion(true);
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::FuseYaw);
        assert!(!mag.mag_fuse_performed());
    }

    #[test]
    fn fuse_magnetometer_runs_only_when_field_states_are_active() {
        let mut mag = MagFusion::new();
        ready_for_yaw(&mut mag);
        mag.set_inhibit_mag_states(false);
        mag.set_mag_state_init_complete(true);
        mag.select_mag_fusion(true);
        assert_eq!(mag.mag_fusion_sel(), MagFuseSel::FuseMag);
        assert!(mag.mag_fuse_performed());
    }

    #[test]
    fn yaw_reset_request_writes_mag_field_earth_from_body_sample() {
        let mut mag = MagFusion::new();
        mag.set_use_compass(true);
        mag.set_tilt_align_complete(true);
        mag.set_mag_data(true, sample());
        mag.request_yaw_reset();
        mag.control_mag_yaw_reset();

        assert!(mag.yaw_align_complete());
        assert!(!mag.mag_yaw_reset_request());
        assert!(mag.mag_state_init_complete());
        let earth = mag.mag_field_earth();
        near(earth.x, 0.22 as Ftype);
        near(earth.y, 0.05 as Ftype);
        near(earth.z, 0.41 as Ftype);
    }

    #[test]
    fn yaw_reset_prefers_table_earth_field_when_limit_armed() {
        let mut mag = MagFusion::new();
        mag.set_use_compass(true);
        mag.set_tilt_align_complete(true);
        mag.set_mag_data(true, sample());
        mag.set_table_earth_field(
            Vector3::new(0.18 as Ftype, 0.04 as Ftype, 0.50 as Ftype),
            50,
        );
        mag.request_yaw_reset();
        mag.control_mag_yaw_reset();

        let earth = mag.mag_field_earth();
        near(earth.x, 0.18 as Ftype);
        near(earth.y, 0.04 as Ftype);
        near(earth.z, 0.50 as Ftype);
    }

    #[test]
    fn yaw_reset_without_compass_does_not_write_earth_field() {
        let mut mag = MagFusion::new();
        mag.set_use_compass(false);
        mag.set_tilt_align_complete(true);
        mag.set_mag_data(true, sample());
        mag.request_yaw_reset();
        mag.control_mag_yaw_reset();

        assert!(mag.mag_yaw_reset_request());
        assert!(!mag.yaw_align_complete());
        let earth = mag.mag_field_earth();
        near(earth.x, 0.0 as Ftype);
        near(earth.y, 0.0 as Ftype);
        near(earth.z, 0.0 as Ftype);
    }

    #[test]
    fn write_and_read_round_trip_states_16_to_18() {
        let mut mag = MagFusion::new();
        mag.set_mag_data(true, sample());
        mag.reset_mag_field_states();
        let mut states: StateVector = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        mag.write_earth_into_states(&mut states);
        near(
            match states.get(StateIndex::EarthMagN.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.22 as Ftype,
        );
        near(
            match states.get(StateIndex::EarthMagD.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.41 as Ftype,
        );

        let mut copy = MagFusion::new();
        copy.read_earth_from_states(&states);
        near(copy.mag_field_earth().x, 0.22 as Ftype);
        near(copy.mag_field_earth().y, 0.05 as Ftype);
        near(copy.mag_field_earth().z, 0.41 as Ftype);
    }
}
