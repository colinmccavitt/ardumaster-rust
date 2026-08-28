//! Wind and earth-magnetic-field constrain, upstream `ConstrainStates`
//! (states 16..18 and 22..23) plus `MagTableConstrain`.
//!
//! Earth-frame magnetic field lives in states 16..18 (Gauss). Wind
//! velocity lives in states 22..23 (m/s, North / East). This slice is
//! the clamp path that `UpdateStrapdownEquationsNED` runs after the
//! delayed IMU integrate:
//!
//! - [`WindMagField::constrain`] is the earth-mag + wind half of
//!   `ConstrainStates`. With no WMM table (or `_mag_ef_limit <= 0`)
//!   each earth-field axis is clamped to `±1.0` Ga. When a table is
//!   armed, [`WindMagField::mag_table_constrain`] clamps each axis to
//!   `table_earth_field_ga ± _mag_ef_limit * 0.001`. Wind is always
//!   clamped to `±100` m/s.
//! - [`WindMagField::reset`] zeros both groups, matching
//!   `InitialiseVariables` (`stateStruct.earth_magfield.zero()` /
//!   `stateStruct.wind_vel.zero()`).
//!
//! Covariance growth, TAS / sideslip fusion that *learn* wind, and the
//! 3-axis mag Kalman update are not here. Gyro / accel bias constrain
//! stay in [`crate::gyro_bias`] / [`crate::accel_bias`].

use ap_math::scalar::constrain_value;
use ap_math::vector2::Vector2;
use ap_math::vector3::Vector3;
use ap_math::Ftype;

use crate::{StateIndex, StateVector};

/// Earth-field clamp when no WMM table is active, Gauss.
///
/// Upstream `ConstrainStates` `[-1.0, 1.0]` for states 16..18.
pub const EARTH_FIELD_LIMIT_GA: Ftype = 1.0;

/// Wind-velocity clamp, m/s.
///
/// Upstream `ConstrainStates` `[-100, 100]` for states 22..23.
pub const WIND_VEL_LIMIT_MPS: Ftype = 100.0;

/// Plane `EK3_MAG_EF_LIM` default (milliGauss).
pub const MAG_EF_LIMIT_DEFAULT: i16 = 50;

/// `_mag_ef_limit` is stored in milliGauss; table clamp uses Gauss.
const MAG_EF_LIMIT_TO_GA: Ftype = 0.001;

/// Earth-field (Gauss) and NE wind (m/s) plus the WMM table clamp.
///
/// Upstream overlays these on `statesArray[16..18]` and
/// `statesArray[22..23]`. The port keeps local vectors so constrain
/// can run without a covariance matrix;
/// [`WindMagField::write_into_states`] copies back onto the 24-vector.
#[derive(Debug, Clone, Copy)]
pub struct WindMagField {
    /// NED earth field (Gauss), upstream `stateStruct.earth_magfield`.
    earth_mag: Vector3<Ftype>,
    /// NE wind (m/s), upstream `stateStruct.wind_vel`.
    wind: Vector2<Ftype>,
    /// WMM table is populated, upstream `have_table_earth_field`.
    have_table_earth_field: bool,
    /// Allowed error from the table (milliGauss), upstream `_mag_ef_limit`.
    mag_ef_limit: i16,
    /// WMM table field (Gauss), upstream `table_earth_field_ga`.
    table_earth_field: Vector3<Ftype>,
}

impl Default for WindMagField {
    fn default() -> Self {
        Self::new()
    }
}

impl WindMagField {
    /// Zero field / wind, no WMM table, Plane `EK3_MAG_EF_LIM` default.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            earth_mag: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            wind: Vector2 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
            },
            have_table_earth_field: false,
            mag_ef_limit: MAG_EF_LIMIT_DEFAULT,
            table_earth_field: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
        }
    }

    /// Re-apply bootstrap zeros and drop the WMM table latch.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// NED earth field (Gauss), upstream `stateStruct.earth_magfield`.
    #[must_use]
    pub const fn earth_mag(&self) -> Vector3<Ftype> {
        self.earth_mag
    }

    /// NE wind (m/s), upstream `stateStruct.wind_vel`.
    #[must_use]
    pub const fn wind(&self) -> Vector2<Ftype> {
        self.wind
    }

    /// WMM table is populated, upstream `have_table_earth_field`.
    #[must_use]
    pub const fn have_table_earth_field(&self) -> bool {
        self.have_table_earth_field
    }

    /// Allowed error from the table (milliGauss), upstream `_mag_ef_limit`.
    #[must_use]
    pub const fn mag_ef_limit(&self) -> i16 {
        self.mag_ef_limit
    }

    /// WMM table field (Gauss), upstream `table_earth_field_ga`.
    #[must_use]
    pub const fn table_earth_field(&self) -> Vector3<Ftype> {
        self.table_earth_field
    }

    /// Poke `earth_magfield` (tests / later fusion write-back).
    pub fn set_earth_mag(&mut self, field: Vector3<Ftype>) {
        self.earth_mag = field;
    }

    /// Poke `wind_vel` (tests / later TAS write-back).
    pub fn set_wind(&mut self, wind: Vector2<Ftype>) {
        self.wind = wind;
    }

    /// Install a WMM table field and `_mag_ef_limit`.
    ///
    /// Arms `have_table_earth_field`. A non-positive limit keeps
    /// [`Self::constrain`] on the ±1 Ga fallback, matching
    /// `frontend->_mag_ef_limit <= 0 || !have_table_earth_field`.
    pub fn set_table_earth_field(&mut self, field: Vector3<Ftype>, mag_ef_limit: i16) {
        self.table_earth_field = field;
        self.have_table_earth_field = true;
        self.mag_ef_limit = mag_ef_limit;
    }

    /// Earth-mag + wind half of upstream `ConstrainStates`.
    ///
    /// Earth field uses [`Self::mag_table_constrain`] when the WMM table
    /// is armed and `_mag_ef_limit > 0`; otherwise each axis is clamped
    /// to ±[`EARTH_FIELD_LIMIT_GA`]. Wind is always ±[`WIND_VEL_LIMIT_MPS`].
    pub fn constrain(&mut self) {
        if self.have_table_earth_field && self.mag_ef_limit > 0 {
            self.mag_table_constrain();
        } else {
            self.earth_mag.x = constrain_value(
                self.earth_mag.x,
                -EARTH_FIELD_LIMIT_GA,
                EARTH_FIELD_LIMIT_GA,
            );
            self.earth_mag.y = constrain_value(
                self.earth_mag.y,
                -EARTH_FIELD_LIMIT_GA,
                EARTH_FIELD_LIMIT_GA,
            );
            self.earth_mag.z = constrain_value(
                self.earth_mag.z,
                -EARTH_FIELD_LIMIT_GA,
                EARTH_FIELD_LIMIT_GA,
            );
        }
        self.wind.x = constrain_value(self.wind.x, -WIND_VEL_LIMIT_MPS, WIND_VEL_LIMIT_MPS);
        self.wind.y = constrain_value(self.wind.y, -WIND_VEL_LIMIT_MPS, WIND_VEL_LIMIT_MPS);
    }

    /// Upstream `NavEKF3_core::MagTableConstrain`.
    ///
    /// `limit_ga = _mag_ef_limit * 0.001`; each earth-field axis is
    /// clamped to the table value ± that limit.
    pub fn mag_table_constrain(&mut self) {
        let limit_ga = Ftype::from(self.mag_ef_limit) * MAG_EF_LIMIT_TO_GA;
        self.earth_mag.x = constrain_value(
            self.earth_mag.x,
            self.table_earth_field.x - limit_ga,
            self.table_earth_field.x + limit_ga,
        );
        self.earth_mag.y = constrain_value(
            self.earth_mag.y,
            self.table_earth_field.y - limit_ga,
            self.table_earth_field.y + limit_ga,
        );
        self.earth_mag.z = constrain_value(
            self.earth_mag.z,
            self.table_earth_field.z - limit_ga,
            self.table_earth_field.z + limit_ga,
        );
    }

    /// Copy earth-mag / wind onto `statesArray[16..18]` and `[22..23]`.
    pub fn write_into_states(&self, states: &mut StateVector) {
        write_axis(states, StateIndex::EarthMagN, self.earth_mag.x);
        write_axis(states, StateIndex::EarthMagE, self.earth_mag.y);
        write_axis(states, StateIndex::EarthMagD, self.earth_mag.z);
        write_axis(states, StateIndex::WindVelN, self.wind.x);
        write_axis(states, StateIndex::WindVelE, self.wind.y);
    }

    /// Read `statesArray[16..18]` / `[22..23]` into the stored field / wind.
    pub fn read_from_states(&mut self, states: &StateVector) {
        self.earth_mag.x = read_axis(states, StateIndex::EarthMagN);
        self.earth_mag.y = read_axis(states, StateIndex::EarthMagE);
        self.earth_mag.z = read_axis(states, StateIndex::EarthMagD);
        self.wind.x = read_axis(states, StateIndex::WindVelN);
        self.wind.y = read_axis(states, StateIndex::WindVelE);
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

    #[test]
    fn reset_zeros_earth_mag_and_wind() {
        let mut field = WindMagField::new();
        field.set_earth_mag(Vector3::new(0.4 as Ftype, -0.1 as Ftype, 0.5 as Ftype));
        field.set_wind(Vector2::new(12.0 as Ftype, -8.0 as Ftype));
        field.set_table_earth_field(
            Vector3::new(0.2 as Ftype, 0.05 as Ftype, 0.4 as Ftype),
            MAG_EF_LIMIT_DEFAULT,
        );
        field.reset();

        let earth = field.earth_mag();
        near(earth.x, 0.0 as Ftype);
        near(earth.y, 0.0 as Ftype);
        near(earth.z, 0.0 as Ftype);
        near(field.wind().x, 0.0 as Ftype);
        near(field.wind().y, 0.0 as Ftype);
        assert!(!field.have_table_earth_field());
        assert_eq!(field.mag_ef_limit(), MAG_EF_LIMIT_DEFAULT);
    }

    #[test]
    fn constrain_clamps_earth_mag_to_one_ga_without_table() {
        let mut field = WindMagField::new();
        field.set_earth_mag(Vector3::new(2.5 as Ftype, -(3.0 as Ftype), 0.4 as Ftype));
        field.constrain();

        let earth = field.earth_mag();
        near(earth.x, EARTH_FIELD_LIMIT_GA);
        near(earth.y, -EARTH_FIELD_LIMIT_GA);
        near(earth.z, 0.4 as Ftype);
    }

    #[test]
    fn constrain_uses_wmm_table_when_armed() {
        let mut field = WindMagField::new();
        let table = Vector3::new(0.22 as Ftype, 0.05 as Ftype, 0.41 as Ftype);
        field.set_table_earth_field(table, MAG_EF_LIMIT_DEFAULT);
        field.set_earth_mag(Vector3::new(2.0 as Ftype, -(2.0 as Ftype), 0.41 as Ftype));
        field.constrain();

        let limit_ga = Ftype::from(MAG_EF_LIMIT_DEFAULT) * MAG_EF_LIMIT_TO_GA;
        let earth = field.earth_mag();
        near(earth.x, table.x + limit_ga);
        near(earth.y, table.y - limit_ga);
        near(earth.z, table.z);
    }

    #[test]
    fn constrain_falls_back_to_one_ga_when_table_limit_non_positive() {
        let mut field = WindMagField::new();
        field.set_table_earth_field(Vector3::new(0.22 as Ftype, 0.05 as Ftype, 0.41 as Ftype), 0);
        field.set_earth_mag(Vector3::new(1.8 as Ftype, -(1.5 as Ftype), 0.2 as Ftype));
        field.constrain();

        let earth = field.earth_mag();
        near(earth.x, EARTH_FIELD_LIMIT_GA);
        near(earth.y, -EARTH_FIELD_LIMIT_GA);
        near(earth.z, 0.2 as Ftype);
    }

    #[test]
    fn constrain_clamps_wind_to_one_hundred_mps() {
        let mut field = WindMagField::new();
        field.set_wind(Vector2::new(150.0 as Ftype, -(180.0 as Ftype)));
        field.constrain();
        near(field.wind().x, WIND_VEL_LIMIT_MPS);
        near(field.wind().y, -WIND_VEL_LIMIT_MPS);

        field.set_wind(Vector2::new(40.0 as Ftype, -(12.0 as Ftype)));
        field.constrain();
        near(field.wind().x, 40.0 as Ftype);
        near(field.wind().y, -(12.0 as Ftype));
    }

    #[test]
    fn write_and_read_round_trip_states_16_to_18_and_22_23() {
        let mut field = WindMagField::new();
        field.set_earth_mag(Vector3::new(0.22 as Ftype, 0.05 as Ftype, 0.41 as Ftype));
        field.set_wind(Vector2::new(7.0 as Ftype, -3.0 as Ftype));
        let mut states: StateVector = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        field.write_into_states(&mut states);
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
        near(
            match states.get(StateIndex::WindVelN.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            7.0 as Ftype,
        );
        near(
            match states.get(StateIndex::WindVelE.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            -3.0 as Ftype,
        );

        let mut round = WindMagField::new();
        round.read_from_states(&states);
        near(round.earth_mag().x, 0.22 as Ftype);
        near(round.earth_mag().y, 0.05 as Ftype);
        near(round.earth_mag().z, 0.41 as Ftype);
        near(round.wind().x, 7.0 as Ftype);
        near(round.wind().y, -3.0 as Ftype);
    }
}
