//! Output observer, upstream `AP_NavEKF3_Outputs.cpp`.
//!
//! Public getters that publish NE position, NED velocity, and Euler
//! angles from the 24-state vector:
//!
//! - [`OutputObserver::get_pos_ne`] is `NavEKF3_core::getPosNE`:
//!   `statesArray[7..8]` (North / East metres). Valid when aiding is
//!   not `AID_NONE`. IMU lever-arm `posOffsetNED`, public-origin
//!   translation, and the GPS / beacon fallbacks used in constant-
//!   position mode are not here.
//! - [`OutputObserver::get_vel_ned`] is `NavEKF3_core::getVelNED`:
//!   `statesArray[4..6]` (NED m/s). The complementary-filter
//!   `outputDataNew.velocity` copy and `velOffsetNED` are not here.
//! - [`OutputObserver::get_euler_angles`] is
//!   `NavEKF3_core::getEulerAngles`: quaternion `statesArray[0..3]`
//!   converted with `QuaternionT::to_euler`. AHRS trim subtraction
//!   (`dal.get_trim`) is not here.
//!
//! Complementary-filter output smoothing (`outputDataNew`) is not here.

use ap_math::quaternion::QuaternionT;
use ap_math::vector2::Vector2;
use ap_math::vector3::Vector3;
use ap_math::Ftype;

use crate::control::AidingMode;
use crate::{StateIndex, StateVector};

/// Published NE / NED / Euler view of the 24-state vector.
///
/// Upstream overlays these on `outputDataNew` after a complementary
/// filter. This stub reads the live `statesArray` slots directly.
#[derive(Debug, Clone, Copy)]
pub struct OutputObserver {
    /// Last NE position published by [`get_pos_ne`](Self::get_pos_ne).
    pos_ne: Vector2<Ftype>,
    /// Last NED velocity published by [`get_vel_ned`](Self::get_vel_ned).
    vel_ned: Vector3<Ftype>,
    /// Last Euler rpy published by [`get_euler_angles`](Self::get_euler_angles).
    euler: Vector3<Ftype>,
    /// Last `getPosNE` validity (`PV_AidingMode != AID_NONE`).
    pos_ne_valid: bool,
}

impl Default for OutputObserver {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputObserver {
    /// Zero outputs, invalid NE (the `AID_NONE` power-on default).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pos_ne: Vector2 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
            },
            vel_ned: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            euler: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            pos_ne_valid: false,
        }
    }

    /// Clear the last-published snapshot.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Last NE position (m), upstream `getPosNE` out-parameter.
    #[must_use]
    pub const fn pos_ne(&self) -> Vector2<Ftype> {
        self.pos_ne
    }

    /// Whether the last [`get_pos_ne`](Self::get_pos_ne) was valid.
    #[must_use]
    pub const fn pos_ne_valid(&self) -> bool {
        self.pos_ne_valid
    }

    /// Last NED velocity (m/s), upstream `getVelNED` out-parameter.
    #[must_use]
    pub const fn vel_ned(&self) -> Vector3<Ftype> {
        self.vel_ned
    }

    /// Last Euler roll / pitch / yaw (rad), upstream `getEulerAngles`.
    #[must_use]
    pub const fn euler(&self) -> Vector3<Ftype> {
        self.euler
    }

    /// Upstream `NavEKF3_core::getPosNE`.
    ///
    /// Writes North / East metres from `statesArray[7..8]`. Returns
    /// `true` when `aiding` is not [`AidingMode::None`], matching the
    /// `PV_AidingMode != AID_NONE` branch that trusts the EKF states.
    /// Constant-position GPS / beacon fallbacks are not here: the
    /// states are still published, but the return is `false`.
    pub fn get_pos_ne(
        &mut self,
        states: &StateVector,
        aiding: AidingMode,
    ) -> (Vector2<Ftype>, bool) {
        self.pos_ne = Vector2::new(
            read_axis(states, StateIndex::PosN),
            read_axis(states, StateIndex::PosE),
        );
        self.pos_ne_valid = aiding != AidingMode::None;
        (self.pos_ne, self.pos_ne_valid)
    }

    /// Upstream `NavEKF3_core::getVelNED`.
    ///
    /// Publishes NED velocity from `statesArray[4..6]`. IMU-offset
    /// correction (`velOffsetNED`) is not applied.
    pub fn get_vel_ned(&mut self, states: &StateVector) -> Vector3<Ftype> {
        self.vel_ned = Vector3::new(
            read_axis(states, StateIndex::VelN),
            read_axis(states, StateIndex::VelE),
            read_axis(states, StateIndex::VelD),
        );
        self.vel_ned
    }

    /// Upstream `NavEKF3_core::getEulerAngles`.
    ///
    /// Converts quaternion `statesArray[0..3]` with
    /// [`QuaternionT::to_euler`]. AHRS trim is not subtracted.
    pub fn get_euler_angles(&mut self, states: &StateVector) -> Vector3<Ftype> {
        let quat = QuaternionT::<Ftype>::new(
            read_axis(states, StateIndex::Quat0),
            read_axis(states, StateIndex::Quat1),
            read_axis(states, StateIndex::Quat2),
            read_axis(states, StateIndex::Quat3),
        );
        let (roll, pitch, yaw) = quat.to_euler();
        self.euler = Vector3::new(roll, pitch, yaw);
        self.euler
    }

    /// Write NE pos, NED vel, and a quaternion built from Euler rpy
    /// onto the 24-vector. Tests use this to reach the getters without
    /// a strapdown / fusion cycle.
    pub fn write_pose_into_states(
        states: &mut StateVector,
        pos_n: Ftype,
        pos_e: Ftype,
        vel_ned: Vector3<Ftype>,
        roll: Ftype,
        pitch: Ftype,
        yaw: Ftype,
    ) {
        write_axis(states, StateIndex::PosN, pos_n);
        write_axis(states, StateIndex::PosE, pos_e);
        write_axis(states, StateIndex::VelN, vel_ned.x);
        write_axis(states, StateIndex::VelE, vel_ned.y);
        write_axis(states, StateIndex::VelD, vel_ned.z);
        let quat = QuaternionT::<Ftype>::from_euler(roll, pitch, yaw);
        write_axis(states, StateIndex::Quat0, quat.q1);
        write_axis(states, StateIndex::Quat1, quat.q2);
        write_axis(states, StateIndex::Quat2, quat.q3);
        write_axis(states, StateIndex::Quat3, quat.q4);
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
    use crate::STATE_VECTOR_LEN;

    fn near(a: Ftype, b: Ftype) {
        let err = if a > b { a - b } else { b - a };
        assert!(err < 1.0e-5 as Ftype, "{a} !~= {b}");
    }

    fn near_vec2(got: Vector2<Ftype>, x: Ftype, y: Ftype) {
        near(got.x, x);
        near(got.y, y);
    }

    fn near_vec3(got: Vector3<Ftype>, x: Ftype, y: Ftype, z: Ftype) {
        near(got.x, x);
        near(got.y, y);
        near(got.z, z);
    }

    #[test]
    fn getters_publish_ne_pos_ned_vel_and_euler_from_states() {
        let mut states: StateVector = [0.0 as Ftype; STATE_VECTOR_LEN];
        let pos_n = 12.5 as Ftype;
        let pos_e = -4.0 as Ftype;
        let vel = Vector3::new(3.0 as Ftype, -1.5 as Ftype, 0.25 as Ftype);
        let roll = 0.10 as Ftype;
        let pitch = -0.20 as Ftype;
        let yaw = 0.30 as Ftype;
        OutputObserver::write_pose_into_states(
            &mut states, pos_n, pos_e, vel, roll, pitch, yaw,
        );

        let mut out = OutputObserver::new();
        let (ne, valid) = out.get_pos_ne(&states, AidingMode::Absolute);
        assert!(valid);
        near_vec2(ne, pos_n, pos_e);
        near_vec2(out.pos_ne(), pos_n, pos_e);
        assert!(out.pos_ne_valid());

        let ned = out.get_vel_ned(&states);
        near_vec3(ned, vel.x, vel.y, vel.z);
        near_vec3(out.vel_ned(), vel.x, vel.y, vel.z);

        let euler = out.get_euler_angles(&states);
        near_vec3(euler, roll, pitch, yaw);
        near_vec3(out.euler(), roll, pitch, yaw);

        // Constant-position mode still publishes the last NE, but the
        // estimate is not valid for flight control.
        let (ne_none, valid_none) = out.get_pos_ne(&states, AidingMode::None);
        assert!(!valid_none);
        near_vec2(ne_none, pos_n, pos_e);
        assert!(!out.pos_ne_valid());
    }
}
