#![no_std]

//! Multirotor motor mixing factors, upstream `AP_Motors/AP_MotorsMatrix`.
//! COP-005.
//!
//! A multirotor has no control surfaces. Roll, pitch, yaw and thrust are all
//! produced by varying the speed of motors that are fixed to the airframe, so
//! every control demand has to be turned into a per-motor contribution. This
//! is the table that does it.
//!
//! # Where the factors come from
//!
//! A motor sits at some angle around the airframe, measured clockwise from
//! forward. Its contribution to pitch is `cos(θ)` — full forward at the nose,
//! nothing at the sides — and to roll is `cos(θ + 90°)`, the same curve turned
//! a quarter turn. Yaw does not follow from position at all: it comes from
//! which way the propeller spins, so it is supplied directly as +1 or −1.
//!
//! # Why they are then normalised
//!
//! The raw factors depend on how many motors there are and where. Scaling each
//! axis so its largest factor is exactly 0.5 means a full-scale roll demand
//! asks the same of a quad as of an octa, and the controller above does not
//! have to know the frame.
//!
//! # This slice
//!
//! The factor model: adding motors by angle or by raw factor, and normalising.
//! The frame *tables* — which angles belong to a quad X against a hexa plus
//! against a Y6 — are 1,400 lines of data in upstream and are their own slice,
//! as is the output stage that turns factors into PWM.

use ap_math::scalar::{is_zero, radians, Real};

pub mod arming;
pub mod current_limit;
mod frames;
pub mod output;
pub mod spool;
pub mod throttle;
pub mod thrust_linearization;

/// Motors a frame may have, upstream `AP_MOTORS_MAX_NUM_MOTORS`.
///
/// 32 where scripting is enabled and 12 where it is not, then clamped to the
/// number of servo channels. SITL enables scripting, so 32 is what the
/// reference build compiles and what the parity fixture measures.
///
/// The first version of this port took the 12 branch, and the fixture reported
/// 32 motors per frame on its first successful run — which is the whole reason
/// for having one.
pub const MAX_NUM_MOTORS: usize = 32;

/// One motor's contribution to each control axis.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MotorFactors {
    /// Contribution to roll.
    pub roll: f32,
    /// Contribution to pitch.
    pub pitch: f32,
    /// Contribution to yaw. Comes from propeller direction, not position.
    pub yaw: f32,
    /// Contribution to collective thrust. One for a normal motor; less for a
    /// tilted one that spends part of its thrust elsewhere.
    pub throttle: f32,
}

/// The per-motor mixing table, upstream `AP_MotorsMatrix`'s factor arrays.
#[derive(Debug, Clone, Copy)]
pub struct MotorMatrix {
    factors: [MotorFactors; MAX_NUM_MOTORS],
    enabled: [bool; MAX_NUM_MOTORS],
    test_order: [u8; MAX_NUM_MOTORS],
}

impl Default for MotorMatrix {
    fn default() -> Self {
        Self {
            factors: [MotorFactors::default(); MAX_NUM_MOTORS],
            enabled: [false; MAX_NUM_MOTORS],
            test_order: [0; MAX_NUM_MOTORS],
        }
    }
}

impl MotorMatrix {
    /// An empty frame with no motors.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a motor by explicit factors, upstream `add_motor_raw`.
    ///
    /// Out-of-range motor numbers are ignored, which is upstream's behaviour —
    /// it checks the bound and does nothing rather than reporting.
    pub fn add_motor_raw(
        &mut self,
        motor_num: i8,
        roll_fac: f32,
        pitch_fac: f32,
        yaw_fac: f32,
        testing_order: u8,
        throttle_factor: f32,
    ) {
        let Ok(i) = usize::try_from(motor_num) else {
            return;
        };
        if i >= MAX_NUM_MOTORS {
            return;
        }
        if let (Some(f), Some(e), Some(o)) = (
            self.factors.get_mut(i),
            self.enabled.get_mut(i),
            self.test_order.get_mut(i),
        ) {
            *e = true;
            *f = MotorFactors {
                roll: roll_fac,
                pitch: pitch_fac,
                yaw: yaw_fac,
                throttle: throttle_factor,
            };
            *o = testing_order;
        }
    }

    /// Add a motor at an angle, with separate roll and pitch angles. Upstream
    /// `add_motor(motor_num, roll_factor_in_degrees, pitch_factor_in_degrees,
    /// yaw_factor, testing_order)`.
    ///
    /// The two angles differ only on frames whose arms are not symmetric —
    /// most give the same value for both.
    pub fn add_motor_by_angles(
        &mut self,
        motor_num: i8,
        roll_factor_in_degrees: f32,
        pitch_factor_in_degrees: f32,
        yaw_factor: f32,
        testing_order: u8,
    ) {
        self.add_motor_raw(
            motor_num,
            Real::cos(radians(roll_factor_in_degrees + 90.0)),
            Real::cos(radians(pitch_factor_in_degrees)),
            yaw_factor,
            testing_order,
            1.0,
        );
    }

    /// Add a motor at an angle, upstream's three-argument `add_motor`.
    ///
    /// The common case: the same angle serves for both roll and pitch.
    pub fn add_motor(
        &mut self,
        motor_num: i8,
        angle_degrees: f32,
        yaw_factor: f32,
        testing_order: u8,
    ) {
        self.add_motor_by_angles(
            motor_num,
            angle_degrees,
            angle_degrees,
            yaw_factor,
            testing_order,
        );
    }

    /// Clear one motor, upstream `remove_motor`.
    ///
    /// The throttle factor goes to zero here, not to one. An empty slot is not
    /// "a motor that contributes nothing but full thrust" — it is not a motor,
    /// and `add_motor_raw` is what puts the factor back to one.
    pub fn remove_motor(&mut self, motor_num: usize) {
        let Some(factors) = self.factors.get_mut(motor_num) else {
            return;
        };
        *factors = MotorFactors {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
        };
        if let Some(enabled) = self.enabled.get_mut(motor_num) {
            *enabled = false;
        }
    }

    /// Build the mixing table for a frame, upstream `setup_motors`.
    ///
    /// Returns whether the class and type are a combination upstream supports.
    /// Both are plain integers because that is what `FRAME_CLASS` and
    /// `FRAME_TYPE` hold: a vehicle can be booted with a value outside the
    /// enum, and the Y6 class deliberately answers for every one of them.
    ///
    /// The normalisation runs whether or not the frame was recognised, which
    /// is upstream's order. On an unsupported frame there is nothing left to
    /// normalise, so it makes no difference today — but the order is kept
    /// because it is the order the reference has, and a frame that ever
    /// half-populates before failing should inherit that behaviour rather
    /// than a tidier one this port invented.
    pub fn setup_motors(&mut self, frame_class: u8, frame_type: u8) -> bool {
        for i in 0..MAX_NUM_MOTORS {
            self.remove_motor(i);
        }

        let Some(frame) = frames::layout(frame_class, frame_type) else {
            self.normalise_rpy_factors();
            return false;
        };

        match frame.layout {
            frames::Layout::Angle(rows) => {
                for (i, &(angle, yaw, order)) in rows.iter().enumerate() {
                    self.add_motor(i as i8, angle, yaw, order);
                }
            }
            frames::Layout::Raw(rows) => {
                for (i, &(roll, pitch, yaw, order)) in rows.iter().enumerate() {
                    self.add_motor_raw(i as i8, roll, pitch, yaw, order, 1.0);
                }
            }
            frames::Layout::ByAngles(rows) => {
                for &(num, roll, pitch, yaw, order) in rows {
                    self.add_motor_by_angles(num, roll, pitch, yaw, order);
                }
            }
        }

        if let Some((limit, step, scale)) = frame.top_layer_scale {
            // Single precision throughout. `0.9` in the C++ reads as a double
            // and would promote the multiply under ordinary rules, but
            // ArduPilot builds with `-fsingle-precision-constant`, so it is a
            // float. Promoting here puts this frame two ulp out.
            for f in self.factors.iter_mut().take(limit).step_by(step) {
                f.roll *= scale;
                f.pitch *= scale;
                f.yaw *= scale;
                f.throttle *= scale;
            }
        }

        self.normalise_rpy_factors();
        true
    }

    /// Scale each axis so its largest factor is 0.5, upstream
    /// `normalise_rpy_factors`.
    ///
    /// Each axis is scaled independently, so a frame with strong roll
    /// authority and weak yaw ends up with both at full scale — the mixer's
    /// job is to use what the frame has, not to preserve their ratio.
    ///
    /// An axis with no authority at all is left alone rather than divided by
    /// zero. Throttle is clamped non-negative: a negative collective
    /// contribution is not a thing a motor can do.
    pub fn normalise_rpy_factors(&mut self) {
        let mut roll_fac = 0.0_f32;
        let mut pitch_fac = 0.0_f32;
        let mut yaw_fac = 0.0_f32;
        let mut throttle_fac = 0.0_f32;

        for (f, &en) in self.factors.iter().zip(self.enabled.iter()) {
            if en {
                roll_fac = roll_fac.max(f.roll.abs());
                pitch_fac = pitch_fac.max(f.pitch.abs());
                yaw_fac = yaw_fac.max(f.yaw.abs());
                throttle_fac = throttle_fac.max(f.throttle.max(0.0));
            }
        }

        for (f, &en) in self.factors.iter_mut().zip(self.enabled.iter()) {
            if !en {
                continue;
            }
            if !is_zero(roll_fac) {
                f.roll = 0.5 * f.roll / roll_fac;
            }
            if !is_zero(pitch_fac) {
                f.pitch = 0.5 * f.pitch / pitch_fac;
            }
            if !is_zero(yaw_fac) {
                f.yaw = 0.5 * f.yaw / yaw_fac;
            }
            if !is_zero(throttle_fac) {
                f.throttle = (f.throttle / throttle_fac).max(0.0);
            }
        }
    }

    /// One motor's factors, or `None` if that motor is not fitted.
    #[must_use]
    pub fn motor(&self, i: usize) -> Option<MotorFactors> {
        if self.enabled.get(i).copied().unwrap_or(false) {
            self.factors.get(i).copied()
        } else {
            None
        }
    }

    /// Whether a motor is fitted.
    #[must_use]
    pub fn is_enabled(&self, i: usize) -> bool {
        self.enabled.get(i).copied().unwrap_or(false)
    }

    /// The order this motor is spun up in during a motor test.
    #[must_use]
    pub fn test_order(&self, i: usize) -> Option<u8> {
        if self.is_enabled(i) {
            self.test_order.get(i).copied()
        } else {
            None
        }
    }

    /// How many motors are fitted.
    #[must_use]
    pub fn num_motors(&self) -> usize {
        self.enabled.iter().filter(|&&e| e).count()
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "an axis with no authority must be left EXACTLY alone by \
normalisation; an epsilon would accept a scaling that should not have happened"
    )]

    use super::*;

    /// A quad in X: motors at the four diagonals, alternating spin.
    fn quad_x() -> MotorMatrix {
        let mut m = MotorMatrix::new();
        m.add_motor(0, 45.0, 1.0, 1);
        m.add_motor(1, -135.0, 1.0, 3);
        m.add_motor(2, -45.0, -1.0, 4);
        m.add_motor(3, 135.0, -1.0, 2);
        m
    }

    /// The angle convention: forward is pitch, right is roll, and the two are
    /// a quarter turn apart.
    #[test]
    fn the_angle_convention_puts_pitch_forward_and_roll_right() {
        let mut m = MotorMatrix::new();
        m.add_motor(0, 0.0, 1.0, 1); // dead ahead
        m.add_motor(1, 90.0, 1.0, 2); // dead right

        let front = m.motor(0).expect("fitted");
        assert!(
            (front.pitch - 1.0).abs() < 1e-6,
            "full pitch: {}",
            front.pitch
        );
        assert!(front.roll.abs() < 1e-6, "no roll: {}", front.roll);

        let right = m.motor(1).expect("fitted");
        assert!(right.pitch.abs() < 1e-6, "no pitch: {}", right.pitch);
        assert!(
            (right.roll + 1.0).abs() < 1e-6,
            "a motor on the right rolls left when it pushes: {}",
            right.roll
        );
    }

    /// Yaw comes from propeller direction, not from position — two motors at
    /// the same angle can have opposite yaw.
    #[test]
    fn yaw_is_independent_of_position() {
        let mut m = MotorMatrix::new();
        m.add_motor(0, 45.0, 1.0, 1);
        m.add_motor(1, 45.0, -1.0, 2);
        assert_eq!(m.motor(0).expect("fitted").yaw, 1.0);
        assert_eq!(m.motor(1).expect("fitted").yaw, -1.0);
        assert!(
            (m.motor(0).expect("fitted").roll - m.motor(1).expect("fitted").roll).abs() < 1e-6,
            "same angle, so the same roll factor"
        );
    }

    /// Normalisation puts every axis's largest factor at exactly 0.5, so a
    /// full-scale demand means the same on any frame.
    #[test]
    fn normalisation_scales_each_axis_to_a_half() {
        let mut m = quad_x();
        m.normalise_rpy_factors();

        let mut max_roll = 0.0_f32;
        let mut max_pitch = 0.0_f32;
        let mut max_yaw = 0.0_f32;
        for i in 0..MAX_NUM_MOTORS {
            if let Some(f) = m.motor(i) {
                max_roll = max_roll.max(f.roll.abs());
                max_pitch = max_pitch.max(f.pitch.abs());
                max_yaw = max_yaw.max(f.yaw.abs());
            }
        }
        assert!((max_roll - 0.5).abs() < 1e-6, "roll: {max_roll}");
        assert!((max_pitch - 0.5).abs() < 1e-6, "pitch: {max_pitch}");
        assert!((max_yaw - 0.5).abs() < 1e-6, "yaw: {max_yaw}");
    }

    /// A symmetric frame's factors sum to nothing on each axis: a pure roll
    /// demand does not also pitch or yaw the vehicle.
    #[test]
    fn a_symmetric_frame_has_no_cross_coupling() {
        let mut m = quad_x();
        m.normalise_rpy_factors();

        let (mut roll, mut pitch, mut yaw) = (0.0_f32, 0.0_f32, 0.0_f32);
        for i in 0..MAX_NUM_MOTORS {
            if let Some(f) = m.motor(i) {
                roll += f.roll;
                pitch += f.pitch;
                yaw += f.yaw;
            }
        }
        assert!(roll.abs() < 1e-5, "roll sum {roll}");
        assert!(pitch.abs() < 1e-5, "pitch sum {pitch}");
        assert!(yaw.abs() < 1e-5, "yaw sum {yaw}");
    }

    /// An axis with no authority is left alone rather than divided by zero.
    /// A coaxial pair with matched props has no yaw authority at all.
    #[test]
    fn an_axis_with_no_authority_is_not_scaled() {
        let mut m = MotorMatrix::new();
        m.add_motor_raw(0, 1.0, 0.0, 0.0, 1, 1.0);
        m.add_motor_raw(1, -1.0, 0.0, 0.0, 2, 1.0);
        m.normalise_rpy_factors();

        assert_eq!(m.motor(0).expect("fitted").yaw, 0.0, "left exactly alone");
        assert_eq!(m.motor(0).expect("fitted").pitch, 0.0);
        assert!((m.motor(0).expect("fitted").roll - 0.5).abs() < 1e-6);
    }

    /// Each axis is scaled independently: a frame with strong roll and weak
    /// yaw gets both at full scale, because the mixer's job is to use what the
    /// frame has.
    #[test]
    fn the_axes_are_scaled_independently() {
        let mut m = MotorMatrix::new();
        m.add_motor_raw(0, 10.0, 0.0, 0.1, 1, 1.0);
        m.add_motor_raw(1, -10.0, 0.0, -0.1, 2, 1.0);
        m.normalise_rpy_factors();

        assert!((m.motor(0).expect("fitted").roll - 0.5).abs() < 1e-6);
        assert!(
            (m.motor(0).expect("fitted").yaw - 0.5).abs() < 1e-6,
            "weak yaw is still scaled to full: {}",
            m.motor(0).expect("fitted").yaw
        );
    }

    /// Throttle is clamped non-negative — a motor cannot contribute negative
    /// collective thrust.
    #[test]
    fn a_negative_throttle_factor_is_clamped() {
        let mut m = MotorMatrix::new();
        m.add_motor_raw(0, 1.0, 0.0, 0.0, 1, 1.0);
        m.add_motor_raw(1, -1.0, 0.0, 0.0, 2, -0.5);
        m.normalise_rpy_factors();
        assert_eq!(m.motor(1).expect("fitted").throttle, 0.0);
    }

    /// Motors outside the array are ignored, as upstream does.
    #[test]
    fn out_of_range_motors_are_ignored() {
        let mut m = MotorMatrix::new();
        m.add_motor(-1, 45.0, 1.0, 1);
        m.add_motor(99, 45.0, 1.0, 1);
        assert_eq!(m.num_motors(), 0);
    }

    /// Test order is carried through, and only for fitted motors.
    #[test]
    fn the_test_order_is_recorded() {
        let m = quad_x();
        assert_eq!(m.test_order(0), Some(1));
        assert_eq!(m.test_order(3), Some(2));
        assert_eq!(m.test_order(4), None, "not fitted");
    }
}
