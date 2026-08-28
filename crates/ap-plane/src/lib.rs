//! Vehicle glue: the fixed-wing roll demand path. FW-025, first slice.
//!
//! This is where `ArduPlane` joins navigation to attitude control. L1 produces
//! a bank angle, the vehicle limits it, subtracts the measured roll, and hands
//! the difference to the roll controller as an angle error.
//!
//! It is three lines of arithmetic, and it exists as a crate rather than as
//! test scaffolding for one reason: it is the first place two ported modules
//! meet. Every crate so far has been verified alone. Composition is the thing
//! that has not been tested, and it cannot be tested from inside either half.

#![no_std]

use ap_math::scalar::constrain_int32;

pub mod ahrs_hookup;
pub mod altitude_glue_hookup;
pub mod set_servos_glue_hookup;
pub mod altitude_tecs_feed_hookup;
pub mod tecs_baro_hookup;
pub mod ahrs_pre_arm_hookup;
pub mod gps_pre_arm_hookup;
pub mod baro_pre_arm_hookup;
pub mod compass_pre_arm_hookup;
pub mod airspeed_pre_arm_hookup;
pub mod airspeed_health_scheduler_hookup;
pub mod airspeed_offset_calibration_hookup;
pub mod compass_health_scheduler_hookup;
pub mod compass_offset_calibration_hookup;
pub mod compass_motor_compensation_hookup;
pub mod baro_arm_calibration_hookup;
pub mod arming_scheduler_hookup;
pub mod ins_hntch_scheduler_hookup;
pub mod sitl_ins_host_files;
pub mod sitl_ins_noise_hookup;
pub mod sitl_yaw_hookup;
pub mod sitl_gps_hookup;
pub mod sitl_ahrs_hookup;
pub mod sitl_baro_hookup;
pub mod sitl_compass_hookup;
pub mod sitl_airspeed_hookup;
pub mod nav_tecs_hookup;
pub mod nav_tecs_scheduler_hookup;
pub mod navigation_scheduler_hookup;
pub mod calc_throttle_glue_hookup;
pub mod entry_state;
pub mod landing_hookup;
pub mod deepstall_override_scheduler_hookup;
pub mod go_around_hookup;
pub mod landing_loop;
pub mod landing_loop_hookup;
pub mod landing_throttle_scheduler_hookup;
pub mod main_loop;
pub mod rangefinder_bump_hookup;
pub mod rangefinder_bump_scheduler_hookup;
pub mod rc_failsafe_scheduler_hookup;
pub mod mission_alt_offset_glue_hookup;
pub mod rangefinder_correction_glue_hookup;
pub mod mission_scheduler_hookup;
pub mod mode;
pub mod mode_run;
pub mod mode_table;
pub mod mode_entry_scheduler_hookup;
pub mod mode_glue_hookup;
pub mod manual_mode_hookup;
pub mod fbwa_mode_hookup;
pub mod stabilize_mode_hookup;
pub mod acro_mode_hookup;
pub mod training_mode_hookup;
pub mod mode_transition_throttle_hookup;
pub mod mode_table_hookup;
pub mod servo_mix;
pub mod srv_output_hookup;
pub mod srv_output_scheduler_hookup;
pub mod suppress_throttle_scheduler_hookup;
pub mod srv_pwm_publish_hookup;
pub mod stabilize_hookup;
pub mod target_altitude;
pub mod throttle_rules;
pub mod throttle_context_hookup;
pub mod yaw_throttle_glue_hookup;

/// The roll demand handed to the attitude controller, upstream's
/// `Plane::nav_roll_cd`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RollDemand {
    /// Demanded bank angle, centidegrees, after limiting.
    pub nav_roll_cd: i32,
}

impl RollDemand {
    /// Limit what the navigation controller asked for, upstream
    /// `Plane::calc_nav_roll`.
    ///
    /// `roll_limit_cd` is the vehicle's *current* limit, not the
    /// `ROLL_LIMIT_DEG` parameter: Plane reduces it during takeoff and
    /// landing, so reading the parameter would be wrong exactly when it
    /// matters.
    #[must_use]
    pub const fn from_navigation(commanded_roll_cd: i32, roll_limit_cd: i32) -> Self {
        let nav_roll_cd = constrain_int32(commanded_roll_cd, -roll_limit_cd, roll_limit_cd);
        Self { nav_roll_cd }
    }

    /// Shift the demand into the same half-turn as the measurement when flying
    /// inverted, upstream's prologue to `Plane::stabilize_roll`.
    ///
    /// Inverted, the demand sits near +-180 degrees, and so does the measured
    /// roll — but they can be on opposite sides of the wrap. Subtracting one
    /// from the other then gives an error near 360 degrees instead of near
    /// zero, and the PID would fight itself. Upstream's fix is to add half a
    /// turn to the demand and, if the measurement is negative, take a full
    /// turn back off, so both end up the same side of zero.
    pub const fn adjust_for_inverted(&mut self, roll_sensor_cd: i32) {
        self.nav_roll_cd += 18000;
        if roll_sensor_cd < 0 {
            self.nav_roll_cd -= 36000;
        }
    }

    /// The angle error the roll controller is given, upstream
    /// `nav_roll_cd - ahrs.roll_sensor`.
    #[must_use]
    pub const fn angle_error_cd(self, roll_sensor_cd: i32) -> i32 {
        self.nav_roll_cd - roll_sensor_cd
    }
}

/// The pitch demand handed to the attitude controller, upstream's
/// `Plane::nav_pitch_cd` and the `demanded_pitch` derived from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PitchDemand {
    /// TECS's demand after limiting, centidegrees. Upstream
    /// `Plane::nav_pitch_cd`.
    pub nav_pitch_cd: i32,
}

impl PitchDemand {
    /// Limit what TECS asked for, upstream `Plane::calc_nav_pitch`.
    ///
    /// The limits are asymmetric and both come from the vehicle rather than
    /// from parameters directly: `pitch_limit_min` is reduced during takeoff
    /// and landing, so reading `PTCH_LIM_MIN_DEG` would be wrong exactly when
    /// it matters.
    #[must_use]
    pub const fn from_tecs(
        commanded_pitch_cd: i32,
        pitch_limit_min_cd: i32,
        pitch_limit_max_cd: i32,
    ) -> Self {
        let nav_pitch_cd =
            constrain_int32(commanded_pitch_cd, pitch_limit_min_cd, pitch_limit_max_cd);
        Self { nav_pitch_cd }
    }

    /// Add the trim and the throttle feed-forward, upstream's
    /// `demanded_pitch` in `stabilize_pitch_get_pitch_out`.
    ///
    /// `pitch_trim_cd` is already `int32_t(PTCH_TRIM_DEG * 100)`; upstream
    /// truncates it before adding, so a trim of 0.7 degrees contributes 70
    /// centidegrees and a trim of 0.709 contributes 70 as well.
    ///
    /// The feed-forward exists because throttle changes pitch on most
    /// airframes: `KFF_THR2PTCH` lets the controller anticipate that instead
    /// of waiting for the error to appear.
    ///
    /// # Mixed arithmetic
    ///
    /// The two integer terms add as integers, the float term promotes the sum,
    /// and the result truncates back to `int32` on assignment. Doing it all in
    /// floating point, or all in integers, differs at the boundaries.
    #[must_use]
    pub fn demanded_pitch_cd(
        self,
        pitch_trim_cd: i32,
        throttle_scaled: f32,
        kff_throttle_to_pitch: f32,
    ) -> i32 {
        let integer_part = self.nav_pitch_cd + pitch_trim_cd;
        #[allow(
            clippy::cast_precision_loss,
            reason = "upstream promotes the integer sum to float to add the \
feed-forward; a pitch demand in centidegrees is far inside f32's exact range"
        )]
        let sum = integer_part as f32 + throttle_scaled * kff_throttle_to_pitch;
        #[allow(
            clippy::cast_possible_truncation,
            reason = "upstream truncates on assignment to int32_t"
        )]
        let out = sum as i32;
        out
    }

    /// The angle error the pitch controller is given, upstream
    /// `demanded_pitch - ahrs.pitch_sensor`.
    #[must_use]
    pub const fn angle_error_cd(demanded_pitch_cd: i32, pitch_sensor_cd: i32) -> i32 {
        demanded_pitch_cd - pitch_sensor_cd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pitch limits are asymmetric, unlike roll's.
    #[test]
    fn the_pitch_demand_is_limited_asymmetrically() {
        assert_eq!(PitchDemand::from_tecs(5000, -2000, 2500).nav_pitch_cd, 2500);
        assert_eq!(
            PitchDemand::from_tecs(-5000, -2000, 2500).nav_pitch_cd,
            -2000
        );
        assert_eq!(PitchDemand::from_tecs(1000, -2000, 2500).nav_pitch_cd, 1000);
    }

    /// The throttle feed-forward is what makes this more than an offset: at
    /// full throttle with the default gain the demand moves by a real amount.
    #[test]
    fn throttle_feeds_forward_into_the_pitch_demand() {
        let d = PitchDemand::from_tecs(1000, -2000, 2500);
        let none = d.demanded_pitch_cd(0, 0.0, 0.5);
        let full = d.demanded_pitch_cd(0, 100.0, 0.5);
        assert_eq!(none, 1000);
        assert_eq!(full, 1050);
    }

    /// Trim is truncated to whole centidegrees before it is added, so a
    /// fractional part of a centidegree is lost rather than rounded.
    #[test]
    fn trim_is_added_as_whole_centidegrees() {
        let d = PitchDemand::from_tecs(1000, -2000, 2500);
        assert_eq!(d.demanded_pitch_cd(70, 0.0, 0.0), 1070);
    }

    #[test]
    fn the_demand_is_limited_symmetrically() {
        assert_eq!(RollDemand::from_navigation(9000, 4500).nav_roll_cd, 4500);
        assert_eq!(RollDemand::from_navigation(-9000, 4500).nav_roll_cd, -4500);
        assert_eq!(RollDemand::from_navigation(1000, 4500).nav_roll_cd, 1000);
    }

    /// A zero limit pins the demand to zero rather than leaving it free, which
    /// is what upstream's constrain does and what a takeoff needs.
    #[test]
    fn a_zero_limit_pins_the_demand() {
        assert_eq!(RollDemand::from_navigation(3000, 0).nav_roll_cd, 0);
    }

    /// Inverted, the error must come out small. Demanding 170 degrees while
    /// measuring -175 is a five degree error the wrong way round, not 345.
    #[test]
    fn inverted_flight_keeps_the_error_small() {
        let mut d = RollDemand::from_navigation(-1000, 18000);
        d.adjust_for_inverted(-17500);
        let err = d.angle_error_cd(-17500);
        assert!(
            err.abs() < 4000,
            "expected a small error across the wrap, got {err}"
        );
    }
}
