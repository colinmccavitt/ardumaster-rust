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
        let limit = roll_limit_cd;
        let nav_roll_cd = if commanded_roll_cd < -limit {
            -limit
        } else if commanded_roll_cd > limit {
            limit
        } else {
            commanded_roll_cd
        };
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

#[cfg(test)]
mod tests {
    use super::*;

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
