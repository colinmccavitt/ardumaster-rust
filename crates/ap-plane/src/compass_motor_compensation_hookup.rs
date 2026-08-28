//! Compass motor compensation stub, upstream `COMPASS_MOT` / `COMPASS_MOTCT`.
//!
//! Current-based hard-iron: latch battery current (or throttle) so the next
//! SITL publish applies `mag += COMPASS_MOT * thr_or_curr` when MOTCT is
//! current or throttle.

use ap_compass::motor_comp::{motor_comp_enabled, motor_offset};
use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Per-tick inputs for motor compensation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassMotorCompensationInputs {
    /// Battery current in amps, or throttle `0..1` when MOTCT is throttle.
    pub thr_or_curr: f32,
}

/// Result of latching motor-compensation current this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassMotorCompensationOutput {
    /// `COMPASS_MOT * thr_or_curr` on the primary instance.
    pub motor_offset: Vector3f,
    /// True when `COMPASS_MOTCT` is throttle or current.
    pub enabled: bool,
}

/// Latch throttle/current used by `correct_field` motor compensation.
#[must_use]
pub fn compass_motor_compensation_tick(
    hookup: &mut SitlCompassHookup,
    inp: CompassMotorCompensationInputs,
) -> CompassMotorCompensationOutput {
    hookup.set_thr_or_curr(inp.thr_or_curr);
    let params = hookup.compass_params();
    let inst = if params.primary == 0 {
        params.compass1
    } else {
        params.compass2
    };
    CompassMotorCompensationOutput {
        motor_offset: motor_offset(
            inst.motor_compensation,
            params.motor_comp_type,
            inp.thr_or_curr,
        ),
        enabled: motor_comp_enabled(params.motor_comp_type),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_compass::motor_comp::COMPASS_MOT_COMP_CURRENT;
    use ap_compass::params::CompassParams;
    use ap_math::vector3::Vector3f;
    use crate::sitl_compass_hookup::SitlCompassHookup;

    #[test]
    fn disabled_reports_zero_offset() {
        let mut hookup = SitlCompassHookup::default();
        let out = compass_motor_compensation_tick(
            &mut hookup,
            CompassMotorCompensationInputs { thr_or_curr: 10.0 },
        );
        assert!(!out.enabled);
        assert_eq!(out.motor_offset, Vector3f::zero());
    }

    #[test]
    fn current_mode_reports_mot_times_amps() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.motor_comp_type = COMPASS_MOT_COMP_CURRENT;
        params.compass1.motor_compensation = Vector3f::new(0.01, -0.02, 0.0);
        hookup.apply_compass_params(params);
        let out = compass_motor_compensation_tick(
            &mut hookup,
            CompassMotorCompensationInputs { thr_or_curr: 10.0 },
        );
        assert!(out.enabled);
        assert!((out.motor_offset.x - 0.1).abs() < 1e-6);
        assert!((out.motor_offset.y + 0.2).abs() < 1e-6);
    }
}
