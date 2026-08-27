//! Navigation scheduler tick glue for `update_control_mode`.
//!
//! Upstream `Plane::calc_nav_roll` and `Plane::calc_nav_pitch` limit L1/TECS
//! demands before stabilize reads them via [`NavTecsPublish`].

use ap_math::scalar::cd_to_rad;

use crate::{PitchDemand, RollDemand};

/// HAL inputs for one navigation scheduler tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NavigationSchedulerInputs {
    /// Raw L1 bank demand, upstream `nav_controller->nav_roll_cd()`.
    pub commanded_roll_cd: i32,
    /// Raw TECS pitch demand, centidegrees.
    pub commanded_pitch_cd: i32,
    /// Current roll limit, upstream `roll_limit_cd`.
    pub roll_limit_cd: i32,
    /// Current pitch floor, upstream `pitch_limit_min`.
    pub pitch_limit_min_cd: i32,
    /// Current pitch ceiling, upstream `pitch_limit_max`.
    pub pitch_limit_max_cd: i32,
}

/// Limited navigation outputs for one scheduler tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NavigationSchedulerOutput {
    pub nav_roll_cd: i32,
    pub tecs_pitch_demand_rad: f32,
}

/// Limit L1/TECS demands, upstream calc_nav_roll / calc_nav_pitch.
#[must_use]
pub fn navigation_scheduler_tick(inp: &NavigationSchedulerInputs) -> NavigationSchedulerOutput {
    let nav_roll_cd = RollDemand::from_navigation(inp.commanded_roll_cd, inp.roll_limit_cd).nav_roll_cd;
    let nav_pitch_cd = PitchDemand::from_tecs(
        inp.commanded_pitch_cd,
        inp.pitch_limit_min_cd,
        inp.pitch_limit_max_cd,
    )
    .nav_pitch_cd;
    NavigationSchedulerOutput {
        nav_roll_cd,
        tecs_pitch_demand_rad: cd_to_rad(nav_pitch_cd as f32),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limits_roll_and_pitch() {
        let out = navigation_scheduler_tick(&NavigationSchedulerInputs {
            commanded_roll_cd: 9000,
            commanded_pitch_cd: 5000,
            roll_limit_cd: 4500,
            pitch_limit_min_cd: -2000,
            pitch_limit_max_cd: 2500,
        });
        assert_eq!(out.nav_roll_cd, 4500);
        assert!((out.tecs_pitch_demand_rad - cd_to_rad(2500.0)).abs() < 1e-6);
    }
}
