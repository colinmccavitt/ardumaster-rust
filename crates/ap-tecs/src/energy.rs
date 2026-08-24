//! Specific energy calculations, ported from `AP_TECS::_update_energies`.
//!
//! TECS works in specific energy — energy per unit mass — so potential and
//! kinetic terms are directly comparable and can be traded against each other.
//! Potential specific energy is `height * g`; kinetic is `½ V²`.
//!
//! # The high-pass on the rate terms is not incidental
//!
//! Both rate terms subtract a low-pass-filtered version of themselves:
//!
//! ```text
//! SKEdot     = TAS_state * (vel_dot     - vel_dot_lpf)
//! SKEdot_dem = TAS_state * (TAS_rate_dem - TAS_rate_dem_lpf)
//! ```
//!
//! That difference is a high-pass filter. Upstream's comments give the reason:
//! on the measurement side it removes bias introduced by the complementary
//! filter, and on the demand side it applies *matching* filtering so demand and
//! measurement are comparable. Dropping either subtraction would leave a
//! filter-induced bias in the energy balance, so both are reproduced exactly.

use crate::speed::GRAVITY_MSS;

/// The specific-energy terms produced each update.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Energies {
    /// Demanded specific potential energy, upstream `_SPE_dem`.
    pub spe_dem: f32,
    /// Demanded specific kinetic energy, upstream `_SKE_dem`.
    pub ske_dem: f32,
    /// Demanded specific kinetic energy rate, upstream `_SKEdot_dem`.
    pub skedot_dem: f32,
    /// Estimated specific potential energy, upstream `_SPE_est`.
    pub spe_est: f32,
    /// Estimated specific kinetic energy, upstream `_SKE_est`.
    pub ske_est: f32,
    /// Specific potential energy rate, upstream `_SPEdot`.
    pub spedot: f32,
    /// Specific kinetic energy rate, upstream `_SKEdot`.
    pub skedot: f32,
}

/// Inputs to one energy update.
#[derive(Debug, Clone, Copy)]
pub struct EnergyInputs {
    /// Demanded height, m. Upstream `_hgt_dem`.
    pub hgt_dem: f32,
    /// Adjusted true airspeed demand, m/s. Upstream `_TAS_dem_adj`.
    pub tas_dem_adj: f32,
    /// Current true airspeed estimate, m/s. Upstream `_TAS_state`.
    pub tas_state: f32,
    /// Demanded airspeed rate of change. Upstream `_TAS_rate_dem`.
    pub tas_rate_dem: f32,
    /// Low-pass filtered demanded airspeed rate. Upstream `_TAS_rate_dem_lpf`.
    pub tas_rate_dem_lpf: f32,
    /// Current height, m. Upstream `_height`.
    pub height: f32,
    /// Current climb rate, m/s. Upstream `_climb_rate`.
    pub climb_rate: f32,
    /// Speed rate of change. Upstream `_vel_dot`.
    pub vel_dot: f32,
    /// Low-pass filtered speed rate of change. Upstream `_vel_dot_lpf`.
    pub vel_dot_lpf: f32,
}

impl Energies {
    /// One `_update_energies` step.
    pub fn update(inp: &EnergyInputs) -> Self {
        Self {
            // demands
            spe_dem: inp.hgt_dem * GRAVITY_MSS,
            ske_dem: 0.5 * inp.tas_dem_adj * inp.tas_dem_adj,
            // high-passed so the demand is filtered to match the measurement
            skedot_dem: inp.tas_state * (inp.tas_rate_dem - inp.tas_rate_dem_lpf),
            // estimates
            spe_est: inp.height * GRAVITY_MSS,
            ske_est: 0.5 * inp.tas_state * inp.tas_state,
            spedot: inp.climb_rate * GRAVITY_MSS,
            // high-passed to remove complementary-filter bias
            skedot: inp.tas_state * (inp.vel_dot - inp.vel_dot_lpf),
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:678-700.

    fn inputs() -> EnergyInputs {
        EnergyInputs {
            hgt_dem: 100.0,
            tas_dem_adj: 20.0,
            tas_state: 18.0,
            tas_rate_dem: 0.0,
            tas_rate_dem_lpf: 0.0,
            height: 90.0,
            climb_rate: 2.0,
            vel_dot: 0.0,
            vel_dot_lpf: 0.0,
        }
    }

    #[test]
    fn potential_energy_is_height_times_gravity() {
        let e = Energies::update(&inputs());
        assert_eq!(e.spe_dem, 100.0 * GRAVITY_MSS);
        assert_eq!(e.spe_est, 90.0 * GRAVITY_MSS);
        assert_eq!(e.spedot, 2.0 * GRAVITY_MSS);
    }

    #[test]
    fn kinetic_energy_is_half_v_squared() {
        let e = Energies::update(&inputs());
        assert_eq!(e.ske_dem, 0.5 * 20.0 * 20.0);
        assert_eq!(e.ske_est, 0.5 * 18.0 * 18.0);
    }

    /// The rate terms are HIGH-PASSED: each subtracts its own low-passed
    /// version. With input equal to its filtered value the result is zero, not
    /// the raw rate. Dropping the subtraction would leave a filter-induced bias
    /// in the energy balance.
    #[test]
    fn rate_terms_are_high_passed_not_raw() {
        let mut i = inputs();
        i.vel_dot = 3.0;
        i.vel_dot_lpf = 3.0;
        i.tas_rate_dem = 2.0;
        i.tas_rate_dem_lpf = 2.0;

        let e = Energies::update(&i);
        assert_eq!(e.skedot, 0.0, "steady rate must high-pass to zero");
        assert_eq!(e.skedot_dem, 0.0, "steady demand rate likewise");

        // only the difference survives, scaled by airspeed
        i.vel_dot = 4.0;
        i.vel_dot_lpf = 3.0;
        let e = Energies::update(&i);
        assert_eq!(e.skedot, 18.0 * 1.0, "TAS_state * (vel_dot - vel_dot_lpf)");
    }

    /// Energy is specific (per unit mass), so both terms share units and can be
    /// traded directly - which is the whole premise of TECS.
    #[test]
    fn potential_and_kinetic_are_comparable_quantities() {
        let mut i = inputs();
        // 10 m of height against the speed that carries the same energy:
        // g*h = 0.5*v^2  =>  v = sqrt(2*g*h)
        i.height = 10.0;
        i.tas_state = libm::sqrtf(2.0 * GRAVITY_MSS * 10.0);
        let e = Energies::update(&i);
        assert!(
            (e.spe_est - e.ske_est).abs() < 1e-3,
            "should balance: spe {} vs ske {}",
            e.spe_est,
            e.ske_est
        );
    }

    #[test]
    fn zero_state_gives_zero_energy() {
        let i = EnergyInputs {
            hgt_dem: 0.0,
            tas_dem_adj: 0.0,
            tas_state: 0.0,
            tas_rate_dem: 0.0,
            tas_rate_dem_lpf: 0.0,
            height: 0.0,
            climb_rate: 0.0,
            vel_dot: 0.0,
            vel_dot_lpf: 0.0,
        };
        assert_eq!(Energies::update(&i), Energies::default());
    }
}
