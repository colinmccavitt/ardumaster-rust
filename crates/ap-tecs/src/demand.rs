//! Speed demand and energy rate limits, ported from
//! `AP_TECS::_update_speed_demand` and `AP_TECS::_update_STE_rate_lim`.
//!
//! Turns the raw airspeed demand into a rate-limited one the energy controller
//! can track, and computes the total-energy-rate bounds several other stages
//! clamp against.
//!
//! # The 50% / 90% split is margin allocation, not relative speed
//!
//! Acceleration is limited to **50%** of the energy-gain rate; deceleration to
//! **90%** of the dissipation rates. Upstream's comment gives the reason: the
//! remaining margin is reserved for the total energy controller. Gaining speed
//! competes with climbing for the same surplus energy, so the speed demand
//! claims only half of it; dissipation is less contended and may claim more.
//!
//! It does **not** follow that the demand sheds speed faster than it gains it.
//! The percentages apply to different energy rates over different denominators:
//!
//! ```text
//! vel_rate_max        = 0.5 * STEdot_max     / TAS_state
//! vel_rate_neg_max    = 0.9 * STEdot_neg_max / TASmax
//! vel_rate_neg_cruise = 0.9 * STEdot_min     / TAScruise
//! ```
//!
//! With the default parameters at 15 m/s these come out at +1.63 and -1.63 —
//! near enough equal. Which side dominates depends on the parameter set and the
//! operating point. An earlier draft of this comment claimed otherwise and the
//! test written from it failed.
//!
//! The deceleration limit is interpolated between its value at maximum speed
//! and at cruise, because the energy available to shed differs across the
//! envelope. Note the interpolation input range runs downward (`TASmax` to
//! `TAScruise`), which `linear_interpolate` supports by swapping both pairs.

use ap_math::scalar::{constrain_value, linear_interpolate};

use crate::params::TecsParams;
use crate::speed::GRAVITY_MSS;

/// Total-energy-rate bounds, upstream `_STEdot_max`/`_min`/`_neg_max`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SteRateLimits {
    /// Maximum total energy rate, upstream `_STEdot_max`.
    pub ste_dot_max: f32,
    /// Minimum total energy rate at idle, upstream `_STEdot_min`.
    pub ste_dot_min: f32,
    /// Most negative total energy rate, upstream `_STEdot_neg_max`.
    pub ste_dot_neg_max: f32,
}

impl SteRateLimits {
    /// One `_update_STE_rate_lim` step.
    ///
    /// `climb_rate_limit` comes from the height demand stage, which is why this
    /// must run after it: the limits track the adaptive climb scaler rather
    /// than the raw parameter.
    pub fn update(params: &TecsParams, climb_rate_limit: f32) -> Self {
        Self {
            ste_dot_max: climb_rate_limit * GRAVITY_MSS,
            ste_dot_min: -params.min_sink_rate * GRAVITY_MSS,
            ste_dot_neg_max: -params.max_sink_rate * GRAVITY_MSS,
        }
    }
}

/// Everything the speed demand stage reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct SpeedDemandInputs {
    /// Raw true airspeed demand, upstream `_TAS_dem`.
    pub tas_dem: f32,
    /// Current true airspeed estimate, upstream `_TAS_state`.
    pub tas_state: f32,
    /// Minimum true airspeed, upstream `_TASmin`.
    pub tas_min: f32,
    /// Maximum true airspeed, upstream `_TASmax`.
    pub tas_max: f32,
    /// Cruise airspeed converted to true, upstream `aparm.airspeed_cruise * EAS2TAS`.
    pub tas_cruise: f32,
    /// How far into the sink limit the height demand sits, upstream `_sink_fraction`.
    pub sink_fraction: f32,
    /// Whether a bad descent is latched.
    pub bad_descent: bool,
    /// Whether underspeed is latched.
    pub underspeed: bool,
    /// Whether the `DESCENT_SPEEDUP` option is set.
    pub descent_speedup: bool,
}

/// Rate-limited speed demand state.
#[derive(Debug, Clone, Copy, Default)]
pub struct SpeedDemand {
    /// Rate-limited true airspeed demand, upstream `_TAS_dem_adj`.
    pub tas_dem_adj: f32,
    /// Airspeed rate demand, upstream `_TAS_rate_dem`.
    pub tas_rate_dem: f32,
    /// Low-passed airspeed rate demand, upstream `_TAS_rate_dem_lpf`.
    pub tas_rate_dem_lpf: f32,
}

impl SpeedDemand {
    /// A demand at rest.
    pub fn new() -> Self {
        Self::default()
    }

    /// One `_update_speed_demand` step.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        inp: &SpeedDemandInputs,
        limits: &SteRateLimits,
        time_constant: f32,
        dt: f32,
        reset: bool,
    ) {
        let mut tas_dem = inp.tas_dem;

        if inp.descent_speedup {
            // let the demand run to maximum when descending at the sink limit
            tas_dem += (inp.tas_max - tas_dem) * inp.sink_fraction;
        }

        // Underspeed or a bad descent both drop the demand to minimum: this
        // minimises descent rate after an engine failure, allows maximum climb,
        // and stops a full-power descent into terrain chasing an unachievable
        // speed.
        if inp.bad_descent || inp.underspeed {
            tas_dem = inp.tas_min;
        }

        tas_dem = constrain_value(tas_dem, inp.tas_min, inp.tas_max);

        // Asymmetric by design: 50% of the energy rate on gain, 90% on
        // dissipation, leaving margin for the total energy controller.
        let vel_rate_max = 0.5 * limits.ste_dot_max / inp.tas_state;
        let vel_rate_neg_max = 0.9 * limits.ste_dot_neg_max / inp.tas_max;
        let vel_rate_neg_cruise = 0.9 * limits.ste_dot_min / inp.tas_cruise;
        // interpolate the deceleration limit across the envelope; note the
        // input range runs downward, which linear_interpolate supports
        let vel_rate_min = linear_interpolate(
            vel_rate_neg_max,
            vel_rate_neg_cruise,
            inp.tas_state,
            inp.tas_max,
            inp.tas_cruise,
        );

        let previous = self.tas_dem_adj;
        if (tas_dem - previous) > (vel_rate_max * dt) {
            self.tas_dem_adj = previous + vel_rate_max * dt;
            self.tas_rate_dem = vel_rate_max;
        } else if (tas_dem - previous) < (vel_rate_min * dt) {
            self.tas_dem_adj = previous + vel_rate_min * dt;
            self.tas_rate_dem = vel_rate_min;
        } else {
            self.tas_rate_dem = (tas_dem - previous) / dt;
            self.tas_dem_adj = tas_dem;
        }

        if reset {
            // re-seed from the measurement rather than filtering across a gap
            self.tas_dem_adj = inp.tas_state;
            self.tas_rate_dem_lpf = self.tas_rate_dem;
        } else {
            let alpha = dt / (dt + time_constant);
            self.tas_rate_dem_lpf =
                self.tas_rate_dem_lpf * (1.0 - alpha) + self.tas_rate_dem * alpha;
        }

        // constrain again, guarding against bad values on initialisation
        self.tas_dem_adj = constrain_value(self.tas_dem_adj, inp.tas_min, inp.tas_max);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:477-534 and 1245-1253.

    fn limits() -> SteRateLimits {
        SteRateLimits {
            ste_dot_max: 5.0 * GRAVITY_MSS,
            ste_dot_min: -2.0 * GRAVITY_MSS,
            ste_dot_neg_max: -5.0 * GRAVITY_MSS,
        }
    }

    fn inputs() -> SpeedDemandInputs {
        SpeedDemandInputs {
            tas_dem: 20.0,
            tas_state: 20.0,
            tas_min: 9.0,
            tas_max: 22.0,
            tas_cruise: 12.0,
            sink_fraction: 0.0,
            bad_descent: false,
            underspeed: false,
            descent_speedup: false,
        }
    }

    /// The energy rate limits track the ADAPTIVE climb limit, not the raw
    /// parameter, which is why this stage must run after the height demand.
    #[test]
    fn ste_limits_track_the_adaptive_climb_limit() {
        let p = TecsParams::default(); // max_climb 5, min_sink 2, max_sink 5
        let full = SteRateLimits::update(&p, 5.0);
        assert_eq!(full.ste_dot_max, 5.0 * GRAVITY_MSS);
        assert_eq!(full.ste_dot_min, -2.0 * GRAVITY_MSS);
        assert_eq!(full.ste_dot_neg_max, -5.0 * GRAVITY_MSS);

        // a scaled-down climb limit reduces only the positive bound
        let reduced = SteRateLimits::update(&p, 2.5);
        assert_eq!(reduced.ste_dot_max, 2.5 * GRAVITY_MSS);
        assert_eq!(reduced.ste_dot_min, full.ste_dot_min, "sink is unaffected");
    }

    /// Underspeed drops the demand to minimum, which is what stops a full-power
    /// descent chasing an unachievable speed.
    #[test]
    fn underspeed_drops_demand_to_minimum() {
        let l = limits();
        let mut i = inputs();
        i.underspeed = true;

        let mut d = SpeedDemand::new();
        d.tas_dem_adj = 20.0;
        // large dt so the rate limiter does not mask the change
        d.update(&i, &l, 5.0, 1.0, false);
        assert!(
            d.tas_dem_adj < 20.0,
            "demand should fall toward minimum, got {}",
            d.tas_dem_adj
        );
    }

    /// A bad descent does the same thing, for the same reason.
    #[test]
    fn bad_descent_drops_demand_to_minimum() {
        let l = limits();
        let mut i = inputs();
        i.bad_descent = true;

        let mut d = SpeedDemand::new();
        d.tas_dem_adj = 20.0;
        d.update(&i, &l, 5.0, 1.0, false);
        assert!(d.tas_dem_adj < 20.0);
    }

    /// Both directions are rate limited, and the rates match the documented
    /// formulas.
    ///
    /// Deliberately does NOT assert that one direction is faster than the
    /// other: with default parameters at 15 m/s they are near enough equal, and
    /// which dominates depends on the parameter set. An earlier version of this
    /// test asserted deceleration was faster and failed - see the module docs.
    #[test]
    fn both_directions_are_rate_limited() {
        let l = limits();

        let mut d_up = SpeedDemand::new();
        d_up.tas_dem_adj = 15.0;
        let mut i_up = inputs();
        i_up.tas_dem = 22.0; // demand a big speed-up
        i_up.tas_state = 15.0;
        d_up.update(&i_up, &l, 5.0, 1.0, false);
        let gained = d_up.tas_dem_adj - 15.0;

        let mut d_dn = SpeedDemand::new();
        d_dn.tas_dem_adj = 15.0;
        let mut i_dn = inputs();
        i_dn.tas_dem = 9.0; // demand a big slow-down
        i_dn.tas_state = 15.0;
        d_dn.update(&i_dn, &l, 5.0, 1.0, false);
        let shed = 15.0 - d_dn.tas_dem_adj;

        // both moved, and neither jumped straight to the demand
        assert!(gained > 0.0 && gained < 7.0, "gained {gained}");
        assert!(shed > 0.0 && shed < 6.0, "shed {shed}");

        // acceleration matches 0.5 * STEdot_max / TAS_state, times dt
        let expected_gain = 0.5 * l.ste_dot_max / 15.0;
        assert!(
            (gained - expected_gain).abs() < 1e-3,
            "gain {gained} should match {expected_gain}"
        );
    }

    /// The deceleration limit is interpolated across the envelope, so it
    /// differs at maximum speed and at cruise.
    #[test]
    fn deceleration_limit_varies_across_the_envelope() {
        let l = limits();

        let shed_at = |tas_state: f32| {
            let mut d = SpeedDemand::new();
            d.tas_dem_adj = tas_state;
            let mut i = inputs();
            i.tas_dem = 0.0; // demand minimum
            i.tas_state = tas_state;
            d.update(&i, &l, 5.0, 1.0, false);
            tas_state - d.tas_dem_adj
        };

        let at_max = shed_at(22.0);
        let at_cruise = shed_at(12.0);
        assert!(at_max > 0.0 && at_cruise > 0.0);
        assert!(
            (at_max - at_cruise).abs() > 1e-3,
            "limit should differ across the envelope: {at_max} vs {at_cruise}"
        );
    }

    /// Reset re-seeds from the measurement rather than filtering across a gap.
    #[test]
    fn reset_seeds_from_measurement() {
        let l = limits();
        let i = inputs();
        let mut d = SpeedDemand::new();
        d.tas_dem_adj = 5.0;
        d.tas_rate_dem_lpf = 99.0;

        d.update(&i, &l, 5.0, 0.02, true);
        assert_eq!(d.tas_dem_adj, i.tas_state, "seeded from measurement");
        assert_eq!(d.tas_rate_dem_lpf, d.tas_rate_dem, "lpf seeded to current");
    }

    /// The demand stays inside the airspeed band, even after rate limiting.
    #[test]
    fn demand_stays_within_the_band() {
        let l = limits();
        let mut i = inputs();
        i.tas_dem = 1000.0;

        let mut d = SpeedDemand::new();
        d.tas_dem_adj = 20.0;
        for _ in 0..200 {
            d.update(&i, &l, 5.0, 0.02, false);
        }
        assert!(d.tas_dem_adj <= i.tas_max + 1e-4, "got {}", d.tas_dem_adj);
        assert!(d.tas_dem_adj >= i.tas_min - 1e-4);
    }

    /// DESCENT_SPEEDUP lets the demand run toward maximum in proportion to how
    /// far into the sink limit the height demand sits.
    #[test]
    fn descent_speedup_raises_demand_with_sink_fraction() {
        let l = limits();

        let run = |speedup: bool, frac: f32| {
            let mut i = inputs();
            i.descent_speedup = speedup;
            i.sink_fraction = frac;
            let mut d = SpeedDemand::new();
            d.tas_dem_adj = 20.0;
            d.update(&i, &l, 5.0, 1.0, false);
            d.tas_dem_adj
        };

        let off = run(false, 1.0);
        let on = run(true, 1.0);
        assert!(on > off, "speedup {on} should exceed baseline {off}");
        // and with no sink, the option changes nothing
        assert_eq!(run(true, 0.0), run(false, 0.0));
    }
}
