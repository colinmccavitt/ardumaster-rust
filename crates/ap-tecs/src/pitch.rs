//! Pitch demand and bad-descent detection, ported from `AP_TECS::_update_pitch`
//! and `AP_TECS::_detect_bad_descent`.
//!
//! Pitch is driven by the specific energy **balance** — potential minus kinetic,
//! weighted — rather than total energy, which is what throttle uses. That split
//! is the core idea of TECS: throttle adds or removes total energy, pitch
//! exchanges one form for the other.
//!
//! # The weighting chain decides what pitch is for
//!
//! `SKE_weighting` ranges 0..2 and sets the priority:
//! - **0** — pitch controls height only. Used with no airspeed sensor, and in
//!   VTOL where the usual speed/height relationship is broken by lift motors.
//! - **1** — equal priority, the normal case.
//! - **2** — pitch controls speed only. Used when underspeed, taking off,
//!   aborting a landing, or gliding — all cases where speed must not be traded
//!   away.
//!
//! The selection is an ordered `if/else` chain and the order is load-bearing:
//! no-airspeed wins over VTOL, which wins over underspeed/takeoff, which wins
//! over landing. Reordering would silently change which mode applies when two
//! conditions hold at once.
//!
//! # A cross-module invariant
//!
//! Two divisions here use `TAS_state`: the `gainInv` conversion from energy rate
//! to pitch angle, and the pitch rate limit. Neither is guarded locally. What
//! makes them safe is the 3 m/s floor applied in
//! [`crate::speed::SpeedState::update`] — see [`crate::speed::MIN_AIRSPEED`].
//! Porting this module without that floor would reintroduce a division by zero.

use ap_math::scalar::{constrain_value, is_zero, radians};

use crate::params::{FlightStage, TecsParams};
use crate::speed::{ClipStatus, GRAVITY_MSS};
use crate::util::min_f32;

/// Everything the pitch path reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct PitchInputs {
    /// Demanded specific potential energy, upstream `_SPE_dem`.
    pub spe_dem: f32,
    /// Demanded specific kinetic energy, upstream `_SKE_dem`.
    pub ske_dem: f32,
    /// Estimated specific potential energy, upstream `_SPE_est`.
    pub spe_est: f32,
    /// Estimated specific kinetic energy, upstream `_SKE_est`.
    pub ske_est: f32,
    /// Potential energy rate, upstream `_SPEdot`.
    pub spedot: f32,
    /// Kinetic energy rate, upstream `_SKEdot`.
    pub skedot: f32,
    /// Height rate demand, upstream `_hgt_rate_dem`.
    pub hgt_rate_dem: f32,
    /// True airspeed estimate, upstream `_TAS_state`. Guaranteed at least
    /// [`crate::speed::MIN_AIRSPEED`], which is what keeps the divisions safe.
    pub tas_state: f32,
    /// Adjusted true airspeed demand, upstream `_TAS_dem_adj`.
    pub tas_dem_adj: f32,
    /// Minimum pitch, radians. Upstream `_PITCHminf`.
    pub pitch_min: f32,
    /// Maximum pitch, radians. Upstream `_PITCHmaxf`.
    pub pitch_max: f32,
    /// Progress along the landing path, 0..1. Upstream `_path_proportion`.
    pub path_proportion: f32,
    /// Whether airspeed is in use, upstream `use_airspeed()`.
    pub use_airspeed: bool,
    /// Whether underspeed is latched.
    pub underspeed: bool,
    /// Whether gliding.
    pub is_gliding: bool,
    /// Whether an automatic landing is in progress.
    pub is_doing_auto_land: bool,
    /// Whether flaring.
    pub is_flaring: bool,
    /// Current flight stage.
    pub flight_stage: FlightStage,
}

/// Pitch demand state.
#[derive(Debug, Clone, Copy, Default)]
pub struct PitchDemand {
    /// Constrained pitch demand, radians. Upstream `_pitch_dem`.
    pub pitch_dem: f32,
    /// Unconstrained pitch demand, upstream `_pitch_dem_unc`.
    pub pitch_dem_unc: f32,
    /// Kinetic energy weighting actually applied, upstream `_SKE_weighting`.
    pub ske_weighting: f32,
    /// Energy-balance-rate clip state, upstream `_SEBdot_dem_clip`.
    pub sebdot_dem_clip: ClipStatus,
    /// Energy balance rate integrator, upstream `_integSEBdot`.
    integ_sebdot: f32,
    /// Kinetic energy trim integrator, upstream `_integKE`.
    integ_ke: f32,
    /// Previous demand, for rate limiting. Upstream `_last_pitch_dem`.
    last_pitch_dem: f32,
    /// Whether a bad descent is latched, upstream `_flags.badDescent`.
    bad_descent: bool,
}

impl PitchDemand {
    /// A demand at rest.
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed the rate-limit history, upstream `_last_pitch_dem`.
    ///
    /// Called on reset so the first demand is rate limited from the aircraft's
    /// actual pitch rather than from zero.
    pub fn seed_last_demand(&mut self, value: f32) {
        self.last_pitch_dem = value;
        self.pitch_dem = value;
    }

    /// Whether a bad descent is latched.
    pub fn bad_descent(&self) -> bool {
        self.bad_descent
    }

    /// Overwrite both integrators, for log replay seeding.
    #[cfg(feature = "replay")]
    pub fn seed_integrators(&mut self, integ_sebdot: f32, integ_ke: f32) {
        self.integ_sebdot = integ_sebdot;
        self.integ_ke = integ_ke;
    }

    /// The energy balance rate integrator, upstream `_integSEBdot`.
    pub fn integ_sebdot(&self) -> f32 {
        self.integ_sebdot
    }

    /// The kinetic energy trim integrator, upstream `_integKE`.
    pub fn integ_ke(&self) -> f32 {
        self.integ_ke
    }

    /// One `_detect_bad_descent` step.
    ///
    /// Detects an airspeed demand the aircraft cannot achieve, which would
    /// otherwise fly it into the ground trading height for unreachable speed.
    /// Latching: once set it stays set until the total energy error returns to
    /// zero, which upstream notes produces an undulating response as it cuts in
    /// and out — accepted, because the alternative is a descent into terrain.
    pub fn detect_bad_descent(
        &mut self,
        inp: &PitchInputs,
        ste_error: f32,
        throttle_dem: f32,
        thr_max: f32,
    ) -> bool {
        // gliding, VTOL and underspeed all legitimately lose energy
        if inp.is_gliding || inp.flight_stage == FlightStage::Vtol || inp.underspeed {
            self.bad_descent = false;
            return false;
        }

        let stedot = inp.spedot + inp.skedot;
        // large energy deficit, energy still falling, and throttle already at
        // 90% or more: the demand cannot be met
        self.bad_descent = (ste_error > 200.0 && stedot < 0.0 && throttle_dem >= thr_max * 0.9)
            || (self.bad_descent && ste_error > 0.0);
        self.bad_descent
    }

    /// Speed/height priority weighting, upstream's opening block of
    /// `_update_pitch`.
    ///
    /// The order of these branches is load-bearing; see the module docs.
    fn ske_weighting(&self, params: &TecsParams, inp: &PitchInputs) -> f32 {
        if !inp.use_airspeed {
            // nothing to control speed with: height only
            0.0
        } else if inp.flight_stage == FlightStage::Vtol {
            // lift motors break the speed/height relationship
            0.0
        } else if inp.underspeed
            || inp.flight_stage == FlightStage::Takeoff
            || inp.flight_stage == FlightStage::AbortLanding
            || inp.is_gliding
        {
            // speed must not be traded away
            2.0
        } else if inp.is_doing_auto_land {
            if params.spd_weight_land < 0.0 {
                // negative is the automatic sentinel: slide from the normal
                // weight down to zero as the landing progresses
                let scaled =
                    params.spd_weight * (1.0 - constrain_value(inp.path_proportion, 0.0, 1.0));
                constrain_value(scaled, 0.0, 2.0)
            } else {
                constrain_value(params.spd_weight_land, 0.0, 2.0)
            }
        } else {
            constrain_value(params.spd_weight, 0.0, 2.0)
        }
    }

    /// One `_update_pitch` step.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        params: &TecsParams,
        inp: &PitchInputs,
        time_constant: f32,
        i_gain: f32,
        dt: f32,
    ) {
        self.ske_weighting = self.ske_weighting(params, inp);
        let mut spe_weighting = 2.0 - self.ske_weighting;

        // Either weight may fade to zero, but neither may exceed 1: going above
        // would destabilise a controller tuned at a weight of 1.
        spe_weighting = min_f32(spe_weighting, 1.0);
        self.ske_weighting = min_f32(self.ske_weighting, 1.0);

        let seb_dem = inp.spe_dem * spe_weighting - inp.ske_dem * self.ske_weighting;
        let seb_est = inp.spe_est * spe_weighting - inp.ske_est * self.ske_weighting;
        let seb_error = seb_dem - seb_est;

        // track the demanded height on the controller time constant
        let mut sebdot_dem =
            inp.hgt_rate_dem * GRAVITY_MSS * spe_weighting + seb_error / time_constant;
        let sebdot_dem_min = -params.max_sink_rate * GRAVITY_MSS;
        let sebdot_dem_max = params.max_climb_rate * GRAVITY_MSS;
        if sebdot_dem < sebdot_dem_min {
            sebdot_dem = sebdot_dem_min;
            self.sebdot_dem_clip = ClipStatus::Min;
        } else if sebdot_dem > sebdot_dem_max {
            sebdot_dem = sebdot_dem_max;
            self.sebdot_dem_clip = ClipStatus::Max;
        } else {
            self.sebdot_dem_clip = ClipStatus::None;
        }

        let sebdot_est = inp.spedot * spe_weighting - inp.skedot * self.ske_weighting;
        let sebdot_error = sebdot_dem - sebdot_est;

        // flare damping wins outright; landing damping only when set
        let pitch_damp = if inp.is_flaring {
            params.land_damp
        } else if !is_zero(params.land_pitch_damp) && inp.is_doing_auto_land {
            params.land_pitch_damp
        } else {
            params.ptch_damp
        };
        let mut sebdot_dem_total = sebdot_dem + sebdot_error * pitch_damp;

        // Inverse gain from energy balance rate to pitch angle. Safe because
        // tas_state carries the MIN_AIRSPEED floor from the speed filter.
        let gain_inv = inp.tas_state * GRAVITY_MSS;

        if matches!(
            inp.flight_stage,
            FlightStage::Takeoff | FlightStage::AbortLanding
        ) {
            // bias so zero speed error demands the minimum pitch, sparing the
            // integrator from having to catch up before the nose can come up
            sebdot_dem_total += inp.pitch_min * gain_inv;
        }

        // Integrator limits allowing 5 degrees of saturation, so gusts do not
        // immediately clip the integrator and blunt its authority.
        let integ_sebdot_min = (gain_inv * (inp.pitch_min - radians(5.0))) - sebdot_dem_total;
        let integ_sebdot_max = (gain_inv * (inp.pitch_max + radians(5.0))) - sebdot_dem_total;

        // Cap one step at 10% of the integrator range, so a single glitched
        // sample cannot swing it wildly. Upstream cites ArduPilot issue #4066.
        let integ_seb_range = integ_sebdot_max - integ_sebdot_min;
        let integ_seb_delta = constrain_value(
            sebdot_error * i_gain * dt,
            -integ_seb_range * 0.1,
            integ_seb_range * 0.1,
        );

        // predict the pitch that unconstrained integration would give
        self.pitch_dem_unc =
            (sebdot_dem_total + self.integ_sebdot + integ_seb_delta + self.integ_ke) / gain_inv;

        // Inhibit only when the integrator would push further past the limit it
        // has already exceeded; integrating back toward range stays allowed.
        let inhibit_integrator = ((self.pitch_dem_unc > inp.pitch_max) && integ_seb_delta > 0.0)
            || ((self.pitch_dem_unc < inp.pitch_min) && integ_seb_delta < 0.0);

        if !inhibit_integrator {
            self.integ_sebdot += integ_seb_delta;
            self.integ_ke += (inp.ske_est - inp.ske_dem) * self.ske_weighting * dt / time_constant;
        } else {
            // fade both integrators out while saturating
            let coef = 1.0 - dt / (dt + time_constant);
            self.integ_sebdot *= coef;
            self.integ_ke *= coef;
        }
        self.integ_sebdot = constrain_value(self.integ_sebdot, integ_sebdot_min, integ_sebdot_max);

        // the speed trim integrator may claim a quarter of the pitch range
        let ke_integ_limit = 0.25 * (inp.pitch_max - inp.pitch_min) * gain_inv;
        self.integ_ke = constrain_value(self.integ_ke, -ke_integ_limit, ke_integ_limit);

        self.pitch_dem_unc = (sebdot_dem_total + self.integ_sebdot + self.integ_ke) / gain_inv;

        if inp.is_gliding {
            // speed-to-pitch feed forward, gliding only
            self.pitch_dem_unc += (inp.tas_dem_adj - params.pitch_ff_v0) * params.pitch_ff_k;
        }

        self.pitch_dem = constrain_value(self.pitch_dem_unc, inp.pitch_min, inp.pitch_max);

        // Rate limit to respect the vertical acceleration limit. Also divides by
        // tas_state, likewise protected by the MIN_AIRSPEED floor.
        let ptch_rate_incr = dt * params.vert_acc_lim / inp.tas_state;
        if (self.pitch_dem - self.last_pitch_dem) > ptch_rate_incr {
            self.pitch_dem = self.last_pitch_dem + ptch_rate_incr;
        } else if (self.pitch_dem - self.last_pitch_dem) < -ptch_rate_incr {
            self.pitch_dem = self.last_pitch_dem - ptch_rate_incr;
        }
        self.last_pitch_dem = self.pitch_dem;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:961-1120.

    fn inputs() -> PitchInputs {
        PitchInputs {
            spe_dem: 100.0 * GRAVITY_MSS,
            ske_dem: 0.5 * 20.0 * 20.0,
            spe_est: 100.0 * GRAVITY_MSS,
            ske_est: 0.5 * 20.0 * 20.0,
            spedot: 0.0,
            skedot: 0.0,
            hgt_rate_dem: 0.0,
            tas_state: 20.0,
            tas_dem_adj: 20.0,
            pitch_min: -0.35,
            pitch_max: 0.35,
            path_proportion: 0.0,
            use_airspeed: true,
            underspeed: false,
            is_gliding: false,
            is_doing_auto_land: false,
            is_flaring: false,
            flight_stage: FlightStage::Normal,
        }
    }

    /// The weighting chain is ordered, and the order decides which mode applies
    /// when two conditions hold at once.
    #[test]
    fn weighting_chain_order_is_respected() {
        let p = TecsParams::default();
        let d = PitchDemand::new();

        // no airspeed beats everything, including underspeed
        let mut i = inputs();
        i.use_airspeed = false;
        i.underspeed = true;
        assert_eq!(d.ske_weighting(&p, &i), 0.0, "no-airspeed wins");

        // VTOL beats underspeed
        let mut i = inputs();
        i.flight_stage = FlightStage::Vtol;
        i.underspeed = true;
        assert_eq!(d.ske_weighting(&p, &i), 0.0, "VTOL wins over underspeed");

        // underspeed beats landing
        let mut i = inputs();
        i.underspeed = true;
        i.is_doing_auto_land = true;
        assert_eq!(d.ske_weighting(&p, &i), 2.0, "underspeed wins over landing");

        // normal cruise takes the plain parameter
        assert_eq!(d.ske_weighting(&p, &inputs()), 1.0);
    }

    /// Takeoff, abort and gliding all pin the weighting to full speed priority.
    #[test]
    fn speed_priority_cases() {
        let p = TecsParams::default();
        let d = PitchDemand::new();
        for stage in [FlightStage::Takeoff, FlightStage::AbortLanding] {
            let mut i = inputs();
            i.flight_stage = stage;
            assert_eq!(d.ske_weighting(&p, &i), 2.0, "{stage:?}");
        }
        let mut i = inputs();
        i.is_gliding = true;
        assert_eq!(d.ske_weighting(&p, &i), 2.0, "gliding");
    }

    /// A negative land weight is the automatic sentinel: slide from the normal
    /// weight to zero as the landing progresses.
    #[test]
    fn negative_land_weight_slides_to_zero() {
        let p = TecsParams::default(); // spd_weight 1.0, spd_weight_land -1.0
        let d = PitchDemand::new();
        let mut i = inputs();
        i.is_doing_auto_land = true;

        i.path_proportion = 0.0;
        assert_eq!(d.ske_weighting(&p, &i), 1.0, "start of path: normal weight");
        i.path_proportion = 0.5;
        assert_eq!(d.ske_weighting(&p, &i), 0.5);
        i.path_proportion = 1.0;
        assert_eq!(d.ske_weighting(&p, &i), 0.0, "touchdown: height only");
        // beyond the end stays clamped
        i.path_proportion = 5.0;
        assert_eq!(d.ske_weighting(&p, &i), 0.0);
    }

    /// A set land weight is used directly rather than the sliding scale.
    #[test]
    fn positive_land_weight_is_used_directly() {
        let p = TecsParams {
            spd_weight_land: 1.5,
            ..Default::default()
        };
        let d = PitchDemand::new();
        let mut i = inputs();
        i.is_doing_auto_land = true;
        i.path_proportion = 0.5;
        assert_eq!(d.ske_weighting(&p, &i), 1.5);
    }

    /// Neither weight may exceed 1, even though SKE_weighting ranges to 2.
    #[test]
    fn weights_are_capped_at_one() {
        let p = TecsParams::default();
        let mut d = PitchDemand::new();
        let mut i = inputs();
        i.underspeed = true; // selects a raw weighting of 2

        d.update(&p, &i, 5.0, 0.3, 0.02);
        assert_eq!(d.ske_weighting, 1.0, "capped from 2 to 1");
    }

    /// A height deficit commands nose up, a surplus nose down.
    #[test]
    fn height_error_drives_pitch_monotonically() {
        let p = TecsParams::default();

        let run = |spe_dem: f32| {
            let mut i = inputs();
            i.spe_dem = spe_dem;
            let mut d = PitchDemand::new();
            // several steps so the rate limiter does not dominate
            for _ in 0..50 {
                d.update(&p, &i, 5.0, 0.3, 0.02);
            }
            d.pitch_dem
        };

        let level = run(100.0 * GRAVITY_MSS);
        let climb = run(110.0 * GRAVITY_MSS);
        let descend = run(90.0 * GRAVITY_MSS);

        assert!(climb > level, "climb {climb} should exceed level {level}");
        assert!(
            descend < level,
            "descend {descend} should be under level {level}"
        );
    }

    /// Pitch demand is rate limited by the vertical acceleration limit.
    #[test]
    fn pitch_demand_is_rate_limited() {
        let p = TecsParams::default(); // vert_acc_lim 7
        let mut i = inputs();
        i.spe_dem = 1000.0 * GRAVITY_MSS; // enormous climb demand

        let mut d = PitchDemand::new();
        d.update(&p, &i, 5.0, 0.3, 0.02);
        // one step allows dt * vert_acc_lim / tas = 0.02*7/20 = 0.007 rad
        assert!(
            d.pitch_dem <= 0.007 + 1e-6,
            "rate limited step, got {}",
            d.pitch_dem
        );
    }

    /// The demand stays inside the pitch limits.
    #[test]
    fn pitch_demand_respects_limits() {
        let p = TecsParams::default();
        let mut i = inputs();
        i.spe_dem = 1000.0 * GRAVITY_MSS;

        let mut d = PitchDemand::new();
        for _ in 0..500 {
            d.update(&p, &i, 5.0, 0.3, 0.02);
        }
        assert!(d.pitch_dem <= i.pitch_max + 1e-6, "got {}", d.pitch_dem);
        assert!(d.pitch_dem >= i.pitch_min - 1e-6);
    }

    /// Bad descent needs a large deficit, falling energy AND high throttle.
    #[test]
    fn bad_descent_requires_all_three_conditions() {
        let mut d = PitchDemand::new();
        let i = inputs();

        // deficit and falling energy but throttle low: not a bad descent
        assert!(!d.detect_bad_descent(&i, 300.0, 0.5, 1.0));
        // high throttle but small deficit: not a bad descent
        assert!(!d.detect_bad_descent(&i, 50.0, 0.95, 1.0));
        // all three: latched
        let mut falling = inputs();
        falling.spedot = -5.0;
        assert!(d.detect_bad_descent(&falling, 300.0, 0.95, 1.0));
    }

    /// Once latched it persists until the energy error returns to zero.
    #[test]
    fn bad_descent_latches_until_error_clears() {
        let mut d = PitchDemand::new();
        let mut i = inputs();
        i.spedot = -5.0;
        assert!(d.detect_bad_descent(&i, 300.0, 0.95, 1.0));

        // throttle backed off but error still positive: stays latched
        assert!(d.detect_bad_descent(&i, 100.0, 0.2, 1.0));
        // error clears: releases
        assert!(!d.detect_bad_descent(&i, -1.0, 0.2, 1.0));
    }

    /// Gliding, VTOL and underspeed all suppress detection - each loses energy
    /// legitimately.
    #[test]
    fn bad_descent_suppressed_when_energy_loss_is_expected() {
        let mut i = inputs();
        i.spedot = -5.0;

        for (label, mutate) in [("gliding", 0usize), ("vtol", 1), ("underspeed", 2)] {
            let mut d = PitchDemand::new();
            let mut j = i;
            match mutate {
                0 => j.is_gliding = true,
                1 => j.flight_stage = FlightStage::Vtol,
                _ => j.underspeed = true,
            }
            assert!(!d.detect_bad_descent(&j, 300.0, 0.95, 1.0), "{label}");
        }
    }

    /// The energy-balance-rate clip status feeds the height demand limiter, so
    /// it must report which bound was hit.
    #[test]
    fn sebdot_clip_status_is_reported() {
        let p = TecsParams::default();
        let mut i = inputs();

        let mut up = PitchDemand::new();
        i.hgt_rate_dem = 100.0; // far beyond max climb
        up.update(&p, &i, 5.0, 0.3, 0.02);
        assert_eq!(up.sebdot_dem_clip, ClipStatus::Max);

        let mut down = PitchDemand::new();
        i.hgt_rate_dem = -100.0;
        down.update(&p, &i, 5.0, 0.3, 0.02);
        assert_eq!(down.sebdot_dem_clip, ClipStatus::Min);

        let mut none = PitchDemand::new();
        i.hgt_rate_dem = 0.0;
        none.update(&p, &i, 5.0, 0.3, 0.02);
        assert_eq!(none.sebdot_dem_clip, ClipStatus::None);
    }

    /// The minimum airspeed floor from the speed filter is what keeps the two
    /// divisions here finite. At the floor the result must still be sane.
    #[test]
    fn survives_minimum_airspeed() {
        let p = TecsParams::default();
        let mut i = inputs();
        i.tas_state = crate::speed::MIN_AIRSPEED;

        let mut d = PitchDemand::new();
        d.update(&p, &i, 5.0, 0.3, 0.02);
        assert!(d.pitch_dem.is_finite(), "got {}", d.pitch_dem);
        assert!(d.pitch_dem_unc.is_finite());
    }
}
