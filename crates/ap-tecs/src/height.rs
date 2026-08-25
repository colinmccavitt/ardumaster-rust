//! Height demand shaping, ported from `AP_TECS::_update_height_demand`.
//!
//! Turns the raw commanded height into a rate-limited, lag-compensated demand
//! the energy controller can track, and adapts the climb and sink limits when
//! the aircraft cannot keep up.
//!
//! Two distinct branches, matching upstream:
//! - **normal**: two-point average, rate limiting, first-order lag, and
//!   adaptive climb/sink scalers
//! - **flaring**: height filter states are pinned to the current height and the
//!   demand is driven kinematically from the flare sink rate
//!
//! # DIVERGENCE D-008
//!
//! Upstream divides by `_hgt_dem_tconst` unguarded while the *adjacent* line
//! guards the same value with `MAX(_hgt_dem_tconst, _DT)`. See DIVERGENCES.md.

use ap_math::scalar::{constrain_value, is_negative};

use crate::params::{FlightStage, TecsParams};
use crate::speed::ClipStatus;
use crate::util::{max_f32, min_f32};

/// Everything `_update_height_demand` reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct HeightInputs {
    /// Raw commanded height, upstream `_hgt_dem_in`.
    pub hgt_dem_in: f32,
    /// Current height, upstream `_height`.
    pub height: f32,
    /// Height above field elevation, upstream `_hgt_afe`.
    pub hgt_afe: f32,
    /// Whether the vehicle is flaring, upstream `_landing.is_flaring()`.
    pub is_flaring: bool,
    /// Whether an automatic landing is in progress, upstream
    /// `_flags.is_doing_auto_land`.
    pub is_doing_auto_land: bool,
    /// Unconstrained pitch demand, upstream `_pitch_dem_unc`.
    pub pitch_dem_unc: f32,
    /// Maximum pitch, upstream `_PITCHmaxf`.
    pub pitch_max: f32,
    /// Minimum pitch, upstream `_PITCHminf`.
    pub pitch_min: f32,
    /// Balance-energy-rate clip state, upstream `_SEBdot_dem_clip`.
    pub sebdot_dem_clip: ClipStatus,
    /// Throttle clip state, upstream `_thr_clip_status`.
    pub thr_clip_status: ClipStatus,
    /// Whether airspeed drives throttle, upstream `_using_airspeed_for_throttle`.
    pub using_airspeed_for_throttle: bool,
    /// Current flight stage.
    pub flight_stage: FlightStage,
    /// Distance flown beyond the landing waypoint.
    pub distance_beyond_land_wp: f32,
}

/// Height demand state carried between updates.
#[derive(Debug, Clone, Copy)]
pub struct HeightDemand {
    /// Rate-limited height demand, upstream `_hgt_dem_rate_ltd`.
    pub hgt_dem_rate_ltd: f32,
    /// Previous raw input, upstream `_hgt_dem_in_prev`.
    ///
    /// Shared with the demand-freeze check in [`crate::tecs`]: upstream keeps
    /// exactly one of these and both readers must see the same value.
    pub hgt_dem_in_prev: f32,
    /// Low-passed height demand, upstream `_hgt_dem_lpf`.
    pub hgt_dem_lpf: f32,
    /// Final height demand, upstream `_hgt_dem`.
    pub hgt_dem: f32,
    /// Previous final demand, upstream `_hgt_dem_prev`.
    pub hgt_dem_prev: f32,
    /// Height rate demand, upstream `_hgt_rate_dem`.
    pub hgt_rate_dem: f32,
    /// Post-takeoff offset, decayed with the demand filter, upstream
    /// `_post_TO_hgt_offset`.
    pub post_to_hgt_offset: f32,
    /// How far into the sink limit the demand is, upstream `_sink_fraction`.
    pub sink_fraction: f32,
    /// Adaptive climb limit scaler, upstream `_max_climb_scaler`.
    pub max_climb_scaler: f32,
    /// Adaptive sink limit scaler, upstream `_max_sink_scaler`.
    pub max_sink_scaler: f32,
    /// Effective climb rate limit, upstream `_climb_rate_limit`.
    pub climb_rate_limit: f32,
    /// Effective sink rate limit, upstream `_sink_rate_limit`.
    pub sink_rate_limit: f32,

    // flare state
    flare_initialised: bool,
    flare_hgt_dem_adj: f32,
    flare_hgt_dem_ideal: f32,
    hgt_at_start_of_flare: f32,
    hgt_rate_dem_at_flare_entry: f32,
}

impl Default for HeightDemand {
    fn default() -> Self {
        Self {
            hgt_dem_rate_ltd: 0.0,
            hgt_dem_in_prev: 0.0,
            hgt_dem_lpf: 0.0,
            hgt_dem: 0.0,
            hgt_dem_prev: 0.0,
            hgt_rate_dem: 0.0,
            post_to_hgt_offset: 0.0,
            sink_fraction: 0.0,
            // scalers start at unity: no adaptation until the aircraft
            // demonstrates it cannot keep up
            max_climb_scaler: 1.0,
            max_sink_scaler: 1.0,
            climb_rate_limit: 0.0,
            sink_rate_limit: 0.0,
            flare_initialised: false,
            flare_hgt_dem_adj: 0.0,
            flare_hgt_dem_ideal: 0.0,
            hgt_at_start_of_flare: 0.0,
            hgt_rate_dem_at_flare_entry: 0.0,
        }
    }
}

impl HeightDemand {
    /// A demand shaper in its rest state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the flare profile has been initialised.
    pub fn flare_initialised(&self) -> bool {
        self.flare_initialised
    }

    /// Height above field elevation when the flare began, upstream
    /// `_hgt_at_start_of_flare`.
    ///
    /// Set on flare entry by the height stage and read by the pitch-limit
    /// stage, which blends the minimum pitch toward the touchdown value as
    /// the aircraft descends through it.
    pub fn hgt_at_start_of_flare(&self) -> f32 {
        self.hgt_at_start_of_flare
    }

    /// Clear the flare latch, so a re-entered flare re-seeds its profile.
    pub fn reset_flare(&mut self) {
        self.flare_initialised = false;
    }

    /// One `_update_height_demand` step.
    pub fn update(&mut self, params: &TecsParams, inp: &HeightInputs, dt: f32) {
        self.climb_rate_limit = params.max_climb_rate * self.max_climb_scaler;
        self.sink_rate_limit = params.max_sink_rate * self.max_sink_scaler;

        if params.max_sink_rate_approach > 0.0 && inp.is_doing_auto_land {
            // steeper approaches and reverse thrust get their own sink limit
            self.sink_rate_limit = params.max_sink_rate_approach;
        }

        if !inp.is_flaring {
            self.update_normal(params, inp, dt);
        } else {
            self.update_flare(params, inp, dt);
        }
    }

    fn update_normal(&mut self, params: &TecsParams, inp: &HeightInputs, dt: f32) {
        // two point moving average on the raw demand
        let hgt_dem = 0.5 * (inp.hgt_dem_in + self.hgt_dem_in_prev);
        self.hgt_dem_in_prev = inp.hgt_dem_in;

        // rate limit
        if (hgt_dem - self.hgt_dem_rate_ltd) > (self.climb_rate_limit * dt) {
            self.hgt_dem_rate_ltd += self.climb_rate_limit * dt;
            self.sink_fraction = 0.0;
        } else if (hgt_dem - self.hgt_dem_rate_ltd) < (-self.sink_rate_limit * dt) {
            self.hgt_dem_rate_ltd -= self.sink_rate_limit * dt;
            self.sink_fraction = 1.0;
        } else {
            // Guarded division: BOTH must be negative before dividing, which
            // also rules out the zero-denominator case.
            let numerator = hgt_dem - self.hgt_dem_rate_ltd;
            let denominator = -self.sink_rate_limit * dt;
            self.sink_fraction = if is_negative(numerator) && is_negative(denominator) {
                numerator / denominator
            } else {
                0.0
            };
            self.hgt_dem_rate_ltd = hgt_dem;
        }

        // first order lag, with post-takeoff offset decayed on the same constant
        let coef = min_f32(dt / (dt + max_f32(params.hgt_dem_tconst, dt)), 1.0);

        // DIVERGENCE D-008: upstream divides by params.hgt_dem_tconst raw here,
        // while the `coef` line directly above guards the same value with
        // MAX(tconst, dt). For every in-range tconst (documented 1.0..5.0) with
        // a normal dt, MAX(tconst, dt) == tconst, so this is identical; it
        // differs only where upstream would divide by ~zero. See DIVERGENCES.md.
        let tconst = max_f32(params.hgt_dem_tconst, dt);
        self.hgt_rate_dem = (self.hgt_dem_rate_ltd - self.hgt_dem_lpf) / tconst;

        self.hgt_dem_lpf = self.hgt_dem_rate_ltd * coef + (1.0 - coef) * self.hgt_dem_lpf;
        self.post_to_hgt_offset *= 1.0 - coef;
        self.hgt_dem = self.hgt_dem_lpf + self.post_to_hgt_offset;

        if inp.is_doing_auto_land {
            // compensate the filter lag on approach
            self.hgt_dem += params.hgt_dem_tconst * self.hgt_rate_dem;
        } else {
            // Do not let the demand run away from a vehicle that cannot follow
            // it; wind the corresponding limit scaler down instead.
            let mut max_climb_condition =
                (inp.pitch_dem_unc > inp.pitch_max) || (inp.sebdot_dem_clip == ClipStatus::Max);
            let mut max_descent_condition =
                (inp.pitch_dem_unc < inp.pitch_min) || (inp.sebdot_dem_clip == ClipStatus::Min);

            if inp.using_airspeed_for_throttle {
                // a saturated throttle also means the demand is unfollowable,
                // except during takeoff or a landing abort where saturation is
                // expected and must not shrink the limit
                max_climb_condition |= (inp.thr_clip_status == ClipStatus::Max)
                    && !matches!(
                        inp.flight_stage,
                        FlightStage::Takeoff | FlightStage::AbortLanding
                    );
                max_descent_condition |=
                    (inp.thr_clip_status == ClipStatus::Min) && !inp.is_flaring;
            }

            let alpha = dt / max_f32(dt + params.hgt_dem_tconst, dt);
            if max_climb_condition && self.hgt_dem > self.hgt_dem_prev {
                self.max_climb_scaler *= 1.0 - alpha;
            } else if max_descent_condition && self.hgt_dem < self.hgt_dem_prev {
                self.max_sink_scaler *= 1.0 - alpha;
            } else {
                // recover both scalers toward unity
                self.max_climb_scaler = self.max_climb_scaler * (1.0 - alpha) + alpha;
                self.max_sink_scaler = self.max_sink_scaler * (1.0 - alpha) + alpha;
            }
        }
        self.hgt_dem_prev = self.hgt_dem;
    }

    fn update_flare(&mut self, params: &TecsParams, inp: &HeightInputs, dt: f32) {
        // Pin the filter states to current height so an aborted flare does not
        // produce a large pitch transient.
        self.hgt_dem_lpf = inp.height;
        self.hgt_dem_rate_ltd = inp.height;
        self.hgt_dem_in_prev = inp.height;

        if !self.flare_initialised {
            self.flare_hgt_dem_adj = self.hgt_dem;
            self.flare_hgt_dem_ideal = inp.height;
            self.hgt_at_start_of_flare = inp.hgt_afe;
            self.hgt_rate_dem_at_flare_entry = self.hgt_rate_dem;
            self.flare_initialised = true;
        }

        // sink faster or slower the further past the landing waypoint
        let land_sink_rate_adj =
            params.land_sink + params.land_sink_rate_change * inp.distance_beyond_land_wp;

        // blend in linearly with height
        let p = if self.hgt_at_start_of_flare > params.flare_holdoff_hgt {
            constrain_value(
                (self.hgt_at_start_of_flare - inp.hgt_afe)
                    / (self.hgt_at_start_of_flare - params.flare_holdoff_hgt),
                0.0,
                1.0,
            )
        } else {
            1.0
        };

        self.hgt_rate_dem = self.hgt_rate_dem_at_flare_entry * (1.0 - p) - land_sink_rate_adj * p;

        // integrate both the ideal profile and the offset-carrying one
        self.flare_hgt_dem_ideal += dt * self.hgt_rate_dem;
        self.flare_hgt_dem_adj += dt * self.hgt_rate_dem;

        // fade from the offset profile onto the ideal one
        self.hgt_dem = self.flare_hgt_dem_adj * (1.0 - p) + self.flare_hgt_dem_ideal * p;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:536-645.

    fn inputs() -> HeightInputs {
        HeightInputs {
            hgt_dem_in: 100.0,
            height: 100.0,
            hgt_afe: 100.0,
            is_flaring: false,
            is_doing_auto_land: false,
            pitch_dem_unc: 0.0,
            pitch_max: 0.3,
            pitch_min: -0.3,
            sebdot_dem_clip: ClipStatus::None,
            thr_clip_status: ClipStatus::None,
            using_airspeed_for_throttle: true,
            flight_stage: FlightStage::Normal,
            distance_beyond_land_wp: 0.0,
        }
    }

    /// Climb is limited to max_climb_rate * dt per step, not applied instantly.
    #[test]
    fn climb_rate_is_limited() {
        let p = TecsParams::default(); // max_climb_rate 5 m/s
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.hgt_dem_in = 1000.0; // far above

        h.update(&p, &i, 0.1);
        // two-point average halves the first step, then the limit applies
        assert!(
            h.hgt_dem_rate_ltd <= 5.0 * 0.1 + 1e-6,
            "climbed {} in one step",
            h.hgt_dem_rate_ltd
        );
        assert_eq!(h.sink_fraction, 0.0, "climbing is not sinking");
    }

    /// Sink is limited symmetrically, and sink_fraction saturates at 1.
    #[test]
    fn sink_rate_is_limited_and_fraction_saturates() {
        let p = TecsParams::default(); // max_sink_rate 5 m/s
        let mut h = HeightDemand::new();
        h.hgt_dem_rate_ltd = 1000.0;
        h.hgt_dem_in_prev = 1000.0;
        let mut i = inputs();
        i.hgt_dem_in = 0.0;

        h.update(&p, &i, 0.1);
        assert_eq!(h.sink_fraction, 1.0, "saturated sink");
        assert!(h.hgt_dem_rate_ltd >= 1000.0 - 5.0 * 0.1 - 1e-3);
    }

    /// Inside the rate limits, sink_fraction reports how far into the sink
    /// allowance the demand sits.
    #[test]
    fn sink_fraction_is_proportional_within_the_limits() {
        let p = TecsParams::default(); // max_sink_rate 5 => denominator -0.1
        let mut h = HeightDemand::new();
        h.hgt_dem_rate_ltd = 100.0;
        h.hgt_dem_in_prev = 100.0;
        let mut i = inputs();
        // two-point average gives 99.95, so numerator is -0.05: half of the
        // -0.1 sink allowance
        i.hgt_dem_in = 99.9;

        h.update(&p, &i, 0.02);
        assert!(
            (h.sink_fraction - 0.5).abs() < 1e-4,
            "expected half the sink allowance, got {}",
            h.sink_fraction
        );
    }

    /// The division is guarded by requiring BOTH operands negative.
    ///
    /// Note a zero sink limit does NOT reach this branch via a negative
    /// numerator - any negative delta then satisfies `< -0.0` and takes the
    /// sink branch instead. What the guard actually protects is a
    /// non-negative numerator arriving with a zero denominator.
    #[test]
    fn sink_fraction_division_is_guarded() {
        // zero sink limit makes the denominator -0.0
        let p = TecsParams {
            max_sink_rate: 0.0,
            ..Default::default()
        };
        let mut h = HeightDemand::new();
        h.hgt_dem_rate_ltd = 100.0;
        h.hgt_dem_in_prev = 100.0;
        let mut i = inputs();
        i.hgt_dem_in = 100.0; // numerator exactly zero

        h.update(&p, &i, 0.02);
        assert!(
            h.sink_fraction.is_finite(),
            "zero denominator must not divide, got {}",
            h.sink_fraction
        );
        assert_eq!(h.sink_fraction, 0.0);
    }

    /// DIVERGENCE D-008, pinned.
    ///
    /// UPSTREAM divides by `_hgt_dem_tconst` raw while the adjacent line guards
    /// the same value with MAX(tconst, dt). A zero tconst therefore produces
    /// inf/NaN in the height rate demand, which feeds pitch.
    /// PORTED applies the same MAX guard upstream uses one line later.
    ///
    /// For every in-range tconst this is IDENTICAL, which the second half of
    /// this test asserts - the divergence only bites where upstream breaks.
    #[test]
    fn d008_height_rate_demand_survives_zero_time_constant() {
        let p = TecsParams {
            hgt_dem_tconst: 0.0,
            ..Default::default()
        };
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.hgt_dem_in = 150.0;

        h.update(&p, &i, 0.02);
        assert!(
            h.hgt_rate_dem.is_finite(),
            "zero tconst must not produce inf, got {}",
            h.hgt_rate_dem
        );
        assert!(h.hgt_dem.is_finite());

        // and for an in-range tconst the guard changes nothing: MAX(3.0, 0.02)
        // is 3.0, so the divisor is exactly upstream's
        let mut p2 = TecsParams::default(); // hgt_dem_tconst 3.0
        let mut h2 = HeightDemand::new();
        h2.hgt_dem_rate_ltd = 10.0;
        h2.hgt_dem_lpf = 4.0;
        p2.max_climb_rate = 1000.0; // avoid the rate limiter interfering
        let mut i2 = inputs();
        i2.hgt_dem_in = 10.0;
        h2.hgt_dem_in_prev = 10.0;
        h2.update(&p2, &i2, 0.02);
        assert!((h2.hgt_rate_dem - (10.0 - 4.0) / 3.0).abs() < 1e-4);
    }

    /// Scalers start at unity and recover toward it when the aircraft is
    /// keeping up.
    #[test]
    fn scalers_recover_toward_unity() {
        let p = TecsParams::default();
        let mut h = HeightDemand::new();
        h.max_climb_scaler = 0.5;
        h.max_sink_scaler = 0.5;
        let i = inputs();

        for _ in 0..2000 {
            h.update(&p, &i, 0.02);
        }
        assert!(h.max_climb_scaler > 0.99, "got {}", h.max_climb_scaler);
        assert!(h.max_sink_scaler > 0.99, "got {}", h.max_sink_scaler);
        assert!(h.max_climb_scaler <= 1.0, "must not overshoot unity");
    }

    /// A saturated pitch demand winds the climb scaler down, so the demand
    /// stops running away from an aircraft that cannot follow it.
    #[test]
    fn unfollowable_climb_winds_the_scaler_down() {
        let p = TecsParams::default();
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.pitch_dem_unc = 1.0; // above pitch_max 0.3
        i.hgt_dem_in = 1000.0;

        let before = h.max_climb_scaler;
        for _ in 0..100 {
            h.update(&p, &i, 0.02);
        }
        assert!(
            h.max_climb_scaler < before,
            "scaler should shrink, got {}",
            h.max_climb_scaler
        );
    }

    /// Throttle saturation during TAKEOFF must NOT shrink the climb limit -
    /// saturation is expected there.
    #[test]
    fn takeoff_throttle_saturation_does_not_shrink_the_limit() {
        let p = TecsParams::default();
        let mut i = inputs();
        i.thr_clip_status = ClipStatus::Max;
        i.hgt_dem_in = 1000.0;

        let mut takeoff = HeightDemand::new();
        i.flight_stage = FlightStage::Takeoff;
        for _ in 0..100 {
            takeoff.update(&p, &i, 0.02);
        }

        let mut normal = HeightDemand::new();
        i.flight_stage = FlightStage::Normal;
        for _ in 0..100 {
            normal.update(&p, &i, 0.02);
        }

        assert!(
            takeoff.max_climb_scaler > normal.max_climb_scaler,
            "takeoff {} should exceed normal {}",
            takeoff.max_climb_scaler,
            normal.max_climb_scaler
        );
    }

    /// Entering the flare pins the filter states to current height, so an
    /// aborted flare does not produce a large pitch transient.
    #[test]
    fn flare_pins_filter_states_to_current_height() {
        let p = TecsParams::default();
        let mut h = HeightDemand::new();
        h.hgt_dem_lpf = 500.0;
        h.hgt_dem_rate_ltd = 500.0;
        let mut i = inputs();
        i.is_flaring = true;
        i.height = 12.0;
        i.hgt_afe = 12.0;

        h.update(&p, &i, 0.02);
        assert_eq!(h.hgt_dem_lpf, 12.0);
        assert_eq!(h.hgt_dem_rate_ltd, 12.0);
        assert_eq!(h.hgt_dem_in_prev, 12.0);
        assert!(h.flare_initialised());
    }

    /// The flare blends in AS THE AIRCRAFT DESCENDS.
    ///
    /// The blend factor is
    /// `p = (hgt_at_start_of_flare - hgt_afe) / (hgt_at_start_of_flare - holdoff)`,
    /// so holding height constant leaves p at zero and the configured sink
    /// rate never takes effect - the demand simply holds the entry rate. The
    /// height must actually fall for the flare profile to engage.
    #[test]
    fn flare_blends_in_as_height_falls() {
        let p = TecsParams::default(); // land_sink 0.25, flare_holdoff_hgt 1.0
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.is_flaring = true;
        i.height = 5.0;
        i.hgt_afe = 5.0;
        // A real flare is entered with the demand already tracking height. Left
        // at zero, flare_hgt_dem_adj seeds to 0 while flare_hgt_dem_ideal seeds
        // to 5, and the blend RISES toward the ideal profile instead of
        // descending - an artefact of the setup, not of the flare.
        h.hgt_dem = 5.0;

        // entering the flare at constant height: p == 0, so the rate demand is
        // whatever it was at entry (zero here), NOT the configured sink rate
        h.update(&p, &i, 0.02);
        assert_eq!(h.hgt_rate_dem, 0.0, "p is zero at flare entry");
        let first = h.hgt_dem;

        // now descend; p rises toward 1 and the sink rate blends in
        for n in 1..=50 {
            i.hgt_afe = 5.0 - 0.05 * n as f32;
            i.height = i.hgt_afe;
            h.update(&p, &i, 0.02);
        }
        assert!(
            h.hgt_rate_dem < 0.0,
            "sink rate should have blended in, got {}",
            h.hgt_rate_dem
        );
        assert!(
            h.hgt_dem < first,
            "demand should descend, got {}",
            h.hgt_dem
        );
    }

    /// At or below the hold-off height the blend is fully engaged, so the
    /// demand follows the configured flare sink rate directly.
    #[test]
    fn flare_below_holdoff_uses_full_sink_rate() {
        let p = TecsParams::default();
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.is_flaring = true;
        // start below the hold-off height, so p is forced to 1
        i.height = 0.5;
        i.hgt_afe = 0.5;

        h.update(&p, &i, 0.02);
        assert!(
            (h.hgt_rate_dem + p.land_sink).abs() < 1e-5,
            "expected -land_sink, got {}",
            h.hgt_rate_dem
        );
    }

    /// reset_flare re-arms the latch so a re-entered flare re-seeds.
    #[test]
    fn flare_latch_can_be_reset() {
        let p = TecsParams::default();
        let mut h = HeightDemand::new();
        let mut i = inputs();
        i.is_flaring = true;
        h.update(&p, &i, 0.02);
        assert!(h.flare_initialised());

        h.reset_flare();
        assert!(!h.flare_initialised());
    }
}
