//! Pitch limits, ported from `AP_TECS::_update_pitch_limits`.
//!
//! # Units: degrees until the last two lines
//!
//! This function works entirely in **degrees** and converts to radians once at
//! the end. Every limit source is degrees — the airframe `PTCH_LIM_*_DEG`
//! parameters, `TECS_PITCH_MIN`/`MAX`, the flare's `pitch_limit_deg`, the ±90
//! external bounds. Getting this wrong is not hypothetical: see D-009.
//!
//! # DIVERGENCE D-009
//!
//! Upstream's takeoff branch uses `cd_to_rad(ptchMinCO_cd)`, producing radians,
//! which the trailing `radians()` then converts *again* as if it were degrees.
//! The configured climbout minimum ends up 57.3× too small. The port converts
//! centidegrees to degrees so the single trailing conversion is correct. See
//! DIVERGENCES.md.

use ap_math::scalar::{constrain_value, radians};

use crate::params::{FlightStage, TecsParams};
use crate::util::{max_f32, min_f32};

/// Airframe pitch limits, upstream `aparm`. Degrees.
#[derive(Debug, Clone, Copy)]
pub struct AirframePitchLimits {
    /// `PTCH_LIM_MAX_DEG`.
    pub pitch_limit_max: f32,
    /// `PTCH_LIM_MIN_DEG`.
    pub pitch_limit_min: f32,
}

impl Default for AirframePitchLimits {
    fn default() -> Self {
        Self {
            pitch_limit_max: 20.0,
            pitch_limit_min: -25.0,
        }
    }
}

/// Everything the pitch limit stage reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct PitchLimitInputs {
    /// Whether flaring, upstream `_landing.is_flaring()`.
    pub is_flaring: bool,
    /// Whether on approach, upstream `_landing.is_on_approach()`.
    pub is_on_approach: bool,
    /// Landing pitch, centidegrees. Upstream `_landing.get_pitch_cd()`.
    pub landing_pitch_cd: f32,
    /// Height above field elevation, upstream `_hgt_afe`.
    pub hgt_afe: f32,
    /// Height at which the flare began, upstream `_hgt_at_start_of_flare`.
    pub hgt_at_start_of_flare: f32,
    /// Whether the flare profile has been initialised.
    pub flare_initialised: bool,
    /// Minimum climbout pitch, **centidegrees**. Upstream `ptchMinCO_cd`.
    pub pitch_min_climbout_cd: f32,
    /// External maximum pitch limit, degrees. Upstream `_PITCHmaxf_ext`.
    pub pitch_max_ext: f32,
    /// External minimum pitch limit, degrees. Upstream `_PITCHminf_ext`.
    pub pitch_min_ext: f32,
    /// Current flight stage.
    pub flight_stage: FlightStage,
}

impl Default for PitchLimitInputs {
    fn default() -> Self {
        Self {
            is_flaring: false,
            is_on_approach: false,
            landing_pitch_cd: 0.0,
            hgt_afe: 0.0,
            hgt_at_start_of_flare: 0.0,
            flare_initialised: false,
            pitch_min_climbout_cd: 0.0,
            // upstream resets these each call
            pitch_max_ext: 90.0,
            pitch_min_ext: -90.0,
            flight_stage: FlightStage::Normal,
        }
    }
}

/// Pitch limit state, carrying the landing hysteresis.
#[derive(Debug, Clone, Copy)]
pub struct PitchLimits {
    /// Maximum pitch, **radians**. Upstream `_PITCHmaxf`.
    pub pitch_max: f32,
    /// Minimum pitch, **radians**. Upstream `_PITCHminf`.
    pub pitch_min: f32,
    /// Ratcheted approach minimum, degrees. Upstream `_land_pitch_min`.
    land_pitch_min: f32,
    /// Minimum pitch when the flare began, degrees.
    /// Upstream `_pitch_min_at_flare_entry`.
    pitch_min_at_flare_entry: f32,
}

impl Default for PitchLimits {
    fn default() -> Self {
        Self {
            pitch_max: 0.0,
            pitch_min: 0.0,
            // upstream's sentinel meaning "not yet latched"
            land_pitch_min: -90.0,
            pitch_min_at_flare_entry: 0.0,
        }
    }
}

impl PitchLimits {
    /// Limits at rest.
    pub fn new() -> Self {
        Self::default()
    }

    /// One `_update_pitch_limits` step. Produces limits in **radians**.
    pub fn update(
        &mut self,
        params: &TecsParams,
        airframe: &AirframePitchLimits,
        inp: &PitchLimitInputs,
        dt: f32,
    ) -> bool {
        // --- degrees from here to the conversion at the end ---

        // zero is the unset sentinel: fall back to the airframe limit
        let mut pitch_max_deg = if params.pitch_max == 0 {
            airframe.pitch_limit_max
        } else {
            params.pitch_max as f32
        };
        let mut pitch_min_deg = if params.pitch_min == 0 {
            airframe.pitch_limit_min
        } else {
            params.pitch_min as f32
        };

        if !inp.is_on_approach {
            // release the ratchet when not landing
            self.land_pitch_min = pitch_min_deg;
        }

        let mut flare_initialised = inp.flare_initialised;

        if inp.is_flaring {
            // move the minimum smoothly to the touchdown value
            let p = if !flare_initialised {
                0.0
            } else if inp.hgt_at_start_of_flare > params.flare_holdoff_hgt {
                constrain_value(
                    (inp.hgt_at_start_of_flare - inp.hgt_afe) / inp.hgt_at_start_of_flare,
                    0.0,
                    1.0,
                )
            } else {
                1.0
            };
            let pitch_limit_deg =
                (1.0 - p) * self.pitch_min_at_flare_entry + p * 0.01 * inp.landing_pitch_cd;
            pitch_min_deg = max_f32(pitch_min_deg, pitch_limit_deg);

            // the flare may exceed the normal auto maximum
            if params.land_pitch_max != 0 {
                pitch_max_deg = params.land_pitch_max as f32;
            }
        } else if inp.is_on_approach {
            pitch_min_deg = max_f32(pitch_min_deg, airframe.pitch_limit_min);
            self.pitch_min_at_flare_entry = pitch_min_deg;
            flare_initialised = false;
        } else {
            flare_initialised = false;
        }

        if inp.is_on_approach {
            // Ratchet: the approach minimum may not decrease, and may only rise
            // slowly. Without this the demand oscillates as the time-to-flare
            // estimate jitters.
            if self.land_pitch_min <= -90.0 {
                self.land_pitch_min = pitch_min_deg;
            }
            const FLARE_PITCH_RANGE_DEG: f32 = 20.0;
            let delta_per_loop = (FLARE_PITCH_RANGE_DEG / params.land_time_const) * dt;
            pitch_min_deg = min_f32(pitch_min_deg, self.land_pitch_min + delta_per_loop);
            self.land_pitch_min = max_f32(self.land_pitch_min, pitch_min_deg);
            pitch_min_deg = max_f32(self.land_pitch_min, pitch_min_deg);
        }

        if matches!(
            inp.flight_stage,
            FlightStage::Takeoff | FlightStage::AbortLanding
        ) {
            // DIVERGENCE D-009: upstream uses cd_to_rad here, producing radians
            // that the trailing radians() then converts again as degrees,
            // making the climbout minimum 57.3x too small. Centidegrees to
            // DEGREES is what the rest of this function expects.
            pitch_min_deg = inp.pitch_min_climbout_cd * 0.01;
        }

        // external limits, also degrees
        pitch_max_deg = min_f32(pitch_max_deg, inp.pitch_max_ext);
        pitch_min_deg = max_f32(pitch_min_deg, inp.pitch_min_ext);

        // --- the single conversion to radians ---
        self.pitch_max = radians(pitch_max_deg);
        self.pitch_min = radians(pitch_min_deg);

        // never let the band invert
        self.pitch_max = max_f32(self.pitch_max, self.pitch_min);

        flare_initialised
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:1494-1578.

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "expected {b}, got {a}");
    }

    /// DIVERGENCE D-009, pinned.
    ///
    /// UPSTREAM applies `cd_to_rad` and then `radians()` again, scaling the
    /// climbout minimum by pi/180 — a configured 10 degrees becomes 0.175.
    /// PORTED converts centidegrees to degrees so the single trailing
    /// conversion is correct.
    #[test]
    fn d009_takeoff_pitch_min_uses_degrees() {
        let p = TecsParams::default();
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        let mut i = PitchLimitInputs {
            flight_stage: FlightStage::Takeoff,
            pitch_min_climbout_cd: 1000.0, // 10 degrees
            ..Default::default()
        };

        l.update(&p, &a, &i, 0.02);
        near(l.pitch_min, radians(10.0_f32));

        // what upstream's double conversion would have produced, for contrast
        let upstream_would_be = radians(1000.0_f32 * (core::f32::consts::PI / 18000.0));
        assert!(
            (l.pitch_min - upstream_would_be).abs() > 0.1,
            "port {} must differ from upstream double conversion {}",
            l.pitch_min,
            upstream_would_be
        );
        assert!(
            upstream_would_be < 0.01,
            "upstream value collapses to near zero: {upstream_would_be}"
        );

        // abort-landing takes the same path
        i.flight_stage = FlightStage::AbortLanding;
        let mut l2 = PitchLimits::new();
        l2.update(&p, &a, &i, 0.02);
        near(l2.pitch_min, radians(10.0_f32));
    }

    /// Zero is the unset sentinel for both TECS pitch parameters: fall back to
    /// the airframe limit rather than clamping to zero.
    #[test]
    fn zero_tecs_limits_fall_back_to_airframe() {
        let p = TecsParams {
            pitch_max: 0,
            pitch_min: 0,
            ..Default::default()
        };
        let a = AirframePitchLimits::default(); // 20 / -25
        let mut l = PitchLimits::new();
        l.update(&p, &a, &PitchLimitInputs::default(), 0.02);

        near(l.pitch_max, radians(20.0_f32));
        near(l.pitch_min, radians(-25.0_f32));
    }

    /// Set TECS limits are used in preference to the airframe ones.
    #[test]
    fn set_tecs_limits_take_precedence() {
        let p = TecsParams::default(); // pitch_max 15, pitch_min 0
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        l.update(&p, &a, &PitchLimitInputs::default(), 0.02);

        near(l.pitch_max, radians(15.0_f32));
        // pitch_min is 0, the sentinel, so the airframe value applies
        near(l.pitch_min, radians(-25.0_f32));
    }

    /// External limits narrow the band and are applied in degrees.
    #[test]
    fn external_limits_narrow_the_band() {
        let p = TecsParams::default();
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        let i = PitchLimitInputs {
            pitch_max_ext: 5.0,
            pitch_min_ext: -5.0,
            ..Default::default()
        };
        l.update(&p, &a, &i, 0.02);

        near(l.pitch_max, radians(5.0_f32));
        near(l.pitch_min, radians(-5.0_f32));
    }

    /// The band may never invert.
    #[test]
    fn band_cannot_invert() {
        let p = TecsParams::default();
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        let i = PitchLimitInputs {
            // force max below min
            pitch_max_ext: -30.0,
            pitch_min_ext: 10.0,
            ..Default::default()
        };
        l.update(&p, &a, &i, 0.02);
        assert!(l.pitch_max >= l.pitch_min, "band inverted");
    }

    /// On approach the minimum ratchets: it may not fall, and may only rise
    /// slowly, which stops the demand oscillating as the flare estimate jitters.
    #[test]
    fn approach_minimum_ratchets_upward_slowly() {
        let p = TecsParams::default(); // land_time_const 2.0
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        let i = PitchLimitInputs {
            is_on_approach: true,
            ..Default::default()
        };

        l.update(&p, &a, &i, 0.02);
        let first = l.pitch_min;

        // the ratchet permits at most (20/2)*0.02 = 0.2 deg per step
        for _ in 0..5 {
            l.update(&p, &a, &i, 0.02);
        }
        let after = l.pitch_min;
        assert!(after >= first - 1e-6, "minimum must not fall on approach");
        assert!(
            after - first <= radians(1.5_f32),
            "rise should be slow, got {} deg",
            (after - first) * 180.0 / core::f32::consts::PI
        );
    }

    /// The flare may exceed the normal auto maximum when LAND_PMAX is set.
    #[test]
    fn flare_can_exceed_the_normal_pitch_maximum() {
        let p = TecsParams {
            pitch_max: 5,
            land_pitch_max: 12,
            ..Default::default()
        };
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();
        let i = PitchLimitInputs {
            is_flaring: true,
            flare_initialised: true,
            hgt_at_start_of_flare: 5.0,
            hgt_afe: 5.0,
            ..Default::default()
        };
        l.update(&p, &a, &i, 0.02);
        near(l.pitch_max, radians(12.0_f32));
    }

    /// Leaving the approach releases the ratchet so it re-arms next time.
    #[test]
    fn leaving_approach_releases_the_ratchet() {
        let p = TecsParams::default();
        let a = AirframePitchLimits::default();
        let mut l = PitchLimits::new();

        let approach = PitchLimitInputs {
            is_on_approach: true,
            ..Default::default()
        };
        l.update(&p, &a, &approach, 0.02);

        // off approach: land_pitch_min tracks the plain minimum again
        let normal = PitchLimitInputs::default();
        l.update(&p, &a, &normal, 0.02);
        near(l.pitch_min, radians(-25.0_f32));
    }
}
