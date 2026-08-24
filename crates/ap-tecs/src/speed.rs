//! Airspeed estimation, ported from `AP_TECS::_update_speed` and
//! `AP_TECS::timeConstant`.
//!
//! Produces `TAS_state`, the smoothed true-airspeed estimate every later stage
//! depends on, plus the airspeed limits `TAS_min`/`TAS_max`.
//!
//! # Injected inputs rather than reached-for globals
//!
//! Upstream reads `_ahrs`, `AP::ins()`, `_landing` and `aparm` directly. Per
//! ADR-0004 those become [`SpeedInputs`], supplied by the caller. Under ADR-0008
//! replay they come from a recorded fixture, which is what lets this be verified
//! before any AHRS, INS or airspeed driver exists.

use ap_filter::average::AverageFilter;
use ap_math::scalar::{constrain_value, is_positive, safe_sqrt};

use crate::params::TecsParams;
use crate::util::{max_f32, min_f32};

/// Standard gravity, upstream `GRAVITY_MSS`.
pub const GRAVITY_MSS: f32 = 9.80665;

/// Lower bound on the airspeed estimate, upstream's `min_airspeed`.
///
/// Not a tuning parameter: a hard floor so downstream divisions by airspeed
/// cannot blow up.
pub const MIN_AIRSPEED: f32 = 3.0;

/// Airframe airspeed limits, upstream's `aparm` (`AP_FixedWing`).
#[derive(Debug, Clone, Copy)]
pub struct AirspeedLimits {
    /// `ARSPD_FBW_MIN`, m/s equivalent airspeed.
    pub airspeed_min: f32,
    /// `ARSPD_FBW_MAX`, m/s equivalent airspeed.
    pub airspeed_max: f32,
    /// `TRIM_ARSPD_CM` converted to m/s.
    pub airspeed_cruise: f32,
    /// Stall speed, m/s; non-positive means unset.
    pub airspeed_stall: f32,
    /// Whether stall prevention raises the minimum airspeed with load factor.
    pub stall_prevention: bool,
}

impl Default for AirspeedLimits {
    fn default() -> Self {
        Self {
            airspeed_min: 9.0,
            airspeed_max: 22.0,
            airspeed_cruise: 12.0,
            airspeed_stall: 0.0,
            stall_prevention: true,
        }
    }
}

/// Everything `_update_speed` reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct SpeedInputs {
    /// `rotMat.c.x` from the body-to-NED DCM: the x component of the third row.
    pub rot_mat_c_x: f32,
    /// Body-frame x acceleration, upstream `AP::ins().get_accel().x`.
    pub accel_x: f32,
    /// Equivalent-to-true airspeed ratio, upstream `_ahrs.get_EAS2TAS()`.
    pub eas2tas: f32,
    /// Measured equivalent airspeed, or `None` when unavailable.
    ///
    /// Upstream signals this with `airspeed_EAS()`'s bool return; `Option`
    /// makes the absent case impossible to read as a zero measurement.
    pub eas: Option<f32>,
    /// Whether airspeed should be used at all, upstream `use_airspeed()`.
    pub should_use_airspeed: bool,
    /// Whether the vehicle is on final approach, upstream `_landing.is_on_final()`.
    pub is_on_final: bool,
    /// Aerodynamic load factor, used by stall prevention.
    pub load_factor: f32,
}

/// Throttle clipping state, upstream `clipStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClipStatus {
    /// Not clipping.
    #[default]
    None,
    /// Clipped at the minimum.
    Min,
    /// Clipped at the maximum.
    Max,
}

/// The speed-estimation state carried between updates.
#[derive(Debug, Clone, Copy)]
pub struct SpeedState {
    /// Smoothed true airspeed estimate, upstream `_TAS_state`.
    pub tas_state: f32,
    /// Complementary filter integrator, upstream `_integDTAS_state`.
    pub integ_dtas_state: f32,
    /// Five-point moving average of speed rate of change, upstream `_vel_dot`.
    pub vel_dot: f32,
    /// Low-pass filtered `vel_dot`, upstream `_vel_dot_lpf`.
    pub vel_dot_lpf: f32,
    /// Demanded true airspeed, upstream `_TAS_dem`.
    pub tas_dem: f32,
    /// Upper airspeed limit, upstream `_TASmax`.
    pub tas_max: f32,
    /// Lower airspeed limit, upstream `_TASmin`.
    pub tas_min: f32,
    /// Upstream `_vdot_filter`, a 5-point moving average.
    vdot_filter: AverageFilter<f32, 5>,
}

impl Default for SpeedState {
    fn default() -> Self {
        Self {
            tas_state: 0.0,
            integ_dtas_state: 0.0,
            vel_dot: 0.0,
            vel_dot_lpf: 0.0,
            tas_dem: 0.0,
            tas_max: 0.0,
            tas_min: 0.0,
            vdot_filter: AverageFilter::new(),
        }
    }
}

/// Controller time constant, upstream `AP_TECS::timeConstant()`.
///
/// Clamped to a floor of 0.1 s. That floor is load-bearing: the constant
/// appears in filter-alpha denominators, so a zero would produce a division by
/// zero rather than merely a fast response.
pub fn time_constant(params: &TecsParams, is_doing_auto_land: bool) -> f32 {
    let c = if is_doing_auto_land {
        params.land_time_const
    } else {
        params.time_const
    };
    if c < 0.1 {
        0.1
    } else {
        c
    }
}

impl SpeedState {
    /// A zeroed state.
    pub fn new() -> Self {
        Self::default()
    }

    /// One `_update_speed` step.
    ///
    /// `reset` corresponds to upstream's `_flags.reset`, set when too long has
    /// elapsed since the last update. On reset the filters are re-seeded rather
    /// than continued, and the function returns early — reproduced exactly,
    /// because continuing a stale filter across a gap is what the reset exists
    /// to prevent.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        params: &TecsParams,
        limits: &AirspeedLimits,
        inp: &SpeedInputs,
        eas_dem: f32,
        dt: f32,
        reset: bool,
        is_doing_auto_land: bool,
        thr_clip_status: ClipStatus,
        ste_dot_min: f32,
        ste_dot_max: f32,
    ) {
        // --- speed rate of change ---
        if reset {
            self.vdot_filter.reset();
            self.vel_dot_lpf = self.vel_dot;
        } else {
            let temp = inp.rot_mat_c_x * GRAVITY_MSS + inp.accel_x;
            self.vel_dot = self.vdot_filter.apply(temp);
            let alpha = dt / (dt + time_constant(params, is_doing_auto_land));
            self.vel_dot_lpf = self.vel_dot_lpf * (1.0 - alpha) + self.vel_dot * alpha;
        }

        // --- convert demands to true airspeed and harmonise limits ---
        let eas2tas = inp.eas2tas;
        self.tas_dem = eas_dem * eas2tas;

        if reset || !inp.should_use_airspeed {
            self.tas_max = limits.airspeed_max * eas2tas;
        } else if thr_clip_status == ClipStatus::Max {
            // wind the upper limit down, or the aircraft cannot climb at max speed
            let vel_rate_min =
                0.5 * ste_dot_min / max_f32(self.tas_state, limits.airspeed_min * eas2tas);
            self.tas_max += dt * vel_rate_min;
            self.tas_max = max_f32(self.tas_max, limits.airspeed_cruise * eas2tas);
        } else {
            // wind it back toward the parameter value
            let vel_rate_max =
                0.5 * ste_dot_max / max_f32(self.tas_state, limits.airspeed_min * eas2tas);
            self.tas_max += dt * vel_rate_max;
        }
        self.tas_max = min_f32(self.tas_max, limits.airspeed_max * eas2tas);
        self.tas_min = limits.airspeed_min * eas2tas;

        if inp.is_on_final && is_positive(limits.airspeed_stall) {
            self.tas_min = limits.airspeed_stall * eas2tas;
        }

        if limits.stall_prevention {
            // raise the floor with aerodynamic load factor
            if is_positive(limits.airspeed_stall) {
                self.tas_min = max_f32(
                    self.tas_min,
                    limits.airspeed_stall * eas2tas * safe_sqrt(inp.load_factor),
                );
            } else {
                self.tas_min *= safe_sqrt(inp.load_factor);
            }
        }

        if self.tas_max < self.tas_min {
            self.tas_max = self.tas_min;
        }

        // --- measured airspeed, or cruise when unavailable ---
        let eas = match (inp.should_use_airspeed, inp.eas) {
            (true, Some(v)) => v,
            _ => constrain_value(
                limits.airspeed_cruise,
                limits.airspeed_min,
                limits.airspeed_max,
            ),
        };

        // --- reset re-seeds and returns, rather than filtering across a gap ---
        if reset {
            self.tas_state = max_f32(eas * eas2tas, MIN_AIRSPEED);
            self.integ_dtas_state = 0.0;
            return;
        }

        // --- second order complementary filter ---
        let aspd_err = (eas * eas2tas) - self.tas_state;
        let mut integ_input = aspd_err * params.spd_comp_filt_omega * params.spd_comp_filt_omega;
        // anti-windup near the floor: only allow the integrator to push up
        if self.tas_state < 3.1 {
            integ_input = max_f32(integ_input, 0.0);
        }
        self.integ_dtas_state += integ_input * dt;
        // Upstream writes the literal 1.4142f, NOT sqrt(2) = 1.41421356...
        // Clippy suggests f32::consts::SQRT_2; that would be a MORE accurate
        // constant and therefore a behavioural divergence, because this is a
        // damping coefficient in a control filter. ADR-0003 says reproduce, so
        // the literal stays and the lint is allowed here only.
        #[allow(clippy::approx_constant)]
        let damping = 1.4142;
        let tas_input =
            self.integ_dtas_state + self.vel_dot + aspd_err * params.spd_comp_filt_omega * damping;
        self.tas_state += tas_input * dt;
        self.tas_state = max_f32(self.tas_state, MIN_AIRSPEED);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no unit tests for AP_TECS. These come from
    // reading AP_TECS.cpp:381-476 and 702-714. Real verification arrives when
    // the full update_pitch_throttle path is wired and replayed against
    // fixtures/tecs_replay.csv (ADR-0008).

    fn inputs() -> SpeedInputs {
        SpeedInputs {
            rot_mat_c_x: 0.0,
            accel_x: 0.0,
            eas2tas: 1.0,
            eas: Some(15.0),
            should_use_airspeed: true,
            is_on_final: false,
            load_factor: 1.0,
        }
    }

    /// The 0.1s floor is load-bearing: the constant is a filter-alpha
    /// denominator, so zero would divide by zero rather than respond fast.
    #[test]
    fn time_constant_floors_at_tenth_of_a_second() {
        let mut p = TecsParams::default();
        assert_eq!(time_constant(&p, false), 5.0);
        assert_eq!(time_constant(&p, true), 2.0, "landing uses land_time_const");

        p.time_const = 0.0;
        assert_eq!(time_constant(&p, false), 0.1);
        p.land_time_const = 0.05;
        assert_eq!(time_constant(&p, true), 0.1);
    }

    /// Reset seeds the estimate from the measurement and returns early rather
    /// than filtering across the gap.
    #[test]
    fn reset_seeds_from_measurement_and_returns() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        s.integ_dtas_state = 99.0;

        s.update(
            &p,
            &l,
            &inputs(),
            15.0,
            0.02,
            true,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );

        assert_eq!(s.tas_state, 15.0, "seeded directly from EAS*EAS2TAS");
        assert_eq!(s.integ_dtas_state, 0.0, "integrator cleared on reset");
    }

    /// The hard 3 m/s floor protects downstream divisions by airspeed.
    #[test]
    fn estimate_never_falls_below_min_airspeed() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        let mut inp = inputs();
        inp.eas = Some(0.0);

        s.update(
            &p,
            &l,
            &inp,
            15.0,
            0.02,
            true,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        assert_eq!(s.tas_state, MIN_AIRSPEED);

        for _ in 0..50 {
            s.update(
                &p,
                &l,
                &inp,
                15.0,
                0.02,
                false,
                false,
                ClipStatus::None,
                -5.0,
                5.0,
            );
            assert!(s.tas_state >= MIN_AIRSPEED, "got {}", s.tas_state);
        }
    }

    /// The complementary filter converges on a steady measurement.
    #[test]
    fn converges_toward_measured_airspeed() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        let mut inp = inputs();
        inp.eas = Some(18.0);

        // seed away from the measurement
        s.update(
            &p,
            &l,
            &inp,
            18.0,
            0.02,
            true,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        s.tas_state = 12.0;

        for _ in 0..400 {
            s.update(
                &p,
                &l,
                &inp,
                18.0,
                0.02,
                false,
                false,
                ClipStatus::None,
                -5.0,
                5.0,
            );
        }
        assert!(
            (s.tas_state - 18.0).abs() < 0.5,
            "should converge to 18, got {}",
            s.tas_state
        );
    }

    /// With no airspeed available the estimate falls back to cruise, clamped
    /// into the min/max band - not to zero.
    #[test]
    fn falls_back_to_cruise_without_airspeed() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        let mut inp = inputs();
        inp.should_use_airspeed = false;
        inp.eas = None;

        s.update(
            &p,
            &l,
            &inp,
            15.0,
            0.02,
            true,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        assert_eq!(s.tas_state, l.airspeed_cruise, "cruise, not zero");
    }

    /// Stall prevention scales the floor by sqrt(load factor).
    #[test]
    fn stall_prevention_raises_minimum_with_load_factor() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        let mut inp = inputs();

        inp.load_factor = 1.0;
        s.update(
            &p,
            &l,
            &inp,
            15.0,
            0.02,
            false,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        let base = s.tas_min;

        inp.load_factor = 4.0; // sqrt = 2
        s.update(
            &p,
            &l,
            &inp,
            15.0,
            0.02,
            false,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        assert!(
            (s.tas_min - base * 2.0).abs() < 1e-4,
            "expected 2x floor, got {} vs base {}",
            s.tas_min,
            base
        );
    }

    /// tas_max is never allowed below tas_min, which would invert the band.
    #[test]
    fn max_is_clamped_up_to_min() {
        let p = TecsParams::default();
        // deliberately inverted band
        let l = AirspeedLimits {
            airspeed_max: 5.0,
            airspeed_min: 20.0,
            ..Default::default()
        };
        let mut s = SpeedState::new();

        s.update(
            &p,
            &l,
            &inputs(),
            15.0,
            0.02,
            false,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        assert!(s.tas_max >= s.tas_min, "band must not invert");
    }

    /// EAS2TAS scales demand and limits together.
    #[test]
    fn eas2tas_scales_demand_and_limits() {
        let p = TecsParams::default();
        let l = AirspeedLimits::default();
        let mut s = SpeedState::new();
        let mut inp = inputs();
        inp.eas2tas = 1.2;

        s.update(
            &p,
            &l,
            &inp,
            20.0,
            0.02,
            true,
            false,
            ClipStatus::None,
            -5.0,
            5.0,
        );
        assert!((s.tas_dem - 24.0).abs() < 1e-4, "demand scaled by EAS2TAS");
        assert!((s.tas_min - l.airspeed_min * 1.2).abs() < 1e-3);
    }
}
