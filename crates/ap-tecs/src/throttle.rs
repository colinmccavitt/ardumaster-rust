//! Throttle demand, ported from `AP_TECS`.
//!
//! This slice covers `constrain_throttle`, `_get_i_gain` and
//! `_update_throttle_without_airspeed`. The airspeed-driven path
//! (`_update_throttle_with_airspeed`) is the next slice.
//!
//! # The no-airspeed path is a pitch-to-throttle map
//!
//! Without an airspeed sensor there is no energy error to regulate, so TECS
//! interpolates throttle from *pitch*: nose up demands more throttle, nose down
//! less, anchored at a nominal cruise setting. The pitch it uses is a blend —
//! high-passed demand plus low-passed measurement — so the mapping tracks
//! commanded changes promptly while following the aircraft's actual trim over
//! time. Both filters are the already-ported [`LowPassFilter`].

use ap_filter::lowpass::LowPassFilterFloat;
use ap_math::scalar::{constrain_value, is_zero, radians, sq};

use crate::params::{FlightStage, TecsParams};
use crate::speed::ClipStatus;
use crate::util::{max_f32, min_f32};

/// Throttle limits, upstream `aparm` plus TECS's own computed bounds.
#[derive(Debug, Clone, Copy)]
pub struct ThrottleLimits {
    /// Cruise throttle, percent. Upstream `aparm.throttle_cruise`.
    pub throttle_cruise: f32,
    /// Maximum throttle, 0..1. Upstream `_THRmaxf`.
    pub thr_max: f32,
    /// Minimum throttle, 0..1. Upstream `_THRminf`.
    pub thr_min: f32,
    /// Maximum pitch, radians. Upstream `_PITCHmaxf`.
    pub pitch_max: f32,
    /// Minimum pitch, radians. Upstream `_PITCHminf`.
    pub pitch_min: f32,
    /// Maximum total energy rate. Upstream `_STEdot_max`.
    pub ste_dot_max: f32,
    /// Minimum total energy rate. Upstream `_STEdot_min`.
    pub ste_dot_min: f32,
}

/// Everything the no-airspeed throttle path reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct ThrottleInputs {
    /// Current pitch demand, radians. Upstream `_pitch_dem`.
    pub pitch_dem: f32,
    /// Measured pitch, radians. Upstream `_ahrs.get_pitch_rad()`.
    pub pitch_measured: f32,
    /// Cosine of bank angle. Upstream `_ahrs.cos_roll()`.
    pub cos_roll: f32,
    /// Whether an automatic landing is in progress.
    pub is_doing_auto_land: bool,
    /// Whether the vehicle is gliding, upstream `_flags.is_gliding`.
    pub is_gliding: bool,
    /// Current flight stage.
    pub flight_stage: FlightStage,
}

/// Throttle demand state.
#[derive(Debug, Clone, Copy)]
pub struct ThrottleDemand {
    /// The demand, upstream `_throttle_dem`.
    pub throttle_dem: f32,
    /// Whether the demand is clipping, upstream `_thr_clip_status`.
    pub clip_status: ClipStatus,
    /// Low-passed total-energy-rate error, upstream `_STEdotErrLast`.
    ste_dot_err_last: f32,
    /// Throttle integrator, upstream `_integTHR_state`.
    integ_thr_state: f32,
    /// Previous demand, for slew limiting. Upstream `_last_throttle_dem`.
    last_throttle_dem: f32,
    /// Low-passed pitch demand, upstream `_pitch_demand_lpf`.
    pitch_demand_lpf: LowPassFilterFloat,
    /// Low-passed measured pitch, upstream `_pitch_measured_lpf`.
    pitch_measured_lpf: LowPassFilterFloat,
}

impl Default for ThrottleDemand {
    fn default() -> Self {
        Self {
            throttle_dem: 0.0,
            clip_status: ClipStatus::None,
            ste_dot_err_last: 0.0,
            integ_thr_state: 0.0,
            last_throttle_dem: 0.0,
            pitch_demand_lpf: LowPassFilterFloat::default(),
            pitch_measured_lpf: LowPassFilterFloat::default(),
        }
    }
}

impl ThrottleDemand {
    /// A demand at rest.
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed the slew-limit history, upstream `_last_throttle_dem`.
    ///
    /// Called on reset so the first demand is slew limited from the nominal
    /// cruise setting rather than from zero.
    pub fn seed_last_demand(&mut self, value: f32) {
        self.last_throttle_dem = value;
    }

    /// Seed both pitch blending filters, upstream's `reset(ahrs pitch)`.
    pub fn seed_pitch_filters(&mut self, pitch: f32) {
        self.pitch_demand_lpf.reset_to(pitch);
        self.pitch_measured_lpf.reset_to(pitch);
    }

    /// Set the cutoff of both pitch blending filters, in Hz.
    pub fn set_pitch_filter_cutoff(&mut self, hz: f32) {
        self.pitch_demand_lpf.set_cutoff_frequency(hz);
        self.pitch_measured_lpf.set_cutoff_frequency(hz);
    }

    /// Clamp the demand into range, recording which bound it hit.
    ///
    /// Upstream `constrain_throttle()`. The clip status is not merely
    /// diagnostic: `_update_speed` winds the airspeed ceiling down when
    /// throttle is clipping at maximum, and `_update_height_demand` shrinks the
    /// climb limit. Reporting the wrong bound would change both.
    pub fn constrain(&mut self, limits: &ThrottleLimits) {
        if self.throttle_dem > limits.thr_max {
            self.clip_status = ClipStatus::Max;
            self.throttle_dem = limits.thr_max;
        } else if self.throttle_dem < limits.thr_min {
            self.clip_status = ClipStatus::Min;
            self.throttle_dem = limits.thr_min;
        } else {
            self.clip_status = ClipStatus::None;
        }
    }

    /// One `_update_throttle_without_airspeed` step.
    pub fn update_without_airspeed(
        &mut self,
        params: &TecsParams,
        limits: &ThrottleLimits,
        inp: &ThrottleInputs,
        throttle_nudge: i16,
        pitch_trim_deg: f32,
        dt: f32,
    ) {
        // synthetic airspeed may have been used previously; start clean
        self.clip_status = ClipStatus::None;

        // nominal throttle: the landing parameter wins only when set
        let nom_thr = if inp.is_doing_auto_land && params.land_throttle >= 0.0 {
            (params.land_throttle + throttle_nudge as f32) * 0.01
        } else {
            (limits.throttle_cruise + throttle_nudge as f32) * 0.01
        };

        // Blend high-passed demand with low-passed measurement, so the mapping
        // responds to commanded changes at once while tracking actual trim
        // slowly. pitch_trim_deg is removed from the measured side only.
        self.pitch_demand_lpf.apply(inp.pitch_dem, dt);
        let pitch_demand_hpf = inp.pitch_dem - self.pitch_demand_lpf.get();
        self.pitch_measured_lpf.apply(inp.pitch_measured, dt);
        let pitch_corrected_lpf = self.pitch_measured_lpf.get() - radians(pitch_trim_deg);
        let pitch_blended = pitch_demand_hpf + pitch_corrected_lpf;

        // Interpolate toward the corresponding throttle limit. Each branch is
        // guarded on the limit's sign, so a zero or wrong-signed limit falls
        // through to nominal rather than dividing by zero.
        self.throttle_dem = if pitch_blended > 0.0 && limits.pitch_max > 0.0 {
            nom_thr + (limits.thr_max - nom_thr) * pitch_blended / limits.pitch_max
        } else if pitch_blended < 0.0 && limits.pitch_min < 0.0 {
            nom_thr + (limits.thr_min - nom_thr) * pitch_blended / limits.pitch_min
        } else {
            nom_thr
        };

        if inp.is_gliding {
            // gliding cuts throttle outright and skips turn compensation
            self.throttle_dem = 0.0;
            return;
        }

        // Turn drag compensation: induced drag rises with bank, scaling as
        // 1/cos²(bank) - 1. cos² is constrained to [0.1, 1] so a knife-edge
        // attitude cannot divide by zero.
        let ste_dot_dem =
            params.roll_comp * (1.0 / constrain_value(sq(inp.cos_roll), 0.1, 1.0) - 1.0);
        self.throttle_dem += ste_dot_dem / (limits.ste_dot_max - limits.ste_dot_min)
            * (limits.thr_max - limits.thr_min);

        self.constrain(limits);
    }
}

/// Integrator gain for the current flight stage, upstream `_get_i_gain()`.
///
/// Takeoff and landing-abort use the takeoff gain unconditionally. Landing uses
/// the landing gain **only if it is non-zero** — zero is the "unset" sentinel
/// meaning fall back to the cruise gain, not "no integrator".
pub fn get_i_gain(params: &TecsParams, flight_stage: FlightStage, is_doing_auto_land: bool) -> f32 {
    match flight_stage {
        FlightStage::Takeoff | FlightStage::AbortLanding => params.integ_gain_takeoff,
        _ if is_doing_auto_land && !is_zero(params.integ_gain_land) => params.integ_gain_land,
        _ => params.integ_gain,
    }
}

/// Everything the airspeed-driven throttle path reads from outside TECS.
#[derive(Debug, Clone, Copy)]
pub struct AirspeedThrottleInputs {
    /// Demanded specific potential energy, upstream `_SPE_dem`.
    pub spe_dem: f32,
    /// Estimated specific potential energy, upstream `_SPE_est`.
    pub spe_est: f32,
    /// Demanded specific kinetic energy, upstream `_SKE_dem`.
    pub ske_dem: f32,
    /// Estimated specific kinetic energy, upstream `_SKE_est`.
    pub ske_est: f32,
    /// Potential energy rate, upstream `_SPEdot`.
    pub spedot: f32,
    /// Kinetic energy rate, upstream `_SKEdot`.
    pub skedot: f32,
    /// Demanded kinetic energy rate, upstream `_SKEdot_dem`.
    pub skedot_dem: f32,
    /// Minimum true airspeed, upstream `_TASmin`.
    pub tas_min: f32,
    /// Maximum true airspeed, upstream `_TASmax`.
    pub tas_max: f32,
    /// Cosine of bank angle, upstream `_ahrs.cos_roll()`.
    pub cos_roll: f32,
    /// Whether underspeed is latched, upstream `_flags.underspeed`.
    pub underspeed: bool,
    /// Whether gliding, upstream `_flags.is_gliding`.
    pub is_gliding: bool,
    /// Whether an automatic landing is in progress.
    pub is_doing_auto_land: bool,
    /// Whether on approach, upstream `_landing.is_on_approach()`.
    pub is_on_approach: bool,
    /// Landing throttle slew rate, percent per second; 0 or less means unset.
    pub land_throttle_slewrate: i8,
    /// Airframe throttle slew rate, upstream `aparm.throttle_slewrate`.
    pub throttle_slewrate: i8,
    /// Whether takeoff airspeed has been reached, upstream
    /// `_flags.reached_speed_takeoff`.
    pub reached_speed_takeoff: bool,
    /// Current flight stage.
    pub flight_stage: FlightStage,
}

impl ThrottleLimits {
    /// Clamp the throttle limits and enforce a minimum usable range.
    ///
    /// Upstream `_update_throttle_limits()`. Returns whether the range had to
    /// be forced open (upstream `_flag_throttle_forced`).
    ///
    /// **This is a load-bearing invariant, not tidying.** The airspeed throttle
    /// path divides by `(thr_max - thr_min)` with no local guard; upstream's own
    /// comment says the 1% floor exists "primarily to prevent TECS numerical
    /// errors". Porting the calculation without this would silently drop the
    /// protection, so the two are kept together.
    pub fn enforce_minimum_range(&mut self) -> bool {
        self.thr_max = min_f32(1.0, self.thr_max);
        self.thr_min = max_f32(-1.0, self.thr_min);

        const THR_EPS: f32 = 0.01;
        if (self.thr_min - self.thr_max).abs() < THR_EPS {
            if self.thr_max < 1.0 {
                self.thr_max = max_f32(self.thr_max, self.thr_min + 0.01);
            } else {
                self.thr_min = min_f32(self.thr_min, self.thr_max - 0.01);
            }
            true
        } else {
            false
        }
    }
}

impl ThrottleDemand {
    /// The throttle integrator state, upstream `_integTHR_state`.
    pub fn integ_thr_state(&self) -> f32 {
        self.integ_thr_state
    }

    /// One `_update_throttle_with_airspeed` step.
    ///
    /// Assumes [`ThrottleLimits::enforce_minimum_range`] has been applied, which
    /// is what makes the gain divisions below safe.
    #[allow(clippy::too_many_arguments)]
    pub fn update_with_airspeed(
        &mut self,
        params: &TecsParams,
        limits: &ThrottleLimits,
        inp: &AirspeedThrottleInputs,
        time_constant: f32,
        i_gain: f32,
        dt: f32,
    ) {
        // Bound the potential energy error so a large height error cannot drive
        // the aircraft past its speed limits chasing altitude.
        let mut spe_err_max = max_f32(inp.ske_est - 0.5 * inp.tas_min * inp.tas_min, 0.0);
        let mut spe_err_min = min_f32(inp.ske_est - 0.5 * inp.tas_max * inp.tas_max, 0.0);

        if inp.flight_stage == FlightStage::Vtol {
            // vertical motors corrupt the total-energy picture, so potential
            // energy error is ignored entirely
            spe_err_max = 0.0;
            spe_err_min = 0.0;
        }

        let spedot_dem = (inp.spe_dem - inp.spe_est) / time_constant;

        let ste_error = constrain_value(inp.spe_dem - inp.spe_est, spe_err_min, spe_err_max)
            + inp.ske_dem
            - inp.ske_est;
        let mut stedot_dem = constrain_value(
            spedot_dem + inp.skedot_dem,
            limits.ste_dot_min,
            limits.ste_dot_max,
        );
        let mut stedot_error = stedot_dem - inp.spedot - inp.skedot;

        // 0.5 s first order filter, removing accelerometer noise. The
        // coefficient is 2*dt, which gives a 0.5 s time constant.
        let filt_coef = 2.0 * dt;
        stedot_error = filt_coef * stedot_error + (1.0 - filt_coef) * self.ste_dot_err_last;
        self.ste_dot_err_last = stedot_error;

        if inp.underspeed {
            // underspeed overrides everything: full throttle
            self.throttle_dem = 1.0;
        } else if inp.is_gliding {
            self.throttle_dem = 0.0;
        } else {
            // Derivative of energy rate with respect to throttle, measured
            // across the usable throttle range. Safe because
            // enforce_minimum_range guarantees at least 1% of range.
            let k_thr2ste =
                (limits.ste_dot_max - limits.ste_dot_min) / (limits.thr_max - limits.thr_min);
            let k_ste2thr = 1.0 / (time_constant * k_thr2ste);

            let nom_thr = limits.throttle_cruise * 0.01;
            // turn drag compensation, as in the no-airspeed path
            stedot_dem +=
                params.roll_comp * (1.0 / constrain_value(sq(inp.cos_roll), 0.1, 1.0) - 1.0);
            let ff_throttle = nom_thr + stedot_dem / k_thr2ste;

            // landing damping wins only when set; zero is the unset sentinel
            let throttle_damp = if inp.is_doing_auto_land && !is_zero(params.land_throttle_damp) {
                params.land_throttle_damp
            } else {
                params.thr_damp
            };

            self.throttle_dem =
                (ste_error + stedot_error * throttle_damp) * k_ste2thr + ff_throttle;

            let thr_min_clipped_to_zero = constrain_value(limits.thr_min, 0.0, limits.thr_max);

            // Integrator limits allow 10% saturation for demand noise, and the
            // amplitude clamp makes the integrator leave its limits faster.
            let max_amp = 0.5 * (limits.thr_max - thr_min_clipped_to_zero);
            let integ_max =
                constrain_value(limits.thr_max - self.throttle_dem + 0.1, -max_amp, max_amp);
            let integ_min =
                constrain_value(limits.thr_min - self.throttle_dem - 0.1, -max_amp, max_amp);

            self.integ_thr_state += (ste_error * i_gain) * dt * k_ste2thr;

            if matches!(
                inp.flight_stage,
                FlightStage::Takeoff | FlightStage::AbortLanding
            ) {
                if !inp.reached_speed_takeoff {
                    // hold full throttle until the takeoff airspeed is reached.
                    // Note the integrator is deliberately NOT constrained here.
                    self.throttle_dem =
                        max_f32(self.throttle_dem, limits.thr_max - self.integ_thr_state);
                }
            } else {
                self.integ_thr_state = constrain_value(self.integ_thr_state, integ_min, integ_max);
            }

            // slew limiting, with the landing rate overriding when set
            let mut slewrate = inp.throttle_slewrate;
            if inp.is_on_approach && inp.land_throttle_slewrate > 0 {
                slewrate = inp.land_throttle_slewrate;
            }
            if slewrate != 0 {
                let incr = dt * (limits.thr_max - thr_min_clipped_to_zero) * slewrate as f32 * 0.01;
                self.throttle_dem = constrain_value(
                    self.throttle_dem,
                    self.last_throttle_dem - incr,
                    self.last_throttle_dem + incr,
                );
                self.last_throttle_dem = self.throttle_dem;
            }

            // the integrator is added AFTER slew limiting, so it is not itself
            // rate limited
            self.throttle_dem += self.integ_thr_state;
        }

        self.constrain(limits);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:882-960.

    fn limits() -> ThrottleLimits {
        ThrottleLimits {
            throttle_cruise: 45.0,
            thr_max: 1.0,
            thr_min: 0.0,
            pitch_max: 0.35,
            pitch_min: -0.35,
            ste_dot_max: 5.0,
            ste_dot_min: -5.0,
        }
    }

    fn inputs() -> ThrottleInputs {
        ThrottleInputs {
            pitch_dem: 0.0,
            pitch_measured: 0.0,
            cos_roll: 1.0,
            is_doing_auto_land: false,
            is_gliding: false,
            flight_stage: FlightStage::Normal,
        }
    }

    /// The clip status drives airspeed-ceiling and climb-limit adaptation
    /// elsewhere, so reporting the right bound matters beyond diagnostics.
    #[test]
    fn constrain_records_which_bound_was_hit() {
        let l = limits();
        let mut t = ThrottleDemand::new();

        t.throttle_dem = 1.5;
        t.constrain(&l);
        assert_eq!(t.throttle_dem, 1.0);
        assert_eq!(t.clip_status, ClipStatus::Max);

        t.throttle_dem = -0.5;
        t.constrain(&l);
        assert_eq!(t.throttle_dem, 0.0);
        assert_eq!(t.clip_status, ClipStatus::Min);

        t.throttle_dem = 0.5;
        t.constrain(&l);
        assert_eq!(t.throttle_dem, 0.5);
        assert_eq!(t.clip_status, ClipStatus::None);
    }

    /// Takeoff and landing-abort take the takeoff gain unconditionally.
    #[test]
    fn takeoff_and_abort_use_the_takeoff_gain() {
        let p = TecsParams {
            integ_gain_takeoff: 0.15,
            ..Default::default()
        };
        assert_eq!(get_i_gain(&p, FlightStage::Takeoff, false), 0.15);
        assert_eq!(get_i_gain(&p, FlightStage::AbortLanding, false), 0.15);
        // even mid-landing, takeoff stage wins
        assert_eq!(get_i_gain(&p, FlightStage::Takeoff, true), 0.15);
    }

    /// Zero is the UNSET sentinel for the landing gain, meaning fall back to
    /// the cruise gain - not "no integrator".
    #[test]
    fn zero_landing_gain_falls_back_to_cruise_gain() {
        let p = TecsParams::default(); // integ_gain 0.3, integ_gain_land 0.0
        assert_eq!(
            get_i_gain(&p, FlightStage::Normal, true),
            0.3,
            "zero land gain must fall back, not disable the integrator"
        );

        let p2 = TecsParams {
            integ_gain_land: 0.1,
            ..Default::default()
        };
        assert_eq!(get_i_gain(&p2, FlightStage::Normal, true), 0.1);
    }

    #[test]
    fn cruise_uses_the_plain_integrator_gain() {
        let p = TecsParams::default();
        assert_eq!(get_i_gain(&p, FlightStage::Normal, false), 0.3);
    }

    /// Level pitch maps to the nominal cruise throttle.
    #[test]
    fn level_pitch_gives_nominal_throttle() {
        let p = TecsParams::default();
        let l = limits();
        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(1.0);

        t.update_without_airspeed(&p, &l, &inputs(), 0, 0.0, 0.02);
        assert!(
            (t.throttle_dem - 0.45).abs() < 1e-5,
            "expected cruise 0.45, got {}",
            t.throttle_dem
        );
    }

    /// Nose up interpolates toward maximum throttle, nose down toward minimum.
    ///
    /// Driven through MEASURED pitch, not demand. The demand term is
    /// high-passed, so a constant demand converges to its own low-pass and
    /// contributes nothing in steady state; the standing mapping comes from
    /// measured pitch. Filters are allowed to settle first.
    #[test]
    fn pitch_maps_monotonically_to_throttle() {
        let p = TecsParams::default();
        let l = limits();

        let settle = |pitch_measured: f32| {
            let mut t = ThrottleDemand::new();
            t.set_pitch_filter_cutoff(5.0);
            let mut i = inputs();
            i.pitch_measured = pitch_measured;
            for _ in 0..400 {
                t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);
            }
            t.throttle_dem
        };

        let up = settle(0.3);
        let level = settle(0.0);
        let down = settle(-0.3);

        assert!(up > level, "nose up {up} should exceed level {level}");
        assert!(
            down < level,
            "nose down {down} should be under level {level}"
        );
        assert!(
            (level - 0.45).abs() < 1e-3,
            "level should be cruise, got {level}"
        );
    }

    /// The demand term is HIGH-PASSED: it responds to a step change and then
    /// decays back, rather than holding a standing offset.
    #[test]
    fn demand_contributes_only_a_transient() {
        let p = TecsParams::default();
        let l = limits();
        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(5.0);
        let mut i = inputs();

        // settle at level
        for _ in 0..200 {
            t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);
        }
        let settled = t.throttle_dem;

        // step the demand up: the high-pass responds immediately
        i.pitch_dem = 0.3;
        t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);
        let transient = t.throttle_dem;
        assert!(
            transient > settled,
            "step should raise throttle: {transient} vs {settled}"
        );

        // hold it: the high-pass decays and throttle returns toward settled
        for _ in 0..400 {
            t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);
        }
        assert!(
            t.throttle_dem < transient,
            "held demand should decay: {} vs {}",
            t.throttle_dem,
            transient
        );
        assert!(
            (t.throttle_dem - settled).abs() < 1e-2,
            "should return near settled, got {} vs {}",
            t.throttle_dem,
            settled
        );
    }

    /// A zero or wrong-signed pitch limit falls through to nominal rather than
    /// dividing by zero.
    #[test]
    fn zero_pitch_limits_fall_through_to_nominal() {
        let p = TecsParams::default();
        let l = ThrottleLimits {
            pitch_max: 0.0,
            pitch_min: 0.0,
            ..limits()
        };
        let mut i = inputs();
        i.pitch_dem = 0.3;

        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(1.0);
        t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);

        assert!(t.throttle_dem.is_finite(), "must not divide by zero");
        assert!(
            (t.throttle_dem - 0.45).abs() < 1e-5,
            "falls through to nominal"
        );
    }

    /// Gliding cuts throttle outright and skips turn compensation entirely.
    #[test]
    fn gliding_cuts_throttle_and_skips_turn_compensation() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = inputs();
        i.is_gliding = true;
        i.cos_roll = 0.5; // steep bank that would otherwise add throttle
        i.pitch_dem = 0.3;

        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(1.0);
        t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);

        assert_eq!(t.throttle_dem, 0.0, "gliding means no throttle");
        assert_eq!(
            t.clip_status,
            ClipStatus::None,
            "early return leaves clip status untouched"
        );
    }

    /// Banking adds throttle to offset induced drag.
    #[test]
    fn banking_adds_turn_compensation() {
        let p = TecsParams::default(); // roll_comp 10
        let l = limits();

        let mut level = ThrottleDemand::new();
        level.set_pitch_filter_cutoff(1.0);
        level.update_without_airspeed(&p, &l, &inputs(), 0, 0.0, 0.02);

        let mut banked = ThrottleDemand::new();
        banked.set_pitch_filter_cutoff(1.0);
        let mut i = inputs();
        i.cos_roll = 0.7; // ~45 degrees
        banked.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);

        assert!(
            banked.throttle_dem > level.throttle_dem,
            "bank {} should exceed level {}",
            banked.throttle_dem,
            level.throttle_dem
        );
    }

    /// cos²(bank) is clamped to a 0.1 floor, so a knife-edge attitude cannot
    /// divide by zero.
    #[test]
    fn knife_edge_bank_does_not_divide_by_zero() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = inputs();
        i.cos_roll = 0.0; // 90 degrees of bank

        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(1.0);
        t.update_without_airspeed(&p, &l, &i, 0, 0.0, 0.02);

        assert!(t.throttle_dem.is_finite(), "got {}", t.throttle_dem);
        assert!((0.0..=1.0).contains(&t.throttle_dem));
    }

    /// The landing throttle parameter is used only while landing AND only when
    /// set to a non-negative value.
    #[test]
    fn landing_throttle_parameter_applies_only_when_set() {
        let l = limits();
        let mut i = inputs();
        i.is_doing_auto_land = true;

        // default land_throttle is -1 (unset): cruise is used
        let p_unset = TecsParams::default();
        let mut a = ThrottleDemand::new();
        a.set_pitch_filter_cutoff(1.0);
        a.update_without_airspeed(&p_unset, &l, &i, 0, 0.0, 0.02);
        assert!(
            (a.throttle_dem - 0.45).abs() < 1e-5,
            "unset falls back to cruise"
        );

        // set: the landing value is used instead
        let p_set = TecsParams {
            land_throttle: 20.0,
            ..Default::default()
        };
        let mut b = ThrottleDemand::new();
        b.set_pitch_filter_cutoff(1.0);
        b.update_without_airspeed(&p_set, &l, &i, 0, 0.0, 0.02);
        assert!(
            (b.throttle_dem - 0.20).abs() < 1e-5,
            "got {}",
            b.throttle_dem
        );
    }

    /// throttle_nudge shifts the nominal setting, in percent.
    #[test]
    fn throttle_nudge_shifts_the_nominal_setting() {
        let p = TecsParams::default();
        let l = limits();
        let mut t = ThrottleDemand::new();
        t.set_pitch_filter_cutoff(1.0);

        t.update_without_airspeed(&p, &l, &inputs(), 10, 0.0, 0.02);
        assert!(
            (t.throttle_dem - 0.55).abs() < 1e-5,
            "cruise 45 + nudge 10 = 55%, got {}",
            t.throttle_dem
        );
    }

    fn airspeed_inputs() -> AirspeedThrottleInputs {
        AirspeedThrottleInputs {
            spe_dem: 100.0 * 9.80665,
            spe_est: 100.0 * 9.80665,
            ske_dem: 0.5 * 20.0 * 20.0,
            ske_est: 0.5 * 20.0 * 20.0,
            spedot: 0.0,
            skedot: 0.0,
            skedot_dem: 0.0,
            tas_min: 9.0,
            tas_max: 22.0,
            cos_roll: 1.0,
            underspeed: false,
            is_gliding: false,
            is_doing_auto_land: false,
            is_on_approach: false,
            land_throttle_slewrate: 0,
            throttle_slewrate: 0,
            reached_speed_takeoff: true,
            flight_stage: FlightStage::Normal,
        }
    }

    /// The 1% floor is what makes the gain divisions in the airspeed path safe.
    /// Upstream calls it out as preventing TECS numerical errors.
    #[test]
    fn throttle_limits_enforce_a_minimum_range() {
        let mut l = ThrottleLimits {
            thr_max: 0.5,
            thr_min: 0.5,
            ..limits()
        };
        assert!(
            l.enforce_minimum_range(),
            "equal limits must be forced open"
        );
        assert!(
            (l.thr_max - l.thr_min).abs() >= 0.01 - 1e-6,
            "range {} too small",
            l.thr_max - l.thr_min
        );

        // at the top of the range it opens downward instead
        let mut top = ThrottleLimits {
            thr_max: 1.0,
            thr_min: 1.0,
            ..limits()
        };
        assert!(top.enforce_minimum_range());
        assert!(top.thr_min <= 0.99 + 1e-6, "got {}", top.thr_min);
        assert_eq!(top.thr_max, 1.0, "must not exceed 1.0");

        // a healthy range is left alone
        let mut ok = limits();
        assert!(!ok.enforce_minimum_range());
        assert_eq!(ok.thr_max, 1.0);
        assert_eq!(ok.thr_min, 0.0);
    }

    /// Underspeed overrides everything with full throttle.
    #[test]
    fn underspeed_commands_full_throttle() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.underspeed = true;
        i.spe_dem = 0.0; // would otherwise demand very little throttle

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
        assert_eq!(t.throttle_dem, 1.0);
    }

    /// Gliding wins over the energy calculation, but not over underspeed.
    #[test]
    fn gliding_commands_zero_throttle() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.is_gliding = true;

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
        assert_eq!(t.throttle_dem, 0.0);

        // underspeed is checked first, so it takes precedence
        i.underspeed = true;
        let mut u = ThrottleDemand::new();
        u.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
        assert_eq!(u.throttle_dem, 1.0, "underspeed outranks gliding");
    }

    /// A height deficit demands more throttle than a height surplus.
    #[test]
    fn height_error_drives_throttle_monotonically() {
        let p = TecsParams::default();
        let l = limits();

        let run = |spe_dem: f32| {
            let mut i = airspeed_inputs();
            i.spe_dem = spe_dem;
            let mut t = ThrottleDemand::new();
            t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
            t.throttle_dem
        };

        let level = run(100.0 * 9.80665);
        let below = run(120.0 * 9.80665); // demand above current height
        let above = run(80.0 * 9.80665);

        assert!(
            below > level,
            "climbing {below} should exceed level {level}"
        );
        assert!(
            above < level,
            "descending {above} should be under level {level}"
        );
    }

    /// Potential energy error is bounded so chasing altitude cannot push the
    /// aircraft past its speed limits.
    #[test]
    fn potential_energy_error_is_bounded_by_speed_limits() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        // an enormous height demand
        i.spe_dem = 10_000.0 * 9.80665;

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
        assert!(t.throttle_dem.is_finite());
        assert!((0.0..=1.0).contains(&t.throttle_dem));
        assert_eq!(
            t.clip_status,
            ClipStatus::Max,
            "should saturate, not diverge"
        );
    }

    /// VTOL zeroes the potential energy error bounds, because vertical motors
    /// corrupt the total-energy picture.
    #[test]
    fn vtol_ignores_potential_energy_error() {
        let p = TecsParams::default();
        let l = limits();

        let run = |stage: FlightStage| {
            let mut i = airspeed_inputs();
            i.flight_stage = stage;
            i.spe_dem = 200.0 * 9.80665; // large height error
            let mut t = ThrottleDemand::new();
            t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
            t.throttle_dem
        };

        assert!(
            run(FlightStage::Vtol) < run(FlightStage::Normal),
            "VTOL should ignore the height error"
        );
    }

    /// Slew limiting bounds how fast the demand can move per step.
    #[test]
    fn slew_rate_limits_demand_movement() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.throttle_slewrate = 10; // 10% per second
                                  // modest error: enough to demand more throttle, not enough to collapse
                                  // the integrator limits
        i.spe_dem = 101.0 * 9.80665;

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);
        // 0.02s at 10%/s over a 1.0 range is a 0.002 step from zero
        assert!(
            t.throttle_dem <= 0.002 + 1e-6,
            "slew limited step, got {}",
            t.throttle_dem
        );
    }

    /// The landing slew rate overrides the airframe one, but only on approach
    /// and only when positive.
    ///
    /// Uses a MODEST height error on purpose. A large one drives the demand so
    /// far past the throttle range that the integrator limits collapse to
    /// -maxAmp (see `integrator_limits_collapse_on_excessive_demand`), which
    /// forces both cases to zero and hides the slew difference.
    #[test]
    fn landing_slew_rate_applies_only_on_approach() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.throttle_slewrate = 10;
        i.land_throttle_slewrate = 50;
        i.spe_dem = 101.0 * 9.80665; // 1 m above current height

        // not on approach: airframe rate applies
        let mut a = ThrottleDemand::new();
        a.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);

        // on approach: the faster landing rate applies
        i.is_on_approach = true;
        let mut b = ThrottleDemand::new();
        b.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);

        assert!(
            b.throttle_dem > a.throttle_dem,
            "approach rate {} should exceed airframe rate {}",
            b.throttle_dem,
            a.throttle_dem
        );
    }

    /// During takeoff before reaching speed, throttle is held at maximum and
    /// the integrator is deliberately left unconstrained.
    #[test]
    fn takeoff_before_speed_holds_full_throttle() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.flight_stage = FlightStage::Takeoff;
        i.reached_speed_takeoff = false;
        i.spe_dem = 0.0; // would otherwise demand almost nothing

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.3, 0.02);
        assert!(
            t.throttle_dem > 0.9,
            "should hold near full throttle, got {}",
            t.throttle_dem
        );
    }

    /// The energy-rate error is low-passed with a 0.5 s constant, so a step in
    /// error does not appear in the demand all at once.
    #[test]
    fn energy_rate_error_is_filtered() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.spedot = -5.0; // sudden sink

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);
        let first = t.ste_dot_err_last;

        for _ in 0..100 {
            t.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);
        }
        assert!(
            t.ste_dot_err_last.abs() > first.abs(),
            "filter should build up: {} then {}",
            first,
            t.ste_dot_err_last
        );
    }

    /// Anti-windup: when the demand runs far past the usable throttle range,
    /// BOTH integrator bounds clamp to -maxAmp, forcing the integrator negative
    /// to unwind the excess.
    ///
    /// This is why an unrealistically large height error produces zero throttle
    /// rather than saturated throttle, and it cost one wrong test premise to
    /// discover.
    #[test]
    fn integrator_limits_collapse_on_excessive_demand() {
        let p = TecsParams::default();
        let l = limits();
        let mut i = airspeed_inputs();
        i.spe_dem = 500.0 * 9.80665; // absurd height error

        let mut t = ThrottleDemand::new();
        t.update_with_airspeed(&p, &l, &i, 5.0, 0.0, 0.02);

        // integrator driven to -maxAmp = -0.5 * (thr_max - 0) despite zero gain
        assert!(
            t.integ_thr_state() < 0.0,
            "integrator should unwind, got {}",
            t.integ_thr_state()
        );
        assert!((t.integ_thr_state() + 0.5).abs() < 1e-5, "expected -maxAmp");
        assert!(t.throttle_dem.is_finite());
        assert!((0.0..=1.0).contains(&t.throttle_dem));
    }
}
