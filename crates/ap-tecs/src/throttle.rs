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
}
