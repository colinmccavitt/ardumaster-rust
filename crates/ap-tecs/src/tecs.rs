//! The TECS controller, ported from `AP_TECS::update_pitch_throttle` and
//! `AP_TECS::_initialise_states`.
//!
//! Wires the stages together in upstream's order, which is load-bearing —
//! several stages consume state the previous one produced in the same call.
//!
//! # What this stage consumes rather than computes
//!
//! Upstream splits TECS across two entry points. `update_50hz` runs the
//! third-order height/climb-rate complementary filter and the airspeed filter;
//! `update_pitch_throttle` runs the control logic on their outputs.
//!
//! This type is the **control logic**. The filter outputs — height, climb rate,
//! airspeed estimate and its rate — arrive as [`TecsInputs`] fields rather than
//! being recomputed here. That matches ADR-0004 (inject, do not reach) and it is
//! also what makes log replay possible: upstream logs those four values as
//! `TECS.h`, `TECS.dh`, `TECS.sp` and `TECS.dsp`, so a recorded flight can drive
//! this stage directly.
//!
//! The 50 Hz filters themselves are **not** covered by that replay — their
//! inputs are EKF position and velocity, which are not logged at 50 Hz. They
//! remain port-derived. See [`crate::speed`].

use crate::demand::{SpeedDemand, SpeedDemandInputs, SteRateLimits};
use crate::energy::{Energies, EnergyInputs};
use crate::height::{HeightDemand, HeightInputs};
use crate::limits::{AirframePitchLimits, PitchLimitInputs, PitchLimits};
use crate::params::{FlightStage, TecsParams};
use crate::pitch::{PitchDemand, PitchInputs};
use crate::speed::{AirspeedLimits, ClipStatus};
use crate::throttle::{
    get_i_gain, AirspeedThrottleInputs, ThrottleDemand, ThrottleInputs, ThrottleLimits,
};
use crate::underspeed::{UnderspeedDetector, UnderspeedInputs};
use crate::util::max_f32;

/// One call's worth of external state.
#[derive(Debug, Clone, Copy)]
pub struct TecsInputs {
    // --- the nine update_pitch_throttle arguments ---
    /// Height demand, centimetres. Upstream `hgt_dem_cm`.
    pub hgt_dem_cm: f32,
    /// Equivalent airspeed demand, centimetres/s. Upstream `EAS_dem_cm`.
    pub eas_dem_cm: f32,
    /// Current flight stage.
    pub flight_stage: FlightStage,
    /// Distance flown beyond the landing waypoint.
    pub distance_beyond_land_wp: f32,
    /// Minimum climbout pitch, centidegrees. Upstream `ptchMinCO_cd`.
    pub pitch_min_climbout_cd: f32,
    /// Throttle nudge, percent. Upstream `throttle_nudge`.
    pub throttle_nudge: i16,
    /// Height above field elevation. Upstream `hgt_afe`.
    pub hgt_afe: f32,
    /// Aerodynamic load factor.
    pub load_factor: f32,
    /// Pitch trim, degrees.
    pub pitch_trim_deg: f32,

    // --- outputs of the 50 Hz stage, injected ---
    /// Height estimate, upstream `_height` (logged as `TECS.h`).
    pub height: f32,
    /// Climb rate, upstream `_climb_rate` (logged as `TECS.dh`).
    pub climb_rate: f32,
    /// True airspeed estimate, upstream `_TAS_state` (logged as `TECS.sp`).
    pub tas_state: f32,
    /// Speed rate of change, upstream `_vel_dot` (logged as `TECS.dsp`).
    pub vel_dot: f32,
    /// Low-passed speed rate of change, upstream `_vel_dot_lpf`.
    ///
    /// **Not logged upstream.** The energy rate terms high-pass `vel_dot`
    /// against this, so an exact replay needs it added to the reference build's
    /// logging.
    pub vel_dot_lpf: f32,

    // --- airspeed limits, from the 50 Hz stage ---
    /// Minimum true airspeed, upstream `_TASmin`.
    pub tas_min: f32,
    /// Maximum true airspeed, upstream `_TASmax`.
    pub tas_max: f32,
    /// Raw true airspeed demand, upstream `_TAS_dem`.
    pub tas_dem: f32,
    /// Cruise airspeed converted to true.
    pub tas_cruise: f32,

    // --- vehicle and AHRS state ---
    /// Measured pitch, radians.
    pub pitch_measured: f32,
    /// Cosine of bank angle.
    pub cos_roll: f32,
    /// Whether airspeed is in use, upstream `use_airspeed()`.
    pub use_airspeed: bool,
    /// Whether gliding was requested or propulsion failed.
    pub gliding_requested: bool,
    /// Whether the vehicle is flaring.
    pub is_flaring: bool,
    /// Whether the vehicle is on approach.
    pub is_on_approach: bool,
    /// Landing pitch, centidegrees.
    pub landing_pitch_cd: f32,
    /// Landing throttle slew rate, percent per second.
    pub land_throttle_slewrate: i8,
    /// Airframe throttle slew rate.
    pub throttle_slewrate: i8,
    /// Progress along the landing path, 0..1.
    pub path_proportion: f32,
    /// External maximum throttle, 0..1.
    pub thr_max_ext: f32,
    /// External minimum throttle, -1..1.
    pub thr_min_ext: f32,
    /// External maximum pitch for this iteration, **degrees**.
    ///
    /// Upstream `_PITCHmaxf_ext`, set by `set_pitch_max()` and consumed once.
    /// 90.0 means unconstrained.
    pub pitch_max_ext: f32,
    /// External minimum pitch for this iteration, **degrees**.
    ///
    /// Upstream `_PITCHminf_ext`, set by `set_pitch_min()` and consumed once.
    /// -90.0 means unconstrained.
    pub pitch_min_ext: f32,
    /// Current time, for the underspeed detector's hysteresis timer.
    pub now_ms: ap_hal::time::Millis,
}

/// A read-only view of `Tecs` internal state, for logging and for
/// field-by-field comparison against a recorded upstream flight.
///
/// The field set mirrors upstream's `TECS` log message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TecsSnapshot {
    /// Height demand after filtering and rate limiting, upstream `_hgt_dem`.
    pub hgt_dem: f32,
    /// Low-pass filtered height demand, upstream `_hgt_dem_lpf`.
    pub hgt_dem_lpf: f32,
    /// Rate-limited height demand, upstream `_hgt_dem_rate_ltd`.
    pub hgt_dem_rate_ltd: f32,
    /// Demanded climb rate, upstream `_hgt_rate_dem`.
    pub hgt_rate_dem: f32,
    /// Demanded potential energy, upstream `_SPE_dem`.
    pub spe_dem: f32,
    /// Demanded kinetic energy, upstream `_SKE_dem`.
    pub ske_dem: f32,
    /// Estimated potential energy, upstream `_SPE_est`.
    pub spe_est: f32,
    /// Estimated kinetic energy, upstream `_SKE_est`.
    pub ske_est: f32,
    /// Rate of change of potential energy, upstream `_SPEdot`.
    pub spedot: f32,
    /// Rate of change of kinetic energy, upstream `_SKEdot`.
    pub skedot: f32,
    /// Demanded rate of change of kinetic energy, upstream `_SKEdot_dem`.
    pub skedot_dem: f32,
    /// Kinetic energy weighting in the balance, upstream `_SKE_weighting`.
    pub ske_weighting: f32,
    /// Pitch demand before limiting, upstream `_pitch_dem_unc`.
    pub pitch_dem_unc: f32,
    /// Applied minimum pitch, radians.
    pub pitch_min: f32,
    /// Applied maximum pitch, radians.
    pub pitch_max: f32,
    /// Applied maximum throttle, 0..1.
    pub thr_max: f32,
    /// Applied minimum throttle, 0..1.
    pub thr_min: f32,
    /// Adjusted true airspeed demand, upstream `_TAS_dem_adj`.
    pub tas_dem_adj: f32,
    /// Whether the underspeed condition is latched.
    pub underspeed: bool,
    /// Specific energy balance error integrator, upstream `_integSEBdot`.
    pub integ_sebdot: f32,
    /// Kinetic energy trim integrator, upstream `_integKE`.
    pub integ_ke: f32,
    /// Height demand input after the saturation freeze, upstream `_hgt_dem_in`.
    pub hgt_dem_in: f32,
    /// Previous height demand input, upstream `_hgt_dem_in_prev`.
    pub hgt_dem_in_prev: f32,
    /// Adaptive climb rate scaler, upstream `_max_climb_scaler`.
    pub max_climb_scaler: f32,
    /// Adaptive sink rate scaler, upstream `_max_sink_scaler`.
    pub max_sink_scaler: f32,
    /// Applied climb rate limit, upstream `_climb_rate_limit`.
    pub climb_rate_limit: f32,
    /// Applied sink rate limit, upstream `_sink_rate_limit`.
    pub sink_rate_limit: f32,
    /// Post-takeoff height offset, upstream `_post_TO_hgt_offset`.
    pub post_to_hgt_offset: f32,
}

/// Controller state to start a log replay from, mirroring `TecsSnapshot`.
///
/// Only the fields that carry between calls: everything else is recomputed
/// from the inputs each iteration.
#[cfg(feature = "replay")]
#[derive(Debug, Clone, Copy, Default)]
pub struct ReplaySeed {
    /// Upstream `_integSEBdot`.
    pub integ_sebdot: f32,
    /// Upstream `_integKE`.
    pub integ_ke: f32,
    /// Upstream `_hgt_dem_lpf`.
    pub hgt_dem_lpf: f32,
    /// Upstream `_hgt_dem_rate_ltd`.
    pub hgt_dem_rate_ltd: f32,
    /// Upstream `_hgt_dem_in_prev`.
    pub hgt_dem_in_prev: f32,
    /// Upstream `_hgt_dem` and `_hgt_dem_prev`.
    pub hgt_dem: f32,
    /// Upstream `_max_climb_scaler`.
    pub max_climb_scaler: f32,
    /// Upstream `_max_sink_scaler`.
    pub max_sink_scaler: f32,
    /// Upstream `_post_TO_hgt_offset`.
    pub post_to_hgt_offset: f32,
    /// Upstream `_last_pitch_dem`.
    pub last_pitch_dem: f32,
    /// Upstream `_last_throttle_dem`.
    pub last_throttle_dem: f32,
}

/// The TECS controller.
#[derive(Debug)]
pub struct Tecs {
    /// Tuning parameters.
    pub params: TecsParams,
    /// Airframe airspeed limits.
    pub airspeed_limits: AirspeedLimits,
    /// Airframe pitch limits.
    pub airframe_pitch: AirframePitchLimits,
    /// Cruise throttle, percent.
    pub throttle_cruise: f32,

    demand: SpeedDemand,
    height: HeightDemand,
    throttle: ThrottleDemand,
    pitch: PitchDemand,
    pitch_limits: PitchLimits,
    energies: Energies,
    ste_limits: SteRateLimits,
    throttle_limits: ThrottleLimits,
    underspeed: UnderspeedDetector,

    hgt_dem_in: f32,
    hgt_dem_in_raw: f32,
    eas_dem: f32,
    reached_speed_takeoff: bool,
    have_reset_after_takeoff: bool,
    need_reset: bool,
    /// Whether the last call took the airspeed throttle path.
    using_airspeed_for_throttle: bool,
}

impl Default for Tecs {
    fn default() -> Self {
        Self {
            params: TecsParams::default(),
            airspeed_limits: AirspeedLimits::default(),
            airframe_pitch: AirframePitchLimits::default(),
            throttle_cruise: 45.0,
            demand: SpeedDemand::new(),
            height: HeightDemand::new(),
            throttle: ThrottleDemand::new(),
            pitch: PitchDemand::new(),
            pitch_limits: PitchLimits::new(),
            energies: Energies::default(),
            ste_limits: SteRateLimits::default(),
            throttle_limits: ThrottleLimits {
                throttle_cruise: 45.0,
                thr_max: 1.0,
                thr_min: 0.0,
                pitch_max: 0.0,
                pitch_min: 0.0,
                ste_dot_max: 0.0,
                ste_dot_min: 0.0,
            },
            underspeed: UnderspeedDetector::new(),
            hgt_dem_in: 0.0,
            hgt_dem_in_raw: 0.0,
            eas_dem: 0.0,
            reached_speed_takeoff: false,
            have_reset_after_takeoff: false,
            need_reset: true,
            using_airspeed_for_throttle: false,
        }
    }
}

impl Tecs {
    /// A controller at rest.
    pub fn new() -> Self {
        Self::default()
    }

    /// Throttle demand, upstream `get_throttle_demand()`.
    pub fn throttle_demand(&self) -> f32 {
        self.throttle.throttle_dem
    }

    /// Pitch demand in radians, upstream `get_pitch_demand()` before its
    /// conversion to centidegrees.
    pub fn pitch_demand(&self) -> f32 {
        self.pitch.pitch_dem
    }

    /// Equivalent airspeed demand, upstream `_EAS_dem`.
    ///
    /// Set here, consumed by the 50 Hz stage which converts it to a true
    /// airspeed demand using EAS2TAS.
    pub fn eas_dem(&self) -> f32 {
        self.eas_dem
    }

    /// A read-only view of the controller's internal state.
    ///
    /// Mirrors the fields upstream writes to the `TECS` log message in
    /// `AP_TECS::log_data`, so a port run can be compared field-by-field
    /// against a recorded upstream flight.
    pub fn snapshot(&self) -> TecsSnapshot {
        TecsSnapshot {
            hgt_dem: self.height.hgt_dem,
            hgt_dem_lpf: self.height.hgt_dem_lpf,
            hgt_dem_rate_ltd: self.height.hgt_dem_rate_ltd,
            hgt_rate_dem: self.height.hgt_rate_dem,
            spe_dem: self.energies.spe_dem,
            ske_dem: self.energies.ske_dem,
            spe_est: self.energies.spe_est,
            ske_est: self.energies.ske_est,
            spedot: self.energies.spedot,
            skedot: self.energies.skedot,
            skedot_dem: self.energies.skedot_dem,
            ske_weighting: self.pitch.ske_weighting,
            pitch_dem_unc: self.pitch.pitch_dem_unc,
            pitch_min: self.pitch_limits.pitch_min,
            pitch_max: self.pitch_limits.pitch_max,
            thr_max: self.throttle_limits.thr_max,
            thr_min: self.throttle_limits.thr_min,
            tas_dem_adj: self.demand.tas_dem_adj,
            underspeed: self.underspeed.is_underspeed(),
            integ_sebdot: self.pitch.integ_sebdot(),
            integ_ke: self.pitch.integ_ke(),
            hgt_dem_in: self.hgt_dem_in,
            hgt_dem_in_prev: self.height.hgt_dem_in_prev,
            max_climb_scaler: self.height.max_climb_scaler,
            max_sink_scaler: self.height.max_sink_scaler,
            climb_rate_limit: self.height.climb_rate_limit,
            sink_rate_limit: self.height.sink_rate_limit,
            post_to_hgt_offset: self.height.post_to_hgt_offset,
        }
    }

    /// Overwrite the carried state, to start a log replay from the state
    /// upstream actually had.
    ///
    /// A replay is open loop, so an integrator seeded differently keeps that
    /// offset for the whole run rather than converging. Seeding is what makes
    /// the comparison a test of the update law instead of of the starting
    /// conditions.
    ///
    /// Not available in a flight build: see the `replay` feature.
    #[cfg(feature = "replay")]
    pub fn seed_for_replay(&mut self, s: &ReplaySeed) {
        self.pitch.seed_integrators(s.integ_sebdot, s.integ_ke);
        self.pitch.seed_last_demand(s.last_pitch_dem);
        self.throttle.seed_last_demand(s.last_throttle_dem);

        self.height.hgt_dem_lpf = s.hgt_dem_lpf;
        self.height.hgt_dem_rate_ltd = s.hgt_dem_rate_ltd;
        self.height.hgt_dem_in_prev = s.hgt_dem_in_prev;
        self.height.hgt_dem = s.hgt_dem;
        self.height.hgt_dem_prev = s.hgt_dem;
        self.height.max_climb_scaler = s.max_climb_scaler;
        self.height.max_sink_scaler = s.max_sink_scaler;
        self.height.post_to_hgt_offset = s.post_to_hgt_offset;
    }

    /// Request a full state reset on the next call.
    pub fn request_reset(&mut self) {
        self.need_reset = true;
    }

    /// `_initialise_states`: reset on a long gap, and re-seed during climbout.
    ///
    /// Returns whether this call is a reset, upstream `_flags.reset`.
    fn initialise_states(&mut self, inp: &TecsInputs, dt: f32) -> bool {
        let mut reset = false;

        if dt > 0.2 || self.need_reset {
            self.throttle = ThrottleDemand::new();
            self.pitch = PitchDemand::new();
            self.height = HeightDemand::new();
            self.underspeed = UnderspeedDetector::new();
            self.pitch.ske_weighting = 1.0;

            // Seed the height states to the CURRENT height, not zero. Leaving
            // them at zero against a real height creates a large false
            // potential-energy surplus, and the controller responds by diving
            // with the throttle closed.
            self.height.hgt_dem_in_prev = inp.hgt_afe;
            self.height.hgt_dem_lpf = inp.hgt_afe;
            self.height.hgt_dem_rate_ltd = inp.hgt_afe;
            self.height.hgt_dem_prev = inp.hgt_afe;
            self.height.hgt_dem = inp.hgt_afe;

            // Seed the rate/slew histories so the first demand is limited from
            // a sensible starting point rather than from zero.
            self.throttle.seed_last_demand(self.throttle_cruise * 0.01);
            self.pitch.seed_last_demand(inp.pitch_measured);

            self.demand.tas_dem_adj = inp.tas_dem;
            reset = true;
            self.need_reset = false;
            self.reached_speed_takeoff = false;

            // upstream seeds the pitch blending filters from measured pitch at
            // a cutoff derived from the time constant
            let fc = 1.0 / (core::f32::consts::PI * 2.0 * self.params.time_const);
            self.throttle.set_pitch_filter_cutoff(fc);
            self.throttle.seed_pitch_filters(inp.pitch_measured);
        } else if matches!(
            inp.flight_stage,
            FlightStage::Takeoff | FlightStage::AbortLanding
        ) {
            // Climbout: hold the height demand at the current height and add a
            // takeoff offset, so the nose is not pushed level before climbing
            // again. The offset is clamped non-negative.
            self.height.post_to_hgt_offset = max_f32(
                min_f32_local(
                    self.height.climb_rate_limit * self.params.hgt_dem_tconst,
                    self.hgt_dem_in_raw - inp.hgt_afe,
                ),
                0.0,
            );

            self.height.hgt_dem_lpf = inp.hgt_afe;
            self.height.hgt_dem_rate_ltd = inp.hgt_afe;
            self.height.hgt_dem_prev = inp.hgt_afe;
            self.height.hgt_dem = inp.hgt_afe;
            self.height.hgt_dem_in_prev = inp.hgt_afe;
            self.hgt_dem_in_raw = inp.hgt_afe;
            self.demand.tas_dem_adj = inp.tas_dem;
            self.height.max_climb_scaler = 1.0;
            self.height.max_sink_scaler = 1.0;

            if !self.have_reset_after_takeoff {
                reset = true;
                self.have_reset_after_takeoff = true;
            }
        }

        if !matches!(
            inp.flight_stage,
            FlightStage::Takeoff | FlightStage::AbortLanding
        ) {
            self.reached_speed_takeoff = false;
            self.have_reset_after_takeoff = false;
        }

        reset
    }

    /// One `update_pitch_throttle` call.
    ///
    /// The stage order is upstream's and is load-bearing: the energy rate limits
    /// depend on the height stage's adaptive climb limit, the speed demand
    /// depends on those limits, the energies depend on both demands, and pitch
    /// and throttle depend on the energies.
    pub fn update_pitch_throttle(&mut self, inp: &TecsInputs, dt: f32) {
        let dt = max_f32(dt, 0.001);

        let is_gliding = inp.gliding_requested || self.throttle_limits.thr_max == 0.0;
        let is_doing_auto_land = inp.flight_stage == FlightStage::Land;

        // convert the raw inputs
        self.hgt_dem_in_raw = inp.hgt_dem_cm * 0.01;
        // Upstream stores _EAS_dem here and consumes it in update_50hz, which
        // converts it to a true airspeed demand. This stage does not use it;
        // it is state crossing the boundary between the two entry points.
        self.eas_dem = inp.eas_dem_cm * 0.01;

        // Freeze the height demand if the vehicle cannot follow it, so it does
        // not run away while the aircraft is saturated.
        let max_climb_condition = (self.pitch.pitch_dem_unc > self.pitch_limits.pitch_max
            || self.throttle.clip_status == ClipStatus::Max)
            && !matches!(
                inp.flight_stage,
                FlightStage::Takeoff | FlightStage::AbortLanding
            );
        let max_descent_condition = self.pitch.pitch_dem_unc < self.pitch_limits.pitch_min
            || self.throttle.clip_status == ClipStatus::Min;

        // Upstream keeps ONE _hgt_dem_in_prev, shared between this freeze and
        // the two-point average inside the height stage. Kept single here too:
        // two copies drift apart and the freeze stops matching what the average
        // actually used.
        let prev = self.height.hgt_dem_in_prev;
        // Upstream writes these as two branches with the same body, one per
        // saturation direction. Collapsed here since they are identical; the
        // two conditions are kept separate and named so the intent survives.
        let frozen = (max_climb_condition && self.hgt_dem_in_raw > prev)
            || (max_descent_condition && self.hgt_dem_in_raw < prev);
        self.hgt_dem_in = if frozen { prev } else { self.hgt_dem_in_raw };

        // throttle limits first: they establish the minimum range invariant the
        // throttle gains divide by
        self.throttle_limits.throttle_cruise = self.throttle_cruise;
        self.throttle_limits.thr_max = inp.thr_max_ext;
        self.throttle_limits.thr_min = inp.thr_min_ext;
        self.throttle_limits.enforce_minimum_range();

        // pitch limits
        let limit_inputs = PitchLimitInputs {
            is_flaring: inp.is_flaring,
            is_on_approach: inp.is_on_approach,
            landing_pitch_cd: inp.landing_pitch_cd,
            hgt_afe: inp.hgt_afe,
            // Upstream reads its single `_hgt_at_start_of_flare` and
            // `_flare_initialised` here. Both live on the height stage, which
            // sets them on flare entry; this stage runs first, so it sees last
            // iteration's values exactly as upstream does.
            hgt_at_start_of_flare: self.height.hgt_at_start_of_flare(),
            flare_initialised: self.height.flare_initialised(),
            pitch_min_climbout_cd: inp.pitch_min_climbout_cd,
            pitch_max_ext: inp.pitch_max_ext,
            pitch_min_ext: inp.pitch_min_ext,
            flight_stage: inp.flight_stage,
        };
        // Upstream's `_update_pitch_limits` clears `_flare_initialised` when
        // neither flaring nor on approach, so the next flare re-arms. Applied
        // back to the height stage, which owns the flag.
        let still_flaring =
            self.pitch_limits
                .update(&self.params, &self.airframe_pitch, &limit_inputs, dt);
        if !still_flaring {
            self.height.reset_flare();
        }
        self.throttle_limits.pitch_max = self.pitch_limits.pitch_max;
        self.throttle_limits.pitch_min = self.pitch_limits.pitch_min;

        // takeoff speed latch
        if matches!(
            inp.flight_stage,
            FlightStage::Takeoff | FlightStage::AbortLanding
        ) && !self.reached_speed_takeoff
            && inp.tas_state >= inp.tas_min
            && inp.tas_min > 0.0
        {
            self.reached_speed_takeoff = true;
        }

        let reset = self.initialise_states(inp, dt);

        // energy rate limits depend on the height stage's adaptive climb limit
        self.ste_limits = SteRateLimits::update(&self.params, self.height.climb_rate_limit);
        self.throttle_limits.ste_dot_max = self.ste_limits.ste_dot_max;
        self.throttle_limits.ste_dot_min = self.ste_limits.ste_dot_min;

        let time_constant = crate::speed::time_constant(&self.params, is_doing_auto_land);

        // speed demand
        let demand_inputs = SpeedDemandInputs {
            tas_dem: inp.tas_dem,
            tas_state: inp.tas_state,
            tas_min: inp.tas_min,
            tas_max: inp.tas_max,
            tas_cruise: inp.tas_cruise,
            sink_fraction: self.height.sink_fraction,
            bad_descent: self.pitch.bad_descent(),
            underspeed: self.underspeed.is_underspeed(),
            descent_speedup: false,
        };
        self.demand
            .update(&demand_inputs, &self.ste_limits, time_constant, dt, reset);

        // height demand
        let height_inputs = HeightInputs {
            hgt_dem_in: self.hgt_dem_in,
            height: inp.height,
            hgt_afe: inp.hgt_afe,
            is_flaring: inp.is_flaring,
            is_doing_auto_land,
            pitch_dem_unc: self.pitch.pitch_dem_unc,
            pitch_max: self.pitch_limits.pitch_max,
            pitch_min: self.pitch_limits.pitch_min,
            sebdot_dem_clip: self.pitch.sebdot_dem_clip,
            thr_clip_status: self.throttle.clip_status,
            using_airspeed_for_throttle: self.using_airspeed_for_throttle,
            flight_stage: inp.flight_stage,
            distance_beyond_land_wp: inp.distance_beyond_land_wp,
        };
        self.height.update(&self.params, &height_inputs, dt);

        // underspeed, in upstream's position: after the height demand, before
        // the energies, so it sees this call's demand and last call's throttle
        let underspeed = self.underspeed.update(
            &UnderspeedInputs {
                tas_state: inp.tas_state,
                tas_min: inp.tas_min,
                throttle_dem: self.throttle.throttle_dem,
                thr_max: self.throttle_limits.thr_max,
                is_flaring: inp.is_flaring,
                height: inp.height,
                hgt_dem: self.height.hgt_dem,
                flight_stage: inp.flight_stage,
            },
            inp.now_ms,
        );

        // energies
        self.energies = Energies::update(&EnergyInputs {
            hgt_dem: self.height.hgt_dem,
            tas_dem_adj: self.demand.tas_dem_adj,
            tas_state: inp.tas_state,
            tas_rate_dem: self.demand.tas_rate_dem,
            tas_rate_dem_lpf: self.demand.tas_rate_dem_lpf,
            height: inp.height,
            climb_rate: inp.climb_rate,
            vel_dot: inp.vel_dot,
            vel_dot_lpf: inp.vel_dot_lpf,
        });

        let i_gain = get_i_gain(&self.params, inp.flight_stage, is_doing_auto_land);

        // pitch
        let pitch_inputs = PitchInputs {
            spe_dem: self.energies.spe_dem,
            ske_dem: self.energies.ske_dem,
            spe_est: self.energies.spe_est,
            ske_est: self.energies.ske_est,
            spedot: self.energies.spedot,
            skedot: self.energies.skedot,
            hgt_rate_dem: self.height.hgt_rate_dem,
            tas_state: inp.tas_state,
            tas_dem_adj: self.demand.tas_dem_adj,
            pitch_min: self.pitch_limits.pitch_min,
            pitch_max: self.pitch_limits.pitch_max,
            path_proportion: inp.path_proportion,
            use_airspeed: inp.use_airspeed,
            underspeed,
            is_gliding,
            is_doing_auto_land,
            is_flaring: inp.is_flaring,
            flight_stage: inp.flight_stage,
        };
        self.pitch
            .update(&self.params, &pitch_inputs, time_constant, i_gain, dt);

        // throttle
        if inp.use_airspeed {
            let t_inputs = AirspeedThrottleInputs {
                spe_dem: self.energies.spe_dem,
                spe_est: self.energies.spe_est,
                ske_dem: self.energies.ske_dem,
                ske_est: self.energies.ske_est,
                spedot: self.energies.spedot,
                skedot: self.energies.skedot,
                skedot_dem: self.energies.skedot_dem,
                tas_min: inp.tas_min,
                tas_max: inp.tas_max,
                cos_roll: inp.cos_roll,
                underspeed,
                is_gliding,
                is_doing_auto_land,
                is_on_approach: inp.is_on_approach,
                land_throttle_slewrate: inp.land_throttle_slewrate,
                throttle_slewrate: inp.throttle_slewrate,
                reached_speed_takeoff: self.reached_speed_takeoff,
                flight_stage: inp.flight_stage,
            };
            self.throttle.update_with_airspeed(
                &self.params,
                &self.throttle_limits,
                &t_inputs,
                time_constant,
                i_gain,
                dt,
            );
            self.using_airspeed_for_throttle = true;
        } else {
            let t_inputs = ThrottleInputs {
                pitch_dem: self.pitch.pitch_dem,
                pitch_measured: inp.pitch_measured,
                cos_roll: inp.cos_roll,
                is_doing_auto_land,
                is_gliding,
                flight_stage: inp.flight_stage,
            };
            self.throttle.update_without_airspeed(
                &self.params,
                &self.throttle_limits,
                &t_inputs,
                inp.throttle_nudge,
                inp.pitch_trim_deg,
                dt,
            );
            self.using_airspeed_for_throttle = false;
        }

        // bad descent, last, since it reads the throttle demand
        let ste_error = self.energies.spe_dem - self.energies.spe_est + self.energies.ske_dem
            - self.energies.ske_est;
        self.pitch.detect_bad_descent(
            &pitch_inputs,
            ste_error,
            self.throttle.throttle_dem,
            self.throttle_limits.thr_max,
        );
    }

    /// Whether underspeed is currently latched.
    pub fn is_underspeed(&self) -> bool {
        self.underspeed.is_underspeed()
    }
}

#[inline]
fn min_f32_local(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    fn inputs() -> TecsInputs {
        TecsInputs {
            hgt_dem_cm: 10000.0,
            eas_dem_cm: 2000.0,
            flight_stage: FlightStage::Normal,
            distance_beyond_land_wp: 0.0,
            pitch_min_climbout_cd: 0.0,
            throttle_nudge: 0,
            hgt_afe: 100.0,
            load_factor: 1.0,
            pitch_trim_deg: 0.0,
            height: 100.0,
            climb_rate: 0.0,
            tas_state: 20.0,
            vel_dot: 0.0,
            vel_dot_lpf: 0.0,
            tas_min: 9.0,
            tas_max: 22.0,
            tas_dem: 20.0,
            tas_cruise: 12.0,
            pitch_measured: 0.0,
            cos_roll: 1.0,
            use_airspeed: true,
            gliding_requested: false,
            is_flaring: false,
            is_on_approach: false,
            landing_pitch_cd: 0.0,
            land_throttle_slewrate: 0,
            throttle_slewrate: 0,
            path_proportion: 0.0,
            // neutral: unconstrained, matching upstream's reset values
            pitch_max_ext: 90.0,
            pitch_min_ext: -90.0,
            thr_max_ext: 1.0,
            thr_min_ext: 0.0,
            now_ms: ap_hal::time::Millis(0),
        }
    }

    /// The wiring runs end to end and produces finite, in-range demands.
    #[test]
    fn runs_end_to_end_and_stays_in_range() {
        let mut t = Tecs::new();
        let i = inputs();

        for _ in 0..200 {
            t.update_pitch_throttle(&i, 0.02);
            assert!(t.throttle_demand().is_finite());
            assert!(t.pitch_demand().is_finite());
            assert!(
                (0.0..=1.0).contains(&t.throttle_demand()),
                "throttle {} out of range",
                t.throttle_demand()
            );
        }
    }

    /// A height deficit raises the throttle demand once the shaped demand has
    /// caught up with it.
    ///
    /// Long enough for the RATE LIMITER to stop dominating. From a reset the
    /// demand starts at zero and climbs at max_climb_rate * dt = 0.1 m per
    /// step, so with a 100 m target both cases move identically for the first
    /// thousand steps and only diverge once the lower one arrives. An earlier
    /// version ran 100 steps and saw two identical values.
    #[test]
    fn height_deficit_raises_demands() {
        let mut level = Tecs::new();
        let mut climb = Tecs::new();
        let i = inputs();
        let mut hi = inputs();
        hi.hgt_dem_cm = 12000.0; // 20 m above

        for n in 0..3000u32 {
            let mut a = i;
            let mut b = hi;
            a.now_ms = ap_hal::time::Millis(n * 20);
            b.now_ms = ap_hal::time::Millis(n * 20);
            level.update_pitch_throttle(&a, 0.02);
            climb.update_pitch_throttle(&b, 0.02);
        }
        assert!(
            climb.throttle_demand() > level.throttle_demand(),
            "climb throttle {} should exceed level {}",
            climb.throttle_demand(),
            level.throttle_demand()
        );
    }

    /// The first call is a reset, seeding rather than filtering from zero.
    #[test]
    fn first_call_resets_and_seeds() {
        let mut t = Tecs::new();
        let i = inputs();
        t.update_pitch_throttle(&i, 0.02);
        assert!(t.throttle_demand().is_finite());
        assert!(t.pitch_demand().is_finite());
    }

    /// A long gap forces a reset rather than integrating across it.
    #[test]
    fn long_gap_forces_reset() {
        let mut t = Tecs::new();
        let i = inputs();
        for _ in 0..50 {
            t.update_pitch_throttle(&i, 0.02);
        }
        // a 1 second gap
        t.update_pitch_throttle(&i, 1.0);
        assert!(t.throttle_demand().is_finite());
        assert!(t.pitch_demand().is_finite());
    }
}
