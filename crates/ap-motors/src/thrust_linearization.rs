//! Thrust-curve linearization and battery-voltage compensation, upstream
//! `AP_Motors/AP_Motors_Thrust_Linearization.{cpp,h}`. COP-006.
//!
//! ESCs and props do not turn PWM into thrust linearly: a motor spends a
//! larger fraction of its output range near the bottom of the throttle curve
//! producing comparatively little thrust, and the fraction near the top
//! producing comparatively more. Left alone, that non-linearity means the
//! same stick movement asks for very different amounts of *thrust* depending
//! on where the stick already is, which is exactly what an attitude
//! controller assuming a linear plant does not want. `Thrust_Linearization`
//! is the piece that hides that curve from everything above it: controllers
//! above this line reason in thrust (0 to 1, roughly proportional to how much
//! the aircraft actually accelerates), and only this class converts to and
//! from the actuator range the ESCs actually see.
//!
//! Riding along on the same class is battery-voltage compensation. A motor's
//! thrust for a given PWM sags as the pack voltage sags, so the same
//! actuator output produces less thrust on a tired battery than a fresh one.
//! `lift_max` folds a filtered estimate of that sag into the same curve, so
//! the thrust-to-actuator conversion also compensates for it — one
//! non-linearity and one voltage correction, applied together every time
//! thrust is converted to or from actuator output.
//!
//! # Scope: multirotor only
//!
//! This port targets `AP_MotorsMulticopter` exclusively (copter-rust's own
//! charter). Upstream conditionally compiles a different set of defaults
//! under `APM_BUILD_TYPE(APM_BUILD_Heli)` (no linearization, `SPIN_MIN=0`,
//! `SPIN_MAX=1`, and it calls `AP_Param::setup_object_defaults` itself rather
//! than relying on the surrounding vehicle to do so). That branch, and its
//! defaults, are deliberately not ported here. [`ThrustLinParams::default`]
//! gives the non-heli defaults only: `THST_EXPO=0.65`, `SPIN_MIN=0.15`,
//! `SPIN_MAX=0.95`, both battery-voltage bounds `0.0` (disabled).
//!
//! # Parameters are plain fields, not `AP_Param`
//!
//! Following the precedent [`crate::spool::SpoolParams`] and
//! [`crate::arming`] already set in this crate: no `AP_Param`-equivalent
//! persistence layer exists here, so the six real parameters
//! (`THST_EXPO`/`SPIN_MIN`/`SPIN_MAX`/`BAT_IDX`/`BAT_V_MAX`/`BAT_V_MIN`) are
//! plain `f32`/`i8` fields on [`ThrustLinParams`], owned and persisted by
//! whatever this port's parameter storage is above this crate.
//!
//! [`ThrustLinearization::update_lift_max_from_batt_voltage`] takes
//! `params: &mut ThrustLinParams` — not `&ThrustLinParams` — because upstream
//! really does write back to one of them: `batt_voltage_min.set(MAX(...))`
//! (`AP_Motors_Thrust_Linearization.cpp:169`) permanently raises a
//! misconfigured minimum up to `0.6 * batt_voltage_max` the first time this
//! runs, the same way [`crate::spool::Spool::update`] writes back to
//! `SpoolParams::spool_up_time`.
//!
//! # No `BATT_RAW_VOLTAGE` option
//!
//! Upstream branches on `motors.has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE)`
//! to choose between the pack's instantaneous `voltage()` and its
//! sag-removed `voltage_resting_estimate()`, and again to choose between
//! filtering the reading or resetting the filter to it outright. This port
//! has no equivalent motor-options flag, so every caller is treated as the
//! option being unset — upstream's own default — which means: always the
//! resting-estimate voltage, and always filtered. See
//! [`ThrustLinearization::update_lift_max_from_batt_voltage`] for where that
//! simplification is applied.
//!
//! # `BatteryState::voltage` is the wrong field
//!
//! [`crate::current_limit::BatteryState`] (COP-004) already carries a
//! `voltage: f32` fed by a real battery-monitor harness, and it is tempting
//! to reuse it here. Investigated directly: it is the wrong quantity.
//! `current_limit.rs`'s port of `get_current_limit_max_throttle` reads
//! `battery.voltage(batt_idx)` (`AP_MotorsMulticopter.cpp:409`) — upstream's
//! *raw, instantaneous* reading, sag included — because that function is
//! computing an ohmic margin against sag and needs the sag to be visible.
//! `update_lift_max_from_batt_voltage` needs the opposite: upstream's
//! *default* path explicitly removes sag via `voltage_resting_estimate()`
//! before it ever reaches the 0.5 Hz filter, precisely so that a hard
//! current pulse does not get read back out as a lift-capacity drop a moment
//! later. Reusing `BatteryState::voltage` here would feed raw, sagging
//! voltage into a computation upstream deliberately shields from sag —
//! doubling up the current-limiter's sag response inside the thrust curve
//! too. So `BatteryState` gained a second, distinct field,
//! `voltage_resting_estimate`, rather than reusing `voltage`.
//!
//! # Air density without an AHRS dependency
//!
//! Upstream reads `AP::ahrs().get_air_density_ratio()`, a *ratio* (density at
//! altitude over sea-level density). This crate has no dependency on AHRS —
//! nothing else in `ap-motors` needs one — so rather than adding one just for
//! this, [`ThrustLinearization::get_compensation_gain`] takes the vehicle's
//! AMSL altitude directly and computes the same ratio itself, via
//! `ap_baro::air_density_for_alt_amsl` (an absolute density, kg/m^3) divided
//! by `ap_baro::SSL_AIR_DENSITY` (the same sea-level constant
//! `ap-baro`/`ap-quadplane` already use for this exact ratio, e.g.
//! `ap-baro/src/sitl.rs:360`). `ap-baro` was added to this crate's
//! dependencies for it.

use ap_filter::lowpass::LowPassFilterFloat;
use ap_math::scalar::{constrain_value, is_positive, is_zero, safe_sqrt};

use crate::current_limit::BatteryState;

/// Battery voltage filter cutoff, upstream `AP_MOTORS_BATT_VOLT_FILT_HZ`.
const BATT_VOLT_FILT_HZ: f32 = 0.5;

/// The six real tunables, upstream `Thrust_Linearization::var_info`.
///
/// Defaults are the non-heli (`AP_MotorsMulticopter`) branch of upstream's
/// `#if APM_BUILD_TYPE(APM_BUILD_Heli)` — see the module docs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustLinParams {
    /// `THST_EXPO`: thrust curve exponent. 0 is linear, 1 is a full second
    /// order curve. Clamped to `[-1, 1]` wherever it is read, never here.
    pub curve_expo: f32,
    /// `SPIN_MIN`: actuator ratio at which thrust starts.
    pub spin_min: f32,
    /// `SPIN_MAX`: actuator ratio at which thrust saturates.
    pub spin_max: f32,
    /// `BAT_IDX`: which battery monitor instance to compensate against. Pure
    /// passthrough — nothing in this module dereferences it, the same way
    /// upstream's own `AP_MotorsMulticopter::update_lift_max_from_batt_voltage`
    /// resolves `AP::battery()` by this index *before* calling in here.
    pub batt_idx: i8,
    /// `BAT_V_MAX`: voltage above which battery scaling stops increasing.
    /// `0.0` disables voltage compensation entirely.
    pub batt_voltage_max: f32,
    /// `BAT_V_MIN`: voltage below which battery scaling stops decreasing.
    pub batt_voltage_min: f32,
}

impl Default for ThrustLinParams {
    fn default() -> Self {
        Self {
            curve_expo: 0.65,
            spin_min: 0.15,
            spin_max: 0.95,
            batt_idx: 0,
            batt_voltage_max: 0.0,
            batt_voltage_min: 0.0,
        }
    }
}

/// Thrust-curve linearization plus battery-voltage compensation. Upstream
/// `Thrust_Linearization`.
///
/// Holds only what upstream's `private:` section holds: the derived
/// `lift_max` and the voltage filter. The six real parameters live outside,
/// in [`ThrustLinParams`] — see the module docs on why.
#[derive(Debug, Clone, Copy)]
pub struct ThrustLinearization {
    /// Maximum lift ratio available from the current battery voltage.
    /// Upstream `lift_max`, default `1.0` (full lift, no compensation) both
    /// at construction and whenever voltage compensation is disabled or
    /// misconfigured.
    lift_max: f32,
    /// Filtered battery voltage, expressed as a fraction of
    /// `batt_voltage_max` (0 to 1), not as an absolute voltage. Upstream
    /// `batt_voltage_filt`.
    batt_voltage_filt: LowPassFilterFloat,
}

impl Default for ThrustLinearization {
    fn default() -> Self {
        let mut batt_voltage_filt = LowPassFilterFloat::new(BATT_VOLT_FILT_HZ);
        // Upstream's constructor: `batt_voltage_filt.reset(1.0)` — full
        // voltage assumed until the first real reading arrives, matching
        // `lift_max`'s own "no compensation yet" default.
        batt_voltage_filt.reset_to(1.0);
        Self {
            lift_max: 1.0,
            batt_voltage_filt,
        }
    }
}

impl ThrustLinearization {
    /// A fresh instance: `lift_max = 1.0`, voltage filter seeded to `1.0`
    /// (full voltage, no compensation). Upstream's constructor.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The maximum lift ratio currently available from the battery. Upstream
    /// `get_lift_max()`.
    #[must_use]
    pub fn lift_max(&self) -> f32 {
        self.lift_max
    }

    /// Applies the thrust curve and battery-voltage scaling to a desired
    /// thrust (0 to 1), returning an actuator-range throttle. Upstream
    /// `apply_thrust_curve_and_volt_scaling`.
    ///
    /// # The zero-expo special case
    ///
    /// The general quadratic-curve formula below divides by `2 * expo`, so
    /// a caller who wants a perfectly linear curve (`THST_EXPO = 0`, a
    /// legitimate and common setting, not just a boundary value) would drive
    /// it straight into a division by zero. Upstream's comment calls this out
    /// explicitly ("avoid floating point exception for small values") and
    /// short-circuits to the linear relationship the quadratic would degrade
    /// to anyway as `expo -> 0`. Ported exactly, including that this branch —
    /// unlike the general one below it — is **not** clamped to `[0, 1]`
    /// before returning; that asymmetry is upstream's, not an omission here.
    #[must_use]
    pub fn apply_thrust_curve_and_volt_scaling(
        &self,
        params: &ThrustLinParams,
        thrust: f32,
    ) -> f32 {
        let battery_scale = if is_positive(self.batt_voltage_filt.get()) {
            1.0 / self.batt_voltage_filt.get()
        } else {
            1.0
        };

        // Domain -1.0 to 1.0, range -1.0 to 1.0.
        let thrust_curve_expo = constrain_value(params.curve_expo, -1.0, 1.0);
        if is_zero(thrust_curve_expo) {
            return self.lift_max * thrust * battery_scale;
        }

        let throttle_ratio = ((thrust_curve_expo - 1.0)
            + safe_sqrt(
                (1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo)
                    + 4.0 * thrust_curve_expo * self.lift_max * thrust,
            ))
            / (2.0 * thrust_curve_expo);
        constrain_value(throttle_ratio * battery_scale, 0.0, 1.0)
    }

    /// The inverse of [`Self::apply_thrust_curve_and_volt_scaling`]: an
    /// actuator-range throttle back to a desired thrust (0 to 1). Upstream
    /// `remove_thrust_curve_and_volt_scaling`, "tested with
    /// `AP_Motors/examples/expo_inverse_test`" per its own comment — so this
    /// is transcribed exactly rather than re-derived from the forward
    /// formula, the same way upstream insists on.
    #[must_use]
    pub fn remove_thrust_curve_and_volt_scaling(
        &self,
        params: &ThrustLinParams,
        throttle: f32,
    ) -> f32 {
        let battery_scale = if is_positive(self.batt_voltage_filt.get()) {
            1.0 / self.batt_voltage_filt.get()
        } else {
            1.0
        };

        let thrust_curve_expo = constrain_value(params.curve_expo, -1.0, 1.0);
        if is_zero(thrust_curve_expo) {
            // As in the forward direction, upstream does not clamp this
            // branch to [0, 1] — only the caller (`actuator_to_thrust`) does.
            return throttle / (self.lift_max * battery_scale);
        }

        let mut thrust =
            ((throttle / battery_scale) * (2.0 * thrust_curve_expo)) - (thrust_curve_expo - 1.0);
        thrust = (thrust * thrust) - ((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo));
        thrust /= 4.0 * thrust_curve_expo * self.lift_max;
        constrain_value(thrust, 0.0, 1.0)
    }

    /// Converts desired thrust (0 to 1) to linearized actuator output (0 to
    /// 1). Upstream `thrust_to_actuator`.
    #[must_use]
    pub fn thrust_to_actuator(&self, params: &ThrustLinParams, thrust_in: f32) -> f32 {
        let thrust_in = constrain_value(thrust_in, 0.0, 1.0);
        params.spin_min
            + (params.spin_max - params.spin_min)
                * self.apply_thrust_curve_and_volt_scaling(params, thrust_in)
    }

    /// The inverse of [`Self::thrust_to_actuator`]. Upstream
    /// `actuator_to_thrust`, used to find the thrust level equivalent to a
    /// direct actuator output — upstream's own comment notes this is used in
    /// tailsitter transitions.
    #[must_use]
    pub fn actuator_to_thrust(&self, params: &ThrustLinParams, actuator: f32) -> f32 {
        let actuator = (actuator - params.spin_min) / (params.spin_max - params.spin_min);
        constrain_value(
            self.remove_thrust_curve_and_volt_scaling(params, actuator),
            0.0,
            1.0,
        )
    }

    /// Refreshes `lift_max` from the current battery voltage. Upstream
    /// `update_lift_max_from_batt_voltage`.
    ///
    /// # The `BATT_RAW_VOLTAGE` simplification
    ///
    /// Upstream reads `motors.has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE)`
    /// twice: once to choose `voltage()` over `voltage_resting_estimate()`,
    /// once to choose resetting the filter outright over actually filtering.
    /// This port has no motor-options flag yet, so both reads are treated as
    /// the option being unset — upstream's own default — unconditionally:
    /// callers always supply the resting-estimate voltage (see
    /// `battery.voltage_resting_estimate` and the module docs on why that is
    /// not `BatteryState::voltage`), and this function always filters rather
    /// than resetting.
    ///
    /// # The write-back
    ///
    /// `params.batt_voltage_min` is raised in place to at least
    /// `0.6 * batt_voltage_max` the first time this runs past the
    /// misconfiguration guard, mirroring upstream's own
    /// `batt_voltage_min.set(MAX(batt_voltage_min, batt_voltage_max * 0.6))`
    /// — a real, permanent write to the parameter, not a local clamp. Hence
    /// `&mut ThrustLinParams`, matching the precedent
    /// `crate::spool::SpoolParams::spool_up_time` already set in this crate
    /// for a parameter upstream mutates the same way.
    pub fn update_lift_max_from_batt_voltage(
        &mut self,
        params: &mut ThrustLinParams,
        battery: &BatteryState,
        dt_s: f32,
    ) {
        let batt_voltage = battery.voltage_resting_estimate;

        // Sanity check batt_voltage_min is not too small. If disabled or
        // misconfigured, exit immediately.
        if params.batt_voltage_max <= 0.0
            || params.batt_voltage_min >= params.batt_voltage_max
            || batt_voltage < 0.25 * params.batt_voltage_min
        {
            self.batt_voltage_filt.reset_to(1.0);
            self.lift_max = 1.0;
            return;
        }

        params.batt_voltage_min = params.batt_voltage_min.max(params.batt_voltage_max * 0.6);

        // Constrain the resting voltage estimate into the configured range.
        let batt_voltage = constrain_value(
            batt_voltage,
            params.batt_voltage_min,
            params.batt_voltage_max,
        );

        // Filter at 0.5 Hz. (Simplification above: always the filtered path.)
        self.batt_voltage_filt
            .apply(batt_voltage / params.batt_voltage_max, dt_s);

        let thrust_curve_expo = constrain_value(params.curve_expo, -1.0, 1.0);
        let filt = self.batt_voltage_filt.get();
        self.lift_max = filt * (1.0 - thrust_curve_expo) + thrust_curve_expo * filt * filt;
    }

    /// Gain-scheduling gain from battery voltage and air density. Upstream
    /// `get_compensation_gain`.
    ///
    /// `alt_amsl` is the vehicle's altitude above mean sea level, metres. See
    /// the module docs for why this takes an altitude rather than reading
    /// AHRS directly: `ap-motors` has no AHRS dependency, so the density
    /// ratio upstream gets from `AP::ahrs().get_air_density_ratio()` is
    /// computed here instead, from `ap-baro`.
    #[must_use]
    pub fn get_compensation_gain(&self, alt_amsl: f32) -> f32 {
        // Avoid divide by zero.
        if self.lift_max <= 0.0 {
            return 1.0;
        }

        let mut ret = 1.0 / self.lift_max;

        // Air density ratio is increasing in density / decreasing in
        // altitude.
        let air_density_ratio =
            ap_baro::air_density_for_alt_amsl(alt_amsl) / ap_baro::SSL_AIR_DENSITY;
        if air_density_ratio > 0.3 && air_density_ratio < 1.5 {
            ret *= 1.0 / constrain_value(air_density_ratio, 0.5, 1.25);
        }
        ret
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn battery(voltage_resting_estimate: f32) -> BatteryState {
        BatteryState {
            current_amps: None,
            resistance: 0.0,
            voltage: voltage_resting_estimate,
            voltage_resting_estimate,
        }
    }

    fn near(a: f32, b: f32, tol: f32) {
        assert!(
            (a - b).abs() < tol,
            "expected {b}, got {a} (diff {})",
            (a - b).abs()
        );
    }

    // --- Round-trip inverse tests -------------------------------------
    //
    // Mirrors upstream's own `AP_Motors/examples/expo_inverse_test`: sweep
    // expo and thrust and check `actuator_to_thrust(thrust_to_actuator(x))`
    // recovers `x`, and the reverse. Default lift_max=1.0/battery_scale=1.0
    // (voltage compensation untouched), like the upstream example.

    #[test]
    fn thrust_actuator_round_trip_across_expo_and_spin_range() {
        let tl = ThrustLinearization::new();
        let expos = [-1.0_f32, -0.5, 0.0, 0.3, 0.65, 1.0];
        let spins = [(0.0_f32, 1.0_f32), (0.15, 0.95), (0.1, 0.8)];
        let thrusts = [0.0_f32, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0];

        for &expo in &expos {
            for &(spin_min, spin_max) in &spins {
                let params = ThrustLinParams {
                    curve_expo: expo,
                    spin_min,
                    spin_max,
                    ..ThrustLinParams::default()
                };
                for &thrust in &thrusts {
                    let actuator = tl.thrust_to_actuator(&params, thrust);
                    let recovered = tl.actuator_to_thrust(&params, actuator);
                    near(recovered, thrust, 1.0e-4);
                }
            }
        }
    }

    /// The zero-expo linear special case, isolated: upstream's explicit
    /// `is_zero(thrust_curve_expo)` guard avoiding a divide-by-zero in the
    /// general quadratic formula. Exercised on its own so a regression here
    /// cannot hide inside the sweep above.
    #[test]
    fn zero_expo_linear_case_round_trips() {
        let tl = ThrustLinearization::new();
        let params = ThrustLinParams {
            curve_expo: 0.0,
            spin_min: 0.15,
            spin_max: 0.95,
            ..ThrustLinParams::default()
        };

        for thrust in [0.0_f32, 0.1, 0.33, 0.5, 0.7, 0.9, 1.0] {
            let actuator = tl.thrust_to_actuator(&params, thrust);
            // Linear: actuator - spin_min should be proportional to thrust.
            let recovered = tl.actuator_to_thrust(&params, actuator);
            near(recovered, thrust, 1.0e-5);
        }

        // And directly: apply/remove agree without going through the spin
        // range at all.
        let applied = tl.apply_thrust_curve_and_volt_scaling(&params, 0.42);
        let removed = tl.remove_thrust_curve_and_volt_scaling(&params, applied);
        near(removed, 0.42, 1.0e-5);
    }

    /// The reverse direction: pick actuator-range values and confirm
    /// `thrust_to_actuator(actuator_to_thrust(a)) == a`.
    #[test]
    fn actuator_thrust_round_trip_the_other_way() {
        let tl = ThrustLinearization::new();
        for &expo in &[-0.7_f32, 0.0, 0.4, 1.0] {
            let params = ThrustLinParams {
                curve_expo: expo,
                spin_min: 0.15,
                spin_max: 0.95,
                ..ThrustLinParams::default()
            };
            // Actuator values must stay within [spin_min, spin_max] to be
            // reachable from some thrust in [0, 1] in the first place.
            for &actuator in &[0.15_f32, 0.3, 0.5, 0.7, 0.95] {
                let thrust = tl.actuator_to_thrust(&params, actuator);
                let recovered = tl.thrust_to_actuator(&params, thrust);
                near(recovered, actuator, 1.0e-4);
            }
        }
    }

    /// Round-trips still hold with a non-trivial battery scale (lift_max
    /// derived from a mid-range voltage), not just the untouched default.
    #[test]
    fn round_trip_holds_under_battery_compensation() {
        let mut tl = ThrustLinearization::new();
        let mut params = ThrustLinParams {
            curve_expo: 0.65,
            spin_min: 0.15,
            spin_max: 0.95,
            batt_idx: 0,
            batt_voltage_max: 16.8,
            batt_voltage_min: 10.5,
        };
        // A single non-saturating update: pack sitting at a mid-range
        // voltage. dt large relative to the 0.5 Hz cutoff so the filter has
        // mostly converged rather than testing the filter itself here.
        tl.update_lift_max_from_batt_voltage(&mut params, &battery(14.0), 10.0);
        assert!(
            tl.lift_max() < 1.0,
            "a sub-max voltage should reduce lift_max"
        );

        for &thrust in &[0.0_f32, 0.2, 0.5, 0.8, 1.0] {
            let actuator = tl.thrust_to_actuator(&params, thrust);
            let recovered = tl.actuator_to_thrust(&params, actuator);
            near(recovered, thrust, 1.0e-4);
        }
    }

    // --- update_lift_max_from_batt_voltage misconfiguration bail-out ---

    #[test]
    fn zero_batt_voltage_max_bails_to_lift_max_one() {
        let mut tl = ThrustLinearization::new();
        // Perturb lift_max first so the reset is observable.
        let mut params = ThrustLinParams {
            batt_voltage_max: 16.8,
            batt_voltage_min: 10.5,
            ..ThrustLinParams::default()
        };
        tl.update_lift_max_from_batt_voltage(&mut params, &battery(14.0), 10.0);
        assert!(tl.lift_max() < 1.0, "setup did not perturb lift_max");

        let mut misconfigured = ThrustLinParams {
            batt_voltage_max: 0.0,
            batt_voltage_min: 10.5,
            ..ThrustLinParams::default()
        };
        tl.update_lift_max_from_batt_voltage(&mut misconfigured, &battery(14.0), 0.02);
        near(tl.lift_max(), 1.0, 1.0e-6);
    }

    #[test]
    fn min_at_or_above_max_bails_to_lift_max_one() {
        let mut tl = ThrustLinearization::new();
        let mut params = ThrustLinParams {
            batt_voltage_max: 16.8,
            batt_voltage_min: 10.5,
            ..ThrustLinParams::default()
        };
        tl.update_lift_max_from_batt_voltage(&mut params, &battery(14.0), 10.0);
        assert!(tl.lift_max() < 1.0, "setup did not perturb lift_max");

        let mut misconfigured = ThrustLinParams {
            batt_voltage_max: 12.0,
            batt_voltage_min: 12.0, // min == max
            ..ThrustLinParams::default()
        };
        tl.update_lift_max_from_batt_voltage(&mut misconfigured, &battery(14.0), 0.02);
        near(tl.lift_max(), 1.0, 1.0e-6);
    }

    #[test]
    fn implausibly_low_voltage_bails_to_lift_max_one() {
        let mut tl = ThrustLinearization::new();
        let mut params = ThrustLinParams {
            batt_voltage_max: 16.8,
            batt_voltage_min: 10.5,
            ..ThrustLinParams::default()
        };
        // 0.25 * 10.5 = 2.625: well below any plausible reading, but this one
        // is deliberately implausible (e.g. a disconnected battery reporting
        // near zero).
        tl.update_lift_max_from_batt_voltage(&mut params, &battery(1.0), 0.02);
        near(tl.lift_max(), 1.0, 1.0e-6);
    }

    /// The write-back: a too-small `batt_voltage_min` is raised in place to
    /// `0.6 * batt_voltage_max`, permanently, not just for this call.
    #[test]
    fn batt_voltage_min_is_raised_in_place_when_too_small() {
        let mut tl = ThrustLinearization::new();
        let mut params = ThrustLinParams {
            batt_voltage_max: 16.8,
            batt_voltage_min: 1.0, // far below 0.6 * 16.8 = 10.08
            ..ThrustLinParams::default()
        };
        tl.update_lift_max_from_batt_voltage(&mut params, &battery(14.0), 10.0);
        near(params.batt_voltage_min, 16.8 * 0.6, 1.0e-6);
    }

    // --- get_compensation_gain air-density gating -----------------------

    /// A ratio inside the gate `(0.3, 1.5)` but below the inner clamp's
    /// floor of `0.5` still gets scaled — by the clamped value, `0.5`, not
    /// the true ratio. Chooses an altitude, and self-checks (rather than
    /// hard-coding upstream's ISA table by hand) that it really lands in
    /// that band before relying on it.
    #[test]
    fn compensation_gain_clamps_the_ratio_when_inside_the_gate() {
        let tl = ThrustLinearization::new(); // lift_max defaults to 1.0
        let alt_amsl = 9500.0_f32;
        let ratio = ap_baro::air_density_for_alt_amsl(alt_amsl) / ap_baro::SSL_AIR_DENSITY;
        assert!(
            ratio > 0.3 && ratio < 0.5,
            "test altitude {alt_amsl} m must land inside the gate but below the clamp \
             floor, got ratio {ratio}"
        );

        let gain = tl.get_compensation_gain(alt_amsl);
        // 1/lift_max * 1/clamp(ratio, 0.5, 1.25) = 1.0 * 1/0.5 = 2.0, exactly
        // the clamped value regardless of the true ratio underneath it.
        near(gain, 2.0, 1.0e-4);
    }

    /// Sea level: density ratio is exactly 1.0, inside the (0.3, 1.5) gate,
    /// and inside the (0.5, 1.25) clamp, so it applies unscaled.
    #[test]
    fn compensation_gain_applies_density_scaling_at_sea_level() {
        let tl = ThrustLinearization::new();
        // lift_max defaults to 1.0, so gain is 1.0 / density_ratio here.
        let gain = tl.get_compensation_gain(0.0);
        near(gain, 1.0, 1.0e-3);
    }

    /// High altitude pushes the density ratio below 0.3, outside the gate:
    /// no additional density scaling is applied (gain is just 1/lift_max).
    #[test]
    fn compensation_gain_skips_density_scaling_outside_the_gate() {
        let tl = ThrustLinearization::new();
        // ~15 km is deep into the stratosphere; density ratio there is well
        // under 0.3 (ISA density ratio at 15 km is roughly 0.19).
        let gain = tl.get_compensation_gain(15_000.0);
        near(gain, 1.0 / tl.lift_max(), 1.0e-6);
    }
}
