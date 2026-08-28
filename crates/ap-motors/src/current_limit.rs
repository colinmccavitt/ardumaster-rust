//! Battery current limiting, upstream
//! `AP_MotorsMulticopter::get_current_limit_max_throttle`. COP-004.
//!
//! A multirotor can ask its battery for more current than the pack can deliver
//! without sagging below the voltage the ESCs need. This is the governor that
//! stops it: it watches measured current against a permissible current and
//! integrates a ceiling on how far above hover the throttle may go.
//!
//! # It is an integrator, not a filter
//!
//! The obvious reading of
//!
//! ```text
//! limit += (dt / (dt + tc)) * (1 - ratio)
//! ```
//!
//! is a first-order lag toward `1 - ratio`, because that is the shape of one.
//! It is not. The target of a lag would be a throttle limit; here the term
//! added is a *current* ratio, so the expression accumulates. Below the
//! permissible current the ceiling rises every iteration; above it, it falls.
//! `tc` sets the step size, not a time constant to settle on.
//!
//! That distinction matters for the port: a lag written as `limit += (target -
//! limit) * k` would look equivalent, settle at `1 - ratio`, and be wrong
//! everywhere except the instant the two happen to cross.

use crate::throttle::HoverThrottle;

/// The tunables, upstream's `MOT_BAT_*` parameters.
#[derive(Debug, Clone, Copy)]
pub struct CurrentLimitParams {
    /// `MOT_BAT_CURR_MAX`: the current above which throttle is limited. Zero
    /// or negative disables limiting entirely.
    pub batt_current_max: f32,
    /// `MOT_BAT_CURR_TC`: scales the integrator's step.
    pub batt_current_time_constant: f32,
    /// `MOT_BAT_VOLT_MIN`: the pack voltage the limiter tries not to sag below.
    pub battery_min_voltage: f32,
}

/// What the battery monitor reports this iteration.
#[derive(Debug, Clone, Copy)]
pub struct BatteryState {
    /// Measured current, or `None` when the pack has no current telemetry.
    /// Upstream spells the absence as `current_amps()` returning false.
    pub current_amps: Option<f32>,
    /// Estimated pack internal resistance. Zero means unknown, and unknown
    /// means no limiting — a resistance of zero would make the ohmic headroom
    /// infinite rather than merely large.
    pub resistance: f32,
    /// Pack voltage — upstream's raw, instantaneous `AP_BattMonitor::voltage()`,
    /// sag included. This is the field `get_current_limit_max_throttle` reads
    /// (`AP_MotorsMulticopter.cpp:409`): it is computing an ohmic margin
    /// against sag, so it needs the sag to be visible.
    ///
    /// Distinct from `voltage_resting_estimate` below — see COP-006's
    /// `thrust_linearization` module docs for why the two must not be
    /// conflated.
    pub voltage: f32,
    /// Upstream's sag-removed `AP_BattMonitor::voltage_resting_estimate()`
    /// — actual voltage with sag backed out based on current draw and
    /// estimated pack resistance. Upstream's `Thrust_Linearization::
    /// update_lift_max_from_batt_voltage` reads this by default (the
    /// non-`BATT_RAW_VOLTAGE` path), precisely so a hard current pulse is
    /// not read back out moments later as a drop in lift capacity.
    pub voltage_resting_estimate: f32,
}

/// The lowest the ceiling may fall to.
///
/// A fifth of the range above hover. The limiter is allowed to take authority
/// away from the pilot, but never all of it — an aircraft that cannot climb at
/// all is not safer than one drawing too much current.
const THROTTLE_LIMIT_MIN: f32 = 0.2;

/// The current limiter's state.
#[derive(Debug, Clone, Copy)]
pub struct CurrentLimit {
    throttle_limit: f32,
}

impl Default for CurrentLimit {
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentLimit {
    /// Starts unlimited, which is where every early exit also puts it.
    pub fn new() -> Self {
        Self {
            throttle_limit: 1.0,
        }
    }

    /// The stored headroom scaler, upstream `_throttle_limit`.
    pub fn throttle_limit(&self) -> f32 {
        self.throttle_limit
    }

    /// The maximum throttle current limiting allows, upstream
    /// `get_current_limit_max_throttle`.
    ///
    /// Returns 1.0 — and resets the stored limit to 1.0 — when limiting is
    /// disabled, when disarmed, when the pack reports no current, or when its
    /// resistance is unknown. Each of those is a case where the limiter has
    /// nothing to act on, and leaving a part-wound-down limit behind would
    /// apply it again on the next arm.
    pub fn update(
        &mut self,
        armed: bool,
        dt_s: f32,
        params: &CurrentLimitParams,
        battery: &BatteryState,
        hover: &HoverThrottle,
    ) -> f32 {
        // Order matters, and it is upstream's. The `||` chain short-circuits on
        // the parameter and the arming state before it asks the battery for a
        // current, so a pack with no telemetry is never consulted on a vehicle
        // that has limiting switched off anyway.
        if params.batt_current_max <= 0.0 || !armed {
            self.throttle_limit = 1.0;
            return 1.0;
        }
        let Some(batt_current) = battery.current_amps else {
            self.throttle_limit = 1.0;
            return 1.0;
        };
        // Upstream tests this with is_zero, so a resistance that is merely
        // tiny still divides — and gives an enormous ohmic headroom, which the
        // MIN against the parameter maximum then discards.
        if ap_math::scalar::is_zero(battery.resistance) {
            self.throttle_limit = 1.0;
            return 1.0;
        }

        // The permissible current: whichever is smaller of what the operator
        // allowed and what the pack can give before sagging past its minimum.
        let batt_current_max = params.batt_current_max.min(
            batt_current + (battery.voltage - params.battery_min_voltage) / battery.resistance,
        );

        let batt_current_ratio = batt_current / batt_current_max;

        // Accumulate, do not lag. See the module docs.
        self.throttle_limit +=
            (dt_s / (dt_s + params.batt_current_time_constant)) * (1.0 - batt_current_ratio);
        self.throttle_limit = self.throttle_limit.clamp(THROTTLE_LIMIT_MIN, 1.0);

        // Map the scaler onto the range above hover, so the limiter never
        // takes away the throttle needed to stay airborne.
        let hover = hover.get();
        hover + ((1.0 - hover) * self.throttle_limit)
    }
}
