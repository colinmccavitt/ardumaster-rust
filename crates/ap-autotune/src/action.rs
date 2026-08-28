//! Action / D-limit hunting, upstream `AP_AutoTune::Action` oscillation paths.
//!
//! After a demand event `update` hunts a D ceiling: raise D until
//! `min_Dmod < 1.0` oscillation, then `LOWER_D` / `LOWER_PD` to store
//! `D_limit`. Idle oscillation uses `IDLE_LOWER_PD`. P-only
//! saturation / overshoot stays in [`crate::update`]; the N-cycle
//! `save_gains` closer stays in [`crate::completeness`].

use crate::gains::AtGains;
use crate::state::AutoTune;

/// Raise-D multiplier, upstream `D *= 1.3` (`Action::RAISE_D`).
pub const RAISE_D_MUL: f32 = 1.3;

/// First D-limit discovery, upstream `D *= 0.3` when `!is_positive(D_limit)`.
pub const LOWER_D_FIRST_MUL: f32 = 0.3;

/// Further D-limit cut, upstream `D *= 0.35` after `D_limit` is set.
pub const LOWER_D_AGAIN_MUL: f32 = 0.35;

/// P cut on joint P/D oscillation, upstream `P *= 0.35` (`Action::LOWER_PD`).
pub const LOWER_PD_P_MUL: f32 = 0.35;

/// D cut on joint P/D oscillation, upstream `D *= 0.75` (`Action::LOWER_PD`).
pub const LOWER_PD_D_MUL: f32 = 0.75;

/// Idle oscillation floor, upstream `gain_mul = 0.5` (`Action::IDLE_LOWER_PD`).
pub const IDLE_LOWER_GAIN_MUL: f32 = 0.5;

/// P-vs-D ratio that chooses `LOWER_PD` over first `LOWER_D`.
///
/// Upstream `max_P > 0.5 * max_D`.
pub const P_DOMINATES_D: f32 = 0.5;

/// D-vs-P ratio that lowers D again after `D_limit` is set.
///
/// Upstream `max_D > 0.8 * max_P`.
pub const D_DOMINATES_P: f32 = 0.8;

/// Idle-oscillation dwell, upstream `now - state_enter_ms > 500`.
pub const IDLE_OSCILLATE_MS: u32 = 500;

/// Idle Dmod trip, upstream `max_Dmod < 0.9`.
pub const IDLE_DMOD_THRESH: f32 = 0.9;

/// Settle after writing `D_limit`, upstream `now - D_set_ms > 2000`.
pub const D_SET_SETTLE_MS: u32 = 2000;

/// Upstream `AP_AutoTune::Action` (logged as `log_ATRP.action`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Action {
    /// No D-hunt rewrite this event.
    None = 0,
    /// Demand too slow, upstream `Action::LOW_RATE` (later reject slice).
    LowRate = 1,
    /// Sample shorter than 100 ms, upstream `Action::SHORT` (later reject slice).
    Short = 2,
    /// Not oscillating, raise P after `D_limit`, upstream `Action::RAISE_PD`.
    RaisePd = 3,
    /// Oscillation without `D_limit` while P dominates D.
    LowerPd = 4,
    /// Oscillating while idle, upstream `Action::IDLE_LOWER_PD`.
    IdleLowerPd = 5,
    /// Not oscillating, no `D_limit` yet, upstream `Action::RAISE_D`.
    RaiseD = 6,
    /// P-only raise, upstream `Action::RAISE_P` (see [`crate::update`]).
    RaiseP = 7,
    /// Oscillation sets or lowers `D_limit`, upstream `Action::LOWER_D`.
    LowerD = 8,
    /// P-only overshoot cut, upstream `Action::LOWER_P` (see [`crate::update`]).
    LowerP = 9,
}

impl Action {
    /// Decode an upstream `Action` discriminant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::None),
            1 => Some(Self::LowRate),
            2 => Some(Self::Short),
            3 => Some(Self::RaisePd),
            4 => Some(Self::LowerPd),
            5 => Some(Self::IdleLowerPd),
            6 => Some(Self::RaiseD),
            7 => Some(Self::RaiseP),
            8 => Some(Self::LowerD),
            9 => Some(Self::LowerP),
            _ => None,
        }
    }

    /// The stored discriminant, matching `log_ATRP.action`.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// True when `D_limit` has been discovered, upstream `is_positive(D_limit)`.
#[must_use]
pub const fn d_limit_is_set(d_limit: f32) -> bool {
    d_limit > 0.0
}

/// Whether P slew dominates D enough to take `LOWER_PD`.
#[must_use]
pub const fn p_dominates_d(max_p: f32, max_d: f32) -> bool {
    max_p > P_DOMINATES_D * max_d
}

/// Whether D slew dominates P enough to lower `D_limit` again.
#[must_use]
pub const fn d_dominates_p(max_p: f32, max_d: f32) -> bool {
    max_d > D_DOMINATES_P * max_p
}

/// Idle oscillation gate, upstream `IDLE` + 500 ms + `max_Dmod < 0.9`.
#[must_use]
pub const fn should_idle_lower_pd(idle_ms: u32, max_dmod: f32) -> bool {
    idle_ms > IDLE_OSCILLATE_MS && max_dmod < IDLE_DMOD_THRESH
}

/// Choose the D-hunt action at event end.
///
/// `oscillating` is `min_Dmod < 1.0`. `ff_ready` is `ff_count >= 4`.
/// `d_settle_ready` is `now - D_set_ms > 2000`.
#[must_use]
pub const fn hunt_d_action(
    oscillating: bool,
    d_limit: f32,
    max_p: f32,
    max_d: f32,
    ff_ready: bool,
    d_settle_ready: bool,
) -> Action {
    if oscillating && !d_limit_is_set(d_limit) {
        if p_dominates_d(max_p, max_d) {
            Action::LowerPd
        } else {
            Action::LowerD
        }
    } else if oscillating && d_settle_ready && d_dominates_p(max_p, max_d) {
        Action::LowerD
    } else if !oscillating && !d_limit_is_set(d_limit) && ff_ready {
        Action::RaiseD
    } else {
        Action::None
    }
}

/// Apply the P/D/`D_limit` rewrite for a D-hunt [`Action`].
///
/// Returns `(p, d, d_limit)`. First `LOWER_D` uses [`LOWER_D_FIRST_MUL`];
/// a later `LOWER_D` (limit already set) uses [`LOWER_D_AGAIN_MUL`].
#[must_use]
pub fn apply_d_hunt(p: f32, d: f32, d_limit: f32, action: Action) -> (f32, f32, f32) {
    match action {
        Action::RaiseD => (p, d * RAISE_D_MUL, d_limit),
        Action::LowerD => {
            let next_d = d * if d_limit_is_set(d_limit) {
                LOWER_D_AGAIN_MUL
            } else {
                LOWER_D_FIRST_MUL
            };
            (p, next_d, next_d)
        }
        Action::LowerPd => (p * LOWER_PD_P_MUL, d * LOWER_PD_D_MUL, d_limit),
        _ => (p, d, d_limit),
    }
}

/// Upstream `linear_interpolate` used by idle `IDLE_LOWER_PD`.
#[must_use]
pub fn linear_interpolate(
    mut output_low: f32,
    mut output_high: f32,
    input_value: f32,
    mut input_low: f32,
    mut input_high: f32,
) -> f32 {
    if input_low > input_high {
        core::mem::swap(&mut input_low, &mut input_high);
        core::mem::swap(&mut output_low, &mut output_high);
    }
    if input_value <= input_low {
        return output_low;
    }
    if input_value >= input_high {
        return output_high;
    }
    let frac = (input_value - input_low) / (input_high - input_low);
    output_low + (output_high - output_low) * frac
}

/// Scale P and D after idle oscillation, upstream `Action::IDLE_LOWER_PD`.
///
/// `P *= lerp(0.5, 1.0, max_SRate_P, slew_sum, 0)` and the same for D.
#[must_use]
pub fn apply_idle_lower_pd(p: f32, d: f32, max_srate_p: f32, max_srate_d: f32) -> (f32, f32) {
    let slew_sum = max_srate_p + max_srate_d;
    let p_mul = linear_interpolate(IDLE_LOWER_GAIN_MUL, 1.0, max_srate_p, slew_sum, 0.0);
    let d_mul = linear_interpolate(IDLE_LOWER_GAIN_MUL, 1.0, max_srate_d, slew_sum, 0.0);
    (p * p_mul, d * d_mul)
}

/// Cap a stored limit after idle reduce, upstream `MIN(limit, current)`.
#[must_use]
pub fn min_limit(limit: f32, current: f32) -> f32 {
    if current < limit {
        current
    } else {
        limit
    }
}

/// One event's D-hunt rewrite plus the next `D_limit`.
///
/// Returns `(gains, d_limit, action)`.
#[must_use]
pub fn hunt_d_gains(
    gains: AtGains,
    d_limit: f32,
    oscillating: bool,
    max_p: f32,
    max_d: f32,
    ff_ready: bool,
    d_settle_ready: bool,
) -> (AtGains, f32, Action) {
    let action = hunt_d_action(oscillating, d_limit, max_p, max_d, ff_ready, d_settle_ready);
    let (p, d, next_limit) = apply_d_hunt(gains.p, gains.d, d_limit, action);
    (AtGains { p, d, ..gains }, next_limit, action)
}

impl AutoTune {
    /// Apply [`hunt_d_gains`] to the live axis when the session is running.
    ///
    /// Further `LOWER_D` (limit already set) clears `done_count`, matching
    /// the "lower D limit some more" branch. No-op when not running.
    pub fn hunt_d_limit(
        &mut self,
        oscillating: bool,
        max_p: f32,
        max_d: f32,
        ff_ready: bool,
        d_settle_ready: bool,
    ) -> Action {
        if !self.running {
            return Action::None;
        }
        let had_limit = d_limit_is_set(self.d_limit);
        let (next, d_limit, action) = hunt_d_gains(
            self.current,
            self.d_limit,
            oscillating,
            max_p,
            max_d,
            ff_ready,
            d_settle_ready,
        );
        self.current = next;
        if action == Action::LowerD {
            self.d_limit = d_limit;
            if had_limit {
                self.done_count = 0;
            }
        }
        action
    }

    /// Idle-oscillation `IDLE_LOWER_PD` reduce when the 500 ms / Dmod gate trips.
    ///
    /// Returns true when P/D were scaled. Caps `p_limit` / `d_limit` with
    /// [`min_limit`].
    pub fn idle_lower_pd(
        &mut self,
        idle_ms: u32,
        max_dmod: f32,
        max_srate_p: f32,
        max_srate_d: f32,
    ) -> bool {
        if !self.running || !should_idle_lower_pd(idle_ms, max_dmod) {
            return false;
        }
        let (p, d) = apply_idle_lower_pd(self.current.p, self.current.d, max_srate_p, max_srate_d);
        self.current.p = p;
        self.current.d = d;
        self.p_limit = min_limit(self.p_limit, p);
        self.d_limit = min_limit(self.d_limit, d);
        true
    }
}
