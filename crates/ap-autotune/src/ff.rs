//! I-term / FF coupling after an AutoTune demand event.
//!
//! Upstream `AP_AutoTune::update` constrains the median-filtered FF
//! against the previous FF (`AUTOTUNE_INCREASE_FF_STEP` 12% /
//! `AUTOTUNE_DECREASE_FF_STEP` 15%), then writes I from P and FF.
//! Roll uses the smaller of FF or P; pitch/yaw use
//! `max(P * AUTOTUNE_I_RATIO, FF / TRIM_TCONST)`. IMAX is clamped to
//! `[AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX]` at `start`. The FF
//! single-event estimate and `ff_filter` stay later slices.

use crate::gains::AtGains;
use crate::state::{AtType, AutoTune};

/// Increase-FF step percent, upstream `AUTOTUNE_INCREASE_FF_STEP`.
pub const AUTOTUNE_INCREASE_FF_STEP: f32 = 12.0;

/// Decrease-FF step percent, upstream `AUTOTUNE_DECREASE_FF_STEP`.
pub const AUTOTUNE_DECREASE_FF_STEP: f32 = 15.0;

/// I / P ratio on pitch/yaw, upstream `AUTOTUNE_I_RATIO`.
pub const AUTOTUNE_I_RATIO: f32 = 0.75;

/// Lower IMAX clamp, upstream `AUTOTUNE_MIN_IMAX`.
pub const AUTOTUNE_MIN_IMAX: f32 = 0.4;

/// Upper IMAX clamp, upstream `AUTOTUNE_MAX_IMAX`.
pub const AUTOTUNE_MAX_IMAX: f32 = 0.9;

/// Rate-trim time constant (seconds), upstream `TRIM_TCONST`.
pub const TRIM_TCONST: f32 = 1.0;

/// Constrain a candidate FF against the previous FF.
///
/// Upstream `constrain_float(FF, old_FF*(1-DECREASE*0.01),
/// old_FF*(1+INCREASE*0.01))`.
#[must_use]
pub fn constrain_ff_step(old_ff: f32, ff: f32) -> f32 {
    let lo = old_ff * (1.0 - AUTOTUNE_DECREASE_FF_STEP * 0.01);
    let hi = old_ff * (1.0 + AUTOTUNE_INCREASE_FF_STEP * 0.01);
    if ff < lo {
        lo
    } else if ff > hi {
        hi
    } else {
        ff
    }
}

/// I-term from P and FF for `axis`.
///
/// Roll: `min(P, FF / TRIM_TCONST)`. Pitch/yaw:
/// `max(P * AUTOTUNE_I_RATIO, FF / TRIM_TCONST)`.
#[must_use]
pub fn couple_i(axis: AtType, p: f32, ff: f32) -> f32 {
    let ff_term = ff / TRIM_TCONST;
    match axis {
        AtType::Roll => {
            if p < ff_term {
                p
            } else {
                ff_term
            }
        }
        AtType::Pitch | AtType::Yaw => {
            let p_term = p * AUTOTUNE_I_RATIO;
            if p_term > ff_term {
                p_term
            } else {
                ff_term
            }
        }
    }
}

/// Clamp IMAX into `[AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX]`.
///
/// Upstream `constrain_float(rpid.kIMAX(), AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX)`
/// in `AP_AutoTune::start`.
#[must_use]
pub fn constrain_imax(imax: f32) -> f32 {
    if imax < AUTOTUNE_MIN_IMAX {
        AUTOTUNE_MIN_IMAX
    } else if imax > AUTOTUNE_MAX_IMAX {
        AUTOTUNE_MAX_IMAX
    } else {
        imax
    }
}

/// Constrain `ff` vs `old_ff`, then couple I. Returns `(ff, i)`.
#[must_use]
pub fn couple_ff_i(axis: AtType, p: f32, old_ff: f32, ff: f32) -> (f32, f32) {
    let ff = constrain_ff_step(old_ff, ff);
    (ff, couple_i(axis, p, ff))
}

/// Apply [`couple_ff_i`] to `gains.i`. FF is not stored on [`AtGains`].
#[must_use]
pub fn apply_ff_i(axis: AtType, gains: AtGains, old_ff: f32, ff: f32) -> (AtGains, f32) {
    let (ff, i) = couple_ff_i(axis, gains.p, old_ff, ff);
    (AtGains { i, ..gains }, ff)
}

impl AutoTune {
    /// Constrain `ff` vs `old_ff` and write the coupled I-term.
    ///
    /// Returns the constrained FF. No-op (returns `old_ff`, leaves `i`)
    /// when not running.
    pub fn couple_ff_i(&mut self, old_ff: f32, ff: f32) -> f32 {
        if !self.running {
            return old_ff;
        }
        let (next, ff) = apply_ff_i(self.axis, self.current, old_ff, ff);
        self.current = next;
        ff
    }
}
