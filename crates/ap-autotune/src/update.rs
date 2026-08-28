//! Saturation / overshoot P update and tau/rmax slew.
//!
//! After a demand event `AP_AutoTune::update` rewrites P — raise when the
//! axis is still saturated and not oscillating (`Action::RAISE_P` /
//! `RAISE_PD`, `P *= 1.3`), lower when it overshoots (`Action::LOWER_P`,
//! `P *= 0.35`) — then `update_rmax` walks `tau` / `rmax` toward the
//! `AUTOTUNE_LEVEL` target. D-limit hunting, FF median filter, and the
//! FF/I inverse-tau clamp are later slices.

use crate::gains::AtGains;
use crate::state::AutoTune;

/// Raise-P multiplier, upstream `P *= 1.3` (`Action::RAISE_P` / `RAISE_PD`).
pub const RAISE_P_MUL: f32 = 1.3;

/// First overshoot cut, upstream `P *= 0.35` (`Action::LOWER_P` / `LOWER_PD`).
pub const LOWER_P_MUL: f32 = 0.35;

/// `RMAX` step per event, upstream `current.rmax_pos.get()±20`.
pub const RMAX_STEP: f32 = 20.0;

/// Conservative `rmax_pos` when the live value is zero.
///
/// Upstream `if (current.rmax_pos == 0) current.rmax_pos.set(75)`.
pub const RMAX_DEFAULT: f32 = 75.0;

/// Lower tau slew bound, upstream `current.tau*0.85`.
pub const TAU_SLEW_DOWN: f32 = 0.85;

/// Upper tau slew bound, upstream `current.tau*1.15`.
pub const TAU_SLEW_UP: f32 = 1.15;

/// Which P rewrite `update` chose this event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainAction {
    /// No P change (neither flag, or still settling FF).
    None,
    /// Saturated, not overshooting — raise P.
    RaiseP,
    /// Overshoot / oscillation — lower P.
    LowerP,
}

/// Choose the P rewrite from the saturation / overshoot flags.
///
/// Overshoot wins: upstream checks `min_Dmod < 1.0` (oscillation) before
/// the "not oscillating, increase P" branch.
#[must_use]
pub const fn gain_action(saturated: bool, overshoot: bool) -> GainAction {
    if overshoot {
        GainAction::LowerP
    } else if saturated {
        GainAction::RaiseP
    } else {
        GainAction::None
    }
}

/// Apply the P multiplier for `action`.
#[must_use]
pub fn apply_p_step(p: f32, action: GainAction) -> f32 {
    match action {
        GainAction::None => p,
        GainAction::RaiseP => p * RAISE_P_MUL,
        GainAction::LowerP => p * LOWER_P_MUL,
    }
}

/// Walk `rmax` toward `target` by at most [`RMAX_STEP`].
///
/// A zero live value is replaced with [`RMAX_DEFAULT`] first.
#[must_use]
pub fn slew_rmax(current: f32, target: f32) -> f32 {
    let current = if current == 0.0 {
        RMAX_DEFAULT
    } else {
        current
    };
    let lo = current - RMAX_STEP;
    let hi = current + RMAX_STEP;
    if target < lo {
        lo
    } else if target > hi {
        hi
    } else {
        target
    }
}

/// Walk `tau` toward `target` by at most 15%.
///
/// Upstream `constrain_float(target_tau, current.tau*0.85, current.tau*1.15)`.
#[must_use]
pub fn slew_tau(current: f32, target: f32) -> f32 {
    let lo = current * TAU_SLEW_DOWN;
    let hi = current * TAU_SLEW_UP;
    if target < lo {
        lo
    } else if target > hi {
        hi
    } else {
        target
    }
}

/// One `update_rmax` step: slew `tau` / `rmax_pos`, copy `rmax_neg`.
///
/// The FF/I inverse-tau raise (`target_tau = MAX(target_tau, 1/invtau)`)
/// is not applied here.
#[must_use]
pub fn couple_tau_rmax(gains: AtGains, target_tau: f32, target_rmax: f32) -> AtGains {
    let rmax_pos = slew_rmax(gains.rmax_pos, target_rmax);
    AtGains {
        tau: slew_tau(gains.tau, target_tau),
        rmax_pos,
        rmax_neg: rmax_pos,
        ..gains
    }
}

/// One event's P rewrite plus the `update_rmax` tau/rmax slew.
///
/// `saturated` is the "not oscillating, increase P" path. `overshoot` is
/// the `min_Dmod < 1.0` / `LOWER_P` path.
#[must_use]
pub fn update_gains(
    gains: AtGains,
    saturated: bool,
    overshoot: bool,
    target_tau: f32,
    target_rmax: f32,
) -> AtGains {
    let mut next = gains;
    next.p = apply_p_step(gains.p, gain_action(saturated, overshoot));
    couple_tau_rmax(next, target_tau, target_rmax)
}

impl AutoTune {
    /// Apply [`update_gains`] to the live axis when the session is running.
    ///
    /// No-op when not running, matching the early return at the top of
    /// `AP_AutoTune::update`.
    pub fn update_gains(
        &mut self,
        saturated: bool,
        overshoot: bool,
        target_tau: f32,
        target_rmax: f32,
    ) {
        if !self.running {
            return;
        }
        self.current = update_gains(self.current, saturated, overshoot, target_tau, target_rmax);
    }
}
