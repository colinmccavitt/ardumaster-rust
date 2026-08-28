//! Roll/pitch gain snapshot, upstream `ATGains` / `save_gains` / `restore_gains`.
//!
//! `start` copies the live axis gains into `restore` and `last_save`.
//! `stop` writes the tuned set only when both P and D limits were found;
//! otherwise it puts the snapshot back (abort / incomplete session).
//! EEPROM `save_*_if_changed` is deferred with the parameter system.

/// One axis of AutoTune gains, upstream `AP_AutoTune::ATGains`.
///
/// Rate/tau fields match `ap-control::RateGains` (FW-017) in meaning but
/// live here so this crate stays free of that dependency. PID terms are
/// the values `get_gains` reads from `AC_PID`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtGains {
    /// Time constant, seconds. Upstream `ATGains::tau`.
    pub tau: f32,
    /// Positive max rate, deg/s. Upstream `ATGains::rmax_pos`.
    pub rmax_pos: f32,
    /// Negative max rate, deg/s. Upstream `ATGains::rmax_neg`.
    pub rmax_neg: f32,
    /// Proportional gain. Upstream `ATGains::P`.
    pub p: f32,
    /// Integral gain. Upstream `ATGains::I`.
    pub i: f32,
    /// Derivative gain. Upstream `ATGains::D`.
    pub d: f32,
}

impl AtGains {
    /// All-zero snapshot, matching an unset `restore` / `last_save`.
    pub const ZERO: Self = Self {
        tau: 0.0,
        rmax_pos: 0.0,
        rmax_neg: 0.0,
        p: 0.0,
        i: 0.0,
        d: 0.0,
    };
}

impl Default for AtGains {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Whether `stop` persists the tuned gains instead of restoring.
///
/// Upstream `is_positive(D_limit) && is_positive(P_limit)`.
#[must_use]
pub const fn should_save_on_stop(p_limit: f32, d_limit: f32) -> bool {
    p_limit > 0.0 && d_limit > 0.0
}

/// Copy live gains into the restore and last-save snapshots.
///
/// Upstream `current = restore = last_save = get_gains()` in `start`.
#[must_use]
pub const fn snapshot_gains(current: AtGains) -> (AtGains, AtGains) {
    (current, current)
}

/// Apply the `stop` save-or-restore choice.
///
/// Returns `(current, last_save)`. Abort copies `restore` onto `current`
/// and leaves `last_save` alone. A completed tune (both limits positive)
/// keeps `current` and records it as `last_save`.
#[must_use]
pub const fn apply_stop_gains(
    current: AtGains,
    restore: AtGains,
    last_save: AtGains,
    p_limit: f32,
    d_limit: f32,
) -> (AtGains, AtGains) {
    if should_save_on_stop(p_limit, d_limit) {
        (current, current)
    } else {
        (restore, last_save)
    }
}
