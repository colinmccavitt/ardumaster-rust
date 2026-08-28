//! P/D slew-rate tracking, upstream `AP_AutoTune` `SlewLimiter` pair.
//!
//! On `start`, a non-positive `rpid.slew_limit()` is written to 150 deg/s.
//! Each `update` step scales P/D (pre-Dmod) by `45/degrees(1)`, feeds
//! `slew_limiter_P` / `slew_limiter_D`, and keeps `max_SRate_P` /
//! `max_SRate_D`.
//!
//! The generic `Filter/SlewLimiter` port already lives in `ap-filter`;
//! this module is the AutoTune wiring, not a rewrite of that filter.
//! Action / D-limit hunting stays in [`crate::action`].

use ap_filter::slew::{SlewLimiter, SlewParams};

/// Default PID slew limit, upstream `rpid.slew_limit().set_and_save(150)`.
pub const SLEW_LIMIT_DEFAULT: f32 = 150.0;

/// Slew limiter tau, upstream `slew_limit_tau = 1.0`.
pub const SLEW_LIMIT_TAU: f32 = 1.0;

/// Scale P/D into limiter units, upstream `45.0 / degrees(1)`.
///
/// `degrees(1)` is `180/π`, so this is `45 * π / 180`.
pub const SLEW_LIMIT_SCALE: f32 = 45.0 / (180.0 / core::f32::consts::PI);

/// Raise a non-positive slew limit to [`SLEW_LIMIT_DEFAULT`].
///
/// Upstream `if (!is_positive(rpid.slew_limit())) { ... set_and_save(150); }`
/// in `AP_AutoTune::start`.
#[must_use]
pub fn floor_slew_limit(slew_limit: f32) -> f32 {
    if slew_limit > 0.0 {
        slew_limit
    } else {
        SLEW_LIMIT_DEFAULT
    }
}

/// Live limiter parameters written each `update` step.
///
/// Upstream `slew_limit_max = rpid.slew_limit(); slew_limit_tau = 1.0`.
#[must_use]
pub fn slew_limit_params(slew_limit: f32) -> SlewParams {
    SlewParams {
        slew_rate_max: slew_limit,
        slew_rate_tau: SLEW_LIMIT_TAU,
    }
}

/// Scale a P or D term from before Dmod.
///
/// Upstream `(pinfo.P / pinfo.Dmod) * slew_limit_scale`. A non-positive
/// `Dmod` skips the divide so a zero modifier cannot produce inf.
#[must_use]
pub fn scale_pd_sample(term: f32, dmod: f32) -> f32 {
    let undmodded = if dmod > 0.0 { term / dmod } else { term };
    undmodded * SLEW_LIMIT_SCALE
}

/// Peak-hold one limiter's slew rate, upstream `MAX(max_SRate_*, get_slew_rate())`.
#[must_use]
pub fn peak_slew_rate(max_srate: f32, slew_rate: f32) -> f32 {
    if slew_rate > max_srate {
        slew_rate
    } else {
        max_srate
    }
}

/// Separate P and D `SlewLimiter`s plus this-cycle peaks.
///
/// Upstream `slew_limiter_P` / `slew_limiter_D` / `max_SRate_P` / `max_SRate_D`.
#[derive(Debug, Clone, Copy)]
pub struct PdSlewTrackers {
    /// Upstream `AP_AutoTune::slew_limiter_P`.
    pub limiter_p: SlewLimiter,
    /// Upstream `AP_AutoTune::slew_limiter_D`.
    pub limiter_d: SlewLimiter,
    /// Upstream `AP_AutoTune::max_SRate_P`.
    pub max_srate_p: f32,
    /// Upstream `AP_AutoTune::max_SRate_D`.
    pub max_srate_d: f32,
    /// Upstream `AP_AutoTune::slew_limit_max`.
    pub slew_limit_max: f32,
    /// Upstream `AP_AutoTune::slew_limit_tau`.
    pub slew_limit_tau: f32,
}

impl Default for PdSlewTrackers {
    fn default() -> Self {
        Self::new()
    }
}

impl PdSlewTrackers {
    /// Fresh limiters and zeroed cycle peaks.
    #[must_use]
    pub fn new() -> Self {
        Self {
            limiter_p: SlewLimiter::new(),
            limiter_d: SlewLimiter::new(),
            max_srate_p: 0.0,
            max_srate_d: 0.0,
            slew_limit_max: 0.0,
            slew_limit_tau: SLEW_LIMIT_TAU,
        }
    }

    /// Clear this-cycle peaks. Upstream zeros them on `state_change`.
    pub fn reset_cycle_peaks(&mut self) {
        self.max_srate_p = 0.0;
        self.max_srate_d = 0.0;
    }

    /// One `update` step: feed pre-Dmod P/D into the limiters and peak-hold.
    ///
    /// `slew_limit` is `rpid.slew_limit()`. `now_ms` is the port's
    /// explicit clock (ADR-0004); upstream `SlewLimiter::modifier` reads
    /// `AP_HAL::millis()` itself.
    pub fn update(&mut self, p: f32, d: f32, dmod: f32, dt: f32, now_ms: u32, slew_limit: f32) {
        self.slew_limit_max = slew_limit;
        self.slew_limit_tau = SLEW_LIMIT_TAU;
        let params = slew_limit_params(slew_limit);
        let _ = self
            .limiter_p
            .modifier(scale_pd_sample(p, dmod), dt, now_ms, params);
        let _ = self
            .limiter_d
            .modifier(scale_pd_sample(d, dmod), dt, now_ms, params);
        self.max_srate_p = peak_slew_rate(self.max_srate_p, self.limiter_p.get_slew_rate());
        self.max_srate_d = peak_slew_rate(self.max_srate_d, self.limiter_d.get_slew_rate());
    }
}
