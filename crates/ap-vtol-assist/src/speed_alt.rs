//! Speed / altitude trigger for VTOL assistance.
//!
//! After the enable / check gate is open, request assist when airspeed is
//! below `Q_ASSIST_SPEED` or height AGL is below `Q_ASSIST_ALT`. Upstream
//! `VTOL_Assist::should_assist` speed / alt half (the `speed_assist` flag
//! and the `alt_error` trigger, without `Q_ASSIST_DELAY` hysteresis).
//!
//! Angle-error, spin recovery, and the rest of `should_assist` are not
//! here.

use crate::assist::{AssistState, VtolAssist};

/// Sensor / estimator inputs the speed / alt half of `should_assist` needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedAltSample {
    /// Estimated airspeed (m/s). Upstream `aspeed`.
    pub aspeed: f32,
    /// True when an airspeed estimate is available. Upstream `have_airspeed`.
    pub have_airspeed: bool,
    /// Height above ground (m). Upstream
    /// `relative_ground_altitude(RangeFinderUse::ASSIST)`.
    pub height_agl: f32,
}

impl SpeedAltSample {
    /// Build a sample from the three `should_assist` speed / alt inputs.
    #[must_use]
    pub const fn new(aspeed: f32, have_airspeed: bool, height_agl: f32) -> Self {
        Self {
            aspeed,
            have_airspeed,
            height_agl,
        }
    }
}

/// Result of one speed / alt evaluation. Mirrors the logging getters
/// `in_force_assist` / `in_speed_assist` / `in_alt_assist`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SpeedAltDecision {
    force_assist: bool,
    speed_assist: bool,
    alt_assist: bool,
}

impl SpeedAltDecision {
    /// No request; all flags clear. Upstream `reset` of the speed / alt
    /// half when assist is disabled.
    #[must_use]
    pub const fn idle() -> Self {
        Self {
            force_assist: false,
            speed_assist: false,
            alt_assist: false,
        }
    }

    /// Upstream `in_force_assist` / `force_assist`.
    #[must_use]
    pub const fn force_assist(&self) -> bool {
        self.force_assist
    }

    /// Upstream `in_speed_assist` / `speed_assist`.
    #[must_use]
    pub const fn speed_assist(&self) -> bool {
        self.speed_assist
    }

    /// Upstream `in_alt_assist` / `alt_error.is_active()` without delay.
    #[must_use]
    pub const fn alt_assist(&self) -> bool {
        self.alt_assist
    }

    /// Whether assist is requested. Upstream
    /// `force_assist || speed_assist || alt_error.is_active()` (angle
    /// left for a later slice).
    #[must_use]
    pub const fn requested(&self) -> bool {
        self.force_assist || self.speed_assist || self.alt_assist
    }
}

/// Evaluate the speed / alt half of `should_assist`.
///
/// - [`AssistState::AssistDisabled`]: flags cleared, no request.
/// - `Q_ASSIST_SPEED <= 0`: speed / alt flags stay clear; only
///   force-enable still requests.
/// - Speed: `have_airspeed && aspeed < Q_ASSIST_SPEED`.
/// - Alt: `Q_ASSIST_ALT > 0 && height_agl < Q_ASSIST_ALT`, only when
///   the speed gate is open (upstream resets `alt_error` when
///   `speed <= 0`).
#[must_use]
pub fn evaluate_speed_alt(assist: &VtolAssist, sample: SpeedAltSample) -> SpeedAltDecision {
    if assist.state() == AssistState::AssistDisabled {
        return SpeedAltDecision::idle();
    }

    let force_assist = assist.state() == AssistState::ForceEnabled;

    if !assist.speed_checks_enabled() {
        return SpeedAltDecision {
            force_assist,
            speed_assist: false,
            alt_assist: false,
        };
    }

    let speed_assist = sample.have_airspeed && sample.aspeed < assist.speed();
    let alt_assist = assist.alt() > 0 && sample.height_agl < f32::from(assist.alt());

    SpeedAltDecision {
        force_assist,
        speed_assist,
        alt_assist,
    }
}
