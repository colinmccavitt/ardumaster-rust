//! Angle-error trigger for VTOL assistance.
//!
//! After the enable / check gate is open, request assist when attitude
//! is outside the flight envelope *and* the roll / pitch error vs nav
//! demand exceeds `Q_ASSIST_ANGLE`. Upstream `VTOL_Assist::should_assist`
//! angle half (`angle_error` trigger, without `Q_ASSIST_DELAY` hysteresis).
//!
//! Force, speed / alt, spin recovery, and the rest of `should_assist`
//! are not here.

use crate::assist::{AssistState, VtolAssist};

/// Extra degrees beyond `ROLL_LIMIT_DEG` / `PTCH_LIM_*` still counted
/// as inside the envelope. Upstream `allowed_envelope_error_deg`.
pub const ALLOWED_ENVELOPE_ERROR_DEG: f32 = 5.0;

/// Attitude / demand / envelope inputs the angle half of `should_assist` needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngleSample {
    /// AHRS roll (deg). Upstream `ahrs.get_roll_deg()`.
    pub roll_deg: f32,
    /// AHRS pitch (deg). Upstream `ahrs.get_pitch_deg()`.
    pub pitch_deg: f32,
    /// Demanded roll (centidegrees). Upstream `nav_roll_cd`.
    pub nav_roll_cd: i32,
    /// Demanded pitch (centidegrees). Upstream `nav_pitch_cd`.
    pub nav_pitch_cd: i32,
    /// `ROLL_LIMIT_DEG`. Upstream `aparm.roll_limit`.
    pub roll_limit_deg: f32,
    /// `PTCH_LIM_MAX_DEG`. Upstream `aparm.pitch_limit_max`.
    pub pitch_limit_max_deg: f32,
    /// `PTCH_LIM_MIN_DEG`. Upstream `aparm.pitch_limit_min`.
    pub pitch_limit_min_deg: f32,
}

impl AngleSample {
    /// Build a sample from the `should_assist` angle inputs.
    #[must_use]
    pub const fn new(
        roll_deg: f32,
        pitch_deg: f32,
        nav_roll_cd: i32,
        nav_pitch_cd: i32,
        roll_limit_deg: f32,
        pitch_limit_max_deg: f32,
        pitch_limit_min_deg: f32,
    ) -> Self {
        Self {
            roll_deg,
            pitch_deg,
            nav_roll_cd,
            nav_pitch_cd,
            roll_limit_deg,
            pitch_limit_max_deg,
            pitch_limit_min_deg,
        }
    }

    /// Demanded roll in degrees. Upstream `nav_roll_cd * 0.01`.
    #[must_use]
    pub fn nav_roll_deg(self) -> f32 {
        self.nav_roll_cd as f32 * 0.01
    }

    /// Demanded pitch in degrees. Upstream `nav_pitch_cd * 0.01`.
    #[must_use]
    pub fn nav_pitch_deg(self) -> f32 {
        self.nav_pitch_cd as f32 * 0.01
    }

    /// Absolute roll error vs nav demand (deg).
    #[must_use]
    pub fn roll_error_deg(self) -> f32 {
        (self.roll_deg - self.nav_roll_deg()).abs()
    }

    /// Absolute pitch error vs nav demand (deg).
    #[must_use]
    pub fn pitch_error_deg(self) -> f32 {
        (self.pitch_deg - self.nav_pitch_deg()).abs()
    }

    /// Attitude is inside the limited envelope plus 5 deg slack.
    ///
    /// Upstream `inside_envelope`: `|roll| <= roll_limit + 5`,
    /// `pitch < pitch_limit_max + 5`, `pitch > pitch_limit_min - 5`.
    #[must_use]
    pub fn inside_envelope(self) -> bool {
        self.roll_deg.abs() <= self.roll_limit_deg + ALLOWED_ENVELOPE_ERROR_DEG
            && self.pitch_deg < self.pitch_limit_max_deg + ALLOWED_ENVELOPE_ERROR_DEG
            && self.pitch_deg > self.pitch_limit_min_deg - ALLOWED_ENVELOPE_ERROR_DEG
    }

    /// Both axes are inside `Q_ASSIST_ANGLE` of the nav demand.
    ///
    /// Upstream `inside_angle_error`: `|roll - nav_roll| < angle` and
    /// `|pitch - nav_pitch| < angle`. Equality is *not* inside.
    #[must_use]
    pub fn inside_angle_error(self, angle_deg: i8) -> bool {
        let limit = f32::from(angle_deg);
        self.roll_error_deg() < limit && self.pitch_error_deg() < limit
    }

    /// Instantaneous angle trigger (no delay). Upstream
    /// `!inside_envelope && !inside_angle_error`.
    #[must_use]
    pub fn trigger(self, angle_deg: i8) -> bool {
        !self.inside_envelope() && !self.inside_angle_error(angle_deg)
    }
}

/// Result of one angle-error evaluation. Mirrors the logging getter
/// `in_angle_assist` plus the force flag from the same `should_assist`
/// return.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AngleDecision {
    force_assist: bool,
    angle_assist: bool,
}

impl AngleDecision {
    /// No request; flags cleared. Upstream `reset` of the angle half
    /// when assist is disabled.
    #[must_use]
    pub const fn idle() -> Self {
        Self {
            force_assist: false,
            angle_assist: false,
        }
    }

    /// Upstream `in_force_assist` / `force_assist`.
    #[must_use]
    pub const fn force_assist(&self) -> bool {
        self.force_assist
    }

    /// Upstream `in_angle_assist` / `angle_error.is_active()` without delay.
    #[must_use]
    pub const fn angle_assist(&self) -> bool {
        self.angle_assist
    }

    /// Whether assist is requested. Upstream
    /// `force_assist || angle_error.is_active()` (speed / alt left
    /// for the other slices).
    #[must_use]
    pub const fn requested(&self) -> bool {
        self.force_assist || self.angle_assist
    }
}

/// `Q_ASSIST_ANGLE > 0` *and* the speed gate is open.
///
/// Upstream resets `angle_error` when `speed <= 0`, and skips the
/// envelope / error test when `angle <= 0`.
#[must_use]
pub fn angle_check_enabled(assist: &VtolAssist) -> bool {
    assist.speed_checks_enabled() && assist.angle() > 0
}

/// Evaluate the angle-error half of `should_assist`.
///
/// - [`AssistState::AssistDisabled`]: flags cleared, no request.
/// - `Q_ASSIST_SPEED <= 0`: angle flag stays clear; only force-enable
///   still requests.
/// - `Q_ASSIST_ANGLE <= 0`: angle assist disabled.
/// - Else: request when outside the envelope *and* outside
///   `Q_ASSIST_ANGLE` of the nav demand.
#[must_use]
pub fn evaluate_angle(assist: &VtolAssist, sample: AngleSample) -> AngleDecision {
    if assist.state() == AssistState::AssistDisabled {
        return AngleDecision::idle();
    }

    let force_assist = assist.state() == AssistState::ForceEnabled;

    if !angle_check_enabled(assist) {
        return AngleDecision {
            force_assist,
            angle_assist: false,
        };
    }

    AngleDecision {
        force_assist,
        angle_assist: sample.trigger(assist.angle()),
    }
}
