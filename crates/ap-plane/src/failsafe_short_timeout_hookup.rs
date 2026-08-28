//! Short RC failsafe entry-delay timer, `FS_SHORT_TIMEOUT`.
//!
//! Upstream `Parameters.cpp` (`FS_SHORT_TIMEOUT`) and the documented
//! short-failsafe entry delay: a failsafe condition must persist for
//! `FS_SHORT_TIMEOUT` seconds before `FS_SHORT_ACTN` fires
//! (`failsafe_short_on_event` / `rc_failsafe_short_on_event` in
//! `ArduPlane/events.cpp` and `Plane::check_short_failsafe` in
//! `ArduPlane/system.cpp`). Plane-4.6 default is 1.5 s with `@Range` 1–100.
//!
//! Later master removed the parameter (`k_param_fs_timeout_short_unused`,
//! PR 30350) and raised SHORT as soon as `failsafe.rc_failsafe` was set.
//! This stub keeps the documented 4.6 entry delay: RC loss stays in
//! `FAILSAFE_NONE` until `millis() - failsafe.last_valid_rc_ms` is
//! strictly older than `FS_SHORT_TIMEOUT` seconds. Landing and already-
//! `FAILSAFE_SHORT` / `FAILSAFE_LONG` / `FAILSAFE_GCS` states do not
//! re-enter. Recovery when SHORT and RC returns is the matching
//! `failsafe_short_off_event` path.
//!
//! Mode change via `FS_SHORT_ACTN` is left to
//! [`crate::failsafe_action_hookup`]. Short-to-long promotion is left to
//! [`crate::failsafe_long_timeout_hookup`].

use crate::failsafe_long_timeout_hookup::FailsafeState;

/// Upstream `FS_SHORT_TIMEOUT` default, seconds.
pub const FS_SHORT_TIMEOUT_DEFAULT: f32 = 1.5;
/// Upstream `@Range` lower bound for `FS_SHORT_TIMEOUT`.
pub const FS_SHORT_TIMEOUT_MIN: f32 = 1.0;
/// Upstream `@Range` upper bound for `FS_SHORT_TIMEOUT`.
pub const FS_SHORT_TIMEOUT_MAX: f32 = 100.0;

/// Inputs for the RC half of `Plane::check_short_failsafe`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShortTimeoutInputs {
    /// `millis()`.
    pub now_ms: u32,
    /// `failsafe.last_valid_rc_ms` — last frame with a healthy throttle PWM.
    pub last_valid_rc_ms: u32,
    /// `failsafe.rc_failsafe`.
    pub rc_failsafe: bool,
    /// `FS_SHORT_TIMEOUT` seconds.
    pub timeout_s: f32,
    /// Current `failsafe.state`.
    pub state: FailsafeState,
    /// `flight_stage == LAND`.
    pub landing: bool,
}

impl Default for ShortTimeoutInputs {
    fn default() -> Self {
        Self {
            now_ms: 0,
            last_valid_rc_ms: 0,
            rc_failsafe: false,
            timeout_s: FS_SHORT_TIMEOUT_DEFAULT,
            state: FailsafeState::None,
            landing: false,
        }
    }
}

/// What `check_short_failsafe` asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShortTimeoutDecision {
    /// Stay in the current failsafe state.
    Hold,
    /// `failsafe_short_on_event(FAILSAFE_SHORT, RADIO_FAILSAFE)`.
    EnterShort,
    /// `failsafe_short_off_event(RADIO_FAILSAFE)` — RC recovered while SHORT.
    Recover,
}

/// True when `now - last_valid_rc` is strictly older than `timeout_s` seconds.
///
/// Matches `(tnow - failsafe.last_valid_rc_ms) > g.fs_timeout_short*1000`.
/// A zero stamp is still a legal start time (boot), same as the long timer.
#[must_use]
pub fn rc_lost_past_short_timeout(now_ms: u32, last_valid_rc_ms: u32, timeout_s: f32) -> bool {
    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream promotes the uint32 age to float against fs_timeout_short*1000"
    )]
    let age_ms = now_ms.wrapping_sub(last_valid_rc_ms) as f32;
    age_ms > timeout_s * 1000.0
}

/// Resolve the RC half of `Plane::check_short_failsafe` with the 4.6 entry delay.
///
/// Entry is gated on `FAILSAFE_NONE` and not landing. Exit from SHORT runs
/// whenever RC is healthy again, including while landing.
#[must_use]
pub fn check_rc_short_failsafe(inp: &ShortTimeoutInputs) -> ShortTimeoutDecision {
    if inp.state == FailsafeState::None && !inp.landing {
        if inp.rc_failsafe
            && rc_lost_past_short_timeout(inp.now_ms, inp.last_valid_rc_ms, inp.timeout_s)
        {
            return ShortTimeoutDecision::EnterShort;
        }
        return ShortTimeoutDecision::Hold;
    }
    if inp.state == FailsafeState::Short && !inp.rc_failsafe {
        return ShortTimeoutDecision::Recover;
    }
    ShortTimeoutDecision::Hold
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_values_match_upstream() {
        assert!((FS_SHORT_TIMEOUT_DEFAULT - 1.5).abs() < 1e-6);
        assert!((FS_SHORT_TIMEOUT_MIN - 1.0).abs() < 1e-6);
        assert!((FS_SHORT_TIMEOUT_MAX - 100.0).abs() < 1e-6);
        assert_eq!(FailsafeState::None as u8, 0);
        assert_eq!(FailsafeState::Short as u8, 1);
    }

    #[test]
    fn enters_after_short_timeout() {
        let mut inp = ShortTimeoutInputs {
            last_valid_rc_ms: 1_000,
            now_ms: 1_000,
            rc_failsafe: true,
            state: FailsafeState::None,
            ..ShortTimeoutInputs::default()
        };
        assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
        inp.now_ms = 1_000 + 1_500;
        assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
        inp.now_ms = 1_000 + 1_501;
        assert_eq!(
            check_rc_short_failsafe(&inp),
            ShortTimeoutDecision::EnterShort
        );
    }
}
