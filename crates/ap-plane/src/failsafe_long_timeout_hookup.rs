//! Short-to-long RC failsafe promotion timer, `FS_LONG_TIMEOUT`.
//!
//! Upstream `Plane::check_long_failsafe` in `ArduPlane/system.cpp` (the RC
//! half) and `Parameters.cpp` (`FS_LONG_TIMEOUT`). Short failsafe is raised
//! as soon as `failsafe.rc_failsafe` is set (`check_short_rc_failsafe`).
//! A long event then fires when
//! `millis() - failsafe.last_valid_rc_ms` is strictly older than
//! `FS_LONG_TIMEOUT` seconds. Landing and already-`FAILSAFE_LONG` /
//! `FAILSAFE_GCS` states do not re-enter. Recovery when LONG and RC
//! returns is the matching `failsafe_long_off_event` path.
//!
//! Mode change via `failsafe_long_on_event` is left to
//! [`crate::failsafe_action_hookup`]. GCS heartbeat timeout is left to
//! [`crate::gcs_failsafe_hookup`].

/// Upstream `FS_LONG_TIMEOUT` default, seconds.
pub const FS_LONG_TIMEOUT_DEFAULT: f32 = 5.0;
/// Upstream `@Range` lower bound for `FS_LONG_TIMEOUT`.
pub const FS_LONG_TIMEOUT_MIN: f32 = 1.0;
/// Upstream `@Range` upper bound for `FS_LONG_TIMEOUT`.
pub const FS_LONG_TIMEOUT_MAX: f32 = 300.0;

/// Upstream `failsafe_state` in `ArduPlane/defines.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailsafeState {
    /// 0 — `FAILSAFE_NONE`.
    None = 0,
    /// 1 — `FAILSAFE_SHORT` (RC loss, short action already applied).
    Short = 1,
    /// 2 — `FAILSAFE_LONG`.
    Long = 2,
    /// 3 — `FAILSAFE_GCS`.
    Gcs = 3,
}

impl FailsafeState {
    /// Whether `check_long_failsafe` treats this as already-long.
    #[must_use]
    pub const fn is_long_or_gcs(self) -> bool {
        matches!(self, Self::Long | Self::Gcs)
    }
}

/// Inputs for the RC half of `Plane::check_long_failsafe`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongTimeoutInputs {
    /// `millis()`.
    pub now_ms: u32,
    /// `failsafe.last_valid_rc_ms` — last frame with a healthy throttle PWM.
    pub last_valid_rc_ms: u32,
    /// `failsafe.rc_failsafe`.
    pub rc_failsafe: bool,
    /// `FS_LONG_TIMEOUT` seconds.
    pub timeout_s: f32,
    /// Current `failsafe.state`.
    pub state: FailsafeState,
    /// `flight_stage == LAND`.
    pub landing: bool,
}

impl Default for LongTimeoutInputs {
    fn default() -> Self {
        Self {
            now_ms: 0,
            last_valid_rc_ms: 0,
            rc_failsafe: false,
            timeout_s: FS_LONG_TIMEOUT_DEFAULT,
            state: FailsafeState::None,
            landing: false,
        }
    }
}

/// What the RC half of `check_long_failsafe` asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LongTimeoutDecision {
    /// Stay in the current failsafe state.
    Hold,
    /// `failsafe_long_on_event(FAILSAFE_LONG, RADIO_FAILSAFE)`.
    PromoteLong,
    /// `failsafe_long_off_event(RADIO_FAILSAFE)` — RC recovered while LONG.
    Recover,
}

/// True when `now - last_valid_rc` is strictly older than `timeout_s` seconds.
///
/// Matches `(tnow - failsafe.last_valid_rc_ms) > g.fs_timeout_long*1000`.
/// Unlike the GCS clocks, a zero stamp is still a legal start time (boot).
#[must_use]
pub fn rc_lost_past_long_timeout(now_ms: u32, last_valid_rc_ms: u32, timeout_s: f32) -> bool {
    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream promotes the uint32 age to float against fs_timeout_long*1000"
    )]
    let age_ms = now_ms.wrapping_sub(last_valid_rc_ms) as f32;
    age_ms > timeout_s * 1000.0
}

/// Resolve the RC half of `Plane::check_long_failsafe`.
///
/// Entry is gated on not already LONG/GCS and not landing. Exit from LONG
/// runs whenever RC is healthy again, including while landing.
#[must_use]
pub fn check_rc_long_failsafe(inp: &LongTimeoutInputs) -> LongTimeoutDecision {
    if !inp.state.is_long_or_gcs() && !inp.landing {
        if inp.rc_failsafe
            && rc_lost_past_long_timeout(inp.now_ms, inp.last_valid_rc_ms, inp.timeout_s)
        {
            return LongTimeoutDecision::PromoteLong;
        }
        return LongTimeoutDecision::Hold;
    }
    if inp.state == FailsafeState::Long && !inp.rc_failsafe {
        return LongTimeoutDecision::Recover;
    }
    LongTimeoutDecision::Hold
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_values_match_upstream() {
        assert!((FS_LONG_TIMEOUT_DEFAULT - 5.0).abs() < 1e-6);
        assert!((FS_LONG_TIMEOUT_MIN - 1.0).abs() < 1e-6);
        assert!((FS_LONG_TIMEOUT_MAX - 300.0).abs() < 1e-6);
        assert_eq!(FailsafeState::None as u8, 0);
        assert_eq!(FailsafeState::Short as u8, 1);
        assert_eq!(FailsafeState::Long as u8, 2);
        assert_eq!(FailsafeState::Gcs as u8, 3);
        assert!(!FailsafeState::None.is_long_or_gcs());
        assert!(!FailsafeState::Short.is_long_or_gcs());
        assert!(FailsafeState::Long.is_long_or_gcs());
        assert!(FailsafeState::Gcs.is_long_or_gcs());
    }

    #[test]
    fn promotes_after_long_timeout() {
        let mut inp = LongTimeoutInputs {
            last_valid_rc_ms: 1_000,
            now_ms: 1_000,
            rc_failsafe: true,
            state: FailsafeState::Short,
            ..LongTimeoutInputs::default()
        };
        assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
        inp.now_ms = 1_000 + 5_000;
        assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
        inp.now_ms = 1_000 + 5_001;
        assert_eq!(
            check_rc_long_failsafe(&inp),
            LongTimeoutDecision::PromoteLong
        );
    }
}
