//! GCS RC override timeout, upstream `RC_Channel::has_override`.
//!
//! A MAVLink `RC_CHANNELS_OVERRIDE` writes a PWM into `override_value` and
//! stamps `last_override_time`. After `RC_OVERRIDE_TIME` seconds the
//! override expires and `read_input` falls back to the receiver. The
//! parameter is not only a timer: 0 disables overrides, and a negative
//! value means they never expire.
//!
//! Scheduler glue that already reads pulses lives in ap-plane; this
//! module is the RC_Channel-side timer.

/// Upstream `RC_Channels::_override_timeout` / `RC_OVERRIDE_TIME` default.
pub const RC_OVERRIDE_TIME_DEFAULT: f32 = 3.0;
/// Upstream `@Range` lower bound for `RC_OVERRIDE_TIME`.
pub const RC_OVERRIDE_TIME_MIN: f32 = 0.0;
/// Upstream `@Range` upper bound for `RC_OVERRIDE_TIME`.
pub const RC_OVERRIDE_TIME_MAX: f32 = 120.0;
/// MAVLink `UINT16_MAX`: leave this channel's override unchanged.
pub const RC_OVERRIDE_IGNORE: u16 = u16::MAX;
/// MAVLink `UINT16_MAX-1` on channels 9–16: release back to the receiver.
pub const RC_OVERRIDE_RELEASE: u16 = u16::MAX - 1;

/// How `RC_OVERRIDE_TIME` is interpreted, upstream `get_override_timeout_ms`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrideTimeout {
    /// `RC_OVERRIDE_TIME > 0` — expire after this many milliseconds.
    ExpireAfter(u32),
    /// `RC_OVERRIDE_TIME == 0` — overrides are disabled.
    Disabled,
    /// `RC_OVERRIDE_TIME < 0` — overrides never expire.
    Never,
}

/// Decode `RC_OVERRIDE_TIME` (seconds) into a timeout policy.
///
/// Upstream `RC_Channels::get_override_timeout_ms`: a positive value
/// returns `true` with `value * 1000` ms; zero returns `true` with 0
/// (disabled); a negative value returns `false` (never time out).
#[must_use]
pub fn override_timeout_from_param(seconds: f32) -> OverrideTimeout {
    if seconds > 0.0 {
        OverrideTimeout::ExpireAfter((seconds * 1_000.0) as u32)
    } else if seconds == 0.0 {
        OverrideTimeout::Disabled
    } else {
        OverrideTimeout::Never
    }
}

/// Per-channel GCS override, upstream `RC_Channel` override fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcOverride {
    /// Upstream `override_value`. Zero means no override is stored.
    pub override_value: u16,
    /// Upstream `last_override_time` (milliseconds).
    pub last_override_time: u32,
}

impl Default for RcOverride {
    fn default() -> Self {
        Self {
            override_value: 0,
            last_override_time: 0,
        }
    }
}

impl RcOverride {
    /// Store a GCS PWM, upstream `RC_Channel::set_override`.
    ///
    /// `timestamp_ms == 0` uses `now_ms`, matching the HAL `millis()`
    /// fallback. When GCS overrides are disabled the call is a no-op.
    pub fn set_override(&mut self, value: u16, timestamp_ms: u32, now_ms: u32, enabled: bool) {
        if !enabled {
            return;
        }
        self.last_override_time = if timestamp_ms != 0 {
            timestamp_ms
        } else {
            now_ms
        };
        self.override_value = value;
    }

    /// Drop a stored override, upstream `RC_Channel::clear_override`.
    pub fn clear_override(&mut self) {
        self.last_override_time = 0;
        self.override_value = 0;
    }

    /// True while a stored override is still live, upstream `has_override`.
    ///
    /// Zero `override_value` is never live. `RC_OVERRIDE_TIME == 0` kills
    /// every override. A negative timeout never expires. Otherwise the
    /// window is exclusive at the deadline (`now - last < timeout_ms`).
    #[must_use]
    pub fn has_override(&self, timeout_s: f32, now_ms: u32) -> bool {
        if self.override_value == 0 {
            return false;
        }
        match override_timeout_from_param(timeout_s) {
            OverrideTimeout::Never => true,
            OverrideTimeout::Disabled => false,
            OverrideTimeout::ExpireAfter(timeout_ms) => {
                if timeout_ms == 0 {
                    return false;
                }
                now_ms.wrapping_sub(self.last_override_time) < timeout_ms
            }
        }
    }

    /// Receiver PWM, or the GCS value while the override is live.
    ///
    /// Upstream `RC_Channel::read_input` replaces `radio_in` when
    /// `has_override()` and `IGNORE_OVERRIDES` is clear.
    #[must_use]
    pub fn read_input(
        &self,
        radio_in: u16,
        timeout_s: f32,
        now_ms: u32,
        ignore_overrides: bool,
    ) -> u16 {
        if !ignore_overrides && self.has_override(timeout_s, now_ms) {
            self.override_value
        } else {
            radio_in
        }
    }
}

/// Apply one MAVLink `RC_CHANNELS_OVERRIDE` field to a channel.
///
/// Channels 1–8 (`chan` 0–7): `UINT16_MAX` is ignored; any other value
/// including 0 is stored (0 releases that channel). Channels 9–16:
/// 0 and `UINT16_MAX` are ignored; `UINT16_MAX-1` stores 0 (release).
/// Returns `true` when the stored override changed.
#[must_use]
pub fn apply_gcs_override_field(
    ov: &mut RcOverride,
    chan: u8,
    raw: u16,
    now_ms: u32,
    enabled: bool,
) -> bool {
    if chan < 8 {
        if raw == RC_OVERRIDE_IGNORE {
            return false;
        }
        ov.set_override(raw, now_ms, now_ms, enabled);
        enabled
    } else {
        if raw == 0 || raw == RC_OVERRIDE_IGNORE {
            return false;
        }
        let value = if raw == RC_OVERRIDE_RELEASE { 0 } else { raw };
        ov.set_override(value, now_ms, now_ms, enabled);
        enabled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_rc_override_time() {
        assert!((RC_OVERRIDE_TIME_DEFAULT - 3.0).abs() < 1e-6);
        assert!((RC_OVERRIDE_TIME_MIN - 0.0).abs() < 1e-6);
        assert!((RC_OVERRIDE_TIME_MAX - 120.0).abs() < 1e-6);
        assert_eq!(
            override_timeout_from_param(RC_OVERRIDE_TIME_DEFAULT),
            OverrideTimeout::ExpireAfter(3000)
        );
        assert_eq!(override_timeout_from_param(0.0), OverrideTimeout::Disabled);
        assert_eq!(override_timeout_from_param(-1.0), OverrideTimeout::Never);
    }

    #[test]
    fn default_timeout_expires_at_three_seconds() {
        let mut ov = RcOverride::default();
        ov.set_override(1600, 1_000, 1_000, true);
        assert!(ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 1_000));
        assert!(ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 3_999));
        assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 4_000));
        assert_eq!(
            ov.read_input(1500, RC_OVERRIDE_TIME_DEFAULT, 3_999, false),
            1600
        );
        assert_eq!(
            ov.read_input(1500, RC_OVERRIDE_TIME_DEFAULT, 4_000, false),
            1500
        );
    }

    #[test]
    fn zero_timeout_disables_overrides() {
        let mut ov = RcOverride::default();
        ov.set_override(1800, 0, 500, true);
        assert_eq!(ov.last_override_time, 500);
        assert!(!ov.has_override(0.0, 500));
        assert_eq!(ov.read_input(1400, 0.0, 500, false), 1400);
    }

    #[test]
    fn negative_timeout_never_expires() {
        let mut ov = RcOverride::default();
        ov.set_override(1700, 10, 10, true);
        assert!(ov.has_override(-1.0, 10));
        assert!(ov.has_override(-1.0, 10 + 3_600_000));
        assert_eq!(ov.read_input(1500, -1.0, 99_999, false), 1700);
    }

    #[test]
    fn disabled_gcs_gate_is_a_noop() {
        let mut ov = RcOverride::default();
        ov.set_override(1600, 100, 100, false);
        assert_eq!(ov.override_value, 0);
        assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 100));
    }

    #[test]
    fn clear_and_zero_value_drop_the_override() {
        let mut ov = RcOverride::default();
        ov.set_override(1600, 100, 100, true);
        ov.clear_override();
        assert_eq!(ov.override_value, 0);
        assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 100));
        ov.set_override(0, 200, 200, true);
        assert!(!ov.has_override(-1.0, 200));
    }

    #[test]
    fn ignore_overrides_keeps_receiver_pwm() {
        let mut ov = RcOverride::default();
        ov.set_override(1600, 100, 100, true);
        assert_eq!(
            ov.read_input(1500, RC_OVERRIDE_TIME_DEFAULT, 100, true),
            1500
        );
    }

    #[test]
    fn gcs_uint16_max_is_ignored() {
        let mut ov = RcOverride::default();
        assert!(!apply_gcs_override_field(
            &mut ov,
            0,
            RC_OVERRIDE_IGNORE,
            50,
            true
        ));
        assert_eq!(ov.override_value, 0);
        assert!(apply_gcs_override_field(&mut ov, 0, 1550, 50, true));
        assert_eq!(ov.override_value, 1550);
        assert!(apply_gcs_override_field(&mut ov, 0, 0, 60, true));
        assert_eq!(ov.override_value, 0);
        assert!(!apply_gcs_override_field(&mut ov, 8, 0, 70, true));
        assert!(apply_gcs_override_field(
            &mut ov,
            8,
            RC_OVERRIDE_RELEASE,
            80,
            true
        ));
        assert_eq!(ov.override_value, 0);
    }
}
