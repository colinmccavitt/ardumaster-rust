//! GCS failsafe enable, upstream `FS_GCS_ENABL` / `Plane::check_long_failsafe`.
//!
//! `ArduPlane/defines.h` (`gcs_failsafe`), `Parameters.cpp` (`FS_GCS_ENABL`,
//! `FS_LONG_TIMEOUT`), and the GCS half of `Plane::check_long_failsafe` in
//! `ArduPlane/system.cpp`. Heartbeat tracking only becomes active after the
//! first heartbeat from `MAV_GCS_SYSID`. `HeartbeatAndRADIO_STATUS` also
//! watches `RADIO_STATUS.remrssi`: a zero remrssi does not refresh the
//! timestamp (`GCS_Common.cpp`), so a one-way link times out the same way.
//!
//! This stub decides whether a GCS failsafe event would fire. Mode change
//! via `failsafe_long_on_event` is left to [`crate::failsafe_action_hookup`].

use crate::mode_table::ModeNumber;

/// Upstream `FS_LONG_TIMEOUT` default, seconds.
pub const FS_LONG_TIMEOUT_DEFAULT: f32 = 5.0;
/// Upstream `@Range` lower bound for `FS_LONG_TIMEOUT`.
pub const FS_LONG_TIMEOUT_MIN: f32 = 1.0;
/// Upstream `@Range` upper bound for `FS_LONG_TIMEOUT`.
pub const FS_LONG_TIMEOUT_MAX: f32 = 300.0;

/// Upstream `gcs_failsafe` / `FS_GCS_ENABL`.
///
/// Default is [`Self::Disabled`]. Heartbeat tracking stays inactive until
/// the first `HEARTBEAT` from the primary GCS (`last_heartbeat_ms != 0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GcsFailsafeEnable {
    /// 0 — `GCS_FAILSAFE_OFF`. No GCS failsafe.
    Disabled = 0,
    /// 1 — `GCS_FAILSAFE_HEARTBEAT`. Timeout after lost HEARTBEAT.
    Heartbeat = 1,
    /// 2 — `GCS_FAILSAFE_HB_RSSI`. Lost HEARTBEAT or stale remrssi.
    HeartbeatAndRadioStatus = 2,
    /// 3 — `GCS_FAILSAFE_HB_AUTO`. HEARTBEAT timeout, but only in AUTO.
    HeartbeatAndAuto = 3,
}

impl GcsFailsafeEnable {
    /// Decode `FS_GCS_ENABL`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::Heartbeat),
            2 => Some(Self::HeartbeatAndRadioStatus),
            3 => Some(Self::HeartbeatAndAuto),
            _ => None,
        }
    }

    /// Upstream `FS_GCS_ENABL` default, `GCS_FAILSAFE_OFF`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::Disabled
    }

    /// Whether any GCS-loss path is armed.
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        !matches!(self, Self::Disabled)
    }
}

/// Last-seen stamps for the primary GCS, upstream `GCS_MAVLINK` clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GcsFailsafeTracker {
    /// `sysid_mygcs_last_seen_time_ms()`. Zero until the first heartbeat.
    pub last_heartbeat_ms: u32,
    /// `last_radio_status_remrssi_ms()`. Zero until remrssi is non-zero.
    pub last_remrssi_ms: u32,
}

impl Default for GcsFailsafeTracker {
    fn default() -> Self {
        Self {
            last_heartbeat_ms: 0,
            last_remrssi_ms: 0,
        }
    }
}

impl GcsFailsafeTracker {
    /// Record a HEARTBEAT from `MAV_GCS_SYSID`.
    pub fn note_heartbeat(&mut self, now_ms: u32) {
        self.last_heartbeat_ms = now_ms;
    }

    /// Record a `RADIO_STATUS`. Only a non-zero remrssi refreshes the stamp.
    pub fn note_radio_status(&mut self, now_ms: u32, remrssi: u8) {
        if remrssi != 0 {
            self.last_remrssi_ms = now_ms;
        }
    }
}

/// Inputs for the GCS half of `Plane::check_long_failsafe`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcsFailsafeInputs {
    /// `FS_GCS_ENABL`.
    pub enable: GcsFailsafeEnable,
    /// `millis()`.
    pub now_ms: u32,
    /// Last-seen stamps from [`GcsFailsafeTracker`].
    pub tracker: GcsFailsafeTracker,
    /// `FS_LONG_TIMEOUT` seconds.
    pub timeout_s: f32,
    /// Current flight mode (only AUTO matters for [`GcsFailsafeEnable::HeartbeatAndAuto`]).
    pub mode: ModeNumber,
    /// Already in `FAILSAFE_LONG` or `FAILSAFE_GCS` — only act on changes.
    pub already_in_long_or_gcs: bool,
    /// `flight_stage == LAND`.
    pub landing: bool,
}

impl Default for GcsFailsafeInputs {
    fn default() -> Self {
        Self {
            enable: GcsFailsafeEnable::default_param(),
            now_ms: 0,
            tracker: GcsFailsafeTracker::default(),
            timeout_s: FS_LONG_TIMEOUT_DEFAULT,
            mode: ModeNumber::Manual,
            already_in_long_or_gcs: false,
            landing: false,
        }
    }
}

/// True when `now - last` is strictly older than `timeout_s` seconds.
///
/// `last_ms == 0` means the clock has never started (first heartbeat /
/// first healthy remrssi), matching `gcs_last_seen_ms != 0`.
#[must_use]
pub fn gcs_link_timed_out(now_ms: u32, last_ms: u32, timeout_s: f32) -> bool {
    if last_ms == 0 {
        return false;
    }
    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream promotes the uint32 age to float against fs_timeout_long*1000"
    )]
    let age_ms = now_ms.wrapping_sub(last_ms) as f32;
    age_ms > timeout_s * 1000.0
}

/// Whether `check_long_failsafe` would raise `FAILSAFE_GCS`.
#[must_use]
pub fn gcs_failsafe_should_trigger(inp: &GcsFailsafeInputs) -> bool {
    if inp.already_in_long_or_gcs || inp.landing {
        return false;
    }
    let heartbeat_lost =
        gcs_link_timed_out(inp.now_ms, inp.tracker.last_heartbeat_ms, inp.timeout_s);
    match inp.enable {
        GcsFailsafeEnable::Disabled => false,
        GcsFailsafeEnable::Heartbeat => heartbeat_lost,
        GcsFailsafeEnable::HeartbeatAndAuto => {
            heartbeat_lost && matches!(inp.mode, ModeNumber::Auto)
        }
        GcsFailsafeEnable::HeartbeatAndRadioStatus => {
            heartbeat_lost
                || gcs_link_timed_out(inp.now_ms, inp.tracker.last_remrssi_ms, inp.timeout_s)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_values_match_upstream_defines() {
        assert_eq!(
            GcsFailsafeEnable::from_param(0),
            Some(GcsFailsafeEnable::Disabled)
        );
        assert_eq!(
            GcsFailsafeEnable::from_param(1),
            Some(GcsFailsafeEnable::Heartbeat)
        );
        assert_eq!(
            GcsFailsafeEnable::from_param(2),
            Some(GcsFailsafeEnable::HeartbeatAndRadioStatus)
        );
        assert_eq!(
            GcsFailsafeEnable::from_param(3),
            Some(GcsFailsafeEnable::HeartbeatAndAuto)
        );
        assert_eq!(GcsFailsafeEnable::from_param(4), None);
        assert_eq!(
            GcsFailsafeEnable::default_param(),
            GcsFailsafeEnable::Disabled
        );
        assert!(!GcsFailsafeEnable::Disabled.is_enabled());
        assert!(GcsFailsafeEnable::Heartbeat.is_enabled());
        assert!((FS_LONG_TIMEOUT_DEFAULT - 5.0).abs() < 1e-6);
        assert!((FS_LONG_TIMEOUT_MIN - 1.0).abs() < 1e-6);
        assert!((FS_LONG_TIMEOUT_MAX - 300.0).abs() < 1e-6);
    }

    #[test]
    fn heartbeat_trips_after_long_timeout() {
        let mut tracker = GcsFailsafeTracker::default();
        tracker.note_heartbeat(1_000);
        let mut inp = GcsFailsafeInputs {
            enable: GcsFailsafeEnable::Heartbeat,
            now_ms: 1_000,
            tracker,
            ..GcsFailsafeInputs::default()
        };
        assert!(!gcs_failsafe_should_trigger(&inp));
        inp.now_ms = 1_000 + 5_000;
        assert!(!gcs_failsafe_should_trigger(&inp));
        inp.now_ms = 1_000 + 5_001;
        assert!(gcs_failsafe_should_trigger(&inp));
    }
}
