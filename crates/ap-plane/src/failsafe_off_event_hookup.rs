//! RC / GCS failsafe recovery, `rc_failsafe_short_off_event` / `failsafe_long_off_event`.
//!
//! Upstream `ArduPlane/events.cpp`. The timeout stubs
//! ([`crate::failsafe_short_timeout_hookup`],
//! [`crate::failsafe_long_timeout_hookup`], [`crate::gcs_failsafe_hookup`])
//! decide *when* to recover. This stub is the off-event itself: clear
//! `failsafe.state` to `FAILSAFE_NONE` and, on a short RC recovery, restore
//! `failsafe.saved_mode_number` when the current mode was entered for
//! `ModeReason::RADIO_FAILSAFE`.
//!
//! Long recovery also clears `long_failsafe_pending` and the GCS notify
//! flag when the reason is `GCS_FAILSAFE`. It does not restore a saved
//! mode — that path is short-RC only. Landing-sequence / `FS_*` action
//! tables are left to the existing modules.

use crate::failsafe_long_timeout_hookup::FailsafeState;
use crate::mode_table::ModeNumber;

/// Upstream `ModeReason::RADIO_FAILSAFE`.
pub const MODE_REASON_RADIO_FAILSAFE: u8 = 3;
/// Upstream `ModeReason::GCS_FAILSAFE`.
pub const MODE_REASON_GCS_FAILSAFE: u8 = 5;
/// Upstream `ModeReason::RADIO_FAILSAFE_RECOVERY`.
pub const MODE_REASON_RADIO_FAILSAFE_RECOVERY: u8 = 48;

/// Which link recovered, matching `failsafe_long_off_event`'s `ModeReason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeOffReason {
    /// RC recovered (`RADIO_FAILSAFE`).
    Radio,
    /// GCS heartbeat recovered (`GCS_FAILSAFE`).
    Gcs,
}

impl FailsafeOffReason {
    /// Upstream `ModeReason` number for this recovery.
    #[must_use]
    pub const fn as_mode_reason(self) -> u8 {
        match self {
            Self::Radio => MODE_REASON_RADIO_FAILSAFE,
            Self::Gcs => MODE_REASON_GCS_FAILSAFE,
        }
    }

    /// Decode a `ModeReason` number into the two off-event reasons.
    #[must_use]
    pub const fn from_mode_reason(number: u8) -> Option<Self> {
        match number {
            MODE_REASON_RADIO_FAILSAFE => Some(Self::Radio),
            MODE_REASON_GCS_FAILSAFE => Some(Self::Gcs),
            _ => None,
        }
    }
}

/// Inputs for `Plane::rc_failsafe_short_off_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShortOffInputs {
    /// Mode running now (`control_mode->mode_number()`).
    pub current_mode: ModeNumber,
    /// `failsafe.saved_mode_number` captured on the short-on event.
    pub saved_mode: ModeNumber,
    /// `control_mode_reason` as the upstream `ModeReason` number.
    pub control_mode_reason: u8,
}

impl Default for ShortOffInputs {
    fn default() -> Self {
        Self {
            current_mode: ModeNumber::Circle,
            saved_mode: ModeNumber::Manual,
            control_mode_reason: MODE_REASON_RADIO_FAILSAFE,
        }
    }
}

/// What `rc_failsafe_short_off_event` asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShortOffResult {
    /// `failsafe.state = FAILSAFE_NONE`.
    pub state: FailsafeState,
    /// `set_mode_by_number(saved, RADIO_FAILSAFE_RECOVERY)` when `Some`.
    pub restore_mode: Option<ModeNumber>,
    /// `ModeReason` used with [`Self::restore_mode`].
    pub restore_reason: u8,
}

/// Inputs for `Plane::failsafe_long_off_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LongOffInputs {
    /// `ModeReason` passed to `failsafe_long_off_event` (RC vs GCS).
    pub reason: FailsafeOffReason,
    /// Current `AP_Notify::flags.failsafe_gcs`.
    pub failsafe_gcs: bool,
}

impl Default for LongOffInputs {
    fn default() -> Self {
        Self {
            reason: FailsafeOffReason::Radio,
            failsafe_gcs: false,
        }
    }
}

/// What `failsafe_long_off_event` asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LongOffResult {
    /// `failsafe.state = FAILSAFE_NONE`.
    pub state: FailsafeState,
    /// `long_failsafe_pending = false`.
    pub long_failsafe_pending: bool,
    /// `AP_Notify::flags.failsafe_gcs` after the event.
    pub failsafe_gcs: bool,
}

/// Upstream `Plane::rc_failsafe_short_off_event`.
///
/// Always clears short failsafe. Restores `saved_mode` only when the
/// current mode is still attributed to `RADIO_FAILSAFE` — a later GCS /
/// pilot change keeps the new mode.
#[must_use]
pub fn rc_failsafe_short_off_event(inp: &ShortOffInputs) -> ShortOffResult {
    let restore_mode = if inp.control_mode_reason == MODE_REASON_RADIO_FAILSAFE {
        Some(inp.saved_mode)
    } else {
        None
    };
    ShortOffResult {
        state: FailsafeState::None,
        restore_mode,
        restore_reason: MODE_REASON_RADIO_FAILSAFE_RECOVERY,
    }
}

/// Upstream `Plane::failsafe_long_off_event`.
///
/// Clears long / GCS failsafe state and the pending-long latch. GCS
/// recovery also drops the notify flag; RC recovery leaves it alone.
/// No saved-mode restore — that is the short-off path.
#[must_use]
pub fn failsafe_long_off_event(inp: &LongOffInputs) -> LongOffResult {
    LongOffResult {
        state: FailsafeState::None,
        long_failsafe_pending: false,
        failsafe_gcs: if inp.reason == FailsafeOffReason::Gcs {
            false
        } else {
            inp.failsafe_gcs
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_reason_numbers_match_upstream() {
        assert_eq!(MODE_REASON_RADIO_FAILSAFE, 3);
        assert_eq!(MODE_REASON_GCS_FAILSAFE, 5);
        assert_eq!(MODE_REASON_RADIO_FAILSAFE_RECOVERY, 48);
        assert_eq!(FailsafeOffReason::Radio.as_mode_reason(), 3);
        assert_eq!(FailsafeOffReason::Gcs.as_mode_reason(), 5);
        assert_eq!(
            FailsafeOffReason::from_mode_reason(3),
            Some(FailsafeOffReason::Radio)
        );
        assert_eq!(
            FailsafeOffReason::from_mode_reason(5),
            Some(FailsafeOffReason::Gcs)
        );
        assert_eq!(FailsafeOffReason::from_mode_reason(2), None);
        assert_eq!(FailsafeState::None as u8, 0);
    }

    #[test]
    fn short_off_clears_state_and_restores_saved_mode() {
        let inp = ShortOffInputs::default();
        let out = rc_failsafe_short_off_event(&inp);
        assert_eq!(out.state, FailsafeState::None);
        assert_eq!(out.restore_mode, Some(ModeNumber::Manual));
        assert_eq!(out.restore_reason, MODE_REASON_RADIO_FAILSAFE_RECOVERY);
    }
}
