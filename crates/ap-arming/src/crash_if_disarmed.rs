//! `ARMING_CRASH_IF_DISARMED` / crash-check-while-disarmed. FW-026.
//!
//! Upstream crash-dump (`AP_Arming::crashdump_checks`) and vehicle
//! crash detectors skip the disarmed path: a leftover crash dump is a
//! pre-arm concern, and in-flight crash detection requires motors
//! armed. This option turns that disarmed path on so the same
//! crash-dump / tip-over sample is inspected while disarmed.
//!
//! Default is 0 (`DISABLED`). This slice is the gate, not the dump
//! parser or the IMU crash body. `CRSDP_IGN` ack is the operator
//! override when a dump is present.

use crate::{Check, NamedCheck};

/// Default `ARMING_CRASH_IF_DISARMED`.
pub const ARMING_CRASH_IF_DISARMED_DEFAULT: CrashIfDisarmed = CrashIfDisarmed::Disabled;

/// Registry name used when this gate fills `Check::Parameters`.
pub const CRASH_DUMP_CHECK_NAME: &str = "CrashDump";

/// Upstream `ARMING_CRASH_IF_DISARMED` — run crash-dump / crash-check
/// while the vehicle is disarmed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CrashIfDisarmed {
    /// 0 — do not inspect crash-dump / crash-check while disarmed.
    Disabled = 0,
    /// 1 — inspect leftover crash-dump / tip-over while disarmed.
    Enabled = 1,
}

impl CrashIfDisarmed {
    /// Decode a stored `ARMING_CRASH_IF_DISARMED` value.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Disabled),
            1 => Some(Self::Enabled),
            _ => None,
        }
    }

    /// The stored parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Whether the parameter turns the disarmed crash-check path on.
    #[must_use]
    pub const fn enabled(self) -> bool {
        matches!(self, Self::Enabled)
    }
}

/// Whether crash-dump / crash-check should run on this tick.
///
/// Armed: always false for *this* option — in-flight crash detection
/// is a different path. Disarmed: true only when the parameter is
/// `Enabled`.
#[must_use]
pub const fn crash_check_while_disarmed(option: CrashIfDisarmed, armed: bool) -> bool {
    !armed && option.enabled()
}

/// Whether a leftover crash dump should refuse arm.
///
/// No dump, or dump acknowledged (`CRSDP_IGN`), is ok. An unacked dump
/// refuses only when the disarmed crash-check gate is running.
#[must_use]
pub const fn crash_dump_allows_arm(
    option: CrashIfDisarmed,
    armed: bool,
    dump_present: bool,
    dump_acked: bool,
) -> bool {
    if !crash_check_while_disarmed(option, armed) {
        return true;
    }
    !dump_present || dump_acked
}

/// Fill `Check::Parameters` from the disarmed crash-dump gate.
///
/// When the gate is off the entry is ok so the registry does not
/// refuse on a leftover dump. When on, an unacked dump fails.
#[must_use]
pub const fn crash_if_disarmed_named_check(
    option: CrashIfDisarmed,
    armed: bool,
    dump_present: bool,
    dump_acked: bool,
) -> NamedCheck {
    NamedCheck {
        check: Check::Parameters,
        name: CRASH_DUMP_CHECK_NAME,
        ok: crash_dump_allows_arm(option, armed, dump_present, dump_acked),
    }
}
