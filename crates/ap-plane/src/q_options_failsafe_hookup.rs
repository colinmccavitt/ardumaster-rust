//! Q_OPTIONS FS_RTL / FS_QRTL override for QuadPlane RC failsafe.
//!
//! Upstream `rc_failsafe_short_on_event` / `failsafe_long_on_event` in
//! `ArduPlane/events.cpp`. QSTABILIZE / QHOVER / QLOITER / QACRO / QAUTOTUNE
//! ignore `FS_SHORT_ACTN` / `FS_LONG_ACTN` and pick:
//! - `Q_OPTIONS` bit 20 `QuadPlane::Option::FS_RTL` → RTL
//! - else `Q_OPTIONS` bit 5 `QuadPlane::Option::FS_QRTL` → QRTL
//! - else QLAND
//!
//! This stub wraps [`crate::failsafe_action_hookup`]; it does not rewrite
//! the stick / AUTO / Never table. QLAND / QRTL / LOITER_ALT_QLAND stay
//! no-action modes.

use crate::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use crate::mode_table::ModeNumber;

/// `Q_OPTIONS` bit 5, `QuadPlane::Option::FS_QRTL`.
pub const Q_OPTIONS_FS_QRTL: u32 = 1 << 5;
/// `Q_OPTIONS` bit 20, `QuadPlane::Option::FS_RTL`.
pub const Q_OPTIONS_FS_RTL: u32 = 1 << 20;

/// Whether `q_options` has `option` set (`QuadPlane::option_is_set`).
#[must_use]
pub const fn option_is_set(q_options: u32, option: u32) -> bool {
    q_options & option != 0
}

/// Modes that consult `Q_OPTIONS` FS_RTL / FS_QRTL on short failsafe.
#[must_use]
pub fn q_options_short_applies(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::QStabilize
            | ModeNumber::QLoiter
            | ModeNumber::QHover
            | ModeNumber::QAutotune
            | ModeNumber::QAcro
    )
}

/// Modes that consult `Q_OPTIONS` FS_RTL / FS_QRTL on long failsafe.
///
/// Same Q-mode set as short — upstream uses the same Option check.
#[must_use]
pub fn q_options_long_applies(mode: ModeNumber) -> bool {
    q_options_short_applies(mode)
}

/// Mode chosen by `Q_OPTIONS` when a Q-mode failsafe fires.
///
/// FS_RTL wins over FS_QRTL when both bits are set.
#[must_use]
pub fn quadplane_failsafe_mode(q_options: u32) -> ModeNumber {
    if option_is_set(q_options, Q_OPTIONS_FS_RTL) {
        ModeNumber::Rtl
    } else if option_is_set(q_options, Q_OPTIONS_FS_QRTL) {
        ModeNumber::QRtl
    } else {
        ModeNumber::QLand
    }
}

/// `rc_failsafe_short_on_event` after the Q_OPTIONS override.
///
/// Does not rewrite [`short_failsafe_action`]. Disabled short failsafe
/// still never enters the event. Non-Q modes stay on the existing table.
#[must_use]
pub fn q_options_short_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionShort,
    q_options: u32,
) -> FailsafeActionResult {
    if !action.is_enabled() {
        return FailsafeActionResult::Continue;
    }
    if q_options_short_applies(mode) {
        return FailsafeActionResult::Switch(quadplane_failsafe_mode(q_options));
    }
    short_failsafe_action(mode, action)
}

/// `failsafe_long_on_event` after the Q_OPTIONS override.
///
/// Does not rewrite [`long_failsafe_action`].
#[must_use]
pub fn q_options_long_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionLong,
    autoland_available: bool,
    q_options: u32,
) -> FailsafeActionResult {
    if q_options_long_applies(mode) {
        return FailsafeActionResult::Switch(quadplane_failsafe_mode(q_options));
    }
    long_failsafe_action(mode, action, autoland_available)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fs_rtl_wins_then_fs_qrtl_else_qland() {
        assert_eq!(Q_OPTIONS_FS_QRTL, 1 << 5);
        assert_eq!(Q_OPTIONS_FS_RTL, 1 << 20);
        assert_eq!(quadplane_failsafe_mode(0), ModeNumber::QLand);
        assert_eq!(quadplane_failsafe_mode(Q_OPTIONS_FS_QRTL), ModeNumber::QRtl);
        assert_eq!(quadplane_failsafe_mode(Q_OPTIONS_FS_RTL), ModeNumber::Rtl);
        assert_eq!(
            quadplane_failsafe_mode(Q_OPTIONS_FS_RTL | Q_OPTIONS_FS_QRTL),
            ModeNumber::Rtl,
            "FS_RTL wins over FS_QRTL when both bits are set"
        );
    }
}
