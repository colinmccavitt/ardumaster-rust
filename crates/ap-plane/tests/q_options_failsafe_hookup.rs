//! Q_OPTIONS FS_RTL / FS_QRTL override for QuadPlane RC failsafe.
//!
//! Upstream `ArduPlane/events.cpp` short (and long) Q-mode failsafe:
//! if FS_RTL set RTL, else if FS_QRTL set QRTL, else QLAND.

use ap_plane::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use ap_plane::mode_table::ModeNumber;
use ap_plane::q_options_failsafe_hookup::{
    option_is_set, q_options_long_applies, q_options_long_failsafe_action, q_options_short_applies,
    q_options_short_failsafe_action, quadplane_failsafe_mode, Q_OPTIONS_FS_QRTL, Q_OPTIONS_FS_RTL,
};

const Q_MODES: &[ModeNumber] = &[
    ModeNumber::QStabilize,
    ModeNumber::QLoiter,
    ModeNumber::QHover,
    ModeNumber::QAutotune,
    ModeNumber::QAcro,
];

#[test]
fn option_bits_match_upstream_quadplane() {
    assert_eq!(Q_OPTIONS_FS_QRTL, 1 << 5);
    assert_eq!(Q_OPTIONS_FS_RTL, 1 << 20);
    assert!(option_is_set(Q_OPTIONS_FS_QRTL, Q_OPTIONS_FS_QRTL));
    assert!(!option_is_set(0, Q_OPTIONS_FS_QRTL));
    assert!(!option_is_set(Q_OPTIONS_FS_QRTL, Q_OPTIONS_FS_RTL));
}

#[test]
fn q_modes_rtl_then_qrtl_else_qland() {
    for mode in Q_MODES {
        assert!(q_options_short_applies(*mode));
        assert!(q_options_long_applies(*mode));
        assert_eq!(
            q_options_short_failsafe_action(*mode, FailsafeActionShort::Circle, 0),
            FailsafeActionResult::Switch(ModeNumber::QLand)
        );
        assert_eq!(
            q_options_short_failsafe_action(*mode, FailsafeActionShort::Circle, Q_OPTIONS_FS_QRTL),
            FailsafeActionResult::Switch(ModeNumber::QRtl)
        );
        assert_eq!(
            q_options_short_failsafe_action(*mode, FailsafeActionShort::Circle, Q_OPTIONS_FS_RTL),
            FailsafeActionResult::Switch(ModeNumber::Rtl)
        );
        assert_eq!(
            q_options_short_failsafe_action(
                *mode,
                FailsafeActionShort::Fbwa,
                Q_OPTIONS_FS_RTL | Q_OPTIONS_FS_QRTL
            ),
            FailsafeActionResult::Switch(ModeNumber::Rtl),
            "FS_RTL wins over FS_QRTL"
        );
        assert_eq!(
            q_options_long_failsafe_action(*mode, FailsafeActionLong::Glide, true, 0),
            FailsafeActionResult::Switch(ModeNumber::QLand)
        );
        assert_eq!(
            q_options_long_failsafe_action(*mode, FailsafeActionLong::Rtl, true, Q_OPTIONS_FS_QRTL),
            FailsafeActionResult::Switch(ModeNumber::QRtl)
        );
        assert_eq!(
            q_options_long_failsafe_action(
                *mode,
                FailsafeActionLong::Continue,
                true,
                Q_OPTIONS_FS_RTL
            ),
            FailsafeActionResult::Switch(ModeNumber::Rtl)
        );
    }
    assert_eq!(quadplane_failsafe_mode(0), ModeNumber::QLand);
}

#[test]
fn disabled_short_still_never_enters_and_never_modes_stay_put() {
    assert_eq!(
        q_options_short_failsafe_action(
            ModeNumber::QStabilize,
            FailsafeActionShort::Disabled,
            Q_OPTIONS_FS_RTL
        ),
        FailsafeActionResult::Continue
    );
    for mode in [
        ModeNumber::QLand,
        ModeNumber::QRtl,
        ModeNumber::LoiterAltQLand,
    ] {
        assert!(!q_options_short_applies(mode));
        assert_eq!(
            q_options_short_failsafe_action(mode, FailsafeActionShort::Circle, Q_OPTIONS_FS_RTL),
            FailsafeActionResult::Continue
        );
        assert_eq!(
            q_options_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, Q_OPTIONS_FS_QRTL),
            FailsafeActionResult::Continue
        );
    }
}

#[test]
fn stick_and_auto_modes_ignore_q_options() {
    for mode in [ModeNumber::Manual, ModeNumber::Auto, ModeNumber::Guided] {
        assert!(!q_options_short_applies(mode));
        assert_eq!(
            q_options_short_failsafe_action(mode, FailsafeActionShort::Circle, Q_OPTIONS_FS_RTL),
            short_failsafe_action(mode, FailsafeActionShort::Circle)
        );
        assert_eq!(
            q_options_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, Q_OPTIONS_FS_QRTL),
            long_failsafe_action(mode, FailsafeActionLong::Rtl, true)
        );
    }
}
