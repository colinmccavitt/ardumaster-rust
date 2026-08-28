//! RC / GCS failsafe recovery, upstream `events.cpp` off-events.
//!
//! `rc_failsafe_short_off_event` clears `FAILSAFE_SHORT` and restores the
//! saved mode when the current mode is still due to radio failsafe.
//! `failsafe_long_off_event` clears long / GCS state without a mode restore.

use ap_plane::failsafe_long_timeout_hookup::FailsafeState;
use ap_plane::failsafe_off_event_hookup::{
    failsafe_long_off_event, rc_failsafe_short_off_event, FailsafeOffReason, LongOffInputs,
    ShortOffInputs, MODE_REASON_GCS_FAILSAFE, MODE_REASON_RADIO_FAILSAFE,
    MODE_REASON_RADIO_FAILSAFE_RECOVERY,
};
use ap_plane::mode::ModeReason;
use ap_plane::mode_table::ModeNumber;

#[test]
fn short_off_clears_state_and_restores_saved_mode_when_reason_is_radio_failsafe() {
    let out = rc_failsafe_short_off_event(&ShortOffInputs {
        current_mode: ModeNumber::Circle,
        saved_mode: ModeNumber::Manual,
        control_mode_reason: MODE_REASON_RADIO_FAILSAFE,
    });
    assert_eq!(out.state, FailsafeState::None);
    assert_eq!(out.restore_mode, Some(ModeNumber::Manual));
    assert_eq!(out.restore_reason, MODE_REASON_RADIO_FAILSAFE_RECOVERY);
    assert_eq!(
        ModeReason::from_number(out.restore_reason).as_number(),
        MODE_REASON_RADIO_FAILSAFE_RECOVERY
    );
}

#[test]
fn short_off_does_not_restore_mode_after_a_later_pilot_or_gcs_change() {
    for reason in [
        ModeReason::GcsCommand.as_number(),
        ModeReason::Initialised.as_number(),
        MODE_REASON_GCS_FAILSAFE,
        MODE_REASON_RADIO_FAILSAFE_RECOVERY,
    ] {
        let out = rc_failsafe_short_off_event(&ShortOffInputs {
            current_mode: ModeNumber::FlyByWireA,
            saved_mode: ModeNumber::Manual,
            control_mode_reason: reason,
        });
        assert_eq!(out.state, FailsafeState::None);
        assert_eq!(out.restore_mode, None);
    }
}

#[test]
fn short_off_still_asks_to_restore_when_saved_mode_matches_current() {
    let out = rc_failsafe_short_off_event(&ShortOffInputs {
        current_mode: ModeNumber::Auto,
        saved_mode: ModeNumber::Auto,
        control_mode_reason: MODE_REASON_RADIO_FAILSAFE,
    });
    assert_eq!(out.restore_mode, Some(ModeNumber::Auto));
}

#[test]
fn long_off_clears_state_and_pending_without_restoring_a_mode() {
    let radio = failsafe_long_off_event(&LongOffInputs {
        reason: FailsafeOffReason::Radio,
        failsafe_gcs: true,
    });
    assert_eq!(radio.state, FailsafeState::None);
    assert!(!radio.long_failsafe_pending);
    assert!(
        radio.failsafe_gcs,
        "RC long off leaves the GCS notify flag alone"
    );

    let gcs = failsafe_long_off_event(&LongOffInputs {
        reason: FailsafeOffReason::Gcs,
        failsafe_gcs: true,
    });
    assert_eq!(gcs.state, FailsafeState::None);
    assert!(!gcs.long_failsafe_pending);
    assert!(!gcs.failsafe_gcs);
}

#[test]
fn long_off_reason_numbers_match_upstream_mode_reason() {
    assert_eq!(
        FailsafeOffReason::Radio.as_mode_reason(),
        MODE_REASON_RADIO_FAILSAFE
    );
    assert_eq!(
        FailsafeOffReason::Gcs.as_mode_reason(),
        MODE_REASON_GCS_FAILSAFE
    );
    assert_eq!(
        ModeReason::from_number(MODE_REASON_RADIO_FAILSAFE).as_number(),
        3
    );
    assert_eq!(
        ModeReason::from_number(MODE_REASON_GCS_FAILSAFE).as_number(),
        5
    );
}
