//! GCS failsafe enable, upstream `FS_GCS_ENABL` / `Plane::check_long_failsafe`.
//!
//! Disabled / Heartbeat / HeartbeatAndRADIO_STATUS; trip after `FS_LONG_TIMEOUT`.

use ap_plane::gcs_failsafe_hookup::{
    gcs_failsafe_should_trigger, GcsFailsafeEnable, GcsFailsafeInputs, GcsFailsafeTracker,
    FS_LONG_TIMEOUT_DEFAULT,
};
use ap_plane::mode_table::ModeNumber;

fn heartbeat_after(enable: GcsFailsafeEnable, seen_at_ms: u32, now_ms: u32) -> GcsFailsafeInputs {
    let mut tracker = GcsFailsafeTracker::default();
    tracker.note_heartbeat(seen_at_ms);
    GcsFailsafeInputs {
        enable,
        now_ms,
        tracker,
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        mode: ModeNumber::Manual,
        already_in_long_or_gcs: false,
        landing: false,
    }
}

#[test]
fn fs_gcs_enabl_values_match_upstream() {
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
    assert!(GcsFailsafeEnable::HeartbeatAndRadioStatus.is_enabled());
}

#[test]
fn disabled_never_trips_after_timeout() {
    let inp = heartbeat_after(GcsFailsafeEnable::Disabled, 1_000, 1_000 + 60_000);
    assert!(!gcs_failsafe_should_trigger(&inp));
}

#[test]
fn first_heartbeat_has_not_started_tracking() {
    let inp = GcsFailsafeInputs {
        enable: GcsFailsafeEnable::Heartbeat,
        now_ms: 60_000,
        tracker: GcsFailsafeTracker::default(),
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        mode: ModeNumber::Auto,
        already_in_long_or_gcs: false,
        landing: false,
    };
    assert!(!gcs_failsafe_should_trigger(&inp));
}

#[test]
fn heartbeat_trips_after_fs_long_timeout() {
    let fresh = heartbeat_after(GcsFailsafeEnable::Heartbeat, 2_000, 2_000);
    assert!(!gcs_failsafe_should_trigger(&fresh));
    // `age > FS_LONG_TIMEOUT * 1000` — exclusive at the deadline.
    let at_deadline = heartbeat_after(GcsFailsafeEnable::Heartbeat, 2_000, 2_000 + 5_000);
    assert!(!gcs_failsafe_should_trigger(&at_deadline));
    let past = heartbeat_after(GcsFailsafeEnable::Heartbeat, 2_000, 2_000 + 5_001);
    assert!(gcs_failsafe_should_trigger(&past));
}

#[test]
fn heartbeat_and_radio_status_trips_on_zero_remrssi_timeout() {
    let mut tracker = GcsFailsafeTracker::default();
    tracker.note_heartbeat(1_000);
    tracker.note_radio_status(1_000, 50);
    // Zero remrssi does not refresh the stamp; a later heartbeat keeps
    // the HEARTBEAT path healthy so only remrssi can trip.
    tracker.note_radio_status(6_001, 0);
    tracker.note_heartbeat(6_001);
    let inp = GcsFailsafeInputs {
        enable: GcsFailsafeEnable::HeartbeatAndRadioStatus,
        now_ms: 6_001,
        tracker,
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        mode: ModeNumber::Manual,
        already_in_long_or_gcs: false,
        landing: false,
    };
    assert!(gcs_failsafe_should_trigger(&inp));

    let heartbeat_only = GcsFailsafeInputs {
        enable: GcsFailsafeEnable::Heartbeat,
        ..inp
    };
    assert!(!gcs_failsafe_should_trigger(&heartbeat_only));
}

#[test]
fn heartbeat_and_auto_only_trips_in_auto() {
    let mut inp = heartbeat_after(GcsFailsafeEnable::HeartbeatAndAuto, 500, 500 + 5_001);
    inp.mode = ModeNumber::Manual;
    assert!(!gcs_failsafe_should_trigger(&inp));
    inp.mode = ModeNumber::Auto;
    assert!(gcs_failsafe_should_trigger(&inp));
}

#[test]
fn landing_or_already_long_does_not_reenter() {
    let mut inp = heartbeat_after(GcsFailsafeEnable::Heartbeat, 100, 100 + 5_001);
    inp.landing = true;
    assert!(!gcs_failsafe_should_trigger(&inp));
    inp.landing = false;
    inp.already_in_long_or_gcs = true;
    assert!(!gcs_failsafe_should_trigger(&inp));
}
