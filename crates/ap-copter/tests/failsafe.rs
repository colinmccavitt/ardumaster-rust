//! Copter radio/GCS failsafe check+action leftover, upstream `events.cpp`.

use ap_copter::failsafe::{
    failsafe_gcs_check, failsafe_gcs_on_event, failsafe_option, failsafe_radio_on_event,
    gcs_param_action, gcs_timeout_ms, radio_param_action, should_disarm_on_failsafe,
    FailsafeAction, FailsafeOption, FsGcsEnable, FsThrEnable, GcsFailsafeActionInputs,
    GcsFailsafeEdge, GcsFailsafeInputs, RadioFailsafeInputs, FS_GCS_TIMEOUT_DEFAULT_S, MODE_ACRO,
    MODE_AUTO, MODE_AUTO_RTL, MODE_STABILIZE,
};

#[test]
fn param_values_match_upstream_defines() {
    assert_eq!(FsThrEnable::from_param(0), Some(FsThrEnable::Disabled));
    assert_eq!(FsThrEnable::from_param(1), Some(FsThrEnable::AlwaysRtl));
    assert_eq!(
        FsThrEnable::from_param(2),
        Some(FsThrEnable::ContinueMission)
    );
    assert_eq!(FsThrEnable::from_param(3), Some(FsThrEnable::AlwaysLand));
    assert_eq!(
        FsThrEnable::from_param(4),
        Some(FsThrEnable::AlwaysSmartrtlOrRtl)
    );
    assert_eq!(
        FsThrEnable::from_param(5),
        Some(FsThrEnable::AlwaysSmartrtlOrLand)
    );
    assert_eq!(FsThrEnable::from_param(6), Some(FsThrEnable::AutoRtlOrRtl));
    assert_eq!(FsThrEnable::from_param(7), Some(FsThrEnable::BrakeOrLand));
    assert_eq!(FsThrEnable::from_param(8), None);
    assert_eq!(FsThrEnable::default_param(), FsThrEnable::AlwaysRtl);

    assert_eq!(FsGcsEnable::from_param(0), Some(FsGcsEnable::Disabled));
    assert_eq!(FsGcsEnable::from_param(1), Some(FsGcsEnable::AlwaysRtl));
    assert_eq!(FsGcsEnable::from_param(5), Some(FsGcsEnable::AlwaysLand));
    assert_eq!(FsGcsEnable::from_param(7), Some(FsGcsEnable::BrakeOrLand));
    assert_eq!(FsGcsEnable::from_param(8), None);
    assert_eq!(FsGcsEnable::default_param(), FsGcsEnable::Disabled);
    assert!(!FsGcsEnable::Disabled.is_enabled());
    assert!(FsGcsEnable::AlwaysRtl.is_enabled());

    assert_eq!(FailsafeAction::None as u8, 0);
    assert_eq!(FailsafeAction::Land as u8, 1);
    assert_eq!(FailsafeAction::Rtl as u8, 2);
    assert_eq!(FailsafeAction::SmartRtl as u8, 3);
    assert_eq!(FailsafeAction::SmartRtlLand as u8, 4);
    assert_eq!(FailsafeAction::Terminate as u8, 5);
    assert_eq!(FailsafeAction::AutoDoLandStart as u8, 6);
    assert_eq!(FailsafeAction::BrakeLand as u8, 7);

    assert_eq!(FailsafeOption::RcContinueIfAuto as u32, 1);
    assert_eq!(FailsafeOption::GcsContinueIfAuto as u32, 2);
    assert_eq!(FailsafeOption::RcContinueIfGuided as u32, 4);
    assert_eq!(FailsafeOption::ContinueIfLanding as u32, 8);
    assert_eq!(FailsafeOption::GcsContinueIfPilotControl as u32, 16);
    assert_eq!(FailsafeOption::ReleaseGripper as u32, 32);
    assert!(failsafe_option(8, FailsafeOption::ContinueIfLanding));
    assert!(!failsafe_option(0, FailsafeOption::ContinueIfLanding));
}

#[test]
fn radio_and_gcs_param_tables_differ_on_unknown() {
    assert_eq!(radio_param_action(0), FailsafeAction::None);
    assert_eq!(radio_param_action(1), FailsafeAction::Rtl);
    assert_eq!(radio_param_action(2), FailsafeAction::Rtl);
    assert_eq!(radio_param_action(3), FailsafeAction::Land);
    assert_eq!(radio_param_action(4), FailsafeAction::SmartRtl);
    assert_eq!(radio_param_action(5), FailsafeAction::SmartRtlLand);
    assert_eq!(radio_param_action(6), FailsafeAction::AutoDoLandStart);
    assert_eq!(radio_param_action(7), FailsafeAction::BrakeLand);
    assert_eq!(radio_param_action(99), FailsafeAction::Land);

    assert_eq!(gcs_param_action(0), FailsafeAction::None);
    assert_eq!(gcs_param_action(1), FailsafeAction::Rtl);
    assert_eq!(gcs_param_action(3), FailsafeAction::SmartRtl);
    assert_eq!(gcs_param_action(5), FailsafeAction::Land);
    assert_eq!(gcs_param_action(99), FailsafeAction::Rtl);
}

fn gcs_seen(enable: u8, seen_at_ms: u32, now_ms: u32, already: bool) -> GcsFailsafeInputs {
    GcsFailsafeInputs {
        enable,
        now_ms,
        last_seen_ms: seen_at_ms,
        timeout_s: FS_GCS_TIMEOUT_DEFAULT_S,
        already_gcs: already,
    }
}

#[test]
fn gcs_check_disabled_or_never_seen_holds() {
    let disabled = GcsFailsafeInputs {
        enable: FsGcsEnable::Disabled as u8,
        now_ms: 60_000,
        last_seen_ms: 1_000,
        timeout_s: FS_GCS_TIMEOUT_DEFAULT_S,
        already_gcs: false,
    };
    assert_eq!(failsafe_gcs_check(&disabled), GcsFailsafeEdge::Hold);

    let never = GcsFailsafeInputs {
        enable: FsGcsEnable::AlwaysRtl as u8,
        now_ms: 60_000,
        last_seen_ms: 0,
        timeout_s: FS_GCS_TIMEOUT_DEFAULT_S,
        already_gcs: false,
    };
    assert_eq!(failsafe_gcs_check(&never), GcsFailsafeEdge::Hold);
}

#[test]
fn gcs_check_trips_only_after_strict_timeout() {
    assert_eq!(gcs_timeout_ms(FS_GCS_TIMEOUT_DEFAULT_S), 5_000);

    let fresh = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 2_000, 2_000, false);
    assert_eq!(failsafe_gcs_check(&fresh), GcsFailsafeEdge::Hold);

    // Equality is a Hold — upstream uses `<` and `>`, not `>=`.
    let at_deadline = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 2_000, 2_000 + 5_000, false);
    assert_eq!(failsafe_gcs_check(&at_deadline), GcsFailsafeEdge::Hold);

    let past = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 2_000, 2_000 + 5_001, false);
    assert_eq!(failsafe_gcs_check(&past), GcsFailsafeEdge::Trigger);

    let already = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 2_000, 2_000 + 5_001, true);
    assert_eq!(failsafe_gcs_check(&already), GcsFailsafeEdge::Hold);
}

#[test]
fn gcs_check_recovers_when_heartbeat_returns() {
    let recovered = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 10_000, 10_100, true);
    assert_eq!(failsafe_gcs_check(&recovered), GcsFailsafeEdge::Recover);

    let still_healthy = gcs_seen(FsGcsEnable::AlwaysRtl as u8, 10_000, 10_100, false);
    assert_eq!(failsafe_gcs_check(&still_healthy), GcsFailsafeEdge::Hold);
}

#[test]
fn radio_on_event_default_is_rtl_then_overrides() {
    let default = RadioFailsafeInputs::default();
    assert_eq!(failsafe_radio_on_event(&default), FailsafeAction::Rtl);

    let disarm = RadioFailsafeInputs {
        should_disarm: true,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(failsafe_radio_on_event(&disarm), FailsafeAction::None);

    let batt_land = RadioFailsafeInputs {
        is_landing: true,
        battery_requires_land: true,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(failsafe_radio_on_event(&batt_land), FailsafeAction::Land);

    let continue_land = RadioFailsafeInputs {
        is_landing: true,
        fs_options: FailsafeOption::ContinueIfLanding as u32,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(
        failsafe_radio_on_event(&continue_land),
        FailsafeAction::Land
    );

    let continue_auto = RadioFailsafeInputs {
        mode_is_auto: true,
        fs_options: FailsafeOption::RcContinueIfAuto as u32,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(
        failsafe_radio_on_event(&continue_auto),
        FailsafeAction::None
    );

    let continue_guided = RadioFailsafeInputs {
        in_guided_mode: true,
        fs_options: FailsafeOption::RcContinueIfGuided as u32,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(
        failsafe_radio_on_event(&continue_guided),
        FailsafeAction::None
    );

    let brake = RadioFailsafeInputs {
        failsafe_throttle: FsThrEnable::BrakeOrLand as u8,
        ..RadioFailsafeInputs::default()
    };
    assert_eq!(failsafe_radio_on_event(&brake), FailsafeAction::BrakeLand);
}

#[test]
fn gcs_on_event_disarmed_wins_and_pilot_control_continues() {
    let default = GcsFailsafeActionInputs::default();
    assert_eq!(failsafe_gcs_on_event(&default), FailsafeAction::Rtl);

    let disarmed = GcsFailsafeActionInputs {
        armed: false,
        should_disarm: true,
        ..GcsFailsafeActionInputs::default()
    };
    assert_eq!(failsafe_gcs_on_event(&disarmed), FailsafeAction::None);

    let continue_pilot = GcsFailsafeActionInputs {
        is_autopilot: false,
        fs_options: FailsafeOption::GcsContinueIfPilotControl as u32,
        ..GcsFailsafeActionInputs::default()
    };
    assert_eq!(failsafe_gcs_on_event(&continue_pilot), FailsafeAction::None);

    let autopilot_stays = GcsFailsafeActionInputs {
        is_autopilot: true,
        fs_options: FailsafeOption::GcsContinueIfPilotControl as u32,
        ..GcsFailsafeActionInputs::default()
    };
    assert_eq!(failsafe_gcs_on_event(&autopilot_stays), FailsafeAction::Rtl);

    let continue_auto = GcsFailsafeActionInputs {
        mode_is_auto: true,
        is_autopilot: true,
        fs_options: FailsafeOption::GcsContinueIfAuto as u32,
        ..GcsFailsafeActionInputs::default()
    };
    assert_eq!(failsafe_gcs_on_event(&continue_auto), FailsafeAction::None);
}

#[test]
fn should_disarm_matches_upstream_mode_groups() {
    assert!(should_disarm_on_failsafe(
        true, MODE_AUTO, false, false, true
    ));
    assert!(should_disarm_on_failsafe(
        false,
        MODE_STABILIZE,
        true,
        false,
        true
    ));
    assert!(should_disarm_on_failsafe(
        false, MODE_ACRO, false, true, true
    ));
    assert!(!should_disarm_on_failsafe(
        false,
        MODE_STABILIZE,
        false,
        false,
        true
    ));
    assert!(should_disarm_on_failsafe(
        false, MODE_AUTO, false, true, false
    ));
    assert!(!should_disarm_on_failsafe(
        false, MODE_AUTO, false, true, true
    ));
    assert!(!should_disarm_on_failsafe(
        false,
        MODE_AUTO_RTL,
        false,
        false,
        false
    ));
    assert!(should_disarm_on_failsafe(false, 5, false, true, true));
    assert!(!should_disarm_on_failsafe(false, 5, true, false, false));
}
