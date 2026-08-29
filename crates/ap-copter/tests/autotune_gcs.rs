//! AutoTune GCS leftover, upstream `AC_AutoTune::update_gcs` /
//! `do_gcs_announcements` / `report_final_gains`.

use ap_copter::autotune_gcs::{
    announce_percent, do_gcs_announcements, do_post_test_gcs_announcements, get_axis_name,
    get_tune_type_name, report_final_gains, send_step_string, testing_axis_piece, update_gcs,
    GcsAnnounceView, ReportGainsView, UpdateGcsKind, AUTOTUNE_ANNOUNCE_INTERVAL_MS,
    AUTOTUNE_ANNOUNCE_PERCENT_STEP, MAV_SEVERITY_CRITICAL, MAV_SEVERITY_INFO, MAV_SEVERITY_NOTICE,
    TEXT_ANGLE_P_FAILED, TEXT_FAILED, TEXT_FAILED_TO_LEVEL, TEXT_MIN_RATE_D, TEXT_MUST_BE_COMPLETE,
    TEXT_PILOT_OVERRIDES_ACTIVE, TEXT_RATE_D_FAILED, TEXT_RATE_P_FAILED, TEXT_SAVED_VERB,
    TEXT_STARTED, TEXT_STEP_ABORTING, TEXT_STEP_LEVELING, TEXT_STEP_PILOT_OVERRIDE,
    TEXT_STEP_TESTING, TEXT_STEP_UNKNOWN, TEXT_STEP_UPDATING, TEXT_STOPPED, TEXT_SUCCESS,
    TEXT_TESTING_END, TEXT_TESTING_VERB, TEXT_TWITCH_SIZE_FAILED,
};
use ap_copter::autotune_load_save::{AUTOTUNE_PI_RATIO_FINAL, AUTOTUNE_YAW_PI_RATIO_FINAL};
use ap_copter::mode_autotune::{
    mode_autotune_run, AutoTuneRunView, AxisType, Step, TuneType, AUTOTUNE_AXIS_BITMASK_DEFAULT,
    AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_ROLL, AUTOTUNE_AXIS_BITMASK_YAW,
    AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_FAILED, AUTOTUNE_MESSAGE_SAVED_GAINS,
    AUTOTUNE_MESSAGE_STARTED, AUTOTUNE_MESSAGE_STOPPED, AUTOTUNE_MESSAGE_SUCCESS,
    AUTOTUNE_MESSAGE_TESTING, AUTOTUNE_MESSAGE_TESTING_END, AUTOTUNE_SUCCESS_COUNT,
};
use ap_math::scalar::{rad_to_cd, radians};

fn almost(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-4, "{a} != {b}");
}

#[test]
fn constants_match_upstream() {
    assert_eq!(AUTOTUNE_ANNOUNCE_INTERVAL_MS, 2000);
    assert_eq!(AUTOTUNE_SUCCESS_COUNT, 4);
    assert_eq!(AUTOTUNE_ANNOUNCE_PERCENT_STEP, 25);
    assert_eq!(MAV_SEVERITY_CRITICAL, 2);
    assert_eq!(MAV_SEVERITY_NOTICE, 5);
    assert_eq!(MAV_SEVERITY_INFO, 6);
    almost(AUTOTUNE_PI_RATIO_FINAL, 1.0);
    almost(AUTOTUNE_YAW_PI_RATIO_FINAL, 0.1);
}

#[test]
fn axis_and_tune_type_names() {
    assert_eq!(get_axis_name(AxisType::Roll), "Roll");
    assert_eq!(get_axis_name(AxisType::Pitch), "Pitch");
    assert_eq!(get_axis_name(AxisType::Yaw), "Yaw(E)");
    assert_eq!(get_axis_name(AxisType::YawD), "Yaw(D)");
    assert_eq!(get_tune_type_name(TuneType::RateDUp), "Rate D Up");
    assert_eq!(get_tune_type_name(TuneType::RateDDown), "Rate D Down");
    assert_eq!(get_tune_type_name(TuneType::RatePUp), "Rate P Up");
    assert_eq!(get_tune_type_name(TuneType::RateFfUp), "Rate FF Up");
    assert_eq!(get_tune_type_name(TuneType::AnglePUp), "Angle P Up");
    assert_eq!(get_tune_type_name(TuneType::AnglePDown), "Angle P Down");
    assert_eq!(get_tune_type_name(TuneType::MaxGains), "Find Max Gains");
    assert_eq!(
        get_tune_type_name(TuneType::TuneCheck),
        "Check Tune Frequency Response"
    );
    assert_eq!(get_tune_type_name(TuneType::TuneComplete), "Tune Complete");
}

#[test]
fn announce_percent_is_counter_times_25() {
    assert_eq!(announce_percent(0), 0);
    assert_eq!(announce_percent(1), 25);
    assert_eq!(announce_percent(2), 50);
    assert_eq!(announce_percent(3), 75);
    assert_eq!(announce_percent(4), 100);
    assert_eq!(announce_percent(-1), 0);
}

#[test]
fn update_gcs_static_bodies() {
    let started = update_gcs(AUTOTUNE_MESSAGE_STARTED, 0).unwrap();
    assert_eq!(started.severity, MAV_SEVERITY_INFO);
    assert_eq!(started.kind, UpdateGcsKind::Started);
    assert_eq!(started.text(), Some(TEXT_STARTED));

    let stopped = update_gcs(AUTOTUNE_MESSAGE_STOPPED, 0).unwrap();
    assert_eq!(stopped.severity, MAV_SEVERITY_INFO);
    assert_eq!(stopped.text(), Some(TEXT_STOPPED));

    let success = update_gcs(AUTOTUNE_MESSAGE_SUCCESS, 0).unwrap();
    assert_eq!(success.severity, MAV_SEVERITY_NOTICE);
    assert_eq!(success.text(), Some(TEXT_SUCCESS));

    let failed = update_gcs(AUTOTUNE_MESSAGE_FAILED, 0).unwrap();
    assert_eq!(failed.severity, MAV_SEVERITY_NOTICE);
    assert_eq!(failed.text(), Some(TEXT_FAILED));

    let end = update_gcs(AUTOTUNE_MESSAGE_TESTING_END, 0).unwrap();
    assert_eq!(end.severity, MAV_SEVERITY_NOTICE);
    assert_eq!(end.text(), Some(TEXT_TESTING_END));

    assert!(update_gcs(99, 0).is_none());
}

#[test]
fn update_gcs_testing_and_saved_use_axes_completed() {
    let mask = AUTOTUNE_AXIS_BITMASK_DEFAULT;
    let testing = update_gcs(AUTOTUNE_MESSAGE_TESTING, mask).unwrap();
    assert_eq!(testing.severity, MAV_SEVERITY_NOTICE);
    assert_eq!(testing.kind, UpdateGcsKind::Testing);
    assert_eq!(testing.gains_verb(), Some(TEXT_TESTING_VERB));
    assert!(testing.roll && testing.pitch && testing.yaw);
    assert!(!testing.yaw_d);

    let saved = update_gcs(
        AUTOTUNE_MESSAGE_SAVED_GAINS,
        mask | AUTOTUNE_AXIS_BITMASK_YAW_D,
    )
    .unwrap();
    assert_eq!(saved.kind, UpdateGcsKind::SavedGains);
    assert_eq!(saved.gains_verb(), Some(TEXT_SAVED_VERB));
    assert!(saved.yaw_d);
}

#[test]
fn testing_axis_suffix_keeps_roll_pitch_spaces() {
    let all = AUTOTUNE_AXIS_BITMASK_ROLL
        | AUTOTUNE_AXIS_BITMASK_PITCH
        | AUTOTUNE_AXIS_BITMASK_YAW
        | AUTOTUNE_AXIS_BITMASK_YAW_D;
    assert_eq!(testing_axis_piece(AUTOTUNE_AXIS_BITMASK_ROLL, all), "Roll ");
    assert_eq!(
        testing_axis_piece(AUTOTUNE_AXIS_BITMASK_PITCH, all),
        "Pitch "
    );
    assert_eq!(testing_axis_piece(AUTOTUNE_AXIS_BITMASK_YAW, all), "Yaw(E)");
    assert_eq!(
        testing_axis_piece(AUTOTUNE_AXIS_BITMASK_YAW_D, all),
        "Yaw(D)"
    );
    assert_eq!(testing_axis_piece(AUTOTUNE_AXIS_BITMASK_ROLL, 0), "");
}

#[test]
fn send_step_string_matches_upstream() {
    assert_eq!(
        send_step_string(true, Step::ExecutingTest),
        TEXT_STEP_PILOT_OVERRIDE
    );
    assert_eq!(
        send_step_string(false, Step::WaitingForLevel),
        TEXT_STEP_LEVELING
    );
    assert_eq!(
        send_step_string(false, Step::UpdateGains),
        TEXT_STEP_UPDATING
    );
    assert_eq!(send_step_string(false, Step::Abort), TEXT_STEP_ABORTING);
    assert_eq!(
        send_step_string(false, Step::ExecutingTest),
        TEXT_STEP_TESTING
    );
    assert_eq!(TEXT_STEP_UNKNOWN, "AutoTune: unknown step");
}

#[test]
fn do_gcs_announcements_skips_inside_interval() {
    let mut view = GcsAnnounceView::typical();
    view.last_announce_ms = view.now_ms - (AUTOTUNE_ANNOUNCE_INTERVAL_MS - 1);
    let out = do_gcs_announcements(&view);
    assert!(!out.sent);
    assert_eq!(out.last_announce_ms, view.last_announce_ms);
}

#[test]
fn do_gcs_announcements_sends_at_interval() {
    let mut view = GcsAnnounceView::typical();
    view.last_announce_ms = view.now_ms - AUTOTUNE_ANNOUNCE_INTERVAL_MS;
    view.success_counter = 2;
    view.axis = AxisType::Pitch;
    view.tune_type = TuneType::RatePUp;
    let out = do_gcs_announcements(&view);
    assert!(out.sent);
    assert_eq!(out.last_announce_ms, view.now_ms);
    assert_eq!(out.severity, MAV_SEVERITY_INFO);
    assert_eq!(out.axis_name, "Pitch");
    assert_eq!(out.tune_type_name, "Rate P Up");
    assert_eq!(out.percent, 50);
}

#[test]
fn do_gcs_announcements_wraps_millis() {
    let mut view = GcsAnnounceView::typical();
    view.now_ms = 100;
    view.last_announce_ms = u32::MAX - 2_100;
    let out = do_gcs_announcements(&view);
    assert!(out.sent);
    assert_eq!(out.last_announce_ms, 100);
}

#[test]
fn multi_post_test_announce_is_noop() {
    assert!(!do_post_test_gcs_announcements());
}

#[test]
fn report_roll_uses_final_i_ratio() {
    let mut view = ReportGainsView::typical();
    view.tune_accel_radss = radians(40.0);
    let out = report_final_gains(&view);
    assert_eq!(out.axis_string, "Roll");
    assert_eq!(out.severity, MAV_SEVERITY_NOTICE);
    almost(out.rate_p, 0.15);
    almost(out.rate_i, 0.15);
    almost(out.rate_d, 0.004);
    almost(out.angle_p, 4.5);
    almost(out.max_accel_cd, rad_to_cd(radians(40.0)));
    assert!((out.max_accel_cd - 4000.0).abs() < 0.01);
}

#[test]
fn report_yaw_e_zeros_d_and_shrinks_i() {
    let mut view = ReportGainsView::typical();
    view.axis = AxisType::Yaw;
    view.tune_rd = 0.012;
    let out = report_final_gains(&view);
    assert_eq!(out.axis_string, "Yaw(E)");
    almost(out.rate_i, 0.015);
    almost(out.rate_d, 0.0);
}

#[test]
fn report_yaw_d_keeps_d() {
    let mut view = ReportGainsView::typical();
    view.axis = AxisType::YawD;
    view.tune_rd = 0.012;
    let out = report_final_gains(&view);
    assert_eq!(out.axis_string, "Yaw(D)");
    almost(out.rate_i, 0.015);
    almost(out.rate_d, 0.012);
}

#[test]
fn report_pitch_matches_roll_ratios() {
    let mut view = ReportGainsView::typical();
    view.axis = AxisType::Pitch;
    let out = report_final_gains(&view);
    assert_eq!(out.axis_string, "Pitch");
    almost(out.rate_i, view.tune_rp * AUTOTUNE_PI_RATIO_FINAL);
}

#[test]
fn other_multi_gcs_texts_are_catalogued() {
    assert_eq!(TEXT_PILOT_OVERRIDES_ACTIVE, "AutoTune: pilot overrides active");
    assert_eq!(
        TEXT_FAILED_TO_LEVEL,
        "AutoTune: Failed to level, please tune manually"
    );
    assert_eq!(
        TEXT_MUST_BE_COMPLETE,
        "AutoTune: must be complete to test gains"
    );
    assert_eq!(
        TEXT_TWITCH_SIZE_FAILED,
        "AutoTune: Twitch Size Determination Failed"
    );
    assert_eq!(TEXT_MIN_RATE_D, "AutoTune: Min Rate D limit reached");
    assert_eq!(
        TEXT_RATE_D_FAILED,
        "AutoTune: Rate D Gain Determination Failed"
    );
    assert_eq!(
        TEXT_RATE_P_FAILED,
        "AutoTune: Rate P Gain Determination Failed"
    );
    assert_eq!(
        TEXT_ANGLE_P_FAILED,
        "AutoTune: Angle P Gain Determination Failed"
    );
}

#[test]
fn run_still_flags_do_gcs_announcements() {
    let view = AutoTuneRunView::typical();
    let out = mode_autotune_run(&view);
    assert!(out.do_gcs_announcements);
    let announce = do_gcs_announcements(&GcsAnnounceView {
        now_ms: view.now_ms,
        last_announce_ms: 0,
        axis: view.axis,
        tune_type: view.tune_type,
        success_counter: view.success_counter,
    });
    assert!(announce.sent);
    assert_eq!(announce.axis_name, "Roll");
    assert_eq!(announce.tune_type_name, "Rate D Up");
    assert_eq!(announce.percent, 0);
}

#[test]
fn init_started_maps_to_update_gcs() {
    let gcs = update_gcs(AUTOTUNE_MESSAGE_STARTED, 0).unwrap();
    assert_eq!(gcs.text(), Some("AutoTune: Started"));
}
