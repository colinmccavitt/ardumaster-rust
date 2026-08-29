//! `ModeGuidedNoGPS` init / run leftover, upstream
//! `ArduCopter/mode_guided_nogps.cpp`.

use ap_copter::mode_guided::{
    guided_angle_control_run, guided_angle_control_start, guided_mode_flags, GuidedAngleControlView,
    GuidedAngleStartView, GuidedSubMode, MODE_NUMBER_GUIDED, MODE_NUMBER_GUIDED_NOGPS,
};
use ap_copter::mode_guided_nogps::{
    guided_nogps_init, guided_nogps_mode_flags, guided_nogps_run,
};

#[test]
fn nogps_number_is_twenty() {
    assert_eq!(MODE_NUMBER_GUIDED_NOGPS, 20);
    assert_ne!(MODE_NUMBER_GUIDED_NOGPS, MODE_NUMBER_GUIDED);
}

#[test]
fn nogps_flags_drop_only_the_position_requirement() {
    let guided = guided_mode_flags();
    let nogps = guided_nogps_mode_flags();
    assert_eq!(nogps.mode_number, MODE_NUMBER_GUIDED_NOGPS);
    assert!(!nogps.requires_position);
    assert_eq!(nogps.has_manual_throttle, guided.has_manual_throttle);
    assert_eq!(nogps.is_autopilot, guided.is_autopilot);
    assert_eq!(nogps.has_user_takeoff, guided.has_user_takeoff);
    assert_eq!(nogps.in_guided_mode, guided.in_guided_mode);
    assert_eq!(
        nogps.requires_terrain_failsafe,
        guided.requires_terrain_failsafe
    );
    assert_eq!(
        nogps.allows_gcs_or_scr_arming_with_throttle_high,
        guided.allows_gcs_or_scr_arming_with_throttle_high
    );
}

#[test]
fn nogps_init_is_angle_start_and_always_succeeds() {
    let view = GuidedAngleStartView::after_init();
    let out = guided_nogps_init(&view, false);
    assert_eq!(out, guided_angle_control_start(&view));
    assert_eq!(out.submode, GuidedSubMode::Angle);

    let ignored = guided_nogps_init(&view, true);
    assert_eq!(ignored, out);
}

#[test]
fn nogps_init_inits_d_when_inactive() {
    let mut view = GuidedAngleStartView::after_init();
    view.d_is_active = false;
    let out = guided_nogps_init(&view, false);
    assert!(out.init_d);
    assert!(!out.init_ne);
}

#[test]
fn nogps_run_is_angle_run() {
    let view = GuidedAngleControlView::after_set_angle();
    assert_eq!(guided_nogps_run(&view), guided_angle_control_run(&view));
}

#[test]
fn nogps_run_has_no_pause_gate() {
    let mut view = GuidedAngleControlView::after_set_angle();
    view.motors_armed = false;
    let out = guided_nogps_run(&view);
    assert_eq!(out, guided_angle_control_run(&view));
    match out.exit {
        ap_copter::mode_guided::GuidedAngleControlExit::Ground { .. } => {}
        other => panic!("expected Ground, got {other:?}"),
    }
}
