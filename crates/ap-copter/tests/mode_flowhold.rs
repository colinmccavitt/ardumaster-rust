//! `ModeFlowHold` init / run leftover, upstream `ArduCopter/mode_flowhold.cpp`.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_althold::AltHoldVertical;
use ap_copter::mode_flowhold::{
    flowhold_enabled, flowhold_has_user_takeoff, flowhold_init, flowhold_mode_flags, flowhold_run,
    FlowHoldInitView, FlowHoldRunView, FLOWHOLD_BRAKE_RATE_DPS_DEFAULT, FLOWHOLD_FILTER_HZ_DEFAULT,
    FLOWHOLD_FLOW_MAX_DEFAULT, FLOWHOLD_HEIGHT_MAX_M, FLOWHOLD_HEIGHT_MIN_M,
    FLOWHOLD_QUALITY_FILTER, FLOWHOLD_QUAL_MIN_DEFAULT, MODE_NUMBER_FLOWHOLD,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn flowhold_number_is_twenty_two() {
    assert_eq!(MODE_NUMBER_FLOWHOLD, 22);
    assert_eq!(flowhold_mode_flags().mode_number, MODE_NUMBER_FLOWHOLD);
}

#[test]
fn flowhold_flags_are_rate_mode_without_gps() {
    let flags = flowhold_mode_flags();
    assert!(!flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(flags.allows_arming);
    assert!(!flags.is_autopilot);
    assert!(flags.allows_flip);
}

#[test]
fn user_takeoff_is_in_place_only() {
    assert!(flowhold_has_user_takeoff(false));
    assert!(!flowhold_has_user_takeoff(true));
}

#[test]
fn enabled_is_optflow_enabled_only() {
    assert!(flowhold_enabled(true));
    assert!(!flowhold_enabled(false));
}

#[test]
fn constructor_defaults_match_upstream() {
    assert_eq!(FLOWHOLD_HEIGHT_MIN_M.to_bits(), 0.1f32.to_bits());
    assert_eq!(FLOWHOLD_HEIGHT_MAX_M.to_bits(), 3.0f32.to_bits());
    assert_eq!(FLOWHOLD_FLOW_MAX_DEFAULT.to_bits(), 0.6f32.to_bits());
    assert_eq!(FLOWHOLD_FILTER_HZ_DEFAULT.to_bits(), 5.0f32.to_bits());
    assert_eq!(FLOWHOLD_QUAL_MIN_DEFAULT, 10);
    assert_eq!(FLOWHOLD_BRAKE_RATE_DPS_DEFAULT, 8);
}

#[test]
fn init_starts_d_only_when_inactive() {
    let view = FlowHoldInitView {
        optflow_enabled: true,
        optflow_healthy: true,
        d_is_active: false,
        speed_dn_ms: 1.5,
        speed_up_ms: 2.5,
        accel_d_mss: 2.0,
        loop_rate_hz: 400.0,
        flow_filter_hz: 5.0,
        pos_estimate_u_m: 1.25,
    };
    let cold = flowhold_init(false, &view);
    assert!(cold.ok);
    assert!(cold.init_d_controller);
    assert!(cold.set_max_speed_accel);
    assert!(cold.set_correction_speed_accel);
    assert!(cold.set_filter_cutoff);
    assert!(cold.reset_i);
    assert!(cold.set_dt);
    assert_eq!(cold.speed_dn_ms.unwrap().to_bits(), 1.5f32.to_bits());
    assert_eq!(cold.speed_up_ms.unwrap().to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.accel_d_mss.unwrap().to_bits(), 2.0f32.to_bits());
    assert_eq!(cold.flow_filter_hz.unwrap().to_bits(), 5.0f32.to_bits());
    assert_eq!(cold.quality_filtered.unwrap().to_bits(), 0.0f32.to_bits());
    assert_eq!(cold.limited, Some(false));
    assert_eq!(cold.dt.unwrap().to_bits(), (1.0f32 / 400.0).to_bits());
    assert_eq!(cold.last_ins_height_m.unwrap().to_bits(), 1.25f32.to_bits());
    assert_eq!(cold.height_offset_m.unwrap().to_bits(), 0.0f32.to_bits());

    let mut hot_view = view;
    hot_view.d_is_active = true;
    let hot = flowhold_init(true, &hot_view);
    assert!(!hot.init_d_controller);
    assert!(hot.ok);
    assert!(hot.set_max_speed_accel);
    assert!(hot.set_correction_speed_accel);
    assert_eq!(hot.last_ins_height_m.unwrap().to_bits(), 1.25f32.to_bits());
}

#[test]
fn disabled_optflow_fails_before_d_writes() {
    let mut view = FlowHoldInitView::typical();
    view.optflow_enabled = false;
    view.d_is_active = false;
    let out = flowhold_init(true, &view);
    assert!(!out.ok);
    assert!(!out.init_d_controller);
    assert!(!out.set_max_speed_accel);
    assert!(!out.set_correction_speed_accel);
    assert!(!out.set_filter_cutoff);
    assert!(!out.reset_i);
    assert!(!out.set_dt);
    assert!(out.speed_dn_ms.is_none());
    assert!(out.last_ins_height_m.is_none());
    assert!(out.height_offset_m.is_none());
}

#[test]
fn unhealthy_optflow_fails_even_when_enabled_and_ignore_checks() {
    let mut view = FlowHoldInitView::typical();
    view.optflow_healthy = false;
    let listed = flowhold_enabled(view.optflow_enabled);
    assert!(listed);
    let out = flowhold_init(true, &view);
    assert!(!out.ok);
    assert!(!out.set_max_speed_accel);
    assert!(out.quality_filtered.is_none());
}

#[test]
fn ignore_checks_cannot_bypass_healthy_gate() {
    let mut view = FlowHoldInitView::typical();
    view.optflow_enabled = false;
    let refused = flowhold_init(false, &view);
    let ignored = flowhold_init(true, &view);
    assert_eq!(refused, ignored);
    assert!(!ignored.ok);
}

#[test]
fn flying_adds_clamped_flow_angles() {
    let mut view = FlowHoldRunView::flying();
    view.flow_angles_rad = (0.1, -0.05);
    view.roll_in_norm = 0.0;
    let out = flowhold_run(&view);
    assert_eq!(out.state, AltHoldModeState::Flying);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert_eq!(out.vertical, AltHoldVertical::ClimbRate);
    assert!(out.update_height_estimate);
    assert!(out.set_max_speed_accel);
    assert!(out.flow_to_angle);
    assert!(!out.stick_input);
    assert!(!out.set_filter_cutoff);
    assert!(!out.reset_flow_i);
    assert!(out.input_euler_angle);
    assert!(out.update_d_controller);
    assert_eq!(out.bf_roll_rad.to_bits(), 0.1f32.to_bits());
    assert_eq!(out.bf_pitch_rad.to_bits(), (-0.05f32).to_bits());
}

#[test]
fn flow_angles_are_clamped_to_half_lean_max_before_add() {
    let mut view = FlowHoldRunView::flying();
    view.lean_angle_max_rad = 0.4;
    view.althold_lean_angle_max_rad = 0.4;
    view.flow_angles_rad = (1.0, -1.0);
    let out = flowhold_run(&view);
    assert!(out.flow_to_angle);
    assert_eq!(out.bf_roll_rad.to_bits(), 0.2f32.to_bits());
    assert_eq!(out.bf_pitch_rad.to_bits(), (-0.2f32).to_bits());
}

#[test]
fn arm_delay_and_low_quality_skip_flow_to_angle() {
    let mut early = FlowHoldRunView::flying();
    early.time_since_arm_ms = 3_000;
    early.flow_angles_rad = (0.2, 0.0);
    let early_out = flowhold_run(&early);
    assert!(!early_out.flow_to_angle);
    assert_eq!(early_out.bf_roll_rad.to_bits(), 0.0f32.to_bits());

    let mut poor = FlowHoldRunView::flying();
    poor.quality_filtered = 0.0;
    poor.optflow_quality = 0.0;
    poor.flow_angles_rad = (0.2, 0.0);
    let poor_out = flowhold_run(&poor);
    assert!(!poor_out.flow_to_angle);
    assert_eq!(poor_out.quality_filtered.to_bits(), 0.0f32.to_bits());
}

#[test]
fn unhealthy_optflow_zeros_quality() {
    let mut view = FlowHoldRunView::flying();
    view.optflow_healthy = false;
    view.quality_filtered = 180.0;
    let out = flowhold_run(&view);
    assert_eq!(out.quality_filtered.to_bits(), 0.0f32.to_bits());
    assert!(!out.flow_to_angle);
}

#[test]
fn quality_filter_is_complementary() {
    let mut view = FlowHoldRunView::flying();
    view.quality_filtered = 100.0;
    view.optflow_quality = 200.0;
    let out = flowhold_run(&view);
    let expected = FLOWHOLD_QUALITY_FILTER * 100.0 + (1.0 - FLOWHOLD_QUALITY_FILTER) * 200.0;
    assert_eq!(out.quality_filtered.to_bits(), expected.to_bits());
}

#[test]
fn motor_stopped_resets_flow_i_and_shuts_down() {
    let mut view = FlowHoldRunView::flying();
    view.armed = false;
    view.spool_state = SpoolState::ShutDown;
    let out = flowhold_run(&view);
    assert_eq!(out.state, AltHoldModeState::MotorStopped);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::ShutDown));
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(out.reset_yaw_target_and_rate);
    assert!(out.reset_flow_i);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
    assert!(out.update_d_controller);
}

#[test]
fn takeoff_sets_unlimited_even_when_the_machine_does_not() {
    let mut view = FlowHoldRunView::flying();
    view.land_complete = true;
    view.auto_armed = true;
    view.target_climb_rate_ms = 1.0;
    view.takeoff_running = false;
    view.takeoff_alt_m = 25.0;
    let out = flowhold_run(&view);
    assert_eq!(out.state, AltHoldModeState::Takeoff);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert!(out.start_takeoff);
    assert_eq!(out.takeoff_start_alt_m.to_bits(), 10.0f32.to_bits());
    assert_eq!(out.vertical, AltHoldVertical::Takeoff);
}

#[test]
fn stick_input_is_control_in_not_norm() {
    let mut view = FlowHoldRunView::flying();
    view.roll_in_norm = 0.0;
    view.roll_control_in = 12;
    let out = flowhold_run(&view);
    assert!(out.stick_input);
}

#[test]
fn filter_cutoff_rewrites_when_the_param_moved() {
    let mut view = FlowHoldRunView::flying();
    view.flow_filter_hz = 8.0;
    view.flow_filter_cutoff_hz = 5.0;
    let out = flowhold_run(&view);
    assert!(out.set_filter_cutoff);
}

#[test]
fn climb_rate_is_clamped_to_pilot_speeds() {
    let mut view = FlowHoldRunView::flying();
    view.target_climb_rate_ms = 10.0;
    view.speed_up_ms = 2.5;
    let out = flowhold_run(&view);
    assert_eq!(out.target_climb_rate_ms.to_bits(), 2.5f32.to_bits());
}
