//! `ModeFlowHold` init leftover, upstream `ArduCopter/mode_flowhold.cpp`.

use ap_copter::mode_flowhold::{
    flowhold_enabled, flowhold_has_user_takeoff, flowhold_init, flowhold_mode_flags,
    FlowHoldInitView, FLOWHOLD_BRAKE_RATE_DPS_DEFAULT, FLOWHOLD_FILTER_HZ_DEFAULT,
    FLOWHOLD_FLOW_MAX_DEFAULT, FLOWHOLD_HEIGHT_MAX_M, FLOWHOLD_HEIGHT_MIN_M,
    FLOWHOLD_QUAL_MIN_DEFAULT, MODE_NUMBER_FLOWHOLD,
};

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
