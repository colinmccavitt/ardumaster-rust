//! `ModeZigZag` init leftover, upstream `ArduCopter/mode_zigzag.cpp`.

use ap_copter::mode_zigzag::{
    zigzag_has_user_takeoff, zigzag_init, zigzag_mode_flags, ZigZagAutoStage, ZigZagInitView,
    ZigZagStage, MODE_NUMBER_ZIGZAG,
};
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_wpnav::{InitTargetContext, Loiter};

#[test]
fn zigzag_number_is_twenty_four() {
    assert_eq!(MODE_NUMBER_ZIGZAG, 24);
    assert_eq!(zigzag_mode_flags().mode_number, MODE_NUMBER_ZIGZAG);
}

#[test]
fn zigzag_flags_are_position_autopilot() {
    let flags = zigzag_mode_flags();
    assert!(flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(flags.allows_arming);
    assert!(flags.is_autopilot);
}

#[test]
fn user_takeoff_ignores_must_navigate() {
    assert!(zigzag_has_user_takeoff(false));
    assert!(zigzag_has_user_takeoff(true));
}

#[test]
fn init_seats_ac_loiter_and_starts_d_only_when_inactive() {
    let mut loiter = Loiter::new();
    let view = ZigZagInitView {
        roll_in_norm: 0.25,
        pitch_in_norm: -0.1,
        has_valid_input: true,
        attitude_lean_angle_max_rad: 0.523_598_8,
        pos_lean_angle_max_rad: 0.523_598_8,
        althold_lean_angle_max_rad: 0.523_598_8,
        d_is_active: false,
        speed_dn_ms: 1.5,
        speed_up_ms: 2.5,
        accel_d_mss: 2.0,
        init_target_ctx: InitTargetContext {
            lean_angle_max_rad: 0.523_598_8,
            accel_target_ne_mss: ap_math::vector2::Vector2f::new(0.4, -0.2),
            roll_rad: 0.05,
            pitch_rad: -0.02,
        },
    };
    let cold = zigzag_init(false, &mut loiter, &view);
    assert!(cold.ok);
    assert!(cold.update_simple_mode);
    assert!(cold.set_pilot_desired_acceleration);
    assert!(cold.init_d_controller);
    assert!(cold.set_max_speed_accel);
    assert!(cold.set_correction_speed_accel);
    assert!(cold.init_target.need_ne_relax_velocity_controller);
    assert!(!cold.init_target.need_ne_init_controller_stopping_point);
    assert!(cold.init_target.pos_desired_ne_m.is_none());
    assert_eq!(cold.speed_dn_ms.to_bits(), 1.5f32.to_bits());
    assert_eq!(cold.speed_up_ms.to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.accel_d_mss.to_bits(), 2.0f32.to_bits());

    let angle_max = Loiter::new().get_angle_max_rad(0.523_598_8, 0.523_598_8);
    let (roll, pitch) = pilot_desired_lean_angles_rad(0.25, -0.1, angle_max, 0.523_598_8, true);
    assert_eq!(cold.target_roll_rad.to_bits(), roll.to_bits());
    assert_eq!(cold.target_pitch_rad.to_bits(), pitch.to_bits());

    let mut hot_loiter = Loiter::new();
    let mut hot_view = view;
    hot_view.d_is_active = true;
    let hot = zigzag_init(true, &mut hot_loiter, &hot_view);
    assert!(!hot.init_d_controller);
    assert!(hot.ok);
    assert!(hot.init_target.need_ne_relax_velocity_controller);
}

#[test]
fn init_forgets_ab_and_runs_init_auto() {
    let mut loiter = Loiter::new();
    let out = zigzag_init(false, &mut loiter, &ZigZagInitView::typical());
    assert_eq!(out.stage, ZigZagStage::StoringPoints);
    assert!(out.dest_a_cleared);
    assert!(out.dest_b_cleared);
    assert!(!out.is_auto);
    assert_eq!(out.auto_stage, ZigZagAutoStage::Manual);
    assert_eq!(out.line_count, 0);
    assert!(!out.is_suspended);
}

#[test]
fn ignore_checks_cannot_change_init() {
    let mut a = Loiter::new();
    let mut b = Loiter::new();
    let view = ZigZagInitView::typical();
    let refused = zigzag_init(false, &mut a, &view);
    let ignored = zigzag_init(true, &mut b, &view);
    assert_eq!(refused, ignored);
}
