//! `ModeZigZag` init / run leftover, upstream `ArduCopter/mode_zigzag.cpp`.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_althold::AltHoldVertical;
use ap_copter::mode_loiter::LoiterNavAction;
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::mode_zigzag::{
    zigzag_auto_control, zigzag_has_user_takeoff, zigzag_init, zigzag_is_disarmed_or_landed,
    zigzag_manual_control, zigzag_mode_flags, zigzag_reached_destination, zigzag_return_to_manual,
    zigzag_run, ZigZagAttitude, ZigZagAutoStage, ZigZagDestination, ZigZagDirection,
    ZigZagInitView, ZigZagReachedView, ZigZagRunAction, ZigZagRunView, ZigZagStage,
    MODE_NUMBER_ZIGZAG, ZIGZAG_LINE_INFINITY, ZIGZAG_WP_RADIUS_M,
};
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_motors::spool::DesiredSpoolState;
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

#[test]
fn storing_points_runs_manual_not_auto() {
    let mut loiter = Loiter::new();
    let view = ZigZagRunView::storing_points();
    let out = zigzag_run(&mut loiter, &view);
    assert!(out.set_max_speed_accel);
    assert_eq!(out.direction, ZigZagDirection::Forward);
    assert_eq!(out.line_num, 0);
    assert_eq!(out.stage, ZigZagStage::StoringPoints);
    assert_eq!(out.action, ZigZagRunAction::None);
    assert!(!out.waypoint_complete);
    assert!(out.auto_control.is_none());
    assert!(out.return_to_manual.is_none());
    let manual = out.manual.expect("manual");
    assert_eq!(manual.state, AltHoldModeState::Flying);
    assert_eq!(manual.vertical, AltHoldVertical::ClimbRate);
    assert_eq!(manual.nav, LoiterNavAction::Update);
    assert_eq!(manual.attitude, ZigZagAttitude::EulerRateYaw);
    assert!(manual.update_d_controller);
}

#[test]
fn direction_and_line_num_are_clamped() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::storing_points();
    view.direction = 99;
    view.line_num = -8;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.direction, ZigZagDirection::Left);
    assert_eq!(out.line_num, ZIGZAG_LINE_INFINITY);
}

#[test]
fn auto_disarmed_drops_to_manual_same_tick() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::auto_enroute();
    view.armed = false;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(
        out.action,
        ZigZagRunAction::ReturnToManual {
            maintain_target: false
        }
    );
    assert_eq!(out.stage, ZigZagStage::ManualRegain);
    assert!(!out.is_auto);
    assert!(out.spray_off);
    let rtm = out.return_to_manual.expect("rtm");
    assert!(rtm.applied);
    assert!(!rtm.maintain_target);
    assert!(rtm.clear_pilot_desired_acceleration);
    assert!(out.manual.is_some());
    assert!(out.auto_control.is_none());
}

#[test]
fn auto_enroute_flies_auto_control() {
    let mut loiter = Loiter::new();
    let view = ZigZagRunView::auto_enroute();
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.action, ZigZagRunAction::AutoControl);
    assert_eq!(out.stage, ZigZagStage::Auto);
    assert!(out.is_auto);
    let ac = out.auto_control.expect("ac");
    assert!(ac.wpnav_ok);
    assert!(!ac.return_to_manual);
    assert_eq!(ac.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(out.manual.is_none());
    assert!(!out.waypoint_complete);
}

#[test]
fn wpnav_failure_returns_to_manual_same_tick() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::auto_enroute();
    view.wpnav_ok = false;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.action, ZigZagRunAction::AutoControl);
    assert_eq!(out.stage, ZigZagStage::ManualRegain);
    assert!(out.auto_control.expect("ac").return_to_manual);
    assert!(out.return_to_manual.expect("rtm").applied);
    assert!(out.manual.is_some());
}

#[test]
fn reached_ab_with_lines_left_moves_sideways() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::auto_enroute();
    view.reached = ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 1.0,
        reach_wp_time_ms: 1_000,
        now_ms: 1_000,
        wp_delay_s: 0,
    };
    view.auto_stage = ZigZagAutoStage::AbMoving;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.action, ZigZagRunAction::MoveToSide);
    assert!(out.waypoint_complete);
    assert!(out.spray_off);
    assert_eq!(out.stage, ZigZagStage::Auto);
    assert!(out.manual.is_none());
}

#[test]
fn reached_sideways_moves_to_the_other_ab() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::auto_enroute();
    view.reached = ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 0.5,
        reach_wp_time_ms: 500,
        now_ms: 500,
        wp_delay_s: 0,
    };
    view.auto_stage = ZigZagAutoStage::Sideways;
    view.ab_dest_stored = ZigZagDestination::A;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.action, ZigZagRunAction::SaveOrMoveToOther);
    assert_eq!(out.move_dest, Some(ZigZagDestination::B));
    assert!(out.waypoint_complete);
    assert!(!out.spray_off);
}

#[test]
fn exhausted_lines_init_auto_then_manual() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::auto_enroute();
    view.reached = ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 0.0,
        reach_wp_time_ms: 100,
        now_ms: 100,
        wp_delay_s: 0,
    };
    view.line_num = 2;
    view.line_count = 2;
    let out = zigzag_run(&mut loiter, &view);
    assert_eq!(out.action, ZigZagRunAction::InitAutoThenManual);
    assert_eq!(out.stage, ZigZagStage::ManualRegain);
    assert!(!out.is_auto);
    assert!(out.return_to_manual.expect("rtm").maintain_target);
    assert!(out.manual.is_some());
}

#[test]
fn reached_destination_waits_out_the_delay() {
    let waiting = zigzag_reached_destination(&ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 1.0,
        reach_wp_time_ms: 1_000,
        now_ms: 1_500,
        wp_delay_s: 2,
    });
    assert!(!waiting.reached);
    assert_eq!(waiting.reach_wp_time_ms, 1_000);

    let done = zigzag_reached_destination(&ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 1.0,
        reach_wp_time_ms: 1_000,
        now_ms: 3_000,
        wp_delay_s: 2,
    });
    assert!(done.reached);

    let too_far = zigzag_reached_destination(&ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: ZIGZAG_WP_RADIUS_M + 0.1,
        reach_wp_time_ms: 0,
        now_ms: 5_000,
        wp_delay_s: 0,
    });
    assert!(!too_far.reached);
    assert_eq!(too_far.reach_wp_time_ms, 0);
}

#[test]
fn first_radius_tick_stamps_reach_time() {
    let out = zigzag_reached_destination(&ZigZagReachedView {
        wp_reached: true,
        wp_distance_m: 0.0,
        reach_wp_time_ms: 0,
        now_ms: 42_000,
        wp_delay_s: 0,
    });
    assert!(out.reached);
    assert_eq!(out.reach_wp_time_ms, 42_000);
}

#[test]
fn return_to_manual_is_noop_when_not_auto() {
    let mut loiter = Loiter::new();
    let out = zigzag_return_to_manual(
        &mut loiter,
        true,
        ZigZagStage::StoringPoints,
        false,
        (1.0, 2.0),
        Default::default(),
    );
    assert!(!out.applied);
    assert_eq!(out.stage, ZigZagStage::StoringPoints);
    assert!(out.init_target.is_none());
}

#[test]
fn landed_manual_uses_thrust_vector() {
    let mut loiter = Loiter::new();
    let mut view = ZigZagRunView::storing_points();
    view.land_complete = true;
    view.auto_armed = false;
    view.spool_state = ap_motors::spool::SpoolState::GroundIdle;
    let manual = zigzag_manual_control(&mut loiter, &view);
    assert_eq!(manual.state, AltHoldModeState::LandedGroundIdle);
    assert_eq!(manual.attitude, ZigZagAttitude::ThrustVector);
    assert_eq!(manual.reset_rate_i, RateIReset::Smooth);
    assert_eq!(manual.nav, LoiterNavAction::InitTarget);
}

#[test]
fn is_disarmed_or_landed_matches_upstream() {
    assert!(zigzag_is_disarmed_or_landed(false, true, false));
    assert!(zigzag_is_disarmed_or_landed(true, false, false));
    assert!(zigzag_is_disarmed_or_landed(true, true, true));
    assert!(!zigzag_is_disarmed_or_landed(true, true, false));
}

#[test]
fn auto_control_records_pilot_yaw() {
    let mut view = ZigZagRunView::auto_enroute();
    view.yaw_in_norm = 0.5;
    let ac = zigzag_auto_control(&view);
    assert!(ac.input_euler_angle);
    assert!(ac.update_d_controller);
    assert!(ac.wpnav_ok);
    assert_ne!(ac.target_yaw_rate_rads.to_bits(), 0.0f32.to_bits());
}
