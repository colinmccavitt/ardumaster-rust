//! `ModeLoiter` leftovers: `init` seats AC_Loiter; `run` ticks or re-seats it.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_althold::AltHoldVertical;
use ap_copter::mode_loiter::{
    loiter_init, loiter_run, LoiterInitView, LoiterNavAction, LoiterRunView, MODE_NUMBER_LOITER,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_yaw_rate_rads;
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use ap_wpnav::{InitTargetContext, Loiter};

#[test]
fn mode_number_is_loiter() {
    assert_eq!(MODE_NUMBER_LOITER, 5);
}

#[test]
fn init_seats_ac_loiter_and_starts_d_only_when_inactive() {
    let mut loiter = Loiter::new();
    let view = LoiterInitView {
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
    let cold = loiter_init(false, &mut loiter, &view);
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
    let hot = loiter_init(true, &mut hot_loiter, &hot_view);
    assert!(!hot.init_d_controller);
    assert!(hot.ok);
    assert!(hot.init_target.need_ne_relax_velocity_controller);
}

#[test]
fn flying_ticks_ac_loiter_update_and_uses_thrust_vector_attitude() {
    let mut loiter = Loiter::new();
    let mut view = LoiterRunView::flying();
    view.target_climb_rate_ms = 1.0;
    view.roll_in_norm = 0.3;
    view.yaw_in_norm = -0.4;
    let out = loiter_run(&mut loiter, &view);

    assert_eq!(out.state, AltHoldModeState::Flying);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert_eq!(out.vertical, AltHoldVertical::ClimbRate);
    assert_eq!(out.nav, LoiterNavAction::Update);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(!out.reset_yaw_target_and_rate);
    assert!(!out.start_takeoff);
    assert!(!out.soften_for_landing);
    assert!(out.set_max_speed_accel);
    assert!(out.set_pilot_desired_acceleration);
    assert!(out.input_thrust_vector_rate_heading);
    assert!(!out.slew_yaw);
    assert!(out.update_d_controller);
    assert!(out.init_target.is_none());
    let update = out.update.expect("flying ticks Loiter::update");
    assert!(update.need_calc_desired_velocity);
    assert!(update.need_ne_update_controller);
    assert!(update.need_set_pos_vel_accel_ne);
    assert_eq!(out.target_climb_rate_ms.to_bits(), 1.0f32.to_bits());

    let angle_max = Loiter::new().get_angle_max_rad(
        view.attitude_lean_angle_max_rad,
        view.pos_lean_angle_max_rad,
    );
    let (roll, pitch) =
        pilot_desired_lean_angles_rad(0.3, 0.0, angle_max, view.althold_lean_angle_max_rad, true);
    let yaw = pilot_desired_yaw_rate_rads(-0.4, view.yaw_rate_degs, view.yaw_expo, true);
    assert_eq!(out.target_roll_rad.to_bits(), roll.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), pitch.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), yaw.to_bits());
    // Loiter's default angle max is 2/3 of the attitude/pos limit, not the
    // attitude limit AltHold uses — that is the position-mode leftover.
    assert!(angle_max < view.attitude_lean_angle_max_rad);
}

#[test]
fn motor_stopped_reseats_ac_loiter_and_resets_yaw_rate() {
    let mut loiter = Loiter::new();
    let mut view = LoiterRunView::flying();
    view.armed = false;
    view.spool_state = SpoolState::ShutDown;
    let out = loiter_run(&mut loiter, &view);
    assert_eq!(out.state, AltHoldModeState::MotorStopped);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::ShutDown));
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(out.reset_yaw_target_and_rate);
    assert!(out.reset_yaw_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
    assert_eq!(out.nav, LoiterNavAction::InitTarget);
    let init = out
        .init_target
        .expect("stopped reseats Loiter::init_target");
    assert!(init.need_ne_relax_velocity_controller);
    assert!(out.update.is_none());
    assert!(out.update_d_controller);
}

#[test]
fn maybe_landed_softens_ac_loiter() {
    let mut loiter = Loiter::new();
    let mut view = LoiterRunView::flying();
    view.land_complete_maybe = true;
    let out = loiter_run(&mut loiter, &view);
    assert!(out.soften_for_landing);
    assert_eq!(out.nav, LoiterNavAction::Update);
}

#[test]
fn takeoff_starts_the_helper_and_ticks_update() {
    let mut loiter = Loiter::new();
    let mut view = LoiterRunView::flying();
    view.land_complete = true;
    view.auto_armed = true;
    view.target_climb_rate_ms = 1.0;
    view.takeoff_running = false;
    view.takeoff_alt_m = 25.0;
    let start = loiter_run(&mut loiter, &view);
    assert_eq!(start.state, AltHoldModeState::Takeoff);
    assert_eq!(start.vertical, AltHoldVertical::Takeoff);
    assert_eq!(start.nav, LoiterNavAction::Update);
    assert!(start.start_takeoff);
    assert_eq!(start.takeoff_start_alt_m.to_bits(), 10.0f32.to_bits());
    assert!(
        start
            .update
            .expect("takeoff ticks update")
            .need_ne_update_controller
    );
}
