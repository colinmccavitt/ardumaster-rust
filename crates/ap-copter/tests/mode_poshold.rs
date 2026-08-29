//! `ModePosHold` leftovers: `init` seats AC_Loiter; `run` mixes pilot / brake / loiter.

use ap_copter::alt_hold::AltHoldModeState;
use ap_copter::mode_althold::AltHoldVertical;
use ap_copter::mode_poshold::{
    mix_controls, poshold_brake_gain, poshold_init, poshold_run, update_brake_angle_from_velocity,
    update_pilot_lean_angle_rad, wind_comp_lean_angles_rad, PosHold, PosHoldInitView,
    PosHoldNavAction, PosHoldRunView, RpMode, MODE_NUMBER_POSHOLD, POSHOLD_BRAKE_RATE_DEFAULT_DEGS,
    POSHOLD_BRAKE_RATE_MIN_DEGS, POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_yaw_rate_rads;
use ap_copter::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{radians, GRAVITY_MSS};
use ap_math::vector2::Vector2f;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use ap_wpnav::{InitTargetContext, Loiter};

fn init_view(land_complete: bool, d_is_active: bool) -> PosHoldInitView {
    PosHoldInitView {
        d_is_active,
        speed_dn_ms: 1.5,
        speed_up_ms: 2.5,
        accel_d_mss: 2.0,
        land_complete,
        brake_rate_degs: POSHOLD_BRAKE_RATE_DEFAULT_DEGS,
        init_target_ctx: InitTargetContext {
            lean_angle_max_rad: 0.523_598_8,
            accel_target_ne_mss: Vector2f::new(0.4, -0.2),
            roll_rad: 0.05,
            pitch_rad: -0.02,
        },
    }
}

#[test]
fn mode_number_is_poshold() {
    assert_eq!(MODE_NUMBER_POSHOLD, 16);
}

#[test]
fn init_landed_starts_in_loiter_and_seats_ac_loiter() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let view = init_view(true, false);
    let out = poshold_init(false, &mut poshold, &mut loiter, &view);

    assert!(out.ok);
    assert_eq!(out.rp_mode, RpMode::Loiter);
    assert_eq!(poshold.roll_mode, RpMode::Loiter);
    assert_eq!(poshold.pitch_mode, RpMode::Loiter);
    assert!(out.zero_pilot_lean);
    assert!(out.init_wind_comp);
    assert!(out.clear_pilot_desired_acceleration);
    assert!(out.init_d_controller);
    assert!(out.set_max_speed_accel);
    assert!(out.set_correction_speed_accel);
    assert!(out.init_target.need_ne_relax_velocity_controller);
    assert!(!out.init_target.need_ne_init_controller_stopping_point);
    assert!(out.init_target.pos_desired_ne_m.is_none());
    assert_eq!(
        out.brake_gain.to_bits(),
        poshold_brake_gain(POSHOLD_BRAKE_RATE_DEFAULT_DEGS).to_bits()
    );
    assert_eq!(poshold.pilot_roll_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(poshold.wind_comp_ne_mss, Vector2f::zero());
}

#[test]
fn init_airborne_starts_in_pilot_override_and_skips_hot_d() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let view = init_view(false, true);
    let out = poshold_init(true, &mut poshold, &mut loiter, &view);

    assert!(out.ok);
    assert!(!out.init_d_controller);
    assert_eq!(out.rp_mode, RpMode::PilotOverride);
    assert_eq!(poshold.roll_mode, RpMode::PilotOverride);
    assert_eq!(poshold.pitch_mode, RpMode::PilotOverride);
}

#[test]
fn flying_with_stick_stays_pilot_override_and_uses_euler_attitude() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let _ = poshold_init(false, &mut poshold, &mut loiter, &init_view(false, true));

    let mut view = PosHoldRunView::flying();
    view.roll_in_norm = 0.4;
    view.yaw_in_norm = -0.3;
    view.target_climb_rate_ms = 1.0;
    let out = poshold_run(&mut poshold, &mut loiter, &view);

    assert_eq!(out.state, AltHoldModeState::Flying);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert_eq!(out.vertical, AltHoldVertical::ClimbRate);
    assert_eq!(out.nav, PosHoldNavAction::None);
    assert!(out.init_target.is_none());
    assert!(out.update.is_none());
    assert_eq!(out.roll_mode, RpMode::PilotOverride);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(!out.reset_yaw_target_and_rate);
    assert!(out.set_max_speed_accel);
    assert!(out.clear_pilot_desired_acceleration);
    assert!(out.input_euler_angle_roll_pitch_euler_rate_yaw);
    assert!(out.update_d_controller);
    assert_eq!(out.target_climb_rate_ms.to_bits(), 1.0f32.to_bits());

    let (roll, pitch) = pilot_desired_lean_angles_rad(
        0.4,
        0.0,
        view.attitude_lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        true,
    );
    let yaw = pilot_desired_yaw_rate_rads(-0.3, view.yaw_rate_degs, view.yaw_expo, true);
    assert_eq!(out.target_roll_rad.to_bits(), roll.to_bits());
    assert_eq!(out.target_pitch_rad.to_bits(), pitch.to_bits());
    assert_eq!(out.target_yaw_rate_rads.to_bits(), yaw.to_bits());
    let mut filtered = 0.0;
    update_pilot_lean_angle_rad(&mut filtered, roll, view.brake_rate_degs, view.dt_s);
    assert_eq!(out.roll_rad.to_bits(), filtered.to_bits());
}

#[test]
fn released_stick_enters_brake() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let _ = poshold_init(false, &mut poshold, &mut loiter, &init_view(false, true));
    poshold.pilot_roll_rad = radians(4.0);
    poshold.pilot_pitch_rad = radians(4.0);

    let view = PosHoldRunView::flying();
    let out = poshold_run(&mut poshold, &mut loiter, &view);
    assert_eq!(out.roll_mode, RpMode::Brake);
    assert_eq!(out.pitch_mode, RpMode::Brake);
    assert_eq!(poshold.brake.roll_rad.to_bits(), 0.0f32.to_bits());
    assert!(!poshold.brake.time_updated_roll);
}

#[test]
fn motor_stopped_reseats_ac_loiter_and_resets_yaw_without_rate() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let mut view = PosHoldRunView::flying();
    view.armed = false;
    view.spool_state = SpoolState::ShutDown;
    let out = poshold_run(&mut poshold, &mut loiter, &view);

    assert_eq!(out.state, AltHoldModeState::MotorStopped);
    assert_eq!(out.desired_spool, Some(DesiredSpoolState::ShutDown));
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
    assert!(out.reset_yaw_target_and_rate);
    assert!(!out.reset_yaw_rate);
    assert_eq!(out.vertical, AltHoldVertical::Relax);
    assert_eq!(out.nav, PosHoldNavAction::InitTarget);
    // Alt-hold forces PILOT_OVERRIDE, then the same tick's RP machine
    // enters BRAKE on a centred stick.
    assert_eq!(out.roll_mode, RpMode::Brake);
    assert_eq!(out.pitch_mode, RpMode::Brake);
    let init = out
        .init_target
        .expect("stopped reseats Loiter::init_target");
    assert!(init.need_ne_relax_velocity_controller);
    assert!(out.update.is_none());
    assert_eq!(poshold.wind_comp_ne_mss, Vector2f::zero());
}

#[test]
fn maybe_landed_softens_ac_loiter() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let _ = poshold_init(false, &mut poshold, &mut loiter, &init_view(false, true));
    let mut view = PosHoldRunView::flying();
    view.land_complete_maybe = true;
    view.roll_in_norm = 0.3;
    let out = poshold_run(&mut poshold, &mut loiter, &view);
    assert!(out.soften_for_landing);
    assert_eq!(out.nav, PosHoldNavAction::None);
}

#[test]
fn takeoff_reseats_ac_loiter_as_pilot_override() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let mut view = PosHoldRunView::flying();
    view.land_complete = true;
    view.auto_armed = true;
    view.target_climb_rate_ms = 1.0;
    view.takeoff_running = false;
    view.takeoff_alt_m = 25.0;
    let out = poshold_run(&mut poshold, &mut loiter, &view);

    assert_eq!(out.state, AltHoldModeState::Takeoff);
    assert_eq!(out.vertical, AltHoldVertical::Takeoff);
    assert_eq!(out.nav, PosHoldNavAction::InitTarget);
    assert!(out.start_takeoff);
    assert_eq!(out.takeoff_start_alt_m.to_bits(), 10.0f32.to_bits());
    assert_eq!(out.roll_mode, RpMode::Brake);
    assert!(out
        .init_target
        .expect("takeoff reseats init_target")
        .need_ne_relax_velocity_controller);
    assert!(out.update.is_none());
}

#[test]
fn both_axes_ready_enter_brake_to_loiter_and_tick_update() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let _ = poshold_init(false, &mut poshold, &mut loiter, &init_view(false, true));
    poshold.roll_mode = RpMode::BrakeReadyToLoiter;
    poshold.pitch_mode = RpMode::BrakeReadyToLoiter;
    poshold.brake.gain = poshold_brake_gain(POSHOLD_BRAKE_RATE_DEFAULT_DEGS);

    let mut view = PosHoldRunView::flying();
    view.pos_target_ne_m = Vector2f::new(3.0, -1.5);
    view.loiter_roll_rad = 0.05;
    view.loiter_pitch_rad = -0.02;
    let out = poshold_run(&mut poshold, &mut loiter, &view);

    assert_eq!(out.nav, PosHoldNavAction::Update);
    let seated = out
        .init_target_m
        .expect("ready pair seats init_target_m");
    assert!(seated.need_ne_init_controller_stopping_point);
    assert_eq!(
        seated.pos_desired_ne_m.expect("init_target_m writes pos"),
        view.pos_target_ne_m
    );
    let update = out.update.expect("same tick ticks Loiter::update");
    assert!(update.need_calc_desired_velocity);
    assert!(update.need_ne_update_controller);
    assert!(!update.need_avoidance_adjust_velocity);
    assert_eq!(out.roll_mode, RpMode::BrakeToLoiter);
    assert_eq!(out.pitch_mode, RpMode::BrakeToLoiter);
    // mix=0 at the transition instant → second control (loiter angles).
    assert_eq!(out.roll_rad.to_bits(), 0.05f32.to_bits());
    assert_eq!(out.pitch_rad.to_bits(), (-0.02f32).to_bits());
}

#[test]
fn loiter_rp_mode_ticks_update_without_avoidance() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let _ = poshold_init(false, &mut poshold, &mut loiter, &init_view(true, true));
    poshold.roll_mode = RpMode::Loiter;
    poshold.pitch_mode = RpMode::Loiter;

    let mut view = PosHoldRunView::flying();
    view.loiter_roll_rad = 0.11;
    view.loiter_pitch_rad = -0.07;
    let out = poshold_run(&mut poshold, &mut loiter, &view);
    assert_eq!(out.nav, PosHoldNavAction::Update);
    assert!(!out
        .update
        .expect("loiter ticks update")
        .need_avoidance_adjust_velocity);
    assert_eq!(out.roll_rad.to_bits(), 0.11f32.to_bits());
    assert_eq!(out.pitch_rad.to_bits(), (-0.07f32).to_bits());
    assert_eq!(out.roll_mode, RpMode::Loiter);
}

#[test]
fn loiter_stick_deflects_to_controller_to_pilot() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    poshold.roll_mode = RpMode::Loiter;
    poshold.pitch_mode = RpMode::Loiter;

    let mut view = PosHoldRunView::flying();
    view.roll_in_norm = 0.5;
    let out = poshold_run(&mut poshold, &mut loiter, &view);
    assert_eq!(out.roll_mode, RpMode::ControllerToPilotOverride);
    assert_eq!(out.pitch_mode, RpMode::BrakeReadyToLoiter);
    assert_eq!(poshold.pilot_roll_rad.to_bits(), 0.0f32.to_bits());
    assert_eq!(poshold.brake.pitch_rad.to_bits(), 0.0f32.to_bits());
}

#[test]
fn brake_rate_below_min_is_clamped() {
    let mut poshold = PosHold::new();
    let mut loiter = Loiter::new();
    let mut view = PosHoldRunView::flying();
    view.brake_rate_degs = 1.0;
    view.roll_in_norm = 0.2;
    let out = poshold_run(&mut poshold, &mut loiter, &view);
    assert!(out.brake_rate_clamped);
    assert_eq!(
        out.brake_rate_degs.to_bits(),
        POSHOLD_BRAKE_RATE_MIN_DEGS.to_bits()
    );
}

#[test]
fn mix_controls_is_first_at_one_and_second_at_zero() {
    assert_eq!(mix_controls(1.0, 10.0, 3.0).to_bits(), 10.0f32.to_bits());
    assert_eq!(mix_controls(0.0, 10.0, 3.0).to_bits(), 3.0f32.to_bits());
    assert_eq!(mix_controls(0.5, 10.0, 0.0).to_bits(), 5.0f32.to_bits());
    assert_eq!(mix_controls(2.0, 10.0, 3.0).to_bits(), 10.0f32.to_bits());
}

#[test]
fn pilot_lean_snaps_on_reverse_or_large_raw() {
    let mut filtered = 0.1;
    update_pilot_lean_angle_rad(&mut filtered, -0.05, 8.0, 0.01);
    assert_eq!(filtered.to_bits(), (-0.05f32).to_bits());

    filtered = 0.05;
    update_pilot_lean_angle_rad(
        &mut filtered,
        POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD + 0.01,
        8.0,
        0.01,
    );
    assert_eq!(
        filtered.to_bits(),
        (POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD + 0.01).to_bits()
    );
}

#[test]
fn pilot_lean_smooths_toward_zero() {
    let mut filtered = 0.2;
    update_pilot_lean_angle_rad(&mut filtered, 0.0, 8.0, 0.01);
    assert!(filtered < 0.2);
    assert!(filtered > 0.0);
}

#[test]
fn brake_angle_opposes_velocity_and_is_slewed() {
    let gain = poshold_brake_gain(8.0);
    let mut angle = 0.0;
    update_brake_angle_from_velocity(&mut angle, 2.0, gain, 8.0, 30.0, 0.01);
    assert!(angle < 0.0);
    let first = angle;
    update_brake_angle_from_velocity(&mut angle, 2.0, gain, 8.0, 30.0, 0.01);
    assert!(angle < first);
}

#[test]
fn wind_comp_lean_is_body_frame_atan() {
    let (roll, pitch) = wind_comp_lean_angles_rad(Vector2f::new(0.0, GRAVITY_MSS), 1.0, 0.0);
    assert!((roll - core::f32::consts::FRAC_PI_4).abs() < 1.0e-5);
    assert!(pitch.abs() < 1.0e-6);
}
