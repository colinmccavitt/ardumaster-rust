//! Attitude leftover, upstream `ArduCopter/Attitude.cpp`.

use ap_copter::attitude::{
    constrain_throttle_deadzone, get_pilot_desired_climb_rate_ms, non_takeoff_throttle,
    pilot_speed_dn_ms, run_rate_controller_main, set_accel_throttle_i_from_pilot_throttle,
    update_throttle_hover, ClimbBand, ClimbRateContext, HoverLearnContext, HoverLearnSkip,
    HOVER_LEARN_DT_S, HOVER_VEL_D_MAX_MS, PILOT_SPD_UP_DEFAULT, THR_DZ_DEFAULT, THR_DZ_MAX,
};

fn almost(got: f32, want: f32) {
    assert!((got - want).abs() < 1.0e-6, "got {got} want {want}");
}

#[test]
fn constants_match_upstream() {
    assert_eq!(THR_DZ_DEFAULT, 100);
    assert_eq!(THR_DZ_MAX, 400);
    almost(PILOT_SPD_UP_DEFAULT, 2.5);
    almost(HOVER_VEL_D_MAX_MS, 0.6);
    almost(HOVER_LEARN_DT_S, 0.01);
}

#[test]
fn non_takeoff_throttle_is_the_attitude_cpp_helper() {
    almost(non_takeoff_throttle(0.5), 0.25);
    almost(non_takeoff_throttle(-0.4), 0.0);
}

#[test]
fn pilot_speed_dn_uses_climb_speed_when_dn_is_zero() {
    almost(pilot_speed_dn_ms(0.0, 2.5), 2.5);
    almost(pilot_speed_dn_ms(1.5, 2.5), 1.5);
    // Nonzero (even negative) takes fabs(dn), not the climb speed.
    almost(pilot_speed_dn_ms(-1.5, 2.5), 1.5);
    almost(pilot_speed_dn_ms(0.0, -2.5), 2.5);
}

#[test]
fn deadzone_is_written_back_clamped() {
    assert_eq!(constrain_throttle_deadzone(-20), 0);
    assert_eq!(constrain_throttle_deadzone(100), 100);
    assert_eq!(constrain_throttle_deadzone(400), 400);
    assert_eq!(constrain_throttle_deadzone(800), 400);
}

#[test]
fn climb_rate_failsafe_is_zero_and_reads_nothing() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        has_valid_input: false,
        throttle_control: 1000.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(leftover.band, ClimbBand::Failsafe);
    almost(leftover.rate_ms, 0.0);
    assert!(!leftover.need_throttle_in);
    assert!(!leftover.need_toy_adjust);
    assert!(!leftover.deadzone_written);
    assert!(!leftover.need_throttle_mid);
    assert!(!leftover.need_speed_dn);
    assert!(!leftover.need_speed_up);
}

#[test]
fn climb_rate_mid_stick_is_deadband() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext::default());
    assert_eq!(leftover.band, ClimbBand::Deadband);
    almost(leftover.rate_ms, 0.0);
    assert!(leftover.need_throttle_in);
    assert!(leftover.deadzone_written);
    assert_eq!(leftover.deadzone, 100);
    assert!(leftover.need_throttle_mid);
    assert!(!leftover.need_speed_dn);
    assert!(!leftover.need_speed_up);
}

#[test]
fn climb_rate_edges_of_deadband_are_still_zero() {
    let low = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 400.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(low.band, ClimbBand::Deadband);
    almost(low.rate_ms, 0.0);

    let high = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 600.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(high.band, ClimbBand::Deadband);
    almost(high.rate_ms, 0.0);
}

#[test]
fn climb_rate_full_down_is_minus_speed_dn() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 0.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(leftover.band, ClimbBand::Below);
    almost(leftover.rate_ms, -2.5);
    assert!(leftover.need_speed_dn);
    assert!(!leftover.need_speed_up);
}

#[test]
fn climb_rate_full_up_is_speed_up() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 1000.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(leftover.band, ClimbBand::Above);
    almost(leftover.rate_ms, 2.5);
    assert!(!leftover.need_speed_dn);
    assert!(leftover.need_speed_up);
}

#[test]
fn climb_rate_below_uses_configured_dn_not_up() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 0.0,
        pilot_speed_dn_ms: 1.0,
        pilot_speed_up_ms: 2.5,
        ..ClimbRateContext::default()
    });
    almost(leftover.rate_ms, -1.0);
}

#[test]
fn climb_rate_below_divides_by_deadband_bottom_not_mid() {
    // mid 500, dz 100 → bottom 400. Stick 200 is halfway from 400 to 0,
    // so half of speed_dn — not 200/500 of it.
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 200.0,
        pilot_speed_dn_ms: 2.0,
        ..ClimbRateContext::default()
    });
    assert_eq!(leftover.band, ClimbBand::Below);
    almost(leftover.rate_ms, 2.0 * (200.0 - 400.0) / 400.0);
}

#[test]
fn climb_rate_constrains_stick_and_deadzone() {
    let leftover = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 1500.0,
        throttle_deadzone: 800,
        ..ClimbRateContext::default()
    });
    almost(leftover.throttle_control, 1000.0);
    assert_eq!(leftover.deadzone, 400);
    assert!(leftover.deadzone_written);
}

#[test]
fn climb_rate_toy_adjust_replaces_the_stick() {
    let unused = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 1000.0,
        toy_mode_compiled: true,
        toy_mode_enabled: false,
        throttle_after_toy_adjust: 0.0,
        ..ClimbRateContext::default()
    });
    assert!(!unused.need_toy_adjust);
    assert_eq!(unused.band, ClimbBand::Above);

    let used = get_pilot_desired_climb_rate_ms(&ClimbRateContext {
        throttle_control: 1000.0,
        toy_mode_compiled: true,
        toy_mode_enabled: true,
        throttle_after_toy_adjust: 0.0,
        ..ClimbRateContext::default()
    });
    assert!(used.need_toy_adjust);
    assert_eq!(used.band, ClimbBand::Below);
    almost(used.throttle_control, 0.0);
}

#[test]
fn hover_default_learns() {
    let leftover = update_throttle_hover(&HoverLearnContext::default());
    assert!(leftover.skip.is_none());
    assert!(leftover.learn);
    assert!(leftover.need_vel_desired_u);
    assert!(leftover.need_velocity_d);
    assert!(leftover.need_throttle);
    assert!(leftover.need_attitude);
    assert!(!leftover.learn_gyro_fft);
    assert!(!leftover.need_throttle_out);
}

#[test]
fn hover_skips_before_reading_velocity() {
    let disarmed = update_throttle_hover(&HoverLearnContext {
        armed: false,
        ..HoverLearnContext::default()
    });
    assert_eq!(disarmed.skip, Some(HoverLearnSkip::Disarmed));
    assert!(!disarmed.need_vel_desired_u);
    assert!(!disarmed.learn);

    let landed = update_throttle_hover(&HoverLearnContext {
        land_complete: true,
        ..HoverLearnContext::default()
    });
    assert_eq!(landed.skip, Some(HoverLearnSkip::LandComplete));

    let standby = update_throttle_hover(&HoverLearnContext {
        standby_active: true,
        ..HoverLearnContext::default()
    });
    assert_eq!(standby.skip, Some(HoverLearnSkip::Standby));

    let manual = update_throttle_hover(&HoverLearnContext {
        has_manual_throttle: true,
        ..HoverLearnContext::default()
    });
    assert_eq!(manual.skip, Some(HoverLearnSkip::ManualThrottle));
    assert!(!manual.need_vel_desired_u);

    let drift = update_throttle_hover(&HoverLearnContext {
        is_drift: true,
        ..HoverLearnContext::default()
    });
    assert_eq!(drift.skip, Some(HoverLearnSkip::Drift));
}

#[test]
fn hover_skips_a_vertical_demand() {
    let leftover = update_throttle_hover(&HoverLearnContext {
        vel_desired_u_ms: 0.5,
        ..HoverLearnContext::default()
    });
    assert_eq!(leftover.skip, Some(HoverLearnSkip::VerticalDemand));
    assert!(leftover.need_vel_desired_u);
    assert!(!leftover.need_velocity_d);
    assert!(!leftover.learn);
}

#[test]
fn hover_failed_velocity_d_is_a_hard_skip() {
    let leftover = update_throttle_hover(&HoverLearnContext {
        velocity_d_ok: false,
        vel_d_ms: 0.0,
        ..HoverLearnContext::default()
    });
    assert_eq!(leftover.skip, Some(HoverLearnSkip::NoVelocityD));
    assert!(leftover.need_velocity_d);
    assert!(!leftover.need_throttle);
    assert!(!leftover.learn);
}

#[test]
fn hover_not_level_reads_attitude_and_does_not_learn() {
    let zero_thr = update_throttle_hover(&HoverLearnContext {
        throttle: 0.0,
        ..HoverLearnContext::default()
    });
    assert_eq!(zero_thr.skip, Some(HoverLearnSkip::NotLevelHover));
    assert!(zero_thr.need_throttle);
    assert!(zero_thr.need_attitude);
    assert!(!zero_thr.learn);

    let descending = update_throttle_hover(&HoverLearnContext {
        vel_d_ms: 0.6,
        ..HoverLearnContext::default()
    });
    assert_eq!(
        descending.skip,
        Some(HoverLearnSkip::NotLevelHover),
        "|vel_d| < 0.6 is strict"
    );

    let pitched = update_throttle_hover(&HoverLearnContext {
        pitch_rad: 0.1,
        ..HoverLearnContext::default()
    });
    assert_eq!(
        pitched.skip,
        Some(HoverLearnSkip::NotLevelHover),
        "0.1 rad is past radians(5)"
    );
}

#[test]
fn hover_accounts_for_heli_roll_trim() {
    let leftover = update_throttle_hover(&HoverLearnContext {
        roll_rad: 0.05,
        roll_trim_rad: 0.05,
        ..HoverLearnContext::default()
    });
    assert!(leftover.learn, "level relative to trim must still learn");
}

#[test]
fn hover_fft_leftover_only_when_compiled_and_learning() {
    let learned = update_throttle_hover(&HoverLearnContext {
        gyro_fft_enabled: true,
        ..HoverLearnContext::default()
    });
    assert!(learned.learn);
    assert!(learned.learn_gyro_fft);
    assert!(learned.need_throttle_out);

    let skipped = update_throttle_hover(&HoverLearnContext {
        gyro_fft_enabled: true,
        armed: false,
        ..HoverLearnContext::default()
    });
    assert!(!skipped.learn_gyro_fft);
    assert!(!skipped.need_throttle_out);
}

#[test]
fn rate_controller_main_always_resets_the_target() {
    let main = run_rate_controller_main(0.0025, false);
    almost(main.dt_s, 0.0025);
    assert!(main.set_pos_control_dt);
    assert!(main.set_attitude_control_dt);
    assert!(main.set_motors_dt);
    assert!(main.run_rate_controller);
    assert!(main.reset_rate_target);

    let threaded = run_rate_controller_main(0.0025, true);
    assert!(!threaded.set_motors_dt);
    assert!(!threaded.run_rate_controller);
    assert!(threaded.reset_rate_target);
    assert!(threaded.set_pos_control_dt);
}

#[test]
fn accel_i_is_hover_minus_pilot() {
    let leftover = set_accel_throttle_i_from_pilot_throttle(0.7, 0.5);
    almost(leftover.pilot_throttle, 0.7);
    almost(leftover.integrator, -0.2);
    assert!(leftover.need_throttle_in);
    assert!(leftover.need_throttle_hover);

    let clamped = set_accel_throttle_i_from_pilot_throttle(1.4, 0.5);
    almost(clamped.pilot_throttle, 1.0);
    almost(clamped.integrator, -0.5);
}
