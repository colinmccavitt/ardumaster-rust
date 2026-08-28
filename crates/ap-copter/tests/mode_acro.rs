//! `ModeAcro` leftovers: trainer-off `run()`, trainer blend, init / exit.

use ap_copter::mode_acro::{
    acro_air_mode_aux_changed, acro_exit, acro_get_pilot_desired_rates_rads, acro_init,
    acro_pilot_desired_rates_rads, acro_run, acro_throttle_hover, AcroAirMode, AcroRatesView,
    AcroRunView, AcroTrainer, AirMode, ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_OPTION_AIR_MODE,
    ACRO_OPTION_RATE_LOOP_ONLY,
};
use ap_copter::mode_stabilize::RateIReset;
use ap_copter::pilot_input::pilot_desired_throttle;
use ap_math::quaternion::Quaternion;
use ap_math::scalar::{constrain_value, wrap_pi};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn flying_asks_unlimited_without_angle_boost() {
    let view = AcroRunView::flying();
    let out = acro_run(&view);
    assert_eq!(out.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(!out.angle_boost);
    assert!(!out.rate_loop_only);
    assert!(!out.scale_i_to_angle_p);
    assert!(!out.reset_target_and_rate);
    assert_eq!(out.reset_rate_i, RateIReset::None);
    assert!(out.clear_land_complete);
    assert_eq!(
        out.throttle_out.to_bits(),
        pilot_desired_throttle(500, 500, 0.5).to_bits()
    );
}

#[test]
fn rate_loop_only_scales_i_and_keeps_the_same_rates() {
    let mut view = AcroRunView::flying();
    view.roll_in_norm = 0.5;
    view.pitch_in_norm = -0.25;
    view.yaw_in_norm = 1.0;
    let attitude = acro_run(&view);
    view.rate_loop_only = true;
    let rate_only = acro_run(&view);

    assert!(rate_only.rate_loop_only);
    assert!(rate_only.scale_i_to_angle_p);
    assert!(!attitude.rate_loop_only);
    assert_eq!(
        rate_only.rates.roll_rads.to_bits(),
        attitude.rates.roll_rads.to_bits()
    );
    assert_eq!(
        rate_only.rates.pitch_rads.to_bits(),
        attitude.rates.pitch_rads.to_bits()
    );
    assert_eq!(
        rate_only.rates.yaw_rads.to_bits(),
        attitude.rates.yaw_rads.to_bits()
    );
    assert_eq!(ACRO_OPTION_RATE_LOOP_ONLY, 1 << 1);
}

#[test]
fn trainer_off_rates_scale_with_the_stick() {
    let half = acro_pilot_desired_rates_rads(0.25, -0.125, 1.0, 360.0, 0.0, 202.5, 0.0);
    let full = acro_pilot_desired_rates_rads(0.5, -0.25, 1.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(
        half.roll_rads.to_bits(),
        (full.roll_rads * 0.5).to_bits()
    );
    assert_eq!(
        half.pitch_rads.to_bits(),
        (full.pitch_rads * 0.5).to_bits()
    );
    assert_eq!(half.yaw_rads.to_bits(), full.yaw_rads.to_bits());
}

#[test]
fn circular_limit_caps_diagonal_stick() {
    let unlimited = acro_pilot_desired_rates_rads(1.0, 0.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    let diagonal = acro_pilot_desired_rates_rads(1.0, 1.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(
        diagonal.roll_rads.to_bits(),
        diagonal.pitch_rads.to_bits()
    );
    assert!(diagonal.roll_rads < unlimited.roll_rads);
    assert!(diagonal.roll_rads > 0.0);
}

#[test]
fn shut_down_resets_the_whole_target() {
    let mut view = AcroRunView::flying();
    view.spool_state = SpoolState::ShutDown;
    view.throttle_control = 800;
    let out = acro_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_target_and_rate);
    assert!(out.reset_target_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Hard);
}

#[test]
fn ground_idle_smooth_resets_the_whole_target() {
    let mut view = AcroRunView::flying();
    view.spool_state = SpoolState::GroundIdle;
    let out = acro_run(&view);
    assert_eq!(out.throttle_out.to_bits(), 0.0f32.to_bits());
    assert!(out.reset_target_and_rate);
    assert!(out.reset_target_rate);
    assert_eq!(out.reset_rate_i, RateIReset::Smooth);
}

#[test]
fn throttle_zero_asks_ground_idle() {
    let mut view = AcroRunView::flying();
    view.throttle_zero = true;
    let out = acro_run(&view);
    assert_eq!(out.desired_spool, DesiredSpoolState::GroundIdle);
}

#[test]
fn trainer_off_matches_the_stick_conversion() {
    let mut view = AcroRatesView::trainer_off();
    view.roll_in_norm = 0.4;
    view.pitch_in_norm = -0.2;
    view.yaw_in_norm = 0.5;
    view.att_target_roll_rad = 0.8;
    let trained = acro_get_pilot_desired_rates_rads(&view);
    let off = acro_pilot_desired_rates_rads(0.4, -0.2, 0.5, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(trained.roll_rads.to_bits(), off.roll_rads.to_bits());
    assert_eq!(trained.pitch_rads.to_bits(), off.pitch_rads.to_bits());
    assert_eq!(trained.yaw_rads.to_bits(), off.yaw_rads.to_bits());
}

#[test]
fn leveling_at_level_target_is_the_stick_request() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Leveling;
    view.roll_in_norm = 0.25;
    let trained = acro_get_pilot_desired_rates_rads(&view);
    let off = acro_pilot_desired_rates_rads(0.25, 0.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(trained.roll_rads.to_bits(), off.roll_rads.to_bits());
}

#[test]
fn leveling_zero_stick_pulls_opposite_the_bank() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Leveling;
    view.att_target_roll_rad = 0.2;
    view.att_target = Quaternion::identity();
    let out = acro_get_pilot_desired_rates_rads(&view);
    let expected = -constrain_value(0.2, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD);
    assert_eq!(out.roll_rads.to_bits(), expected.to_bits());
    assert_eq!(out.pitch_rads.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.yaw_rads.to_bits(), 0.0f32.to_bits());
    assert!(out.roll_rads < 0.0);
}

#[test]
fn leveling_wraps_the_attitude_target() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Leveling;
    view.att_target_roll_rad = 3.5;
    let out = acro_get_pilot_desired_rates_rads(&view);
    let wrapped = wrap_pi(3.5);
    let expected = -constrain_value(wrapped, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD);
    assert_eq!(out.roll_rads.to_bits(), expected.to_bits());
}

#[test]
fn leveling_full_stick_drops_the_level_mix() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Leveling;
    view.roll_in_norm = 1.0;
    view.att_target_roll_rad = 0.4;
    let trained = acro_get_pilot_desired_rates_rads(&view);
    let off = acro_pilot_desired_rates_rads(1.0, 0.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    assert_eq!(trained.roll_rads.to_bits(), off.roll_rads.to_bits());
}

#[test]
fn limited_within_lean_max_adds_the_uncapped_level_rate() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Limited;
    view.att_target_roll_rad = 0.2;
    view.roll_in_norm = 0.5;
    let trained = acro_get_pilot_desired_rates_rads(&view);
    let off = acro_pilot_desired_rates_rads(0.5, 0.0, 0.0, 360.0, 0.0, 202.5, 0.0);
    let level = -constrain_value(0.2, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD);
    assert_eq!(
        trained.roll_rads.to_bits(),
        (off.roll_rads + level).to_bits()
    );
}

#[test]
fn limited_past_lean_max_is_more_than_the_clamped_balance() {
    let mut view = AcroRatesView::trainer_off();
    view.trainer = AcroTrainer::Limited;
    view.att_target_roll_rad = 0.8;
    view.lean_angle_max_rad = ACRO_LEVEL_MAX_ANGLE_RAD;
    view.accel_roll_max_radss = 10.0;
    let limited = acro_get_pilot_desired_rates_rads(&view);
    view.trainer = AcroTrainer::Leveling;
    let leveling = acro_get_pilot_desired_rates_rads(&view);
    assert!(limited.roll_rads < leveling.roll_rads);
    assert!(limited.roll_rads < -ACRO_LEVEL_MAX_ANGLE_RAD);
}

#[test]
fn init_with_air_mode_enables_and_always_succeeds() {
    let mut state = AcroAirMode {
        air_mode: AirMode::None,
        disable_air_mode_reset: true,
    };
    assert!(acro_init(false, true, &mut state));
    assert!(acro_init(true, true, &mut state));
    assert_eq!(state.air_mode, AirMode::Enabled);
    assert!(!state.disable_air_mode_reset);
    assert_eq!(ACRO_OPTION_AIR_MODE, 1 << 0);
}

#[test]
fn init_without_air_mode_touches_nothing() {
    let mut state = AcroAirMode {
        air_mode: AirMode::Disabled,
        disable_air_mode_reset: true,
    };
    assert!(acro_init(false, false, &mut state));
    assert_eq!(state.air_mode, AirMode::Disabled);
    assert!(state.disable_air_mode_reset);
}

#[test]
fn exit_disables_air_mode_unless_aux_claimed_it() {
    let mut state = AcroAirMode::fresh();
    assert!(acro_init(false, true, &mut state));
    acro_exit(true, &mut state);
    assert_eq!(state.air_mode, AirMode::Disabled);
    assert!(!state.disable_air_mode_reset);

    assert!(acro_init(false, true, &mut state));
    acro_air_mode_aux_changed(&mut state);
    acro_exit(true, &mut state);
    assert_eq!(state.air_mode, AirMode::Enabled);
    assert!(!state.disable_air_mode_reset);
}

#[test]
fn exit_without_the_option_still_clears_the_disable_flag() {
    let mut state = AcroAirMode {
        air_mode: AirMode::Enabled,
        disable_air_mode_reset: true,
    };
    acro_exit(false, &mut state);
    assert_eq!(state.air_mode, AirMode::Enabled);
    assert!(!state.disable_air_mode_reset);
}

#[test]
fn throttle_hover_uses_mid_only_when_positive() {
    assert_eq!(acro_throttle_hover(0.4, 0.5).to_bits(), 0.4f32.to_bits());
    assert_eq!(acro_throttle_hover(0.0, 0.5).to_bits(), 0.5f32.to_bits());
    assert_eq!(acro_throttle_hover(-0.1, 0.5).to_bits(), 0.5f32.to_bits());
}
