//! THERMAL mode hookup for soaring-assisted nav roll.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::thermal_mode_hookup::{
    thermal_mode_nav_tick, ThermalModeNavInputs, SOAR_THML_BANK_DEFAULT_DEG,
};

fn soaring_features() -> BuildFeatures {
    BuildFeatures {
        soaring: true,
        ..BuildFeatures::default()
    }
}

fn thermal_inp() -> ThermalModeNavInputs {
    ThermalModeNavInputs {
        control_mode: ModeNumber::Thermal.as_number(),
        features: soaring_features(),
        thermal_bank_deg: SOAR_THML_BANK_DEFAULT_DEG,
        roll_limit_cd: 4500,
    }
}

#[test]
fn thermal_mode_nav_banks_soar_thml_bank() {
    let out = thermal_mode_nav_tick(&thermal_inp());
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 3000);
}

#[test]
fn thermal_mode_nav_constrains_bank_to_roll_limit() {
    let mut inp = thermal_inp();
    inp.roll_limit_cd = 2000;
    let out = thermal_mode_nav_tick(&inp);
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 2000);
}

#[test]
fn thermal_mode_nav_skips_without_soaring_feature() {
    let mut inp = thermal_inp();
    inp.features = BuildFeatures::default();
    let out = thermal_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn thermal_mode_nav_skips_other_modes() {
    let mut inp = thermal_inp();
    inp.control_mode = ModeNumber::Circle.as_number();
    let out = thermal_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn main_loop_applies_thermal_mode_nav_bank() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.soaring = true;
    vehicle.mode.control_mode = ModeNumber::Thermal.as_number();
    vehicle.thermal_bank_deg = SOAR_THML_BANK_DEFAULT_DEG;
    vehicle.stabilize_demands.roll_limit_cd = 4500;
    vehicle.stabilize_demands.pitch_limit_min_cd = -1500;
    vehicle.stabilize_demands.pitch_limit_max_cd = 2000;
    vehicle.rc_failsafe_inputs = RcFailsafeSchedulerInputs {
        has_valid_input: true,
        roll_pwm: Some(1600),
        pitch_pwm: Some(1300),
        yaw_pwm: None,
        throttle_pwm: None,
        roll_cfg: RcChannelConfig::default(),
        pitch_cfg: RcChannelConfig::default(),
        yaw_cfg: RcChannelConfig::default(),
        throttle_cfg: RcChannelConfig::default(),
        failsafe_cfg: RcFailsafeConfig::default(),
        flap_pwm: None,
        flap_cfg: RcChannelConfig::default(),
    };
    vehicle.update_control_mode();

    assert!(vehicle.thermal_mode_nav_applied);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.autotune_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.fbwb_mode_nav_applied);
    assert!(!vehicle.cruise_mode_nav_applied);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
    assert!(!vehicle.training_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 3000);
    // Soaring-assisted: elevator stick must not overwrite TECS pitch the way FBWA does
    // (FBWA maps pitch PWM 1300 / limits -1500..2000 to -750).
    assert_ne!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}
