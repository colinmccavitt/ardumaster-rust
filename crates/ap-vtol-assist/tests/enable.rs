//! VTOL_Assist enable / check gate.

use ap_vtol_assist::{
    q_assist_force_enable_set, AssistOption, AssistState, AuxSwitchPos, VtolAssist,
    ASSIST_ALT_DEFAULT, ASSIST_ANGLE_DEFAULT, ASSIST_DELAY_DEFAULT, ASSIST_OPTIONS_DEFAULT,
    ASSIST_SPEED_DEFAULT, DISABLE_SYNTHETIC_AIRSPEED_ASSIST, Q_ASSIST_FORCE_ENABLE,
};

#[test]
fn groupinfo_defaults_match_upstream() {
    let assist = VtolAssist::new();
    assert_eq!(assist.speed(), ASSIST_SPEED_DEFAULT);
    assert_eq!(assist.speed(), 0.0);
    assert_eq!(assist.angle(), ASSIST_ANGLE_DEFAULT);
    assert_eq!(assist.angle(), 30);
    assert_eq!(assist.alt(), ASSIST_ALT_DEFAULT);
    assert_eq!(assist.alt(), 0);
    assert_eq!(assist.delay(), ASSIST_DELAY_DEFAULT);
    assert_eq!(assist.delay(), 0.5);
    assert_eq!(assist.options(), ASSIST_OPTIONS_DEFAULT);
    assert_eq!(assist.options(), 0);
    assert_eq!(assist.state(), AssistState::AssistEnabled);
}

#[test]
fn assist_state_matches_upstream_declaration_order() {
    assert_eq!(AssistState::AssistDisabled.as_u8(), 0);
    assert_eq!(AssistState::AssistEnabled.as_u8(), 1);
    assert_eq!(AssistState::ForceEnabled.as_u8(), 2);
    assert_eq!(AssistState::from_u8(0), Some(AssistState::AssistDisabled));
    assert_eq!(AssistState::from_u8(1), Some(AssistState::AssistEnabled));
    assert_eq!(AssistState::from_u8(2), Some(AssistState::ForceEnabled));
    assert_eq!(AssistState::from_u8(3), None);
}

#[test]
fn aux_switch_maps_to_state() {
    assert_eq!(
        AssistState::from_aux(AuxSwitchPos::Low),
        AssistState::AssistDisabled
    );
    assert_eq!(
        AssistState::from_aux(AuxSwitchPos::Middle),
        AssistState::AssistEnabled
    );
    assert_eq!(
        AssistState::from_aux(AuxSwitchPos::High),
        AssistState::ForceEnabled
    );

    let mut assist = VtolAssist::new();
    assist.set_state_from_aux(AuxSwitchPos::Low);
    assert_eq!(assist.state(), AssistState::AssistDisabled);
    assist.set_state_from_aux(AuxSwitchPos::High);
    assert_eq!(assist.state(), AssistState::ForceEnabled);
}

#[test]
fn default_speed_zero_disables_checks() {
    let assist = VtolAssist::new();
    assert!(!assist.speed_checks_enabled());
    assert!(!assist.alt_check_enabled());
    assert!(!assist.should_check());
    assert!(!assist.is_enabled());
}

#[test]
fn positive_assist_speed_enables_checks() {
    let mut assist = VtolAssist::new();
    assist.set_speed(8.0);
    assert!(assist.speed_checks_enabled());
    assert!(assist.should_check());
    assert!(assist.is_enabled());
    assert!(!assist.alt_check_enabled());
}

#[test]
fn assist_speed_minus_one_disables_all_checks() {
    let mut assist = VtolAssist::new();
    assist.set_speed(-1.0);
    assist.set_alt(20);
    assert!(!assist.speed_checks_enabled());
    assert!(!assist.alt_check_enabled());
    assert!(!assist.should_check());
    assert!(!assist.is_enabled());
}

#[test]
fn assist_alt_alone_does_not_open_the_speed_gate() {
    let mut assist = VtolAssist::new();
    assist.set_alt(15);
    assert!(!assist.speed_checks_enabled());
    assert!(!assist.alt_check_enabled());
    assert!(!assist.should_check());
    assert!(!assist.is_enabled());
}

#[test]
fn assist_alt_is_live_only_when_speed_is_positive() {
    let mut assist = VtolAssist::new();
    assist.set_speed(5.0);
    assist.set_alt(15);
    assert!(assist.alt_check_enabled());
    assert!(assist.should_check());
    assert!(assist.is_enabled());

    assist.set_alt(0);
    assert!(!assist.alt_check_enabled());
    assert!(assist.should_check());
    assert!(assist.is_enabled());
}

#[test]
fn disabled_state_turns_assist_off_even_with_speed() {
    let mut assist = VtolAssist::new();
    assist.set_speed(8.0);
    assist.set_alt(10);
    assist.set_state(AssistState::AssistDisabled);
    assert!(assist.speed_checks_enabled());
    assert!(assist.alt_check_enabled());
    assert!(!assist.should_check());
    assert!(!assist.is_enabled());
}

#[test]
fn force_enabled_assists_when_speed_is_zero() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::ForceEnabled);
    assert!(!assist.speed_checks_enabled());
    assert!(!assist.should_check());
    assert!(assist.is_enabled());
}

#[test]
fn q_options_force_enable_latches_force_state() {
    assert_eq!(Q_ASSIST_FORCE_ENABLE, 1 << 7);
    assert_eq!(DISABLE_SYNTHETIC_AIRSPEED_ASSIST, 1 << 12);
    assert!(q_assist_force_enable_set(Q_ASSIST_FORCE_ENABLE));
    assert!(!q_assist_force_enable_set(0));
    assert!(!q_assist_force_enable_set(
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST
    ));

    let mut assist = VtolAssist::new();
    assist.apply_q_options(0);
    assert_eq!(assist.state(), AssistState::AssistEnabled);
    assert!(!assist.is_enabled());

    assist.apply_q_options(Q_ASSIST_FORCE_ENABLE);
    assert_eq!(assist.state(), AssistState::ForceEnabled);
    assert!(assist.is_enabled());
}

#[test]
fn assist_option_bits_match_upstream() {
    assert_eq!(AssistOption::FwForceDisabled.as_i16(), 1 << 0);
    assert_eq!(AssistOption::SpinDisabled.as_i16(), 1 << 1);

    let mut assist = VtolAssist::new();
    assert!(!assist.option_is_set(AssistOption::FwForceDisabled));
    assert!(!assist.option_is_set(AssistOption::SpinDisabled));

    assist.set_options(AssistOption::FwForceDisabled.as_i16());
    assert!(assist.option_is_set(AssistOption::FwForceDisabled));
    assert!(!assist.option_is_set(AssistOption::SpinDisabled));

    assist
        .set_options(AssistOption::FwForceDisabled.as_i16() | AssistOption::SpinDisabled.as_i16());
    assert!(assist.option_is_set(AssistOption::FwForceDisabled));
    assert!(assist.option_is_set(AssistOption::SpinDisabled));
}
