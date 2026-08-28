//! Option-switch PWM ranges, upstream `AUX_PWM_TRIGGER_*` vs `AUX_SWITCH_PWM_TRIGGER_*`.

use ap_rc::{
    get_stick_gesture_pos, option_switch_asserted, option_switch_has_three_positions,
    read_2pos_switch, read_3pos_switch, read_option_switch, AuxFunc, AuxSwitchPos,
    AUX_PWM_TRIGGER_HIGH, AUX_PWM_TRIGGER_LOW, AUX_SWITCH_PWM_TRIGGER_HIGH,
    AUX_SWITCH_PWM_TRIGGER_LOW, STICK_GESTURE_MAX_PWM, STICK_GESTURE_MIN_PWM,
};

#[test]
fn two_pos_thresholds_are_tighter_than_three_pos() {
    assert_eq!(AUX_PWM_TRIGGER_LOW, 1300);
    assert_eq!(AUX_PWM_TRIGGER_HIGH, 1700);
    assert!(AUX_SWITCH_PWM_TRIGGER_LOW < AUX_PWM_TRIGGER_LOW);
    assert!(AUX_PWM_TRIGGER_HIGH < AUX_SWITCH_PWM_TRIGGER_HIGH);
}

#[test]
fn mid_band_pwm_differs_between_two_pos_and_three_pos() {
    // 1250 is MIDDLE on the 3-pos table (1200/1800) and LOW on 2-pos (1300/1700).
    assert_eq!(read_3pos_switch(1250, false), Some(AuxSwitchPos::Middle));
    assert_eq!(read_2pos_switch(1250, false), Some(AuxSwitchPos::Low));
    // 1750 is MIDDLE on 3-pos and HIGH on 2-pos.
    assert_eq!(read_3pos_switch(1750, false), Some(AuxSwitchPos::Middle));
    assert_eq!(read_2pos_switch(1750, false), Some(AuxSwitchPos::High));
}

#[test]
fn aux_function_picks_the_pwm_table() {
    assert!(option_switch_has_three_positions(AuxFunc::Fence));
    assert!(option_switch_has_three_positions(AuxFunc::QAssist));
    assert!(option_switch_has_three_positions(AuxFunc::Soaring));
    assert!(!option_switch_has_three_positions(AuxFunc::ReverseThrottle));
    assert!(!option_switch_has_three_positions(AuxFunc::ArmDisarm));
    assert!(!option_switch_has_three_positions(AuxFunc::DoNothing));

    assert_eq!(
        read_option_switch(1250, false, AuxFunc::QAssist),
        Some(AuxSwitchPos::Middle)
    );
    assert_eq!(
        read_option_switch(1250, false, AuxFunc::ReverseThrottle),
        Some(AuxSwitchPos::Low)
    );
    assert_eq!(
        read_option_switch(1750, false, AuxFunc::Soaring),
        Some(AuxSwitchPos::Middle)
    );
    assert_eq!(
        read_option_switch(1750, false, AuxFunc::ArmDisarm),
        Some(AuxSwitchPos::High)
    );
    assert_eq!(read_option_switch(1500, false, AuxFunc::DoNothing), None);
}

#[test]
fn reverse_throttle_asserts_only_on_high() {
    let pos = read_option_switch(1900, false, AuxFunc::ReverseThrottle);
    assert_eq!(pos, Some(AuxSwitchPos::High));
    assert!(option_switch_asserted(pos.unwrap()));
    let mid = read_option_switch(1500, false, AuxFunc::ReverseThrottle);
    assert_eq!(mid, Some(AuxSwitchPos::Middle));
    assert!(!option_switch_asserted(mid.unwrap()));
}

#[test]
fn stick_gesture_invalid_floor_is_900_not_800() {
    assert_eq!(STICK_GESTURE_MIN_PWM, 900);
    assert_eq!(STICK_GESTURE_MAX_PWM, 2200);
    assert_eq!(get_stick_gesture_pos(900, false), AuxSwitchPos::Low);
    assert_eq!(get_stick_gesture_pos(2200, false), AuxSwitchPos::Low);
    // 850 is a valid 2-pos channel sample (800/2200) but an invalid stick gesture.
    assert_eq!(read_2pos_switch(850, false), Some(AuxSwitchPos::Low));
    assert_eq!(get_stick_gesture_pos(850, false), AuxSwitchPos::Low);
    assert_eq!(get_stick_gesture_pos(1701, true), AuxSwitchPos::Low);
}
