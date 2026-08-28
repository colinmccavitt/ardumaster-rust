//! RC aux-function switch latch, upstream `RC_Channel::read_aux` / `RCn_OPTION`.

use ap_rc::{
    read_3pos_switch, AuxFunc, AuxSwitchLatch, AuxSwitchPos, AUX_SWITCH_PWM_TRIGGER_HIGH,
    AUX_SWITCH_PWM_TRIGGER_LOW, SWITCH_DEBOUNCE_TIME_MS,
};

#[test]
fn option_zero_is_disabled() {
    let mut latch = AuxSwitchLatch::new(AuxFunc::DoNothing);
    assert_eq!(latch.read_aux(1100, SWITCH_DEBOUNCE_TIME_MS), None);
}

#[test]
fn rcn_option_fence_latches_low_then_high() {
    let mut latch = AuxSwitchLatch::new(AuxFunc::Fence);
    assert!(AUX_SWITCH_PWM_TRIGGER_LOW < AUX_SWITCH_PWM_TRIGGER_HIGH);
    assert_eq!(read_3pos_switch(1100, false), Some(AuxSwitchPos::Low));

    assert_eq!(latch.read_aux(1100, 0), None);
    assert_eq!(
        latch.read_aux(1100, SWITCH_DEBOUNCE_TIME_MS),
        Some(AuxSwitchPos::Low)
    );

    assert_eq!(latch.read_aux(1900, SWITCH_DEBOUNCE_TIME_MS), None);
    assert_eq!(
        latch.read_aux(1900, SWITCH_DEBOUNCE_TIME_MS * 2),
        Some(AuxSwitchPos::High)
    );
    assert_eq!(latch.current_position(), Some(AuxSwitchPos::High));
}

#[test]
fn q_assist_middle_band_is_not_an_edge() {
    let mut latch = AuxSwitchLatch::new(AuxFunc::QAssist);
    assert_eq!(latch.read_aux(1500, 0), None);
    assert_eq!(
        latch.read_aux(1500, SWITCH_DEBOUNCE_TIME_MS),
        Some(AuxSwitchPos::Middle)
    );
    assert_eq!(latch.read_aux(1600, SWITCH_DEBOUNCE_TIME_MS + 50), None);
}
