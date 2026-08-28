//! RC aux-function switch latch vehicle hookup.

use ap_plane::rc_aux_switch_hookup::{read_aux_option, RcAuxSwitchHookup};
use ap_rc::{AuxFunc, AuxSwitchPos, SWITCH_DEBOUNCE_TIME_MS};

#[test]
fn hookup_disabled_option_never_fires() {
    let mut hookup = RcAuxSwitchHookup::default();
    assert_eq!(hookup.option(), AuxFunc::DoNothing);
    assert_eq!(
        read_aux_option(&mut hookup, 1100, SWITCH_DEBOUNCE_TIME_MS),
        None
    );
}

#[test]
fn hookup_latches_fence_switch_low() {
    let mut hookup = RcAuxSwitchHookup::from_option(AuxFunc::Fence);
    assert_eq!(hookup.read(1100, 0), None);
    assert_eq!(
        hookup.read(1100, SWITCH_DEBOUNCE_TIME_MS),
        Some(AuxSwitchPos::Low)
    );
    assert_eq!(hookup.current_position(), Some(AuxSwitchPos::Low));
}

#[test]
fn hookup_armdisarm_does_not_fire_on_first_high() {
    let mut hookup = RcAuxSwitchHookup::from_option(AuxFunc::ArmDisarm);
    assert_eq!(hookup.read(1900, 0), None);
    assert_eq!(hookup.current_position(), Some(AuxSwitchPos::High));
    assert_eq!(
        hookup.read(1100, SWITCH_DEBOUNCE_TIME_MS),
        None
    );
    assert_eq!(
        hookup.read(1100, SWITCH_DEBOUNCE_TIME_MS * 2),
        Some(AuxSwitchPos::Low)
    );
}
