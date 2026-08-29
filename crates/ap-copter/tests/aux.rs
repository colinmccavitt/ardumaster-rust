//! Copter aux-function leftover, upstream `RC_Channel_Copter.cpp`.

use ap_copter::aux::{
    arming_check_throttle, do_aux_function, do_aux_function_change_air_mode,
    do_aux_function_change_force_flying, do_aux_function_change_mode, do_aux_function_option,
    get_arming_channel, has_valid_input, in_rc_failsafe, init_aux_kind, AirMode, ArmingChannel,
    AuxDispatch, AuxLeftover, CopterAuxFunc, InitAuxKind, SimpleMode, MODE_ACRO, MODE_ALT_HOLD,
    MODE_AUTO, MODE_AUTO_RTL, MODE_BRAKE, MODE_CIRCLE, MODE_DRIFT, MODE_FLIP, MODE_FLOWHOLD,
    MODE_FOLLOW, MODE_GUIDED, MODE_LAND, MODE_LOITER, MODE_POSHOLD, MODE_REASON_AUX_FUNCTION,
    MODE_RTL, MODE_SMART_RTL, MODE_STABILIZE, MODE_THROW, MODE_TURTLE, MODE_ZIGZAG,
    THR_BEHAVE_FEEDBACK_FROM_MID_STICK,
};
use ap_rc::AuxSwitchPos;

#[test]
fn option_numbers_match_upstream() {
    assert_eq!(CopterAuxFunc::Flip as u16, 2);
    assert_eq!(CopterAuxFunc::SimpleMode as u16, 3);
    assert_eq!(CopterAuxFunc::Rtl as u16, 4);
    assert_eq!(CopterAuxFunc::SuperSimpleMode as u16, 13);
    assert_eq!(CopterAuxFunc::AcroTrainer as u16, 14);
    assert_eq!(CopterAuxFunc::Auto as u16, 16);
    assert_eq!(CopterAuxFunc::Land as u16, 18);
    assert_eq!(CopterAuxFunc::Guided as u16, 55);
    assert_eq!(CopterAuxFunc::Loiter as u16, 56);
    assert_eq!(CopterAuxFunc::Stabilize as u16, 68);
    assert_eq!(CopterAuxFunc::Althold as u16, 70);
    assert_eq!(CopterAuxFunc::AirMode as u16, 84);
    assert_eq!(CopterAuxFunc::AutoRtl as u16, 99);
    assert_eq!(CopterAuxFunc::Turtle as u16, 151);
    assert_eq!(CopterAuxFunc::SimpleHeadingReset as u16, 152);
    assert_eq!(CopterAuxFunc::ArmDisarmAirMode as u16, 154);
    assert_eq!(CopterAuxFunc::ForceFlying as u16, 159);
    assert_eq!(CopterAuxFunc::WeatherVaneEnable as u16, 160);
    assert_eq!(CopterAuxFunc::FlightmodePause as u16, 178);
    assert_eq!(CopterAuxFunc::AhrsAutoTrim as u16, 182);
    assert_eq!(CopterAuxFunc::TransmitterTuning as u16, 219);
    assert_eq!(
        CopterAuxFunc::from_option(3),
        Some(CopterAuxFunc::SimpleMode)
    );
    assert_eq!(CopterAuxFunc::from_option(0), None);
    assert_eq!(
        CopterAuxFunc::from_option(11),
        None,
        "FENCE is the RC base leftover"
    );
    assert_eq!(MODE_REASON_AUX_FUNCTION, 53);
    assert_eq!(MODE_FLIP, 14);
    assert_eq!(MODE_AUTO_RTL, 27);
}

#[test]
fn init_does_not_fire_mode_change_options() {
    assert_eq!(init_aux_kind(CopterAuxFunc::Rtl, true), InitAuxKind::NoInit);
    assert_eq!(
        init_aux_kind(CopterAuxFunc::Flip, true),
        InitAuxKind::NoInit
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::Land, true),
        InitAuxKind::NoInit
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::ArmDisarmAirMode, true),
        InitAuxKind::NoInit
    );
}

#[test]
fn init_runs_airmode_and_simple_when_tuning_is_compiled_in() {
    assert_eq!(
        init_aux_kind(CopterAuxFunc::AirMode, true),
        InitAuxKind::RunNow
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::SimpleMode, true),
        InitAuxKind::RunNow
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::ForceFlying, true),
        InitAuxKind::RunNow
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::MotorInterlock, true),
        InitAuxKind::RunNow
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::TransmitterTuning, true),
        InitAuxKind::RunNow
    );
}

#[test]
fn init_run_now_group_falls_to_base_when_tuning_is_compiled_out() {
    assert_eq!(
        init_aux_kind(CopterAuxFunc::AirMode, false),
        InitAuxKind::DelegateToBase
    );
    assert_eq!(
        init_aux_kind(CopterAuxFunc::TransmitterTuning, false),
        InitAuxKind::DelegateToBase
    );
}

#[test]
fn change_mode_high_engages_low_and_middle_only_reset_when_current() {
    assert_eq!(
        do_aux_function_change_mode(MODE_RTL, AuxSwitchPos::High, MODE_STABILIZE),
        AuxLeftover::SetMode { mode: MODE_RTL }
    );
    assert_eq!(
        do_aux_function_change_mode(MODE_RTL, AuxSwitchPos::Low, MODE_RTL),
        AuxLeftover::ResetModeSwitch
    );
    assert_eq!(
        do_aux_function_change_mode(MODE_RTL, AuxSwitchPos::Middle, MODE_RTL),
        AuxLeftover::ResetModeSwitch
    );
    assert_eq!(
        do_aux_function_change_mode(MODE_RTL, AuxSwitchPos::Low, MODE_STABILIZE),
        AuxLeftover::None,
        "LOW must not yank a mode the six-position switch already chose"
    );
}

fn dispatch(func: CopterAuxFunc, pos: AuxSwitchPos, current: u8) -> AuxLeftover {
    do_aux_function(AuxDispatch {
        func,
        pos,
        current_mode: current,
        acro_air_mode_hook: true,
    })
}

#[test]
fn mode_change_options_share_the_helper() {
    for (func, mode) in [
        (CopterAuxFunc::Rtl, MODE_RTL),
        (CopterAuxFunc::Auto, MODE_AUTO),
        (CopterAuxFunc::Land, MODE_LAND),
        (CopterAuxFunc::Guided, MODE_GUIDED),
        (CopterAuxFunc::Loiter, MODE_LOITER),
        (CopterAuxFunc::Follow, MODE_FOLLOW),
        (CopterAuxFunc::Brake, MODE_BRAKE),
        (CopterAuxFunc::Throw, MODE_THROW),
        (CopterAuxFunc::SmartRtl, MODE_SMART_RTL),
        (CopterAuxFunc::Stabilize, MODE_STABILIZE),
        (CopterAuxFunc::Poshold, MODE_POSHOLD),
        (CopterAuxFunc::Althold, MODE_ALT_HOLD),
        (CopterAuxFunc::Acro, MODE_ACRO),
        (CopterAuxFunc::Flowhold, MODE_FLOWHOLD),
        (CopterAuxFunc::Circle, MODE_CIRCLE),
        (CopterAuxFunc::Drift, MODE_DRIFT),
        (CopterAuxFunc::Zigzag, MODE_ZIGZAG),
        (CopterAuxFunc::AutoRtl, MODE_AUTO_RTL),
        (CopterAuxFunc::Turtle, MODE_TURTLE),
    ] {
        assert_eq!(
            func.change_mode_number(),
            Some(mode),
            "{func:?} must map to mode {mode}"
        );
        assert_eq!(
            dispatch(func, AuxSwitchPos::High, MODE_STABILIZE),
            AuxLeftover::SetMode { mode }
        );
        assert_eq!(
            dispatch(func, AuxSwitchPos::Low, mode),
            AuxLeftover::ResetModeSwitch
        );
        assert_eq!(
            dispatch(func, AuxSwitchPos::Low, MODE_STABILIZE),
            if mode == MODE_STABILIZE {
                AuxLeftover::ResetModeSwitch
            } else {
                AuxLeftover::None
            }
        );
    }
}

#[test]
fn flip_does_not_use_change_mode() {
    assert_eq!(CopterAuxFunc::Flip.change_mode_number(), None);
    assert_eq!(
        dispatch(CopterAuxFunc::Flip, AuxSwitchPos::High, MODE_STABILIZE),
        AuxLeftover::SetMode { mode: MODE_FLIP }
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Flip, AuxSwitchPos::Low, MODE_FLIP),
        AuxLeftover::None,
        "releasing Flip must not reset the mode switch"
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Flip, AuxSwitchPos::Middle, MODE_FLIP),
        AuxLeftover::None
    );
}

#[test]
fn simple_and_supersimple_use_different_maps() {
    assert_eq!(
        dispatch(CopterAuxFunc::SimpleMode, AuxSwitchPos::Low, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::None)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SimpleMode, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::Simple)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SimpleMode, AuxSwitchPos::High, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::Simple)
    );

    assert_eq!(
        dispatch(CopterAuxFunc::SuperSimpleMode, AuxSwitchPos::Low, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::None)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SuperSimpleMode, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::Simple)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SuperSimpleMode, AuxSwitchPos::High, 0),
        AuxLeftover::SetSimpleMode(SimpleMode::SuperSimple)
    );
}

#[test]
fn air_mode_and_force_flying_ignore_middle() {
    assert_eq!(
        do_aux_function_change_air_mode(AuxSwitchPos::High),
        Some(AirMode::Enabled)
    );
    assert_eq!(
        do_aux_function_change_air_mode(AuxSwitchPos::Low),
        Some(AirMode::Disabled)
    );
    assert_eq!(do_aux_function_change_air_mode(AuxSwitchPos::Middle), None);
    assert_eq!(
        do_aux_function_change_force_flying(AuxSwitchPos::High),
        Some(true)
    );
    assert_eq!(
        do_aux_function_change_force_flying(AuxSwitchPos::Low),
        Some(false)
    );
    assert_eq!(
        do_aux_function_change_force_flying(AuxSwitchPos::Middle),
        None
    );

    assert_eq!(
        dispatch(CopterAuxFunc::AirMode, AuxSwitchPos::High, 0),
        AuxLeftover::SetAirMode {
            air_mode: AirMode::Enabled,
            notify_acro: true,
        }
    );
    assert_eq!(
        do_aux_function(AuxDispatch {
            func: CopterAuxFunc::AirMode,
            pos: AuxSwitchPos::High,
            current_mode: 0,
            acro_air_mode_hook: false,
        }),
        AuxLeftover::SetAirMode {
            air_mode: AirMode::Enabled,
            notify_acro: false,
        }
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AirMode, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ForceFlying, AuxSwitchPos::Low, 0),
        AuxLeftover::SetForceFlying(false)
    );
}

#[test]
fn later_copter_bodies_are_pending_not_base() {
    assert_eq!(
        dispatch(CopterAuxFunc::SaveWp, AuxSwitchPos::High, 0),
        AuxLeftover::Pending
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Standby, AuxSwitchPos::High, 0),
        AuxLeftover::Pending
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ParachuteRelease, AuxSwitchPos::High, 0),
        AuxLeftover::Pending
    );
    assert_eq!(
        do_aux_function_option(11, AuxSwitchPos::High, 0, true),
        AuxLeftover::DelegateToBase,
        "FENCE is RC_Channel::do_aux_function, not Copter's table"
    );
    assert_eq!(
        do_aux_function_option(4, AuxSwitchPos::High, 0, true),
        AuxLeftover::SetMode { mode: MODE_RTL }
    );
}

#[test]
fn has_valid_input_rejects_pending_radio_failsafe() {
    assert!(has_valid_input(false, 0, true));
    assert!(!has_valid_input(true, 0, true));
    assert!(
        !has_valid_input(false, 1, true),
        "radio_counter still counting is already invalid"
    );
    assert!(!has_valid_input(false, 0, false));
    assert!(in_rc_failsafe(true));
    assert!(!in_rc_failsafe(false));
}

#[test]
fn sprung_throttle_skips_the_library_arming_check() {
    assert!(!arming_check_throttle(
        THR_BEHAVE_FEEDBACK_FROM_MID_STICK,
        true
    ));
    assert!(arming_check_throttle(0, true));
    assert!(!arming_check_throttle(0, false));
    assert_eq!(get_arming_channel(), ArmingChannel::Yaw);
}
