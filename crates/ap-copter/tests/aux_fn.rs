//! Copter aux-function leftover, upstream `RC_Channel_Copter.cpp`.

use ap_copter::aux_fn::{
    arming_check_throttle, auto_trim_cancel, auto_trim_run, do_aux_function,
    do_aux_function_acro_trainer, do_aux_function_ahrs_auto_trim, do_aux_function_change_air_mode,
    do_aux_function_change_force_flying, do_aux_function_change_mode, do_aux_function_option,
    do_aux_function_parachute_release, do_aux_function_zigzag_save_wp, eeprom_simple_mode,
    flight_mode_channel, flight_mode_channel_number, get_arming_channel, get_control_in,
    get_control_in_zero_dz, get_control_mid, get_throttle_mid, has_valid_input, in_rc_failsafe,
    init_aux_function, init_aux_kind, init_rc_in_map, mode_switch_changed, save_trim, AcroTrainer,
    AhrsAutoTrimAction, AirMode, ArmingChannel, AutoTrimCancelReason, AutoTrimRunLeftover,
    AuxDispatch, AuxLeftover, ControlType, CopterAuxFunc, FlightmodePauseAction, InitAuxKind,
    ModeSwitchLeftover, Parachute3Pos, SaveTrimLeftover, SimpleMode, SurfaceTracking,
    WinchEnableAction, ZigzagAutoAction, ZigzagSaveWp, CH_MODE_DEFAULT, DEADZONE_ROLL_PITCH,
    DEADZONE_THROTTLE_HELI, DEADZONE_THROTTLE_MULTICOPTER, DEADZONE_YAW_HELI,
    DEADZONE_YAW_MULTICOPTER, MODE_ACRO, MODE_ALT_HOLD, MODE_AUTO, MODE_AUTO_RTL, MODE_BRAKE,
    MODE_CIRCLE, MODE_DRIFT, MODE_FLIP, MODE_FLOWHOLD, MODE_FOLLOW, MODE_GUIDED, MODE_LAND,
    MODE_LOITER, MODE_POSHOLD, MODE_REASON_AUX_FUNCTION, MODE_REASON_RC_COMMAND, MODE_RTL,
    MODE_SMART_RTL, MODE_STABILIZE, MODE_THROW, MODE_TURTLE, MODE_ZIGZAG, NUM_FLIGHT_MODES,
    ROLL_PITCH_YAW_INPUT_MAX, THROTTLE_CONTROL_RANGE, THR_BEHAVE_FEEDBACK_FROM_MID_STICK,
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
    assert_eq!(MODE_REASON_RC_COMMAND, 1);
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
fn unknown_options_still_fall_to_base() {
    assert_eq!(
        do_aux_function_option(11, AuxSwitchPos::High, 0, true),
        AuxLeftover::DelegateToBase,
        "FENCE is RC_Channel::do_aux_function, not Copter's table"
    );
    assert_eq!(
        do_aux_function_option(4, AuxSwitchPos::High, 0, true),
        AuxLeftover::SetMode { mode: MODE_RTL }
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ResetToArmedYaw, AuxSwitchPos::High, 0),
        AuxLeftover::DelegateToBase,
        "RESETTOARMEDYAW has no Copter do_aux case"
    );
}

#[test]
fn parachute_release_and_save_high_only() {
    assert_eq!(
        do_aux_function_parachute_release(AuxSwitchPos::High),
        AuxLeftover::ReleaseParachute
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ParachuteRelease, AuxSwitchPos::High, 0),
        AuxLeftover::ReleaseParachute
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ParachuteRelease, AuxSwitchPos::Low, 0),
        AuxLeftover::None,
        "LOW must not dump the canopy"
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SaveTrim, AuxSwitchPos::High, 0),
        AuxLeftover::SaveTrim
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SaveTrim, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SaveWp, AuxSwitchPos::High, 0),
        AuxLeftover::SaveWp
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SaveWp, AuxSwitchPos::Low, 0),
        AuxLeftover::None
    );
}

#[test]
fn winch_control_is_consumed_elsewhere() {
    assert_eq!(
        dispatch(CopterAuxFunc::WinchControl, AuxSwitchPos::High, 0),
        AuxLeftover::None,
        "WINCH_CONTROL is processed in AP_Winch, not here"
    );
}

#[test]
fn zigzag_options_require_zigzag_mode() {
    assert_eq!(
        do_aux_function_zigzag_save_wp(AuxSwitchPos::Low, MODE_STABILIZE),
        AuxLeftover::None,
        "ZigZag save from Stabilize must not write a destination"
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ZigzagSaveWp, AuxSwitchPos::Low, MODE_ZIGZAG),
        AuxLeftover::ZigzagSaveWp(ZigzagSaveWp::DestinationA)
    );
    assert_eq!(
        dispatch(
            CopterAuxFunc::ZigzagSaveWp,
            AuxSwitchPos::Middle,
            MODE_ZIGZAG
        ),
        AuxLeftover::ZigzagSaveWp(ZigzagSaveWp::ReturnToManual)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ZigzagSaveWp, AuxSwitchPos::High, MODE_ZIGZAG),
        AuxLeftover::ZigzagSaveWp(ZigzagSaveWp::DestinationB)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ZigzagAuto, AuxSwitchPos::High, MODE_ZIGZAG),
        AuxLeftover::ZigzagAuto(ZigzagAutoAction::Run)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ZigzagAuto, AuxSwitchPos::Low, MODE_ZIGZAG),
        AuxLeftover::ZigzagAuto(ZigzagAutoAction::Suspend)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ZigzagAuto, AuxSwitchPos::High, MODE_LOITER),
        AuxLeftover::None
    );
}

#[test]
fn remaining_copter_owned_bodies() {
    assert_eq!(
        dispatch(CopterAuxFunc::SimpleHeadingReset, AuxSwitchPos::High, 0),
        AuxLeftover::ResetSimpleHeading
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SimpleHeadingReset, AuxSwitchPos::Low, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ArmDisarmAirMode, AuxSwitchPos::High, 0),
        AuxLeftover::ArmDisarmAirMode
    );
    assert_eq!(
        dispatch(CopterAuxFunc::TurbineStart, AuxSwitchPos::High, 0),
        AuxLeftover::SetTurbineStart(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::TurbineStart, AuxSwitchPos::Low, 0),
        AuxLeftover::SetTurbineStart(false)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::TurbineStart, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::FlightmodePause, AuxSwitchPos::High, 0),
        AuxLeftover::FlightmodePause(FlightmodePauseAction::Pause)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::FlightmodePause, AuxSwitchPos::Low, 0),
        AuxLeftover::FlightmodePause(FlightmodePauseAction::Resume)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::FlightmodePause, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AutotuneTestGains, AuxSwitchPos::High, 0),
        AuxLeftover::AutotuneTestGains(AuxSwitchPos::High)
    );
    assert_eq!(
        do_aux_function_ahrs_auto_trim(AuxSwitchPos::High),
        AuxLeftover::AhrsAutoTrim(AhrsAutoTrimAction::Start)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AhrsAutoTrim, AuxSwitchPos::Low, 0),
        AuxLeftover::AhrsAutoTrim(AhrsAutoTrimAction::SaveIfRunning)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AhrsAutoTrim, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::UserFunc2, AuxSwitchPos::Middle, 0),
        AuxLeftover::CallUserFunc {
            which: 2,
            pos: AuxSwitchPos::Middle,
        }
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

fn init_dispatch(func: CopterAuxFunc, pos: AuxSwitchPos, tuning: bool) -> AuxLeftover {
    init_aux_function(
        AuxDispatch {
            func,
            pos,
            current_mode: MODE_STABILIZE,
            acro_air_mode_hook: true,
        },
        tuning,
    )
}

#[test]
fn init_aux_function_must_not_fire_noinit_from_a_boot_high() {
    assert_eq!(
        init_dispatch(CopterAuxFunc::Rtl, AuxSwitchPos::High, true),
        AuxLeftover::None,
        "RTL HIGH on boot must not set_mode"
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::Flip, AuxSwitchPos::High, true),
        AuxLeftover::None,
        "Flip HIGH on boot must not enter Flip"
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::Land, AuxSwitchPos::High, true),
        AuxLeftover::None
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::ParachuteRelease, AuxSwitchPos::High, true),
        AuxLeftover::None,
        "chute release is NoInit so boot HIGH cannot dump the canopy"
    );
}

#[test]
fn init_aux_function_runs_runnow_when_tuning_is_compiled_in() {
    assert_eq!(
        init_dispatch(CopterAuxFunc::AirMode, AuxSwitchPos::High, true),
        AuxLeftover::SetAirMode {
            air_mode: AirMode::Enabled,
            notify_acro: true,
        }
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::AcroTrainer, AuxSwitchPos::High, true),
        AuxLeftover::SetAcroTrainer(AcroTrainer::Limited)
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::MotorInterlock, AuxSwitchPos::Middle, true),
        AuxLeftover::SetMotorInterlock(true)
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::SimpleMode, AuxSwitchPos::High, true),
        AuxLeftover::SetSimpleMode(SimpleMode::Simple)
    );
}

#[test]
fn init_aux_function_falls_to_base_when_tuning_is_compiled_out() {
    assert_eq!(
        init_dispatch(CopterAuxFunc::AirMode, AuxSwitchPos::High, false),
        AuxLeftover::DelegateToBase
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::AcroTrainer, AuxSwitchPos::Low, false),
        AuxLeftover::DelegateToBase
    );
    assert_eq!(
        init_dispatch(CopterAuxFunc::Rtl, AuxSwitchPos::High, false),
        AuxLeftover::None,
        "NoInit stays quiet even when the RunNow group falls through"
    );
}

#[test]
fn acro_trainer_maps_all_three_positions() {
    assert_eq!(
        do_aux_function_acro_trainer(AuxSwitchPos::Low),
        AuxLeftover::SetAcroTrainer(AcroTrainer::Off)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AcroTrainer, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetAcroTrainer(AcroTrainer::Leveling)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AcroTrainer, AuxSwitchPos::High, 0),
        AuxLeftover::SetAcroTrainer(AcroTrainer::Limited)
    );
}

#[test]
fn runnow_high_enables_and_interlock_middle() {
    assert_eq!(
        dispatch(CopterAuxFunc::AttconFeedfwd, AuxSwitchPos::High, 0),
        AuxLeftover::SetAttconFeedfwd(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::AttconAccelLim, AuxSwitchPos::Low, 0),
        AuxLeftover::SetAttconAccelLim(false)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::MotorInterlock, AuxSwitchPos::Low, 0),
        AuxLeftover::SetMotorInterlock(false)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::MotorInterlock, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetMotorInterlock(true),
        "interlock is on above LOW, not HIGH-only"
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Rangefinder, AuxSwitchPos::High, 0),
        AuxLeftover::SetRangefinderHigh(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::ParachuteEnable, AuxSwitchPos::High, 0),
        AuxLeftover::SetParachuteEnabled(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::CustomController, AuxSwitchPos::Low, 0),
        AuxLeftover::SetCustomController(false)
    );
}

#[test]
fn parachute_3pos_and_surface_tracking_maps() {
    assert_eq!(
        dispatch(CopterAuxFunc::Parachute3pos, AuxSwitchPos::Low, 0),
        AuxLeftover::SetParachute3pos(Parachute3Pos::Disable)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Parachute3pos, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetParachute3pos(Parachute3Pos::Enable)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Parachute3pos, AuxSwitchPos::High, 0),
        AuxLeftover::SetParachute3pos(Parachute3Pos::EnableAndRelease)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SurfaceTracking, AuxSwitchPos::Low, 0),
        AuxLeftover::SetSurfaceTracking(SurfaceTracking::Ground)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SurfaceTracking, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetSurfaceTracking(SurfaceTracking::None)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::SurfaceTracking, AuxSwitchPos::High, 0),
        AuxLeftover::SetSurfaceTracking(SurfaceTracking::Ceiling)
    );
}

#[test]
fn high_low_hold_middle_is_noop() {
    assert_eq!(
        dispatch(CopterAuxFunc::PrecisionLoiter, AuxSwitchPos::High, 0),
        AuxLeftover::SetPrecisionLoiter(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::PrecisionLoiter, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::WeatherVaneEnable, AuxSwitchPos::Low, 0),
        AuxLeftover::SetWeatherVane(false)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::WeatherVaneEnable, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Inverted, AuxSwitchPos::High, 0),
        AuxLeftover::SetInverted(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Inverted, AuxSwitchPos::Middle, 0),
        AuxLeftover::None
    );
}

#[test]
fn standby_winch_and_tuning_bodies() {
    assert_eq!(
        dispatch(CopterAuxFunc::Standby, AuxSwitchPos::High, 0),
        AuxLeftover::SetStandby(true)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::Standby, AuxSwitchPos::Middle, 0),
        AuxLeftover::SetStandby(false)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::WinchEnable, AuxSwitchPos::High, 0),
        AuxLeftover::SetWinchEnable(WinchEnableAction::Stop)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::WinchEnable, AuxSwitchPos::Low, 0),
        AuxLeftover::SetWinchEnable(WinchEnableAction::Relax)
    );
    assert_eq!(
        dispatch(CopterAuxFunc::TransmitterTuning, AuxSwitchPos::High, 0),
        AuxLeftover::None,
        "tuning PWM is consumed in tuning.cpp, not here"
    );
}

#[test]
fn mode_switch_changed_rejects_out_of_range() {
    assert_eq!(
        mode_switch_changed(-1, 6, MODE_STABILIZE, false, false, 0, 0),
        ModeSwitchLeftover::Invalid
    );
    assert_eq!(
        mode_switch_changed(7, 6, MODE_RTL, false, false, 0, 0),
        ModeSwitchLeftover::Invalid
    );
    assert_eq!(
        mode_switch_changed(6, 6, MODE_LOITER, false, false, 0, 0),
        ModeSwitchLeftover::Engage {
            mode: MODE_LOITER,
            simple: Some(SimpleMode::None),
        },
        "upstream compares `>` not `>=`, so pos == num_flight_modes is in range"
    );
}

#[test]
fn mode_switch_changed_eeprom_simple_and_aux_override() {
    assert_eq!(MODE_REASON_RC_COMMAND, 1);
    assert_eq!(
        eeprom_simple_mode(0, 0b0000_0001, 0),
        SimpleMode::SuperSimple
    );
    assert_eq!(eeprom_simple_mode(1, 0, 0b0000_0010), SimpleMode::Simple);
    assert_eq!(
        eeprom_simple_mode(2, 0b0000_0100, 0b0000_0100),
        SimpleMode::SuperSimple,
        "super_simple wins when both bits are set"
    );
    assert_eq!(
        mode_switch_changed(1, 6, MODE_ALT_HOLD, false, false, 0, 0b0000_0010),
        ModeSwitchLeftover::Engage {
            mode: MODE_ALT_HOLD,
            simple: Some(SimpleMode::Simple),
        }
    );
    assert_eq!(
        mode_switch_changed(0, 6, MODE_STABILIZE, true, false, 0b0000_0001, 0),
        ModeSwitchLeftover::Engage {
            mode: MODE_STABILIZE,
            simple: None,
        },
        "a SIMPLE aux channel owns Simple; EEPROM bits must not run"
    );
    assert_eq!(
        mode_switch_changed(0, 6, MODE_STABILIZE, false, true, 0b0000_0001, 0),
        ModeSwitchLeftover::Engage {
            mode: MODE_STABILIZE,
            simple: None,
        }
    );
}

#[test]
fn flight_mode_channel_is_copter_chan_five_not_plane_eight() {
    assert_eq!(CH_MODE_DEFAULT, 5);
    assert_eq!(NUM_FLIGHT_MODES, 6);
    assert_eq!(flight_mode_channel_number(CH_MODE_DEFAULT), 5);
    assert_eq!(flight_mode_channel(CH_MODE_DEFAULT), Some(4));
    assert_eq!(
        flight_mode_channel(0),
        None,
        "FLTMODE_CH 0 disables the switch"
    );
    assert_eq!(
        flight_mode_channel(16),
        None,
        "channel 16 is >= NUM_RC_CHANNELS"
    );
    assert_eq!(flight_mode_channel(15), Some(14));
    assert_ne!(
        CH_MODE_DEFAULT,
        ap_rc::FLTMODE_CH_DEFAULT,
        "Plane default 8 must not become Copter's mode switch"
    );
}

#[test]
fn init_rc_in_maps_angle_4500_and_range_1000() {
    let map = init_rc_in_map(false);
    assert_eq!(map.roll.type_in, ControlType::Angle);
    assert_eq!(map.pitch.type_in, ControlType::Angle);
    assert_eq!(map.yaw.type_in, ControlType::Angle);
    assert_eq!(map.throttle.type_in, ControlType::Range);
    assert_eq!(map.roll.high_in, ROLL_PITCH_YAW_INPUT_MAX);
    assert_eq!(map.throttle.high_in, THROTTLE_CONTROL_RANGE);
    assert_eq!(map.roll.cal.deadzone, DEADZONE_ROLL_PITCH);
    assert_eq!(map.yaw.cal.deadzone, DEADZONE_YAW_MULTICOPTER);
    assert_eq!(map.throttle.cal.deadzone, DEADZONE_THROTTLE_MULTICOPTER);
    let heli = init_rc_in_map(true);
    assert_eq!(heli.throttle.cal.deadzone, DEADZONE_THROTTLE_HELI);
    assert_eq!(heli.yaw.cal.deadzone, DEADZONE_YAW_HELI);
    assert_eq!(heli.roll.cal.deadzone, DEADZONE_ROLL_PITCH);
}

#[test]
fn get_control_in_throttle_is_range_not_signed_stick() {
    let thr = init_rc_in_map(false).throttle;
    assert_eq!(get_control_in(&thr, 1100), 0);
    assert_eq!(get_control_in(&thr, 1130), 0, "min+dz is still zero");
    assert_eq!(get_control_in(&thr, 1900), 1000);
    // float 1000*370/770 = 480.519… truncated, matching int16 control_in
    assert_eq!(get_control_in(&thr, 1500), 480);
    assert_eq!(
        get_control_mid(&thr),
        480,
        "get_control_mid is int32 1000*370/770"
    );
    assert!(
        (get_control_in_zero_dz(&thr, 1500) - 500.0).abs() < 1e-4,
        "zero-dz mid-stick is 500, not the deadzoned 480"
    );
    assert_eq!(get_throttle_mid(&thr, None), 480);
    assert_eq!(get_throttle_mid(&thr, Some(512)), 512);
}

#[test]
fn get_control_in_angle_is_centidegrees_about_trim() {
    let roll = init_rc_in_map(false).roll;
    assert_eq!(get_control_in(&roll, 1500), 0);
    assert_eq!(get_control_in(&roll, 1520), 0, "trim+dz is still zero");
    assert_eq!(get_control_in(&roll, 1900), 4500);
    assert_eq!(get_control_in(&roll, 1100), -4500);
    assert_eq!(
        get_control_mid(&roll),
        0,
        "ANGLE get_control_mid is always 0"
    );
    // 4500 * 1 / 380 ≈ 11.84 → 11
    assert_eq!(get_control_in(&roll, 1521), 11);
    assert!((get_control_in_zero_dz(&roll, 1700) - 2250.0).abs() < 1e-3);
}

#[test]
fn reversed_range_mirrors_pwm_before_control_in() {
    let mut thr = init_rc_in_map(false).throttle;
    thr.cal.reversed = true;
    assert_eq!(
        get_control_in(&thr, 1100),
        1000,
        "reversed min PWM is full RANGE"
    );
    assert_eq!(get_control_in(&thr, 1900), 0);
}

#[test]
fn save_trim_skips_stick_lean_when_auto_trim_is_running() {
    assert_eq!(
        save_trim(false),
        SaveTrimLeftover {
            auto_trim_running: false,
            need_pilot_lean: true,
            persist: true,
        }
    );
    assert_eq!(
        save_trim(true),
        SaveTrimLeftover {
            auto_trim_running: false,
            need_pilot_lean: false,
            persist: true,
        },
        "running auto-trim already applied the increments; do not sample sticks"
    );
    assert_eq!(
        auto_trim_cancel(),
        SaveTrimLeftover {
            auto_trim_running: false,
            need_pilot_lean: false,
            persist: false,
        }
    );
}

#[test]
fn auto_trim_run_gates_and_divides_att_target_by_twenty() {
    assert_eq!(
        auto_trim_run(false, true, false, 0.4, -0.2),
        AutoTrimRunLeftover::Idle
    );
    assert_eq!(
        auto_trim_run(true, false, false, 0.4, -0.2),
        AutoTrimRunLeftover::Cancel {
            reason: AutoTrimCancelReason::ModeDisallows,
        }
    );
    assert_eq!(
        auto_trim_run(true, true, true, 0.4, -0.2),
        AutoTrimRunLeftover::Cancel {
            reason: AutoTrimCancelReason::LandCompleteMaybe,
        }
    );
    match auto_trim_run(true, true, false, 0.4, -0.2) {
        AutoTrimRunLeftover::Apply {
            roll_trim_adj_rad,
            pitch_trim_adj_rad,
        } => {
            assert!((roll_trim_adj_rad - 0.02).abs() < 1e-6);
            assert!((pitch_trim_adj_rad + 0.01).abs() < 1e-6);
        }
        other => panic!("expected Apply, got {other:?}"),
    }
}
