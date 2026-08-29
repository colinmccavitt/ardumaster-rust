//! Copter aux-switch leftover, upstream `ArduCopter/RC_Channel_Copter.cpp`.
//!
//! Tracked as **COP-022**. The shared [`ap_rc`] crate already latches PWM into
//! LOW / MIDDLE / HIGH. What lives here is the vehicle leftover: which
//! `RCn_OPTION` values Copter handles, whether they need init, and what
//! [`do_aux_function`] / [`do_aux_function_change_mode`] ask the vehicle to do.
//!
//! # Mode-change is an edge, not a level
//!
//! Almost every flight-mode option shares [`do_aux_function_change_mode`]:
//! HIGH engages the mode (`ModeReason::AUX_FUNCTION`); LOW or MIDDLE only
//! resets the flight-mode switch *if the aircraft is already in that mode*.
//! A port that treated LOW as "leave the mode" would yank the aircraft out
//! of RTL the pilot had selected on the six-position switch.
//!
//! [`CopterAuxFunc::Flip`] is the exception that proves the rule. HIGH
//! enters Flip; releasing the switch does **not** reset the mode switch.
//! Folding Flip into the shared helper would bounce back to Stabilize mid-flip.
//!
//! # SIMPLE is two options with different maps
//!
//! `SIMPLE_MODE` is a 2-position feel: LOW is off, MIDDLE or HIGH is Simple.
//! `SUPERSIMPLE_MODE` is a real 3-position map (off / Simple / SuperSimple).
//! Sharing one decoder would give SuperSimple on MIDDLE.

use ap_rc::AuxSwitchPos;

/// Upstream `ModeReason::AUX_FUNCTION`.
pub const MODE_REASON_AUX_FUNCTION: u8 = 53;

/// `Mode::Number::STABILIZE`.
pub const MODE_STABILIZE: u8 = 0;
/// `Mode::Number::ACRO`.
pub const MODE_ACRO: u8 = 1;
/// `Mode::Number::ALT_HOLD`.
pub const MODE_ALT_HOLD: u8 = 2;
/// `Mode::Number::AUTO`.
pub const MODE_AUTO: u8 = 3;
/// `Mode::Number::GUIDED`.
pub const MODE_GUIDED: u8 = 4;
/// `Mode::Number::LOITER`.
pub const MODE_LOITER: u8 = 5;
/// `Mode::Number::RTL`.
pub const MODE_RTL: u8 = 6;
/// `Mode::Number::CIRCLE`.
pub const MODE_CIRCLE: u8 = 7;
/// `Mode::Number::LAND`.
pub const MODE_LAND: u8 = 9;
/// `Mode::Number::DRIFT`.
pub const MODE_DRIFT: u8 = 11;
/// `Mode::Number::FLIP`.
pub const MODE_FLIP: u8 = 14;
/// `Mode::Number::AUTOTUNE`.
pub const MODE_AUTOTUNE: u8 = 15;
/// `Mode::Number::POSHOLD`.
pub const MODE_POSHOLD: u8 = 16;
/// `Mode::Number::BRAKE`.
pub const MODE_BRAKE: u8 = 17;
/// `Mode::Number::THROW`.
pub const MODE_THROW: u8 = 18;
/// `Mode::Number::SMART_RTL`.
pub const MODE_SMART_RTL: u8 = 21;
/// `Mode::Number::FLOWHOLD`.
pub const MODE_FLOWHOLD: u8 = 22;
/// `Mode::Number::FOLLOW`.
pub const MODE_FOLLOW: u8 = 23;
/// `Mode::Number::ZIGZAG`.
pub const MODE_ZIGZAG: u8 = 24;
/// `Mode::Number::AUTO_RTL`.
pub const MODE_AUTO_RTL: u8 = 27;
/// `Mode::Number::TURTLE`.
pub const MODE_TURTLE: u8 = 28;

/// Upstream `THR_BEHAVE_FEEDBACK_FROM_MID_STICK` in `defines.h`.
pub const THR_BEHAVE_FEEDBACK_FROM_MID_STICK: u8 = 1 << 0;

/// `RCn_OPTION` values `RC_Channel_Copter` switches on.
///
/// The shared [`ap_rc::AuxFunc`] stub keeps the Plane latch set. Copter's
/// table is the leftover: these numbers are what a Copter `RCn_OPTION`
/// parameter actually stores.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum CopterAuxFunc {
    /// `AUX_FUNC::FLIP` = 2.
    Flip = 2,
    /// `AUX_FUNC::SIMPLE_MODE` = 3.
    SimpleMode = 3,
    /// `AUX_FUNC::RTL` = 4.
    Rtl = 4,
    /// `AUX_FUNC::SAVE_TRIM` = 5.
    SaveTrim = 5,
    /// `AUX_FUNC::SAVE_WP` = 7.
    SaveWp = 7,
    /// `AUX_FUNC::RANGEFINDER` = 10.
    Rangefinder = 10,
    /// `AUX_FUNC::RESETTOARMEDYAW` = 12 (unused).
    ResetToArmedYaw = 12,
    /// `AUX_FUNC::SUPERSIMPLE_MODE` = 13.
    SuperSimpleMode = 13,
    /// `AUX_FUNC::ACRO_TRAINER` = 14.
    AcroTrainer = 14,
    /// `AUX_FUNC::AUTO` = 16.
    Auto = 16,
    /// `AUX_FUNC::AUTOTUNE_MODE` = 17.
    AutotuneMode = 17,
    /// `AUX_FUNC::LAND` = 18.
    Land = 18,
    /// `AUX_FUNC::PARACHUTE_ENABLE` = 21.
    ParachuteEnable = 21,
    /// `AUX_FUNC::PARACHUTE_RELEASE` = 22.
    ParachuteRelease = 22,
    /// `AUX_FUNC::PARACHUTE_3POS` = 23.
    Parachute3pos = 23,
    /// `AUX_FUNC::ATTCON_FEEDFWD` = 25.
    AttconFeedfwd = 25,
    /// `AUX_FUNC::ATTCON_ACCEL_LIM` = 26.
    AttconAccelLim = 26,
    /// `AUX_FUNC::MOTOR_INTERLOCK` = 32.
    MotorInterlock = 32,
    /// `AUX_FUNC::BRAKE` = 33.
    Brake = 33,
    /// `AUX_FUNC::THROW` = 37.
    Throw = 37,
    /// `AUX_FUNC::PRECISION_LOITER` = 39.
    PrecisionLoiter = 39,
    /// `AUX_FUNC::SMART_RTL` = 42.
    SmartRtl = 42,
    /// `AUX_FUNC::INVERTED` = 43.
    Inverted = 43,
    /// `AUX_FUNC::WINCH_ENABLE` = 44.
    WinchEnable = 44,
    /// `AUX_FUNC::WINCH_CONTROL` = 45.
    WinchControl = 45,
    /// `AUX_FUNC::USER_FUNC1` = 47.
    UserFunc1 = 47,
    /// `AUX_FUNC::USER_FUNC2` = 48.
    UserFunc2 = 48,
    /// `AUX_FUNC::USER_FUNC3` = 49.
    UserFunc3 = 49,
    /// `AUX_FUNC::ACRO` = 52.
    Acro = 52,
    /// `AUX_FUNC::GUIDED` = 55.
    Guided = 55,
    /// `AUX_FUNC::LOITER` = 56.
    Loiter = 56,
    /// `AUX_FUNC::FOLLOW` = 57.
    Follow = 57,
    /// `AUX_FUNC::ZIGZAG` = 60.
    Zigzag = 60,
    /// `AUX_FUNC::ZIGZAG_SaveWP` = 61.
    ZigzagSaveWp = 61,
    /// `AUX_FUNC::STABILIZE` = 68.
    Stabilize = 68,
    /// `AUX_FUNC::POSHOLD` = 69.
    Poshold = 69,
    /// `AUX_FUNC::ALTHOLD` = 70.
    Althold = 70,
    /// `AUX_FUNC::FLOWHOLD` = 71.
    Flowhold = 71,
    /// `AUX_FUNC::CIRCLE` = 72.
    Circle = 72,
    /// `AUX_FUNC::DRIFT` = 73.
    Drift = 73,
    /// `AUX_FUNC::SURFACE_TRACKING` = 75.
    SurfaceTracking = 75,
    /// `AUX_FUNC::STANDBY` = 76.
    Standby = 76,
    /// `AUX_FUNC::ZIGZAG_Auto` = 83.
    ZigzagAuto = 83,
    /// `AUX_FUNC::AIRMODE` = 84.
    AirMode = 84,
    /// `AUX_FUNC::AUTO_RTL` = 99.
    AutoRtl = 99,
    /// `AUX_FUNC::CUSTOM_CONTROLLER` = 109.
    CustomController = 109,
    /// `AUX_FUNC::TURTLE` = 151.
    Turtle = 151,
    /// `AUX_FUNC::SIMPLE_HEADING_RESET` = 152.
    SimpleHeadingReset = 152,
    /// `AUX_FUNC::ARMDISARM_AIRMODE` = 154.
    ArmDisarmAirMode = 154,
    /// `AUX_FUNC::FORCEFLYING` = 159.
    ForceFlying = 159,
    /// `AUX_FUNC::WEATHER_VANE_ENABLE` = 160.
    WeatherVaneEnable = 160,
    /// `AUX_FUNC::TURBINE_START` = 161.
    TurbineStart = 161,
    /// `AUX_FUNC::FLIGHTMODE_PAUSE` = 178.
    FlightmodePause = 178,
    /// `AUX_FUNC::AUTOTUNE_TEST_GAINS` = 180.
    AutotuneTestGains = 180,
    /// `AUX_FUNC::AHRS_AUTO_TRIM` = 182.
    AhrsAutoTrim = 182,
    /// `AUX_FUNC::TRANSMITTER_TUNING` = 219.
    TransmitterTuning = 219,
    /// `AUX_FUNC::TRANSMITTER_TUNING2` = 220.
    TransmitterTuning2 = 220,
}

impl CopterAuxFunc {
    /// Decode a stored `RCn_OPTION`. Unknown codes are `None`.
    #[must_use]
    pub const fn from_option(value: u16) -> Option<Self> {
        match value {
            2 => Some(Self::Flip),
            3 => Some(Self::SimpleMode),
            4 => Some(Self::Rtl),
            5 => Some(Self::SaveTrim),
            7 => Some(Self::SaveWp),
            10 => Some(Self::Rangefinder),
            12 => Some(Self::ResetToArmedYaw),
            13 => Some(Self::SuperSimpleMode),
            14 => Some(Self::AcroTrainer),
            16 => Some(Self::Auto),
            17 => Some(Self::AutotuneMode),
            18 => Some(Self::Land),
            21 => Some(Self::ParachuteEnable),
            22 => Some(Self::ParachuteRelease),
            23 => Some(Self::Parachute3pos),
            25 => Some(Self::AttconFeedfwd),
            26 => Some(Self::AttconAccelLim),
            32 => Some(Self::MotorInterlock),
            33 => Some(Self::Brake),
            37 => Some(Self::Throw),
            39 => Some(Self::PrecisionLoiter),
            42 => Some(Self::SmartRtl),
            43 => Some(Self::Inverted),
            44 => Some(Self::WinchEnable),
            45 => Some(Self::WinchControl),
            47 => Some(Self::UserFunc1),
            48 => Some(Self::UserFunc2),
            49 => Some(Self::UserFunc3),
            52 => Some(Self::Acro),
            55 => Some(Self::Guided),
            56 => Some(Self::Loiter),
            57 => Some(Self::Follow),
            60 => Some(Self::Zigzag),
            61 => Some(Self::ZigzagSaveWp),
            68 => Some(Self::Stabilize),
            69 => Some(Self::Poshold),
            70 => Some(Self::Althold),
            71 => Some(Self::Flowhold),
            72 => Some(Self::Circle),
            73 => Some(Self::Drift),
            75 => Some(Self::SurfaceTracking),
            76 => Some(Self::Standby),
            83 => Some(Self::ZigzagAuto),
            84 => Some(Self::AirMode),
            99 => Some(Self::AutoRtl),
            109 => Some(Self::CustomController),
            151 => Some(Self::Turtle),
            152 => Some(Self::SimpleHeadingReset),
            154 => Some(Self::ArmDisarmAirMode),
            159 => Some(Self::ForceFlying),
            160 => Some(Self::WeatherVaneEnable),
            161 => Some(Self::TurbineStart),
            178 => Some(Self::FlightmodePause),
            180 => Some(Self::AutotuneTestGains),
            182 => Some(Self::AhrsAutoTrim),
            219 => Some(Self::TransmitterTuning),
            220 => Some(Self::TransmitterTuning2),
            _ => None,
        }
    }

    /// Flight mode this option would engage, if it is a mode-change switch.
    ///
    /// [`CopterAuxFunc::Flip`] is omitted: Flip is not routed through
    /// [`do_aux_function_change_mode`].
    #[must_use]
    pub const fn change_mode_number(self) -> Option<u8> {
        match self {
            Self::Rtl => Some(MODE_RTL),
            Self::Auto => Some(MODE_AUTO),
            Self::AutotuneMode => Some(MODE_AUTOTUNE),
            Self::Land => Some(MODE_LAND),
            Self::Brake => Some(MODE_BRAKE),
            Self::Throw => Some(MODE_THROW),
            Self::SmartRtl => Some(MODE_SMART_RTL),
            Self::Acro => Some(MODE_ACRO),
            Self::Guided => Some(MODE_GUIDED),
            Self::Loiter => Some(MODE_LOITER),
            Self::Follow => Some(MODE_FOLLOW),
            Self::Zigzag => Some(MODE_ZIGZAG),
            Self::Stabilize => Some(MODE_STABILIZE),
            Self::Poshold => Some(MODE_POSHOLD),
            Self::Althold => Some(MODE_ALT_HOLD),
            Self::Flowhold => Some(MODE_FLOWHOLD),
            Self::Circle => Some(MODE_CIRCLE),
            Self::Drift => Some(MODE_DRIFT),
            Self::AutoRtl => Some(MODE_AUTO_RTL),
            Self::Turtle => Some(MODE_TURTLE),
            _ => None,
        }
    }
}

/// What `init_aux_function` does with a Copter option.
///
/// The big "do not initialise" list is not a no-op of convenience: those
/// functions must not fire on boot from a switch that happens to be HIGH.
/// The `RunNow` list *does* fire at init (airmode, interlock, simple mode)
/// so the vehicle matches the switch before the first `read_aux`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitAuxKind {
    /// The option is listed and the switch body is `break`.
    NoInit,
    /// `run_aux_function(..., Source::INIT, ...)`.
    RunNow,
    /// Fall through to `RC_Channel::init_aux_function`.
    DelegateToBase,
}

/// Classify `RC_Channel_Copter::init_aux_function`.
///
/// `transmitter_tuning_enabled` is `AP_RC_TRANSMITTER_TUNING_ENABLED`. When
/// that compile switch is off, the `RunNow` cases lose their shared `break`
/// and fall through to the base — the leftover records that, rather than
/// pretending the grouping is a runtime table.
#[must_use]
pub const fn init_aux_kind(func: CopterAuxFunc, transmitter_tuning_enabled: bool) -> InitAuxKind {
    match func {
        CopterAuxFunc::Althold
        | CopterAuxFunc::Auto
        | CopterAuxFunc::AutotuneMode
        | CopterAuxFunc::AutotuneTestGains
        | CopterAuxFunc::Brake
        | CopterAuxFunc::Circle
        | CopterAuxFunc::Drift
        | CopterAuxFunc::Flip
        | CopterAuxFunc::Flowhold
        | CopterAuxFunc::Follow
        | CopterAuxFunc::Guided
        | CopterAuxFunc::Land
        | CopterAuxFunc::Loiter
        | CopterAuxFunc::ParachuteRelease
        | CopterAuxFunc::Poshold
        | CopterAuxFunc::ResetToArmedYaw
        | CopterAuxFunc::Rtl
        | CopterAuxFunc::SaveTrim
        | CopterAuxFunc::SaveWp
        | CopterAuxFunc::SmartRtl
        | CopterAuxFunc::Stabilize
        | CopterAuxFunc::Throw
        | CopterAuxFunc::UserFunc1
        | CopterAuxFunc::UserFunc2
        | CopterAuxFunc::UserFunc3
        | CopterAuxFunc::WinchControl
        | CopterAuxFunc::Zigzag
        | CopterAuxFunc::ZigzagAuto
        | CopterAuxFunc::ZigzagSaveWp
        | CopterAuxFunc::Acro
        | CopterAuxFunc::AutoRtl
        | CopterAuxFunc::Turtle
        | CopterAuxFunc::SimpleHeadingReset
        | CopterAuxFunc::ArmDisarmAirMode
        | CopterAuxFunc::TurbineStart
        | CopterAuxFunc::FlightmodePause
        | CopterAuxFunc::AhrsAutoTrim => InitAuxKind::NoInit,
        CopterAuxFunc::AcroTrainer
        | CopterAuxFunc::AttconAccelLim
        | CopterAuxFunc::AttconFeedfwd
        | CopterAuxFunc::Inverted
        | CopterAuxFunc::MotorInterlock
        | CopterAuxFunc::Parachute3pos
        | CopterAuxFunc::ParachuteEnable
        | CopterAuxFunc::PrecisionLoiter
        | CopterAuxFunc::Rangefinder
        | CopterAuxFunc::SimpleMode
        | CopterAuxFunc::Standby
        | CopterAuxFunc::SuperSimpleMode
        | CopterAuxFunc::SurfaceTracking
        | CopterAuxFunc::WinchEnable
        | CopterAuxFunc::AirMode
        | CopterAuxFunc::ForceFlying
        | CopterAuxFunc::CustomController
        | CopterAuxFunc::WeatherVaneEnable => {
            if transmitter_tuning_enabled {
                InitAuxKind::RunNow
            } else {
                InitAuxKind::DelegateToBase
            }
        }
        CopterAuxFunc::TransmitterTuning | CopterAuxFunc::TransmitterTuning2 => {
            if transmitter_tuning_enabled {
                InitAuxKind::RunNow
            } else {
                InitAuxKind::DelegateToBase
            }
        }
    }
}

/// Upstream `Copter::SimpleMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SimpleMode {
    /// 0 — simple / super-simple off.
    None = 0,
    /// 1 — Simple.
    Simple = 1,
    /// 2 — SuperSimple.
    SuperSimple = 2,
}

/// Upstream `AirMode` in `defines.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AirMode {
    /// `AIRMODE_NONE`. Never written by the air-mode aux switch.
    None = 0,
    /// `AIRMODE_DISABLED`. LOW on the air-mode switch.
    Disabled = 1,
    /// `AIRMODE_ENABLED`. HIGH on the air-mode switch.
    Enabled = 2,
}

/// What `do_aux_function` asked the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuxLeftover {
    /// Handled; no vehicle write (MIDDLE on air-mode, Flip not HIGH, …).
    None,
    /// `copter.set_mode(mode, ModeReason::AUX_FUNCTION)`.
    SetMode {
        /// `Mode::Number` to enter.
        mode: u8,
    },
    /// `rc().reset_mode_switch()` — only when already in the option's mode.
    ResetModeSwitch,
    /// `copter.set_simple_mode`.
    SetSimpleMode(SimpleMode),
    /// `do_aux_function_change_air_mode` plus the optional Acro hook.
    SetAirMode {
        /// New `copter.air_mode`.
        air_mode: AirMode,
        /// `mode_acro.air_mode_aux_changed()` when Acro is compiled in.
        notify_acro: bool,
    },
    /// `do_aux_function_change_force_flying`.
    SetForceFlying(bool),
    /// Copter owns this option; a later COP-022 slice fills the body.
    Pending,
    /// `return RC_Channel::do_aux_function(trigger)`.
    DelegateToBase,
}

/// Inputs [`do_aux_function`] reads besides the trigger itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxDispatch {
    /// Assigned `RCn_OPTION`.
    pub func: CopterAuxFunc,
    /// Debounced switch position.
    pub pos: AuxSwitchPos,
    /// `flightmode->mode_number()`. Used only by mode-change LOW/MIDDLE.
    pub current_mode: u8,
    /// `MODE_ACRO_ENABLED && FRAME_CONFIG != HELI_FRAME`.
    pub acro_air_mode_hook: bool,
}

/// `RC_Channel_Copter::do_aux_function_change_mode`.
///
/// HIGH always asks for the mode. Any other position only asks to restore
/// the flight-mode switch when the aircraft is *already* in that mode.
#[must_use]
pub const fn do_aux_function_change_mode(
    mode: u8,
    pos: AuxSwitchPos,
    current_mode: u8,
) -> AuxLeftover {
    match pos {
        AuxSwitchPos::High => AuxLeftover::SetMode { mode },
        AuxSwitchPos::Low | AuxSwitchPos::Middle => {
            if current_mode == mode {
                AuxLeftover::ResetModeSwitch
            } else {
                AuxLeftover::None
            }
        }
    }
}

/// `RC_Channel_Copter::do_aux_function_change_air_mode`.
///
/// MIDDLE is a no-op: a 3-position switch parked in the centre must not
/// toggle airmode every time `read_aux` re-fires the last position.
#[must_use]
pub const fn do_aux_function_change_air_mode(pos: AuxSwitchPos) -> Option<AirMode> {
    match pos {
        AuxSwitchPos::High => Some(AirMode::Enabled),
        AuxSwitchPos::Low => Some(AirMode::Disabled),
        AuxSwitchPos::Middle => None,
    }
}

/// `RC_Channel_Copter::do_aux_function_change_force_flying`.
#[must_use]
pub const fn do_aux_function_change_force_flying(pos: AuxSwitchPos) -> Option<bool> {
    match pos {
        AuxSwitchPos::High => Some(true),
        AuxSwitchPos::Low => Some(false),
        AuxSwitchPos::Middle => None,
    }
}

/// `RC_Channel_Copter::do_aux_function` leftover for this slice.
///
/// Mode-change options, Flip, Simple / SuperSimple, AirMode and ForceFlying
/// are the first real Copter dispatch. Other Copter-owned cases return
/// [`AuxLeftover::Pending`]. Codes this file does not switch on are not in
/// [`CopterAuxFunc`]; the caller treats `from_option == None` as
/// [`AuxLeftover::DelegateToBase`].
#[must_use]
pub const fn do_aux_function(dispatch: AuxDispatch) -> AuxLeftover {
    if let Some(mode) = dispatch.func.change_mode_number() {
        return do_aux_function_change_mode(mode, dispatch.pos, dispatch.current_mode);
    }
    match dispatch.func {
        CopterAuxFunc::Flip => {
            if matches!(dispatch.pos, AuxSwitchPos::High) {
                AuxLeftover::SetMode { mode: MODE_FLIP }
            } else {
                AuxLeftover::None
            }
        }
        CopterAuxFunc::SimpleMode => AuxLeftover::SetSimpleMode(match dispatch.pos {
            AuxSwitchPos::Low => SimpleMode::None,
            AuxSwitchPos::Middle | AuxSwitchPos::High => SimpleMode::Simple,
        }),
        CopterAuxFunc::SuperSimpleMode => AuxLeftover::SetSimpleMode(match dispatch.pos {
            AuxSwitchPos::Low => SimpleMode::None,
            AuxSwitchPos::Middle => SimpleMode::Simple,
            AuxSwitchPos::High => SimpleMode::SuperSimple,
        }),
        CopterAuxFunc::AirMode => match do_aux_function_change_air_mode(dispatch.pos) {
            Some(air_mode) => AuxLeftover::SetAirMode {
                air_mode,
                notify_acro: dispatch.acro_air_mode_hook,
            },
            None => AuxLeftover::None,
        },
        CopterAuxFunc::ForceFlying => match do_aux_function_change_force_flying(dispatch.pos) {
            Some(force) => AuxLeftover::SetForceFlying(force),
            None => AuxLeftover::None,
        },
        _ => AuxLeftover::Pending,
    }
}

/// Dispatch a raw `RCn_OPTION`. Unknown codes fall to the RC base leftover.
#[must_use]
pub const fn do_aux_function_option(
    option: u16,
    pos: AuxSwitchPos,
    current_mode: u8,
    acro_air_mode_hook: bool,
) -> AuxLeftover {
    match CopterAuxFunc::from_option(option) {
        Some(func) => do_aux_function(AuxDispatch {
            func,
            pos,
            current_mode,
            acro_air_mode_hook,
        }),
        None => AuxLeftover::DelegateToBase,
    }
}

/// `RC_Channels_Copter::in_rc_failsafe` — `copter.failsafe.radio`.
#[must_use]
pub const fn in_rc_failsafe(radio_failsafe: bool) -> bool {
    radio_failsafe
}

/// `RC_Channels_Copter::has_valid_input`.
///
/// A pending radio-failsafe counter (`radio_counter != 0`) is already
/// invalid, even before `failsafe.radio` latches. Skipping that check
/// would keep flying on the last good pulses while the failsafe is
/// counting down.
#[must_use]
pub const fn has_valid_input(
    radio_failsafe: bool,
    radio_counter: u8,
    base_has_valid: bool,
) -> bool {
    if in_rc_failsafe(radio_failsafe) {
        return false;
    }
    if radio_counter != 0 {
        return false;
    }
    base_has_valid
}

/// `RC_Channels_Copter::arming_check_throttle`.
///
/// A sprung (centre-detent) throttle is Copter's own arming check. The
/// library throttle-zero test must not run: mid-stick is a valid idle, not
/// an uncalibrated high throttle.
#[must_use]
pub const fn arming_check_throttle(throttle_behavior: u8, base_check: bool) -> bool {
    if (throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0 {
        false
    } else {
        base_check
    }
}

/// Which stick `RC_Channels_Copter::get_arming_channel` returns.
///
/// Copter arms on yaw, not throttle. A port that reused Plane's throttle
/// arming channel would require the collective to be the arming stick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmingChannel {
    /// `copter.channel_yaw`.
    Yaw,
}

/// `RC_Channels_Copter::get_arming_channel`.
#[must_use]
pub const fn get_arming_channel() -> ArmingChannel {
    ArmingChannel::Yaw
}
