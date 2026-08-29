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

use ap_rc::{
    flight_mode_channel_index, reverse_range_pwm, AuxSwitchPos, RcChannel, NUM_RC_CHANNELS,
};

/// Upstream `ModeReason::AUX_FUNCTION`.
pub const MODE_REASON_AUX_FUNCTION: u8 = 53;

/// Upstream `ModeReason::RC_COMMAND` — flight-mode switch, not an aux option.
pub const MODE_REASON_RC_COMMAND: u8 = 1;

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
    /// `g.acro_trainer.set` from `AUX_FUNC::ACRO_TRAINER`.
    SetAcroTrainer(AcroTrainer),
    /// `attitude_control->bf_feedforward(ch_flag == HIGH)`.
    SetAttconFeedfwd(bool),
    /// `attitude_control->accel_limiting(ch_flag == HIGH)`.
    SetAttconAccelLim(bool),
    /// `ap.motor_interlock_switch` — HIGH or MIDDLE is on.
    SetMotorInterlock(bool),
    /// `rangefinder_state.enabled`. Vehicle ANDs HIGH with a downward sensor.
    SetRangefinderHigh(bool),
    /// `parachute.enabled(ch_flag == HIGH)`.
    SetParachuteEnabled(bool),
    /// `AUX_FUNC::PARACHUTE_3POS`.
    SetParachute3pos(Parachute3Pos),
    /// `mode_loiter.set_precision_loiter_enabled`. MIDDLE is a no-op.
    SetPrecisionLoiter(bool),
    /// `copter.standby_active` — HIGH is on, anything else is off.
    SetStandby(bool),
    /// `surface_tracking.set_surface`.
    SetSurfaceTracking(SurfaceTracking),
    /// `g2.winch` stop-vs-relax from `WINCH_ENABLE`.
    SetWinchEnable(WinchEnableAction),
    /// `custom_control.set_custom_controller(ch_flag == HIGH)`.
    SetCustomController(bool),
    /// `weathervane.allow_weathervaning`. MIDDLE is a no-op.
    SetWeatherVane(bool),
    /// `attitude_control->set_inverted_flight`. Vehicle checks `allows_inverted` on true.
    SetInverted(bool),
    /// `parachute_manual_release` from `AUX_FUNC::PARACHUTE_RELEASE`.
    ReleaseParachute,
    /// `g2.rc_channels.save_trim` from `AUX_FUNC::SAVE_TRIM`.
    ///
    /// Vehicle still requires `allows_save_trim` and zero throttle.
    SaveTrim,
    /// `AUX_FUNC::SAVE_WP`. Vehicle still rejects Auto, disarmed, and a
    /// first waypoint at zero throttle.
    SaveWp,
    /// `userhook_auxSwitchN` from `USER_FUNC1/2/3`.
    CallUserFunc {
        /// 1, 2 or 3.
        which: u8,
        /// Debounced switch position passed to the hook.
        pos: AuxSwitchPos,
    },
    /// `mode_zigzag.save_or_move_to_destination` / `return_to_manual_control`.
    ZigzagSaveWp(ZigzagSaveWp),
    /// `mode_zigzag.run_auto` / `suspend_auto`.
    ZigzagAuto(ZigzagAutoAction),
    /// `init_simple_bearing` from `SIMPLE_HEADING_RESET`.
    ResetSimpleHeading,
    /// `RC_Channel::do_aux_function_armdisarm` plus `armed_with_airmode_switch`.
    ArmDisarmAirMode,
    /// `motors->set_turb_start` from `TURBINE_START` (heli).
    SetTurbineStart(bool),
    /// `flightmode->pause` / `resume` from `FLIGHTMODE_PAUSE`.
    FlightmodePause(FlightmodePauseAction),
    /// `mode_autotune.autotune.do_aux_function`.
    AutotuneTestGains(AuxSwitchPos),
    /// `RC_Channels_Copter::do_aux_function_ahrs_auto_trim`.
    AhrsAutoTrim(AhrsAutoTrimAction),
    /// Copter owns this option; a later COP-022 slice fills the body.
    Pending,
    /// `return RC_Channel::do_aux_function(trigger)`.
    DelegateToBase,
}

/// `ModeAcro::Trainer` written by `AUX_FUNC::ACRO_TRAINER`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AcroTrainer {
    /// 0 — `Trainer::OFF`. LOW.
    Off = 0,
    /// 1 — `Trainer::LEVELING`. MIDDLE.
    Leveling = 1,
    /// 2 — `Trainer::LIMITED`. HIGH.
    Limited = 2,
}

/// `AUX_FUNC::PARACHUTE_3POS` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parachute3Pos {
    /// LOW — disable.
    Disable,
    /// MIDDLE — enable, do not release.
    Enable,
    /// HIGH — enable and `parachute_manual_release`.
    EnableAndRelease,
}

/// `Copter::SurfaceTracking::Surface`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SurfaceTracking {
    /// MIDDLE — tracking off.
    None = 0,
    /// LOW — ground.
    Ground = 1,
    /// HIGH — ceiling.
    Ceiling = 2,
}

/// `AUX_FUNC::WINCH_ENABLE` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WinchEnableAction {
    /// HIGH — `set_desired_rate(0)`.
    Stop,
    /// LOW or MIDDLE — `relax()`.
    Relax,
}

/// `AUX_FUNC::ZIGZAG_SaveWP` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigzagSaveWp {
    /// LOW — `save_or_move_to_destination(Destination::A)`.
    DestinationA,
    /// HIGH — `save_or_move_to_destination(Destination::B)`.
    DestinationB,
    /// MIDDLE — `return_to_manual_control(false)`.
    ReturnToManual,
}

/// `AUX_FUNC::ZIGZAG_Auto` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigzagAutoAction {
    /// HIGH — `run_auto`.
    Run,
    /// LOW or MIDDLE — `suspend_auto`.
    Suspend,
}

/// `AUX_FUNC::FLIGHTMODE_PAUSE` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightmodePauseAction {
    /// HIGH — `flightmode->pause()`.
    Pause,
    /// LOW — `flightmode->resume()`.
    Resume,
}

/// `AUX_FUNC::AHRS_AUTO_TRIM` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsAutoTrimAction {
    /// HIGH — start; vehicle still requires `allows_auto_trim`.
    Start,
    /// LOW — `save_trim` only if auto-trim is already running.
    SaveIfRunning,
}

/// Inputs [`do_aux_function`] reads besides the trigger itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxDispatch {
    /// Assigned `RCn_OPTION`.
    pub func: CopterAuxFunc,
    /// Debounced switch position.
    pub pos: AuxSwitchPos,
    /// `flightmode->mode_number()`. Mode-change LOW/MIDDLE and ZigZag-only options.
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

/// `RC_Channel_Copter::do_aux_function_acro_trainer`.
///
/// LOW / MIDDLE / HIGH are Off / Leveling / Limited. Folding this into a
/// HIGH-only toggle would leave LEVELING unreachable.
#[must_use]
pub const fn do_aux_function_acro_trainer(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetAcroTrainer(match pos {
        AuxSwitchPos::Low => AcroTrainer::Off,
        AuxSwitchPos::Middle => AcroTrainer::Leveling,
        AuxSwitchPos::High => AcroTrainer::Limited,
    })
}

/// HIGH-is-on leftover shared by feed-forward, accel-limit, parachute-enable
/// and the custom controller.
#[must_use]
pub const fn do_aux_function_high_enables(pos: AuxSwitchPos) -> bool {
    matches!(pos, AuxSwitchPos::High)
}

/// `AUX_FUNC::MOTOR_INTERLOCK` — on in HIGH or MIDDLE.
///
/// The vehicle still skips the write in heli passthrough RSC; the leftover
/// reports the switch, not the rotor-speed mode.
#[must_use]
pub const fn do_aux_function_motor_interlock(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetMotorInterlock(matches!(pos, AuxSwitchPos::High | AuxSwitchPos::Middle))
}

/// `AUX_FUNC::PARACHUTE_3POS`.
#[must_use]
pub const fn do_aux_function_parachute_3pos(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetParachute3pos(match pos {
        AuxSwitchPos::Low => Parachute3Pos::Disable,
        AuxSwitchPos::Middle => Parachute3Pos::Enable,
        AuxSwitchPos::High => Parachute3Pos::EnableAndRelease,
    })
}

/// HIGH / LOW write a bool; MIDDLE is a no-op (precision loiter, weathervane,
/// inverted).
#[must_use]
pub const fn do_aux_function_high_low_hold(pos: AuxSwitchPos) -> Option<bool> {
    match pos {
        AuxSwitchPos::High => Some(true),
        AuxSwitchPos::Low => Some(false),
        AuxSwitchPos::Middle => None,
    }
}

/// `AUX_FUNC::STANDBY` — HIGH on, any other position off.
#[must_use]
pub const fn do_aux_function_standby(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetStandby(matches!(pos, AuxSwitchPos::High))
}

/// `AUX_FUNC::SURFACE_TRACKING`.
///
/// LOW is ground, MIDDLE is off, HIGH is ceiling. Swapping LOW/HIGH would
/// track the ceiling when the switch is down.
#[must_use]
pub const fn do_aux_function_surface_tracking(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetSurfaceTracking(match pos {
        AuxSwitchPos::Low => SurfaceTracking::Ground,
        AuxSwitchPos::Middle => SurfaceTracking::None,
        AuxSwitchPos::High => SurfaceTracking::Ceiling,
    })
}

/// `AUX_FUNC::WINCH_ENABLE`.
#[must_use]
pub const fn do_aux_function_winch_enable(pos: AuxSwitchPos) -> AuxLeftover {
    AuxLeftover::SetWinchEnable(match pos {
        AuxSwitchPos::High => WinchEnableAction::Stop,
        AuxSwitchPos::Low | AuxSwitchPos::Middle => WinchEnableAction::Relax,
    })
}

/// `AUX_FUNC::PARACHUTE_RELEASE`. LOW/MIDDLE do nothing.
#[must_use]
pub const fn do_aux_function_parachute_release(pos: AuxSwitchPos) -> AuxLeftover {
    if do_aux_function_high_enables(pos) {
        AuxLeftover::ReleaseParachute
    } else {
        AuxLeftover::None
    }
}

/// `AUX_FUNC::SAVE_TRIM`.
///
/// Vehicle still requires `allows_save_trim` and zero throttle.
#[must_use]
pub const fn do_aux_function_save_trim(pos: AuxSwitchPos) -> AuxLeftover {
    if do_aux_function_high_enables(pos) {
        AuxLeftover::SaveTrim
    } else {
        AuxLeftover::None
    }
}

/// `AUX_FUNC::SAVE_WP`.
///
/// Vehicle still rejects Auto, disarmed, and a first waypoint at zero throttle.
#[must_use]
pub const fn do_aux_function_save_wp(pos: AuxSwitchPos) -> AuxLeftover {
    if do_aux_function_high_enables(pos) {
        AuxLeftover::SaveWp
    } else {
        AuxLeftover::None
    }
}

/// `AUX_FUNC::ZIGZAG_SaveWP`.
///
/// The switch only talks to ZigZag while that mode is already engaged.
/// Firing it from Stabilize must not save a destination or change mode.
#[must_use]
pub const fn do_aux_function_zigzag_save_wp(pos: AuxSwitchPos, current_mode: u8) -> AuxLeftover {
    if current_mode != MODE_ZIGZAG {
        return AuxLeftover::None;
    }
    AuxLeftover::ZigzagSaveWp(match pos {
        AuxSwitchPos::Low => ZigzagSaveWp::DestinationA,
        AuxSwitchPos::Middle => ZigzagSaveWp::ReturnToManual,
        AuxSwitchPos::High => ZigzagSaveWp::DestinationB,
    })
}

/// `AUX_FUNC::ZIGZAG_Auto`.
#[must_use]
pub const fn do_aux_function_zigzag_auto(pos: AuxSwitchPos, current_mode: u8) -> AuxLeftover {
    if current_mode != MODE_ZIGZAG {
        return AuxLeftover::None;
    }
    AuxLeftover::ZigzagAuto(match pos {
        AuxSwitchPos::High => ZigzagAutoAction::Run,
        AuxSwitchPos::Low | AuxSwitchPos::Middle => ZigzagAutoAction::Suspend,
    })
}

/// `AUX_FUNC::SIMPLE_HEADING_RESET`.
#[must_use]
pub const fn do_aux_function_simple_heading_reset(pos: AuxSwitchPos) -> AuxLeftover {
    if do_aux_function_high_enables(pos) {
        AuxLeftover::ResetSimpleHeading
    } else {
        AuxLeftover::None
    }
}

/// `AUX_FUNC::TURBINE_START` — HIGH start, LOW stop, MIDDLE hold.
#[must_use]
pub const fn do_aux_function_turbine_start(pos: AuxSwitchPos) -> AuxLeftover {
    match pos {
        AuxSwitchPos::High => AuxLeftover::SetTurbineStart(true),
        AuxSwitchPos::Low => AuxLeftover::SetTurbineStart(false),
        AuxSwitchPos::Middle => AuxLeftover::None,
    }
}

/// `AUX_FUNC::FLIGHTMODE_PAUSE`.
#[must_use]
pub const fn do_aux_function_flightmode_pause(pos: AuxSwitchPos) -> AuxLeftover {
    match pos {
        AuxSwitchPos::High => AuxLeftover::FlightmodePause(FlightmodePauseAction::Pause),
        AuxSwitchPos::Low => AuxLeftover::FlightmodePause(FlightmodePauseAction::Resume),
        AuxSwitchPos::Middle => AuxLeftover::None,
    }
}

/// `AUX_FUNC::AHRS_AUTO_TRIM`.
///
/// HIGH starts (vehicle still requires `allows_auto_trim`). LOW saves
/// only if the trim is already running. MIDDLE is a no-op.
#[must_use]
pub const fn do_aux_function_ahrs_auto_trim(pos: AuxSwitchPos) -> AuxLeftover {
    match pos {
        AuxSwitchPos::High => AuxLeftover::AhrsAutoTrim(AhrsAutoTrimAction::Start),
        AuxSwitchPos::Low => AuxLeftover::AhrsAutoTrim(AhrsAutoTrimAction::SaveIfRunning),
        AuxSwitchPos::Middle => AuxLeftover::None,
    }
}

/// `RC_Channel_Copter::do_aux_function` leftover.
///
/// Mode-change, Flip, Simple / SuperSimple, AirMode and ForceFlying stay
/// the first dispatch. RunNow bodies (AcroTrainer, interlock, rangefinder,
/// parachute-enable / 3pos, attcon, …) stay next. This slice fills the
/// remaining Copter-owned arms: parachute release, winch control, save-trim
/// / save-wp, zigzag, pause, auto-trim, user hooks, and the other still-stub
/// cases.
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
        CopterAuxFunc::AcroTrainer => do_aux_function_acro_trainer(dispatch.pos),
        CopterAuxFunc::AttconFeedfwd => {
            AuxLeftover::SetAttconFeedfwd(do_aux_function_high_enables(dispatch.pos))
        }
        CopterAuxFunc::AttconAccelLim => {
            AuxLeftover::SetAttconAccelLim(do_aux_function_high_enables(dispatch.pos))
        }
        CopterAuxFunc::MotorInterlock => do_aux_function_motor_interlock(dispatch.pos),
        CopterAuxFunc::Rangefinder => {
            AuxLeftover::SetRangefinderHigh(do_aux_function_high_enables(dispatch.pos))
        }
        CopterAuxFunc::ParachuteEnable => {
            AuxLeftover::SetParachuteEnabled(do_aux_function_high_enables(dispatch.pos))
        }
        CopterAuxFunc::Parachute3pos => do_aux_function_parachute_3pos(dispatch.pos),
        CopterAuxFunc::PrecisionLoiter => match do_aux_function_high_low_hold(dispatch.pos) {
            Some(on) => AuxLeftover::SetPrecisionLoiter(on),
            None => AuxLeftover::None,
        },
        CopterAuxFunc::Standby => do_aux_function_standby(dispatch.pos),
        CopterAuxFunc::SurfaceTracking => do_aux_function_surface_tracking(dispatch.pos),
        CopterAuxFunc::WinchEnable => do_aux_function_winch_enable(dispatch.pos),
        CopterAuxFunc::CustomController => {
            AuxLeftover::SetCustomController(do_aux_function_high_enables(dispatch.pos))
        }
        CopterAuxFunc::WeatherVaneEnable => match do_aux_function_high_low_hold(dispatch.pos) {
            Some(on) => AuxLeftover::SetWeatherVane(on),
            None => AuxLeftover::None,
        },
        CopterAuxFunc::Inverted => match do_aux_function_high_low_hold(dispatch.pos) {
            Some(on) => AuxLeftover::SetInverted(on),
            None => AuxLeftover::None,
        },
        CopterAuxFunc::ParachuteRelease => do_aux_function_parachute_release(dispatch.pos),
        CopterAuxFunc::WinchControl => AuxLeftover::None,
        CopterAuxFunc::SaveTrim => do_aux_function_save_trim(dispatch.pos),
        CopterAuxFunc::SaveWp => do_aux_function_save_wp(dispatch.pos),
        CopterAuxFunc::UserFunc1 => AuxLeftover::CallUserFunc {
            which: 1,
            pos: dispatch.pos,
        },
        CopterAuxFunc::UserFunc2 => AuxLeftover::CallUserFunc {
            which: 2,
            pos: dispatch.pos,
        },
        CopterAuxFunc::UserFunc3 => AuxLeftover::CallUserFunc {
            which: 3,
            pos: dispatch.pos,
        },
        CopterAuxFunc::ZigzagSaveWp => {
            do_aux_function_zigzag_save_wp(dispatch.pos, dispatch.current_mode)
        }
        CopterAuxFunc::ZigzagAuto => {
            do_aux_function_zigzag_auto(dispatch.pos, dispatch.current_mode)
        }
        CopterAuxFunc::SimpleHeadingReset => do_aux_function_simple_heading_reset(dispatch.pos),
        CopterAuxFunc::ArmDisarmAirMode => AuxLeftover::ArmDisarmAirMode,
        CopterAuxFunc::TurbineStart => do_aux_function_turbine_start(dispatch.pos),
        CopterAuxFunc::FlightmodePause => do_aux_function_flightmode_pause(dispatch.pos),
        CopterAuxFunc::AutotuneTestGains => AuxLeftover::AutotuneTestGains(dispatch.pos),
        CopterAuxFunc::AhrsAutoTrim => do_aux_function_ahrs_auto_trim(dispatch.pos),
        CopterAuxFunc::ResetToArmedYaw => AuxLeftover::DelegateToBase,
        CopterAuxFunc::TransmitterTuning | CopterAuxFunc::TransmitterTuning2 => AuxLeftover::None,
        _ => AuxLeftover::Pending,
    }
}

/// `RC_Channel_Copter::init_aux_function`.
///
/// [`init_aux_kind`] is the classifier; this is the leftover that *uses* it.
/// NoInit options (`Flip`, RTL, Land, …) must not fire from a boot-time
/// HIGH switch. RunNow options *do* run so trainer / interlock / airmode
/// match the switch before the first `read_aux`. When transmitter-tuning
/// is compiled out, the RunNow group falls through to the base leftover.
#[must_use]
pub const fn init_aux_function(
    dispatch: AuxDispatch,
    transmitter_tuning_enabled: bool,
) -> AuxLeftover {
    match init_aux_kind(dispatch.func, transmitter_tuning_enabled) {
        InitAuxKind::NoInit => AuxLeftover::None,
        InitAuxKind::RunNow => do_aux_function(dispatch),
        InitAuxKind::DelegateToBase => AuxLeftover::DelegateToBase,
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

/// What `RC_Channel_Copter::mode_switch_changed` asked the vehicle to do.
///
/// `set_mode` can still fail; the vehicle applies [`ModeSwitchLeftover::Engage`]
/// simple only after the mode change succeeds. Applying EEPROM Simple bits
/// first would arm SuperSimple in a mode that never entered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSwitchLeftover {
    /// `new_pos < 0` or `new_pos > num_flight_modes`.
    Invalid,
    /// `set_mode(flight_modes[new_pos], ModeReason::RC_COMMAND)`.
    Engage {
        /// `Mode::Number` from `flight_modes[new_pos]`.
        mode: u8,
        /// EEPROM Simple / SuperSimple, or `None` when an aux channel owns it.
        simple: Option<SimpleMode>,
    },
}

/// EEPROM Simple bits for one flight-mode-switch position.
///
/// SuperSimple wins when both `g.super_simple` and `g.simple_modes` have
/// the bit: that is the `BIT_IS_SET` order in `mode_switch_changed`.
#[must_use]
pub const fn eeprom_simple_mode(pos: u8, super_simple: u8, simple_modes: u8) -> SimpleMode {
    if (super_simple & (1 << pos)) != 0 {
        SimpleMode::SuperSimple
    } else if (simple_modes & (1 << pos)) != 0 {
        SimpleMode::Simple
    } else {
        SimpleMode::None
    }
}

/// `RC_Channel_Copter::mode_switch_changed`.
///
/// The six-position switch is `ModeReason::RC_COMMAND`, not AUX_FUNCTION.
/// When neither SIMPLE nor SUPERSIMPLE is assigned to an aux channel,
/// Simple comes from the EEPROM bitmasks for *this* switch position.
/// An aux assignment owns Simple and the EEPROM bits must not run.
#[must_use]
pub const fn mode_switch_changed(
    new_pos: i8,
    num_flight_modes: u8,
    flight_mode: u8,
    simple_aux: bool,
    supersimple_aux: bool,
    super_simple_bits: u8,
    simple_modes_bits: u8,
) -> ModeSwitchLeftover {
    if new_pos < 0 || (new_pos as u8) > num_flight_modes {
        return ModeSwitchLeftover::Invalid;
    }
    let simple = if !simple_aux && !supersimple_aux {
        Some(eeprom_simple_mode(
            new_pos as u8,
            super_simple_bits,
            simple_modes_bits,
        ))
    } else {
        None
    };
    ModeSwitchLeftover::Engage {
        mode: flight_mode,
        simple,
    }
}

/// Copter `CH_MODE_DEFAULT` / `FLTMODE_CH` default.
///
/// Channel 5, not Plane's 8. Reusing `ap_rc::FLTMODE_CH_DEFAULT` would
/// read the wrong receiver channel for the six-position switch.
pub const CH_MODE_DEFAULT: i8 = 5;

/// `Copter::num_flight_modes` — slots `FLTMODE1` through `FLTMODE6`.
pub const NUM_FLIGHT_MODES: u8 = 6;

/// `ROLL_PITCH_YAW_INPUT_MAX` — ANGLE `high_in` for roll / pitch / yaw.
pub const ROLL_PITCH_YAW_INPUT_MAX: u16 = 4500;

/// `channel_throttle->set_range(1000)` — RANGE `high_in` for throttle.
pub const THROTTLE_CONTROL_RANGE: u16 = 1000;

/// Multicopter / heli `default_dead_zones` for roll and pitch.
pub const DEADZONE_ROLL_PITCH: u16 = 20;

/// Multicopter `channel_throttle` deadzone.
pub const DEADZONE_THROTTLE_MULTICOPTER: u16 = 30;

/// Multicopter `channel_yaw` deadzone.
pub const DEADZONE_YAW_MULTICOPTER: u16 = 20;

/// Heli `channel_throttle` deadzone.
pub const DEADZONE_THROTTLE_HELI: u16 = 10;

/// Heli `channel_yaw` deadzone.
pub const DEADZONE_YAW_HELI: u16 = 15;

/// `auto_trim_run` divisor — att-target radians / 20.
pub const AUTO_TRIM_DIVISOR: f32 = 20.0;

/// `RC_Channels_Copter::flight_mode_channel_number` — `g.flight_mode_chan`.
#[must_use]
pub const fn flight_mode_channel_number(flight_mode_chan: i8) -> i8 {
    flight_mode_chan
}

/// 0-based receiver index for Copter's `FLTMODE_CH`.
///
/// `RC_Channels::flight_mode_channel` rejects `<= 0` and
/// `>= NUM_RC_CHANNELS`, so channel 16 is never the mode switch.
#[must_use]
pub const fn flight_mode_channel(flight_mode_chan: i8) -> Option<usize> {
    // Same exclusive max as `RC_Channels::flight_mode_channel`.
    const _: () = assert!(NUM_RC_CHANNELS == 16);
    flight_mode_channel_index(flight_mode_channel_number(flight_mode_chan))
}

/// `RC_Channel::ControlType` — ANGLE (roll/pitch/yaw) vs RANGE (throttle).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlType {
    /// `set_angle` — `control_in` is centidegrees about trim.
    Angle,
    /// `set_range` — `control_in` is 0..`high_in` above min+deadzone.
    Range,
}

/// One Copter stick after `init_rc_in` mapping, plus the PWM calibration.
///
/// `RcChannel` is min/trim/max/deadzone/reverse. Copter then stamps
/// ANGLE 4500 on roll/pitch/yaw and RANGE 1000 on throttle so
/// [`get_control_in`] is not a signed stick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CopterRcChannel {
    /// Radio min/trim/max/deadzone/reverse.
    pub cal: RcChannel,
    /// `type_in` after `set_angle` / `set_range`.
    pub type_in: ControlType,
    /// `high_in` — 4500 for ANGLE, 1000 for throttle RANGE.
    pub high_in: u16,
}

impl CopterRcChannel {
    /// `set_angle(high)` on a default-calibrated channel with `deadzone`.
    #[must_use]
    pub const fn angle(high_in: u16, deadzone: u16) -> Self {
        let mut cal = RcChannel {
            radio_min: ap_rc::RC_CHAN_MIN_DEFAULT,
            radio_trim: ap_rc::RC_CHAN_TRIM_DEFAULT,
            radio_max: ap_rc::RC_CHAN_MAX_DEFAULT,
            deadzone: ap_rc::RC_CHAN_DEADZONE_DEFAULT,
            reversed: false,
        };
        cal.deadzone = deadzone;
        Self {
            cal,
            type_in: ControlType::Angle,
            high_in,
        }
    }

    /// `set_range(high)` on a default-calibrated channel with `deadzone`.
    #[must_use]
    pub const fn range(high_in: u16, deadzone: u16) -> Self {
        let mut cal = RcChannel {
            radio_min: ap_rc::RC_CHAN_MIN_DEFAULT,
            radio_trim: ap_rc::RC_CHAN_TRIM_DEFAULT,
            radio_max: ap_rc::RC_CHAN_MAX_DEFAULT,
            deadzone: ap_rc::RC_CHAN_DEADZONE_DEFAULT,
            reversed: false,
        };
        cal.deadzone = deadzone;
        Self {
            cal,
            type_in: ControlType::Range,
            high_in,
        }
    }
}

/// Roll / pitch / yaw / throttle after `Copter::init_rc_in` + `default_dead_zones`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CopterStickMap {
    /// `channel_roll` — ANGLE 4500.
    pub roll: CopterRcChannel,
    /// `channel_pitch` — ANGLE 4500.
    pub pitch: CopterRcChannel,
    /// `channel_yaw` — ANGLE 4500.
    pub yaw: CopterRcChannel,
    /// `channel_throttle` — RANGE 1000.
    pub throttle: CopterRcChannel,
}

/// `Copter::default_dead_zones` — `(roll, pitch, throttle, yaw)`.
///
/// Heli uses a tighter collective and yaw window. Applying the multicopter
/// 30 µs throttle deadzone to a heli would swallow the bottom of the
/// collective travel that `get_control_in` treats as zero.
#[must_use]
pub const fn default_dead_zones(heli: bool) -> (u16, u16, u16, u16) {
    if heli {
        (
            DEADZONE_ROLL_PITCH,
            DEADZONE_ROLL_PITCH,
            DEADZONE_THROTTLE_HELI,
            DEADZONE_YAW_HELI,
        )
    } else {
        (
            DEADZONE_ROLL_PITCH,
            DEADZONE_ROLL_PITCH,
            DEADZONE_THROTTLE_MULTICOPTER,
            DEADZONE_YAW_MULTICOPTER,
        )
    }
}

/// `Copter::init_rc_in` stick mapping leftover (type + high + deadzone).
///
/// Channel *identity* (which receiver index is roll) stays in `RCMAP_*`.
/// This is the Copter leftover that turns those four sticks into ANGLE
/// 4500 / RANGE 1000 so [`get_control_in`] matches `radio.cpp`.
#[must_use]
pub const fn init_rc_in_map(heli: bool) -> CopterStickMap {
    let (roll_dz, pitch_dz, thr_dz, yaw_dz) = default_dead_zones(heli);
    CopterStickMap {
        roll: CopterRcChannel::angle(ROLL_PITCH_YAW_INPUT_MAX, roll_dz),
        pitch: CopterRcChannel::angle(ROLL_PITCH_YAW_INPUT_MAX, pitch_dz),
        yaw: CopterRcChannel::angle(ROLL_PITCH_YAW_INPUT_MAX, yaw_dz),
        throttle: CopterRcChannel::range(THROTTLE_CONTROL_RANGE, thr_dz),
    }
}

fn constrain_pwm(pwm: u16, min: u16, max: u16) -> u16 {
    if pwm < min {
        min
    } else if pwm > max {
        max
    } else {
        pwm
    }
}

/// `RC_Channel::pwm_to_angle_dz_trim`.
///
/// Deadzone is a window around `_trim`, not around min. A RANGE-style
/// floor at min+dz would push roll/pitch off-centre at trim.
#[must_use]
pub fn pwm_to_angle_dz_trim(ch: &CopterRcChannel, pwm: u16, dead_zone: u16, trim: u16) -> f32 {
    let radio_trim_high = trim.saturating_add(dead_zone);
    let radio_trim_low = trim.saturating_sub(dead_zone);
    let reverse_mul = if ch.cal.reversed { -1.0 } else { 1.0 };
    let r_in = constrain_pwm(pwm, ch.cal.radio_min, ch.cal.radio_max);
    let high = f32::from(ch.high_in);
    if r_in > radio_trim_high && ch.cal.radio_max != radio_trim_high {
        reverse_mul * (high * f32::from(r_in - radio_trim_high))
            / f32::from(ch.cal.radio_max - radio_trim_high)
    } else if r_in < radio_trim_low && radio_trim_low != ch.cal.radio_min {
        reverse_mul * (high * (f32::from(r_in) - f32::from(radio_trim_low)))
            / f32::from(radio_trim_low - ch.cal.radio_min)
    } else {
        0.0
    }
}

/// `RC_Channel::pwm_to_range_dz`.
#[must_use]
pub fn pwm_to_range_dz(ch: &CopterRcChannel, pwm: u16, dead_zone: u16) -> f32 {
    let r_in = reverse_range_pwm(pwm, &ch.cal);
    let radio_trim_low = ch.cal.radio_min.saturating_add(dead_zone);
    if r_in > radio_trim_low && ch.cal.radio_max != radio_trim_low {
        (f32::from(ch.high_in) * f32::from(r_in - radio_trim_low))
            / f32::from(ch.cal.radio_max - radio_trim_low)
    } else {
        0.0
    }
}

/// `RC_Channel::get_control_in` after `update()` — ANGLE or RANGE.
///
/// The library stores a truncated `int16_t`, not the float. Modes that
/// compare throttle `control_in == 0` (save-trim, throttle-zero) need
/// that truncation, not a rounded mid-stick.
#[must_use]
pub fn get_control_in(ch: &CopterRcChannel, pwm: u16) -> i16 {
    let value = match ch.type_in {
        ControlType::Range => pwm_to_range_dz(ch, pwm, ch.cal.deadzone),
        ControlType::Angle => pwm_to_angle_dz_trim(ch, pwm, ch.cal.deadzone, ch.cal.radio_trim),
    };
    if value > f32::from(i16::MAX) {
        i16::MAX
    } else if value < f32::from(i16::MIN) {
        i16::MIN
    } else {
        value as i16
    }
}

/// `RC_Channel::get_control_in_zero_dz`.
#[must_use]
pub fn get_control_in_zero_dz(ch: &CopterRcChannel, pwm: u16) -> f32 {
    match ch.type_in {
        ControlType::Range => pwm_to_range_dz(ch, pwm, 0),
        ControlType::Angle => pwm_to_angle_dz_trim(ch, pwm, 0, ch.cal.radio_trim),
    }
}

/// `RC_Channel::get_control_mid` — integer RANGE mid-stick as `control_in`.
///
/// ANGLE channels return 0. The mid PWM is `(min+max)/2`, then the same
/// RANGE map as `pwm_to_range_dz` but in `int32` so a 370/770 mid-stick
/// is 480, not a rounded 481. Copter's `thr_mid` is this value.
#[must_use]
pub fn get_control_mid(ch: &CopterRcChannel) -> i16 {
    match ch.type_in {
        ControlType::Angle => 0,
        ControlType::Range => {
            let r_in = (i32::from(ch.cal.radio_min) + i32::from(ch.cal.radio_max)) / 2;
            let radio_trim_low = i32::from(ch.cal.radio_min) + i32::from(ch.cal.deadzone);
            let denom = i32::from(ch.cal.radio_max) - radio_trim_low;
            if denom == 0 {
                return 0;
            }
            let value = (i32::from(ch.high_in) * (r_in - radio_trim_low)) / denom;
            if value > i32::from(i16::MAX) {
                i16::MAX
            } else if value < i32::from(i16::MIN) {
                i16::MIN
            } else {
                value as i16
            }
        }
    }
}

/// `Copter::get_throttle_mid`.
///
/// Toy mode can replace the stick mid. Passing `None` is the normal
/// `channel_throttle->get_control_mid()` path.
#[must_use]
pub fn get_throttle_mid(throttle: &CopterRcChannel, toy_mode_mid: Option<i16>) -> i16 {
    match toy_mode_mid {
        Some(mid) => mid,
        None => get_control_mid(throttle),
    }
}

/// What `RC_Channels_Copter::save_trim` asked AHRS to store.
///
/// When auto-trim is already running the stick lean is **not** sampled —
/// those increments were applied live by [`auto_trim_run`]. Sampling
/// again would double the trim. The leftover only clears `running` and
/// persists the already-applied values (`add_trim(0, 0)` with persist).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SaveTrimLeftover {
    /// `auto_trim.running` after the call (always cleared).
    pub auto_trim_running: bool,
    /// Call `get_pilot_desired_lean_angles_rad` for the add_trim deltas.
    pub need_pilot_lean: bool,
    /// `AP::ahrs().add_trim` + `LogEvent::SAVE_TRIM`.
    pub persist: bool,
}

/// `RC_Channels_Copter::save_trim`.
#[must_use]
pub const fn save_trim(auto_trim_running: bool) -> SaveTrimLeftover {
    SaveTrimLeftover {
        auto_trim_running: false,
        need_pilot_lean: !auto_trim_running,
        persist: true,
    }
}

/// Why [`auto_trim_run`] cancelled instead of applying a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoTrimCancelReason {
    /// `!flightmode->allows_auto_trim()`.
    ModeDisallows,
    /// `ap.land_complete_maybe` — must be started and stopped mid-air.
    LandCompleteMaybe,
}

/// What `RC_Channels_Copter::auto_trim_run` asked the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AutoTrimRunLeftover {
    /// `auto_trim.running` is false — return without touching AHRS.
    Idle,
    /// `auto_trim_cancel` — running and notify `save_trim` both go false.
    Cancel {
        /// Which gate fired.
        reason: AutoTrimCancelReason,
    },
    /// `AP::ahrs().add_trim(roll/20, pitch/20, false)` — do not persist.
    Apply {
        /// `att_target.x / 20`.
        roll_trim_adj_rad: f32,
        /// `att_target.y / 20`.
        pitch_trim_adj_rad: f32,
    },
}

/// `RC_Channels_Copter::auto_trim_cancel`.
#[must_use]
pub const fn auto_trim_cancel() -> SaveTrimLeftover {
    SaveTrimLeftover {
        auto_trim_running: false,
        need_pilot_lean: false,
        persist: false,
    }
}

/// `RC_Channels_Copter::auto_trim_run`.
///
/// The att-target divisor is subjective (`/ 20`) so the feel matches the
/// old stick-trim method. Persisting each step would write EEPROM every
/// loop; persist happens only on `save_trim`.
#[must_use]
pub fn auto_trim_run(
    running: bool,
    allows_auto_trim: bool,
    land_complete_maybe: bool,
    att_target_roll_rad: f32,
    att_target_pitch_rad: f32,
) -> AutoTrimRunLeftover {
    if !running {
        return AutoTrimRunLeftover::Idle;
    }
    if !allows_auto_trim {
        return AutoTrimRunLeftover::Cancel {
            reason: AutoTrimCancelReason::ModeDisallows,
        };
    }
    if land_complete_maybe {
        return AutoTrimRunLeftover::Cancel {
            reason: AutoTrimCancelReason::LandCompleteMaybe,
        };
    }
    AutoTrimRunLeftover::Apply {
        roll_trim_adj_rad: att_target_roll_rad / AUTO_TRIM_DIVISOR,
        pitch_trim_adj_rad: att_target_pitch_rad / AUTO_TRIM_DIVISOR,
    }
}
