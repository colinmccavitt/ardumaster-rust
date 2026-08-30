//! Radio-init and throttle-failsafe leftover, upstream `ArduCopter/radio.cpp`.
//!
//! Tracked as **COP-022**. Stick ANGLE/RANGE mapping, `get_control_in`, and
//! `FLTMODE_CH` already live in [`crate::aux_fn`]. What lives here is the rest
//! of that file: which receiver channels `init_rc_in` binds, the three-count
//! throttle PWM latch, the lost-frame timeout, and the throttle-zero
//! debounce that tells the vehicle it is not flying.
//!
//! # The PWM floor is exclusive, and it is counted
//!
//! Copter trips on `throttle_pwm < FS_THR_VALUE`, so a pulse *at* the
//! threshold is healthy. Reusing Plane's `radio_in <= THR_FS_VALUE` would
//! failsafe a resting 975 us stick. The latch also waits
//! [`FS_COUNTER`] consecutive low pulses — a single glitch must not
//! `set_failsafe_radio`. A pending `radio_counter` is already "invalid
//! input" (`has_valid_input` in `aux_fn.rs`) even before the latch.
//!
//! # Lost frames are a second door
//!
//! `read_radio` only counts throttle PWM when `read_input` succeeded.
//! Silence uses `RC_FS_TIMEOUT` (floored at 100 ms). Already-latched
//! failsafe, a disabled `FS_THR_ENABLE`, and "never seen a receiver and
//! not armed" all refuse to trip. Folding those into the PWM counter
//! would log `RADIO_LATE_FRAME` on a bench with no radio.

use crate::aux_fn::{
    get_control_in_zero_dz, init_rc_in_map, AirMode, CopterAuxFunc, CopterStickMap,
};
use ap_rc::{norm_input, rcmap_index, RcMap, FS_THR_VALUE_DEFAULT, NUM_RC_CHANNELS};

/// Consecutive low-throttle pulses before `failsafe.radio` latches.
///
/// Upstream `FS_COUNTER` in `radio.cpp`.
pub const FS_COUNTER: i8 = 3;

/// Debounce before `ap.throttle_zero` becomes true, milliseconds.
///
/// Upstream `THROTTLE_ZERO_DEBOUNCE_TIME_MS`.
pub const THROTTLE_ZERO_DEBOUNCE_TIME_MS: u32 = 400;

/// `RC_FS_TIMEOUT` default, seconds.
pub const RC_FS_TIMEOUT_DEFAULT_S: f32 = 1.0;

/// Floor of `RC_Channels::get_fs_timeout_ms`, milliseconds.
pub const RC_FS_TIMEOUT_MIN_MS: u32 = 100;

/// Copter `FS_THR_VALUE` default — same PWM as [`FS_THR_VALUE_DEFAULT`].
pub const FS_THR_VALUE_COPTER_DEFAULT: u16 = FS_THR_VALUE_DEFAULT;

/// `FS_THR_DISABLED`.
pub const FS_THR_DISABLED: u8 = 0;

/// Heli `RC8_OPTION` default from `init_rc_in` — `AUX_FUNC::MOTOR_INTERLOCK`.
pub const HELI_RC8_OPTION_DEFAULT: u16 = CopterAuxFunc::MotorInterlock as u16;

/// 1-based channel that heli default-assigns to motor interlock.
pub const HELI_RC8_CHANNEL: u8 = 8;

/// Motor PWM min forced when the throttle channel is unconfigured.
pub const MOTOR_PWM_MIN_DEFAULT: u16 = 1000;

/// Motor PWM max forced when the throttle channel is unconfigured.
pub const MOTOR_PWM_MAX_DEFAULT: u16 = 2000;

/// `BoardConfig.set_default_safety_ignore_mask` keeps the low 14 bits.
pub const SAFETY_IGNORE_MASK_BITS: u16 = 0x3FFF;

/// Scale from RANGE `control_in_zero_dz` (0..1000) to motors passthrough.
pub const PASSTHROUGH_THROTTLE_SCALE: f32 = 0.001;

/// 0-based receiver indices after `channel_roll = &rc().get_roll_channel()`.
///
/// `RCMAP_*` identity. Invalid map numbers become [`None`] — the same
/// dummy-channel fallback as `get_rcmap_channel_nonnull`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StickAssignment {
    /// `channel_roll` receiver index.
    pub roll: Option<usize>,
    /// `channel_pitch` receiver index.
    pub pitch: Option<usize>,
    /// `channel_throttle` receiver index.
    pub throttle: Option<usize>,
    /// `channel_yaw` receiver index.
    pub yaw: Option<usize>,
}

/// Bind the four Copter sticks from `RCMAP_*`.
#[must_use]
pub const fn assign_stick_channels(map: RcMap) -> StickAssignment {
    StickAssignment {
        roll: rcmap_index(map.roll),
        pitch: rcmap_index(map.pitch),
        throttle: rcmap_index(map.throttle),
        yaw: rcmap_index(map.yaw),
    }
}

/// First 0-based channel whose `RCn_OPTION` equals `option`.
///
/// Upstream `RC_Channels::find_channel_for_option`. Scan order is
/// channel 0..[`NUM_RC_CHANNELS`]; a second copy of the same option is
/// ignored. `None` is the nullptr return.
#[must_use]
pub fn find_channel_for_option(options: &[u16], option: u16) -> Option<usize> {
    for (i, &stored) in options.iter().enumerate() {
        if i >= NUM_RC_CHANNELS as usize {
            break;
        }
        if stored == option {
            return Some(i);
        }
    }
    None
}

/// What `Copter::init_rc_in` asked the vehicle to remember.
///
/// ANGLE 4500 / RANGE 1000 plus deadzones are [`init_rc_in_map`]. This
/// leftover adds the pointer bind, the heli `RC8_OPTION` default, the
/// transmitter-tuning finds, and `ap.throttle_zero = true`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitRcInLeftover {
    /// `channel_roll` / pitch / throttle / yaw after the RCMAP bind.
    pub assignment: StickAssignment,
    /// Type + high + deadzone after `default_dead_zones`.
    pub map: CopterStickMap,
    /// `ap.throttle_zero` after init — always true.
    pub throttle_zero: bool,
    /// Heli-only `{"RC8_OPTION", 32}` default (set if unconfigured).
    pub heli_rc8_option_default: Option<u16>,
    /// `rc().find_channel_for_option(TRANSMITTER_TUNING)`.
    pub rc_tuning: Option<usize>,
    /// `rc().find_channel_for_option(TRANSMITTER_TUNING2)`.
    pub rc_tuning2: Option<usize>,
}

/// `Copter::init_rc_in`.
///
/// `transmitter_tuning_enabled` is `AP_RC_TRANSMITTER_TUNING_ENABLED`.
/// When that compile switch is off the finds do not run.
#[must_use]
pub fn init_rc_in(
    rcmap: RcMap,
    heli: bool,
    transmitter_tuning_enabled: bool,
    options: &[u16],
) -> InitRcInLeftover {
    let (rc_tuning, rc_tuning2) = if transmitter_tuning_enabled {
        (
            find_channel_for_option(options, CopterAuxFunc::TransmitterTuning as u16),
            find_channel_for_option(options, CopterAuxFunc::TransmitterTuning2 as u16),
        )
    } else {
        (None, None)
    };
    InitRcInLeftover {
        assignment: assign_stick_channels(rcmap),
        map: init_rc_in_map(heli),
        throttle_zero: true,
        heli_rc8_option_default: if heli {
            Some(HELI_RC8_OPTION_DEFAULT)
        } else {
            None
        },
        rc_tuning,
        rc_tuning2,
    }
}

/// Inputs to `Copter::set_throttle_and_failsafe`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleFailsafeInputs {
    /// `g.failsafe_throttle` / `FS_THR_ENABLE`.
    pub failsafe_throttle: u8,
    /// `g.failsafe_throttle_value` / `FS_THR_VALUE`.
    pub failsafe_throttle_value: u16,
    /// `channel_throttle->get_radio_in()`.
    pub throttle_pwm: u16,
    /// `failsafe.radio` before the call.
    pub radio: bool,
    /// `failsafe.radio_counter` before the call (`int8_t`).
    pub radio_counter: i8,
    /// `rc().has_ever_seen_rc_input()`.
    pub has_ever_seen_rc_input: bool,
    /// `motors->armed()`.
    pub armed: bool,
}

/// `failsafe.radio` + `radio_counter` after `set_throttle_and_failsafe`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleFailsafeLeftover {
    /// `failsafe.radio` after the call.
    pub radio: bool,
    /// `failsafe.radio_counter` after the call.
    pub radio_counter: i8,
}

/// `Copter::set_throttle_and_failsafe`.
///
/// Disabled clears the latch immediately and leaves the counter alone.
/// Low PWM while already failed, or while never-seen and disarmed, is a
/// pass-through — incrementing there would arm the counter on the bench.
#[must_use]
pub const fn set_throttle_and_failsafe(
    inputs: ThrottleFailsafeInputs,
) -> ThrottleFailsafeLeftover {
    if inputs.failsafe_throttle == FS_THR_DISABLED {
        return ThrottleFailsafeLeftover {
            radio: false,
            radio_counter: inputs.radio_counter,
        };
    }

    if inputs.throttle_pwm < inputs.failsafe_throttle_value {
        if inputs.radio || !(inputs.has_ever_seen_rc_input || inputs.armed) {
            return ThrottleFailsafeLeftover {
                radio: inputs.radio,
                radio_counter: inputs.radio_counter,
            };
        }
        let mut radio_counter = inputs.radio_counter;
        if radio_counter < i8::MAX {
            radio_counter += 1;
        }
        if radio_counter >= FS_COUNTER {
            radio_counter = FS_COUNTER;
            return ThrottleFailsafeLeftover {
                radio: true,
                radio_counter,
            };
        }
        return ThrottleFailsafeLeftover {
            radio: inputs.radio,
            radio_counter,
        };
    }

    let mut radio_counter = inputs.radio_counter;
    if radio_counter > i8::MIN {
        radio_counter -= 1;
    }
    if radio_counter <= 0 {
        radio_counter = 0;
        return ThrottleFailsafeLeftover {
            radio: false,
            radio_counter,
        };
    }
    ThrottleFailsafeLeftover {
        radio: inputs.radio,
        radio_counter,
    }
}

/// Inputs to `Copter::set_throttle_zero_flag`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleZeroInputs {
    /// `channel_throttle->get_control_in()`.
    pub throttle_control: i16,
    /// `ap.using_interlock`.
    pub using_interlock: bool,
    /// `SRV_Channels::get_emergency_stop()`.
    pub emergency_stop: bool,
    /// `motors->get_interlock()`.
    pub motor_interlock: bool,
    /// `ap.armed_with_airmode_switch`.
    pub armed_with_airmode_switch: bool,
    /// `copter.air_mode`.
    pub air_mode: AirMode,
    /// Static `last_nonzero_throttle_ms`.
    pub last_nonzero_throttle_ms: u32,
    /// `millis()`.
    pub now_ms: u32,
    /// `ap.throttle_zero` before the call.
    pub throttle_zero: bool,
}

/// `ap.throttle_zero` + the debounce timestamp after the flag update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleZeroLeftover {
    /// `ap.throttle_zero` after the call.
    pub throttle_zero: bool,
    /// `last_nonzero_throttle_ms` after the call.
    pub last_nonzero_throttle_ms: u32,
}

/// True when the leftover treats the aircraft as flying.
///
/// Interlock *replaces* the throttle-control test — a high collective
/// with interlock compiled in does not clear `throttle_zero` unless the
/// motor interlock is actually on. Air-mode (switch or armed-with-airmode)
/// is an immediate "flying" regardless of stick.
#[must_use]
pub const fn throttle_is_flying(inputs: &ThrottleZeroInputs) -> bool {
    (!inputs.using_interlock && inputs.throttle_control > 0 && !inputs.emergency_stop)
        || (inputs.using_interlock && inputs.motor_interlock)
        || inputs.armed_with_airmode_switch
        || matches!(inputs.air_mode, AirMode::Enabled)
}

/// `Copter::set_throttle_zero_flag`.
///
/// The 400 ms compare is `>`, not `>=`. Equality leaves the flag where
/// it was so a 400 ms pulse-and-hold is still "flying".
#[must_use]
pub const fn set_throttle_zero_flag(inputs: ThrottleZeroInputs) -> ThrottleZeroLeftover {
    if throttle_is_flying(&inputs) {
        return ThrottleZeroLeftover {
            throttle_zero: false,
            last_nonzero_throttle_ms: inputs.now_ms,
        };
    }
    if inputs
        .now_ms
        .wrapping_sub(inputs.last_nonzero_throttle_ms)
        > THROTTLE_ZERO_DEBOUNCE_TIME_MS
    {
        return ThrottleZeroLeftover {
            throttle_zero: true,
            last_nonzero_throttle_ms: inputs.last_nonzero_throttle_ms,
        };
    }
    ThrottleZeroLeftover {
        throttle_zero: inputs.throttle_zero,
        last_nonzero_throttle_ms: inputs.last_nonzero_throttle_ms,
    }
}

/// `RC_Channels::get_fs_timeout_ms` — `MAX(_fs_timeout * 1000, 100)`.
///
/// The C++ `MAX` is a float compare, then the result is truncated toward
/// zero into `uint32_t`. A 0.05 s parameter becomes 100 ms, not 50.
#[must_use]
pub fn get_fs_timeout_ms(fs_timeout_s: f32) -> u32 {
    let ms = fs_timeout_s * 1000.0;
    let floored = if ms > RC_FS_TIMEOUT_MIN_MS as f32 {
        ms
    } else {
        RC_FS_TIMEOUT_MIN_MS as f32
    };
    if floored <= 0.0 {
        0
    } else if floored >= u32::MAX as f32 {
        u32::MAX
    } else {
        floored as u32
    }
}

/// Stick values `Copter::radio_passthrough_to_motors` hands the mixer.
///
/// Roll / pitch / yaw are `norm_input` (no deadzone). Throttle is
/// `get_control_in_zero_dz() * 0.001`, not a signed stick — a mid
/// collective must be ~0.5, not 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadioPassthrough {
    /// `channel_roll->norm_input()`.
    pub roll: f32,
    /// `channel_pitch->norm_input()`.
    pub pitch: f32,
    /// `channel_throttle->get_control_in_zero_dz() * 0.001`.
    pub throttle: f32,
    /// `channel_yaw->norm_input()`.
    pub yaw: f32,
}

/// `Copter::radio_passthrough_to_motors`.
#[must_use]
pub fn radio_passthrough_to_motors(
    map: &CopterStickMap,
    roll_pwm: u16,
    pitch_pwm: u16,
    throttle_pwm: u16,
    yaw_pwm: u16,
) -> RadioPassthrough {
    RadioPassthrough {
        roll: norm_input(roll_pwm, &map.roll.cal),
        pitch: norm_input(pitch_pwm, &map.pitch.cal),
        throttle: get_control_in_zero_dz(&map.throttle, throttle_pwm) * PASSTHROUGH_THROTTLE_SCALE,
        yaw: norm_input(yaw_pwm, &map.yaw.cal),
    }
}

/// Inputs to `Copter::read_radio`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReadRadioInputs {
    /// `rc().read_input()` returned true this tick.
    pub got_input: bool,
    /// `millis()`.
    pub now_ms: u32,
    /// `last_radio_update_ms` before the call.
    pub last_radio_update_ms: u32,
    /// `RC_FS_TIMEOUT` seconds.
    pub fs_timeout_s: f32,
    /// PWM / enable / latch for [`set_throttle_and_failsafe`].
    pub failsafe: ThrottleFailsafeInputs,
    /// Debounce inputs for [`set_throttle_zero_flag`].
    pub throttle_zero: ThrottleZeroInputs,
}

/// What `Copter::read_radio` asked the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadRadioLeftover {
    /// New frame — pass-through, filter, and throttle/failsafe updates.
    Frame {
        /// After `set_throttle_and_failsafe`.
        failsafe: ThrottleFailsafeLeftover,
        /// After `set_throttle_zero_flag`.
        throttle_zero: ThrottleZeroLeftover,
        /// `last_radio_update_ms` after the call.
        last_radio_update_ms: u32,
    },
    /// No frame and `failsafe.radio` is already set — do not re-trip.
    AlreadyFailed,
    /// No frame, elapsed still below `get_fs_timeout_ms()`.
    Waiting,
    /// No frame and `FS_THR_ENABLE == 0`.
    TimeoutDisabled,
    /// No frame, never seen a receiver, and disarmed.
    NeverSeenDisarmed,
    /// Timed out — `set_failsafe_radio(true)` and log `RADIO_LATE_FRAME`.
    LateFrame,
}

/// `Copter::read_radio`.
///
/// The timeout door does not run `set_throttle_and_failsafe`. A late
/// frame latches `failsafe.radio` without touching `radio_counter`,
/// matching the C++ fall-through that only calls `set_failsafe_radio`.
#[must_use]
pub fn read_radio(inputs: &ReadRadioInputs) -> ReadRadioLeftover {
    if inputs.got_input {
        return ReadRadioLeftover::Frame {
            failsafe: set_throttle_and_failsafe(inputs.failsafe),
            throttle_zero: set_throttle_zero_flag(inputs.throttle_zero),
            last_radio_update_ms: inputs.now_ms,
        };
    }
    if inputs.failsafe.radio {
        return ReadRadioLeftover::AlreadyFailed;
    }
    let elapsed_ms = inputs.now_ms.wrapping_sub(inputs.last_radio_update_ms);
    if elapsed_ms < get_fs_timeout_ms(inputs.fs_timeout_s) {
        return ReadRadioLeftover::Waiting;
    }
    if inputs.failsafe.failsafe_throttle == FS_THR_DISABLED {
        return ReadRadioLeftover::TimeoutDisabled;
    }
    if !inputs.failsafe.has_ever_seen_rc_input && !inputs.failsafe.armed {
        return ReadRadioLeftover::NeverSeenDisarmed;
    }
    ReadRadioLeftover::LateFrame
}

/// What `Copter::init_rc_out` asked motors / BoardConfig to do.
///
/// `motors->init` / `set_update_rate` stay with the motors crate. This
/// leftover is the PWM-range source and the safety-ignore mask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitRcOutLeftover {
    /// Multicopter `convert_pwm_min_max_param` arguments.
    pub motor_pwm: Option<(u16, u16)>,
    /// Heli `hal.rcout->set_esc_scaling(radio_min, radio_max)`.
    pub esc_scaling: Option<(u16, u16)>,
    /// Multicopter `(~motors->get_motor_mask()) & 0x3FFF`.
    pub safety_ignore_mask: Option<u16>,
}

/// `Copter::init_rc_out` PWM / safety leftover.
///
/// An unconfigured throttle channel forces 1000/2000 so a later RC
/// calibration cannot rewrite motor PWM. Heli skips that copy and the
/// safety mask; it only scales ESCs to the collective radio range.
#[must_use]
pub const fn init_rc_out(
    heli: bool,
    throttle_configured: bool,
    throttle_radio_min: u16,
    throttle_radio_max: u16,
    motor_mask: u16,
) -> InitRcOutLeftover {
    if heli {
        return InitRcOutLeftover {
            motor_pwm: None,
            esc_scaling: Some((throttle_radio_min, throttle_radio_max)),
            safety_ignore_mask: None,
        };
    }
    let motor_pwm = if throttle_configured {
        Some((throttle_radio_min, throttle_radio_max))
    } else {
        Some((MOTOR_PWM_MIN_DEFAULT, MOTOR_PWM_MAX_DEFAULT))
    };
    InitRcOutLeftover {
        motor_pwm,
        esc_scaling: None,
        safety_ignore_mask: Some((!motor_mask) & SAFETY_IGNORE_MASK_BITS),
    }
}
