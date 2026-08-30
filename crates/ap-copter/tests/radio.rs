//! Copter radio leftover, upstream `ArduCopter/radio.cpp`.

use ap_copter::aux_fn::{
    init_rc_in_map, AirMode, CopterAuxFunc, ControlType, DEADZONE_THROTTLE_HELI,
    DEADZONE_THROTTLE_MULTICOPTER, ROLL_PITCH_YAW_INPUT_MAX, THROTTLE_CONTROL_RANGE,
};
use ap_copter::radio::{
    assign_stick_channels, find_channel_for_option, get_fs_timeout_ms, init_rc_in, init_rc_out,
    radio_passthrough_to_motors, read_radio, set_throttle_and_failsafe, set_throttle_zero_flag,
    throttle_is_flying, InitRcOutLeftover, ReadRadioInputs, ReadRadioLeftover, StickAssignment,
    ThrottleFailsafeInputs, ThrottleFailsafeLeftover, ThrottleZeroInputs, ThrottleZeroLeftover,
    FS_COUNTER, FS_THR_DISABLED, FS_THR_VALUE_COPTER_DEFAULT, HELI_RC8_CHANNEL,
    HELI_RC8_OPTION_DEFAULT, MOTOR_PWM_MAX_DEFAULT, MOTOR_PWM_MIN_DEFAULT,
    PASSTHROUGH_THROTTLE_SCALE, RC_FS_TIMEOUT_DEFAULT_S, RC_FS_TIMEOUT_MIN_MS,
    SAFETY_IGNORE_MASK_BITS, THROTTLE_ZERO_DEBOUNCE_TIME_MS,
};
use ap_rc::{
    RcMap, FS_THR_VALUE_DEFAULT, RCMAP_PITCH_DEFAULT, RCMAP_ROLL_DEFAULT, RCMAP_THROTTLE_DEFAULT,
    RCMAP_YAW_DEFAULT,
};

#[test]
fn constants_match_upstream_radio_cpp() {
    assert_eq!(FS_COUNTER, 3);
    assert_eq!(THROTTLE_ZERO_DEBOUNCE_TIME_MS, 400);
    assert_eq!(RC_FS_TIMEOUT_DEFAULT_S, 1.0);
    assert_eq!(RC_FS_TIMEOUT_MIN_MS, 100);
    assert_eq!(FS_THR_DISABLED, 0);
    assert_eq!(FS_THR_VALUE_COPTER_DEFAULT, 975);
    assert_eq!(FS_THR_VALUE_COPTER_DEFAULT, FS_THR_VALUE_DEFAULT);
    assert_eq!(HELI_RC8_OPTION_DEFAULT, 32);
    assert_eq!(HELI_RC8_OPTION_DEFAULT, CopterAuxFunc::MotorInterlock as u16);
    assert_eq!(HELI_RC8_CHANNEL, 8);
    assert_eq!(MOTOR_PWM_MIN_DEFAULT, 1000);
    assert_eq!(MOTOR_PWM_MAX_DEFAULT, 2000);
    assert_eq!(SAFETY_IGNORE_MASK_BITS, 0x3FFF);
    assert!((PASSTHROUGH_THROTTLE_SCALE - 0.001).abs() < 1e-9);
    assert_eq!(get_fs_timeout_ms(RC_FS_TIMEOUT_DEFAULT_S), 1000);
    assert_eq!(get_fs_timeout_ms(0.05), 100, "MAX(50, 100) floors at 100 ms");
    assert_eq!(get_fs_timeout_ms(0.1), 100);
    assert_eq!(get_fs_timeout_ms(2.5), 2500);
}

#[test]
fn assign_stick_channels_follows_rcmap() {
    let default = assign_stick_channels(RcMap::default());
    assert_eq!(
        default,
        StickAssignment {
            roll: Some((RCMAP_ROLL_DEFAULT - 1) as usize),
            pitch: Some((RCMAP_PITCH_DEFAULT - 1) as usize),
            throttle: Some((RCMAP_THROTTLE_DEFAULT - 1) as usize),
            yaw: Some((RCMAP_YAW_DEFAULT - 1) as usize),
        }
    );
    let remapped = assign_stick_channels(RcMap::from_params(2, 3, 4, 1));
    assert_eq!(remapped.roll, Some(1));
    assert_eq!(remapped.pitch, Some(2));
    assert_eq!(remapped.throttle, Some(3));
    assert_eq!(remapped.yaw, Some(0));
    let dummy = assign_stick_channels(RcMap::from_params(0, 17, 3, 1));
    assert_eq!(dummy.roll, None, "RCMAP 0 is the dummy channel");
    assert_eq!(dummy.pitch, None, "RCMAP 17 is out of range");
    assert_eq!(dummy.throttle, Some(2));
}

#[test]
fn find_channel_for_option_is_first_match() {
    let mut options = [0_u16; 16];
    options[4] = CopterAuxFunc::TransmitterTuning as u16;
    options[9] = CopterAuxFunc::TransmitterTuning as u16;
    options[6] = CopterAuxFunc::TransmitterTuning2 as u16;
    assert_eq!(
        find_channel_for_option(&options, CopterAuxFunc::TransmitterTuning as u16),
        Some(4)
    );
    assert_eq!(
        find_channel_for_option(&options, CopterAuxFunc::TransmitterTuning2 as u16),
        Some(6)
    );
    assert_eq!(
        find_channel_for_option(&options, CopterAuxFunc::MotorInterlock as u16),
        None
    );
}

#[test]
fn init_rc_in_binds_map_and_starts_throttle_zero() {
    let mut options = [0_u16; 8];
    options[2] = CopterAuxFunc::TransmitterTuning as u16;
    let multi = init_rc_in(RcMap::default(), false, true, &options);
    assert_eq!(multi.assignment.roll, Some(0));
    assert_eq!(multi.assignment.throttle, Some(2));
    assert!(multi.throttle_zero, "init_rc_in sets ap.throttle_zero = true");
    assert_eq!(multi.heli_rc8_option_default, None);
    assert_eq!(multi.rc_tuning, Some(2));
    assert_eq!(multi.rc_tuning2, None);
    assert_eq!(multi.map.roll.type_in, ControlType::Angle);
    assert_eq!(multi.map.roll.high_in, ROLL_PITCH_YAW_INPUT_MAX);
    assert_eq!(multi.map.throttle.high_in, THROTTLE_CONTROL_RANGE);
    assert_eq!(
        multi.map.throttle.cal.deadzone,
        DEADZONE_THROTTLE_MULTICOPTER
    );

    let heli = init_rc_in(RcMap::default(), true, false, &options);
    assert_eq!(heli.heli_rc8_option_default, Some(32));
    assert_eq!(heli.rc_tuning, None, "compile switch off skips the finds");
    assert_eq!(heli.rc_tuning2, None);
    assert_eq!(heli.map.throttle.cal.deadzone, DEADZONE_THROTTLE_HELI);
    assert_eq!(heli.map, init_rc_in_map(true));
}

fn fs(pwm: u16) -> ThrottleFailsafeInputs {
    ThrottleFailsafeInputs {
        failsafe_throttle: 1,
        failsafe_throttle_value: FS_THR_VALUE_COPTER_DEFAULT,
        throttle_pwm: pwm,
        radio: false,
        radio_counter: 0,
        has_ever_seen_rc_input: true,
        armed: true,
    }
}

#[test]
fn disabled_clears_radio_and_leaves_the_counter() {
    let leftover = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        failsafe_throttle: FS_THR_DISABLED,
        radio: true,
        radio_counter: 2,
        throttle_pwm: 0,
        ..fs(0)
    });
    assert_eq!(
        leftover,
        ThrottleFailsafeLeftover {
            radio: false,
            radio_counter: 2,
        }
    );
}

#[test]
fn pwm_at_fs_thr_value_is_healthy_not_plane_inclusive() {
    let at = set_throttle_and_failsafe(fs(FS_THR_VALUE_COPTER_DEFAULT));
    assert_eq!(
        at,
        ThrottleFailsafeLeftover {
            radio: false,
            radio_counter: 0,
        },
        "Copter uses exclusive less-than, so 975 us must not count as low"
    );
    let low = set_throttle_and_failsafe(fs(FS_THR_VALUE_COPTER_DEFAULT - 1));
    assert_eq!(
        low,
        ThrottleFailsafeLeftover {
            radio: false,
            radio_counter: 1,
        }
    );
}

#[test]
fn three_low_pulses_latch_two_do_not() {
    let first = set_throttle_and_failsafe(fs(900));
    assert_eq!(first.radio_counter, 1);
    assert!(!first.radio);
    let second = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        radio_counter: first.radio_counter,
        ..fs(900)
    });
    assert_eq!(second.radio_counter, 2);
    assert!(!second.radio);
    let third = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        radio_counter: second.radio_counter,
        ..fs(900)
    });
    assert_eq!(
        third,
        ThrottleFailsafeLeftover {
            radio: true,
            radio_counter: FS_COUNTER,
        }
    );
    let fourth = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        radio: true,
        radio_counter: FS_COUNTER,
        ..fs(900)
    });
    assert_eq!(
        fourth.radio_counter, FS_COUNTER,
        "already-failed low PWM is a pass-through, not increment"
    );
}

#[test]
fn never_seen_and_disarmed_ignores_low_pwm() {
    let leftover = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        has_ever_seen_rc_input: false,
        armed: false,
        ..fs(0)
    });
    assert_eq!(
        leftover,
        ThrottleFailsafeLeftover {
            radio: false,
            radio_counter: 0,
        }
    );
    let armed_unseen = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        has_ever_seen_rc_input: false,
        armed: true,
        ..fs(0)
    });
    assert_eq!(armed_unseen.radio_counter, 1);
    let seen_disarmed = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        has_ever_seen_rc_input: true,
        armed: false,
        ..fs(0)
    });
    assert_eq!(seen_disarmed.radio_counter, 1);
}

#[test]
fn three_good_pulses_clear_a_latched_failsafe() {
    let mut state = ThrottleFailsafeLeftover {
        radio: true,
        radio_counter: FS_COUNTER,
    };
    for expect in [2, 1, 0] {
        state = set_throttle_and_failsafe(ThrottleFailsafeInputs {
            radio: state.radio,
            radio_counter: state.radio_counter,
            ..fs(1500)
        });
        assert_eq!(state.radio_counter, expect);
        if expect == 0 {
            assert!(!state.radio);
        } else {
            assert!(state.radio, "counter {expect} still latched");
        }
    }
    let extra = set_throttle_and_failsafe(ThrottleFailsafeInputs {
        radio: false,
        radio_counter: 0,
        ..fs(1500)
    });
    assert_eq!(extra.radio_counter, 0);
}

fn tz(control: i16, last_ms: u32, now_ms: u32, zero: bool) -> ThrottleZeroInputs {
    ThrottleZeroInputs {
        throttle_control: control,
        using_interlock: false,
        emergency_stop: false,
        motor_interlock: false,
        armed_with_airmode_switch: false,
        air_mode: AirMode::None,
        last_nonzero_throttle_ms: last_ms,
        now_ms,
        throttle_zero: zero,
    }
}

#[test]
fn throttle_zero_clears_immediately_on_nonzero_stick() {
    let leftover = set_throttle_zero_flag(tz(1, 0, 10, true));
    assert_eq!(
        leftover,
        ThrottleZeroLeftover {
            throttle_zero: false,
            last_nonzero_throttle_ms: 10,
        }
    );
    assert!(throttle_is_flying(&tz(1, 0, 10, true)));
    assert!(!throttle_is_flying(&tz(0, 0, 10, true)));
}

#[test]
fn throttle_zero_debounce_is_exclusive_at_400ms() {
    let at = set_throttle_zero_flag(tz(0, 1000, 1400, false));
    assert_eq!(
        at,
        ThrottleZeroLeftover {
            throttle_zero: false,
            last_nonzero_throttle_ms: 1000,
        },
        "exclusive compare: 400 ms exactly is still flying"
    );
    let after = set_throttle_zero_flag(tz(0, 1000, 1401, false));
    assert_eq!(
        after,
        ThrottleZeroLeftover {
            throttle_zero: true,
            last_nonzero_throttle_ms: 1000,
        }
    );
}

#[test]
fn interlock_and_airmode_override_the_stick() {
    let interlock_off = ThrottleZeroInputs {
        using_interlock: true,
        throttle_control: 800,
        motor_interlock: false,
        ..tz(800, 0, 10, true)
    };
    assert!(!throttle_is_flying(&interlock_off));
    let interlock_on = ThrottleZeroInputs {
        motor_interlock: true,
        ..interlock_off
    };
    assert!(throttle_is_flying(&interlock_on));

    let estop = ThrottleZeroInputs {
        emergency_stop: true,
        throttle_control: 800,
        ..tz(800, 0, 10, true)
    };
    assert!(!throttle_is_flying(&estop));

    let air = ThrottleZeroInputs {
        air_mode: AirMode::Enabled,
        throttle_control: 0,
        ..tz(0, 0, 10, true)
    };
    assert!(throttle_is_flying(&air));
    let armed_air = ThrottleZeroInputs {
        armed_with_airmode_switch: true,
        throttle_control: 0,
        ..tz(0, 0, 10, true)
    };
    assert!(throttle_is_flying(&armed_air));
    assert!(!throttle_is_flying(&ThrottleZeroInputs {
        air_mode: AirMode::Disabled,
        ..tz(0, 0, 10, true)
    }));
}

fn radio_in(got: bool, now: u32, last: u32, failsafe: ThrottleFailsafeInputs) -> ReadRadioInputs {
    ReadRadioInputs {
        got_input: got,
        now_ms: now,
        last_radio_update_ms: last,
        fs_timeout_s: RC_FS_TIMEOUT_DEFAULT_S,
        failsafe,
        throttle_zero: tz(0, 0, now, true),
    }
}

#[test]
fn read_radio_frame_updates_failsafe_and_clock() {
    match read_radio(&radio_in(true, 250, 100, fs(1500))) {
        ReadRadioLeftover::Frame {
            failsafe,
            throttle_zero,
            last_radio_update_ms,
        } => {
            assert!(!failsafe.radio);
            assert_eq!(failsafe.radio_counter, 0);
            assert!(throttle_zero.throttle_zero);
            assert_eq!(last_radio_update_ms, 250);
        }
        other => panic!("expected Frame, got {other:?}"),
    }
}

#[test]
fn read_radio_timeout_ladder() {
    assert_eq!(
        read_radio(&radio_in(
            false,
            2000,
            0,
            ThrottleFailsafeInputs {
                radio: true,
                ..fs(1500)
            }
        )),
        ReadRadioLeftover::AlreadyFailed
    );
    assert_eq!(
        read_radio(&radio_in(false, 999, 0, fs(1500))),
        ReadRadioLeftover::Waiting,
        "999 ms is still below the 1000 ms default"
    );
    assert_eq!(
        read_radio(&radio_in(false, 1000, 0, fs(1500))),
        ReadRadioLeftover::LateFrame,
        "elapsed < timeout, so equality times out"
    );
    assert_eq!(
        read_radio(&radio_in(
            false,
            2000,
            0,
            ThrottleFailsafeInputs {
                failsafe_throttle: FS_THR_DISABLED,
                ..fs(1500)
            }
        )),
        ReadRadioLeftover::TimeoutDisabled
    );
    assert_eq!(
        read_radio(&radio_in(
            false,
            2000,
            0,
            ThrottleFailsafeInputs {
                has_ever_seen_rc_input: false,
                armed: false,
                ..fs(1500)
            }
        )),
        ReadRadioLeftover::NeverSeenDisarmed
    );
    assert_eq!(
        read_radio(&radio_in(
            false,
            2000,
            0,
            ThrottleFailsafeInputs {
                has_ever_seen_rc_input: false,
                armed: true,
                ..fs(1500)
            }
        )),
        ReadRadioLeftover::LateFrame
    );
}

#[test]
fn passthrough_uses_norm_input_and_zero_dz_throttle() {
    let map = init_rc_in_map(false);
    let mid = radio_passthrough_to_motors(&map, 1500, 1500, 1500, 1500);
    assert!(mid.roll.abs() < 1e-6);
    assert!(mid.pitch.abs() < 1e-6);
    assert!(mid.yaw.abs() < 1e-6);
    assert!(
        (mid.throttle - 0.5).abs() < 1e-4,
        "zero-dz mid collective is 0.5, not deadzoned 0.48; got {}",
        mid.throttle
    );
    let full = radio_passthrough_to_motors(&map, 1900, 1100, 1900, 1900);
    assert!((full.roll - 1.0).abs() < 1e-6);
    assert!((full.pitch + 1.0).abs() < 1e-6);
    assert!((full.throttle - 1.0).abs() < 1e-4);
    assert!((full.yaw - 1.0).abs() < 1e-6);
}

#[test]
fn init_rc_out_forces_defaults_when_throttle_unconfigured() {
    assert_eq!(
        init_rc_out(false, true, 1100, 1900, 0x000F),
        InitRcOutLeftover {
            motor_pwm: Some((1100, 1900)),
            esc_scaling: None,
            safety_ignore_mask: Some((!0x000F_u16) & SAFETY_IGNORE_MASK_BITS),
        }
    );
    assert_eq!(
        init_rc_out(false, false, 1100, 1900, 0x000F),
        InitRcOutLeftover {
            motor_pwm: Some((MOTOR_PWM_MIN_DEFAULT, MOTOR_PWM_MAX_DEFAULT)),
            esc_scaling: None,
            safety_ignore_mask: Some((!0x000F_u16) & SAFETY_IGNORE_MASK_BITS),
        },
        "unconfigured throttle must not let a later RC cal rewrite motors"
    );
    assert_eq!(
        init_rc_out(true, true, 1100, 1900, 0x000F),
        InitRcOutLeftover {
            motor_pwm: None,
            esc_scaling: Some((1100, 1900)),
            safety_ignore_mask: None,
        }
    );
}
