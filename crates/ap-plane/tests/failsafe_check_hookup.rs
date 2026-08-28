//! Main-loop lockup heartbeat, upstream `Plane::failsafe_check`.
//!
//! The 1 kHz timer interrupt treats an advancing `scheduler.ticks()` as
//! the last-heartbeat that the scheduler is still running. A 200 ms stall
//! latches `in_failsafe`; while latched, every 20 ms the interrupt would
//! pass RC through and pulse `afs.heartbeat()` during calibration.

use ap_plane::failsafe_check_hookup::{
    failsafe_check, FailsafeCheckInputs, FailsafeCheckState, FAILSAFE_CHECK_LOCKUP_US,
    FAILSAFE_CHECK_MIN_RC_CHANNELS, FAILSAFE_CHECK_PASSTHROUGH_US,
    FAILSAFE_CHECK_TIMER_PERIOD_US,
};

fn healthy(ticks: u16, now_us: u32) -> FailsafeCheckInputs {
    FailsafeCheckInputs {
        now_us,
        scheduler_ticks: ticks,
        in_calibration: false,
        valid_channel_count: 8,
        armed_and_safety_off: true,
    }
}

#[test]
fn timer_period_and_ages_match_upstream() {
    assert_eq!(FAILSAFE_CHECK_TIMER_PERIOD_US, 1_000);
    assert_eq!(FAILSAFE_CHECK_LOCKUP_US, 200_000);
    assert_eq!(FAILSAFE_CHECK_PASSTHROUGH_US, 20_000);
    assert_eq!(FAILSAFE_CHECK_MIN_RC_CHANNELS, 5);
}

#[test]
fn advancing_ticks_are_a_healthy_heartbeat() {
    let mut state = FailsafeCheckState::default();
    let out = failsafe_check(&state, &healthy(1, 1_000));
    assert!(!out.in_failsafe);
    assert!(!out.pass_rc_through);
    assert!(!out.afs_heartbeat);
    assert_eq!(out.last_ticks, 1);
    assert_eq!(out.last_timestamp_us, 1_000);
    state.apply(&out);

    let later = failsafe_check(&state, &healthy(2, 5_000));
    assert!(!later.in_failsafe);
    assert_eq!(later.last_ticks, 2);
    assert_eq!(later.last_timestamp_us, 5_000);
}

#[test]
fn stalled_ticks_hold_until_200ms_then_latch() {
    let mut state = FailsafeCheckState {
        last_ticks: 4,
        last_timestamp_us: 10_000,
        in_failsafe: false,
    };
    let at_deadline = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: 10_000 + FAILSAFE_CHECK_LOCKUP_US,
            scheduler_ticks: 4,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(
        !at_deadline.in_failsafe,
        "upstream uses exclusive `>` against 200000 us"
    );
    assert!(!at_deadline.pass_rc_through);

    let past = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: 10_000 + FAILSAFE_CHECK_LOCKUP_US + 1,
            scheduler_ticks: 4,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(past.in_failsafe);
    assert!(
        past.pass_rc_through,
        "first lockup age is also older than the 20 ms passthrough window"
    );
    assert_eq!(past.last_timestamp_us, 10_000 + FAILSAFE_CHECK_LOCKUP_US + 1);
    state.apply(&past);
    assert!(state.in_failsafe);
}

#[test]
fn lockup_passthrough_repeats_every_20ms() {
    let mut state = FailsafeCheckState {
        last_ticks: 7,
        last_timestamp_us: 0,
        in_failsafe: true,
    };
    let too_soon = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_PASSTHROUGH_US,
            scheduler_ticks: 7,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(too_soon.in_failsafe);
    assert!(!too_soon.pass_rc_through);
    assert_eq!(too_soon.last_timestamp_us, 0);

    let pulse = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_PASSTHROUGH_US + 1,
            scheduler_ticks: 7,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(pulse.pass_rc_through);
    assert!(!pulse.afs_heartbeat);
    assert!(!pulse.zero_throttle);
    assert_eq!(pulse.last_timestamp_us, FAILSAFE_CHECK_PASSTHROUGH_US + 1);
    state.apply(&pulse);

    let next = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: state.last_timestamp_us + FAILSAFE_CHECK_PASSTHROUGH_US + 1,
            scheduler_ticks: 7,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(next.pass_rc_through);
}

#[test]
fn calibration_pulses_afs_heartbeat_only_on_passthrough() {
    let state = FailsafeCheckState {
        last_ticks: 1,
        last_timestamp_us: 0,
        in_failsafe: true,
    };
    let quiet = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: 10_000,
            scheduler_ticks: 1,
            in_calibration: true,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(quiet.in_failsafe);
    assert!(!quiet.afs_heartbeat);

    let pulse = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_PASSTHROUGH_US + 1,
            scheduler_ticks: 1,
            in_calibration: true,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(pulse.afs_heartbeat);
    assert!(pulse.pass_rc_through);
}

#[test]
fn fewer_than_five_rc_channels_blocks_passthrough_but_still_heartbeats_afs() {
    let state = FailsafeCheckState {
        last_ticks: 3,
        last_timestamp_us: 0,
        in_failsafe: true,
    };
    let out = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_PASSTHROUGH_US + 1,
            scheduler_ticks: 3,
            in_calibration: true,
            valid_channel_count: 4,
            armed_and_safety_off: true,
        },
    );
    assert!(out.in_failsafe);
    assert!(!out.pass_rc_through);
    assert!(
        out.afs_heartbeat,
        "afs.heartbeat() runs before the channel-count return"
    );
    assert!(!out.zero_throttle);
    assert_eq!(out.last_timestamp_us, FAILSAFE_CHECK_PASSTHROUGH_US + 1);
}

#[test]
fn disarmed_passthrough_forces_zero_throttle() {
    let state = FailsafeCheckState {
        last_ticks: 9,
        last_timestamp_us: 0,
        in_failsafe: true,
    };
    let out = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_PASSTHROUGH_US + 1,
            scheduler_ticks: 9,
            armed_and_safety_off: false,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(out.pass_rc_through);
    assert!(out.zero_throttle);
}

#[test]
fn recovering_ticks_clear_in_failsafe_without_passthrough() {
    let state = FailsafeCheckState {
        last_ticks: 20,
        last_timestamp_us: 0,
        in_failsafe: true,
    };
    let out = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: 1_000_000,
            scheduler_ticks: 21,
            in_calibration: true,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(!out.in_failsafe);
    assert!(!out.pass_rc_through);
    assert!(!out.afs_heartbeat);
}

#[test]
fn micros_wrap_still_detects_lockup() {
    let state = FailsafeCheckState {
        last_ticks: 1,
        last_timestamp_us: u32::MAX - 1_000,
        in_failsafe: false,
    };
    let hold = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_LOCKUP_US - 2_002,
            scheduler_ticks: 1,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(!hold.in_failsafe);

    let lock = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            // (MAX-1000) wrapping_add 200000 == LOCKUP-1001 → age == 200000
            now_us: FAILSAFE_CHECK_LOCKUP_US - 1_001,
            scheduler_ticks: 1,
            ..FailsafeCheckInputs::default()
        },
    );
    // exclusive `>` so exactly 200000 still holds; +1 locks.
    assert!(!lock.in_failsafe);

    let past = failsafe_check(
        &state,
        &FailsafeCheckInputs {
            now_us: FAILSAFE_CHECK_LOCKUP_US - 1_000,
            scheduler_ticks: 1,
            ..FailsafeCheckInputs::default()
        },
    );
    assert!(past.in_failsafe);
}
