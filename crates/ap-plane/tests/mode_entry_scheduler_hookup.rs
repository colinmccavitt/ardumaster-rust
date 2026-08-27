use ap_plane::entry_state::{AutoState, CrashState, ModeEntryState, SteerState};
use ap_plane::mode_entry_scheduler_hookup::{
    mode_entry_scheduler_tick, ModeEntrySchedulerInputs,
};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn fresh_entry() -> ModeEntryState {
    ModeEntryState {
        auto: AutoState {
            inverted_flight: false,
            next_wp_crosstrack: false,
            checked_for_autoland: false,
            highest_airspeed: 0.0,
            initial_pitch_cd: 0,
            fbwa_tdrag_takeoff_mode: false,
            rotation_complete: false,
            vtol_mode: false,
            vtol_loiter: false,
            idle_mode: false,
        },
        steer: SteerState {
            locked_course: false,
            locked_course_err: 0.0,
        },
        crash: CrashState {
            is_crashed: false,
            impact_detected: false,
        },
        waiting_for_rudder_neutral: false,
        loiter_start_time_ms: 0,
        new_airspeed_cm: -1,
        long_failsafe_pending: false,
        throttle_suppressed: false,
    }
}

#[test]
fn no_reset_when_mode_unchanged() {
    let mut entry = fresh_entry();
    entry.auto.inverted_flight = true;
    let out = mode_entry_scheduler_tick(
        &mut entry,
        &ModeEntrySchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            previous_tracked_mode: ModeNumber::Auto.as_number(),
            current_pitch_cd: 500,
            features: BuildFeatures::default(),
        },
    );
    assert!(!out.mode_changed);
    assert!(entry.auto.inverted_flight);
}

#[test]
fn reset_clears_sentinel_on_mode_change() {
    let mut entry = fresh_entry();
    entry.auto.inverted_flight = true;
    entry.auto.initial_pitch_cd = -999;
    entry.steer.locked_course = true;
    entry.steer.locked_course_err = 1.5;
    entry.new_airspeed_cm = 777;
    let out = mode_entry_scheduler_tick(
        &mut entry,
        &ModeEntrySchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            previous_tracked_mode: ModeNumber::Manual.as_number(),
            current_pitch_cd: 1200,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.mode_changed);
    assert_eq!(out.tracked_mode, ModeNumber::Auto.as_number());
    assert!(!entry.auto.inverted_flight);
    assert!(!entry.steer.locked_course);
    assert_eq!(entry.auto.initial_pitch_cd, 1200);
    assert_eq!(entry.new_airspeed_cm, -1);
    assert!(entry.throttle_suppressed);
}

#[test]
fn manual_mode_does_not_suppress_throttle() {
    let mut entry = fresh_entry();
    let out = mode_entry_scheduler_tick(
        &mut entry,
        &ModeEntrySchedulerInputs {
            control_mode: ModeNumber::Manual.as_number(),
            previous_tracked_mode: ModeNumber::Auto.as_number(),
            current_pitch_cd: 0,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.mode_changed);
    assert!(!entry.throttle_suppressed);
}
