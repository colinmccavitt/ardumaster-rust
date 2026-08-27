use ap_gps::{FixType, GpsStatus};
use ap_math::vector3::Vector3f;
use ap_plane::entry_state::{AutoState, CrashState, ModeEntryState, SteerState};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::mode_transition_throttle_hookup::{
    mode_transition_throttle_tick, ModeTransitionThrottleInputs,
};

fn fresh_entry(suppressed: bool) -> ModeEntryState {
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
        throttle_suppressed: suppressed,
    }
}

fn moving_gps() -> GpsStatus {
    GpsStatus {
        fix_type: FixType::Fix3D,
        num_sats: 12,
        have_fix: true,
        lag_sec: 0.0,
        velocity_ned: Vector3f::new(6.0, 0.0, 0.0),
        ground_speed: 6.0,
        ground_course_deg: 0.0,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 100.0,
        last_fix_time_ms: 0,
    }
}

#[test]
fn manual_mode_clears_suppression_flag() {
    let mut entry = fresh_entry(true);
    let out = mode_transition_throttle_tick(
        &mut entry,
        &ModeTransitionThrottleInputs {
            control_mode: ModeNumber::Manual.as_number(),
            relative_altitude_m: 0.0,
            gps: None,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.cleared);
    assert!(!out.throttle_suppressed);
    assert!(!entry.throttle_suppressed);
}

#[test]
fn stabilize_mode_clears_suppression_flag() {
    let mut entry = fresh_entry(true);
    let out = mode_transition_throttle_tick(
        &mut entry,
        &ModeTransitionThrottleInputs {
            control_mode: ModeNumber::Stabilize.as_number(),
            relative_altitude_m: 0.0,
            gps: None,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.cleared);
    assert!(!entry.throttle_suppressed);
}

#[test]
fn auto_mode_keeps_suppression_until_altitude() {
    let mut entry = fresh_entry(true);
    let out = mode_transition_throttle_tick(
        &mut entry,
        &ModeTransitionThrottleInputs {
            control_mode: ModeNumber::Auto.as_number(),
            relative_altitude_m: 2.0,
            gps: None,
            features: BuildFeatures::default(),
        },
    );
    assert!(!out.cleared);
    assert!(out.throttle_suppressed);
    assert!(entry.throttle_suppressed);
}

#[test]
fn auto_mode_clears_on_relative_altitude() {
    let mut entry = fresh_entry(true);
    let out = mode_transition_throttle_tick(
        &mut entry,
        &ModeTransitionThrottleInputs {
            control_mode: ModeNumber::Auto.as_number(),
            relative_altitude_m: 12.0,
            gps: None,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.cleared);
    assert!(!entry.throttle_suppressed);
}

#[test]
fn auto_mode_clears_on_gps_movement() {
    let mut entry = fresh_entry(true);
    let out = mode_transition_throttle_tick(
        &mut entry,
        &ModeTransitionThrottleInputs {
            control_mode: ModeNumber::Auto.as_number(),
            relative_altitude_m: 0.0,
            gps: Some(moving_gps()),
            features: BuildFeatures::default(),
        },
    );
    assert!(out.cleared);
    assert!(!entry.throttle_suppressed);
}
