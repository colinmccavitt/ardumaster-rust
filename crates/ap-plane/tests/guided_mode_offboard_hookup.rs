//! GUIDED offboard `GUIDED_TIMEOUT` forced-RPY/throttle branch selection —
//! `ModeGuided::update()`'s own outer structure (FW-043, real lines 30-102).
//!
//! `calc_nav_roll`/`calc_nav_pitch`/`calc_throttle`/the heading-slew PID
//! value are all supplied as fixed external inputs in these tests — this
//! ticket only covers which of the three sources wins per axis, not those
//! larger separate computations.

use ap_plane::guided_mode_hookup::{
    guided_mode_offboard_tick, GuidedModeOffboardTickInputs, GuidedPitchSource, GuidedRollSource,
    GuidedThrottleSource,
};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_servo::function::Function;

const GUIDED_TIMEOUT_S: f32 = 3.0;

fn base_inp() -> GuidedModeOffboardTickInputs {
    GuidedModeOffboardTickInputs {
        control_mode: ModeNumber::Guided.as_number(),
        features: BuildFeatures::default(),
        vtol_loiter_active: false,
        now_ms: 10_000,
        forced_roll_cd: 0,
        last_forced_roll_ms: 0,
        forced_pitch_cd: 0,
        last_forced_pitch_ms: 0,
        forced_throttle: 0.0,
        last_forced_throttle_ms: 0,
        guided_timeout_s: GUIDED_TIMEOUT_S,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 3000,
        offboard_slew_enabled: true,
        heading_slew_active: false,
        heading_slew_nav_roll_cd: 0,
        calc_nav_roll_cd: 111,
        calc_nav_pitch_cd: 222,
        calc_throttle: 33.0,
        guided_throttle_passthru: false,
        throttle_passthru_input: 77.0,
        throttle_cruise: 65.0,
    }
}

// --- VTOL early return ------------------------------------------------

#[test]
fn vtol_loiter_active_short_circuits_everything() {
    let mut inp = base_inp();
    inp.vtol_loiter_active = true;
    // These would otherwise clearly win their own branches, proving the
    // early return really does short-circuit before any of roll/pitch/
    // throttle logic runs.
    inp.last_forced_roll_ms = 9_900;
    inp.last_forced_pitch_ms = 9_900;
    inp.guided_throttle_passthru = true;

    let out = guided_mode_offboard_tick(&inp);

    assert!(out.applied);
    assert!(out.vtol_early_return);
}

#[test]
fn non_guided_mode_does_not_apply_and_is_not_the_vtol_return() {
    let mut inp = base_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    let out = guided_mode_offboard_tick(&inp);
    assert!(!out.applied);
    assert!(!out.vtol_early_return);
}

// --- Roll: three-way, with the update_load_factor asymmetry -----------

#[test]
fn roll_forced_rpy_wins_within_timeout_and_calls_update_load_factor() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 9_900; // 100ms old, well under 3000ms
    inp.forced_roll_cd = 6000; // beyond +-4500 limit, must clamp
    let out = guided_mode_offboard_tick(&inp);

    assert!(out.applied);
    assert!(!out.vtol_early_return);
    assert_eq!(out.roll_source, GuidedRollSource::ForcedRpy);
    assert_eq!(out.nav_roll_cd, 4500); // constrain_int32(6000, -4500, 4500)
    assert!(out.update_load_factor);
}

#[test]
fn roll_forced_rpy_clamps_negative_beyond_symmetric_limit() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 9_900;
    inp.forced_roll_cd = -9000;
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::ForcedRpy);
    assert_eq!(out.nav_roll_cd, -4500);
    assert!(out.update_load_factor);
}

#[test]
fn roll_heading_slew_wins_when_forced_rpy_absent_and_calls_update_load_factor() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 0; // never heard from external controller
    inp.heading_slew_active = true;
    inp.heading_slew_nav_roll_cd = 1234;
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::HeadingSlew);
    assert_eq!(out.nav_roll_cd, 1234);
    assert!(out.update_load_factor);
}

#[test]
fn roll_heading_slew_wins_when_forced_rpy_has_expired() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 1_000; // 9000ms old, past 3000ms timeout
    inp.forced_roll_cd = 999; // would win if timeout were ignored
    inp.heading_slew_active = true;
    inp.heading_slew_nav_roll_cd = 1234;
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::HeadingSlew);
    assert_eq!(out.nav_roll_cd, 1234);
    assert!(out.update_load_factor);
}

#[test]
fn roll_heading_slew_gated_off_by_offboard_slew_enabled_flag() {
    let mut inp = base_inp();
    inp.heading_slew_active = true;
    inp.heading_slew_nav_roll_cd = 1234;
    inp.offboard_slew_enabled = false; // #if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED off
    let out = guided_mode_offboard_tick(&inp);

    // Falls through to calc_nav_roll() fallback instead.
    assert_eq!(out.roll_source, GuidedRollSource::CalcNavRoll);
    assert_eq!(out.nav_roll_cd, 111);
    assert!(!out.update_load_factor);
}

#[test]
fn roll_calc_nav_roll_fallback_wins_and_does_not_call_update_load_factor() {
    let inp = base_inp(); // no forced RPY, no heading-slew
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::CalcNavRoll);
    assert_eq!(out.nav_roll_cd, 111);
    // Real asymmetry: the plain fallback never calls update_load_factor(),
    // unlike the first two roll branches.
    assert!(!out.update_load_factor);
}

// --- Pitch: two-way, asymmetric clamp, never touches update_load_factor

#[test]
fn pitch_forced_rpy_wins_within_timeout_with_asymmetric_clamp() {
    let mut inp = base_inp();
    inp.last_forced_pitch_ms = 9_900;
    inp.forced_pitch_cd = 5000; // beyond max (3000), clamp shape differs from roll's +-limit
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.pitch_source, GuidedPitchSource::ForcedRpy);
    assert_eq!(out.nav_pitch_cd, 3000); // pitch_limit_max_cd, not a symmetric bound
                                        // Pitch never triggers update_load_factor, regardless of which pitch
                                        // branch won.
    assert!(!out.update_load_factor);
}

#[test]
fn pitch_forced_rpy_clamps_to_independent_min_not_negated_max() {
    let mut inp = base_inp();
    inp.last_forced_pitch_ms = 9_900;
    inp.forced_pitch_cd = -9000; // beyond min (-2000)
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.pitch_source, GuidedPitchSource::ForcedRpy);
    // pitch_limit_min_cd (-2000) is independently configured, NOT -max (-3000).
    assert_eq!(out.nav_pitch_cd, -2000);
    assert!(!out.update_load_factor);
}

#[test]
fn pitch_calc_nav_pitch_fallback_wins_when_never_forced() {
    let inp = base_inp();
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.pitch_source, GuidedPitchSource::CalcNavPitch);
    assert_eq!(out.nav_pitch_cd, 222);
    assert!(!out.update_load_factor);
}

#[test]
fn pitch_calc_nav_pitch_fallback_wins_once_forced_pitch_expires() {
    let mut inp = base_inp();
    inp.last_forced_pitch_ms = 1_000; // 9000ms old, past the 3000ms timeout
    inp.forced_pitch_cd = 2500;
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.pitch_source, GuidedPitchSource::CalcNavPitch);
    assert_eq!(out.nav_pitch_cd, 222);
}

// --- Roll/pitch .x/.y timeout windows are independent, not shared -----

#[test]
fn roll_expired_pitch_fresh_are_independent_windows() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 1_000; // expired (9000ms old)
    inp.forced_roll_cd = 4000;
    inp.last_forced_pitch_ms = 9_900; // fresh (100ms old)
    inp.forced_pitch_cd = 2500;

    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::CalcNavRoll);
    assert_eq!(out.nav_roll_cd, 111);
    assert_eq!(out.pitch_source, GuidedPitchSource::ForcedRpy);
    assert_eq!(out.nav_pitch_cd, 2500);
}

#[test]
fn roll_fresh_pitch_expired_are_independent_windows() {
    let mut inp = base_inp();
    inp.last_forced_roll_ms = 9_900; // fresh (100ms old)
    inp.forced_roll_cd = 4000;
    inp.last_forced_pitch_ms = 1_000; // expired (9000ms old)
    inp.forced_pitch_cd = 2500;

    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.roll_source, GuidedRollSource::ForcedRpy);
    assert_eq!(out.nav_roll_cd, 4000);
    assert_eq!(out.pitch_source, GuidedPitchSource::CalcNavPitch);
    assert_eq!(out.nav_pitch_cd, 222);
}

// --- Throttle: three-way, with the real throttle_cruise > 1 extra gate

#[test]
fn throttle_passthrough_wins_unconditionally() {
    let mut inp = base_inp();
    inp.guided_throttle_passthru = true;
    // Even a valid, recent forced-throttle message must not win — passthru
    // is unconditional.
    inp.last_forced_throttle_ms = 9_900;
    inp.forced_throttle = 55.0;
    inp.throttle_cruise = 65.0;

    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.throttle_source, GuidedThrottleSource::Passthrough);
    assert_eq!(out.throttle, (Function::THROTTLE, 77.0));
}

#[test]
fn throttle_forced_wins_when_recent_and_throttle_cruise_above_one() {
    let mut inp = base_inp();
    inp.last_forced_throttle_ms = 9_900; // 100ms old
    inp.forced_throttle = 55.0;
    inp.throttle_cruise = 65.0; // > 1

    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.throttle_source, GuidedThrottleSource::ForcedThrottle);
    assert_eq!(out.throttle, (Function::THROTTLE, 55.0));
}

#[test]
fn throttle_cruise_not_above_one_falls_through_to_tecs_even_with_valid_recent_message() {
    let mut inp = base_inp();
    inp.last_forced_throttle_ms = 9_900; // recent and otherwise valid
    inp.forced_throttle = 55.0;
    inp.throttle_cruise = 1.0; // NOT > 1 -- the real, easy-to-miss extra gate

    let out = guided_mode_offboard_tick(&inp);

    // Without the throttle_cruise > 1 gate this would wrongly pick
    // ForcedThrottle; the real extra condition must send it to calc_throttle.
    assert_eq!(out.throttle_source, GuidedThrottleSource::CalcThrottle);
    assert_eq!(out.throttle, (Function::THROTTLE, 33.0));
}

#[test]
fn throttle_forced_expires_and_falls_through_to_tecs() {
    let mut inp = base_inp();
    inp.last_forced_throttle_ms = 1_000; // 9000ms old, past 3000ms timeout
    inp.forced_throttle = 55.0;
    inp.throttle_cruise = 65.0;

    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.throttle_source, GuidedThrottleSource::CalcThrottle);
    assert_eq!(out.throttle, (Function::THROTTLE, 33.0));
}

#[test]
fn throttle_never_forced_falls_through_to_tecs() {
    let inp = base_inp(); // last_forced_throttle_ms == 0
    let out = guided_mode_offboard_tick(&inp);

    assert_eq!(out.throttle_source, GuidedThrottleSource::CalcThrottle);
    assert_eq!(out.throttle, (Function::THROTTLE, 33.0));
}
