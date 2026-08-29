//! GUIDED offboard airspeed ramp and altitude stepping (FW-045).
//!
//! Covers envelope reject, same-target early return, zero-accel → 1000,
//! sign vs `plane.target_airspeed_cm`, default alt -1 / time 0 fall-through,
//! stepping in the target location's own alt frame, and constrain toward
//! the target from the current altitude.

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::scalar::{constrain_int32, is_equal};
use ap_plane::guided_offboard_alt::{
    handle_change_airspeed, target_location_alt_is_minus_one, update_target_altitude,
    GuidedUpdateTargetAltitude, GuidedUpdateTargetAltitudeInputs, HandleChangeAirspeedInputs,
};

fn airspeed_inp() -> HandleChangeAirspeedInputs {
    HandleChangeAirspeedInputs {
        airspeed: 16.0,
        acceleration: 2.0,
        airspeed_min: 9.0,
        airspeed_max: 22.0,
        guided_target_airspeed_cm: -1.0,
        plane_target_airspeed_cm: 1500.0,
        now_ms: 12_000,
    }
}

fn default_offboard_target() -> Location {
    let mut loc = Location::new(0, 0);
    loc.set_alt_cm(-1, AltFrame::Absolute);
    loc
}

fn placed(lat: i32, lng: i32, alt_cm: i32, frame: AltFrame) -> Location {
    Location::new_with_alt(lat, lng, alt_cm, frame)
}

fn expected_delta_amt_i(now_ms: u32, target_alt_time_ms: u32, target_alt_rate: f32) -> i32 {
    let delta = 1e-3_f32 * now_ms.wrapping_sub(target_alt_time_ms) as f32;
    let delta_amt_f = delta * target_alt_rate;
    (100.0_f64 * f64::from(delta_amt_f)) as i32
}

fn base_alt_inp() -> GuidedUpdateTargetAltitudeInputs {
    GuidedUpdateTargetAltitudeInputs {
        now_ms: 5_000,
        target_alt_time_ms: 0,
        target_alt_rate: 10.0,
        target_location: default_offboard_target(),
        current_loc: Location::new(0, 0),
        alt_ctx: AltContext::default(),
    }
}

// --- handle_change_airspeed --------------------------------------------------

#[test]
fn envelope_rejects_above_max() {
    let mut inp = airspeed_inp();
    inp.airspeed = inp.airspeed_max + 0.1;
    let out = handle_change_airspeed(&inp);
    assert!(!out.accepted);
    assert!(out.ramp.is_none());
}

#[test]
fn envelope_rejects_below_min() {
    let mut inp = airspeed_inp();
    inp.airspeed = inp.airspeed_min - 0.1;
    let out = handle_change_airspeed(&inp);
    assert!(!out.accepted);
    assert!(out.ramp.is_none());
}

#[test]
fn envelope_accepts_the_closed_bounds() {
    let mut inp = airspeed_inp();
    inp.airspeed = inp.airspeed_min;
    assert!(handle_change_airspeed(&inp).accepted);
    inp.airspeed = inp.airspeed_max;
    assert!(handle_change_airspeed(&inp).accepted);
}

#[test]
fn same_target_is_a_noop_and_keeps_existing_accel_time() {
    let mut inp = airspeed_inp();
    inp.guided_target_airspeed_cm = 16.0 * 100.0;
    inp.acceleration = 99.0;
    inp.now_ms = 99_000;
    let out = handle_change_airspeed(&inp);
    assert!(out.accepted);
    assert!(
        out.ramp.is_none(),
        "same-target must not rewrite accel or time"
    );
}

#[test]
fn zero_acceleration_stores_1000() {
    let mut inp = airspeed_inp();
    inp.acceleration = 0.0;
    // New target 16 m/s is above the current 15 m/s vehicle target, so sign stays +.
    let out = handle_change_airspeed(&inp);
    let ramp = out.ramp.expect("new target must write a ramp");
    assert!(out.accepted);
    assert!(is_equal(ramp.target_airspeed_cm, 1600.0));
    assert_eq!(ramp.target_airspeed_time_ms, inp.now_ms);
    assert!(is_equal(ramp.target_airspeed_accel, 1000.0));
}

#[test]
fn non_zero_acceleration_uses_fabs() {
    let mut inp = airspeed_inp();
    inp.acceleration = -2.5;
    let ramp = handle_change_airspeed(&inp).ramp.expect("ramp");
    assert!(is_equal(ramp.target_airspeed_accel, 2.5));
}

#[test]
fn accel_sign_is_negative_when_new_target_is_below_current_vehicle_target() {
    let mut inp = airspeed_inp();
    inp.airspeed = 12.0;
    inp.plane_target_airspeed_cm = 1500.0;
    inp.acceleration = 3.0;
    let ramp = handle_change_airspeed(&inp).ramp.expect("ramp");
    assert!(is_equal(ramp.target_airspeed_cm, 1200.0));
    assert!(
        is_equal(ramp.target_airspeed_accel, -3.0),
        "sign vs plane.target_airspeed_cm, not the previous guided target"
    );
}

#[test]
fn accel_sign_stays_positive_when_new_target_is_at_or_above_current() {
    let mut inp = airspeed_inp();
    inp.airspeed = 15.0;
    inp.plane_target_airspeed_cm = 1500.0;
    inp.acceleration = 3.0;
    let at = handle_change_airspeed(&inp).ramp.expect("ramp");
    assert!(is_equal(at.target_airspeed_accel, 3.0));

    inp.airspeed = 18.0;
    let above = handle_change_airspeed(&inp).ramp.expect("ramp");
    assert!(is_equal(above.target_airspeed_accel, 3.0));
}

#[test]
fn accel_sign_uses_plane_target_not_the_previous_guided_target() {
    // Previous guided demand already 12 m/s, vehicle is still at 18 m/s,
    // new demand 14 m/s: 1400 < 1800 so accel is negative even though
    // 14 > the old guided 12.
    let mut inp = airspeed_inp();
    inp.guided_target_airspeed_cm = 1200.0;
    inp.plane_target_airspeed_cm = 1800.0;
    inp.airspeed = 14.0;
    inp.acceleration = 4.0;
    let ramp = handle_change_airspeed(&inp).ramp.expect("ramp");
    assert!(is_equal(ramp.target_airspeed_accel, -4.0));
}

// --- target_location_alt_is_minus_one ----------------------------------------

#[test]
fn minus_one_uses_the_locations_own_frame() {
    let ctx = AltContext::default();
    let mut absolute = Location::new(0, 0);
    absolute.set_alt_cm(-1, AltFrame::Absolute);
    assert!(target_location_alt_is_minus_one(&absolute, &ctx));

    // Same sentinel in AboveHome, and no home in ctx — an assumed Absolute
    // conversion would fail and report "not -1".
    let mut above_home = Location::new(0, 0);
    above_home.set_alt_cm(-1, AltFrame::AboveHome);
    assert!(
        target_location_alt_is_minus_one(&above_home, &ctx),
        "must read get_alt_cm(own frame), not an assumed frame"
    );

    let mut other = Location::new(0, 0);
    other.set_alt_cm(0, AltFrame::Absolute);
    assert!(!target_location_alt_is_minus_one(&other, &ctx));
}

// --- update_target_altitude --------------------------------------------------

#[test]
fn default_alt_minus_one_and_time_zero_falls_through() {
    let inp = base_alt_inp();
    assert_eq!(inp.target_alt_time_ms, 0);
    assert!(target_location_alt_is_minus_one(
        &inp.target_location,
        &inp.alt_ctx
    ));
    assert_eq!(
        update_target_altitude(&inp),
        GuidedUpdateTargetAltitude::UseBaseMode
    );
}

#[test]
fn minus_one_in_non_absolute_frame_still_falls_through_at_time_zero() {
    let mut inp = base_alt_inp();
    inp.target_location.set_alt_cm(-1, AltFrame::AboveHome);
    // No home — assumed-Absolute get_alt_cm would fail and incorrectly enter.
    inp.alt_ctx = AltContext::default();
    assert_eq!(
        update_target_altitude(&inp),
        GuidedUpdateTargetAltitude::UseBaseMode
    );
}

#[test]
fn time_nonzero_enters_offboard_even_when_alt_is_minus_one() {
    let mut inp = base_alt_inp();
    inp.target_alt_time_ms = 1_000;
    inp.current_loc = placed(-35_000_000, 149_000_000, 10_000, AltFrame::Absolute);
    inp.target_location = placed(-35_000_000, 149_000_000, -1, AltFrame::Absolute);
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            target_alt_time_ms, ..
        } => assert_eq!(target_alt_time_ms, inp.now_ms),
        GuidedUpdateTargetAltitude::UseBaseMode => panic!("time != 0 must enter offboard"),
    }
}

#[test]
fn uninitialised_locations_consume_time_but_do_not_set() {
    let mut inp = base_alt_inp();
    inp.target_location.set_alt_cm(8_000, AltFrame::Absolute);
    // lat/lng still 0 → not initialised.
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location,
            target_alt_time_ms,
        } => {
            assert!(set_target_altitude_location.is_none());
            assert_eq!(target_alt_time_ms, inp.now_ms);
        }
        GuidedUpdateTargetAltitude::UseBaseMode => panic!("alt != -1 must enter offboard"),
    }
}

#[test]
fn steps_in_the_targets_own_frame_not_an_assumed_frame() {
    // current 50 m AMSL, home 20 m AMSL → 30 m above home.
    // target 80 m above home. A 1 s step at 10 m/s is 1000 cm.
    // constrain(8000, 3000-1000, 3000+1000) = 4000 in AboveHome.
    // If the previous alt were wrongly taken as Absolute 5000:
    // constrain(8000, 4000, 6000) = 6000 — a different answer.
    let mut inp = base_alt_inp();
    inp.now_ms = 1_000;
    inp.target_alt_time_ms = 0;
    inp.target_alt_rate = 10.0;
    inp.current_loc = placed(-35_000_000, 149_000_000, 5_000, AltFrame::Absolute);
    inp.target_location = placed(-35_000_000, 149_100_000, 8_000, AltFrame::AboveHome);
    inp.alt_ctx = AltContext {
        home_alt_cm: Some(2_000),
        origin_alt_cm: None,
        terrain_alt_cm: None,
    };

    let previous = inp
        .current_loc
        .get_alt_cm(AltFrame::AboveHome, &inp.alt_ctx)
        .expect("home is set");
    assert_eq!(previous, 3_000);
    let delta_amt_i = expected_delta_amt_i(inp.now_ms, inp.target_alt_time_ms, inp.target_alt_rate);
    assert_eq!(delta_amt_i, 1_000);
    let want = constrain_int32(8_000, previous - delta_amt_i, previous + delta_amt_i);
    assert_eq!(want, 4_000);
    let assumed_absolute = constrain_int32(8_000, 5_000 - delta_amt_i, 5_000 + delta_amt_i);
    assert_ne!(
        want, assumed_absolute,
        "the frame choice must change the stepped alt"
    );

    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location: Some(loc),
            target_alt_time_ms,
        } => {
            assert_eq!(target_alt_time_ms, inp.now_ms);
            assert_eq!(loc.alt_frame(), AltFrame::AboveHome);
            assert_eq!(loc.alt, want);
            assert_eq!(loc.lat, inp.target_location.lat);
            assert_eq!(loc.lng, inp.target_location.lng);
        }
        other => panic!("expected a set in the target frame, got {other:?}"),
    }
}

#[test]
fn constrains_toward_target_from_current() {
    let mut inp = base_alt_inp();
    inp.now_ms = 1_000;
    inp.target_alt_time_ms = 0;
    inp.target_alt_rate = 10.0;
    inp.current_loc = placed(-35_000_000, 149_000_000, 3_000, AltFrame::Absolute);
    inp.target_location = placed(-35_000_000, 149_100_000, 8_000, AltFrame::Absolute);
    inp.alt_ctx = AltContext::default();

    let delta_amt_i = expected_delta_amt_i(inp.now_ms, inp.target_alt_time_ms, inp.target_alt_rate);
    // Climbing: one step from 30 m toward 80 m is 40 m, not the full 80 m.
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location: Some(loc),
            ..
        } => {
            assert_eq!(loc.alt, 3_000 + delta_amt_i);
            assert_ne!(loc.alt, inp.target_location.alt);
        }
        other => panic!("expected a constrained set, got {other:?}"),
    }

    // Already past the remaining distance: snap to the target, not beyond it.
    inp.current_loc.set_alt_cm(7_500, AltFrame::Absolute);
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location: Some(loc),
            ..
        } => assert_eq!(loc.alt, 8_000),
        other => panic!("expected snap to target, got {other:?}"),
    }

    // Descending toward a lower target from above.
    inp.current_loc.set_alt_cm(10_000, AltFrame::Absolute);
    inp.target_location.set_alt_cm(5_000, AltFrame::Absolute);
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location: Some(loc),
            ..
        } => assert_eq!(loc.alt, 10_000 - delta_amt_i),
        other => panic!("expected a down-step, got {other:?}"),
    }
}

#[test]
fn missing_frame_context_skips_the_set_but_still_consumes_time() {
    let mut inp = base_alt_inp();
    inp.current_loc = placed(-35_000_000, 149_000_000, 5_000, AltFrame::Absolute);
    inp.target_location = placed(-35_000_000, 149_100_000, 8_000, AltFrame::AboveHome);
    inp.alt_ctx = AltContext::default(); // no home → get_alt_cm fails
    match update_target_altitude(&inp) {
        GuidedUpdateTargetAltitude::Offboard {
            set_target_altitude_location,
            target_alt_time_ms,
        } => {
            assert!(set_target_altitude_location.is_none());
            assert_eq!(target_alt_time_ms, inp.now_ms);
        }
        GuidedUpdateTargetAltitude::UseBaseMode => panic!("must stay on the offboard path"),
    }
}
