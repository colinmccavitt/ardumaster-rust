//! The pilot's part in a landing, against the real firmware.
//!
//! None of these decisions is a return value — the function returns nothing
//! and works through vehicle state and controller calls — so the recording
//! carries the flight mode after the call, the two Copter flags, and whether
//! the position controller was softened.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::land_horizontal::{
    land_cancel_destination, land_cancelled_by_throttle, land_horizontal_input,
    max_pilot_reposition_speed_ms, precision_landing_active, reposition_state,
    LandCancelDestination, LandHorizontalInput, RepositionState,
};

/// Upstream's mode numbers for the two the recording can end in.
const MODE_LOITER: i32 = 5;
const MODE_LAND: i32 = 9;

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_land_horizontal.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_landing_pilot_controls_match_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut cancelled = 0_usize;
    let mut took_over = 0_usize;
    let mut released = 0_usize;
    let mut precland = 0_usize;
    let mut softened = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 13, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let land_complete_maybe = b(&r[1]);
        let throttle_behavior: i32 = r[2].trim().parse().expect("throttle behavior");
        let throttle_filtered: f32 = r[3].trim().parse().expect("throttle");
        let repositioning = b(&r[4]);
        let roll_in: i32 = r[5].trim().parse().expect("roll");
        let repo_before = b(&r[6]);
        let target_acquired = b(&r[7]);
        let allow_after_repo = b(&r[8]);

        let mode_after: i32 = r[9].trim().parse().expect("mode");
        let repo_after = b(&r[10]);
        let prec_active = b(&r[11]);
        let was_softened = b(&r[12]);

        // The RC has been seen throughout the recording.
        let has_valid_input = true;

        // --- the cancel ---
        let cancels =
            land_cancelled_by_throttle(throttle_behavior, throttle_filtered, has_valid_input);
        let ended_elsewhere = mode_after != MODE_LAND;
        assert_eq!(
            cancels, ended_elsewhere,
            "row {idx}: port says cancel={cancels}, upstream ended in mode \
             {mode_after}"
        );
        if cancels {
            cancelled += 1;
            // Every recorded cancel was accepted by Loiter; the Alt-Hold
            // fallback is not reachable here and is pinned separately.
            assert_eq!(land_cancel_destination(true), LandCancelDestination::Loiter);
            assert_eq!(mode_after, MODE_LOITER, "row {idx}");
        }

        // --- the reposition state ---
        //
        // The stick is either centred or clearly deflected, so the pilot's
        // velocity is zero exactly when the stick is centred.
        let pilot_velocity_is_zero = roll_in == 1500;
        let state = reposition_state(
            repositioning,
            has_valid_input,
            pilot_velocity_is_zero,
            allow_after_repo,
        );

        let want_repo_after = match state {
            RepositionState::PilotRepositioning => true,
            RepositionState::ReleasedToPrecland => false,
            RepositionState::Unchanged => repo_before,
        };
        assert_eq!(
            want_repo_after, repo_after,
            "row {idx}: reposition state {state:?} gives {want_repo_after}, \
             upstream {repo_after}"
        );
        match state {
            RepositionState::PilotRepositioning => took_over += 1,
            RepositionState::ReleasedToPrecland => released += 1,
            RepositionState::Unchanged => {}
        }

        // --- precision landing ---
        let want_prec = precision_landing_active(repo_after, target_acquired);
        assert_eq!(
            want_prec, prec_active,
            "row {idx}: precland active {want_prec} against upstream \
             {prec_active}"
        );
        if prec_active {
            precland += 1;
            assert_eq!(
                land_horizontal_input(prec_active),
                LandHorizontalInput::PrecisionTarget
            );
        } else {
            assert_eq!(
                land_horizontal_input(prec_active),
                LandHorizontalInput::VelocityCorrection
            );
        }

        // --- the softening ---
        assert_eq!(
            land_complete_maybe, was_softened,
            "row {idx}: the controller should be softened exactly when the \
             landing detector is unsure"
        );
        if was_softened {
            softened += 1;
        }
    }

    assert!(
        cancelled > 0 && cancelled < rows.len(),
        "the cancel never varies"
    );
    assert!(took_over > 0, "no row had the pilot repositioning");
    assert!(released > 0, "no row released back to precision landing");
    assert!(precland > 0, "precision landing is never active");
    assert!(softened > 0, "the controller is never softened");

    println!(
        "{} rows: {cancelled} cancelled, {took_over} repositioning, \
         {released} released, {precland} precland, {softened} softened",
        rows.len()
    );
}

/// Cancelling a landing by throttle is opt-in, and reads the filtered stick.
///
/// A pilot resting a hand on the throttle during an automatic descent should
/// not thereby abort it, which is why the behaviour sits behind a `THR_BEHAVE`
/// bit. And upstream reads the filtered throttle rather than the raw stick,
/// so one noisy sample cannot put the aircraft back in the air with a pilot
/// who was not expecting to be flying.
#[test]
fn cancelling_a_landing_is_opt_in_and_needs_a_sustained_stick() {
    // The bit clear: no throttle cancels.
    for throttle in [0.0_f32, 700.0, 701.0, 1000.0] {
        assert!(
            !land_cancelled_by_throttle(0, throttle, true),
            "throttle {throttle} cancelled with the option off"
        );
    }

    // The bit set: strictly above the threshold.
    assert!(
        !land_cancelled_by_throttle(2, 700.0, true),
        "at the threshold"
    );
    assert!(land_cancelled_by_throttle(2, 700.1, true), "just above it");

    // Other bits in the parameter do not enable it.
    assert!(!land_cancelled_by_throttle(1, 900.0, true));
    assert!(!land_cancelled_by_throttle(4, 900.0, true));
    assert!(
        land_cancelled_by_throttle(3, 900.0, true),
        "bit 1 among others"
    );

    // And no radio means no cancel, whatever the stale throttle says.
    assert!(!land_cancelled_by_throttle(2, 1000.0, false));
}

/// A cancelled landing prefers Loiter and falls back to Alt-Hold.
///
/// The order is the useful one: Loiter holds position as well as height, so a
/// pilot who has just grabbed the aircraft gets it stopped rather than
/// drifting. Alt-Hold needs no position estimate, which is the likeliest
/// reason Loiter would refuse.
///
/// Every recorded cancel was accepted by Loiter, so the fallback is pinned
/// here rather than by the recording.
#[test]
fn a_cancelled_landing_falls_back_to_alt_hold() {
    assert_eq!(land_cancel_destination(true), LandCancelDestination::Loiter);
    assert_eq!(
        land_cancel_destination(false),
        LandCancelDestination::AltHold
    );
}

/// Letting go of the sticks does not automatically give the landing back.
///
/// `land_repo_active` stays set after the pilot releases, unless
/// `PLND_OPTION_PRECLAND_AFTER_REPOSITION` says otherwise. The default is
/// that a pilot who has intervened has taken the landing, and the precision
/// target — which they presumably moved away from on purpose — does not get
/// to pull the aircraft back.
#[test]
fn releasing_the_sticks_does_not_hand_the_landing_back_by_default() {
    // Repositioning: the pilot takes it.
    assert_eq!(
        reposition_state(true, true, false, false),
        RepositionState::PilotRepositioning
    );

    // Released, option off: nothing changes, so a previously-set flag stays.
    assert_eq!(
        reposition_state(true, true, true, false),
        RepositionState::Unchanged
    );

    // Released, option on: precision landing may resume.
    assert_eq!(
        reposition_state(true, true, true, true),
        RepositionState::ReleasedToPrecland
    );

    // Repositioning disabled entirely, or no radio: the pilot cannot take it
    // however the sticks are set.
    assert_eq!(
        reposition_state(false, true, false, true),
        RepositionState::Unchanged
    );
    assert_eq!(
        reposition_state(true, false, false, true),
        RepositionState::Unchanged
    );
}

/// A repositioning pilot outranks an acquired target.
///
/// The target does not know why the pilot moved the aircraft, so it must not
/// pull it back while they are still asking for something else.
#[test]
fn a_repositioning_pilot_outranks_the_precision_target() {
    assert!(!precision_landing_active(true, true));
    assert!(precision_landing_active(false, true));
    assert!(!precision_landing_active(false, false));
    assert!(!precision_landing_active(true, false));

    // And exactly one input drives the controller each iteration.
    assert_eq!(
        land_horizontal_input(true),
        LandHorizontalInput::PrecisionTarget
    );
    assert_eq!(
        land_horizontal_input(false),
        LandHorizontalInput::VelocityCorrection
    );
}

/// The pilot's repositioning speed is half the waypoint acceleration.
///
/// Upstream's reasoning: half the acceleration as a velocity means the
/// aircraft stops from full repositioning speed in under a second. A pilot
/// nudging a descending aircraft sideways needs it to stop when they let go,
/// not coast on over whatever they were avoiding.
#[test]
fn the_reposition_speed_stops_within_a_second() {
    for accel in [1.0_f32, 2.5, 10.0] {
        let speed = max_pilot_reposition_speed_ms(accel);
        assert!(
            (speed - accel * 0.5).abs() < 1e-6,
            "speed {speed} for acceleration {accel}"
        );
        // The stopping time is speed / acceleration, which is half a second
        // for any acceleration — the property the halving buys.
        assert!(
            (speed / accel - 0.5).abs() < 1e-6,
            "stopping time should be half a second, got {}",
            speed / accel
        );
    }
}
