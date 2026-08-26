//! Where a Plane mode's target altitude comes from, against the firmware.
//!
//! The function returns nothing and every branch writes the same field, so a
//! branch is identified by which call it makes and with what. The landing
//! wrapper hands back a location with a distinctive altitude, which is what
//! separates the landing-target branch from the next-waypoint ones — counting
//! calls alone could not, and the first recording could not either.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::target_altitude::{target_altitude, TargetAltitude, TargetAltitudeInputs};

/// The altitude the harness puts on the next waypoint.
const WAYPOINT_ALT: i32 = 7700;
/// The altitude the harness's landing wrapper reports.
const LANDING_TARGET_ALT: i32 = 4242;

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn n(s: &str) -> i32 {
    s.trim().parse().expect("number")
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_target_altitude.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

/// What the firmware did, read from the calls it made.
fn observed(r: &[String]) -> TargetAltitude {
    let glide = n(&r[10]) > 0;
    let reset_offset = n(&r[11]) > 0;
    let set_location = n(&r[12]) > 0;
    let proportion = n(&r[13]) > 0;
    let constrain = n(&r[14]) > 0;
    let location_alt = n(&r[16]);

    match (glide, reset_offset, set_location, proportion, constrain) {
        (true, false, false, false, false) => TargetAltitude::LandingGlideSlope,
        (false, true, true, false, false) => TargetAltitude::HoldCurrentAndResetOffset,
        (false, false, true, false, false) => {
            if location_alt == LANDING_TARGET_ALT {
                TargetAltitude::FromLandingTarget
            } else {
                assert_eq!(
                    location_alt, WAYPOINT_ALT,
                    "a location was set from neither the waypoint nor the \
                     landing target"
                );
                TargetAltitude::FromNextWaypoint
            }
        }
        (false, false, false, true, true) => TargetAltitude::ProportionalToNextWaypoint,
        (false, false, false, false, false) => TargetAltitude::TerrainProportion,
        other => panic!("unrecognised call pattern {other:?} in row {}", r[0]),
    }
}

#[test]
fn the_target_altitude_source_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut seen = std::collections::BTreeMap::new();
    let mut terrain_attempts = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 17, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let inputs = TargetAltitudeInputs {
            landing_is_flaring: b(&r[1]),
            landing_is_on_approach: b(&r[2]),
            landing_has_target_location: b(&r[3]),
            // Never active in the recording; see the test below.
            soaring_gliding: b(&r[4]),
            reached_loiter_target: b(&r[5]),
            next_wp_is_terrain_alt: b(&r[6]),
            offset_cm: n(&r[8]),
            past_interval_finish_line: b(&r[9]),
        };
        let terrain_ok = b(&r[7]);

        // The closure must run exactly when the firmware called it, which the
        // recorded call count says.
        let mut attempts = 0_usize;
        let got = target_altitude(&inputs, || {
            attempts += 1;
            terrain_ok
        });

        let recorded_attempts: usize = n(&r[15]).try_into().expect("terrain calls");
        assert_eq!(
            attempts, recorded_attempts,
            "row {idx}: the port tried the terrain proportion {attempts} times, \
             upstream {recorded_attempts} — the short circuit differs"
        );
        terrain_attempts += attempts;

        let want = observed(r);
        assert_eq!(
            got, want,
            "row {idx}: {got:?} against upstream {want:?}, inputs {inputs:?}"
        );

        *seen.entry(format!("{want:?}")).or_insert(0_usize) += 1;
    }

    assert!(
        terrain_attempts > 0,
        "the terrain branch is never attempted, so its short circuit is untested"
    );
    assert!(
        seen.len() >= 4,
        "only {} outcomes reached: {seen:?}",
        seen.len()
    );
    println!("{} rows, outcomes {seen:?}", rows.len());
}

/// The terrain attempt runs only when every branch above it has declined.
///
/// Upstream's condition is
/// `next_WP_loc.terrain_alt && set_target_altitude_proportion_terrain()`, and
/// the right-hand side *sets the target altitude* as well as reporting
/// whether it could. So calling it early does not merely waste time, it
/// writes a target the aircraft was not supposed to have. The port takes a
/// closure rather than a bool so the short circuit is in the signature and
/// cannot be got wrong by a caller.
#[test]
fn the_terrain_attempt_has_a_side_effect_and_is_not_run_early() {
    let base = TargetAltitudeInputs {
        landing_is_flaring: false,
        landing_is_on_approach: false,
        landing_has_target_location: false,
        soaring_gliding: false,
        reached_loiter_target: false,
        next_wp_is_terrain_alt: true,
        offset_cm: 500,
        past_interval_finish_line: false,
    };

    // Every branch above the terrain one, in turn: none of them may attempt it.
    let higher = [
        TargetAltitudeInputs {
            landing_is_flaring: true,
            ..base
        },
        TargetAltitudeInputs {
            landing_is_on_approach: true,
            ..base
        },
        TargetAltitudeInputs {
            landing_has_target_location: true,
            ..base
        },
        TargetAltitudeInputs {
            soaring_gliding: true,
            ..base
        },
        TargetAltitudeInputs {
            reached_loiter_target: true,
            ..base
        },
    ];
    for inputs in higher {
        let mut attempts = 0;
        let _ = target_altitude(&inputs, || {
            attempts += 1;
            true
        });
        assert_eq!(
            attempts, 0,
            "the terrain proportion was attempted despite a higher branch \
             winning: {inputs:?}"
        );
    }

    // And a waypoint that is not terrain-relative does not attempt it either,
    // because the flag is tested first.
    let mut attempts = 0;
    let _ = target_altitude(
        &TargetAltitudeInputs {
            next_wp_is_terrain_alt: false,
            ..base
        },
        || {
            attempts += 1;
            true
        },
    );
    assert_eq!(attempts, 0, "attempted the terrain proportion off-terrain");

    // When it is reached and declines, the ladder continues below it.
    let mut attempts = 0;
    let declined = target_altitude(&base, || {
        attempts += 1;
        false
    });
    assert_eq!(attempts, 1);
    assert_eq!(declined, TargetAltitude::ProportionalToNextWaypoint);
}

/// The proportional climb stops at the finish line.
///
/// Past it, holding a proportional target would keep commanding a climb the
/// aircraft has already completed — so the leg's altitude offset is abandoned
/// in favour of the waypoint's own altitude.
#[test]
fn the_proportional_climb_ends_at_the_finish_line() {
    let base = TargetAltitudeInputs {
        landing_is_flaring: false,
        landing_is_on_approach: false,
        landing_has_target_location: false,
        soaring_gliding: false,
        reached_loiter_target: false,
        next_wp_is_terrain_alt: false,
        offset_cm: 2500,
        past_interval_finish_line: false,
    };

    assert_eq!(
        target_altitude(&base, || false),
        TargetAltitude::ProportionalToNextWaypoint
    );
    assert_eq!(
        target_altitude(
            &TargetAltitudeInputs {
                past_interval_finish_line: true,
                ..base
            },
            || false
        ),
        TargetAltitude::FromNextWaypoint
    );

    // And with no offset there is no climb to spread, whichever side of the
    // line the aircraft is on.
    for past in [false, true] {
        assert_eq!(
            target_altitude(
                &TargetAltitudeInputs {
                    offset_cm: 0,
                    past_interval_finish_line: past,
                    ..base
                },
                || false
            ),
            TargetAltitude::FromNextWaypoint
        );
    }
}

/// The soaring branch, which the recording cannot reach.
///
/// `soaring_controller`'s predicates are inlined, so they are not among
/// `mode.cpp.o`'s undefined symbols and there is nothing for `--wrap` to
/// redirect; standing up a real soaring controller is a different slice.
/// Every recorded row therefore has soaring inactive, and this pins the
/// branch instead.
///
/// It requires *both* an active controller and a suppressed throttle: a
/// soaring controller that is still using the motor has not started gliding,
/// and holding the target at the current altitude then would fight the climb.
#[test]
fn soaring_holds_the_current_altitude_but_only_while_gliding() {
    let base = TargetAltitudeInputs {
        landing_is_flaring: false,
        landing_is_on_approach: false,
        landing_has_target_location: false,
        soaring_gliding: true,
        reached_loiter_target: true,
        next_wp_is_terrain_alt: false,
        offset_cm: 0,
        past_interval_finish_line: false,
    };

    assert_eq!(
        target_altitude(&base, || false),
        TargetAltitude::HoldCurrentAndResetOffset,
        "soaring should outrank a reached loiter target"
    );
    assert_eq!(
        target_altitude(
            &TargetAltitudeInputs {
                soaring_gliding: false,
                ..base
            },
            || false
        ),
        TargetAltitude::FromNextWaypoint
    );

    // But the landing branches outrank soaring: an aircraft on approach is
    // not gliding for lift.
    for landing in [
        TargetAltitudeInputs {
            landing_is_flaring: true,
            ..base
        },
        TargetAltitudeInputs {
            landing_is_on_approach: true,
            ..base
        },
        TargetAltitudeInputs {
            landing_has_target_location: true,
            ..base
        },
    ] {
        assert_ne!(
            target_altitude(&landing, || false),
            TargetAltitude::HoldCurrentAndResetOffset,
            "soaring outranked a landing branch: {landing:?}"
        );
    }
}
