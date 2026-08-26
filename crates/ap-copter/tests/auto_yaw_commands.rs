//! AutoYaw's fixed-yaw command, rate command, arrival test and look-ahead,
//! against the real firmware.

#![allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
// float_cmp: the comparisons here are exact by intent. A relative command
// returns its argument unchanged, and the held look-ahead heading is the
// previous value verbatim -- an epsilon would let a port that perturbed
// either of them pass.

use ap_copter::auto_yaw::{
    fixed_yaw_offset_rad, fixed_yaw_slew_rate_rads, look_ahead_yaw_rad, reached_fixed_yaw_target,
    roi_action, set_rate, FixedYawDirection, RoiAction, YawMode,
};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn mode(s: &str) -> YawMode {
    let n: u8 = s.trim().parse().expect("mode number");
    YawMode::from_number(n).unwrap_or_else(|| panic!("unknown recorded yaw mode {n}"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_auto_yaw3.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.starts_with(|c: char| c.is_alphabetic()) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// The fixed-yaw command: what offset it produces and at what slew rate.
#[test]
fn the_fixed_yaw_command_matches_upstream() {
    let rows = rows("fixed_yaw");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut long_way = 0_usize;
    let mut capped = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 11, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let angle = f(&r[1]);
        let current = f(&r[2]);
        let dir_raw: i8 = r[3].trim().parse().expect("direction");
        let relative = b(&r[4]);
        let slew_req = f(&r[6]);
        let slew_max = f(&r[7]);

        let direction = FixedYawDirection::from_sign(dir_raw);
        let got_offset = fixed_yaw_offset_rad(angle, current, direction, relative);
        let got_slew = fixed_yaw_slew_rate_rads(slew_req, slew_max);

        let want_offset = f(&r[8]);
        let want_slew = f(&r[9]);

        assert!(
            (got_offset - want_offset).abs() < 1e-5,
            "row {idx}: offset {got_offset} against upstream {want_offset} \
             (angle {angle}, current {current}, {direction:?}, relative {relative})"
        );
        assert!(
            (got_slew - want_slew).abs() < 1e-6,
            "row {idx}: slew {got_slew} against upstream {want_slew}"
        );

        // An absolute command going more than half a turn is the direction
        // override doing its job.
        if !relative && want_offset.abs() > core::f32::consts::PI {
            long_way += 1;
        }
        if slew_req > 0.0 && want_slew < slew_req {
            capped += 1;
        }
    }

    assert!(
        long_way > 0,
        "no row was forced the long way round, so the direction override is \
         untested"
    );
    assert!(
        capped > 0,
        "no requested slew rate was capped by the controller's maximum"
    );
    println!(
        "{} fixed-yaw rows, {long_way} forced the long way, {capped} capped",
        rows.len()
    );
}

/// A direction argument forces the turn the long way round.
///
/// This is what lets `CONDITION_YAW` command three quarters of a turn one way
/// rather than a quarter the other — which matters when a camera is tracking
/// something on the way round, and is the whole reason the argument exists.
#[test]
fn a_direction_forces_the_long_way_round() {
    // Target a quarter turn clockwise from here. The short way is clockwise.
    let quarter = core::f32::consts::FRAC_PI_2;

    let shortest = fixed_yaw_offset_rad(quarter, 0.0, FixedYawDirection::Shortest, false);
    assert!(
        (shortest - quarter).abs() < 1e-6,
        "the short way should be a quarter turn clockwise, got {shortest}"
    );

    let clockwise = fixed_yaw_offset_rad(quarter, 0.0, FixedYawDirection::Clockwise, false);
    assert!(
        (clockwise - quarter).abs() < 1e-6,
        "asking for the way it was already going should change nothing"
    );

    let counter = fixed_yaw_offset_rad(quarter, 0.0, FixedYawDirection::CounterClockwise, false);
    assert!(
        (counter - (quarter - core::f32::consts::TAU)).abs() < 1e-6,
        "counter-clockwise should take the long way, got {counter}"
    );
    assert!(
        counter.abs() > core::f32::consts::PI,
        "the long way should be more than half a turn"
    );
}

/// A relative command is already an offset, so "shortest" and "clockwise"
/// mean the same thing.
///
/// There is nothing to be shortest about once the caller has said how far to
/// turn. Upstream writes `direction >= 0 ? 1.0 : -1.0`, so zero takes the
/// positive branch.
#[test]
fn a_relative_command_has_no_short_way() {
    let angle = 2.0_f32;
    assert_eq!(
        fixed_yaw_offset_rad(angle, 0.7, FixedYawDirection::Shortest, true),
        angle
    );
    assert_eq!(
        fixed_yaw_offset_rad(angle, 0.7, FixedYawDirection::Clockwise, true),
        angle
    );
    assert_eq!(
        fixed_yaw_offset_rad(angle, 0.7, FixedYawDirection::CounterClockwise, true),
        -angle
    );

    // And the current heading does not enter into it at all.
    for current in [-3.0_f32, 0.0, 1.5, 3.0] {
        assert_eq!(
            fixed_yaw_offset_rad(angle, current, FixedYawDirection::Shortest, true),
            angle,
            "a relative command should ignore the current heading"
        );
    }
}

/// `set_rate_rad`'s ordering: the mode change zeroes the rate, so it has to
/// happen first.
#[test]
fn the_rate_command_survives_the_mode_change() {
    let rows = rows("set_rate");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut from_other = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let from = mode(&r[0]);
        let requested = f(&r[1]);
        let want_rate = f(&r[2]);
        let want_mode = mode(&r[3]);

        let mut stored = -9.5_f32;
        let got_mode = set_rate(from, requested, &mut stored);

        assert_eq!(got_mode, want_mode, "from {from:?}: mode after");
        assert!(
            (stored - want_rate).abs() < 1e-6,
            "from {from:?}: rate {stored} against upstream {want_rate} — the \
             commanded rate did not survive the mode change"
        );
        assert!(
            (stored - requested).abs() < 1e-6,
            "from {from:?}: the requested rate must survive"
        );

        if from != YawMode::Rate {
            from_other += 1;
        }
    }

    assert!(
        from_other > 0,
        "every row was already in RATE, so the zeroing path is untested"
    );
    println!(
        "{} set-rate rows, {from_other} a real transition",
        rows.len()
    );
}

/// The arrival test.
#[test]
fn the_fixed_yaw_arrival_matches_upstream() {
    let rows = rows("reached");
    assert!(!rows.is_empty(), "no recorded rows");

    let (mut wrong_mode, mut still_slewing, mut arrived, mut short) = (0, 0, 0, 0);
    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let m = mode(&r[0]);
        let offset = f(&r[1]);
        let angle = f(&r[2]);
        let measured = f(&r[3]);
        let want = b(&r[4]);

        let got = reached_fixed_yaw_target(m, offset, angle, measured);
        assert_eq!(
            got, want,
            "{m:?} offset {offset} angle {angle}: {got} against upstream {want}"
        );

        if m != YawMode::Fixed {
            wrong_mode += 1;
        } else if offset != 0.0 {
            still_slewing += 1;
        } else if want {
            arrived += 1;
        } else {
            short += 1;
        }
    }

    assert!(
        wrong_mode > 0 && still_slewing > 0 && arrived > 0 && short > 0,
        "not every branch is reached: wrong mode {wrong_mode}, slewing \
         {still_slewing}, arrived {arrived}, short {short}"
    );
    println!(
        "{} arrival rows: {wrong_mode} wrong mode, {still_slewing} slewing, \
         {arrived} arrived, {short} short of target",
        rows.len()
    );
}

/// Being in the wrong mode reports arrival, and that is the safe direction.
///
/// Upstream returns true with a comment saying it should not happen. A caller
/// waiting on this is usually a mission command waiting to advance, and
/// blocking forever on a mode that will never arrive would stall the mission.
#[test]
fn the_wrong_mode_reports_arrival_rather_than_blocking() {
    for m in [
        YawMode::Hold,
        YawMode::Roi,
        YawMode::LookAhead,
        YawMode::Rate,
        YawMode::Circle,
    ] {
        assert!(
            reached_fixed_yaw_target(m, 5.0, 0.0, 3.0),
            "{m:?} should report arrival rather than block a mission"
        );
    }

    // In FIXED both conditions are required: the slew finished AND the
    // aircraft caught up. A slew can complete while the airframe is still
    // swinging.
    assert!(
        !reached_fixed_yaw_target(YawMode::Fixed, 0.5, 0.0, 0.0),
        "an unfinished slew has not arrived even if the aircraft is on heading"
    );
    assert!(
        !reached_fixed_yaw_target(YawMode::Fixed, 0.0, 1.0, 0.0),
        "a finished slew has not arrived while the aircraft is still swinging"
    );
    assert!(reached_fixed_yaw_target(YawMode::Fixed, 0.0, 0.0, 0.0));
}

/// The look-ahead heading, and what it does when there is nothing to look at.
#[test]
fn the_look_ahead_heading_matches_upstream() {
    let rows = rows("look_ahead");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut held = 0_usize;
    let mut updated = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 6, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let prev = f(&r[1]);
        let position_ok = b(&r[2]);
        let vel_n = f(&r[3]);
        let vel_e = f(&r[4]);
        let want = f(&r[5]);

        let got = look_ahead_yaw_rad(prev, position_ok, vel_n, vel_e);
        assert!(
            (got - want).abs() < 1e-6,
            "row {idx}: {got} against upstream {want} (vel {vel_n},{vel_e}, \
             position_ok {position_ok})"
        );

        if want == prev {
            held += 1;
        } else {
            updated += 1;
        }
    }

    assert!(
        held > 0 && updated > 0,
        "the threshold never bites: {held} held, {updated} updated"
    );
    println!(
        "{} look-ahead rows: {held} held, {updated} updated",
        rows.len()
    );
}

/// Below the speed threshold the last heading is held, not zeroed.
///
/// A hovering aircraft drifting a few centimetres a second has a well-defined
/// velocity direction only in the arithmetic sense. Zeroing would swing the
/// nose to north; holding leaves it where the last real motion put it, so a
/// brief slowdown does not produce a spurious turn.
#[test]
fn a_slow_aircraft_holds_its_last_heading() {
    let held = 1.25_f32;

    // Just under a metre per second in total.
    assert_eq!(look_ahead_yaw_rad(held, true, 0.5, 0.5), held);
    // And exactly at the threshold: the test is strictly greater than.
    assert_eq!(look_ahead_yaw_rad(held, true, 1.0, 0.0), held);
    // Just over it updates.
    assert_ne!(look_ahead_yaw_rad(held, true, 1.001, 0.0), held);

    // No position estimate holds too, however fast the aircraft is going.
    assert_eq!(look_ahead_yaw_rad(held, false, 20.0, 20.0), held);
}

/// The arrival test measures against the *measured* heading, not away from it.
///
/// Every recorded row has a measured heading of zero, where subtracting and
/// adding it read identically — so this is the case the recording cannot
/// distinguish, and the mutation gate found it.
#[test]
fn arrival_is_measured_against_the_aircraft_not_away_from_it() {
    // Target and aircraft both at half a radian: arrived.
    assert!(
        reached_fixed_yaw_target(YawMode::Fixed, 0.0, 0.5, 0.5),
        "a target the aircraft is already holding should read as arrived"
    );

    // Adding rather than subtracting would give an error of one radian here
    // and call it arrived, which is the mutation being excluded.
    assert!(
        !reached_fixed_yaw_target(YawMode::Fixed, 0.0, 0.5, -0.5),
        "a target half a radian the other side is not arrived"
    );

    // Either side of the two-degree window, with a non-zero measurement so
    // the sign is doing work.
    let two_deg = 2.0_f32.to_radians();
    assert!(reached_fixed_yaw_target(
        YawMode::Fixed,
        0.0,
        1.0 + two_deg * 0.9,
        1.0
    ));
    assert!(!reached_fixed_yaw_target(
        YawMode::Fixed,
        0.0,
        1.0 + two_deg * 1.1,
        1.0
    ));

    // And the comparison wraps: just either side of the discontinuity is a
    // small error, not a full turn.
    let pi = core::f32::consts::PI;
    assert!(reached_fixed_yaw_target(
        YawMode::Fixed,
        0.0,
        pi - 0.005,
        -pi + 0.005
    ));
}

/// A region-of-interest command, and what an empty location means.
///
/// A `Location` of all zeros is a real point in the Gulf of Guinea, so
/// upstream tests `initialised()` rather than the coordinates. A mission
/// clearing its region of interest sends zeros, and reading that literally
/// would swing the aircraft to face a point thousands of kilometres away.
#[test]
fn an_empty_roi_cancels_rather_than_pointing_at_the_equator() {
    assert_eq!(roi_action(false, false), RoiAction::Cancel);
    assert_eq!(
        roi_action(false, true),
        RoiAction::Cancel,
        "an empty location cancels whatever the mount can do"
    );
}

/// A panning mount tracks the target itself, so the airframe is left to fly.
///
/// Only a fixed mount makes the whole aircraft the pointing mechanism.
#[test]
fn a_panning_mount_spares_the_airframe() {
    assert_eq!(roi_action(true, true), RoiAction::MountOnly);
    assert_eq!(roi_action(true, false), RoiAction::PointAirframe);
}
