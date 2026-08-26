//! AutoYaw's absolute and relative yaw commands, and the ROI bearing.

#![allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
// float_cmp: exactness is the assertion throughout this file. An absolute
// command is taken verbatim, an offset command zeroes the rate, and losing
// the position estimate returns the standing target unchanged -- an epsilon
// would let a port that perturbed any of those pass.

use ap_copter::auto_yaw::{roi_yaw_rad, set_yaw_angle_and_rate, set_yaw_angle_offset, YawMode};

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
        .map(|p| p.join("fixtures/copter_auto_yaw4.csv"))
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

#[test]
fn the_absolute_yaw_command_matches_upstream() {
    let rows = rows("angle_and_rate");
    assert!(!rows.is_empty(), "no recorded rows");

    for r in &rows {
        assert_eq!(r.len(), 6, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let got = set_yaw_angle_and_rate(f(&r[1]), f(&r[2]));

        assert!(
            (got.yaw_angle_rad - f(&r[3])).abs() < 1e-6,
            "row {idx}: angle {} against upstream {}",
            got.yaw_angle_rad,
            f(&r[3])
        );
        assert!(
            (got.yaw_rate_rads - f(&r[4])).abs() < 1e-6,
            "row {idx}: rate {} against upstream {}",
            got.yaw_rate_rads,
            f(&r[4])
        );
        assert_eq!(got.mode, mode(&r[5]), "row {idx}: mode");
    }
    println!("{} absolute-command rows", rows.len());
}

/// Both values are taken exactly as given, including angles outside a turn.
///
/// This is the `SET_POSITION_TARGET` path: a companion computer has said
/// precisely what it wants and there is nothing to derive, so nothing is
/// wrapped or clamped on the way in.
#[test]
fn an_absolute_command_is_taken_verbatim() {
    for angle in [-7.0_f32, -1.0, 0.0, 1.0, 7.0] {
        for rate in [-2.0_f32, 0.0, 2.0] {
            let got = set_yaw_angle_and_rate(angle, rate);
            assert_eq!(got.yaw_angle_rad, angle, "the angle should not be wrapped");
            assert_eq!(got.yaw_rate_rads, rate);
            assert_eq!(got.mode, YawMode::AngleRate);
        }
    }
}

#[test]
fn the_relative_yaw_command_matches_upstream() {
    let rows = rows("angle_offset");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut wrapped = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 6, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let current = f(&r[1]);
        let offset_deg = f(&r[2]);
        let got = set_yaw_angle_offset(current, offset_deg);

        let want_angle = f(&r[3]);
        assert!(
            (got.yaw_angle_rad - want_angle).abs() < 1e-5,
            "row {idx}: angle {} against upstream {want_angle} (current \
             {current}, offset {offset_deg} deg)",
            got.yaw_angle_rad
        );
        assert!(
            (got.yaw_rate_rads - f(&r[4])).abs() < 1e-6,
            "row {idx}: rate should be zeroed"
        );
        assert_eq!(got.mode, mode(&r[5]), "row {idx}: mode");

        // The result must always land inside one turn.
        assert!(
            (0.0..core::f32::consts::TAU).contains(&want_angle),
            "row {idx}: upstream returned {want_angle}, outside 0..2pi"
        );
        if current + offset_deg.to_radians() != want_angle {
            wrapped += 1;
        }
    }

    assert!(wrapped > 0, "no row actually wrapped");
    println!("{} relative-command rows, {wrapped} wrapped", rows.len());
}

/// A relative command wraps to a full turn, where the fixed-yaw path wraps to
/// a half.
///
/// The two produce the same physical heading with different numbers, so
/// anything comparing a target against a −π..π measurement has to account for
/// it. Worth pinning because the two functions sit next to each other and
/// look interchangeable.
#[test]
fn a_relative_command_wraps_to_a_full_turn() {
    let tau = core::f32::consts::TAU;

    // Just past a full turn comes back near zero, not near 2π.
    let past = set_yaw_angle_offset(6.0, 60.0);
    assert!(
        past.yaw_angle_rad < 1.0,
        "should have wrapped past 2pi, got {}",
        past.yaw_angle_rad
    );

    // A negative result is brought up into the positive range rather than
    // left below zero, which is where wrap_PI would leave it.
    let negative = set_yaw_angle_offset(0.1, -45.0);
    assert!(
        negative.yaw_angle_rad > core::f32::consts::PI,
        "a small negative angle should wrap up to near 2pi, got {}",
        negative.yaw_angle_rad
    );

    // Every result is inside one turn, whatever is asked for.
    for current in [-10.0_f32, 0.0, 3.0, 10.0] {
        for offset in [-720.0_f32, -45.0, 0.0, 45.0, 720.0] {
            let got = set_yaw_angle_offset(current, offset);
            assert!(
                (0.0..tau).contains(&got.yaw_angle_rad),
                "current {current} offset {offset} gave {}",
                got.yaw_angle_rad
            );
        }
    }
}

/// An offset command zeroes the rate.
///
/// It says where to end up, not how fast. Leaving a previously commanded rate
/// would have the aircraft sail past the new target, because `ANGLE_RATE`
/// integrates the rate into the angle every iteration.
#[test]
fn an_offset_command_zeroes_the_rate() {
    let got = set_yaw_angle_offset(1.0, 90.0);
    assert_eq!(got.yaw_rate_rads, 0.0);
    assert_eq!(got.mode, YawMode::AngleRate);
}

#[test]
fn the_roi_bearing_matches_upstream() {
    let rows = rows("roi");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut held = 0_usize;
    let mut computed = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 8, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let position = b(&r[1]).then(|| (f(&r[2]), f(&r[3])));
        let roi = (f(&r[4]), f(&r[5]));
        let attitude_target = f(&r[6]);

        let got = roi_yaw_rad(position, roi, attitude_target);
        let want = f(&r[7]);

        assert!(
            (got - want).abs() < 1e-5,
            "row {idx}: {got} against upstream {want} (position {position:?}, \
             roi {roi:?})"
        );

        if position.is_none() {
            held += 1;
            assert_eq!(
                want, attitude_target,
                "row {idx}: with no position upstream should hold the \
                 standing target"
            );
        } else {
            computed += 1;
        }
    }

    assert!(held > 0 && computed > 0, "one branch is never taken");
    println!("{} ROI rows: {held} held, {computed} computed", rows.len());
}

/// Losing the position estimate holds the standing target rather than
/// pointing north.
///
/// Returning zero would swing the aircraft to north; returning the measured
/// heading would let the target drift with every gust the airframe took.
/// Returning the attitude controller's current target leaves the demand
/// exactly where it was, so a momentary loss of position produces no yaw at
/// all.
#[test]
fn losing_position_holds_the_standing_target() {
    let standing = 1.234_f32;
    assert_eq!(roi_yaw_rad(None, (100.0, 100.0), standing), standing);

    // With a position, the bearing is computed and is not the standing
    // target.
    let computed = roi_yaw_rad(Some((0.0, 0.0)), (100.0, 0.0), standing);
    assert!(
        (computed - 0.0).abs() < 1e-6,
        "due north should be zero, got {computed}"
    );
    let east = roi_yaw_rad(Some((0.0, 0.0)), (0.0, 100.0), standing);
    assert!(
        (east - core::f32::consts::FRAC_PI_2).abs() < 1e-6,
        "due east should be a quarter turn, got {east}"
    );

    // A target at the vehicle's own position is degenerate; upstream's
    // atan2(0, 0) is zero rather than an error.
    assert_eq!(roi_yaw_rad(Some((5.0, 5.0)), (5.0, 5.0), standing), 0.0);
}
