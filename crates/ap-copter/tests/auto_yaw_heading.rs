//! AutoYaw's heading command and its two time-stepped modes, against the
//! real firmware.
//!
//! The fixed slew and the angle-rate integration both take their timestep
//! from `millis()`, which a harness cannot dictate. The recording seeds
//! `_last_update_ms` a known distance behind the clock and records the dt the
//! firmware actually used, so the port is compared against upstream's own
//! timestep rather than one the harness hoped for.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::auto_yaw::{
    angle_rate_step, default_yaw_mode, fixed_yaw_step, heading_mode, pilot_yaw_override,
    weathervane_action, HeadingMode, PilotYawOverride, WeathervaneAction, WpYawBehaviour, YawMode,
};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn mode(s: &str) -> YawMode {
    let n: u8 = s.trim().parse().expect("mode number");
    YawMode::from_number(n).unwrap_or_else(|| panic!("unknown recorded yaw mode {n}"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_auto_yaw2.csv"))
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

/// The heading command's kind, and the two mode transitions `get_heading`
/// makes on the way to producing it.
#[test]
fn the_heading_command_matches_upstream() {
    let rows = rows("heading_mode");
    assert_eq!(rows.len(), 11, "one row per yaw mode");

    let mut rate_only = 0_usize;
    let mut angle_and_rate = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let before = mode(&r[0]);
        let after = mode(&r[1]);
        let want_heading: i32 = r[2].trim().parse().expect("heading mode");
        let behaviour_num: u8 = r[3].trim().parse().expect("wp_yaw_behavior");
        let behaviour = WpYawBehaviour::from_number(behaviour_num);

        // The recording has never seen RC input, so the pilot branch takes
        // its else and the weathervane controller wants nothing.
        let expected_after = match pilot_yaw_override(before, false, true, 0.0) {
            PilotYawOverride::ReleaseToHold => YawMode::Hold,
            PilotYawOverride::TakeControl => YawMode::PilotRate,
            PilotYawOverride::None => {
                // The weathervane path can still release the axis, and the
                // harness leaves _last_mode at HOLD, which sends it to the
                // default rather than back.
                match weathervane_action(before, YawMode::Hold, false, false) {
                    WeathervaneAction::ReleaseToDefault => default_yaw_mode(behaviour, false),
                    WeathervaneAction::ReleaseTo(m) => m,
                    WeathervaneAction::Engage => YawMode::Weathervane,
                    WeathervaneAction::None => before,
                }
            }
        };

        assert_eq!(
            expected_after, after,
            "starting in {before:?}, the port ends in {expected_after:?} and \
             upstream in {after:?}"
        );

        // The heading mode is read from the mode the vehicle ended in.
        let got = heading_mode(after);
        let got_num = match got {
            HeadingMode::AngleOnly => 0,
            HeadingMode::AngleAndRate => 1,
            HeadingMode::RateOnly => 2,
        };
        assert_eq!(
            got_num, want_heading,
            "{after:?}: heading mode {got:?} against upstream {want_heading}"
        );

        match got {
            HeadingMode::RateOnly => rate_only += 1,
            HeadingMode::AngleAndRate => angle_and_rate += 1,
            HeadingMode::AngleOnly => {
                panic!("get_heading should never produce Angle_Only")
            }
        }
    }

    assert!(
        rate_only > 0 && angle_and_rate > 0,
        "one kind never appears"
    );
    println!("11 modes: {rate_only} rate-only, {angle_and_rate} angle-and-rate");
}

/// The fixed-yaw slew, against upstream's own timestep.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "counts rows where the value moved at all, not where it moved by \
some amount; an epsilon here would silently stop counting the smallest real \
steps, which are the ones worth knowing the sweep reached"
)]
fn the_fixed_yaw_slew_matches_upstream() {
    let rows = rows("fixed_slew");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut moved = 0_usize;
    let mut clamped = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 7, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let angle_in = f(&r[1]);
        let offset_in = f(&r[2]);
        let slew = f(&r[3]);
        let dt_ms: u32 = r[4].trim().parse().expect("dt");
        #[allow(
            clippy::cast_precision_loss,
            reason = "reproduces upstream's (now_ms - last_ms) * 0.001, which \
is the same u32-to-float conversion"
        )]
        let dt_s = dt_ms as f32 * 0.001;

        let (angle_out, offset_out) = fixed_yaw_step(angle_in, offset_in, slew, dt_s);

        let want_angle = f(&r[5]);
        let want_offset = f(&r[6]);

        assert!(
            (angle_out - want_angle).abs() < 1e-6,
            "row {idx}: angle {angle_out} against upstream {want_angle}"
        );
        assert!(
            (offset_out - want_offset).abs() < 1e-6,
            "row {idx}: remaining offset {offset_out} against upstream \
             {want_offset}"
        );

        if angle_out != angle_in {
            moved += 1;
        }
        if offset_out != 0.0 && offset_in != 0.0 {
            clamped += 1;
        }
    }

    assert!(moved > 0, "no row actually slewed");
    assert!(
        clamped > 0,
        "no row was rate-limited, so the constrain is untested"
    );
    println!(
        "{} slew rows, {moved} moved, {clamped} rate-limited",
        rows.len()
    );
}

/// The angle-rate integration, against upstream's own timestep.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "counts rows where the value moved at all, not where it moved by \
some amount; an epsilon here would silently stop counting the smallest real \
steps, which are the ones worth knowing the sweep reached"
)]
fn the_angle_rate_integration_matches_upstream() {
    let rows = rows("angle_rate");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut moved = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let angle_in = f(&r[1]);
        let rate = f(&r[2]);
        let dt_ms: u32 = r[3].trim().parse().expect("dt");
        #[allow(
            clippy::cast_precision_loss,
            reason = "reproduces upstream's (now_ms - last_ms) * 0.001"
        )]
        let dt_s = dt_ms as f32 * 0.001;

        let got = angle_rate_step(angle_in, rate, dt_s);
        let want = f(&r[4]);

        assert!(
            (got - want).abs() < 1e-6,
            "row {idx}: {got} against upstream {want}"
        );
        if got != angle_in {
            moved += 1;
        }
    }
    assert!(moved > 0, "no row actually integrated");
    println!("{} angle-rate rows, {moved} moved", rows.len());
}

/// The slew consumes its offset rather than tracking a target.
///
/// A fixed-yaw command arrives as an offset to fly through. Each iteration
/// takes as much as the rate allows and subtracts it, so the target walks to
/// the commanded heading and stops when nothing is left — no separate
/// "finished" flag, and an interrupted slew resumes from where it got to.
#[test]
fn the_slew_consumes_its_offset() {
    let mut angle = 0.0_f32;
    let mut offset = 1.0_f32;
    let slew = 2.0;
    let dt = 0.1;

    for _ in 0..20 {
        let (a, o) = fixed_yaw_step(angle, offset, slew, dt);
        angle = a;
        offset = o;
    }

    assert!(
        offset.abs() < 1e-6,
        "the offset should have been consumed, {offset} remains"
    );
    assert!(
        (angle - 1.0).abs() < 1e-6,
        "the angle should have absorbed the whole offset, got {angle}"
    );

    // And a negative offset slews the other way at the same rate.
    let (a, o) = fixed_yaw_step(0.0, -1.0, 2.0, 0.1);
    assert!((a + 0.2).abs() < 1e-6, "stepped {a}, expected -0.2");
    assert!((o + 0.8).abs() < 1e-6, "remaining {o}, expected -0.8");
}

/// Losing the yaw axis goes to HOLD, not back to what was running before.
///
/// When the radio fails or the mode stops accepting pilot yaw, upstream sets
/// HOLD rather than restoring the previous mode. That is worth not
/// "improving": the previous mode may be pointing at something the aircraft
/// can no longer see, and holding zero rate is the only answer that is safe
/// without knowing why the pilot's input went away.
#[test]
fn losing_the_yaw_axis_holds_rather_than_restoring() {
    assert_eq!(
        pilot_yaw_override(YawMode::PilotRate, false, true, 0.0),
        PilotYawOverride::ReleaseToHold,
        "an RC failsafe should release the axis"
    );
    assert_eq!(
        pilot_yaw_override(YawMode::PilotRate, true, false, 0.5),
        PilotYawOverride::ReleaseToHold,
        "a mode that stops accepting pilot yaw should release it too"
    );

    // Only from PILOT_RATE: a mode the pilot never took is left alone.
    for m in [YawMode::Roi, YawMode::Circle, YawMode::LookAtNextWp] {
        assert_eq!(
            pilot_yaw_override(m, false, true, 0.0),
            PilotYawOverride::None
        );
    }

    // Any non-zero stick takes control; the deadzone was applied upstream of
    // here, so a rate that arrives at all is one the pilot meant.
    assert_eq!(
        pilot_yaw_override(YawMode::Roi, true, true, 1e-3),
        PilotYawOverride::TakeControl
    );
    assert_eq!(
        pilot_yaw_override(YawMode::Roi, true, true, 0.0),
        PilotYawOverride::None
    );
}

/// Weathervaning hands the axis back to the default when what it took it from
/// was HOLD.
///
/// The asymmetry looks arbitrary and is not. HOLD is what the pilot-override
/// path leaves behind when it takes the axis away, so a recorded HOLD usually
/// means "nothing chose this" — restoring it would strand the aircraft
/// holding zero rate for the rest of the mission. Consulting WP_YAW_BEHAVIOR
/// gives the operator's standing preference instead.
#[test]
fn weathervaning_hands_back_to_the_default_from_hold() {
    assert_eq!(
        weathervane_action(YawMode::Weathervane, YawMode::Hold, false, false),
        WeathervaneAction::ReleaseToDefault
    );
    assert_eq!(
        weathervane_action(YawMode::Weathervane, YawMode::Roi, false, false),
        WeathervaneAction::ReleaseTo(YawMode::Roi),
        "a mode something actually chose is restored"
    );

    // Engaging needs both permission and a controller that wants the axis.
    assert_eq!(
        weathervane_action(YawMode::Hold, YawMode::Hold, true, true),
        WeathervaneAction::Engage
    );
    assert_eq!(
        weathervane_action(YawMode::Hold, YawMode::Hold, true, false),
        WeathervaneAction::None
    );
    assert_eq!(
        weathervane_action(YawMode::Hold, YawMode::Hold, false, true),
        WeathervaneAction::None
    );

    // And a mode that was never weathervaning is not released.
    assert_eq!(
        weathervane_action(YawMode::Circle, YawMode::Hold, false, false),
        WeathervaneAction::None
    );
}
