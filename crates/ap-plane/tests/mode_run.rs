//! What a Plane mode does each iteration, against the real firmware.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::mode_run::{
    applies_fbw_stick_mixing, pilot_throttle_source, pre_arm_checks, PilotThrottleSource,
    PreArmResult, SteerReset, StickMixing, GENERIC_REFUSAL,
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

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_mode_run.csv"))
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

/// The stick-mixing dispatch, over every value the parameter can hold.
///
/// Including values outside the enum. Upstream's switch has no default case,
/// so what an out-of-range value does is a fact about the code rather than an
/// impossibility, and the recording sweeps past both ends.
#[test]
fn the_stick_mixing_dispatch_matches_upstream() {
    let rows = rows("stick_mixing");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut mixed = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let raw: i32 = r[0].trim().parse().expect("stick mixing");

        // The harness casts through the parameter's own type, so a negative
        // value arrives as its unsigned bit pattern -- which is what the
        // firmware would see from a corrupted parameter too.
        #[allow(
            clippy::cast_sign_loss,
            clippy::cast_possible_truncation,
            reason = "reproduces the harness's cast through StickMixing, which \
is how an out-of-range stored value reaches the switch"
        )]
        let number = raw as u8;

        let setting = StickMixing::from_number(number);
        let got = applies_fbw_stick_mixing(setting);
        let want = b(&r[1]);
        assert_eq!(
            got, want,
            "STICK_MIXING {raw} (as {number}): mixing {got} against upstream \
             {want}, parsed as {setting:?}"
        );

        // Every row must have reached all three stabilise calls, or a "no
        // mixing" row would be indistinguishable from run() bailing out.
        assert!(
            b(&r[2]) && b(&r[3]) && b(&r[4]),
            "STICK_MIXING {raw}: run() did not reach the stabilise calls, so \
             the mixing column means nothing for this row"
        );

        if want {
            mixed += 1;
        }
    }

    assert!(
        mixed > 0 && mixed < rows.len(),
        "the mixing decision never varies across {} rows",
        rows.len()
    );
    println!("{} stick-mixing rows, {mixed} apply mixing", rows.len());
}

/// A removed option still mixes, deliberately.
///
/// `DIRECT_REMOVED` was direct stick mixing, taken out of the firmware. It
/// maps to fly-by-wire mixing rather than to nothing, because an aircraft
/// configured for direct mixing would otherwise lose stick authority
/// entirely at the next update — a worse surprise than a different flavour of
/// mixing, and cheaper than a parameter conversion.
#[test]
fn the_removed_option_still_mixes() {
    assert!(applies_fbw_stick_mixing(Some(StickMixing::DirectRemoved)));

    // The two that do not mix mean something else rather than nothing:
    // no mixing at all, and VTOL yaw mixing, which is not this.
    assert!(!applies_fbw_stick_mixing(Some(StickMixing::None)));
    assert!(!applies_fbw_stick_mixing(Some(StickMixing::VtolYaw)));

    // And an out-of-range parameter matches no case, so nothing happens.
    assert!(!applies_fbw_stick_mixing(None));
    for number in 5..=u8::MAX {
        assert_eq!(
            StickMixing::from_number(number),
            None,
            "{number} should not be a stick-mixing setting"
        );
    }
}

/// Every named setting round-trips through its stored number.
#[test]
fn the_stick_mixing_numbers_round_trip() {
    for setting in [
        StickMixing::None,
        StickMixing::Fbw,
        StickMixing::DirectRemoved,
        StickMixing::VtolYaw,
        StickMixing::FbwNoPitch,
    ] {
        assert_eq!(
            StickMixing::from_number(setting.as_number()),
            Some(setting),
            "{setting:?} did not survive a round trip"
        );
    }
}

#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactness is the assertion: the reset writes a literal zero and \
the recording holds the firmware's own bits, so a value merely near zero \
would mean the field kept part of what it held"
)]
fn the_controller_reset_matches_upstream() {
    let rows = rows("reset");
    assert!(!rows.is_empty(), "no recorded rows");

    for r in &rows {
        assert_eq!(r.len(), 3, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        // The harness sets a locked course and a non-zero error before each
        // call, so anything the reset misses arrives still set.
        let mut state = SteerReset {
            locked_course: true,
            locked_course_err: 2.75,
        };
        state.reset();

        assert_eq!(
            state.locked_course,
            b(&r[1]),
            "row {idx}: locked_course after reset"
        );
        assert_eq!(
            state.locked_course_err,
            f(&r[2]),
            "row {idx}: locked_course_err after reset"
        );
    }
}

/// The pre-arm decisions that were recorded, and an honest note about the one
/// that was not.
#[test]
fn the_pre_arm_checks_match_upstream() {
    let rows = rows("pre_arm");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut refused = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let allowed = b(&r[2]);
        let message = r[3].trim();

        let mode_message = if message == "-" { "" } else { message };
        let got = pre_arm_checks(allowed, mode_message);

        if allowed {
            assert_eq!(got, PreArmResult::Allowed, "row {idx}");
            assert_eq!(message, "-", "row {idx}: allowed but carried a message");
        } else {
            refused += 1;
            assert_eq!(got, PreArmResult::Refused(mode_message), "row {idx}");
        }
    }

    // No recorded row refuses: the only refusal in `_pre_arm_checks` needs a
    // quadplane enabled with ONLY_ARM_IN_QMODE_OR_AUTO set, which this
    // firmware is not configured for. Stated rather than left to look like
    // coverage — the substitution below is what the recording cannot show.
    assert_eq!(
        refused, 0,
        "a pre-arm refusal was recorded after all — the reasoning in \
         `a_silent_refusal_still_explains_itself` can be replaced by it"
    );
    println!("{} pre-arm rows, all allowed", rows.len());
}

/// A mode that refuses without saying why still tells the pilot something.
///
/// The return value is false either way, so this substitution is invisible
/// except in the buffer. It matters because a refusal with no text reads to a
/// pilot as a broken ground station rather than as a decision the aircraft
/// made.
#[test]
fn a_silent_refusal_still_explains_itself() {
    assert_eq!(
        pre_arm_checks(false, ""),
        PreArmResult::Refused(GENERIC_REFUSAL)
    );

    // A mode that does explain itself is passed through unchanged rather than
    // having its message replaced by the generic one.
    assert_eq!(
        pre_arm_checks(false, "not Q mode"),
        PreArmResult::Refused("not Q mode")
    );

    // And an allowed mode carries no message at all.
    assert_eq!(pre_arm_checks(true, ""), PreArmResult::Allowed);
    assert_eq!(pre_arm_checks(true, "ignored"), PreArmResult::Allowed);
}

/// `THR_PASS_STAB` chooses between the raw stick and the trim-adjusted one.
#[test]
fn the_pilot_throttle_source_follows_the_passthrough_option() {
    assert_eq!(pilot_throttle_source(true), PilotThrottleSource::Direct);
    assert_eq!(
        pilot_throttle_source(false),
        PilotThrottleSource::TrimAdjusted
    );
}
