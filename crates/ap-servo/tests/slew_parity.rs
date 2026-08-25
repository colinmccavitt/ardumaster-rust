//! The SRV_Channels slew limiter against the real firmware.

#![allow(
    clippy::float_cmp,
    reason = "these comparisons are exact on purpose: an unlimited output must pass through bit-identically, and repeated peeks must return the same value rather than merely a close one. A tolerance here would hide the defect."
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_servo::function::Function;
use ap_servo::registry::Registry;

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn sections() -> std::collections::HashMap<String, Vec<Vec<String>>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_slew.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out: std::collections::HashMap<String, Vec<Vec<String>>> = Default::default();
    let mut current = String::new();
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag.to_owned();
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        out.entry(current.clone())
            .or_default()
            .push(line.split(',').map(str::to_owned).collect());
    }
    out
}

/// Three functions in three states: a real limit, a zero limit that still
/// keeps an entry, and no entry at all.
#[test]
fn the_slew_limiter_matches_upstream() {
    let s = sections();
    let funcs = &s.get("functions").expect("functions section")[0];
    assert_eq!(funcs.len(), 3, "malformed functions row");

    let thr = Function(funcs[0].trim().parse().expect("throttle function"));
    let flap = Function(funcs[1].trim().parse().expect("flap function"));
    let elev = Function(funcs[2].trim().parse().expect("elevator function"));

    let rows = s.get("slew").expect("slew section");
    let dt = 0.02_f32;

    let mut reg = Registry::new();
    // Flap takes a zero rate: an entry is still made, and it must keep
    // tracking so a rate installed later starts from the right place.
    assert!(reg.set_slew_rate(flap, 0.0, 100, dt), "flap entry");

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    let mut throttle_lagged = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 9, "malformed slew row");
        let step: usize = r[0].parse().expect("step");
        let rate = f(&r[1]);
        let demand = f(&r[2]);

        // Plane calls this every loop; so does the recording.
        assert!(reg.set_slew_rate(thr, rate, 100, dt), "throttle entry");

        reg.set_output_scaled(thr, demand);
        reg.set_output_scaled(flap, demand);
        reg.set_output_scaled(elev, demand);

        // Peek before applying: this must not advance the history.
        let peek_thr = reg.slew_limited_output_scaled(thr);
        let peek_flap = reg.slew_limited_output_scaled(flap);
        let peek_elev = reg.slew_limited_output_scaled(elev);

        reg.apply_slew_limits();

        for (label, got, want) in [
            ("peek_thr", peek_thr, f(&r[3])),
            ("after_thr", reg.output_scaled(thr), f(&r[4])),
            ("peek_flap", peek_flap, f(&r[5])),
            ("after_flap", reg.output_scaled(flap), f(&r[6])),
            ("peek_elev", peek_elev, f(&r[7])),
            ("after_elev", reg.output_scaled(elev), f(&r[8])),
        ] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-5,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }

        if (f(&r[4]) - demand).abs() > 1e-6 {
            throttle_lagged += 1;
        }
    }

    // A sequence where the limiter never binds would pass with the whole
    // clamp removed.
    assert!(
        throttle_lagged > 100,
        "the throttle only lagged its demand on {throttle_lagged} steps; the \
         limiter is barely engaging"
    );
    // And the other two must NOT be limited, or the test cannot tell a
    // disabled entry from an enabled one.
    assert!(
        rows.iter().all(|r| (f(&r[6]) - f(&r[2])).abs() < 1e-6),
        "the flap has a zero rate and must never be limited"
    );

    println!(
        "{} slew steps, {checked} values, largest difference {largest:e}, \
         throttle lagged on {throttle_lagged}",
        rows.len()
    );
}

/// Peeking repeatedly must not move anything.
///
/// `get_slew_limited_output_scaled` clamps against the history without
/// advancing it. A port that folded the peek and the step together would give
/// a different answer each call and drift the output on nothing but reads.
#[test]
fn peeking_does_not_advance_the_slew_history() {
    let s = sections();
    let funcs = &s.get("functions").expect("functions section")[0];
    let thr = Function(funcs[0].trim().parse().expect("throttle function"));
    let rows = s.get("peek").expect("peek section");
    assert!(rows.len() > 1, "need several peeks to show they agree");

    // The recording peeks after the sequence above, so the entry carries that
    // sequence's history and its final rate of zero. Replaying the sequence is
    // the only honest way to arrive at the same state.
    let (mut reg, _) = replay_slew_sequence();
    reg.set_output_scaled(thr, 500.0);

    let mut seen = Vec::new();
    for r in rows {
        assert_eq!(r.len(), 2, "malformed peek row");
        let got = reg.slew_limited_output_scaled(thr);
        let want = f(&r[1]);
        assert!(
            (got - want).abs() < 3e-5,
            "peek {}: {got} != upstream {want}",
            r[0]
        );
        seen.push(got);
    }

    assert!(
        seen.windows(2).all(|w| w[0] == w[1]),
        "the peeks disagreed with each other: {seen:?}"
    );
    println!("{} peeks, all {}", seen.len(), seen[0]);
}

/// A zero rate keeps its entry tracking, which is what makes enabling a limit
/// later safe.
///
/// Upstream says so in a comment, and it is the difference between installing
/// a slew rate mid-flight and having the first limited step be a jump — the
/// one thing the limiter exists to prevent.
#[test]
fn a_disabled_limit_still_tracks_the_output() {
    let thr = Function(70);
    let dt = 0.02_f32;

    let mut reg = Registry::new();
    reg.set_slew_rate(thr, 0.0, 100, dt);

    // Drive the output a long way with the limit switched off.
    for _ in 0..50 {
        reg.set_output_scaled(thr, 400.0);
        reg.apply_slew_limits();
    }
    assert_eq!(reg.output_scaled(thr), 400.0, "a zero rate must not limit");

    // Now enable it. The first limited step must move from 400, not from 0.
    reg.set_slew_rate(thr, 10.0, 100, dt);
    reg.set_output_scaled(thr, 0.0);
    reg.apply_slew_limits();

    let step = 100.0 * 10.0 * 0.01 * dt;
    assert!(
        (reg.output_scaled(thr) - (400.0 - step)).abs() < 1e-4,
        "the first limited step should leave {}, got {}",
        400.0 - step,
        reg.output_scaled(thr)
    );
}

/// The table is bounded where upstream's list is not, and running out behaves
/// like upstream's failed allocation rather than like a new failure.
#[test]
fn a_full_slew_table_leaves_the_function_unlimited() {
    use ap_servo::registry::MAX_SLEW_ENTRIES;

    let mut reg = Registry::new();
    for i in 0..MAX_SLEW_ENTRIES {
        let func = Function(u8::try_from(i + 1).expect("small index"));
        assert!(
            reg.set_slew_rate(func, 50.0, 100, 0.02),
            "entry {i} should fit"
        );
    }
    assert_eq!(reg.slew_entries(), MAX_SLEW_ENTRIES);

    // One too many: refused, and the function is simply not limited — which
    // is what upstream does when its allocation fails.
    let overflow = Function(u8::try_from(MAX_SLEW_ENTRIES + 1).expect("small index"));
    assert!(
        !reg.set_slew_rate(overflow, 50.0, 100, 0.02),
        "table is full"
    );

    reg.set_output_scaled(overflow, 900.0);
    reg.apply_slew_limits();
    assert_eq!(
        reg.output_scaled(overflow),
        900.0,
        "a function with no entry passes through unlimited"
    );
}

/// Replay the recorded slew sequence, returning the registry it leaves behind.
///
/// The fixture's later sections continue from this state rather than starting
/// clean, because the firmware's slew list is a process-lifetime static that
/// is only ever appended to.
fn replay_slew_sequence() -> (Registry, Function) {
    let s = sections();
    let funcs = &s.get("functions").expect("functions section")[0];
    let thr = Function(funcs[0].trim().parse().expect("throttle function"));
    let flap = Function(funcs[1].trim().parse().expect("flap function"));
    let elev = Function(funcs[2].trim().parse().expect("elevator function"));
    let dt = 0.02_f32;

    let mut reg = Registry::new();
    reg.set_slew_rate(flap, 0.0, 100, dt);

    for r in s.get("slew").expect("slew section") {
        reg.set_slew_rate(thr, f(&r[1]), 100, dt);
        let demand = f(&r[2]);
        reg.set_output_scaled(thr, demand);
        reg.set_output_scaled(flap, demand);
        reg.set_output_scaled(elev, demand);
        reg.apply_slew_limits();
    }
    (reg, thr)
}

/// A new entry starts its history at the output's current value.
///
/// Every other test here installs its limit on a fresh registry, where the
/// output is zero and "seed from the current value" and "seed from zero" are
/// indistinguishable — mutation testing found exactly that. This one installs
/// a limit on a function that is already somewhere.
///
/// It matters for the same reason the disabled-entry tracking does: a limit
/// installed mid-flight must slew from where the surface actually is. Seeding
/// from zero would make the first step a full-scale jerk toward zero, at
/// whatever rate was just configured.
#[test]
fn a_new_entry_starts_from_the_current_output() {
    let thr = Function(70);
    let dt = 0.02_f32;

    let mut reg = Registry::new();

    // No limit yet, so this lands wherever it is put.
    reg.set_output_scaled(thr, 250.0);
    reg.apply_slew_limits();
    assert_eq!(
        reg.output_scaled(thr),
        250.0,
        "unlimited output should pass through"
    );

    // Now install one. The history must start at 250, not at 0.
    assert!(reg.set_slew_rate(thr, 10.0, 100, dt));
    reg.set_output_scaled(thr, 250.0);
    reg.apply_slew_limits();
    assert_eq!(
        reg.output_scaled(thr),
        250.0,
        "holding still must not move the output; a history seeded at zero \
         would have clamped this to a fraction of a unit"
    );

    // And a step away from it moves by exactly one increment.
    let step = 100.0 * 10.0 * 0.01 * dt;
    reg.set_output_scaled(thr, 1000.0);
    reg.apply_slew_limits();
    assert!(
        (reg.output_scaled(thr) - (250.0 + step)).abs() < 1e-4,
        "expected {}, got {}",
        250.0 + step,
        reg.output_scaled(thr)
    );
}
