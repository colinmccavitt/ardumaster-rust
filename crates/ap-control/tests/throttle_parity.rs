//! The multicopter throttle and lean-angle logic against the real firmware.
//!
//! One coverage limit is worth stating plainly. `get_throttle_boosted` reads
//! `cos_pitch * cos_roll` from the AHRS, which cannot be driven in a harness
//! built outside a vehicle — it reports level, so every recorded row has
//! `cos_tilt == 1` and `inverted_factor` pinned at 1. The 60-to-90-degree
//! fade is therefore *not* parity-verified; it is covered by
//! [`the_inverted_fade_behaves`] below, which exercises the port directly.
//! Recording that gap is the point: an untested branch that nobody has
//! written down reads as a tested one.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_control::throttle_mix::{ThrottleMix, ThrottleMixConfig, VehicleThrottleState};

/// Bit-exact float from the fixture's `%u` column.
fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

const TOL: f32 = 3e-5;

struct Fixture {
    config: ThrottleMixConfig,
    state: VehicleThrottleState,
    dt: f32,
    sections: std::collections::HashMap<String, Vec<Vec<String>>>,
}

fn fixture() -> Fixture {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/throttle_mix.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut sections: std::collections::HashMap<String, Vec<Vec<String>>> = Default::default();
    let mut current = String::new();
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag.to_owned();
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        sections
            .entry(current.clone())
            .or_default()
            .push(line.split(',').map(str::to_owned).collect());
    }

    let c = &sections.get("config").expect("config section")[0];
    assert_eq!(c.len(), 8, "malformed config row");

    Fixture {
        config: ThrottleMixConfig {
            angle_limit_tc: f(&c[1]),
            thr_mix_man: 0.5,
            thr_mix_min: f(&c[2]),
            thr_mix_max: f(&c[3]),
            throttle_gain_boost: 0.0,
            angle_boost_enabled: c[7].trim() == "1",
        },
        state: VehicleThrottleState {
            throttle_thrust_max: f(&c[5]),
            throttle_hover: f(&c[4]),
            throttle_in: 0.0,
            throttle_out: 0.0,
            throttle_slew_rate: 0.0,
            cos_tilt: f(&c[6]),
            thrust_angle_rad: 0.0,
        },
        dt: f(&c[0]),
        sections,
    }
}

/// The lean-angle limit's filtered trajectory.
#[test]
fn the_lean_angle_limit_matches_upstream() {
    let fx = fixture();
    let rows = fx.sections.get("leanangle").expect("leanangle section");
    assert!(
        f(&fx.sections["config"][0][5]) > 0.0,
        "throttle_thrust_max is zero, so every row takes the divide-by-zero \
         guard and the filter underneath is untested"
    );

    let mut mix = ThrottleMix::new();
    let mut largest = 0.0_f32;
    let mut span = (f32::MAX, f32::MIN);

    for r in rows {
        assert_eq!(r.len(), 3, "malformed lean-angle row");
        let step: usize = r[0].parse().expect("step");
        mix.update_althold_lean_angle_max(f(&r[1]), &fx.state, &fx.config, fx.dt);

        let got = mix.althold_lean_angle_max_rad();
        let want = f(&r[2]);
        let diff = libm::fabsf(got - want);
        largest = largest.max(diff);
        assert!(
            diff < TOL,
            "step {step} lean_max: {got} != upstream {want} (diff {diff})"
        );
        span = (span.0.min(want), span.1.max(want));
    }

    assert!(
        span.1 - span.0 > 0.5,
        "the recorded limit only moved over {:?}; the sequence is not \
         exercising the filter",
        span
    );
    println!(
        "{} lean-angle steps, largest difference {largest:e}, limit swept {:.4}..{:.4} rad",
        rows.len(),
        span.0,
        span.1
    );
}

/// Tilt compensation, swept over throttle and target lean.
#[test]
fn the_throttle_boost_matches_upstream() {
    let fx = fixture();
    let rows = fx.sections.get("boost").expect("boost section");

    let mut mix = ThrottleMix::new();
    let mut largest = 0.0_f32;

    for r in rows {
        assert_eq!(r.len(), 5, "malformed boost row");
        let idx: usize = r[0].parse().expect("idx");
        let state = VehicleThrottleState {
            thrust_angle_rad: f(&r[2]),
            ..fx.state
        };

        let got = mix.get_throttle_boosted(f(&r[1]), &state, &fx.config);
        for (label, value, want) in [
            ("boosted", got, f(&r[3])),
            ("angle_boost", mix.angle_boost(), f(&r[4])),
        ] {
            let diff = libm::fabsf(value - want);
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }
    }

    println!("{} boost rows, largest difference {largest:e}", rows.len());
}

/// The average-maximum throttle handed to the mixer.
#[test]
fn the_avg_max_throttle_matches_upstream() {
    let fx = fixture();
    let rows = fx.sections.get("avgmax").expect("avgmax section");

    let mut largest = 0.0_f32;
    for r in rows {
        assert_eq!(r.len(), 4, "malformed avgmax row");
        let idx: usize = r[0].parse().expect("idx");

        let mut mix = ThrottleMix::new();
        mix.set_throttle_mix_value(f(&r[1]));

        let got = mix.get_throttle_avg_max(f(&r[2]), &fx.state);
        let want = f(&r[3]);
        let diff = libm::fabsf(got - want);
        largest = largest.max(diff);
        assert!(
            diff < TOL,
            "row {idx} avg_max: {got} != upstream {want} (diff {diff})"
        );
    }

    println!(
        "{} avg-max rows, largest difference {largest:e}",
        rows.len()
    );
}

/// The mix slew, both directions including the snap-down.
#[test]
fn the_mix_slew_matches_upstream() {
    let fx = fixture();
    let rows = fx.sections.get("mixslew").expect("mixslew section");

    let mut mix = ThrottleMix::new();
    mix.set_throttle_mix_value(0.1);

    let mut largest = 0.0_f32;
    let mut rose = false;
    let mut fell = false;
    let mut previous = 0.1_f32;

    for r in rows {
        assert_eq!(r.len(), 5, "malformed mixslew row");
        let step: usize = r[0].parse().expect("step");

        mix.set_throttle_mix_desired(f(&r[1]));
        let state = VehicleThrottleState {
            throttle_in: f(&r[2]),
            throttle_out: f(&r[3]),
            ..fx.state
        };
        mix.update_throttle_rpy_mix(&state, fx.dt);

        let got = mix.mix();
        let want = f(&r[4]);
        let diff = libm::fabsf(got - want);
        largest = largest.max(diff);
        assert!(
            diff < TOL,
            "step {step} mix: {got} != upstream {want} (diff {diff})"
        );

        if want > previous + 1e-9 {
            rose = true;
        }
        if want < previous - 1e-9 {
            fell = true;
        }
        previous = want;
    }

    // Rising and falling are separate code with different rates, and the
    // snap-down lives only in the falling branch.
    assert!(
        rose && fell,
        "the sequence must slew both ways (rose {rose}, fell {fell})"
    );

    // The falling branch has two features the slew rate alone does not reach:
    // the snap-down to the mix the mixer actually used, and the final clamp
    // floor. A sequence that misses either passes while testing neither --
    // mutation testing caught precisely that, twice.
    let mixes: Vec<f32> = rows.iter().map(|r| f(&r[4])).collect();
    let biggest_drop = mixes
        .windows(2)
        .map(|w| w[0] - w[1])
        .fold(0.0_f32, f32::max);
    assert!(
        biggest_drop > 0.5 * fx.dt * 2.0,
        "no step fell faster than the slew rate, so the snap-down never bound          (biggest drop {biggest_drop})"
    );
    let floor = mixes.iter().copied().fold(f32::MAX, f32::min);
    assert!(
        libm::fabsf(floor - 0.1) < 1e-6,
        "the mix should have been held at the 0.1 clamp floor, bottomed at {floor}"
    );

    println!(
        "{} mix-slew steps, largest difference {largest:e}, both directions covered",
        rows.len()
    );
}

/// The inverted fade, which the recording cannot reach.
///
/// The harness AHRS reports level, so every recorded row has `cos_tilt == 1`.
/// This drives the port's own input across the fade instead. It is not parity
/// against upstream — it is the behaviour upstream's source describes, checked
/// against the port, and it is labelled as such rather than counted as
/// verification it is not.
#[test]
fn the_inverted_fade_behaves() {
    let fx = fixture();
    let mut mix = ThrottleMix::new();
    let state = |cos_tilt: f32| VehicleThrottleState {
        cos_tilt,
        thrust_angle_rad: 0.0,
        ..fx.state
    };

    // Upright: full throttle through, no boost at zero target lean.
    let upright = mix.get_throttle_boosted(0.5, &state(1.0), &fx.config);
    assert!(
        libm::fabsf(upright - 0.5) < 1e-6,
        "level and unbanked should pass throttle unchanged, got {upright}"
    );

    // 60 degrees: cos is 0.5, so 10*cos is still above 1 and nothing fades.
    let at_60 = mix.get_throttle_boosted(0.5, &state(0.5), &fx.config);
    assert!(
        libm::fabsf(at_60 - 0.5) < 1e-6,
        "the fade should not have started at 60 degrees, got {at_60}"
    );

    // Past 84 degrees cos drops under 0.1 and the fade bites. Asserted as a
    // value, not a range: "somewhere between 0 and 0.5" admits any wrong
    // fade rate, and a mutation moving the 10.0 slipped through exactly that
    // gap.
    let at_85 = mix.get_throttle_boosted(0.5, &state(0.08), &fx.config);
    let expected = 0.5 * (10.0 * 0.08_f32).clamp(0.0, 1.0);
    assert!(
        libm::fabsf(at_85 - expected) < 1e-6,
        "the fade is linear in 10*cos_tilt: expected {expected}, got {at_85}"
    );

    // Inverted: cos is negative, the factor clamps to zero, throttle is cut.
    let inverted = mix.get_throttle_boosted(0.5, &state(-0.7), &fx.config);
    assert!(
        libm::fabsf(inverted) < 1e-9,
        "inverted, the boost must be withdrawn entirely, got {inverted}"
    );

    println!(
        "inverted fade: 1.0 -> {upright}, 0.5 -> {at_60}, 0.08 -> {at_85}, -0.7 -> {inverted}"
    );
}

/// The parameter sanity check, including the pair rule.
#[test]
fn the_parameter_sanity_check_behaves() {
    use ap_control::throttle_mix::{
        parameter_sanity_check, THR_MIX_MAX_DEFAULT, THR_MIX_MIN_DEFAULT,
    };

    // In range: untouched.
    assert_eq!(parameter_sanity_check(0.5, 0.2, 0.8), (0.5, 0.2, 0.8));

    // Out of range: clamped to the per-parameter limits.
    let (man, min, max) = parameter_sanity_check(9.0, 0.0, 99.0);
    assert!((man - 4.0).abs() < 1e-6, "man clamps to 4.0, got {man}");
    assert!((min - 0.1).abs() < 1e-6, "min clamps to 0.1, got {min}");
    assert!((max - 5.0).abs() < 1e-6, "max clamps to 5.0, got {max}");

    // The fourth rule -- floor above ceiling, replace both with defaults --
    // cannot fire, and this proves it rather than asserting a reset that
    // never happens.
    //
    // The individual clamps land min in [0.1, 0.5] and max in [0.5, 5.0], so
    // after them min <= 0.5 <= max always. `min > max` would need min > 0.5,
    // which its own clamp forbids. Sweeping the corners, including the values
    // that would invert before clamping:
    for &(man, min_in, max_in) in &[
        (0.5_f32, 0.45_f32, 0.5_f32),
        (0.5, 0.5, 0.5),
        (0.5, 0.5, 0.49),
        (0.5, 0.5, 0.0),
        (0.5, 5.0, 0.1),
        (0.5, 0.5, -100.0),
        (0.5, f32::MAX, f32::MIN),
    ] {
        let (_, min, max) = parameter_sanity_check(man, min_in, max_in);
        assert!(
            (min, max) != (THR_MIX_MIN_DEFAULT, THR_MIX_MAX_DEFAULT)
                || (min_in, max_in) == (THR_MIX_MIN_DEFAULT, THR_MIX_MAX_DEFAULT),
            "({min_in}, {max_in}) reached the pair rule, which was thought \
             unreachable -- the reasoning above is wrong and needs redoing"
        );
        assert!(
            min <= max,
            "({min_in}, {max_in}) produced an inverted pair: {min} > {max}"
        );
    }

    // NaN does not reach it either: every comparison against NaN is false, so
    // no clamp applies and the pair test fails too.
    let (_, min, max) = parameter_sanity_check(0.5, f32::NAN, f32::NAN);
    assert!(
        min.is_nan() && max.is_nan(),
        "NaN passes through untouched, got {min}, {max}"
    );

    println!("the pair rule is unreachable after the individual clamps");
}
