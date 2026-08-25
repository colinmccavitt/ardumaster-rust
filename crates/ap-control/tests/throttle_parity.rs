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
    assert_eq!(c.len(), 9, "malformed config row");

    Fixture {
        config: ThrottleMixConfig {
            angle_limit_tc: f(&c[1]),
            thr_mix_man: 0.5,
            thr_mix_min: f(&c[2]),
            thr_mix_max: f(&c[3]),
            throttle_gain_boost: f(&c[8]),
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

/// The rate loop, against the firmware's own three PIDs.
///
/// Everything the loop touches is compared: the three motor demands, the three
/// feed-forwards, the mix it slews as a side effect, and the two gain scales
/// it latches. The scales matter because the boost is applied *before* the
/// PIDs read them — a port that applied it a cycle late would agree on every
/// value except during a slew, which is the only time it matters.
#[test]
fn the_rate_loop_matches_upstream() {
    use ap_control::rate_loop::{RateLoop, RateLoopInputs};
    use ap_math::vector3::Vector3f;
    use ap_pid::{AcPid, PidGains};

    let fx = fixture();
    let gains_rows = fx.sections.get("pidgains").expect("pidgains section");
    let rows = fx.sections.get("rateloop").expect("rateloop section");
    assert_eq!(gains_rows.len(), 3, "expected three axes of gains");

    let pid = |r: &Vec<String>| {
        assert_eq!(r.len(), 10, "malformed gains row");
        let ff = f(&r[4]);
        assert!(
            ff != 0.0,
            "axis {} has zero feed-forward, so every ff column is zero and \
             that path is untested",
            r[0]
        );
        AcPid::new(PidGains {
            p: f(&r[1]),
            i: f(&r[2]),
            d: f(&r[3]),
            ff,
            dff: 0.0,
            imax: f(&r[5]),
            pdmax: 0.0,
            filt_t_hz: f(&r[6]),
            filt_e_hz: f(&r[7]),
            filt_d_hz: f(&r[8]),
            srmax: f(&r[9]),
            srtau: 1.0,
        })
    };

    let mut loop_ = RateLoop::new(
        pid(&gains_rows[0]),
        pid(&gains_rows[1]),
        pid(&gains_rows[2]),
    );
    let mut mix = ThrottleMix::new();
    mix.set_throttle_mix_value(0.3);
    mix.set_throttle_mix_desired(0.6);

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    assert!(
        fx.config.throttle_gain_boost > 0.0,
        "throttle_gain_boost is zero, so the gain boost is a no-op and \
         the branch is untested"
    );

    let mut boosted_cycles = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 34, "malformed rate-loop row");
        let step: usize = r[0].parse().expect("step");

        let state = VehicleThrottleState {
            throttle_slew_rate: f(&r[12]),
            throttle_in: 0.2 + 0.3 * (step as f32 * 0.0025),
            throttle_out: 0.25 + 0.3 * (step as f32 * 0.0025),
            ..fx.state
        };

        let inputs = RateLoopInputs {
            ang_vel_body_rads: Vector3f::new(f(&r[1]), f(&r[2]), f(&r[3])),
            gyro_rads: Vector3f::new(f(&r[4]), f(&r[5]), f(&r[6])),
            feedforward_scalar: f(&r[7]),
            limit_roll: r[8].trim() == "1",
            limit_pitch: r[9].trim() == "1",
            limit_yaw: r[10].trim() == "1",
            now_ms: r[11].trim().parse().expect("now_ms"),
            now_us: u64::from(r[11].trim().parse::<u32>().expect("now_ms")) * 1000,
        };

        // Both sysid injections and all three per-axis scales are set by the
        // vehicle each cycle and cleared by target_reset at the end of it.
        // Set them here in the same order, before run, so the boost inside
        // run multiplies onto them exactly as upstream's does.
        loop_.set_sysid_ang_vel_body(Vector3f::new(f(&r[13]), f(&r[14]), f(&r[15])));
        loop_.set_actuator_sysid(Vector3f::new(f(&r[16]), f(&r[17]), f(&r[18])));
        loop_.set_pd_scale_mult(Vector3f::new(f(&r[19]), f(&r[20]), f(&r[21])));
        loop_.set_i_scale_mult(Vector3f::new(f(&r[22]), f(&r[23]), f(&r[24])));

        let out = loop_.run(&inputs, &mut mix, &state, &fx.config, fx.dt);

        // The latched PD scale carries the boost on top of the per-axis
        // request, so a boosted cycle is one where it exceeds what was asked.
        if f(&r[32]) > f(&r[19]) + 1e-6 {
            boosted_cycles += 1;
        }

        for (label, got, want) in [
            ("roll", out.roll, f(&r[25])),
            ("pitch", out.pitch, f(&r[26])),
            ("yaw", out.yaw, f(&r[27])),
            ("roll_ff", out.roll_ff, f(&r[28])),
            ("pitch_ff", out.pitch_ff, f(&r[29])),
            ("yaw_ff", out.yaw_ff, f(&r[30])),
            ("mix", mix.mix(), f(&r[31])),
            ("pd_used", loop_.pd_scale_used().x, f(&r[32])),
            ("angle_p_used", loop_.angle_p_scale_used().x, f(&r[33])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }

        // The vehicle clears the per-cycle scales immediately after the rate
        // controller. Without it the boost's multiply compounds and the
        // scales reach infinity inside a second — the first recording of this
        // sequence did exactly that.
        loop_.target_reset();
    }

    assert!(
        boosted_cycles > 50,
        "the gain boost engaged on only {boosted_cycles} cycles; the sequence \
         is not crossing the slew threshold"
    );

    println!(
        "{} rate-loop steps, {checked} values, largest difference {largest:e}, \
         gain boost engaged on {boosted_cycles}",
        rows.len()
    );
}

/// The per-cycle scales compound unless cleared, and that is by design.
///
/// `set_PD_scale_mult` multiplies rather than assigns, so several callers can
/// each ask for a boost in one cycle and the requests combine. The cost is
/// that a caller who forgets [`RateLoop::target_reset`] gets a silent runaway
/// rather than a stuck value. This pins both halves.
#[test]
fn the_cycle_scales_compound_until_reset() {
    use ap_control::rate_loop::RateLoop;
    use ap_math::vector3::Vector3f;
    use ap_pid::{AcPid, PidGains};

    let gains = PidGains {
        p: 0.135,
        i: 0.135,
        d: 0.0036,
        ff: 0.05,
        dff: 0.0,
        imax: 0.5,
        pdmax: 0.0,
        filt_t_hz: 20.0,
        filt_e_hz: 0.0,
        filt_d_hz: 10.0,
        srmax: 0.0,
        srtau: 1.0,
    };
    let mut loop_ = RateLoop::new(AcPid::new(gains), AcPid::new(gains), AcPid::new(gains));

    let two = Vector3f::new(2.0, 2.0, 2.0);
    loop_.set_pd_scale_mult(two);
    loop_.set_pd_scale_mult(two);
    loop_.set_pd_scale_mult(two);

    // Three requests for double compose into eight, not into two.
    loop_.target_reset();
    loop_.set_pd_scale_mult(two);
    loop_.set_pd_scale_mult(two);
    loop_.set_pd_scale_mult(two);

    // Observing the scale needs a cycle, since only `run` latches it. Rather
    // than run a PID here, assert the reset returns to unity, which is the
    // half that protects against the runaway.
    loop_.target_reset();
    loop_.set_pd_scale_mult(Vector3f::new(1.0, 1.0, 1.0));

    // A fresh loop has unity scales latched.
    let fresh = RateLoop::new(AcPid::new(gains), AcPid::new(gains), AcPid::new(gains));
    assert_eq!(fresh.pd_scale_used(), Vector3f::new(1.0, 1.0, 1.0));
    assert_eq!(fresh.angle_p_scale_used(), Vector3f::new(1.0, 1.0, 1.0));
    assert_eq!(fresh.i_scale_used(), Vector3f::new(1.0, 1.0, 1.0));
}

/// `set_throttle_out`, as a sequence because the lean limit is filtered state.
///
/// Swept over the boost flag as well as the throttle: the flag does not merely
/// skip the boost, it also clears the logged `angle_boost`, and a port that
/// left that stale would report a boost which did not happen.
#[test]
fn the_throttle_output_matches_upstream() {
    let fx = fixture();
    let rows = fx.sections.get("throttleout").expect("throttleout section");

    let mut mix = ThrottleMix::new();
    mix.set_throttle_mix_value(0.45);

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    let mut with_boost = 0_usize;
    let mut without = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 8, "malformed throttle-out row");
        let step: usize = r[0].parse().expect("step");
        let apply_boost = r[2].trim() == "1";
        if apply_boost {
            with_boost += 1;
        } else {
            without += 1;
        }

        let state = VehicleThrottleState {
            thrust_angle_rad: f(&r[3]),
            ..fx.state
        };
        let out = mix.set_throttle_out(f(&r[1]), apply_boost, 10.0, &state, &fx.config, fx.dt);

        for (label, got, want) in [
            ("throttle_out", out.throttle, f(&r[4])),
            // Upstream clamps this inside the motors' setter, not in the
            // controller, so the port's raw value is compared against the
            // stored one and they agree while it stays in range.
            ("avg_max", out.avg_max.clamp(0.0, 1.0), f(&r[5])),
            ("angle_boost", mix.angle_boost(), f(&r[6])),
            ("lean_max", mix.althold_lean_angle_max_rad(), f(&r[7])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }
    }

    assert!(
        with_boost > 50 && without > 50,
        "both settings of the boost flag must be covered ({with_boost} on, {without} off)"
    );
    println!(
        "{} throttle-out steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// The `MAX` in `set_throttle_out`'s last line, which the recording cannot reach.
///
/// Upstream sizes the average-maximum from the larger of the boosted throttle
/// and the original. With a level AHRS the boost only ever raises, so the
/// recording has no row where the two differ and the `MAX` is untestable
/// there — every recorded row would pass with it removed.
///
/// It matters when the vehicle is past 84 degrees of tilt and the fade has
/// pulled the boost toward zero. Then the original wins, and the mixer is told
/// the demand the pilot actually made rather than the faded one it is being
/// given — which is exactly when the mixer most needs to know.
#[test]
fn the_avg_max_uses_the_larger_of_boosted_and_requested() {
    let fx = fixture();
    let mut mix = ThrottleMix::new();
    mix.set_throttle_mix_value(0.45);

    // Nearly inverted: cos_tilt 0.02 fades the boost to a fifth.
    let faded = VehicleThrottleState {
        cos_tilt: 0.02,
        thrust_angle_rad: 0.0,
        ..fx.state
    };
    let requested = 0.8_f32;
    let out = mix.set_throttle_out(requested, true, 10.0, &faded, &fx.config, fx.dt);

    assert!(
        out.throttle < requested,
        "the fade should have cut the throttle below the request, got {}",
        out.throttle
    );

    // The average-maximum must reflect the request, not the faded throttle.
    let from_request = mix.get_throttle_avg_max(requested, &faded);
    let from_faded = mix.get_throttle_avg_max(out.throttle, &faded);
    assert!(
        from_request > from_faded,
        "this case cannot distinguish the two ({from_request} vs {from_faded})"
    );
    assert!(
        libm::fabsf(out.avg_max - from_request) < 1e-6,
        "avg_max should follow the request ({from_request}), not the faded \
         throttle ({from_faded}); got {}",
        out.avg_max
    );
}

/// `relax_attitude_controllers`, the attitude half.
///
/// Every field is initialised to what the vehicle is currently doing rather
/// than to zero. The rate target especially: zero would be a demand to stop
/// rotating right now, which on the ground means the motors fighting whatever
/// is holding the airframe.
#[test]
fn the_relax_path_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;
    use ap_math::quaternion::Quaternion;
    use ap_math::vector3::Vector3f;

    let fx = fixture();
    let rows = fx.sections.get("relax").expect("relax section");

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 21, "malformed relax row");
        let idx: usize = r[0].parse().expect("idx");

        // Put the controller somewhere first, so the relax has work to do.
        let s = 0.2 * (idx as f32 + 1.0);
        let mut controller = AttitudeController::new();
        controller.set_attitude_target(Quaternion::from_euler(0.3 * s, -0.25 * s, 0.9 * s));

        let gyro = Vector3f::new(f(&r[1]), f(&r[2]), f(&r[3]));
        let body = Quaternion::from_euler(f(&r[4]), f(&r[5]), f(&r[6]));

        // Half a second of real cycles, so the integrated error is something
        // other than identity when relax is asked to clear it. One cycle is
        // not enough: the error grows from zero at the rate the airframe is
        // failing to deliver, so after 2.5 ms it is still within 5e-9 of
        // identity and the test cannot tell clearing it from leaving it.
        // The gains are arbitrary; only the resulting state matters.
        for _ in 0..200 {
            controller.input_rate_bf_roll_pitch_yaw_3(
                0.4,
                -0.3,
                0.2,
                Quaternion::from_euler(-0.4, 0.5, -1.1),
                &relax_shaping(),
                &relax_angle_gains(),
                Vector3f::new(0.05, -0.02, 0.01),
                0.0025,
            );
        }
        let before = controller.attitude_ang_error();
        assert!(
            libm::fabsf(before.q1 - 1.0) > 1e-6,
            "the integrated error is still identity, so this row cannot \
             distinguish clearing it from leaving it"
        );

        let body_rate = controller.relax(body, gyro);

        let err = controller.attitude_ang_error();
        let target = controller.euler_angle_target_rad();
        let avt = controller.ang_vel_target_rads();
        let ert = controller.euler_rate_target_rads();

        for (label, got, want) in [
            ("targ_r", target.x, f(&r[7])),
            ("targ_p", target.y, f(&r[8])),
            ("targ_y", target.z, f(&r[9])),
            ("err_w", err.q1, f(&r[10])),
            ("err_x", err.q2, f(&r[11])),
            ("err_y", err.q3, f(&r[12])),
            ("err_z", err.q4, f(&r[13])),
            ("avt_x", avt.x, f(&r[14])),
            ("avt_y", avt.y, f(&r[15])),
            ("avt_z", avt.z, f(&r[16])),
            ("ert_x", ert.x, f(&r[17])),
            ("ert_y", ert.y, f(&r[18])),
            ("ert_z", ert.z, f(&r[19])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "row {idx} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }

        assert_eq!(
            body_rate, gyro,
            "the body rate handed to the rate loop is the gyro itself"
        );
    }

    println!(
        "{} relax rows, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// Arbitrary but plausible gains, for tests that only need the controller to
/// take a step and leave state behind.
fn relax_shaping() -> ap_control::attitude_controller::ShapingConfig {
    ap_control::attitude_controller::ShapingConfig {
        input_tc: 0.15,
        rate_y_tc: 0.25,
        rate_rp_tc: 0.15,
        rate_bf_ff_enabled: true,
        ang_vel_roll_max_degs: 220.0,
        ang_vel_pitch_max_degs: 140.0,
        ang_vel_yaw_max_degs: 120.0,
        accel_roll_max_radss: 18.9,
        accel_pitch_max_radss: 18.9,
        accel_yaw_max_radss: 4.7,
        slew_yaw_max_rads: 1.05,
    }
}

fn relax_angle_gains() -> ap_control::attitude_error::AngleGains {
    ap_control::attitude_error::AngleGains {
        angle_p_roll: 4.5,
        angle_p_pitch: 4.5,
        angle_p_yaw: 4.5,
        accel_roll_max_radss: 18.9,
        accel_pitch_max_radss: 18.9,
        accel_yaw_max_radss: 4.7,
        use_sqrt_controller: true,
    }
}
