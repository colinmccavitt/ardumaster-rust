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
    assert!(funcs.len() >= 3, "malformed functions row");

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

/// The loop-based override, and what it does to the pulse width.
///
/// Neither the counter nor the override flag is reachable from outside
/// `SRV_Channels`, so the recording captures the only observable: the width.
/// That is the right one — holding the width is the mechanism's entire
/// purpose — and the channel's scaled value sweeps underneath it, so the
/// moment an override lapses the output visibly returns to tracking it.
///
/// Four requests, each testing something different:
///
/// - 20 ms on a 2.5 ms loop: eight loops, the ordinary case.
/// - 0 ms: documented as clearing the override. It does not hold the width for
///   even one loop, because `calc_pwm` steps the counters before converting,
///   sees zero and clears the flag first.
/// - 1 ms: shorter than a loop. The round-up gives it one anyway, which is the
///   difference between a scripted override working and silently doing nothing.
/// - 60000 ms: 24000 loops, long enough to hold for the rest of the sequence.
#[test]
fn the_override_timeout_matches_upstream() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let s = sections();
    let cfg = &s.get("functions").expect("functions section")[0];
    assert_eq!(cfg.len(), 10, "malformed functions row");

    let elev = Function(cfg[2].trim().parse().expect("elevator function"));
    let recorded_channels: usize = cfg[3].trim().parse().expect("channel count");
    assert_eq!(
        recorded_channels, NUM_SERVO_CHANNELS,
        "the firmware was built with {recorded_channels} servo channels but the \
         port assumes {NUM_SERVO_CHANNELS}; every channel index below would be \
         against a different table"
    );

    let loop_period_us: u32 = cfg[4].trim().parse().expect("loop period");
    let chan: usize = cfg[5].trim().parse().expect("channel");
    let servo_min: u16 = cfg[6].trim().parse().expect("servo min");
    let servo_trim: u16 = cfg[7].trim().parse().expect("servo trim");
    let servo_max: u16 = cfg[8].trim().parse().expect("servo max");

    let rows = s.get("override").expect("override section");

    // The channel's output type and full-deflection value are private in
    // upstream with no accessor, so they cannot be recorded. They do not need
    // to be: the sequence records fifty-odd scaled-to-width pairs on the steps
    // where no override is holding, and those pin the conversion completely.
    // If this were the wrong type the parity comparison would fail, which is
    // the verification rather than an assumption.
    let config = ServoChannel::angle(servo_min, servo_trim, servo_max, 4500);

    let mut reg = Registry::new();
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let function = if i == chan { elev } else { Function(0) };
            OutputChannel::new(config, function, u8::try_from(i).expect("channel fits"))
        })
        .collect();
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);

    let mut held = 0_usize;
    let mut tracked = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 5, "malformed override row");
        let step: usize = r[0].parse().expect("step");
        let request: i32 = r[1].trim().parse().expect("request");
        let pwm: i32 = r[2].trim().parse().expect("pwm");

        reg.set_output_scaled(elev, f(&r[3]));

        if request >= 0 {
            reg.set_output_pwm_chan_timeout(
                &mut channels,
                chan,
                u16::try_from(pwm).expect("pwm fits"),
                u16::try_from(request).expect("timeout fits"),
                loop_period_us,
            );
        }

        reg.calc_pwm(&mut channels, false);

        let got = channels[chan].output_pwm();
        let want: u16 = r[4].trim().parse().expect("out pwm");
        assert_eq!(
            got, want,
            "step {step}: pulse width {got} != upstream {want}"
        );

        if reg.override_counter(chan) > 0 {
            held += 1;
        } else {
            tracked += 1;
        }
    }

    // A sequence that never held, or never let go, would pass with the counter
    // hard-wired either way.
    assert!(
        held > 5 && tracked > 5,
        "the override must both hold and lapse ({held} held, {tracked} tracked)"
    );

    println!(
        "{} override steps, {held} held, {tracked} tracked",
        rows.len()
    );
}

/// The millisecond-to-loop conversion rounds up.
///
/// A request shorter than one loop would round to zero and do nothing at all,
/// which for a scripted override is the difference between working and
/// silently failing. Checked directly because the recording covers one such
/// case and this covers the boundary either side of it.
#[test]
fn a_sub_loop_timeout_still_gets_one_loop() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let period_us = 2500_u32;
    let config = ServoChannel::angle(1100, 1500, 1900, 4500);
    let mut reg = Registry::new();
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| OutputChannel::new(config, Function(0), u8::try_from(i).expect("fits")))
        .collect();

    for (timeout_ms, expected_loops) in [(0_u16, 0_u16), (1, 1), (2, 1), (3, 2), (5, 2), (25, 10)] {
        reg.set_output_pwm_chan_timeout(&mut channels, 0, 1500, timeout_ms, period_us);
        assert_eq!(
            reg.override_counter(0),
            expected_loops,
            "{timeout_ms} ms on a {period_us} us loop should be {expected_loops} loops"
        );
    }
}

/// The `had_pwm` branch, both ways.
///
/// Upstream clears the channel's pulse-width bit only when it was not already
/// set, and its comment explains why: with the bit clear the channel returns to
/// its scaled value once the override lapses, but the pre-override width is not
/// stored anywhere, so a channel that *was* driven by a width has nothing to go
/// back to and keeping it frozen is the only honest option.
///
/// The recorded sequence cannot show this. It writes a scaled value every step,
/// which clears the bit anyway, so both branches produce the same mask and the
/// distinction is invisible — mutation testing found exactly that.
#[test]
fn the_had_pwm_branch_decides_what_happens_after_the_override() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let elev = Function(19);
    let config = ServoChannel::angle(1100, 1500, 1900, 4500);
    let make = || -> Vec<OutputChannel> {
        (0..NUM_SERVO_CHANNELS)
            .map(|i| {
                let function = if i == 4 { elev } else { Function(0) };
                OutputChannel::new(config, function, u8::try_from(i).expect("fits"))
            })
            .collect()
    };
    let functions: Vec<Function> = make().iter().map(|c| c.function).collect();

    // Not previously driven by a width: the bit is cleared, so the channel
    // returns to its scaled value when the override lapses.
    let mut reg = Registry::new();
    let mut channels = make();
    reg.update_aux_servo_function(&functions);
    reg.set_output_scaled(elev, 0.0);
    reg.set_output_pwm_chan_timeout(&mut channels, 4, 1777, 3, 2500);
    assert_eq!(reg.have_pwm_mask() & (1 << 4), 0, "the bit should be clear");
    reg.calc_pwm(&mut channels, false);

    // Previously driven by a width: the bit stays, so the width persists.
    let mut reg2 = Registry::new();
    let mut channels2 = make();
    reg2.update_aux_servo_function(&functions);
    reg2.set_output_pwm(&mut channels2, elev, 1600);
    assert_ne!(
        reg2.have_pwm_mask() & (1 << 4),
        0,
        "a direct width write should set the bit; this test is vacuous without it"
    );
    reg2.set_output_pwm_chan_timeout(&mut channels2, 4, 1777, 3, 2500);
    assert_ne!(
        reg2.have_pwm_mask() & (1 << 4),
        0,
        "the bit must survive, because there is no pre-override width to restore"
    );
}

/// The loop count saturates rather than wrapping.
///
/// Unreachable at Copter's and Plane's 2.5 ms loop: a `u16` timeout tops out at
/// 65.5 seconds, which is 26214 loops, comfortably inside `u16`. It becomes
/// reachable on a fast loop — at 400 us the same request is 163837 loops, and a
/// wrap would turn the longest possible override into a very short one.
#[test]
fn a_long_timeout_on_a_fast_loop_saturates() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let config = ServoChannel::angle(1100, 1500, 1900, 4500);
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| OutputChannel::new(config, Function(0), u8::try_from(i).expect("fits")))
        .collect();
    let mut reg = Registry::new();

    // 65.535 s at 400 us is 163838 loops, which does not fit in u16.
    reg.set_output_pwm_chan_timeout(&mut channels, 0, 1500, u16::MAX, 400);
    assert_eq!(
        reg.override_counter(0),
        u16::MAX,
        "the count must saturate; wrapping would turn the longest override \
         into one of a few hundred loops"
    );

    // And the same request on a 2.5 ms loop fits, so it is not saturating
    // everything indiscriminately.
    reg.set_output_pwm_chan_timeout(&mut channels, 1, 1500, u16::MAX, 2500);
    assert_eq!(reg.override_counter(1), 26214);
}

/// The function-scoped setters, across two channels that disagree.
///
/// Both carry the same function but one is reversed with wider travel, so
/// every endpoint resolves differently on each. `Limit::Min` sends the upright
/// channel to its smallest width and the reversed one to its largest, because
/// `Min` names an end of the *surface's* travel rather than a pulse width.
#[test]
fn the_function_setters_match_upstream() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::Limit;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let s = sections();
    let rows = s.get("setters").expect("setters section");
    let elev = Function(19);

    // The channels the recording configured.
    let mut configs = [ServoChannel::angle(1100, 1500, 1900, 4500); 2];
    configs[1] = ServoChannel::angle(1000, 1500, 2000, 4500);
    configs[1].reversed = true;

    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let (config, function) = match i {
                6 => (configs[0], elev),
                7 => (configs[1], elev),
                _ => (configs[0], Function(0)),
            };
            OutputChannel::new(config, function, u8::try_from(i).expect("fits"))
        })
        .collect();

    let mut reg = Registry::new();
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);
    Registry::set_trim_to_pwm_for(&mut channels, elev, 1500);

    // The same ten steps the recording took.
    #[derive(Clone, Copy)]
    enum Step {
        SetLimit(Limit),
        ToTrim,
        TrimToPwm(u16),
        TrimToMin(bool),
    }
    let steps = [
        Step::SetLimit(Limit::Trim),
        Step::SetLimit(Limit::Min),
        Step::SetLimit(Limit::Max),
        Step::SetLimit(Limit::ZeroPwm),
        Step::ToTrim,
        Step::TrimToPwm(1234),
        Step::TrimToMin(false),
        Step::SetLimit(Limit::Min),
        Step::TrimToMin(true),
        Step::SetLimit(Limit::Min),
    ];

    let mut checked = 0_usize;
    let mut disagreed = 0_usize;

    for (case, step) in steps.iter().enumerate() {
        match *step {
            Step::SetLimit(l) => reg.set_output_limit(&mut channels, elev, l),
            Step::ToTrim => reg.set_output_to_trim(&mut channels, elev),
            Step::TrimToPwm(pwm) => Registry::set_trim_to_pwm_for(&mut channels, elev, pwm),
            Step::TrimToMin(ignore) => {
                Registry::set_trim_to_min_for(&mut channels, elev, ignore);
            }
        }

        let case_rows: Vec<&Vec<String>> = rows
            .iter()
            .filter(|r| r[0].trim().parse::<usize>() == Ok(case))
            .collect();
        assert_eq!(case_rows.len(), 2, "case {case}: expected two channels");

        let mut widths = Vec::new();
        for r in case_rows {
            assert_eq!(r.len(), 7, "malformed setters row");
            let chan: usize = r[1].trim().parse().expect("chan");
            let ch = &channels[chan];
            for (label, got, want) in [
                (
                    "reversed",
                    u16::from(ch.config.reversed),
                    r[2].trim().parse().expect("rev"),
                ),
                (
                    "servo_min",
                    ch.config.servo_min,
                    r[3].trim().parse().expect("min"),
                ),
                (
                    "servo_trim",
                    ch.config.servo_trim,
                    r[4].trim().parse().expect("trim"),
                ),
                (
                    "servo_max",
                    ch.config.servo_max,
                    r[5].trim().parse().expect("max"),
                ),
                (
                    "out_pwm",
                    ch.output_pwm(),
                    r[6].trim().parse().expect("pwm"),
                ),
            ] {
                assert_eq!(got, want, "case {case} channel {chan} {label}");
                checked += 1;
            }
            widths.push(ch.output_pwm());
        }
        if widths[0] != widths[1] {
            disagreed += 1;
        }
    }

    // If the two channels never diverged, the reversal is not being exercised
    // and every endpoint could be resolved globally without failing.
    assert!(
        disagreed >= 4,
        "the two channels only differed on {disagreed} of {} steps; the \
         per-channel endpoint resolution is barely covered",
        steps.len()
    );

    println!(
        "{} setter values checked, channels diverged on {disagreed} steps",
        checked
    );
}

/// The normalised read, swept by scaled value across two channel shapes.
///
/// Driven the way the aggregate is actually used: it recomputes the width from
/// the function's scaled value before reading, so this covers the conversion
/// and the normalisation together.
///
/// The reversed channel has asymmetric travel and a trim off centre, which is
/// where the two independently-scaled halves show up — its rest position reads
/// -0.2 rather than zero, because the normalisation measures position within
/// the travel, not distance from trim.
#[test]
fn the_normalised_read_matches_upstream() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let s = sections();
    let rows = s.get("norm").expect("norm section");
    assert!(!rows.is_empty(), "no norm rows");

    let mut reg = Registry::new();
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            OutputChannel::new(
                ServoChannel::angle(1100, 1500, 1900, 4500),
                Function(0),
                u8::try_from(i).expect("fits"),
            )
        })
        .collect();

    // Configure exactly what the recording did, read from the recording.
    for r in rows {
        assert_eq!(r.len(), 10, "malformed norm row");
        let chan: usize = r[2].trim().parse().expect("chan");
        let function = Function(r[1].trim().parse().expect("function"));
        let mut config = ServoChannel::angle(
            r[4].trim().parse().expect("min"),
            r[5].trim().parse().expect("trim"),
            r[6].trim().parse().expect("max"),
            4500,
        );
        config.reversed = r[3].trim() == "1";
        channels[chan].config = config;
        channels[chan].function = function;
    }
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    let mut saturated = 0_usize;

    for r in rows {
        let idx: usize = r[0].trim().parse().expect("idx");
        let function = Function(r[1].trim().parse().expect("function"));
        let scaled: f32 = r[7].trim().parse().expect("scaled");

        reg.set_output_scaled(function, scaled);
        let norm = reg.output_norm(&mut channels, function, false);
        let pwm = reg
            .output_pwm_for(&mut channels, function, false)
            .expect("assigned function");

        let want_pwm: u16 = r[8].trim().parse().expect("pwm");
        assert_eq!(pwm, want_pwm, "row {idx} pwm");

        let want_norm = f(&r[9]);
        let diff = (norm - want_norm).abs();
        largest = largest.max(diff);
        assert!(
            diff < 3e-5,
            "row {idx} norm: {norm} != upstream {want_norm} (diff {diff})"
        );
        checked += 2;

        if want_norm.abs() >= 1.0 {
            saturated += 1;
        }
    }

    // Both ends of the travel must appear, or the independent scaling of the
    // two halves is untested.
    assert!(
        saturated > 10,
        "only {saturated} rows reached full deflection; the sweep is not \
         covering the ends"
    );

    println!(
        "{} norm rows, {checked} values, largest difference {largest:e}, \
         {saturated} at full deflection",
        rows.len()
    );
}

/// Three shapes the recorded sweep does not contain.
///
/// Each was found by mutation testing: the recording's channels all have an
/// even min-plus-max, exactly one channel per function, and a sane span, so
/// three branches of `output_norm` and one of `find_channel` were never
/// reached.
#[test]
fn the_normalised_read_handles_the_awkward_shapes() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let make = |config: ServoChannel, function: Function, at: usize| -> Vec<OutputChannel> {
        (0..NUM_SERVO_CHANNELS)
            .map(|i| {
                let f = if i == at { function } else { Function(0) };
                OutputChannel::new(config, f, u8::try_from(i).expect("fits"))
            })
            .collect()
    };

    // 1. An odd span, where mid truncates down and the two divisors differ.
    //    min 1000, max 1901 -> mid 1450, so below uses 450 and above uses 451.
    let odd = ServoChannel::angle(1000, 1450, 1901, 4500);
    let mut ch = OutputChannel::new(odd, Function(21), 0);

    ch.set_output_pwm(1000, true);
    let low = ch.output_norm();
    ch.set_output_pwm(1901, true);
    let high = ch.output_norm();
    assert!(
        (low + 1.0).abs() < 1e-6,
        "the bottom of the travel should read -1, got {low}"
    );
    assert!(
        (high - 1.0).abs() < 1e-6,
        "the top of the travel should read +1, got {high}"
    );

    // One microsecond either side of centre: the divisors differ by one, so
    // the magnitudes must too. Collapsing the branches makes these equal.
    ch.set_output_pwm(1449, true);
    let just_below = ch.output_norm().abs();
    ch.set_output_pwm(1451, true);
    let just_above = ch.output_norm().abs();
    assert!(
        just_below != just_above,
        "an odd span must scale its halves differently; both read {just_below}"
    );

    // 2. A degenerate channel: max at min, so mid <= min.
    let flat = ServoChannel::angle(1500, 1500, 1500, 4500);
    let mut degenerate = OutputChannel::new(flat, Function(21), 0);
    degenerate.set_output_pwm(1700, true);
    assert_eq!(
        degenerate.output_norm(),
        0.0,
        "a channel with no travel must read zero rather than dividing by it"
    );

    // 3. Two channels on one function: the aggregate answers from the first.
    let a = ServoChannel::angle(1100, 1500, 1900, 4500);
    let mut b = a;
    b.reversed = true;
    let func = Function(21);
    let mut channels = make(a, func, 3);
    channels[9].config = b;
    channels[9].function = func;

    let mut reg = Registry::new();
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);
    reg.set_output_scaled(func, 3000.0);

    let norm = reg.output_norm(&mut channels, func, false);
    assert!(
        norm > 0.0,
        "the aggregate must answer from channel 3, the first carrying the \
         function; channel 9 is reversed and would read {norm} negated"
    );
    assert_eq!(
        reg.find_channel(func),
        Some(3),
        "find_channel must return the lowest channel, not the highest"
    );
}

/// `adjust_trim`, held into both of its bounds on both channels.
///
/// Three behaviours, all of which a plausible port gets wrong:
///
/// The magnitude of the argument is ignored — only its sign is read, and the
/// trim moves one microsecond per call regardless. The recorded sequence
/// varies the magnitude from 0.25 to 5.5 precisely so a port scaling by it
/// diverges on the second step.
///
/// The bounds are asymmetric: up stops at 60% of travel, down at 40%. Both
/// channels reach both, which takes a long hold in each direction — the first
/// version of this sequence had each channel finding one bound and the test
/// would have passed with either limit wrong.
///
/// Zero returns immediately and must move nothing, which is why the sequence
/// spends ten steps there.
#[test]
fn the_trim_adjustment_matches_upstream() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let s = sections();
    let rows = s.get("adjtrim").expect("adjtrim section");
    let func = Function(3);

    let upright = ServoChannel::angle(1000, 1500, 2000, 4500);
    let mut reversed = upright;
    reversed.reversed = true;

    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let (config, f) = match i {
                11 => (upright, func),
                12 => (reversed, func),
                _ => (upright, Function(0)),
            };
            OutputChannel::new(config, f, u8::try_from(i).expect("fits"))
        })
        .collect();

    let mut reg = Registry::new();
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);

    let mut checked = 0_usize;
    let mut last_step = usize::MAX;
    let mut seen: std::collections::BTreeMap<usize, (u16, u16)> = Default::default();

    for r in rows {
        assert_eq!(r.len(), 5, "malformed adjtrim row");
        let step: usize = r[0].parse().expect("step");
        let chan: usize = r[2].trim().parse().expect("chan");
        let want: u16 = r[3].trim().parse().expect("trim");

        // Two rows per step; adjust once, then check both channels.
        if step != last_step {
            reg.adjust_trim(&mut channels, func, f(&r[1]));
            last_step = step;
        }

        let got = channels[chan].config.servo_trim;
        assert_eq!(got, want, "step {step} channel {chan} trim");
        checked += 1;

        let entry = seen.entry(chan).or_insert((u16::MAX, 0));
        entry.0 = entry.0.min(got);
        entry.1 = entry.1.max(got);
    }

    // Both channels must have reached both bounds, or a wrong limit passes.
    for (chan, (lo, hi)) in &seen {
        assert_eq!(
            (*lo, *hi),
            (1400, 1600),
            "channel {chan} only covered {lo}..{hi}; with 1000..2000 of travel \
             the bounds are 1400 and 1600 and both must be reached"
        );
    }

    // And the trimmed mask must have recorded both.
    let mask = reg.trimmed_mask();
    assert_eq!(
        mask & ((1 << 11) | (1 << 12)),
        (1 << 11) | (1 << 12),
        "both nudged channels should be in the trimmed mask, got {mask:#x}"
    );

    // Taking it clears it, so a second save writes nothing.
    assert_eq!(reg.take_trimmed(), mask);
    assert_eq!(reg.trimmed_mask(), 0, "taking the set must clear it");

    println!(
        "{checked} trim values checked across {} steps",
        rows.len() / 2
    );
}

/// A degenerate channel is skipped rather than clamped.
///
/// With no travel there is no fraction to express the trim as, so upstream
/// leaves the channel alone instead of giving it an arbitrary answer. Not
/// reachable from the recording, whose channels all have a thousand
/// microseconds of travel.
#[test]
fn adjust_trim_skips_a_channel_with_no_travel() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let func = Function(3);
    let flat = ServoChannel::angle(1500, 1500, 1500, 4500);
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let f = if i == 2 { func } else { Function(0) };
            OutputChannel::new(flat, f, u8::try_from(i).expect("fits"))
        })
        .collect();

    let mut reg = Registry::new();
    let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&functions);

    for _ in 0..50 {
        reg.adjust_trim(&mut channels, func, 1.0);
        reg.adjust_trim(&mut channels, func, -1.0);
    }

    assert_eq!(
        channels[2].config.servo_trim, 1500,
        "a channel with no travel must be left alone"
    );
    assert_eq!(
        reg.trimmed_mask(),
        0,
        "and must not be recorded as needing a save"
    );

    // Inverted, not merely flat. Equal endpoints are harmless without the
    // guard — the division gives NaN and both comparisons fail — but max
    // below min underflows the unsigned subtraction, which here is a panic
    // where upstream would carry on with a negative divisor.
    let inverted = ServoChannel::angle(1600, 1500, 1500, 4500);
    let mut backwards: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let f = if i == 2 { func } else { Function(0) };
            OutputChannel::new(inverted, f, u8::try_from(i).expect("fits"))
        })
        .collect();
    let mut reg2 = Registry::new();
    let fns: Vec<Function> = backwards.iter().map(|c| c.function).collect();
    reg2.update_aux_servo_function(&fns);

    reg2.adjust_trim(&mut backwards, func, 1.0);
    reg2.adjust_trim(&mut backwards, func, -1.0);
    assert_eq!(
        backwards[2].config.servo_trim, 1500,
        "an inverted channel must be skipped, not underflowed"
    );
}

/// A `v` small enough to be zero, but strictly positive.
///
/// `is_zero` compares against an epsilon while the sign tests below it compare
/// against literal zero, so the guard is not redundant with them: without it a
/// value of 1e-12 trims exactly as a value of 1.0 does, because the magnitude
/// is never read. The recorded sequence uses 0.0 for its no-op stretch and so
/// cannot tell the two apart.
#[test]
fn a_negligible_but_nonzero_adjustment_does_nothing() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let func = Function(3);
    let config = ServoChannel::angle(1000, 1500, 2000, 4500);
    let mut channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
        .map(|i| {
            let f = if i == 5 { func } else { Function(0) };
            OutputChannel::new(config, f, u8::try_from(i).expect("fits"))
        })
        .collect();

    let mut reg = Registry::new();
    let fns: Vec<Function> = channels.iter().map(|c| c.function).collect();
    reg.update_aux_servo_function(&fns);

    for _ in 0..40 {
        reg.adjust_trim(&mut channels, func, 1e-12);
    }
    assert_eq!(
        channels[5].config.servo_trim, 1500,
        "a negligible adjustment must not move the trim; the magnitude is \
         never read, so without the zero guard this trims as fast as full input"
    );
    assert_eq!(reg.trimmed_mask(), 0, "and records nothing to save");

    // A value just outside the epsilon does move it, so the test is not
    // passing because nothing ever trims.
    reg.adjust_trim(&mut channels, func, 0.01);
    assert_eq!(
        channels[5].config.servo_trim, 1501,
        "a real adjustment must still work"
    );
}

/// The four function-scoped operations that had no test at all.
///
/// Found by mutation testing: replacing each body with `()` left every test
/// passing. Each is checked for what it does, that it visits *every* matching
/// channel rather than the first, and that it declines an unassigned function.
#[test]
fn the_remaining_function_operations_do_something() {
    use ap_servo::output_channel::OutputChannel;
    use ap_servo::Limit;
    use ap_servo::{ServoChannel, NUM_SERVO_CHANNELS};

    let func = Function(19);
    let other = Function(21);
    let config = ServoChannel::angle(1000, 1500, 2000, 4500);

    let build = || {
        let channels: Vec<OutputChannel> = (0..NUM_SERVO_CHANNELS)
            .map(|i| {
                let f = if i == 2 || i == 5 { func } else { Function(0) };
                OutputChannel::new(config, f, u8::try_from(i).expect("fits"))
            })
            .collect();
        let mut reg = Registry::new();
        let functions: Vec<Function> = channels.iter().map(|c| c.function).collect();
        reg.update_aux_servo_function(&functions);
        (reg, channels)
    };

    // set_trim_to_servo_out_for: trim follows the current output, on both.
    {
        let (reg, mut channels) = build();
        channels[2].set_output_pwm(1750, true);
        channels[5].set_output_pwm(1250, true);
        reg.set_trim_to_servo_out_for(&mut channels, func);
        assert_eq!(channels[2].config.servo_trim, 1750);
        assert_eq!(
            channels[5].config.servo_trim, 1250,
            "the second matching channel must be visited too"
        );

        // An unassigned function changes nothing.
        let before = channels[2].config.servo_trim;
        reg.set_trim_to_servo_out_for(&mut channels, other);
        assert_eq!(channels[2].config.servo_trim, before);
    }

    // set_output_norm: drives both channels, and the value round-trips.
    {
        let (reg, mut channels) = build();
        reg.set_output_norm(&mut channels, func, 0.5);
        let a = channels[2].output_pwm();
        let b = channels[5].output_pwm();
        assert_ne!(a, 0, "set_output_norm must write a width");
        assert_eq!(a, b, "both matching channels must be driven");
        assert!(
            (channels[2].output_norm() - 0.5).abs() < 1e-3,
            "0.5 should read back as 0.5, got {}",
            channels[2].output_norm()
        );

        reg.set_output_norm(&mut channels, other, -0.5);
        assert_eq!(
            channels[2].output_pwm(),
            a,
            "an unassigned function must change nothing"
        );
    }

    // for_each_failsafe_target: visits every matching channel, resolving the
    // limit per channel when one is given.
    {
        let (reg, channels) = build();
        let mut seen: Vec<(u8, u16)> = Vec::new();
        reg.for_each_failsafe_target(&channels, func, None, 1666, |ch, pwm| {
            seen.push((ch, pwm));
        });
        assert_eq!(
            seen,
            vec![(2, 1666), (5, 1666)],
            "explicit width, both channels"
        );

        seen.clear();
        reg.for_each_failsafe_target(&channels, func, Some(Limit::Max), 0, |ch, pwm| {
            seen.push((ch, pwm));
        });
        assert_eq!(
            seen,
            vec![(2, 2000), (5, 2000)],
            "limit resolved per channel"
        );

        seen.clear();
        reg.for_each_failsafe_target(&channels, other, None, 1500, |ch, pwm| {
            seen.push((ch, pwm));
        });
        assert!(seen.is_empty(), "an unassigned function must visit nothing");
    }

    // for_each_non_motor_trim: every channel that is not a motor.
    {
        let (_, mut channels) = build();
        channels[7].function = Function::motor(0);
        channels[8].function = Function::motor(3);

        let mut seen: Vec<u8> = Vec::new();
        Registry::for_each_non_motor_trim(&channels, |ch, pwm| {
            assert_eq!(pwm, 1500, "each channel reports its own trim");
            seen.push(ch);
        });

        assert!(
            !seen.contains(&7) && !seen.contains(&8),
            "motors are excluded"
        );
        assert!(
            seen.contains(&2) && seen.contains(&5),
            "non-motors are included"
        );
        assert_eq!(
            seen.len(),
            NUM_SERVO_CHANNELS - 2,
            "every other channel should have been visited"
        );
    }
}
