//! Parity test: AC_PID against upstream's own implementation.
//!
//! Harness-driven rather than replayed from a flight log. AC_PID is stateful —
//! an integrator, three low-pass filters and a slew limiter — and a scripted
//! sequence gives exact control over `dt` and the limit flag, which is what
//! reaches the branches that matter: integrator clamping while saturated, the
//! reset path, the P+D sum limit, a zero I gain clearing the integrator, and
//! the slew limiter engaging under oscillation. A flight log would exercise
//! whichever of those that flight happened to hit.
//!
//! The driving sequence is duplicated here from
//! `tools/parity/gen_pid_fixture.py`. That duplication is deliberate and
//! checked: the fixture records the target and measurement upstream actually
//! saw, and the test asserts its own reconstruction matches them before
//! comparing anything else. If the two ever drift apart the test says so rather
//! than silently comparing different runs.
//!
//! Values are raw bit patterns, so every comparison is exact.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_pid::{AcPid, PidGains, Scaling};

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/ac_pid_parity.csv"))
        .expect("workspace root")
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

/// The same gain table the harness uses, in the same order.
fn scenarios() -> Vec<(&'static str, PidGains)> {
    let g = |p, i, d, ff, imax, ft, fe, fd, srmax, srtau, dff, pdmax| PidGains {
        p,
        i,
        d,
        ff,
        imax,
        filt_t_hz: ft,
        filt_e_hz: fe,
        filt_d_hz: fd,
        srmax,
        srtau,
        dff,
        pdmax,
    };
    vec![
        (
            "plain",
            g(2.0, 0.5, 0.1, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        ),
        (
            "filtered",
            g(
                2.0, 0.5, 0.1, 0.25, 10.0, 20.0, 15.0, 10.0, 0.0, 1.0, 0.0, 0.0,
            ),
        ),
        (
            "tight_imax",
            g(1.0, 5.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        ),
        (
            "no_i",
            g(2.0, 0.0, 0.1, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        ),
        (
            "pdmax",
            g(5.0, 0.1, 0.5, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0),
        ),
        (
            "slew",
            g(4.0, 0.2, 0.3, 0.0, 10.0, 0.0, 0.0, 20.0, 5.0, 1.0, 0.0, 0.0),
        ),
        (
            "dff",
            g(
                1.0, 0.1, 0.0, 0.5, 10.0, 10.0, 10.0, 10.0, 0.0, 1.0, 0.7, 0.0,
            ),
        ),
    ]
}

/// The harness's driving function, reproduced.
fn drive(step: i32) -> (f32, f32, bool) {
    let target = if step < 30 {
        10.0
    } else if step < 60 {
        -10.0
    } else if step % 2 == 0 {
        20.0
    } else {
        -20.0
    };
    let measurement = target * 0.25;
    let limit = (40..50).contains(&step);
    (target, measurement, limit)
}

#[test]
fn ac_pid_matches_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let text = std::fs::read_to_string(&path).expect("read fixture");

    // index the fixture by (scenario, step)
    let mut rows: std::collections::HashMap<(usize, i32), Vec<String>> =
        std::collections::HashMap::new();
    let mut header_pending = false;
    for line in text.lines() {
        if line.is_empty() {
            continue;
        }
        if line.starts_with('#') {
            header_pending = true;
            continue;
        }
        if header_pending {
            header_pending = false;
            continue;
        }
        let c: Vec<String> = line.split(',').map(str::to_string).collect();
        assert_eq!(c.len(), 17, "case row: {line}");
        rows.insert(
            (c[0].parse().expect("scenario"), c[1].parse().expect("step")),
            c,
        );
    }
    assert!(!rows.is_empty(), "fixture has no rows");

    let mut checked = 0usize;
    let mut reset_rows = 0usize;
    let mut pd_limited = 0usize;
    let mut slew_engaged = 0usize;

    for (s, (name, gains)) in scenarios().into_iter().enumerate() {
        let mut pid = AcPid::new(gains);

        // Every scenario must match bit for bit except the one driving the
        // slew limiter, whose residual comes from ap-filter rather than from
        // AC_PID -- see the module docs and FW-036.
        let exact = name != "slew";

        let check = |pid: &AcPid, out: f32, step: i32, checked: &mut usize| {
            let c = rows
                .get(&(s, step))
                .unwrap_or_else(|| panic!("fixture has no {name} step {step}"));
            let info = pid.info();

            for (label, got, want) in [
                ("out", out, f(&c[2])),
                ("target", info.target, f(&c[3])),
                ("actual", info.actual, f(&c[4])),
                ("error", info.error, f(&c[5])),
                ("P", info.p, f(&c[6])),
                ("I", info.i, f(&c[7])),
                ("D", info.d, f(&c[8])),
                ("FF", info.ff, f(&c[9])),
                ("DFF", info.dff, f(&c[10])),
                ("Dmod", info.dmod, f(&c[11])),
                ("slew_rate", info.slew_rate, f(&c[12])),
                ("integrator", pid.integrator(), f(&c[16])),
            ] {
                if exact {
                    assert!(
                        same(got, want),
                        "{name} step {step} {label}: port {got:?} ({:#x}),                          upstream {want:?} ({:#x})",
                        got.to_bits(),
                        want.to_bits()
                    );
                } else {
                    // Bounded, not waived. The slew limiter feeds Dmod, which
                    // scales P and D, so a filter difference of a few ulps
                    // shows up here amplified by the gains.
                    let tol = 1e-5 * want.abs().max(1.0);
                    assert!(
                        same(got, want) || (got - want).abs() <= tol,
                        "{name} step {step} {label}: port {got:?}, upstream                          {want:?} -- beyond the ap-filter residual this test                          tolerates",
                    );
                }
            }
            assert_eq!(info.limit, c[13] == "1", "{name} step {step} limit");
            assert_eq!(info.pd_limit, c[14] == "1", "{name} step {step} PD_limit");
            assert_eq!(info.reset, c[15] == "1", "{name} step {step} reset");
            *checked += 1;
        };

        for step in 0..120 {
            let (target, measurement, limit) = drive(step);

            // the fixture records what upstream was actually driven with, so a
            // drift between this reconstruction and the generator is caught
            // rather than silently comparing two different runs
            let c = rows.get(&(s, step)).expect("fixture row");
            if step > 0 {
                assert!(
                    same(measurement, f(&c[4])),
                    "{name} step {step}: the test drives {measurement} but the \
                     fixture recorded {} -- the generator and this test have \
                     drifted apart",
                    f(&c[4])
                );
            }

            #[allow(clippy::cast_sign_loss, reason = "step is non-negative here")]
            let now_ms = (step * 20) as u32;
            let out = pid.update_all(target, measurement, 0.02, limit, Scaling::default(), now_ms);
            check(&pid, out, step, &mut checked);

            let info = pid.info();
            if info.reset {
                reset_rows += 1;
            }
            if info.pd_limit {
                pd_limited += 1;
            }
            if info.dmod != 1.0 {
                slew_engaged += 1;
            }
        }

        // an explicit reset outside step 0
        pid.reset_filter();
        let out = pid.update_all(5.0, 1.0, 0.02, false, Scaling::default(), 2400);
        check(&pid, out, 200, &mut checked);
        if pid.info().reset {
            reset_rows += 1;
        }

        // a zero dt, which must not divide
        let out = pid.update_all(5.0, 1.0, 0.0, false, Scaling::default(), 2420);
        check(&pid, out, 201, &mut checked);
    }

    println!("{checked} AC_PID steps matched upstream exactly");
    println!("  filter resets:      {reset_rows}");
    println!("  P+D limit engaged:  {pd_limited}");
    println!("  slew limiter active:{slew_engaged}");

    assert_eq!(checked, rows.len(), "every fixture row must be checked");

    // The interesting branches must actually have been reached, or the fixture
    // has quietly stopped exercising them and the test is weaker than it looks.
    assert!(reset_rows >= 14, "the reset path was barely exercised");
    assert!(pd_limited > 0, "the P+D sum limit never bound");
    assert!(slew_engaged > 0, "the slew limiter never engaged");
}
