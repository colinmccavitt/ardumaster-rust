//! Log-replay test for the ported steering controller (ADR-0008).
//!
//! Driven by `STCI`, logged from inside `get_steering_out_rate` — the single
//! point all three entry points funnel through. The wrappers that convert an
//! angle error or a lateral acceleration into a rate demand are covered by
//! unit tests; this covers the body they all share, against a real takeoff
//! roll and landing rollout.
//!
//! # D-018 makes a continuous replay useless, so this runs two passes
//!
//! Upstream's integrator winds to its IMAX limit while the aircraft is
//! standing still — that is exactly the defect D-018 fixes — and the port's
//! stays at zero. A continuous replay therefore parts company on the third
//! sample and can compare nothing downstream of the integrator for the rest of
//! the run. The measured divergence is the full clamp, 15.0, not a rounding
//! difference.
//!
//! So:
//!
//! 1. **Continuous.** Everything computed fresh from logged inputs each call —
//!    the demanded rate, the measured rate, P, D and FF — must agree exactly.
//!    D-018 does not touch any of them.
//! 2. **Seeded.** Each step starts from the integrator and previous output
//!    upstream actually had, and the resulting integrator must equal what
//!    upstream carried into the *next* call. This checks the integrator
//!    arithmetic — the saturation guards, the IMAX clamp, the gain conversion
//!    — on every consecutive pair where both samples are above
//!    `STEER2SRV_MINSPD`, which is everywhere the two gates agree.
//!
//! And the divergence itself is pinned: the port must integrate nothing below
//! the floor, and must match upstream above it.

#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes adjacent fixture rows inside a bounds-checked range; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_control::{SteerController, SteerGains, SteerInputs};
use ap_replay::{Comparison, Fixture, Params};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn controller_from_params(p: &Params) -> SteerController {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "an int16 parameter round-tripped through the log as a float"
    )]
    let gains = SteerGains {
        tau: p.f32("STEER2SRV_TCONST"),
        k_ff: p.f32("STEER2SRV_FF"),
        k_p: p.f32("STEER2SRV_P"),
        k_i: p.f32("STEER2SRV_I"),
        k_d: p.f32("STEER2SRV_D"),
        minspeed: p.f32("STEER2SRV_MINSPD"),
        imax: p.f32("STEER2SRV_IMAX") as i16,
        deratespeed: p.f32("STEER2SRV_DRTSPD"),
        deratefactor: p.f32("STEER2SRV_DRTFCT"),
        mindegree: p.f32("STEER2SRV_DRTMIN"),
    };
    SteerController::new(gains)
}

fn inputs(row: &ap_replay::Row) -> SteerInputs {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "millis since boot, logged as a float but integral"
    )]
    SteerInputs {
        now_ms: row.input("ms") as u32,
        groundspeed: row.input("gs") as f32,
        yaw_rate_earth_rad: row.input("yr") as f32,
    }
}

#[test]
fn steering_controller_replay_against_upstream_flight() {
    let dir = fixtures_dir();
    let fx_path = dir.join("steer_replay.csv");
    let pm_path = dir.join("yaw_steer_replay_params.csv");
    if !fx_path.exists() || !pm_path.exists() {
        eprintln!("skipping: fixture or parameter set not present");
        return;
    }
    let fx = Fixture::load(&fx_path).expect("fixture should load");
    let params = Params::load(&pm_path).expect("parameters should load");
    assert!(
        fx.len() > 100,
        "expected a real ground run, got {}",
        fx.len()
    );

    let minspeed = params.f32("STEER2SRV_MINSPD");

    // ---- pass 1: the terms D-018 does not touch, replayed continuously ----
    let mut tgt = Comparison::new("demanded rate (deg/s)", 0.001);
    let mut act = Comparison::new("measured rate (deg/s)", 0.001);
    let mut p_term = Comparison::new("P", 0.001);
    let mut d_term = Comparison::new("D", 0.001);
    let mut ff_term = Comparison::new("FF", 0.001);

    let mut c = controller_from_params(&params);
    let mut below_floor = 0usize;
    let mut port_integrated_below_floor = 0usize;

    for row in &fx.rows {
        let inp = inputs(row);
        c.set_reverse(row.input("rv") != 0.0);
        let slow = inp.groundspeed < minspeed;

        c.steering_out_rate(row.input("dr") as f32, &inp);
        let info = c.info();

        if slow {
            below_floor += 1;
            // Below the floor the port takes upstream's `else` branch, which
            // zeroes the integrator. So the property is not "unchanged" -- a
            // wound integrator legitimately drops to zero on the way down --
            // it is that no winding survives the call.
            if info.i != 0.0 {
                port_integrated_below_floor += 1;
            }
        }

        tgt.sample(row.time_us, row.output("tgt"), info.target.into());
        act.sample(row.time_us, row.output("act"), info.actual.into());
        p_term.sample(row.time_us, row.output("P"), info.p.into());
        d_term.sample(row.time_us, row.output("D"), info.d.into());
        ff_term.sample(row.time_us, row.output("F"), info.ff.into());
    }

    // ---- pass 2: the integrator arithmetic, seeded from upstream ----
    let mut integ = Comparison::new("integrator after one seeded step", 1e-6);
    let mut out = Comparison::new("steering out (centidegrees)", 1.0);
    let mut steps = 0usize;

    for i in 1..fx.rows.len() {
        let prev = &fx.rows[i - 1];
        let row = &fx.rows[i];
        let inp = inputs(row);

        // Only where the two gates agree: below the floor D-018 is supposed to
        // make them differ, and checking it there would be checking the
        // divergence rather than the arithmetic.
        if inp.groundspeed < minspeed {
            continue;
        }
        // A gap means upstream ran a call that was never recorded, so the
        // previous row is not the previous call.
        if row.time_us - prev.time_us > 100_000 {
            continue;
        }
        // The integrator leaving this call is the next call's recorded `ig`.
        let Some(next) = fx.rows.get(i + 1) else {
            break;
        };
        if next.time_us - row.time_us > 100_000 {
            continue;
        }

        let mut c = controller_from_params(&params);
        c.set_reverse(row.input("rv") != 0.0);
        // `last_out` is the previous call's four terms summed. Upstream saves
        // it before the output limiter, so the logged terms rebuild it exactly.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "millis since boot, logged as a float but integral"
        )]
        let last_ms = prev.input("ms") as u32;
        let last_out =
            (prev.output("P") + prev.output("I") + prev.output("D") + prev.output("F")) as f32;
        c.seed_for_replay(row.input("ig") as f32, last_out, last_ms);

        let got = c.steering_out_rate(row.input("dr") as f32, &inp);
        integ.sample(next.time_us, next.input("ig"), c.info().i.into());
        out.sample(row.time_us, row.output("out"), f64::from(got));
        steps += 1;
    }

    println!("replayed {} steering calls", fx.len());
    println!("  {below_floor} below STEER2SRV_MINSPD ({minspeed} m/s)");
    for cmp in [&tgt, &act, &p_term, &d_term, &ff_term] {
        println!("  {}", cmp.report());
    }
    println!("  {steps} seeded integrator step(s)");
    for cmp in [&integ, &out] {
        println!("  {}", cmp.report());
    }

    assert!(
        tgt.passed() && act.passed() && p_term.passed() && d_term.passed() && ff_term.passed(),
        "the terms D-018 does not touch must agree exactly\n  {}\n  {}\n  {}\n  {}\n  {}",
        tgt.report(),
        act.report(),
        p_term.report(),
        d_term.report(),
        ff_term.report()
    );
    assert!(
        steps > 500,
        "too few seeded steps to mean anything: {steps}"
    );
    assert!(
        integ.passed() && out.passed(),
        "the integrator arithmetic diverges above the floor, where D-018 \
         should change nothing\n  {}\n  {}",
        integ.report(),
        out.report()
    );

    // D-018 itself
    assert!(
        below_floor > 0,
        "the flight never went below STEER2SRV_MINSPD, so this run does not \
         exercise D-018 and proves nothing about it"
    );
    assert_eq!(
        port_integrated_below_floor, 0,
        "D-018: the port must hold the integrator at zero below \
         STEER2SRV_MINSPD, and {below_floor} sample(s) were below it"
    );
}
