//! Log-replay test for the ported pitch controller (ADR-0008).
//!
//! `AP_PitchController` cannot be driven by a linked harness the way `AC_PID`
//! was: the rate loop and the turn-coordination offset both reach into
//! `AP::ahrs()`, and standing that up outside a vehicle means linking most of
//! the firmware. So this replays a real flight instead, driven by the inputs
//! upstream actually saw — recorded by the `PCTI`/`PCTO` reference-build patch.
//!
//! # The replay runs continuously, and the integrator is the state check
//!
//! A row cannot be replayed in isolation. The controller carries filter state —
//! the PID's target, error and derivative low-passes — which is not logged, so
//! constructing a fresh controller per row makes every call take the reset path
//! and the D term comes out zero every time. The steps have to run in order.
//!
//! `PCTI` records the integrator as it stood entering each call, and the replay
//! never reseeds it inside a segment. Agreeing with that value across thousands
//! of consecutive steps is what proves the state evolved identically, rather
//! than merely producing the right output from a seeded one.
//!
//! # Configuration comes from the flight
//!
//! Gains are read from the log's own `PARM` records. Every parameter the TECS
//! work wrote out by hand turned out to be wrong.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]

use ap_control::{PitchController, PitchInputs, RateGains};
use ap_pid::PidGains;
use ap_replay::{Comparison, Fixture, Params};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

/// Build the controller from the reference flight's own parameters.
fn controller_from_params(p: &Params) -> PitchController {
    let pid = PidGains {
        p: p.f32("PTCH_RATE_P"),
        i: p.f32("PTCH_RATE_I"),
        d: p.f32("PTCH_RATE_D"),
        ff: p.f32("PTCH_RATE_FF"),
        dff: 0.0,
        imax: p.f32("PTCH_RATE_IMAX"),
        pdmax: 0.0,
        filt_t_hz: p.f32("PTCH_RATE_FLTT"),
        filt_e_hz: p.f32("PTCH_RATE_FLTE"),
        filt_d_hz: p.f32("PTCH_RATE_FLTD"),
        srmax: p.f32("PTCH_RATE_SMAX"),
        srtau: 1.0,
    };
    let gains = RateGains {
        tau: p.f32("PTCH2SRV_TCONST"),
        rmax_pos: p.f32("PTCH2SRV_RMAX_UP"),
        rmax_neg: p.f32("PTCH2SRV_RMAX_DN"),
    };
    PitchController::new(pid, gains, p.f32("PTCH2SRV_RLL"))
}

#[test]
fn pitch_controller_replay_against_upstream_flight() {
    let dir = fixtures_dir();
    let fx_path = dir.join("pitch_replay.csv");
    let pm_path = dir.join("pitch_replay_params.csv");
    if !fx_path.exists() || !pm_path.exists() {
        eprintln!("skipping: fixture or parameter set not present");
        return;
    }
    let fx = Fixture::load(&fx_path).expect("fixture should load");
    let params = Params::load(&pm_path).expect("parameters should load");
    assert!(
        fx.len() > 2000,
        "expected a substantial flight, got {}",
        fx.len()
    );

    // AP_Int16 upstream; the log records them as floats.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "int16 parameters round-tripped through the log as floats"
    )]
    let (airspeed_min, airspeed_max) = (
        params.f32("AIRSPEED_MIN") as i16,
        params.f32("AIRSPEED_MAX") as i16,
    );
    let roll_limit_deg = params.f32("ROLL_LIMIT_DEG");

    let mut out = Comparison::new("elevator out (centidegrees)", 1.0);
    let mut tgt = Comparison::new("demanded rate (deg/s)", 0.01);
    let mut act = Comparison::new("measured rate (deg/s)", 0.001);
    let mut p_term = Comparison::new("P", 0.01);
    let mut i_term = Comparison::new("I", 0.01);
    let mut d_term = Comparison::new("D", 0.01);
    let mut ff_term = Comparison::new("FF", 0.01);
    let mut integ = Comparison::new("integrator carried into the call", 0.001);

    /// Steps to skip after a reseed, while the unlogged filter state converges.
    const WARMUP_STEPS: usize = 50;

    let mut c = controller_from_params(&params);
    let mut prev_us: Option<u64> = None;
    let mut segments = 1usize;
    let mut warmup_left = WARMUP_STEPS;
    let mut compared = 0usize;

    for row in &fx.rows {
        let dt = row.input("dt") as f32;

        // A gap of more than about one loop period means upstream ran calls
        // that were never recorded, and the filters cannot be carried across
        // it. The threshold is generous because the scheduler jitters by up to
        // a millisecond: an exact comparison treated jittery timestamps as
        // holes and threw away half the flight to warm-up.
        let hole = match prev_us {
            None => true,
            Some(p) => (row.time_us - p) as f64 * 1e-6 > f64::from(dt) * 1.5,
        };
        prev_us = Some(row.time_us);

        if hole {
            c = controller_from_params(&params);
            c.controller.rate_pid.set_integrator(row.input("ig") as f32);
            segments += 1;
            warmup_left = WARMUP_STEPS;
        }

        #[allow(
            clippy::cast_possible_truncation,
            reason = "attitudes in centidegrees, and milliseconds since boot, \
are logged as floats but are integral"
        )]
        let inp = PitchInputs {
            scaler: row.input("sc") as f32,
            disable_integrator: row.input("di") != 0.0,
            ground_mode: row.input("gm") != 0.0,
            pitch_rate_rad: row.input("gy") as f32,
            airspeed_eas: Some(row.input("as") as f32),
            airspeed_min,
            airspeed_max,
            roll_limit_deg,
            roll_rad: row.input("rr") as f32,
            pitch_rad: row.input("pr") as f32,
            roll_sensor_cd: row.input("rs") as i32,
            pitch_sensor_cd: row.input("ps") as i32,
            eas2tas: row.input("e2t") as f32,
            dt,
            now_ms: (row.time_us / 1000) as u32,
        };

        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged angle error is an int32 widened to float"
        )]
        let angle_err_cd = row.input("ae") as i32;

        // PCTI records the integrator as it stood ENTERING the call, so it must
        // be sampled before the port integrates, not after.
        let integrator_before = c.controller.rate_pid.integrator();

        let got = c.servo_out(angle_err_cd, &inp);
        let info = c.controller.info();

        // The filters need a few steps to converge on upstream's state after a
        // reseed, since their history is not recorded.
        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }

        out.sample(row.time_us, row.output("out"), got.into());
        tgt.sample(row.time_us, row.output("tgt"), info.target.into());
        act.sample(row.time_us, row.output("act"), info.actual.into());
        p_term.sample(row.time_us, row.output("P"), info.p.into());
        i_term.sample(row.time_us, row.output("I"), info.i.into());
        d_term.sample(row.time_us, row.output("D"), info.d.into());
        ff_term.sample(row.time_us, row.output("F"), info.ff.into());
        integ.sample(row.time_us, row.input("ig"), integrator_before.into());

        compared += 1;
    }

    println!("replayed {} control calls", fx.len());
    println!("  {compared} compared across {segments} contiguous segment(s)");
    for cmp in [
        &out, &tgt, &act, &p_term, &i_term, &d_term, &ff_term, &integ,
    ] {
        println!("  {}", cmp.report());
    }

    assert!(compared > 8000, "too few samples compared: {compared}");
    assert!(
        integ.passed(),
        "the integrator diverged, so the state did not evolve identically\n  {}",
        integ.report()
    );
    assert!(
        out.passed() && tgt.passed() && act.passed(),
        "divergence from upstream\n  {}\n  {}\n  {}",
        out.report(),
        tgt.report(),
        act.report()
    );
    assert!(
        p_term.passed() && i_term.passed() && d_term.passed() && ff_term.passed(),
        "PID contributions diverge\n  {}\n  {}\n  {}\n  {}",
        p_term.report(),
        i_term.report(),
        d_term.report(),
        ff_term.report()
    );
}
