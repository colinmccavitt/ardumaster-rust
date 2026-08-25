//! Log-replay test for the ported yaw sideslip damper (ADR-0008).
//!
//! The damper is inert under stock SITL parameters: `YAW2SRV_DAMP` is zero and
//! `get_servo_out` returns immediately when the damping gain is below 0.0001.
//! The reference flight is therefore flown with the damper enabled, by
//! `patches/enable_yaw_damper.py`, using ArduPilot's own recommended starting
//! values — the flight still has to pass upstream's autotest with them.
//!
//! # The integrator is mutated from outside the control path
//!
//! `AP_YawController` exposes `reset_I()` and `decay_I()`, and the vehicle
//! calls them on mode changes, on disarm, and from the quadplane code. So
//! `_integrator` can change between two consecutive `get_servo_out` calls
//! without either call causing it, and a replay that carries its own
//! integrator forward is not reproducing upstream — it is reproducing a
//! counterfactual in which nobody reset anything.
//!
//! That is not a guess. An independent transcription of upstream's C++ into
//! this test, using glibc rather than `libm` so the math library was identical
//! to the reference build, still failed to track the recorded integrator —
//! worse than the port did — while matching the D term exactly. Two
//! independent implementations disagreeing the same way, on the one quantity
//! the vehicle can reach around them, is the vehicle reaching around them. The
//! clearest single sample: three seconds after a logging gap upstream records
//! `I` as exactly zero, which the arithmetic alone cannot produce.
//!
//! So the replay carries the high-pass forward and reseeds the integrator on
//! every step. Those are the right treatments for two different kinds of
//! state: the high-pass is private to the controller and its two variables are
//! not logged, so it has to be evolved and warmed up; the integrator is public
//! and externally mutable, so carrying it forward would be reproducing a
//! flight nobody flew.
//!
//! With both handled, every reported quantity is comparable — the D term
//! proves the high-pass evolved identically, and `I` and the rudder output
//! follow from that plus the seeded integrator.
//!
//! Splitting this into a seeded second pass does not work, for a reason worth
//! recording: `_last_rate_hp_in` is neither logged nor recoverable from the
//! recorded terms, so a step started cold has the wrong high-pass output and
//! therefore the wrong D and the wrong output — off by 166 centidegrees at
//! worst, when measured.

#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]

use ap_control::{SideslipInputs, YawController, YawGains};
use ap_pid::PidGains;
use ap_replay::{Comparison, Fixture, Params, Row};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn controller_from_params(p: &Params) -> YawController {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "an int16 parameter round-tripped through the log as a float"
    )]
    let gains = YawGains {
        k_a: p.f32("YAW2SRV_SLIP"),
        k_i: p.f32("YAW2SRV_INT"),
        k_d: p.f32("YAW2SRV_DAMP"),
        k_ff: p.f32("YAW2SRV_RLL"),
        imax: p.f32("YAW2SRV_IMAX") as i16,
    };
    // The rate PID is not on this path; upstream's constructor defaults are
    // used so the controller is fully formed.
    let pid = PidGains {
        p: 0.04,
        i: 0.15,
        d: 0.0,
        ff: 0.15,
        dff: 0.0,
        imax: 0.666,
        pdmax: 0.0,
        filt_t_hz: 3.0,
        filt_e_hz: 0.0,
        filt_d_hz: 12.0,
        srmax: 150.0,
        srtau: 1.0,
    };
    YawController::new(gains, pid)
}

fn inputs(row: &Row, airspeed_min: i16, airspeed_max: i16) -> SideslipInputs {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "millis since boot, logged as a float but integral"
    )]
    SideslipInputs {
        scaler: row.input("sc") as f32,
        disable_integrator: row.input("di") != 0.0,
        now_ms: row.input("ms") as u32,
        roll_rad: row.input("rr") as f32,
        airspeed_eas: Some(row.input("as") as f32),
        airspeed_min,
        airspeed_max,
        yaw_rate_rad: row.input("yr") as f32,
        accel_y: row.input("ay") as f32,
        accel_bias_y: row.input("ab") as f32,
    }
}

#[test]
fn yaw_damper_replay_against_upstream_flight() {
    let dir = fixtures_dir();
    let fx_path = dir.join("yaw_replay.csv");
    let pm_path = dir.join("yaw_steer_replay_params.csv");
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
    assert!(
        params.f32("YAW2SRV_DAMP") > 0.0001,
        "the reference flight had the damper disabled, so this replay would \
         only be checking that zero equals zero"
    );

    #[allow(
        clippy::cast_possible_truncation,
        reason = "int16 parameters round-tripped through the log as floats"
    )]
    let (airspeed_min, airspeed_max) = (
        params.f32("AIRSPEED_MIN") as i16,
        params.f32("AIRSPEED_MAX") as i16,
    );

    // ---- pass 1: the high-pass, carried forward ----
    //
    // The coefficient is 0.996008 per call, so the memory is about 250 calls
    // and a seeding error decays as 0.996008^n. Two thousand steps leaves
    // under 0.04% of it.
    const WARMUP_STEPS: usize = 2000;

    let mut d_term = Comparison::new("D", 0.001);
    let mut i_term = Comparison::new("I", 0.001);
    let mut out = Comparison::new("rudder out (centidegrees)", 1.0);
    let mut c = controller_from_params(&params);
    let mut warmup_left = WARMUP_STEPS;
    let mut prev_ms: Option<u32> = None;
    let mut segments = 1usize;
    let mut hp_compared = 0usize;

    for row in &fx.rows {
        let inp = inputs(row, airspeed_min, airspeed_max);
        let hole = match prev_ms {
            None => true,
            Some(p) => inp.now_ms.saturating_sub(p) > 100,
        };
        prev_ms = Some(inp.now_ms);
        if hole {
            c = controller_from_params(&params);
            segments += 1;
            warmup_left = WARMUP_STEPS;
        }

        // The integrator is reseeded every step so that an external reset
        // cannot drag the reported D off course through the saturation guards.
        c.seed_for_replay(row.input("ig") as f32);
        let got = c.servo_out(&inp);

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }
        d_term.sample(row.time_us, row.output("D"), c.info().d.into());
        i_term.sample(row.time_us, row.output("I"), c.info().i.into());
        out.sample(row.time_us, row.output("out"), f64::from(got));
        hp_compared += 1;
    }

    println!("replayed {} damper calls", fx.len());
    println!("  {hp_compared} compared across {segments} segment(s)");
    for cmp in [&d_term, &i_term, &out] {
        println!("  {}", cmp.report());
    }

    assert!(
        hp_compared > 5000,
        "too few samples compared: {hp_compared}"
    );
    assert!(
        d_term.passed(),
        "the high-pass state did not evolve identically\n  {}",
        d_term.report()
    );
    assert!(
        i_term.passed() && out.passed(),
        "divergence from upstream\n  {}\n  {}",
        i_term.report(),
        out.report()
    );
}
