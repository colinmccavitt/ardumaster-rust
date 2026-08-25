//! Log-replay differential test for the ported TECS (ADR-0008).
//!
//! Drives the ported controller with real flight recorded from upstream
//! `Plane-4.7.0`, and compares its throttle and pitch demands against
//! upstream's own outputs at the same instants.
//!
//! This is the first verification in the port that tests the code against
//! **upstream's recorded behaviour** rather than against tests written from
//! reading upstream's source.
//!
//! # What this does and does not cover
//!
//! Covered: the whole `update_pitch_throttle` control path — energy
//! calculations, speed and height demands, underspeed detection, throttle and
//! pitch demands, and every limit and clamp along the way.
//!
//! Not covered: the 50 Hz complementary filters in `update_50hz`. Their inputs
//! are EKF position and velocity, which are not logged at 50 Hz, so their
//! outputs are injected from the log instead (`in_h`, `in_dh`, `in_sp`,
//! `in_dsp`, `in_vdlpf`). Those filters remain port-derived.
//!
//! # Configuration comes from the log, not from this file
//!
//! Every tunable is read from the flight's own `PARM` records. An earlier
//! version wrote them out by hand and got all of them wrong — `TRIM_THROTTLE`
//! as 45 when the flight used 50 put a constant 0.05 bias on every throttle
//! sample, since throttle feed-forward is seeded from `throttle_cruise * 0.01`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes a fixture whose length the test asserts; a bad index is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's logged values is what the test is for"
)]
use ap_replay::{Comparison, Fixture, Params};
use ap_tecs::params::{FlightStage, TecsParams};
use ap_tecs::tecs::{ReplaySeed, Tecs, TecsInputs};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

/// Build the controller from the reference flight's own parameters.
///
/// Each line names the upstream parameter it binds, so a rename or a default
/// change upstream shows up as a missing-parameter panic rather than as a
/// silent behavioural difference.
pub fn tecs_from_params(p: &Params) -> Tecs {
    let mut t = Tecs::new();

    t.throttle_cruise = p.f32("TRIM_THROTTLE");

    t.airspeed_limits.airspeed_min = p.f32("AIRSPEED_MIN");
    t.airspeed_limits.airspeed_max = p.f32("AIRSPEED_MAX");
    t.airspeed_limits.airspeed_cruise = p.f32("AIRSPEED_CRUISE");
    t.airspeed_limits.stall_prevention = p.bool("STALL_PREVENTION");

    t.airframe_pitch.pitch_limit_max = p.f32("PTCH_LIM_MAX_DEG");
    t.airframe_pitch.pitch_limit_min = p.f32("PTCH_LIM_MIN_DEG");

    t.params = TecsParams {
        max_climb_rate: p.f32("TECS_CLMB_MAX"),
        min_sink_rate: p.f32("TECS_SINK_MIN"),
        max_sink_rate: p.f32("TECS_SINK_MAX"),
        time_const: p.f32("TECS_TIME_CONST"),
        land_time_const: p.f32("TECS_LAND_TCONST"),
        ptch_damp: p.f32("TECS_PTCH_DAMP"),
        land_pitch_damp: p.f32("TECS_LAND_PDAMP"),
        land_damp: p.f32("TECS_LAND_DAMP"),
        thr_damp: p.f32("TECS_THR_DAMP"),
        land_throttle_damp: p.f32("TECS_LAND_TDAMP"),
        integ_gain: p.f32("TECS_INTEG_GAIN"),
        integ_gain_takeoff: p.f32("TECS_TKOFF_IGAIN"),
        integ_gain_land: p.f32("TECS_LAND_IGAIN"),
        vert_acc_lim: p.f32("TECS_VERT_ACC"),
        hgt_comp_filt_omega: p.f32("TECS_HGT_OMEGA"),
        spd_comp_filt_omega: p.f32("TECS_SPD_OMEGA"),
        roll_comp: p.f32("TECS_RLL2THR"),
        spd_weight: p.f32("TECS_SPDWEIGHT"),
        spd_weight_land: p.f32("TECS_LAND_SPDWGT"),
        land_throttle: p.f32("TECS_LAND_THR"),
        land_airspeed: p.f32("TECS_LAND_ARSPD"),
        land_sink: p.f32("TECS_LAND_SINK"),
        land_sink_rate_change: p.f32("TECS_LAND_SRC"),
        pitch_max: p.i8("TECS_PITCH_MAX"),
        pitch_min: p.i8("TECS_PITCH_MIN"),
        land_pitch_max: p.i8("TECS_LAND_PMAX"),
        max_sink_rate_approach: p.f32("TECS_APPR_SMAX"),
        options: p.i32("TECS_OPTIONS"),
        flare_holdoff_hgt: p.f32("TECS_FLARE_HGT"),
        hgt_dem_tconst: p.f32("TECS_HDEM_TCONST"),
        pitch_ff_k: p.f32("TECS_PTCH_FF_K"),
        pitch_ff_v0: p.f32("TECS_PTCH_FF_V0"),
        use_synthetic_airspeed: p.i8("TECS_SYNAIRSPEED"),
        thr_min_pct_ext_rate_lim: p.i8("TECS_THR_ERATE"),
    };

    t
}

/// Translate one fixture row into the controller's input struct.
pub fn inputs_from_row(row: &ap_replay::Row, p: &Params) -> TecsInputs {
    let stage = FlightStage::from_u8(row.input("stg") as u8).unwrap_or_else(|| {
        panic!(
            "unknown flight stage {} at t={}",
            row.input("stg"),
            row.time_us
        )
    });

    TecsInputs {
        hgt_dem_cm: row.input("hdem") as f32,
        eas_dem_cm: row.input("easd") as f32,
        flight_stage: stage,
        distance_beyond_land_wp: row.input("dbey") as f32,
        pitch_min_climbout_cd: row.input("pmin") as f32,
        throttle_nudge: row.input("thnu") as i16,
        hgt_afe: row.input("hafe") as f32,
        load_factor: row.input("ldf") as f32,
        pitch_trim_deg: row.input("ptrm") as f32,

        // injected outputs of the 50 Hz stage
        height: row.input("h") as f32,
        climb_rate: row.input("dh") as f32,
        tas_state: row.input("sp") as f32,
        vel_dot: row.input("dsp") as f32,
        vel_dot_lpf: row.input("vdlpf") as f32,
        tas_min: row.input("tasmn") as f32,
        tas_max: row.input("tasmx") as f32,
        tas_dem: row.input("tasd") as f32,
        tas_cruise: row.input("tascr") as f32,

        pitch_measured: row.input("ptchm") as f32,
        cos_roll: row.input("cosr") as f32,

        // AP_Landing-derived state, from TECK. Hard-coding these to
        // false/zero is what held the kinetic-energy weighting at 1.0 through
        // the approach, where upstream slides it to 0 as path_proportion
        // reaches 1 (TECS_LAND_SPDWGT is -1 on this airframe).
        use_airspeed: row.input("uas") != 0.0,
        gliding_requested: row.input("glid") != 0.0,
        is_flaring: row.input("flar") != 0.0,
        is_on_approach: row.input("appr") != 0.0,
        landing_pitch_cd: row.input("lpcd") as f32,
        path_proportion: row.input("prop") as f32,

        land_throttle_slewrate: 0,
        throttle_slewrate: p.i8("THR_SLEWRATE"),

        // external limits as the caller set them for this iteration
        thr_max_ext: row.input("tmxe") as f32,
        thr_min_ext: row.input("tmne") as f32,
        pitch_max_ext: row.input("pmxe") as f32,
        pitch_min_ext: row.input("pmne") as f32,
        now_ms: ap_hal::time::Millis((row.time_us / 1000) as u32),
    }
}

#[test]
fn tecs_replay_against_upstream_flight() {
    let dir = fixtures_dir();
    let fx_path = dir.join("tecs_replay.csv");
    let pm_path = dir.join("tecs_replay_params.csv");
    if !fx_path.exists() || !pm_path.exists() {
        eprintln!("skipping: fixture or parameter set not present");
        return;
    }

    let fx = Fixture::load(&fx_path).expect("fixture should load");
    let params = Params::load(&pm_path).expect("parameters should load");
    assert!(
        fx.len() > 1000,
        "expected a substantial flight, got {}",
        fx.len()
    );

    // Tolerances reflect what the port actually achieves, not a comfortable
    // margin: pitch matches upstream exactly, throttle to 0.005 (a single
    // transient at a segment start, before its integrator settles). Loose
    // bounds here would let a real regression through unnoticed.
    let mut thr = Comparison::new("throttle", 0.01);
    let mut ph = Comparison::new("pitch", 0.001);

    // Split at holes: places where the timestamp delta disagrees with
    // upstream's own _DT, meaning it ran steps that were never logged.
    let segments = split_into_segments(&fx);
    println!(
        "replaying {} rows in {} contiguous segment(s)",
        fx.len(),
        segments.len()
    );

    for (seg_no, seg) in segments.iter().enumerate() {
        let rows = &fx.rows[seg.clone()];
        println!(
            "  segment {}: {} rows, t={:.1}..{:.1}s",
            seg_no,
            rows.len(),
            rows[0].time_us as f64 / 1e6,
            rows[rows.len() - 1].time_us as f64 / 1e6
        );

        // a fresh controller per segment: it cannot inherit state from steps
        // it never replayed
        let mut tecs = tecs_from_params(&params);

        // Run the first row, then restore the state upstream recorded at that
        // instant. TECL and friends are logged at the end of the update, so
        // after this call the true state is exactly what the row holds.
        //
        // Without this the comparison would measure the starting conditions
        // rather than the update law: the replay is open loop, so an
        // integrator seeded differently holds its offset forever instead of
        // converging. Segment 1 showed a constant 26.4 offset in _integSEBdot
        // across its whole 121 seconds.
        tecs.update_pitch_throttle(
            &inputs_from_row(&rows[0], &params),
            rows[0].output("dt") as f32,
        );
        tecs.seed_for_replay(&seed_from_row(&rows[0]));

        for row in &rows[1..] {
            tecs.update_pitch_throttle(&inputs_from_row(row, &params), row.output("dt") as f32);
            thr.sample(row.time_us, row.output("th"), tecs.throttle_demand() as f64);
            ph.sample(row.time_us, row.output("ph"), tecs.pitch_demand() as f64);
        }
    }

    println!("  {}", thr.report());
    println!("  {}", ph.report());

    // A comparison that saw no samples reports passed() as true. Guard against
    // a fixture or filter change turning this into a vacuous pass.
    assert!(
        thr.compared() > 1000 && ph.compared() > 1000,
        "too few samples compared: throttle {}, pitch {}",
        thr.compared(),
        ph.compared()
    );

    // Report first, assert second: the numbers are the point even when they
    // fail, and a bare assertion would hide them.
    assert!(
        thr.passed() && ph.passed(),
        "divergence from upstream\n  {}\n  {}",
        thr.report(),
        ph.report()
    );
}

/// The parameter set must actually cover what the controller reads.
///
/// A missing parameter panics inside `tecs_from_params`; this makes that a
/// named failure rather than a confusing panic inside the replay loop.
#[test]
fn reference_parameters_cover_the_controller() {
    let path = fixtures_dir().join("tecs_replay_params.csv");
    if !path.exists() {
        eprintln!("skipping: parameter set not present");
        return;
    }
    let p = Params::load(&path).expect("parameters should load");
    assert!(
        p.len() > 500,
        "expected a full parameter set, got {}",
        p.len()
    );

    let t = tecs_from_params(&p);

    // spot-check that binding happened, rather than silently keeping defaults
    assert_eq!(t.throttle_cruise, p.f32("TRIM_THROTTLE"));
    assert_eq!(t.params.time_const, p.f32("TECS_TIME_CONST"));
    assert_eq!(t.airframe_pitch.pitch_limit_min, p.f32("PTCH_LIM_MIN_DEG"));
}

/// Split the fixture at holes in the recording.
///
/// A hole is a row whose gap from the previous row disagrees with upstream's
/// own logged `_DT`: the controller kept running and took steps that were
/// never written to the log. Those steps cannot be replayed, so the row starts
/// a new segment rather than being treated as one long step.
pub fn split_into_segments(fx: &Fixture) -> Vec<std::ops::Range<usize>> {
    const TOL_S: f64 = 1e-3;
    let mut segs = Vec::new();
    let mut start = 0usize;
    for n in 1..fx.rows.len() {
        let gap = (fx.rows[n].time_us - fx.rows[n - 1].time_us) as f64 / 1e6;
        let dt = fx.rows[n].output("dt");
        if (gap - dt).abs() > TOL_S {
            segs.push(start..n);
            start = n;
        }
    }
    segs.push(start..fx.rows.len());
    // a segment shorter than the warm-up contributes nothing to the comparison
    segs.retain(|s| s.len() > 25);
    segs
}

/// The controller state upstream recorded at this row.
///
/// Every field comes from a log message written at the end of upstream's own
/// update, so together they are the exact state it carried into the next call.
pub fn seed_from_row(row: &ap_replay::Row) -> ReplaySeed {
    ReplaySeed {
        integ_sebdot: row.output("I") as f32,
        integ_ke: row.output("KI") as f32,
        hgt_dem_lpf: row.output("hlpf") as f32,
        hgt_dem_rate_ltd: row.output("hrtl") as f32,
        hgt_dem_in_prev: row.output("hdip") as f32,
        hgt_dem: row.output("hdem") as f32,
        max_climb_scaler: row.output("mcs") as f32,
        max_sink_scaler: row.output("mss") as f32,
        post_to_hgt_offset: row.output("pto") as f32,
        last_pitch_dem: row.output("ph") as f32,
        last_throttle_dem: row.output("th") as f32,
    }
}
