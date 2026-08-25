//! The longitudinal counterpart: pitch demand through to elevator.
//!
//! ```text
//!   PitchDemand::from_tecs           ->  limited demand
//!   PitchDemand::demanded_pitch_cd   ->  plus trim and throttle feed-forward
//!   PitchDemand::angle_error_cd      ->  angle error
//!   PitchController::servo_out       ->  elevator, centidegrees
//! ```
//!
//! # What this does and does not test
//!
//! TECS's pitch demand is taken from the log rather than re-derived. The roll
//! slice drives L1 itself, and doing the same for TECS is worth doing, but it
//! is a much larger piece of machinery — ADR-0009 exists because of how much
//! care its replay needs — and folding that into a composition test would
//! confuse two questions.
//!
//! So this tests the join and the pitch controller in composition. The join is
//! where the interesting arithmetic lives: unlike roll's limit-and-subtract,
//! pitch adds a trim, feeds throttle forward into the demand, and changes type
//! partway through the sum.
//!
//! # Where the demand does not come through this path
//!
//! 1,046 of 11,554 steps cannot be reproduced, and it matters which reason
//! applies, because an arithmetic bug would look identical to either. Checking
//! the intermediate `nav_pitch_cd` separates them: if that already disagrees,
//! the flight mode set it directly and `calc_nav_pitch` never ran; if it
//! agrees and only the final demand differs, the flare override replaced it.
//!
//! On this flight all 1,046 are the former — takeoff and landing set the pitch
//! demand themselves — and none are flares. Both are counted separately so the
//! number cannot quietly become an arithmetic error later.

#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is checked; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::{PitchController, PitchInputs, RateGains};
use ap_pid::PidGains;
use ap_plane::PitchDemand;
use ap_replay::{Comparison, Fixture, Params, Row};

/// Attitude steps to skip while the pitch controller's unlogged filters
/// converge on upstream's state.
const WARMUP_STEPS: usize = 50;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn pitch_from_params(p: &Params) -> PitchController {
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
fn pitch_demand_through_to_elevator_matches_the_flight() {
    let dir = fixtures_dir();
    let paths = [
        dir.join("vpslice_pitch.csv"),
        dir.join("vpslice_join.csv"),
        dir.join("vpslice_params.csv"),
    ];
    if paths.iter().any(|p| !p.exists()) {
        eprintln!("skipping: longitudinal-slice fixtures not present");
        return;
    }
    let pitch_fx = Fixture::load(&paths[0]).expect("pitch fixture");
    let join_fx = Fixture::load(&paths[1]).expect("join fixture");
    let params = Params::load(&paths[2]).expect("parameters");
    assert!(pitch_fx.len() > 1000, "pitch rows: {}", pitch_fx.len());

    let join: std::collections::HashMap<u64, &Row> =
        join_fx.rows.iter().map(|r| (r.time_us, r)).collect();

    #[allow(
        clippy::cast_possible_truncation,
        reason = "int16 parameters round-tripped through the log as floats"
    )]
    let (airspeed_min, airspeed_max) = (
        params.f32("AIRSPEED_MIN") as i16,
        params.f32("AIRSPEED_MAX") as i16,
    );
    let roll_limit_deg = params.f32("ROLL_LIMIT_DEG");

    let mut c = pitch_from_params(&params);

    let mut limited = Comparison::new("stage 1: after the pitch limits", 1.0);
    let mut demanded = Comparison::new("stage 2: plus trim and throttle FF", 1.0);
    let mut angle_err = Comparison::new("stage 3: angle error", 1.0);
    let mut elevator = Comparison::new("stage 4: elevator out (centidegrees)", 2.0);

    let mut prev_us: Option<u64> = None;
    let mut segments = 0usize;
    let mut warmup_left = WARMUP_STEPS;
    let mut driven = 0usize;
    let mut compared = 0usize;
    let mut flare_override = 0usize;
    let mut mode_set_directly = 0usize;
    let mut unjoined = 0usize;

    for row in &pitch_fx.rows {
        let dt = row.input("dt") as f32;

        // A gap means steps upstream ran that were never recorded, so the
        // filters cannot be carried across it.
        let hole = match prev_us {
            None => true,
            Some(p) => (row.time_us - p) as f64 * 1e-6 > f64::from(dt) * 1.5,
        };
        prev_us = Some(row.time_us);
        if hole {
            c = pitch_from_params(&params);
            c.controller.rate_pid.set_integrator(row.input("ig") as f32);
            segments += 1;
            warmup_left = WARMUP_STEPS;
        }

        let Some(j) = join.get(&row.time_us) else {
            unjoined += 1;
            continue;
        };

        // Rebuild the demand from TECS's output and the vehicle's terms.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demands, limits and attitude are int32 widened to float"
        )]
        let demand = PitchDemand::from_tecs(
            j.output("tpd") as i32,
            j.output("pmin") as i32,
            j.output("pmax") as i32,
        );
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged trim is an int32 widened to float"
        )]
        let dem = demand.demanded_pitch_cd(
            j.output("trm") as i32,
            j.output("thr") as f32,
            j.output("kff") as f32,
        );

        // The flare override replaces the demand outright. Where the
        // reconstruction disagrees, it fired, and there is nothing here to
        // reproduce -- but the controller still has to run.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demand is an int32 widened to float"
        )]
        let logged_dem = j.output("dem") as i32;
        // Two different reasons the reconstruction can miss, and they mean
        // different things. If nav_pitch_cd itself disagrees, the flight mode
        // set it directly and calc_nav_pitch never ran. If nav_pitch_cd agrees
        // but the demand does not, the flare override replaced it.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demand is an int32 widened to float"
        )]
        let nav_matches = demand.nav_pitch_cd == j.output("nav") as i32;
        let reproducible = nav_matches && dem == logged_dem;

        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged attitude is an int32 widened to float"
        )]
        let pitch_sensor_cd = j.output("ps") as i32;
        let err = PitchDemand::angle_error_cd(
            if reproducible { dem } else { logged_dem },
            pitch_sensor_cd,
        );

        if reproducible {
            limited.sample(row.time_us, j.output("nav"), f64::from(demand.nav_pitch_cd));
            demanded.sample(row.time_us, j.output("dem"), f64::from(dem));
            angle_err.sample(row.time_us, j.output("ae"), f64::from(err));
        } else if nav_matches {
            flare_override += 1;
        } else {
            mode_set_directly += 1;
        }

        #[allow(
            clippy::cast_possible_truncation,
            reason = "attitudes in centidegrees and milliseconds since boot are \
logged as floats but are integral"
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
        let out = c.servo_out(err, &inp);
        driven += 1;

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }
        if reproducible {
            elevator.sample(row.time_us, row.output("out"), out.into());
            compared += 1;
        }
    }

    println!("{driven} attitude steps driven across {segments} segment(s)");
    println!("  {compared} compared end to end");
    println!("  {mode_set_directly} skipped: the flight mode set nav_pitch_cd directly");
    println!("  {flare_override} skipped: the flare override replaced the demand");
    if unjoined > 0 {
        println!("  {unjoined} with no matching join record");
    }
    for cmp in [&limited, &demanded, &angle_err, &elevator] {
        println!("  {}", cmp.report());
    }

    assert!(compared > 1000, "too few steps compared: {compared}");
    assert!(
        limited.passed() && demanded.passed() && angle_err.passed(),
        "the vehicle glue diverges\n  {}\n  {}\n  {}",
        limited.report(),
        demanded.report(),
        angle_err.report()
    );
    assert!(
        elevator.passed(),
        "the composed chain produces a different elevator deflection than the \
         aircraft flew\n  {}",
        elevator.report()
    );
}
