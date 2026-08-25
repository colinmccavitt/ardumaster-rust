//! The first end-to-end test: navigation through to aileron.
//!
//! Every crate in this port has been verified alone against upstream's own
//! recorded behaviour, and none of them had ever been run together. This does
//! that: L1 produces a bank angle, the vehicle glue limits it and subtracts the
//! measured roll, and the roll controller turns the difference into an aileron
//! deflection — compared against what the aircraft actually flew.
//!
//! ```text
//!   L1Control::update_waypoint      10 Hz  ->  lateral acceleration
//!   L1Control::nav_roll_cd          50 Hz  ->  bank angle
//!   RollDemand::from_navigation            ->  limited demand
//!   RollDemand::angle_error_cd             ->  angle error
//!   RollController::servo_out       50 Hz  ->  aileron, centidegrees
//! ```
//!
//! # Three things about the composition that the per-module tests could not see
//!
//! **`nav_roll_cd()` is a view, not a value.** It converts the held lateral
//! acceleration into a bank angle using the pitch at the moment it is called,
//! so the vehicle gets a different answer at every 50 Hz attitude step even
//! though navigation last ran at 10 Hz. Computing it once per navigation
//! update was wrong on 4,104 of 10,971 steps, by up to 84 degrees.
//!
//! **Attitude control runs before navigation within a tick.** A navigation
//! update sharing a timestamp with an attitude step is not visible to it.
//! Consuming updates with `<=` rather than `<` put the bank angle one step
//! ahead of the aircraft's for 1,202 steps.
//!
//! **The roll controller runs on every step, including the ones this slice
//! cannot compare.** It carries filter and integrator state, so skipping those
//! steps entirely left it on a trajectory the aircraft never flew. They are
//! driven from the logged angle error instead, which keeps it on upstream's.
//!
//! # What is deliberately not compared
//!
//! Only waypoint tracking is replayed. Where the vehicle was loitering or
//! holding a heading, or setting the roll demand directly as it does on
//! takeoff and landing, the bank angle comes from a path this slice does not
//! model. Rather than enumerate flight modes, the test asks upstream: if its
//! own recorded demand is not what constraining the navigation output would
//! give, that step is counted and skipped. Each exclusion is reported.

#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows inside a bounds-checked range; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::{RateGains, RollController, RollInputs};
use ap_math::location::Location;
use ap_math::vector2::Vector2f;
use ap_nav::{L1Control, L1Gains, NavInputs};
use ap_pid::PidGains;
use ap_plane::RollDemand;
use ap_replay::{Comparison, Fixture, Params, Row};

/// One navigation period. An attitude step further behind than this means a
/// navigation update ran that was not a waypoint call.
const NAV_PERIOD_US: u64 = 100_000;

/// Looser bound for deciding whether two waypoint calls were consecutive: the
/// navigation rate itself jitters by about a millisecond.
const NAV_RUN_GAP_US: u64 = 110_000;

/// Attitude steps to skip while the roll controller's unlogged filters
/// converge on upstream's state.
const WARMUP_STEPS: usize = 50;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn l1_from_params(p: &Params) -> L1Control {
    L1Control::new(L1Gains {
        period: p.f32("NAVL1_PERIOD"),
        damping: p.f32("NAVL1_DAMPING"),
        xtrack_i_gain: p.f32("NAVL1_XTRACK_I"),
        loiter_bank_limit: p.f32("NAVL1_LIM_BANK"),
    })
}

fn roll_from_params(p: &Params) -> RollController {
    let pid = PidGains {
        p: p.f32("RLL_RATE_P"),
        i: p.f32("RLL_RATE_I"),
        d: p.f32("RLL_RATE_D"),
        ff: p.f32("RLL_RATE_FF"),
        dff: 0.0,
        imax: p.f32("RLL_RATE_IMAX"),
        pdmax: 0.0,
        filt_t_hz: p.f32("RLL_RATE_FLTT"),
        filt_e_hz: p.f32("RLL_RATE_FLTE"),
        filt_d_hz: p.f32("RLL_RATE_FLTD"),
        srmax: p.f32("RLL_RATE_SMAX"),
        srtau: 1.0,
    };
    let gains = RateGains {
        tau: p.f32("RLL2SRV_TCONST"),
        rmax_pos: p.f32("RLL2SRV_RMAX"),
        rmax_neg: p.f32("RLL2SRV_RMAX"),
    };
    RollController::new(pid, gains)
}

#[allow(
    clippy::cast_possible_truncation,
    reason = "coordinates and counters are logged as floats but are integral"
)]
fn nav_inputs(row: &Row) -> NavInputs {
    NavInputs {
        now_us: row.input("us") as u32,
        now_ms: row.input("ms") as u32,
        location: Some(Location::new(
            row.input("la") as i32,
            row.input("ln") as i32,
        )),
        groundspeed_vector: Vector2f::new(row.input("gx") as f32, row.input("gy") as f32),
        yaw_rad: row.input("yw") as f32,
        yaw_sensor_cd: row.input("ys") as i32,
        pitch_rad: row.input("pt") as f32,
        eas2tas: row.input("e2") as f32,
    }
}

#[test]
fn navigation_through_to_aileron_matches_the_flight() {
    let dir = fixtures_dir();
    let paths = [
        dir.join("vslice_l1.csv"),
        dir.join("vslice_roll.csv"),
        dir.join("vslice_join.csv"),
        dir.join("vslice_params.csv"),
    ];
    if paths.iter().any(|p| !p.exists()) {
        eprintln!("skipping: vertical-slice fixtures not present");
        return;
    }
    let l1_fx = Fixture::load(&paths[0]).expect("L1 fixture");
    let roll_fx = Fixture::load(&paths[1]).expect("roll fixture");
    let join_fx = Fixture::load(&paths[2]).expect("join fixture");
    let params = Params::load(&paths[3]).expect("parameters");

    assert!(l1_fx.len() > 300, "L1 rows: {}", l1_fx.len());
    assert!(roll_fx.len() > 1000, "roll rows: {}", roll_fx.len());

    let join: std::collections::HashMap<u64, &Row> =
        join_fx.rows.iter().map(|r| (r.time_us, r)).collect();

    #[allow(
        clippy::cast_possible_truncation,
        reason = "an int16 parameter round-tripped through the log as a float"
    )]
    let airspeed_min = params.f32("AIRSPEED_MIN") as i16;

    let mut l1 = l1_from_params(&params);
    let mut roll = roll_from_params(&params);

    let mut nav_roll = Comparison::new("stage 1: nav_roll_cd from L1", 1.0);
    let mut limited = Comparison::new("stage 2: after the roll limit", 1.0);
    let mut angle_err = Comparison::new("stage 3: angle error", 1.0);
    let mut aileron = Comparison::new("stage 4: aileron out (centidegrees)", 2.0);

    let mut next_l1 = 0usize;
    let mut held_inputs: Option<NavInputs> = None;
    let mut last_l1_us: Option<u64> = None;
    let mut consecutive_l1 = 0usize;

    let mut warmup_left = WARMUP_STEPS;
    let mut driven = 0usize;
    let mut compared = 0usize;
    let mut l1_calls = 0usize;
    let mut skipped_other_mode = 0usize;
    let mut skipped_direct = 0usize;
    let mut unjoined = 0usize;
    let mut prev_roll_us: Option<u64> = None;
    let mut segments = 0usize;

    for row in &roll_fx.rows {
        // Navigation updates that fell due STRICTLY BEFORE this attitude step.
        // Strictly, because attitude control runs first within a tick.
        while next_l1 < l1_fx.rows.len() && l1_fx.rows[next_l1].time_us < row.time_us {
            let lrow = &l1_fx.rows[next_l1];
            let inp = nav_inputs(lrow);

            // Put L1 back on upstream's trajectory where another entry point
            // moved the shared state; the L1 replay test measures how often.
            l1.seed_for_replay(lrow.output("enu") as f32, lrow.output("exi") as f32);

            let prev_wp = Location::new(lrow.input("pa") as i32, lrow.input("po") as i32);
            let next_wp = Location::new(lrow.input("na") as i32, lrow.input("no") as i32);
            l1.update_waypoint(prev_wp, next_wp, lrow.input("dm") as f32, &inp);

            consecutive_l1 = match last_l1_us {
                Some(prev) if lrow.time_us.saturating_sub(prev) <= NAV_RUN_GAP_US => {
                    consecutive_l1 + 1
                }
                _ => 1,
            };
            held_inputs = Some(inp);
            last_l1_us = Some(lrow.time_us);
            l1_calls += 1;
            next_l1 += 1;
        }

        // A gap means attitude steps upstream ran that were never recorded,
        // so the controller's unlogged filter history is not reconstructible
        // across it. Rebuild, seed the integrator from the log, and warm up
        // again -- exactly what the standalone roll replay does.
        let hole = match prev_roll_us {
            None => true,
            Some(p) => (row.time_us - p) as f64 * 1e-6 > f64::from(row.input("dt") as f32) * 1.5,
        };
        prev_roll_us = Some(row.time_us);
        if hole {
            roll = roll_from_params(&params);
            roll.controller
                .rate_pid
                .set_integrator(row.input("ig") as f32);
            segments += 1;
            warmup_left = WARMUP_STEPS;
        }

        let Some(j) = join.get(&row.time_us) else {
            unjoined += 1;
            continue;
        };

        // Decide whether this step's demand is one the slice can reproduce.
        // Whatever the answer, the roll controller still has to run: it carries
        // filter and integrator state, and skipping steps would leave it on a
        // trajectory the aircraft never flew.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demand, limit and attitude are int32 widened to float"
        )]
        let logged_err = j.output("ae") as i32;

        let in_waypoint_run = consecutive_l1 >= 2
            && last_l1_us.is_some_and(|t| row.time_us.saturating_sub(t) <= NAV_PERIOD_US);

        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demand and limit are int32 widened to float"
        )]
        let upstream_would_be =
            RollDemand::from_navigation(j.output("nrc") as i32, j.output("rlc") as i32);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged demand is an int32 widened to float"
        )]
        let derived_from_navigation = upstream_would_be.nav_roll_cd == j.output("nav") as i32;

        let err = if let (true, true, Some(base_inputs)) =
            (in_waypoint_run, derived_from_navigation, held_inputs)
        {
            // nav_roll_cd() is a view over the held lateral acceleration: it
            // converts using the pitch at the moment it is called.
            let step_inputs = NavInputs {
                pitch_rad: j.output("pt") as f32,
                ..base_inputs
            };
            let nav_roll_cd = l1.nav_roll_cd(&step_inputs);
            nav_roll.sample(row.time_us, j.output("nrc"), f64::from(nav_roll_cd));

            #[allow(
                clippy::cast_possible_truncation,
                reason = "the logged limit is an int32 widened to float"
            )]
            let demand = RollDemand::from_navigation(nav_roll_cd, j.output("rlc") as i32);
            limited.sample(row.time_us, j.output("nav"), f64::from(demand.nav_roll_cd));

            #[allow(
                clippy::cast_possible_truncation,
                reason = "the logged attitude is an int32 widened to float"
            )]
            let err = demand.angle_error_cd(j.output("rs") as i32);
            angle_err.sample(row.time_us, j.output("ae"), f64::from(err));
            Some(err)
        } else {
            if !in_waypoint_run {
                skipped_other_mode += 1;
            } else if !derived_from_navigation {
                skipped_direct += 1;
            }
            None
        };

        #[allow(
            clippy::cast_possible_truncation,
            reason = "milliseconds since boot fits a u32 for any flight"
        )]
        let rinp = RollInputs {
            scaler: row.input("sc") as f32,
            disable_integrator: row.input("di") != 0.0,
            ground_mode: row.input("gm") != 0.0,
            roll_rate_rad: row.input("gy") as f32,
            airspeed_eas: Some(row.input("as") as f32),
            airspeed_min,
            eas2tas: row.input("e2t") as f32,
            dt: row.input("dt") as f32,
            now_ms: (row.time_us / 1000) as u32,
        };
        let out = roll.servo_out(err.unwrap_or(logged_err), &rinp);
        driven += 1;

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }
        if err.is_some() {
            aileron.sample(row.time_us, row.output("out"), out.into());
            compared += 1;
        }
    }

    println!("{l1_calls} navigation updates, {driven} attitude steps driven");
    println!("  {compared} steps compared end to end across {segments} segment(s)");
    println!("  {skipped_other_mode} skipped: loitering or holding a heading");
    println!("  {skipped_direct} skipped: the vehicle set the demand directly");
    if unjoined > 0 {
        println!("  {unjoined} with no matching join record");
    }
    for cmp in [&nav_roll, &limited, &angle_err, &aileron] {
        println!("  {}", cmp.report());
    }

    assert!(
        compared > 1000,
        "too few steps compared end to end: {compared}"
    );
    assert!(l1_calls > 300, "too few navigation updates: {l1_calls}");
    assert!(
        nav_roll.passed(),
        "L1 asked for a different bank angle than the aircraft used\n  {}",
        nav_roll.report()
    );
    assert!(
        limited.passed() && angle_err.passed(),
        "the vehicle glue diverges\n  {}\n  {}",
        limited.report(),
        angle_err.report()
    );
    assert!(
        aileron.passed(),
        "the composed chain produces a different aileron deflection than the \
         aircraft flew\n  {}",
        aileron.report()
    );
}
