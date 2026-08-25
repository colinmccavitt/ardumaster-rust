//! Log-replay test for the ported L1 navigation law (ADR-0008, FW-016).
//!
//! `AP_L1_Control` reads four things from the AHRS — yaw, the groundspeed
//! vector, the current position and pitch — and all four are in the log, which
//! is why this can be verified without porting `AP_AHRS` at all.
//!
//! # Only one of four entry points is logged
//!
//! The vehicle also calls `update_loiter`, `update_heading_hold` and
//! `update_level_flight`, and all four share `_last_Nu`, `_L1_dist` and the
//! crosstrack integrator. A replay that ran only the logged waypoint calls in
//! order would be evolving state upstream had changed underneath it.
//!
//! So `L1O` records `_last_Nu` and `_L1_xtrack_i` as they stood *entering*
//! each call. The replay carries its own state forward, checks it against
//! those two on every step, and reseeds when they disagree — reporting how
//! often, so a quiet drift cannot pass as agreement. Inside a run of
//! consecutive waypoint calls the state must evolve identically; between runs
//! the reseed is expected and counted.

#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's recorded values is the point"
)]

use ap_math::location::Location;
use ap_math::vector2::Vector2f;
use ap_nav::{L1Control, L1Gains, NavInputs};
use ap_replay::{Comparison, Fixture, Params, Row};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn controller_from_params(p: &Params) -> L1Control {
    L1Control::new(L1Gains {
        period: p.f32("NAVL1_PERIOD"),
        damping: p.f32("NAVL1_DAMPING"),
        xtrack_i_gain: p.f32("NAVL1_XTRACK_I"),
        loiter_bank_limit: p.f32("NAVL1_LIM_BANK"),
    })
}

#[allow(
    clippy::cast_possible_truncation,
    reason = "coordinates and microsecond counters are logged as floats but are \
integral; the log stores them from the int32 and uint32 upstream used"
)]
fn inputs(row: &Row) -> NavInputs {
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
fn l1_waypoint_navigation_replay_against_upstream_flight() {
    let dir = fixtures_dir();
    let fx_path = dir.join("l1_replay.csv");
    let pm_path = dir.join("l1_replay_params.csv");
    if !fx_path.exists() || !pm_path.exists() {
        eprintln!("skipping: fixture or parameter set not present");
        return;
    }
    let fx = Fixture::load(&fx_path).expect("fixture should load");
    let params = Params::load(&pm_path).expect("parameters should load");
    assert!(fx.len() > 500, "expected a real flight, got {}", fx.len());

    let mut lat_acc = Comparison::new("lateral acceleration (m/s2)", 0.001);
    let mut nav_roll = Comparison::new("nav_roll_cd (centidegrees)", 1.0);
    let mut nav_bearing = Comparison::new("nav bearing (rad)", 0.0001);
    let mut bearing_err = Comparison::new("bearing error (rad)", 0.0001);
    let mut xtrack = Comparison::new("crosstrack error (m)", 0.001);
    let mut l1_dist = Comparison::new("L1 distance (m)", 0.001);
    let mut xtrack_i = Comparison::new("crosstrack integrator", 1e-6);

    let mut c = controller_from_params(&params);
    let mut compared = 0usize;
    let mut reseeds = 0usize;
    let mut target_bearing_mismatch = 0usize;

    for row in &fx.rows {
        let inp = inputs(row);

        // The state upstream carried into this call. Where it differs from
        // what the port carried, another entry point ran in between and the
        // port has to be put back on upstream's trajectory.
        let want_last_nu = row.output("enu") as f32;
        let want_xtrack_i = row.output("exi") as f32;
        let carried = c.state_for_replay();
        if carried.last_nu != want_last_nu || carried.xtrack_i != want_xtrack_i {
            c.seed_for_replay(want_last_nu, want_xtrack_i);
            reseeds += 1;
        }

        let prev_wp = Location::new(row.input("pa") as i32, row.input("po") as i32);
        let next_wp = Location::new(row.input("na") as i32, row.input("no") as i32);
        c.update_waypoint(prev_wp, next_wp, row.input("dm") as f32, &inp);
        let st = c.state_for_replay();

        lat_acc.sample(row.time_us, row.output("lad"), st.lat_acc_dem.into());
        nav_roll.sample(
            row.time_us,
            row.output("nrc"),
            f64::from(c.nav_roll_cd(&inp)),
        );
        nav_bearing.sample(row.time_us, row.output("nbr"), st.nav_bearing.into());
        bearing_err.sample(row.time_us, row.output("ber"), st.bearing_error.into());
        xtrack.sample(row.time_us, row.output("xte"), st.crosstrack_error.into());
        l1_dist.sample(row.time_us, row.output("l1d"), st.l1_dist.into());
        xtrack_i.sample(row.time_us, row.output("xti"), st.xtrack_i.into());

        #[allow(
            clippy::cast_possible_truncation,
            reason = "the logged bearing is an int32 widened to float"
        )]
        let want_tbc = row.output("tbc") as i32;
        if st.target_bearing_cd != want_tbc {
            target_bearing_mismatch += 1;
        }
        compared += 1;
    }

    println!("ekf-double in force: {}", ap_math::EKF_DOUBLE);
    println!("replayed {compared} waypoint calls");
    println!("  {reseeds} reseed(s) where another L1 entry point had moved the shared state");
    for cmp in [
        &lat_acc,
        &nav_roll,
        &nav_bearing,
        &bearing_err,
        &xtrack,
        &l1_dist,
        &xtrack_i,
    ] {
        println!("  {}", cmp.report());
    }

    assert!(compared > 500, "too few samples: {compared}");
    assert_eq!(
        target_bearing_mismatch, 0,
        "the target bearing is computed from logged positions alone, so it must \
         agree on every call"
    );
    assert!(
        reseeds * 4 < compared,
        "{reseeds} of {compared} calls needed a reseed, so the shared state is \
         being moved by other entry points more often than not and this is no \
         longer testing continuous evolution"
    );
    assert!(
        lat_acc.passed() && nav_roll.passed(),
        "the demanded lateral acceleration or bank angle diverges\n  {}\n  {}",
        lat_acc.report(),
        nav_roll.report()
    );
    assert!(
        nav_bearing.passed() && bearing_err.passed() && xtrack.passed() && l1_dist.passed(),
        "a reported navigation quantity diverges\n  {}\n  {}\n  {}\n  {}",
        nav_bearing.report(),
        bearing_err.report(),
        xtrack.report(),
        l1_dist.report()
    );
    assert!(
        xtrack_i.passed(),
        "the crosstrack integrator diverged, so the state did not evolve \
         identically\n  {}",
        xtrack_i.report()
    );
}
