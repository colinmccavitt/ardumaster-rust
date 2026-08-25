//! Stage-by-stage verification against upstream's logged intermediates.
//!
//! The replay test in `replay.rs` compares the two outputs. This compares every
//! intermediate the controller produces, against the values upstream recorded
//! at the same instants: the height demand chain, the four energies, the
//! speed/height weightings, both pitch integrators, the applied limits.
//!
//! Two reasons it exists separately:
//!
//! * A pair of compensating errors can leave throttle and pitch correct while
//!   the chain that produced them is wrong. Asserting the intermediates makes
//!   that impossible.
//! * When something does break, the report names the earliest stage to leave
//!   tolerance rather than the largest deviation. In a feedback loop the
//!   largest deviation is almost always a consequence, not the cause -- ranking
//!   by magnitude sent this investigation to the wrong stage twice.
//!
//! Tolerances are tight deliberately. ADR-0004 does not require bit-exact float
//! parity, and these are not bit-exact assertions; they are close enough that
//! any behavioural difference shows up rather than hiding under a loose bound.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes a fixture whose length the test asserts; a bad index is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's logged values is what the test is for"
)]
use ap_replay::{Fixture, Params};

#[path = "replay.rs"]
mod replay;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

struct Track {
    name: &'static str,
    tol: f64,
    first_us: Option<u64>,
    up: f64,
    port: f64,
    count: usize,
}

#[test]
fn every_stage_matches_upstream() {
    let dir = fixtures_dir();
    let fx = match Fixture::load(dir.join("tecs_replay.csv")) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("skipping: {e}");
            return;
        }
    };
    let params = Params::load(&dir.join("tecs_replay_params.csv")).expect("params");

    let mut tr: Vec<Track> = [
        // height stage
        ("hdin", 0.005),
        ("hdip", 0.005),
        ("hrtl", 0.005),
        ("hlpf", 0.005),
        ("mcs", 0.002),
        ("mss", 0.002),
        ("crl", 0.005),
        ("srl", 0.005),
        ("pto", 0.005),
        ("hdem", 0.005),
        ("dhdem", 0.005),
        ("spdem", 0.005),
        // pitch stage
        ("PEW", 1e-3),
        ("KEW", 1e-3),
        ("P", 0.02),
        ("K", 0.02),
        ("Pdem", 0.02),
        ("Kdem", 0.02),
        ("EBD", 0.05),
        ("EBE", 0.05),
        ("EBDE", 0.02),
        ("I", 0.1),
        ("KI", 0.1),
        ("pmin", 0.002),
        ("pmax", 0.002),
        ("pdu", 0.002),
        // outputs
        ("th", 0.005),
        ("ph", 0.005),
    ]
    .iter()
    .map(|(name, tol)| Track {
        name,
        tol: *tol,
        first_us: None,
        up: 0.0,
        port: 0.0,
        count: 0,
    })
    .collect();

    let mut compared = 0usize;
    let segments = replay::split_into_segments(&fx);

    for seg in &segments {
        let rows = &fx.rows[seg.clone()];
        let mut tecs = replay::tecs_from_params(&params);
        tecs.update_pitch_throttle(
            &replay::inputs_from_row(&rows[0], &params),
            rows[0].output("dt") as f32,
        );
        tecs.seed_for_replay(&replay::seed_from_row(&rows[0]));

        for row in &rows[1..] {
            let dt = row.output("dt") as f32;
            tecs.update_pitch_throttle(&replay::inputs_from_row(row, &params), dt);
            compared += 1;
            let s = tecs.snapshot();
            let pew = (2.0 - s.ske_weighting).min(1.0);
            let port = [
                s.hgt_dem_in as f64,
                s.hgt_dem_in_prev as f64,
                s.hgt_dem_rate_ltd as f64,
                s.hgt_dem_lpf as f64,
                s.max_climb_scaler as f64,
                s.max_sink_scaler as f64,
                s.climb_rate_limit as f64,
                s.sink_rate_limit as f64,
                s.post_to_hgt_offset as f64,
                s.hgt_dem as f64,
                s.hgt_rate_dem as f64,
                s.tas_dem_adj as f64,
                pew as f64,
                s.ske_weighting as f64,
                s.spe_est as f64,
                s.ske_est as f64,
                s.spe_dem as f64,
                s.ske_dem as f64,
                (s.spe_dem * pew - s.ske_dem * s.ske_weighting) as f64,
                (s.spe_est * pew - s.ske_est * s.ske_weighting) as f64,
                (s.spedot * pew - s.skedot * s.ske_weighting) as f64,
                s.integ_sebdot as f64,
                s.integ_ke as f64,
                s.pitch_min as f64,
                s.pitch_max as f64,
                s.pitch_dem_unc as f64,
                tecs.throttle_demand() as f64,
                tecs.pitch_demand() as f64,
            ];

            for (k, t) in tr.iter_mut().enumerate() {
                let d = (row.output(t.name) - port[k]).abs();
                if d > t.tol {
                    t.count += 1;
                    if t.first_us.is_none() {
                        t.first_us = Some(row.time_us);
                        t.up = row.output(t.name);
                        t.port = port[k];
                    }
                }
            }
        }
    }

    let mut order: Vec<&Track> = tr.iter().collect();
    order.sort_by_key(|t| t.first_us.unwrap_or(u64::MAX));

    println!("{} segment(s), stage-by-stage:", segments.len());
    let mut failed = Vec::new();
    for t in order {
        match t.first_us {
            None => println!("  {:6} ok (tol {})", t.name, t.tol),
            Some(us) => {
                let line = format!(
                    "{} first diverges at t={:.3}s: upstream {:.4}, port {:.4} ({} rows, tol {})",
                    t.name,
                    us as f64 / 1e6,
                    t.up,
                    t.port,
                    t.count,
                    t.tol
                );
                println!("  DIVERGES {line}");
                failed.push(line);
            }
        }
    }

    // A comparison that saw nothing passes vacuously.
    assert!(
        compared > 2000,
        "expected the whole flight to be compared, got {compared} rows"
    );

    assert!(
        failed.is_empty(),
        "port diverges from upstream, earliest first:\n  {}",
        failed.join("\n  ")
    );
}
