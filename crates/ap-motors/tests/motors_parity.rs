//! Parity test: the multirotor mixing factors against upstream.
//!
//! Six standard layouts — quad X and plus, hexa X and plus, octa X, and a
//! tricopter — built motor by motor and compared both before and after
//! normalisation. Comparing the raw stage too means a transcription error in
//! the angle conversion cannot be masked by a compensating one in the scaling.
//!
//! Every slot in the array is compared, not just the fitted ones: an unfitted
//! motor's factors must stay at zero, and a loop that wrote past its frame
//! would show up there and nowhere else.
//!
//! # The harness
//!
//! `AP_MotorsMatrix`'s constructor chain reaches the AHRS, the battery
//! monitor, `SRV_Channels` and parameter storage. `AP_Param` is linked *for
//! real*, so the object gets its parameter defaults the way a vehicle does
//! rather than sitting at zero — along with the storage manager, semaphores
//! and ring buffer it needs. `SRV_Channels` is not: linking it drags in every
//! ESC backend in the tree, and all `add_motor_num` does with it is register
//! an output channel, which cannot reach the factor arrays. Everything else is
//! an aborting stub, so an unexpected call dies rather than returning a
//! fallback.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::{MotorMatrix, MAX_NUM_MOTORS};

struct Frame {
    name: &'static str,
    motors: &'static [(i8, f32, f32, u8)],
}

/// Mirrors the table in `tools/parity/gen_motors_fixture.py`.
const FRAMES: &[Frame] = &[
    Frame {
        name: "quad_x",
        motors: &[
            (0, 45.0, 1.0, 1),
            (1, -135.0, 1.0, 3),
            (2, -45.0, -1.0, 4),
            (3, 135.0, -1.0, 2),
        ],
    },
    Frame {
        name: "quad_plus",
        motors: &[
            (0, 90.0, 1.0, 2),
            (1, -90.0, 1.0, 4),
            (2, 0.0, -1.0, 1),
            (3, 180.0, -1.0, 3),
        ],
    },
    Frame {
        name: "hexa_x",
        motors: &[
            (0, 90.0, 1.0, 2),
            (1, -90.0, -1.0, 5),
            (2, -30.0, 1.0, 6),
            (3, 150.0, -1.0, 3),
            (4, 30.0, -1.0, 1),
            (5, -150.0, 1.0, 4),
        ],
    },
    Frame {
        name: "hexa_plus",
        motors: &[
            (0, 0.0, 1.0, 1),
            (1, 180.0, -1.0, 4),
            (2, -120.0, -1.0, 5),
            (3, 60.0, 1.0, 2),
            (4, -60.0, 1.0, 6),
            (5, 120.0, -1.0, 3),
        ],
    },
    Frame {
        name: "octa_x",
        motors: &[
            (0, 22.5, 1.0, 1),
            (1, -157.5, 1.0, 5),
            (2, 67.5, -1.0, 2),
            (3, 157.5, -1.0, 4),
            (4, 112.5, 1.0, 3),
            (5, -22.5, -1.0, 8),
            (6, -67.5, 1.0, 7),
            (7, -112.5, -1.0, 6),
        ],
    },
    Frame {
        name: "tri",
        motors: &[(0, 60.0, 1.0, 1), (1, -60.0, 1.0, 3), (2, 180.0, 0.0, 2)],
    },
];

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motors_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_mixing_factors_match_upstream() {
    let text = fixture();
    let mut rows: Vec<Vec<&str>> = Vec::new();
    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("frame,") {
            continue;
        }
        rows.push(line.split(',').collect());
    }

    let mut checked = 0_usize;
    let mut exact = 0_usize;

    for frame in FRAMES {
        let mut m = MotorMatrix::new();
        for &(num, angle, yaw, order) in frame.motors {
            m.add_motor(num, angle, yaw, order);
        }

        for stage in ["raw", "norm"] {
            if stage == "norm" {
                m.normalise_rpy_factors();
            }

            let mine: Vec<&Vec<&str>> = rows
                .iter()
                .filter(|r| r[0] == frame.name && r[2] == stage)
                .collect();
            assert_eq!(
                mine.len(),
                MAX_NUM_MOTORS,
                "{}/{stage}: the fixture has {} slots, the port has {MAX_NUM_MOTORS} — \
                 if these differ, AP_MOTORS_MAX_NUM_MOTORS was taken from the wrong \
                 branch of its #if",
                frame.name,
                mine.len()
            );

            for (i, row) in mine.iter().enumerate() {
                assert_eq!(row.len(), 9);
                assert_eq!(row[1].parse::<usize>().expect("motor index"), i);

                let want_enabled = row[7] == "1";
                assert_eq!(
                    m.is_enabled(i),
                    want_enabled,
                    "{}/{stage} motor {i}: enabled flag",
                    frame.name
                );

                // Every slot, fitted or not: an unfitted one must stay zero.
                let got = m.motor(i).unwrap_or_default();
                for (label, g, w) in [
                    ("roll", got.roll, f(row[3])),
                    ("pitch", got.pitch, f(row[4])),
                    ("yaw", got.yaw, f(row[5])),
                    ("throttle", got.throttle, f(row[6])),
                ] {
                    assert!(
                        same(g, w),
                        "{}/{stage} motor {i} {label}: {g} ({:#010x}) != upstream {w} ({:#010x})",
                        frame.name,
                        g.to_bits(),
                        w.to_bits()
                    );
                    exact += 1;
                    checked += 1;
                }

                if want_enabled {
                    assert_eq!(
                        m.test_order(i),
                        Some(row[8].parse::<u8>().expect("order")),
                        "{}/{stage} motor {i}: test order",
                        frame.name
                    );
                    checked += 1;
                }
            }
        }
    }

    assert!(checked > 1500, "fixture looks truncated: {checked} values");
    println!(
        "{checked} values across {} frames, {exact} factor comparisons all bit-exact",
        FRAMES.len()
    );
}
