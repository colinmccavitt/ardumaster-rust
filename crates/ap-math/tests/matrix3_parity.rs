//! Parity test for `Matrix3::rotate` (FW-002, needed by FW-008).
//!
//! The DCM integration step is `_dcm_matrix.rotate(omega * dt)`, so this is
//! the arithmetic the attitude estimate is built from. It is a pure function,
//! and it is compared against upstream's compiled code over a grid that mixes
//! matrices which are attitudes with ones that are not, and rotation vectors
//! from a realistic 400 Hz gyro step up to values far past where a first-order
//! approximation means anything.
//!
//! Exact agreement is required. There is nothing here that could legitimately
//! differ — no transcendentals, no accumulated state, just multiplies and
//! adds.

#![allow(
    clippy::float_cmp,
    reason = "bit equality against upstream's recorded values is the assertion"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture fields whose count is checked first; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

#[test]
fn matrix3_rotate_matches_upstream() {
    let path = fixtures_dir().join("matrix3_rotate.csv");
    let Ok(text) = std::fs::read_to_string(&path) else {
        eprintln!("skipping: matrix3_rotate.csv not present");
        return;
    };

    let mut checked = 0usize;
    let mut mismatches = Vec::new();

    for line in text.lines().skip(1) {
        let f: Vec<f32> = line
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if f.len() != 21 {
            continue;
        }

        let mut m = Matrix3f {
            a: Vector3f::new(f[0], f[1], f[2]),
            b: Vector3f::new(f[3], f[4], f[5]),
            c: Vector3f::new(f[6], f[7], f[8]),
        };
        m.rotate(Vector3f::new(f[9], f[10], f[11]));

        let want = [
            f[12], f[13], f[14], f[15], f[16], f[17], f[18], f[19], f[20],
        ];
        let got = [
            m.a.x, m.a.y, m.a.z, m.b.x, m.b.y, m.b.z, m.c.x, m.c.y, m.c.z,
        ];
        for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
            if g.to_bits() != w.to_bits() && mismatches.len() < 8 {
                mismatches.push(format!(
                    "element {i} for rotation ({}, {}, {}): port {g}, upstream {w}",
                    f[9], f[10], f[11]
                ));
            }
        }
        checked += 1;
    }

    println!("{checked} rotations compared");
    assert!(checked >= 50, "too few cases compared: {checked}");
    assert!(
        mismatches.is_empty(),
        "{} element(s) disagree; first few:\n  {}",
        mismatches.len(),
        mismatches.join("\n  ")
    );
}

/// PORT-DERIVED. The property that makes this a DCM integration step rather
/// than a rotation: for a small angle it advances the attitude, and it leaves
/// the matrix slightly non-orthonormal, which is why normalisation follows.
#[test]
fn a_small_rotation_advances_the_attitude_and_distorts_it() {
    let mut m = Matrix3f::identity();
    // 1 rad/s of yaw for one 400 Hz step
    m.rotate(Vector3f::new(0.0, 0.0, 0.0025));

    // The first row swings toward -y, not +y: the update is M += M x g, so
    // the cross product puts the yaw rate's effect on the row's y component
    // with a negative sign. Getting this backwards by hand is easy, which is
    // why the parity fixture above is the real check and this only pins the
    // magnitude and the convention.
    assert!(
        (m.a.y + 0.0025).abs() < 1e-6,
        "expected the row to advance by the step angle toward -y, got {}",
        m.a.y
    );
    // and its length should now exceed one: this is not a rotation
    assert!(
        m.a.length() > 1.0,
        "a first-order step should stretch the row, length {}",
        m.a.length()
    );
}
