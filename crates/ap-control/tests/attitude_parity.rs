//! Parity test: the attitude controller against upstream.
//!
//! Forty-nine attitude pairs driven through the real
//! `AC_AttitudeControl_Multi` — the error decomposition, the yaw cap, and the
//! rate target it produces — compared against the port.
//!
//! Everything else in this ticket is pinned by properties: that the two
//! rotations compose back to the target, that a heading command does not leak
//! into roll, that a stronger axis is capped tighter. Those catch a great deal,
//! and this session has shown they also catch the *test author* being wrong
//! more often than the code. What they cannot catch is the port and the
//! properties sharing a misreading of upstream. This can.
//!
//! # Tolerance
//!
//! Not bit-exact. The decomposition runs through `acosf` and `atan2f`, and the
//! rate target through the square-root controller's `sqrtf` — three
//! transcendentals whose last bits are not guaranteed to agree between libm
//! and the platform's C library. The tolerance below is loose enough to absorb
//! that and far tighter than any difference an arithmetic or sign error would
//! produce: a wrong axis, a missing frame conversion or a dropped cap all move
//! these by 1e-2 or more, not 1e-5.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::attitude_error::{
    thrust_heading_rotation_angles, update_ang_vel_target_from_att_error, AngleGains, YawLimitGains,
};
use ap_math::quaternion::Quaternion;

/// Absorbs transcendental disagreement; far below any structural error.
///
/// Set from measurement, not from caution: the largest difference across the
/// whole sweep is 4.8e-7, about four ulp at these magnitudes. 2e-6 leaves room
/// for a different libm without leaving room for a mistake -- a wrong axis, a
/// missing frame conversion or a dropped cap all move these by 1e-2 or more.
const TOL: f32 = 2e-6;

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

struct Fixture {
    gains: Vec<f32>,
    use_sqrt: bool,
    rows: Vec<Vec<String>>,
}

fn fixture() -> Fixture {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/attitude_error.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_attitude_fixture.py",
            path.display()
        )
    });

    let mut section = "";
    let mut gains = Vec::new();
    let mut use_sqrt = false;
    let mut rows = Vec::new();

    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            section = match tag {
                "gains" => "gains",
                "rows" => "rows",
                other => panic!("unknown section {other}"),
            };
            continue;
        }
        if line.is_empty() || line.starts_with("angle_p_roll") || line.starts_with("body_r") {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        match section {
            "gains" => {
                assert_eq!(c.len(), 9, "malformed gains row");
                gains = c[..7].iter().map(|s| f(s)).collect();
                use_sqrt = c[7] == "1";
                gains.push(f(c[8])); // dt
            }
            "rows" => {
                assert_eq!(c.len(), 14, "malformed row: {line}");
                rows.push(c.iter().map(|s| (*s).to_owned()).collect());
            }
            _ => panic!("row outside any section"),
        }
    }

    assert!(!gains.is_empty(), "no gains in the fixture");
    assert!(!rows.is_empty(), "no rows in the fixture");
    Fixture {
        gains,
        use_sqrt,
        rows,
    }
}

#[test]
fn the_attitude_controller_matches_upstream() {
    let fx = fixture();
    let (angle_p_roll, angle_p_pitch, angle_p_yaw) = (fx.gains[0], fx.gains[1], fx.gains[2]);
    let (accel_roll, accel_pitch, accel_yaw) = (fx.gains[3], fx.gains[4], fx.gains[5]);
    let rate_yaw_kp = fx.gains[6];
    let dt = fx.gains[7];

    let yaw_gains = YawLimitGains {
        accel_yaw_max_radss: accel_yaw,
        rate_yaw_kp,
        angle_yaw_kp: angle_p_yaw,
        ..YawLimitGains::default()
    };
    let angle_gains = AngleGains {
        angle_p_roll,
        angle_p_pitch,
        angle_p_yaw,
        accel_roll_max_radss: accel_roll,
        accel_pitch_max_radss: accel_pitch,
        accel_yaw_max_radss: accel_yaw,
        use_sqrt_controller: fx.use_sqrt,
    };

    let mut checked = 0_usize;
    let mut largest = 0.0_f32;
    let mut capped_cases = 0_usize;

    for r in &fx.rows {
        let body = Quaternion::from_euler(f(&r[0]), f(&r[1]), f(&r[2]));
        let target = Quaternion::from_euler(f(&r[3]), f(&r[4]), f(&r[5]));

        let (_, e) = thrust_heading_rotation_angles(target, body, &yaw_gains);
        let rate = update_ang_vel_target_from_att_error(e.error_rad, &angle_gains, dt);

        for (label, got, want) in [
            ("err_x", e.error_rad.x, f(&r[6])),
            ("err_y", e.error_rad.y, f(&r[7])),
            ("err_z", e.error_rad.z, f(&r[8])),
            ("thrust_angle", e.thrust_angle_rad, f(&r[9])),
            ("thrust_err", e.thrust_error_angle_rad, f(&r[10])),
            ("rate_x", rate.x, f(&r[11])),
            ("rate_y", rate.y, f(&r[12])),
            ("rate_z", rate.z, f(&r[13])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "body ({},{},{}) target ({},{},{}) {label}: {got} != upstream \
                 {want} (diff {diff})",
                f(&r[0]),
                f(&r[1]),
                f(&r[2]),
                f(&r[3]),
                f(&r[4]),
                f(&r[5])
            );
            checked += 1;
        }

        // Count the cases where the yaw cap actually bound, so the assertion
        // below can prove the sweep exercised it.
        if libm::fabsf(e.error_rad.z) > 0.7 {
            capped_cases += 1;
        }
    }

    assert!(
        capped_cases > 0,
        "no case reached a large heading error, so the yaw cap went unexercised"
    );
    println!(
        "{} attitude pairs, {checked} values, largest difference {largest:e}",
        fx.rows.len()
    );
}
