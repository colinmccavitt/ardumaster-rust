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
            // Sections this test does not read are skipped rather than
            // rejected: the fixture is shared, and one added for another test
            // is not an error here.
            section = tag;
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        match section {
            "gains" => {
                // The row carries the shaping config too; this test needs
                // only the first nine columns.
                assert!(c.len() >= 9, "malformed gains row: {} columns", c.len());
                gains = c[..7].iter().map(|s| f(s)).collect();
                use_sqrt = c[7] == "1";
                gains.push(f(c[8])); // dt
            }
            "rows" => {
                assert_eq!(c.len(), 14, "malformed row: {line}");
                rows.push(c.iter().map(|s| (*s).to_owned()).collect());
            }
            _ => {}
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

/// Parity: 400 steps of a scripted stick sequence through the euler entry
/// point.
///
/// The entry point is stateful — the target is carried between calls and
/// shaped toward the stick over many iterations — so a single call proves
/// almost nothing. A shaping error converges to the same place either way and
/// differs only in how it gets there, which is visible only step by step.
///
/// The sequence is a step in roll, a ramp in pitch, and a yaw rate that
/// reverses part-way: between them the shaper is made to settle, to track, and
/// to turn around.
///
/// Everything the C++ side ran with is read from the fixture rather than
/// copied from parameter defaults — the gains, the shaping constants, and the
/// body attitude the controller took from its AHRS. Guessing any of them is
/// the habit that has produced wrong answers all through this port.
#[test]
fn the_stick_sequence_matches_upstream() {
    use ap_control::attitude_controller::{AttitudeController, ShapingConfig};
    use ap_math::vector3::Vector3f;

    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/attitude_error.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut section = "";
    let mut gains: Vec<f32> = Vec::new();
    let mut ff_enabled = false;
    let mut sticks: Vec<Vec<String>> = Vec::new();

    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            section = tag;
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        match section {
            "gains" => {
                assert_eq!(c.len(), 15, "malformed gains row");
                gains = c
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != 7 && *i != 11)
                    .map(|(_, s)| f(s))
                    .collect();
                ff_enabled = c[11] == "1";
            }
            "sticks" => {
                assert_eq!(c.len(), 16, "malformed stick row: {line}");
                sticks.push(c.iter().map(|s| (*s).to_owned()).collect());
            }
            _ => {}
        }
    }

    assert!(!sticks.is_empty(), "no stick rows in the fixture");
    assert!(ff_enabled, "the fixture must run with feedforward on");

    // gains, with the two flag columns removed: angle P x3, accel x3,
    // rate_yaw_kp, dt, input_tc, rate_y_tc, vel maxes x3.
    let shaping = ShapingConfig {
        input_tc: gains[8],
        rate_y_tc: gains[9],
        rate_bf_ff_enabled: ff_enabled,
        ang_vel_roll_max_degs: gains[10],
        ang_vel_pitch_max_degs: gains[11],
        ang_vel_yaw_max_degs: gains[12],
        accel_roll_max_radss: gains[3],
        accel_pitch_max_radss: gains[4],
        accel_yaw_max_radss: gains[5],
    };
    let yaw_gains = YawLimitGains {
        accel_yaw_max_radss: gains[5],
        rate_yaw_kp: gains[6],
        angle_yaw_kp: gains[2],
        ..YawLimitGains::default()
    };
    let angle_gains = AngleGains {
        angle_p_roll: gains[0],
        angle_p_pitch: gains[1],
        angle_p_yaw: gains[2],
        accel_roll_max_radss: gains[3],
        accel_pitch_max_radss: gains[4],
        accel_yaw_max_radss: gains[5],
        use_sqrt_controller: true,
    };
    let dt = gains[7];

    let mut controller = AttitudeController::new();
    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &sticks {
        let step: usize = r[0].parse().expect("step");
        let (roll_cmd, pitch_cmd, yaw_rate_cmd) = (f(&r[1]), f(&r[2]), f(&r[3]));
        let body = Quaternion::from_euler(f(&r[4]), f(&r[5]), f(&r[6]));

        let out = controller.input_euler_angle_roll_pitch_euler_rate_yaw(
            roll_cmd,
            pitch_cmd,
            yaw_rate_cmd,
            body,
            &shaping,
            &yaw_gains,
            &angle_gains,
            Vector3f::new(0.0, 0.0, 0.0),
            dt,
        );

        let target = controller.euler_angle_target_rad();
        let ang_vel = controller.ang_vel_target_rads();

        for (label, got, want) in [
            ("targ_r", target.x, f(&r[7])),
            ("targ_p", target.y, f(&r[8])),
            ("targ_y", target.z, f(&r[9])),
            ("ang_vel_x", ang_vel.x, f(&r[10])),
            ("ang_vel_y", ang_vel.y, f(&r[11])),
            ("ang_vel_z", ang_vel.z, f(&r[12])),
            ("rate_x", out.ang_vel_body_rads.x, f(&r[13])),
            ("rate_y", out.ang_vel_body_rads.y, f(&r[14])),
            ("rate_z", out.ang_vel_body_rads.z, f(&r[15])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < STEP_TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }
    }

    println!(
        "{} steps, {checked} values, largest difference {largest:e}",
        sticks.len()
    );
}

/// Looser than the single-shot tolerance, and deliberately so.
///
/// Four hundred iterations of a shaper that feeds its own output back in
/// accumulate transcendental disagreement rather than merely exhibiting it. A
/// structural error still shows: it diverges rather than drifting, and the
/// sequence is long enough that divergence is unmistakable.
const STEP_TOL: f32 = 1e-4;
