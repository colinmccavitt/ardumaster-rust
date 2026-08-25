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

use ap_control::attitude_controller::ShapingConfig;
use ap_control::attitude_error::{
    thrust_heading_rotation_angles, update_ang_vel_target_from_att_error, AngleGains, YawLimitGains,
};
use ap_math::quaternion::Quaternion;
use ap_math::vector3::Vector3f;

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
                assert!(
                    c.len() >= 9,
                    "gains row has {} columns, need at least 9",
                    c.len()
                );
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
                assert!(
                    c.len() >= 17,
                    "gains row has {} columns, need at least 17",
                    c.len()
                );
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
        // Not exercised by this sequence, which commands a yaw *rate*.
        slew_yaw_max_rads: gains[13],
        rate_rp_tc: gains[14],
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
/// Iterating a shaper that feeds its own output back accumulates
/// transcendental disagreement rather than merely exhibiting it. A structural
/// error still shows: it diverges rather than drifting, and the sequences are
/// long enough that divergence is unmistakable.
///
/// Set from measurement: the stick sequence reaches 1.4e-5 over 400 steps and
/// the heading sequence 1.3e-6 over 800. The heading run is TIGHTER despite
/// being longer, because it keeps clear of the region where acos loses its
/// precision -- see the note on that test.
const STEP_TOL: f32 = 3e-5;

/// Parity: a heading command through the yaw-angle entry point, run with and
/// without slew limiting.
///
/// # Why the target starts away from the body
///
/// The first version of this sequence began with target == body, and diverged
/// at step 7. The cause was not the controller.
///
/// The decomposition takes `acosf` of the dot product of two thrust vectors.
/// For a small attitude error θ that is `cos θ ≈ 1 − θ²/2`, and in `f32`
/// anything within about 6e-8 of 1.0 *rounds to exactly 1.0* — so `acos`
/// returns exactly zero and the whole error contribution vanishes. Starting at
/// zero error meant creeping up through that region: for steps 0 to 6 `rate_x`
/// equalled `ang_vel_x` exactly, and at step 7 the contribution appeared
/// abruptly. A few ulp of accumulated difference decided which side of the
/// boundary each implementation landed on, so the comparison was measuring a
/// discontinuity rather than the controller.
///
/// The target now starts about 0.36 rad away from the body, four orders of
/// magnitude clear of it. Recorded because the next sequence written here will
/// have the same trap available: an attitude comparison that begins at zero
/// error spends its first steps in a region where `acos` has no precision
/// left.
#[test]
fn the_heading_command_matches_upstream() {
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
    let mut rows: Vec<Vec<String>> = Vec::new();

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
                assert!(
                    c.len() >= 16,
                    "gains row has {} columns, need at least 16",
                    c.len()
                );
                gains = c
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != 7 && *i != 11)
                    .map(|(_, s)| f(s))
                    .collect();
                ff_enabled = c[11] == "1";
            }
            "heading" => rows.push(c.iter().map(|s| (*s).to_owned()).collect()),
            _ => {}
        }
    }

    assert!(!rows.is_empty(), "no heading rows in the fixture");

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
        slew_yaw_max_rads: gains[13],
        rate_rp_tc: gains[14],
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
    let mut current_slew: Option<bool> = None;
    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    // Final heading target for each run, to prove the flag changed something.
    let mut final_yaw = [0.0_f32; 2];

    for r in &rows {
        assert_eq!(r.len(), 17, "malformed heading row");
        let slew = r[0] == "1";
        let step: usize = r[1].parse().expect("step");

        // A new run starts a fresh controller with the same offset target the
        // harness sets -- see the doc comment for why it is offset at all.
        if current_slew != Some(slew) {
            controller = AttitudeController::new();
            controller.set_attitude_target(Quaternion::from_euler(0.30, -0.20, 0.50));
            current_slew = Some(slew);
        }

        let body = Quaternion::from_euler(f(&r[5]), f(&r[6]), f(&r[7]));

        let out = controller.input_euler_angle_roll_pitch_yaw(
            f(&r[2]),
            f(&r[3]),
            f(&r[4]),
            slew,
            body,
            &shaping,
            &yaw_gains,
            &angle_gains,
            Vector3f::new(0.0, 0.0, 0.0),
            dt,
        );

        let target = controller.euler_angle_target_rad();
        let ang_vel = controller.ang_vel_target_rads();
        final_yaw[usize::from(slew)] = target.z;

        for (label, got, want) in [
            ("targ_r", target.x, f(&r[8])),
            ("targ_p", target.y, f(&r[9])),
            ("targ_y", target.z, f(&r[10])),
            ("ang_vel_x", ang_vel.x, f(&r[11])),
            ("ang_vel_y", ang_vel.y, f(&r[12])),
            ("ang_vel_z", ang_vel.z, f(&r[13])),
            ("rate_x", out.ang_vel_body_rads.x, f(&r[14])),
            ("rate_y", out.ang_vel_body_rads.y, f(&r[15])),
            ("rate_z", out.ang_vel_body_rads.z, f(&r[16])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < STEP_TOL,
                "slew={slew} step {step} {label}: {got} != upstream {want} \
                 (diff {diff})"
            );
            checked += 1;
        }
    }

    assert!(
        libm::fabsf(final_yaw[0] - final_yaw[1]) > 1e-3,
        "the two slew settings should not reach the same heading at the same \
         step, or the flag is doing nothing: {} vs {}",
        final_yaw[0],
        final_yaw[1]
    );

    println!(
        "{} heading steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// Parity: an euler-rate sequence, the acro-style entry point.
///
/// Every axis takes a rate here, so the shaping runs with a zero angle error
/// and the command carried as a desired velocity — a different path through
/// the same shaper than either angle entry point takes.
///
/// Roll and pitch use `rate_rp_tc` and yaw uses `rate_y_tc`; neither is
/// `input_tc`. Three constants for three cases, and a port that reached for
/// the wrong one would still converge to the right place, just with the wrong
/// feel. Only a step-by-step comparison sees that.
#[test]
fn the_euler_rate_sequence_matches_upstream() {
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
    let mut rows: Vec<Vec<String>> = Vec::new();

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
                assert!(
                    c.len() >= 17,
                    "gains row has {} columns, need at least 17",
                    c.len()
                );
                gains = c
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != 7 && *i != 11)
                    .map(|(_, s)| f(s))
                    .collect();
                ff_enabled = c[11] == "1";
            }
            "eulerrate" => rows.push(c.iter().map(|s| (*s).to_owned()).collect()),
            _ => {}
        }
    }

    assert!(!rows.is_empty(), "no euler-rate rows in the fixture");

    let shaping = ShapingConfig {
        input_tc: gains[8],
        rate_y_tc: gains[9],
        rate_rp_tc: gains[14],
        rate_bf_ff_enabled: ff_enabled,
        ang_vel_roll_max_degs: gains[10],
        ang_vel_pitch_max_degs: gains[11],
        ang_vel_yaw_max_degs: gains[12],
        accel_roll_max_radss: gains[3],
        accel_pitch_max_radss: gains[4],
        accel_yaw_max_radss: gains[5],
        slew_yaw_max_rads: gains[13],
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
    controller.set_attitude_target(Quaternion::from_euler(-0.25, 0.35, -0.40));

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 16, "malformed euler-rate row");
        let step: usize = r[0].parse().expect("step");
        let body = Quaternion::from_euler(f(&r[4]), f(&r[5]), f(&r[6]));

        let out = controller.input_euler_rate_roll_pitch_yaw(
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
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
        "{} euler-rate steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// The acro path: rates commanded directly in the body frame.
///
/// Distinct from the euler-rate sequence in more than units. The shaping runs
/// on the body-frame targets themselves rather than on Euler rates that are
/// then converted, so an error in which frame the acceleration limit belongs
/// to shows up here and nowhere else.
#[test]
fn the_body_rate_sequence_matches_upstream() {
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
    let mut rows: Vec<Vec<String>> = Vec::new();

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
                assert!(
                    c.len() >= 17,
                    "gains row has {} columns, need at least 17",
                    c.len()
                );
                gains = c
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != 7 && *i != 11)
                    .map(|(_, s)| f(s))
                    .collect();
                ff_enabled = c[11] == "1";
            }
            "bfrate" => rows.push(c.iter().map(|s| (*s).to_owned()).collect()),
            _ => {}
        }
    }

    assert!(!rows.is_empty(), "no body-rate rows in the fixture");
    assert!(
        ff_enabled,
        "the fixture has feedforward off, so this exercises the fallback path only"
    );

    let shaping = ShapingConfig {
        input_tc: gains[8],
        rate_y_tc: gains[9],
        rate_rp_tc: gains[14],
        rate_bf_ff_enabled: ff_enabled,
        ang_vel_roll_max_degs: gains[10],
        ang_vel_pitch_max_degs: gains[11],
        ang_vel_yaw_max_degs: gains[12],
        accel_roll_max_radss: gains[3],
        accel_pitch_max_radss: gains[4],
        accel_yaw_max_radss: gains[5],
        slew_yaw_max_rads: gains[13],
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
    controller.set_attitude_target(Quaternion::from_euler(0.40, 0.30, 1.10));

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 16, "malformed body-rate row");
        let step: usize = r[0].parse().expect("step");
        let body = Quaternion::from_euler(f(&r[4]), f(&r[5]), f(&r[6]));

        let out = controller.input_rate_bf_roll_pitch_yaw(
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
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
        "{} body-rate steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// Looser than `STEP_TOL`, and the reason is the algorithm rather than the port.
///
/// Both entry points obtain their thrust error as `acos` of a dot product of
/// unit vectors. Near alignment that is ill-conditioned: at an error angle of
/// theta the dot product is about `1 - theta^2/2`, so at theta = 0.01 rad it
/// sits a few hundred ulps below 1.0, while `d(acos)/d(dot) = 1/sin(theta)` is
/// about 100. One ulp of difference in the accumulated target therefore comes
/// out as roughly 1e-5 rad of angle.
///
/// The thrust-angle sequence drives the target through a minimum error of half
/// a degree, and every value exceeding `STEP_TOL` -- ten of 3600 -- falls in
/// the twenty-two steps around it. That region is a real flight condition, so
/// the sequence keeps it rather than steering around it, and the tolerance is
/// sized to what the conditioning permits.
///
/// The margin is still wide. Measured by applying each mutation with the bound
/// lifted and reading the largest difference over the whole sequence: the five
/// plausible mistakes checked against these tests diverge by 0.52 to 4.7, four
/// to five orders above this bound, against 2.8e-5 unmutated.
const THRUST_TOL: f32 = 1e-4;

/// The gains the harness configured, read once.
///
/// Every test had been repeating this block, and the repetition was not free:
/// adding a column for one test broke the length assertion in the others three
/// times. New tests read it from here. The five older ones still carry their
/// own copy; converting them is worth doing but is not this slice's work.
struct Gains {
    shaping: ShapingConfig,
    yaw: YawLimitGains,
    angle: AngleGains,
    dt: f32,
}

fn gains_and_rows(section: &str) -> (Gains, Vec<Vec<String>>) {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/attitude_error.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut current = "";
    let mut g: Vec<f32> = Vec::new();
    let mut ff_enabled = false;
    let mut rows: Vec<Vec<String>> = Vec::new();

    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        if current == "gains" {
            assert!(
                c.len() >= 17,
                "gains row has {} columns, need at least 17",
                c.len()
            );
            // Columns 7 and 11 are flags, not floats, so they are dropped and
            // every index below is into the filtered list.
            g = c
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != 7 && *i != 11)
                .map(|(_, s)| f(s))
                .collect();
            ff_enabled = c[11] == "1";
        } else if current == section {
            rows.push(c.iter().map(|s| (*s).to_owned()).collect());
        }
    }

    assert!(!rows.is_empty(), "no {section} rows in the fixture");
    assert!(
        ff_enabled,
        "the fixture has feedforward off, so only the fallback path is covered"
    );

    let gains = Gains {
        shaping: ShapingConfig {
            input_tc: g[8],
            rate_y_tc: g[9],
            rate_rp_tc: g[14],
            rate_bf_ff_enabled: ff_enabled,
            ang_vel_roll_max_degs: g[10],
            ang_vel_pitch_max_degs: g[11],
            ang_vel_yaw_max_degs: g[12],
            accel_roll_max_radss: g[3],
            accel_pitch_max_radss: g[4],
            accel_yaw_max_radss: g[5],
            slew_yaw_max_rads: g[13],
        },
        yaw: YawLimitGains {
            accel_yaw_max_radss: g[5],
            rate_yaw_kp: g[6],
            angle_yaw_kp: g[2],
            ..YawLimitGains::default()
        },
        angle: AngleGains {
            angle_p_roll: g[0],
            angle_p_pitch: g[1],
            angle_p_yaw: g[2],
            accel_roll_max_radss: g[3],
            accel_pitch_max_radss: g[4],
            accel_yaw_max_radss: g[5],
            use_sqrt_controller: true,
        },
        dt: g[7],
    };
    (gains, rows)
}

/// Compare one step's nine recorded outputs, given the column each sits in.
fn compare_step(
    step: usize,
    target: Vector3f,
    ang_vel: Vector3f,
    body_rate: Vector3f,
    row: &[String],
    first: usize,
    largest: &mut f32,
) -> usize {
    let got = [
        ("targ_r", target.x),
        ("targ_p", target.y),
        ("targ_y", target.z),
        ("ang_vel_x", ang_vel.x),
        ("ang_vel_y", ang_vel.y),
        ("ang_vel_z", ang_vel.z),
        ("rate_x", body_rate.x),
        ("rate_y", body_rate.y),
        ("rate_z", body_rate.z),
    ];
    for (i, (label, value)) in got.iter().enumerate() {
        let want = f(&row[first + i]);
        let diff = libm::fabsf(value - want);
        *largest = largest.max(diff);
        assert!(
            diff < THRUST_TOL,
            "step {step} {label}: {value} != upstream {want} (diff {diff})"
        );
    }
    got.len()
}

/// A thrust direction with yaw commanded as a rate — the position
/// controller's entry point.
///
/// Recorded twice, once per `slew_yaw` setting, because the flag chooses
/// between two different yaw limits and the commanded rate sits between them.
#[test]
fn the_thrust_vector_rate_heading_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("thrustrate");

    let mut controller = AttitudeController::new();
    let mut current_slew = -1_i32;
    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 18, "malformed thrust-rate row");
        let slew: i32 = r[0].parse().expect("slew");
        let step: usize = r[1].parse().expect("step");

        // Each pass is its own run; the harness rebuilt the probe.
        if slew != current_slew {
            current_slew = slew;
            controller = AttitudeController::new();
            controller.set_attitude_target(Quaternion::from_euler(0.10, -0.15, 0.60));
        }

        let thrust = Vector3f::new(f(&r[2]), f(&r[3]), f(&r[4]));
        let body = Quaternion::from_euler(f(&r[6]), f(&r[7]), f(&r[8]));

        let out = controller.input_thrust_vector_rate_heading(
            thrust,
            f(&r[5]),
            slew != 0,
            body,
            &g.shaping,
            &g.yaw,
            &g.angle,
            Vector3f::new(0.0, 0.0, 0.0),
            g.dt,
        );

        checked += compare_step(
            step,
            controller.euler_angle_target_rad(),
            controller.ang_vel_target_rads(),
            out.ang_vel_body_rads,
            r,
            9,
            &mut largest,
        );
    }

    println!(
        "{} thrust-rate steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// A thrust direction with yaw commanded as an angle plus a feedforward rate.
///
/// The only path whose yaw shaper receives an angle error and a rate at the
/// same time, so both are held non-zero throughout.
#[test]
fn the_thrust_vector_heading_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("thrustangle");

    let mut controller = AttitudeController::new();
    controller.set_attitude_target(Quaternion::from_euler(-0.20, 0.25, -0.50));

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 18, "malformed thrust-angle row");
        let step: usize = r[0].parse().expect("step");

        let thrust = Vector3f::new(f(&r[1]), f(&r[2]), f(&r[3]));
        let body = Quaternion::from_euler(f(&r[6]), f(&r[7]), f(&r[8]));

        let out = controller.input_thrust_vector_heading(
            thrust,
            f(&r[4]),
            f(&r[5]),
            body,
            &g.shaping,
            &g.yaw,
            &g.angle,
            Vector3f::new(0.0, 0.0, 0.0),
            g.dt,
        );

        checked += compare_step(
            step,
            controller.euler_angle_target_rad(),
            controller.ang_vel_target_rads(),
            out.ang_vel_body_rads,
            r,
            9,
            &mut largest,
        );
    }

    println!(
        "{} thrust-angle steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// A full attitude demand with a body-frame rate.
///
/// The demanded quaternion is compared as well as the controller's outputs,
/// because the call advances the caller's own quaternion. A port that returned
/// the right rates while integrating the demand differently would pass a test
/// that only looked at the rates, and would drift apart over a longer flight.
#[test]
fn the_quaternion_input_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("quatinput");

    let mut controller = AttitudeController::new();
    controller.set_attitude_target(Quaternion::from_euler(0.15, -0.30, 0.80));

    let mut desired = Quaternion::from_euler(-0.10, 0.20, -0.35);
    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 20, "malformed quaternion-input row");
        let step: usize = r[0].parse().expect("step");

        let body = Quaternion::from_euler(f(&r[8]), f(&r[9]), f(&r[10]));
        let w = Vector3f::new(f(&r[1]), f(&r[2]), f(&r[3]));

        let out = controller.input_quaternion(
            &mut desired,
            w,
            body,
            &g.shaping,
            &g.yaw,
            &g.angle,
            Vector3f::new(0.0, 0.0, 0.0),
            g.dt,
        );

        for (label, got, want) in [
            ("des_w", desired.q1, f(&r[4])),
            ("des_x", desired.q2, f(&r[5])),
            ("des_y", desired.q3, f(&r[6])),
            ("des_z", desired.q4, f(&r[7])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < THRUST_TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }

        checked += compare_step(
            step,
            controller.euler_angle_target_rad(),
            controller.ang_vel_target_rads(),
            out.ang_vel_body_rads,
            r,
            11,
            &mut largest,
        );
    }

    println!(
        "{} quaternion-input steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// The roll/pitch rate predictor, swept rather than stepped.
///
/// Compared with `dt` equal to the controller's own step, which is the one
/// case where the port and upstream must agree — see `D-025` and
/// [`the_rate_predictor_honours_its_dt`].
#[test]
fn the_rate_predictor_matches_upstream() {
    use ap_control::attitude_controller::{command_model_rate_predictor, Vector2Pair};
    use ap_math::vector2::Vector2f;

    let (g, rows) = gains_and_rows("predictor");

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 12, "malformed predictor row");
        let idx: usize = r[0].parse().expect("idx");

        let got = command_model_rate_predictor(
            Vector2f::new(f(&r[1]), f(&r[2])),
            Vector2Pair {
                ang_vel: Vector2f::new(f(&r[3]), f(&r[4])),
                ang_accel: Vector2f::new(f(&r[5]), f(&r[6])),
            },
            &g.shaping,
            &g.angle,
            f(&r[7]),
        );

        for (label, value, want) in [
            ("out_vel_x", got.ang_vel.x, f(&r[8])),
            ("out_vel_y", got.ang_vel.y, f(&r[9])),
            ("out_acc_x", got.ang_accel.x, f(&r[10])),
            ("out_acc_y", got.ang_accel.y, f(&r[11])),
        ] {
            let diff = libm::fabsf(value - want);
            largest = largest.max(diff);
            assert!(
                diff < STEP_TOL,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }
    }

    println!(
        "{} predictor rows, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// D-025: the port's predictor uses the `dt` it is given.
///
/// Upstream declares the parameter and never reads it — every internal call
/// substitutes the controller's own `_dt_s`. The one existing caller,
/// `AC_Loiter.cpp:191`, passes `get_dt_s()`, so upstream is correct today by
/// coincidence of that single call site rather than by construction.
///
/// This pins the divergence from both sides: a different `dt` must change the
/// answer, and the answer at the controller's own `dt` must still be the value
/// upstream recorded. Without the second half, "honours its dt" could be
/// satisfied by a predictor that had drifted away from upstream entirely.
#[test]
fn the_rate_predictor_honours_its_dt() {
    use ap_control::attitude_controller::{command_model_rate_predictor, Vector2Pair};
    use ap_math::vector2::Vector2f;

    let (g, rows) = gains_and_rows("predictor");

    // A row with a real error and a non-zero starting state, so the shaper has
    // something to integrate and dt can actually matter.
    let row = rows
        .iter()
        .find(|r| f(&r[1]) != 0.0 && f(&r[3]) != 0.0 && f(&r[5]) != 0.0)
        .expect("a row with error, rate and acceleration all non-zero");

    let error = Vector2f::new(f(&row[1]), f(&row[2]));
    let state = Vector2Pair {
        ang_vel: Vector2f::new(f(&row[3]), f(&row[4])),
        ang_accel: Vector2f::new(f(&row[5]), f(&row[6])),
    };
    let upstream_dt = f(&row[7]);

    let at_upstream_dt =
        command_model_rate_predictor(error, state, &g.shaping, &g.angle, upstream_dt);
    let recorded = Vector2f::new(f(&row[8]), f(&row[9]));

    assert!(
        libm::fabsf(at_upstream_dt.ang_vel.x - recorded.x) < STEP_TOL
            && libm::fabsf(at_upstream_dt.ang_vel.y - recorded.y) < STEP_TOL,
        "at the controller's own dt the port must still match upstream: \
         {:?} vs recorded {recorded:?}",
        at_upstream_dt.ang_vel
    );

    // Four times the step. Upstream would return the value above, unchanged,
    // because it never looks at the argument.
    let at_other_dt =
        command_model_rate_predictor(error, state, &g.shaping, &g.angle, upstream_dt * 4.0);

    let moved = libm::fabsf(at_other_dt.ang_vel.x - at_upstream_dt.ang_vel.x)
        + libm::fabsf(at_other_dt.ang_vel.y - at_upstream_dt.ang_vel.y);
    assert!(
        moved > 1e-4,
        "D-025: a different dt must change the prediction, but it moved by only {moved:e}. \
         Reproducing upstream's dead parameter is the defect this pins."
    );

    println!("D-025 pinned: quadrupling dt moves the prediction by {moved:e}");
}

/// Rate-only acro: the shaped rate goes straight to the rate loop.
///
/// The attitude controller never runs on this path, so what is compared is
/// `_ang_vel_body_rads` rather than a controller result — and the target is
/// checked too, because its only job here is to be coherent for the next mode.
#[test]
fn the_rate_only_acro_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("acro2");

    let mut controller = AttitudeController::new();

    // Deliberately start the target somewhere the recording never was. This
    // path assigns the target from the vehicle attitude every step, so a
    // correct port converges onto the recorded values immediately no matter
    // where it began. The harness AHRS never moves, so without this the
    // assignment and "leave the target alone" produce identical numbers and
    // the step is untested -- mutation testing found exactly that.
    controller.set_attitude_target(Quaternion::from_euler(0.4, -0.3, 1.2));

    let mut largest = 0.0_f32;
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 16, "malformed rate-only acro row");
        let step: usize = r[0].parse().expect("step");
        let body = Quaternion::from_euler(f(&r[4]), f(&r[5]), f(&r[6]));

        let out = controller.input_rate_bf_roll_pitch_yaw_2(
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
            body,
            &g.shaping,
            g.dt,
        );

        checked += compare_step(
            step,
            controller.euler_angle_target_rad(),
            controller.ang_vel_target_rads(),
            out,
            r,
            7,
            &mut largest,
        );
    }

    println!(
        "{} rate-only acro steps, {checked} values, largest difference {largest:e}",
        rows.len()
    );
}

/// Acro with integrated rate error — Plane's acro law.
///
/// The integrated error quaternion is compared as well as the rate output.
/// It is the whole state of this controller: get the integration subtly wrong
/// and the rates still look plausible for a while, then diverge.
#[test]
fn the_integrating_acro_matches_upstream() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("acro3");

    let mut controller = AttitudeController::new();
    let mut largest = 0.0_f32;
    let mut checked = 0_usize;
    let mut peak_err = 0.0_f32;

    for r in &rows {
        assert_eq!(r.len(), 20, "malformed integrating-acro row");
        let step: usize = r[0].parse().expect("step");
        let body = Quaternion::from_euler(f(&r[7]), f(&r[8]), f(&r[9]));
        let gyro = Vector3f::new(f(&r[4]), f(&r[5]), f(&r[6]));

        let out = controller.input_rate_bf_roll_pitch_yaw_3(
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
            body,
            &g.shaping,
            &g.angle,
            gyro,
            g.dt,
        );

        let err = controller.attitude_ang_error();
        peak_err = peak_err.max(err.to_axis_angle().length());

        for (label, got, want) in [
            ("err_w", err.q1, f(&r[13])),
            ("err_x", err.q2, f(&r[14])),
            ("err_y", err.q3, f(&r[15])),
            ("err_z", err.q4, f(&r[16])),
            ("targ_r", controller.euler_angle_target_rad().x, f(&r[10])),
            ("targ_p", controller.euler_angle_target_rad().y, f(&r[11])),
            ("targ_y", controller.euler_angle_target_rad().z, f(&r[12])),
            ("out_x", out.x, f(&r[17])),
            ("out_y", out.y, f(&r[18])),
            ("out_z", out.z, f(&r[19])),
        ] {
            let diff = libm::fabsf(got - want);
            largest = largest.max(diff);
            assert!(
                diff < THRUST_TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }
    }

    // The anti-windup clamp is the only thing bounding this integrator, so a
    // sequence that never approaches it has not tested the interesting half.
    let limit = 30.0_f32.to_radians();
    assert!(
        peak_err > limit * 0.9,
        "the integrated error peaked at {peak_err} rad, never reaching the \
         {limit} rad clamp — this sequence does not exercise the anti-windup"
    );

    println!(
        "{} integrating-acro steps, {checked} values, largest difference {largest:e}, \
         peak integrated error {peak_err:.4} rad against a {limit:.4} rad clamp",
        rows.len()
    );
}

/// The reset paths, which have no numerics to compare but do have invariants.
///
/// `inertial_frame_reset` is the one that matters: an EKF reset moves the
/// estimated attitude discontinuously while the aircraft does not move, so the
/// controller must see no change in its error. That is the property asserted
/// here, rather than a recorded value.
#[test]
fn the_reset_paths_hold_their_invariants() {
    use ap_control::attitude_controller::AttitudeController;

    let (g, rows) = gains_and_rows("acro3");
    let mut controller = AttitudeController::new();

    // Run a while so the controller carries real state rather than identity.
    for r in rows.iter().take(120) {
        let body = Quaternion::from_euler(f(&r[7]), f(&r[8]), f(&r[9]));
        controller.input_rate_bf_roll_pitch_yaw_3(
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
            body,
            &g.shaping,
            &g.angle,
            Vector3f::new(f(&r[4]), f(&r[5]), f(&r[6])),
            g.dt,
        );
    }

    // An EKF reset: the estimate jumps, the aircraft has not moved.
    let before = controller.attitude_ang_error();
    let jumped = Quaternion::from_euler(0.6, -0.4, 2.1);
    controller.inertial_frame_reset(jumped);

    let target = controller.attitude_target();
    let error_after = jumped.inverse() * target;
    for (label, got, want) in [
        ("w", error_after.q1, before.q1),
        ("x", error_after.q2, before.q2),
        ("y", error_after.q3, before.q3),
        ("z", error_after.q4, before.q4),
    ] {
        assert!(
            libm::fabsf(got - want) < 1e-5,
            "inertial_frame_reset changed the error in {label}: {got} != {want}. \
             The aircraft did not move, so the controller must not react."
        );
    }

    // reset_target_and_rate puts the target on the vehicle and, when asked,
    // clears the feedforward.
    let body = Quaternion::from_euler(-0.2, 0.35, 1.4);
    controller.reset_target_and_rate(body, false);
    let kept = controller.ang_vel_target_rads();
    assert!(
        kept.length() > 0.0,
        "reset_rate false must leave the feedforward alone so the rate loop keeps running"
    );

    controller.reset_target_and_rate(body, true);
    assert_eq!(
        controller.ang_vel_target_rads(),
        Vector3f::new(0.0, 0.0, 0.0),
        "reset_rate true must clear the feedforward"
    );
    let (r, p, y) = body.to_euler();
    let euler = controller.euler_angle_target_rad();
    assert!(
        libm::fabsf(euler.x - r) < 1e-6
            && libm::fabsf(euler.y - p) < 1e-6
            && libm::fabsf(euler.z - y) < 1e-6,
        "the target must land on the vehicle attitude"
    );

    // reset_yaw_target_and_rate moves heading only.
    controller.reset_target_and_rate(Quaternion::from_euler(0.3, -0.25, 0.5), true);
    let lean_before = controller.euler_angle_target_rad();
    controller.reset_yaw_target_and_rate(2.0, true);

    // Read the target itself, not the cached Euler form: this path rotates the
    // quaternion and deliberately leaves the cache stale until the next entry
    // point refreshes it. Asserting on the cache would assert the staleness.
    let (after_r, after_p, after_y) = controller.attitude_target().to_euler();
    let lean_after = Vector3f::new(after_r, after_p, after_y);

    assert!(
        libm::fabsf(lean_after.z - 2.0) < 1e-5,
        "heading should now be the vehicle's, got {}",
        lean_after.z
    );

    // And pin the staleness itself, so nobody "fixes" it into a divergence
    // without meaning to.
    assert!(
        libm::fabsf(controller.euler_angle_target_rad().z - lean_before.z) < 1e-9,
        "upstream leaves the cached Euler target untouched here; see the note          on reset_yaw_target_and_rate"
    );
    assert!(
        libm::fabsf(lean_after.x - lean_before.x) < 1e-5
            && libm::fabsf(lean_after.y - lean_before.y) < 1e-5,
        "roll and pitch must survive a heading reset: {lean_before:?} became {lean_after:?}"
    );
}
