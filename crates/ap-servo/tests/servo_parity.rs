//! Parity test for `SRV_Channel`'s output conversion (FW-018).
//!
//! These are pure functions of the channel's configuration, so they are
//! compared directly against upstream's compiled code rather than against a
//! flight. That is the stronger test here: a flight exercises whatever
//! configuration it happened to have, while the fixture sweeps the awkward
//! ones deliberately — a maximum below the minimum, a zero range, a trim
//! sitting on an endpoint so one half of the travel has no span at all.
//!
//! Every case is required to match exactly. There is no arithmetic here that
//! could legitimately differ: no transcendentals, no accumulated state, just
//! a scale and a truncating cast.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture fields whose count is checked first; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_servo::{OutputType, ServoChannel};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

#[test]
fn pwm_conversion_matches_upstream() {
    let path = fixtures_dir().join("servo_pwm.csv");
    let Ok(text) = std::fs::read_to_string(&path) else {
        eprintln!("skipping: servo_pwm.csv not present");
        return;
    };

    let mut checked = 0usize;
    let mut angle_mismatch = Vec::new();
    let mut range_mismatch = Vec::new();
    let mut degenerate = 0usize;

    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 8 {
            continue;
        }
        let servo_min: i32 = f[0].parse().expect("min");
        let servo_trim: i32 = f[1].parse().expect("trim");
        let servo_max: i32 = f[2].parse().expect("max");
        let high_out: i32 = f[3].parse().expect("high_out");
        let reversed = f[4] != "0";
        let scaled: f32 = f[5].parse().expect("scaled");
        let want_angle: u16 = f[6].parse().expect("pwm_angle");
        let want_range: u16 = f[7].parse().expect("pwm_range");

        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "the fixture carries the int16 parameters upstream holds"
        )]
        let mut ch = ServoChannel {
            servo_min: servo_min as u16,
            servo_max: servo_max as u16,
            servo_trim: servo_trim as u16,
            reversed,
            output_type: OutputType::Angle,
            #[allow(clippy::cast_sign_loss, reason = "high_out is non-negative")]
            high_out: high_out as u16,
        };

        let got_angle = ch.pwm_from_angle(scaled);
        if got_angle != want_angle && angle_mismatch.len() < 8 {
            angle_mismatch.push(format!(
                "angle min={servo_min} trim={servo_trim} max={servo_max} \
                 high={high_out} rev={reversed} v={scaled}: port {got_angle}, \
                 upstream {want_angle}"
            ));
        }

        ch.output_type = OutputType::Range;
        let got_range = ch.pwm_from_range(scaled);
        if got_range != want_range && range_mismatch.len() < 8 {
            range_mismatch.push(format!(
                "range min={servo_min} trim={servo_trim} max={servo_max} \
                 high={high_out} rev={reversed} v={scaled}: port {got_range}, \
                 upstream {want_range}"
            ));
        }

        // dispatch must agree with the branch it selects
        ch.output_type = OutputType::Angle;
        assert_eq!(ch.pwm_from_scaled_value(scaled), got_angle);
        ch.output_type = OutputType::Range;
        assert_eq!(ch.pwm_from_scaled_value(scaled), got_range);

        if servo_max <= servo_min || high_out == 0 {
            degenerate += 1;
        }
        checked += 1;
    }

    println!("{checked} conversions compared, {degenerate} of them degenerate");
    assert!(checked > 500, "too few cases compared: {checked}");
    assert!(
        degenerate > 50,
        "the fixture barely covers the misconfigured cases ({degenerate}), which \
         are the ones with early returns"
    );
    assert!(
        angle_mismatch.is_empty(),
        "{} angle conversion(s) disagree; first few:\n  {}",
        angle_mismatch.len(),
        angle_mismatch.join("\n  ")
    );
    assert!(
        range_mismatch.is_empty(),
        "{} range conversion(s) disagree; first few:\n  {}",
        range_mismatch.len(),
        range_mismatch.join("\n  ")
    );
}

/// PORT-DERIVED: upstream has no unit test for this, and the fixture cannot
/// express it — that a full-scale demand reaches exactly the endpoint, rather
/// than one microsecond short of it, is the property a surface actually
/// depends on.
#[test]
fn full_deflection_reaches_the_endpoints() {
    let ch = ServoChannel::angle(1000, 1500, 2000, 4500);
    assert_eq!(ch.pwm_from_angle(4500.0), 2000);
    assert_eq!(ch.pwm_from_angle(-4500.0), 1000);
    assert_eq!(ch.pwm_from_angle(0.0), 1500);

    let ch = ServoChannel::range(1000, 2000, 100);
    assert_eq!(ch.pwm_from_range(100.0), 2000);
    assert_eq!(ch.pwm_from_range(0.0), 1000);
}

/// PORT-DERIVED. An off-centre trim scales each half independently, so both
/// endpoints are still reached at full deflection — the surface does not lose
/// travel on one side because it was trimmed toward the other.
#[test]
fn an_off_centre_trim_still_reaches_both_endpoints() {
    let ch = ServoChannel::angle(1000, 1200, 2000, 4500);
    assert_eq!(ch.pwm_from_angle(4500.0), 2000);
    assert_eq!(ch.pwm_from_angle(-4500.0), 1000);
    assert_eq!(ch.pwm_from_angle(0.0), 1200);
    // and the halves have genuinely different gains
    assert_eq!(ch.pwm_from_angle(2250.0), 1600);
    assert_eq!(ch.pwm_from_angle(-2250.0), 1100);
}
