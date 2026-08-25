//! Parity test: the motor library's channel outputs against upstream.
//!
//! Three sweeps: `update_throttle_range` over every output type and both
//! digital-output states, and the boost-throttle and roll/pitch/yaw/thrust
//! channel values read back out of `SRV_Channels`.
//!
//! # This test could not exist before the harness linked the real firmware
//!
//! With `SRV_Channels` stubbed, `set_output_scaled` did nothing and there was
//! nothing to read back — so the boost and rpyt outputs could only have been
//! compared against numbers the harness itself produced. And
//! `have_digital_outputs` returning zero meant `update_throttle_range` took
//! the analog branch every time, leaving the digital branch untested while the
//! sweep still looked complete.
//!
//! The digital sweep also pins something easy to get backwards:
//! `have_digital_outputs` asks which output *channels* are digital, not what
//! `MOT_PWM_TYPE` says. A DShot vehicle whose channels are not marked digital
//! still takes the analog branch — visible in the fixture as types 4 through 7
//! keeping their configured endpoints while `digital` is 0.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::output::{
    boost_throttle_output, rpyt_outputs, update_throttle_range, PwmParams, PwmType,
};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motors_channels.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    })
}

fn section(text: &str, name: &str) -> Vec<Vec<String>> {
    let mut rows = Vec::new();
    let mut inside = false;
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            inside = tag == name;
            continue;
        }
        if !inside || line.is_empty() {
            continue;
        }
        if line
            .split(',')
            .next()
            .is_some_and(|f| f.parse::<f64>().is_err())
        {
            continue;
        }
        rows.push(line.split(',').map(str::to_owned).collect());
    }
    assert!(!rows.is_empty(), "fixture section #{name} is empty");
    rows
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

fn pwm_type_of(n: i32) -> PwmType {
    match n {
        0 => PwmType::Normal,
        1 => PwmType::OneShot,
        2 => PwmType::OneShot125,
        3 => PwmType::Brushed,
        4 => PwmType::DShot150,
        5 => PwmType::DShot300,
        6 => PwmType::DShot600,
        7 => PwmType::DShot1200,
        8 => PwmType::PwmRange,
        9 => PwmType::PwmAngle,
        _ => panic!("unknown PWM type {n}"),
    }
}

#[test]
fn the_throttle_range_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "range");
    let mut digital_seen = [false, false];

    for r in &rows {
        assert_eq!(r.len(), 6);
        let pwm_type = pwm_type_of(r[0].parse().expect("pwm_type"));
        let digital = r[1] == "1";
        digital_seen[usize::from(digital)] = true;

        let start_min: i16 = r[2].parse().expect("start_min");
        let start_max: i16 = r[3].parse().expect("start_max");
        let want_min: i16 = r[4].parse().expect("end_min");
        let want_max: i16 = r[5].parse().expect("end_max");

        let mut params = PwmParams {
            pwm_min: start_min,
            pwm_max: start_max,
            disarm_disable_pwm: false,
            pwm_min_default: start_min,
            pwm_max_default: start_max,
        };
        let (got_min, got_max) = update_throttle_range(&mut params, pwm_type, digital);

        assert_eq!(
            (got_min, got_max),
            (want_min, want_max),
            "{pwm_type:?} digital={digital}: endpoints"
        );
        assert_eq!(
            (params.pwm_min, params.pwm_max),
            (want_min, want_max),
            "{pwm_type:?} digital={digital}: the parameters themselves"
        );

        // set_and_default moves the default with the value, so after a change
        // the two agree; after no change the default is left alone.
        let changed = (want_min, want_max) != (start_min, start_max);
        let expected_default = if changed {
            (want_min, want_max)
        } else {
            (start_min, start_max)
        };
        assert_eq!(
            (params.pwm_min_default, params.pwm_max_default),
            expected_default,
            "{pwm_type:?} digital={digital}: parameter defaults"
        );
    }

    assert!(
        digital_seen[0] && digital_seen[1],
        "the fixture must cover both digital-output states, got {digital_seen:?}"
    );
    println!("{} throttle-range cases, all exact", rows.len());
}

#[test]
fn the_boost_throttle_output_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "boost");

    for r in &rows {
        assert_eq!(r.len(), 3);
        let boost_scale = f(&r[0]);
        let throttle = f(&r[1]);
        let want = f(&r[2]);

        let got = boost_throttle_output(throttle, boost_scale);
        assert!(
            same(got, want),
            "scale {boost_scale} throttle {throttle}: {got} ({:#010x}) != \
             upstream {want} ({:#010x})",
            got.to_bits(),
            want.to_bits()
        );
    }

    println!("{} boost-throttle values, all bit-exact", rows.len());
}

#[test]
fn the_rpyt_outputs_match_upstream() {
    let text = fixture();
    let rows = section(&text, "rpyt");

    for r in &rows {
        assert_eq!(r.len(), 8);
        let (roll, pitch, yaw, throttle) = (f(&r[0]), f(&r[1]), f(&r[2]), f(&r[3]));
        let got = rpyt_outputs(roll, pitch, yaw, throttle);
        let want = (f(&r[4]), f(&r[5]), f(&r[6]), f(&r[7]));

        for (label, g, w) in [
            ("roll_out", got.0, want.0),
            ("pitch_out", got.1, want.1),
            ("yaw_out", got.2, want.2),
            ("thrust_out", got.3, want.3),
        ] {
            assert!(
                same(g, w),
                "r={roll} p={pitch} y={yaw} t={throttle} {label}: {g} \
                 ({:#010x}) != upstream {w} ({:#010x})",
                g.to_bits(),
                w.to_bits()
            );
        }
    }

    println!("{} rpyt rows, all bit-exact", rows.len());
}

/// A boost scale of zero or less writes zero rather than skipping the write.
///
/// Skipping would leave the channel holding whatever it had, which on a
/// vehicle with no boost motor configured is not obviously wrong until
/// something else has written to that channel.
#[test]
fn no_boost_motor_writes_zero_rather_than_nothing() {
    for scale in [-5.0_f32, -0.001, 0.0] {
        for throttle in [0.0_f32, 0.5, 1.0] {
            assert!(
                same(boost_throttle_output(throttle, scale), 0.0),
                "scale {scale} throttle {throttle} should write zero"
            );
        }
    }
}

/// The boost output saturates before scaling to thousandths, not after.
#[test]
fn the_boost_output_clamps_before_it_is_scaled() {
    // 0.9 * 5.0 = 4.5, clamped to 1.0, then * 1000.
    assert!(same(boost_throttle_output(0.9, 5.0), 1000.0));
    // Half throttle at unity scale is half of full range.
    assert!(same(boost_throttle_output(0.5, 1.0), 500.0));
}
