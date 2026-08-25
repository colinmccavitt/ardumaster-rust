//! Parity test: the motor output stage against upstream.
//!
//! Four sweeps in one fixture, each pinning a different piece of the path from
//! "the mixer decided this motor should run at 0.62" to "the ESC sees a
//! 1620 microsecond pulse":
//!
//! - `#pwm` — `output_to_pwm` over every spool state, both arming states, both
//!   `MOT_SAFE_DISARM` settings and three endpoint pairs.
//! - `#slew` — `set_actuator_with_slew` over a grid of slew times against a
//!   step in each direction, six iterations deep so the ramp is compared, not
//!   just its first step.
//! - `#idle` — `actuator_spin_up_to_ground_idle` across the ramp.
//! - `#pwmvalid` — `check_mot_pwm_params` over the endpoint pairs worth
//!   arguing about.
//!
//! The PWM sweep includes actuator values that land just under an integer
//! pulse width, because upstream computes a `float` and returns it through an
//! `int16_t`: the result truncates rather than rounds, and a port that rounded
//! would agree everywhere except exactly there.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::output::{
    actuator_spin_up_to_ground_idle, output_to_pwm, set_actuator_with_slew, PwmParams, SlewParams,
};
use ap_motors::spool::SpoolState;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motors_output.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    })
}

/// Rows of one `#name` sub-section, header and banner stripped.
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
        // The header is the one row whose first field is not numeric.
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

fn state_of(n: i32) -> SpoolState {
    match n {
        0 => SpoolState::ShutDown,
        1 => SpoolState::GroundIdle,
        2 => SpoolState::SpoolingUp,
        3 => SpoolState::ThrottleUnlimited,
        4 => SpoolState::SpoolingDown,
        _ => panic!("unknown spool state {n}"),
    }
}

#[test]
fn the_pwm_mapping_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "pwm");
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 7);
        let state = state_of(r[0].parse().expect("state"));
        let armed = r[1] == "1";
        let disarm_disable_pwm = r[2] == "1";
        let params = PwmParams {
            pwm_min: r[3].parse().expect("pwm_min"),
            pwm_max: r[4].parse().expect("pwm_max"),
            disarm_disable_pwm,
        };
        let actuator = f(&r[5]);
        let want: i16 = r[6].parse().expect("pwm");

        let got = output_to_pwm(state, armed, &params, actuator);
        assert_eq!(
            got, want,
            "state {state:?} armed={armed} safe_disarm={disarm_disable_pwm} \
             [{}, {}] actuator {actuator}",
            params.pwm_min, params.pwm_max
        );
        checked += 1;
    }

    println!("{checked} pwm mappings, all exact");
}

#[test]
fn the_actuator_slew_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "slew");
    let mut checked = 0_usize;

    // The fixture runs six iterations per case; rebuild the same run and
    // compare each step, so a limit computed from the destination rather than
    // the current output shows up on iteration two rather than hiding.
    let mut i = 0_usize;
    while i < rows.len() {
        let r = &rows[i];
        assert_eq!(r.len(), 7);
        let params = SlewParams {
            slew_up_time: f(&r[0]),
            slew_dn_time: f(&r[1]),
        };
        let dt = f(&r[2]);
        let start = f(&r[3]);
        let input = f(&r[4]);

        let mut out = start;
        for iter in 0..6 {
            let row = &rows[i + iter];
            assert_eq!(
                row[5].parse::<usize>().expect("iter"),
                iter,
                "fixture rows out of order at {i}"
            );
            set_actuator_with_slew(&mut out, input, dt, &params);
            let want = f(&row[6]);
            assert!(
                same(out, want),
                "up={} dn={} dt={dt} start={start} input={input} iter {iter}: \
                 {out} ({:#010x}) != upstream {want} ({:#010x})",
                params.slew_up_time,
                params.slew_dn_time,
                out.to_bits(),
                want.to_bits()
            );
            checked += 1;
        }
        i += 6;
    }

    println!("{checked} slew steps, all bit-exact");
}

#[test]
fn the_ground_idle_ramp_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "idle");

    for r in &rows {
        assert_eq!(r.len(), 3);
        let ratio = f(&r[0]);
        let spin_min = f(&r[1]);
        let want = f(&r[2]);
        let got = actuator_spin_up_to_ground_idle(ratio, spin_min);
        assert!(
            same(got, want),
            "ratio {ratio} spin_min {spin_min}: {got} ({:#010x}) != upstream \
             {want} ({:#010x})",
            got.to_bits(),
            want.to_bits()
        );
    }

    println!("{} ground-idle values, all bit-exact", rows.len());
}

#[test]
fn the_pwm_endpoint_check_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "pwmvalid");

    for r in &rows {
        assert_eq!(r.len(), 3);
        let params = PwmParams {
            pwm_min: r[0].parse().expect("pwm_min"),
            pwm_max: r[1].parse().expect("pwm_max"),
            disarm_disable_pwm: false,
        };
        let want = r[2] == "1";
        assert_eq!(
            params.valid(),
            want,
            "endpoints [{}, {}]",
            params.pwm_min,
            params.pwm_max
        );
    }
}

/// The ground-idle ramp scales by `SPIN_MIN`, which the fixture cannot vary.
///
/// `MOT_SPIN_MIN` lives inside the thrust linearisation, which a subclass of
/// `AP_MotorsMatrix` cannot reach, so the fixture only ever sees the default.
/// In the port it is an argument.
#[test]
fn the_ground_idle_ramp_scales_by_spin_min() {
    for spin_min in [0.0_f32, 0.05, 0.15, 0.3, 1.0] {
        assert!(same(actuator_spin_up_to_ground_idle(0.0, spin_min), 0.0));
        assert!(same(
            actuator_spin_up_to_ground_idle(1.0, spin_min),
            spin_min
        ));
        // The clamp is what stops a ramp that overshot by one iteration from
        // commanding more than idle.
        assert!(same(
            actuator_spin_up_to_ground_idle(1.7, spin_min),
            spin_min
        ));
        assert!(same(actuator_spin_up_to_ground_idle(-2.0, spin_min), 0.0));
    }
}

/// A slew time past the half-second cap is treated as the cap.
///
/// Upstream clamps inside the division, so an absurd `MOT_SLEW_UP_TIME` makes
/// the aircraft sluggish rather than unflyable. The fixture covers 2.0 s; this
/// states the property directly.
#[test]
fn an_absurd_slew_time_is_capped_rather_than_honoured() {
    let capped = SlewParams {
        slew_up_time: 0.5,
        slew_dn_time: 0.0,
    };
    let absurd = SlewParams {
        slew_up_time: 60.0,
        slew_dn_time: 0.0,
    };

    let mut a = 0.0_f32;
    let mut b = 0.0_f32;
    for _ in 0..40 {
        set_actuator_with_slew(&mut a, 1.0, 0.0025, &capped);
        set_actuator_with_slew(&mut b, 1.0, 0.0025, &absurd);
    }
    assert!(same(a, b), "60 s should behave as 0.5 s: {a} vs {b}");
}

/// Zero slew time means unlimited, not instantaneous-but-clamped.
#[test]
fn a_zero_slew_time_leaves_the_direction_unlimited() {
    let up_only = SlewParams {
        slew_up_time: 0.2,
        slew_dn_time: 0.0,
    };

    // Down is unlimited: one call lands on the input.
    let mut out = 1.0_f32;
    set_actuator_with_slew(&mut out, 0.0, 0.0025, &up_only);
    assert!(same(out, 0.0), "down should be unlimited, got {out}");

    // Up is limited: one call moves by dt/time, not to the input. Written as
    // the division rather than as 0.0125, because that is not what 0.0025/0.2
    // is in f32 -- it is 0.012499999, and a decimal literal here would be
    // asserting against a number the arithmetic never produces.
    let mut out = 0.0_f32;
    set_actuator_with_slew(&mut out, 1.0, 0.0025, &up_only);
    assert!(
        same(out, 0.0025_f32 / 0.2_f32),
        "up should be limited, got {out}"
    );
}
