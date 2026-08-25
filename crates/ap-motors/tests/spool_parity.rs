//! Parity test: the spool state machine against upstream.
//!
//! Eight scripted flights driven through both `output_logic` and the port, one
//! 400 Hz step at a time, comparing every piece of state the machine owns at
//! every step. 25,200 steps, no tolerance anywhere.
//!
//! Stepping rather than sampling is the point. A state machine can agree at
//! the endpoints and disagree in the middle — a transition taken one iteration
//! early, a ramp that starts from the wrong value and catches up — and only a
//! step-by-step comparison sees that. Every scenario here ends up somewhere
//! unremarkable; what is being checked is the path.
//!
//! # What the scenarios cover
//!
//! 0. The whole arc: arm, idle, spool up, fly, spool down, shut down.
//! 1. Interlock dropped in flight — must slam to shut down with no ramp.
//! 2. `MOT_SPOOL_TIME` below the minimum, which upstream writes back into the
//!    parameter rather than into a local. The last column tracks that.
//! 3. The disarm-PWM safe-time window gating the exit from shut down.
//! 4. A failed-motor thrust boost slewing in while unbalanced, then out.
//! 5. Reversal: back to `THROTTLE_UNLIMITED` part-way through spooling down.
//! 6. Asymmetric ramps — two seconds down, a fifth of a second up.
//! 7. Disarmed mid-flight, which also resets the safe timer.
//!
//! # What it does not cover
//!
//! The current-limited ceiling. `get_current_limit_max_throttle` binds the
//! battery singleton before it checks whether limiting is even enabled, and
//! the harness has no battery, so the scenarios leave `MOT_BAT_CURR_MAX` at 0
//! and the ceiling is always 1.0. The port takes that ceiling as an explicit
//! input, so the paths that use it are covered by unit tests below instead.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::spool::{DesiredSpoolState, Spool, SpoolInputs, SpoolParams, SpoolState};

/// `AP_MOTORS_SPIN_MIN_DEFAULT`. The scenarios leave `MOT_SPIN_MIN` alone —
/// it lives inside the thrust linearisation, which the probe cannot reach —
/// and vary `MOT_SPIN_ARM` instead, which moves the same ratio.
const SPIN_MIN: f32 = 0.15;

/// Current limiting is off in every scenario, so the ceiling is always 1.0.
const CURRENT_LIMIT_MAX_THROTTLE: f32 = 1.0;

const DT: f32 = 0.0025;
const NEVER: f32 = 1.0e9;

/// Mirrors the table in `tools/parity/gen_motors_fixture.py`.
struct Scenario {
    steps: usize,
    spool_up_time: f32,
    spool_down_time: f32,
    safe_time: f32,
    spin_arm: f32,
    idle_time_delay: f32,
    disarm_disable_pwm: bool,
    throttle: f32,
    arm_at: f32,
    disarm_at: f32,
    interlock_at: f32,
    interlock_off_at: f32,
    desired_ul_at: f32,
    desired_down_at: f32,
    desired_gi_at: f32,
    clear_block_at: f32,
    boost_at: f32,
    unbalanced_at: f32,
}

const SCENARIOS: &[Scenario] = &[
    Scenario {
        steps: 4000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay: 0.5,
        disarm_disable_pwm: false,
        throttle: 0.5,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: 6.0,
        desired_gi_at: NEVER,
        clear_block_at: 1.5,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 3000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay: 0.2,
        disarm_disable_pwm: false,
        throttle: 0.7,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: 4.0,
        desired_ul_at: 0.05,
        desired_down_at: NEVER,
        desired_gi_at: NEVER,
        clear_block_at: 1.0,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 1200,
        spool_up_time: 0.01,
        spool_down_time: 0.0,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay: 0.1,
        disarm_disable_pwm: false,
        throttle: 0.4,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: NEVER,
        desired_gi_at: NEVER,
        clear_block_at: 0.5,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 2000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 1.0,
        spin_arm: 0.10,
        idle_time_delay: 0.2,
        disarm_disable_pwm: true,
        throttle: 0.4,
        arm_at: 0.5,
        disarm_at: NEVER,
        interlock_at: 0.5,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: NEVER,
        desired_gi_at: NEVER,
        clear_block_at: 2.0,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 4000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay: 0.2,
        disarm_disable_pwm: false,
        throttle: 0.6,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: NEVER,
        desired_gi_at: NEVER,
        clear_block_at: 1.0,
        boost_at: 3.0,
        unbalanced_at: 3.0,
    },
    Scenario {
        steps: 4000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay: 0.2,
        disarm_disable_pwm: false,
        throttle: 0.5,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: 5.0,
        desired_gi_at: NEVER,
        clear_block_at: 1.0,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 4000,
        spool_up_time: 0.2,
        spool_down_time: 2.0,
        safe_time: 0.0,
        spin_arm: 0.20,
        idle_time_delay: 0.3,
        disarm_disable_pwm: false,
        throttle: 0.8,
        arm_at: 0.0,
        disarm_at: NEVER,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: 5.0,
        desired_gi_at: 7.0,
        clear_block_at: 1.0,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
    Scenario {
        steps: 3000,
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.5,
        spin_arm: 0.10,
        idle_time_delay: 0.2,
        disarm_disable_pwm: true,
        throttle: 0.5,
        arm_at: 0.0,
        disarm_at: 4.0,
        interlock_at: 0.0,
        interlock_off_at: NEVER,
        desired_ul_at: 0.05,
        desired_down_at: NEVER,
        desired_gi_at: NEVER,
        clear_block_at: 1.5,
        boost_at: NEVER,
        unbalanced_at: NEVER,
    },
];

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/spool_parity.csv"))
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

fn state_num(s: SpoolState) -> i32 {
    match s {
        SpoolState::ShutDown => 0,
        SpoolState::GroundIdle => 1,
        SpoolState::SpoolingUp => 2,
        SpoolState::ThrottleUnlimited => 3,
        SpoolState::SpoolingDown => 4,
    }
}

fn desired_num(d: DesiredSpoolState) -> i32 {
    match d {
        DesiredSpoolState::ShutDown => 0,
        DesiredSpoolState::GroundIdle => 1,
        DesiredSpoolState::ThrottleUnlimited => 2,
    }
}

#[test]
fn the_spool_state_machine_matches_upstream() {
    let text = fixture();
    let mut rows: Vec<Vec<&str>> = Vec::new();
    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("scenario,") {
            continue;
        }
        rows.push(line.split(',').collect());
    }

    let expected_rows: usize = SCENARIOS.iter().map(|s| s.steps).sum();
    assert_eq!(
        rows.len(),
        expected_rows,
        "the fixture and the test disagree about the scenario table"
    );

    let mut cursor = 0_usize;
    let mut compared = 0_usize;

    for (sc, s) in SCENARIOS.iter().enumerate() {
        let mut params = SpoolParams {
            spool_up_time: s.spool_up_time,
            spool_down_time: s.spool_down_time,
            safe_time: s.safe_time,
            spin_arm: s.spin_arm,
            idle_time_delay_s: s.idle_time_delay,
            disarm_disable_pwm: s.disarm_disable_pwm,
        };
        let mut spool = Spool::new();
        // The harness starts each scenario with limits set, matching a machine
        // that has just come out of reset in SHUT_DOWN.

        for step in 0..s.steps {
            #[expect(
                clippy::cast_precision_loss,
                reason = "step counts here are in the thousands; f32 is exact \
well past that, and the harness computes the same product the same way"
            )]
            let tsec = step as f32 * DT;

            spool.set_desired(if tsec >= s.desired_ul_at && tsec < s.desired_down_at {
                DesiredSpoolState::ThrottleUnlimited
            } else if tsec >= s.desired_gi_at {
                DesiredSpoolState::GroundIdle
            } else {
                DesiredSpoolState::ShutDown
            });
            if tsec >= s.boost_at {
                spool.set_thrust_boost(true);
            }
            if tsec >= s.clear_block_at {
                spool.set_spoolup_block(false);
            }

            let input = SpoolInputs {
                armed: tsec >= s.arm_at && tsec < s.disarm_at,
                interlock: tsec >= s.interlock_at && tsec < s.interlock_off_at,
                dt_s: DT,
                spin_min: SPIN_MIN,
                throttle: s.throttle,
                current_limit_max_throttle: CURRENT_LIMIT_MAX_THROTTLE,
                thrust_balanced: tsec < s.unbalanced_at,
            };

            spool.update(&mut params, &input);

            let r = &rows[cursor];
            cursor += 1;
            assert_eq!(r.len(), 15, "scenario {sc} step {step}: malformed row");
            assert_eq!(r[0].parse::<usize>().expect("scenario"), sc);
            assert_eq!(r[1].parse::<usize>().expect("step"), step);

            let where_ = format!("scenario {sc} step {step} (t={tsec:.4}s)");

            assert_eq!(
                state_num(spool.state()),
                r[2].parse::<i32>().expect("state"),
                "{where_}: spool state"
            );
            assert_eq!(
                desired_num(spool.desired()),
                r[3].parse::<i32>().expect("desired"),
                "{where_}: desired state"
            );

            for (label, got, want) in [
                ("spin_up_ratio", spool.spin_up_ratio(), f(r[4])),
                ("throttle_thrust_max", spool.throttle_thrust_max(), f(r[5])),
                ("thrust_boost_ratio", spool.thrust_boost_ratio(), f(r[11])),
                // Upstream clamps MOT_SPOOL_TIME back into the parameter, so
                // this column is checking a write-back, not a read.
                ("spool_up_time", params.spool_up_time, f(r[14])),
            ] {
                assert!(
                    same(got, want),
                    "{where_} {label}: {got} ({:#010x}) != upstream {want} ({:#010x})",
                    got.to_bits(),
                    want.to_bits()
                );
                compared += 1;
            }

            assert_eq!(
                i32::from(spool.spoolup_block()),
                r[9].parse::<i32>().expect("spoolup_block"),
                "{where_}: spoolup block"
            );
            assert_eq!(
                i32::from(spool.thrust_boost()),
                r[10].parse::<i32>().expect("thrust_boost"),
                "{where_}: thrust boost"
            );
            assert_eq!(
                i32::from(spool.limits().roll),
                r[12].parse::<i32>().expect("limit_roll"),
                "{where_}: roll limit"
            );
            assert_eq!(
                i32::from(spool.limits().throttle_lower),
                r[13].parse::<i32>().expect("limit_throttle_lower"),
                "{where_}: lower throttle limit"
            );
            compared += 6;
        }
    }

    println!("{cursor} steps, {compared} values, all bit-exact");
}

/// Every scenario reaches a state the machine is supposed to reach.
///
/// Without this, a bug that pinned the machine in `SHUT_DOWN` would still pass
/// the parity test — as long as the port were pinned the same way, which a
/// shared misreading of the source would do. Asserting the scenarios go
/// somewhere keeps the fixture from agreeing about nothing.
#[test]
fn the_scenarios_actually_exercise_every_state() {
    let mut seen = [false; 5];

    for s in SCENARIOS {
        let mut params = SpoolParams {
            spool_up_time: s.spool_up_time,
            spool_down_time: s.spool_down_time,
            safe_time: s.safe_time,
            spin_arm: s.spin_arm,
            idle_time_delay_s: s.idle_time_delay,
            disarm_disable_pwm: s.disarm_disable_pwm,
        };
        let mut spool = Spool::new();

        for step in 0..s.steps {
            #[expect(clippy::cast_precision_loss, reason = "see the parity test")]
            let tsec = step as f32 * DT;

            spool.set_desired(if tsec >= s.desired_ul_at && tsec < s.desired_down_at {
                DesiredSpoolState::ThrottleUnlimited
            } else if tsec >= s.desired_gi_at {
                DesiredSpoolState::GroundIdle
            } else {
                DesiredSpoolState::ShutDown
            });
            if tsec >= s.boost_at {
                spool.set_thrust_boost(true);
            }
            if tsec >= s.clear_block_at {
                spool.set_spoolup_block(false);
            }

            spool.update(
                &mut params,
                &SpoolInputs {
                    armed: tsec >= s.arm_at && tsec < s.disarm_at,
                    interlock: tsec >= s.interlock_at && tsec < s.interlock_off_at,
                    dt_s: DT,
                    spin_min: SPIN_MIN,
                    throttle: s.throttle,
                    current_limit_max_throttle: CURRENT_LIMIT_MAX_THROTTLE,
                    thrust_balanced: tsec < s.unbalanced_at,
                },
            );

            #[expect(
                clippy::indexing_slicing,
                reason = "state_num returns 0..=4 by construction"
            )]
            {
                seen[state_num(spool.state()) as usize] = true;
            }
        }
    }

    assert!(seen.iter().all(|&b| b), "states reached: {seen:?}");
}

/// The current-limited ceiling, which the fixture cannot reach.
///
/// `get_current_limit_max_throttle` binds the battery singleton before it
/// checks whether limiting is enabled, so the harness — which has no battery —
/// can only run with limiting off. In the port the ceiling is an input, so it
/// can simply be set.
#[test]
fn the_ceiling_follows_the_current_limit() {
    const LIMIT: f32 = 0.62;

    let mut params = SpoolParams {
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay_s: 0.1,
        disarm_disable_pwm: false,
    };
    let mut spool = Spool::new();

    let input = |limit: f32| SpoolInputs {
        armed: true,
        interlock: true,
        dt_s: DT,
        spin_min: SPIN_MIN,
        // A demand above the limit, so the limit is what binds.
        throttle: 0.95,
        current_limit_max_throttle: limit,
        thrust_balanced: true,
    };

    spool.set_desired(DesiredSpoolState::ThrottleUnlimited);
    for _ in 0..2000 {
        spool.set_spoolup_block(false);
        spool.update(&mut params, &input(LIMIT));
    }

    assert_eq!(spool.state(), SpoolState::ThrottleUnlimited);
    assert!(
        same(spool.throttle_thrust_max(), LIMIT),
        "ceiling {} should be the current limit {LIMIT}",
        spool.throttle_thrust_max()
    );

    // Tightening the limit moves the ceiling with it, with no ramp: in
    // THROTTLE_UNLIMITED the ceiling is assigned, not slewed.
    spool.update(&mut params, &input(0.4));
    assert!(same(spool.throttle_thrust_max(), 0.4));
}

/// A `MOT_SPOOL_TIME` below the floor is written back into the parameter.
///
/// Upstream calls `.set()` on the `AP_Float`, not on a local, so the clamp is
/// visible to everything that reads the parameter afterwards — a GCS included.
/// The port takes the parameters by `&mut` for exactly this reason, and this
/// pins the behaviour independently of the fixture.
#[test]
fn a_too_short_spool_time_is_clamped_in_the_parameter() {
    let mut params = SpoolParams {
        spool_up_time: 0.004,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay_s: 0.1,
        disarm_disable_pwm: false,
    };
    let mut spool = Spool::new();

    spool.update(
        &mut params,
        &SpoolInputs {
            armed: true,
            interlock: true,
            dt_s: DT,
            spin_min: SPIN_MIN,
            throttle: 0.5,
            current_limit_max_throttle: 1.0,
            thrust_balanced: true,
        },
    );

    assert!(
        same(params.spool_up_time, 0.05),
        "the parameter should have been clamped, got {}",
        params.spool_up_time
    );
}

/// With no `SPIN_MIN` the idle target is zero rather than a division by it.
#[test]
fn a_zero_spin_min_does_not_divide_by_zero() {
    let mut params = SpoolParams {
        spool_up_time: 0.5,
        spool_down_time: 0.5,
        safe_time: 0.0,
        spin_arm: 0.10,
        idle_time_delay_s: 0.1,
        disarm_disable_pwm: false,
    };
    let mut spool = Spool::new();
    spool.set_desired(DesiredSpoolState::GroundIdle);

    for _ in 0..400 {
        spool.update(
            &mut params,
            &SpoolInputs {
                armed: true,
                interlock: true,
                dt_s: DT,
                spin_min: 0.0,
                throttle: 0.0,
                current_limit_max_throttle: 1.0,
                thrust_balanced: true,
            },
        );
        assert!(
            spool.spin_up_ratio().is_finite(),
            "spin ratio went non-finite with SPIN_MIN at zero"
        );
    }

    assert!(same(spool.spin_up_ratio(), 0.0));
}
