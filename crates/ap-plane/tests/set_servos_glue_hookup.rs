//! set_servos calc_throttle glue: landing and mode-entry throttle guards.

use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::mode_run::PilotThrottleSource;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
use ap_plane::set_servos_glue_hookup::{set_servos_calc_throttle_tick, SetServosGlueInputs};
use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

fn base_servos(throttle: f32) -> ServoOutputState {
    ServoOutputState {
        throttle_scaled: throttle,
        ..ServoOutputState::default()
    }
}

fn auto_inputs() -> SetServosGlueInputs {
    SetServosGlueInputs {
        control_mode: ModeNumber::Auto.as_number(),
        features: BuildFeatures::default(),
        tecs_throttle_demand: 75.0,
        throttle_nudge: 0,
        landing_throttle_applied: false,
        disarm_throttle_applied: false,
        mode_entry_applied: false,
        mode_glue_throttle_restored: false,
        pilot_throttle: PilotThrottleGlueInputs::default(),
    }
}

#[test]
fn landing_guard_skips_calc_throttle_in_auto() {
    let inp = SetServosGlueInputs {
        landing_throttle_applied: true,
        ..auto_inputs()
    };
    let out = set_servos_calc_throttle_tick(base_servos(0.0), &inp);
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
}

#[test]
fn mode_entry_guard_skips_calc_throttle() {
    let inp = SetServosGlueInputs {
        mode_entry_applied: true,
        ..auto_inputs()
    };
    let out = set_servos_calc_throttle_tick(base_servos(0.0), &inp);
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
}

#[test]
fn mode_restore_guard_skips_calc_throttle() {
    let inp = SetServosGlueInputs {
        mode_glue_throttle_restored: true,
        ..auto_inputs()
    };
    let out = set_servos_calc_throttle_tick(base_servos(55.0), &inp);
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 55.0);
}

#[test]
fn disarm_guard_skips_calc_throttle() {
    let inp = SetServosGlueInputs {
        disarm_throttle_applied: true,
        ..auto_inputs()
    };
    let out = set_servos_calc_throttle_tick(base_servos(0.0), &inp);
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
}

#[test]
fn auto_mode_applies_tecs_throttle_when_guards_clear() {
    let out = set_servos_calc_throttle_tick(base_servos(10.0), &auto_inputs());
    assert!(out.applied);
    assert!((out.servos.throttle_scaled - 75.0).abs() < 1e-6);
}

#[test]
fn calc_throttle_does_not_zero_existing_throttle() {
    let inp = SetServosGlueInputs {
        control_mode: ModeNumber::Manual.as_number(),
        tecs_throttle_demand: 0.0,
        pilot_throttle: PilotThrottleGlueInputs {
            throttle_pwm: Some(1000),
            throttle_cfg: RcChannelConfig {
                radio_min: 1000,
                radio_max: 2000,
                ..Default::default()
            },
            pilot_throttle_source: PilotThrottleSource::Direct,
            ..Default::default()
        },
        ..auto_inputs()
    };
    let out = set_servos_calc_throttle_tick(base_servos(40.0), &inp);
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 40.0);
}
