//! AUTOTUNE_LEVEL / aggressiveness table stub.

use ap_autotune::level::{
    aggressiveness_target, constrain_autotune_level, tuning_row, AUTOTUNE_LEVEL_DEFAULT,
    AUTOTUNE_LEVEL_MAX, AUTOTUNE_LEVEL_MIN, LevelTarget, PITCH_TAU_SCALE, TUNING_TABLE, TuningRow,
};
use ap_autotune::AtType;

fn table_eq(got: TuningRow, tau: f32, rmax: f32) {
    assert!((got.tau - tau).abs() < 1e-6, "tau {} != {}", got.tau, tau);
    assert!((got.rmax - rmax).abs() < 1e-6, "rmax {} != {}", got.rmax, rmax);
}

#[test]
fn table_matches_upstream_rows() {
    assert_eq!(TUNING_TABLE.len(), 11);
    let expected = [
        (1.00, 20.0),
        (0.90, 30.0),
        (0.80, 40.0),
        (0.70, 50.0),
        (0.60, 60.0),
        (0.50, 75.0),
        (0.30, 90.0),
        (0.2, 120.0),
        (0.15, 160.0),
        (0.1, 210.0),
        (0.1, 300.0),
    ];
    for (i, &(tau, rmax)) in expected.iter().enumerate() {
        let Some(stored) = TUNING_TABLE.get(i) else {
            panic!("missing table row {i}");
        };
        table_eq(*stored, tau, rmax);
        table_eq(tuning_row((i as u8) + 1).expect("level in table"), tau, rmax);
    }
}

#[test]
fn level_zero_keeps_existing_rmax_and_tau() {
    assert_eq!(
        aggressiveness_target(0, AtType::Roll),
        LevelTarget::KeepExisting
    );
    assert_eq!(
        aggressiveness_target(0, AtType::Pitch),
        LevelTarget::KeepExisting
    );
    assert_eq!(tuning_row(0), None);
}

#[test]
fn default_level_is_six() {
    assert_eq!(AUTOTUNE_LEVEL_DEFAULT, 6);
    match aggressiveness_target(i32::from(AUTOTUNE_LEVEL_DEFAULT), AtType::Roll) {
        LevelTarget::Table(row) => table_eq(row, 0.50, 75.0),
        LevelTarget::KeepExisting => panic!("level 6 should hit the table"),
    }
}

#[test]
fn pitch_uses_fifty_percent_longer_tau() {
    assert!((PITCH_TAU_SCALE - 1.5).abs() < 1e-6);
    match aggressiveness_target(6, AtType::Pitch) {
        LevelTarget::Table(row) => table_eq(row, 0.50 * 1.5, 75.0),
        LevelTarget::KeepExisting => panic!("level 6 pitch should hit the table"),
    }
    match aggressiveness_target(1, AtType::Yaw) {
        LevelTarget::Table(row) => table_eq(row, 1.00, 20.0),
        LevelTarget::KeepExisting => panic!("level 1 yaw should hit the table"),
    }
}

#[test]
fn level_is_constrained_to_zero_through_eleven() {
    assert_eq!(AUTOTUNE_LEVEL_MIN, 0);
    assert_eq!(AUTOTUNE_LEVEL_MAX, 11);
    assert_eq!(constrain_autotune_level(-3), 0);
    assert_eq!(constrain_autotune_level(0), 0);
    assert_eq!(constrain_autotune_level(11), 11);
    assert_eq!(constrain_autotune_level(12), 11);
    match aggressiveness_target(99, AtType::Roll) {
        LevelTarget::Table(row) => table_eq(row, 0.1, 300.0),
        LevelTarget::KeepExisting => panic!("clamped 11 should hit the table"),
    }
    assert_eq!(
        aggressiveness_target(-1, AtType::Roll),
        LevelTarget::KeepExisting
    );
}
