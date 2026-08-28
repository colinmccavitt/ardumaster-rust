//! AUTOTUNE_LEVEL aggressiveness table, upstream `tuning_table`.
//!
//! Plane-4.7.0 `AP_AutoTune::update_rmax` looks up a `(tau, rmax)` pair
//! from `AUTOTUNE_LEVEL` (1..=11). Level 0 keeps the existing `RMAX` and
//! `TCONST` values and only tunes the PID terms. Pitch uses a 50% longer
//! time constant than the table row.
//!
//! The gradual step toward the target (`±20 deg/s` on `rmax`, `±15%` on
//! `tau`) and the FF/I inverse-tau clamp are a later slice.

use crate::state::AtType;

/// Pitch time-constant scale, upstream `target_tau *= 1.5`.
pub const PITCH_TAU_SCALE: f32 = 1.5;

/// Default `AUTOTUNE_LEVEL` (`ASCALAR(..., 6)`).
pub const AUTOTUNE_LEVEL_DEFAULT: u8 = 6;

/// Lowest accepted `AUTOTUNE_LEVEL` after `constrain_int32`.
pub const AUTOTUNE_LEVEL_MIN: u8 = 0;

/// Highest accepted `AUTOTUNE_LEVEL`. Upstream constrains to
/// `ARRAY_SIZE(tuning_table)` (11), even though the param docs say 0..10.
pub const AUTOTUNE_LEVEL_MAX: u8 = 11;

/// One `tuning_table` row: starting `tau` (s) and `rmax` (deg/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TuningRow {
    /// Starting time constant, seconds. Upstream `tuning_table[].tau`.
    pub tau: f32,
    /// Starting max rate, deg/s. Upstream `tuning_table[].rmax`.
    pub rmax: f32,
}

/// Upstream `tuning_table[]`. Index 0 is level 1; index 10 is level 11
/// (`yes, it goes to 11`).
pub const TUNING_TABLE: [TuningRow; 11] = [
    TuningRow {
        tau: 1.00,
        rmax: 20.0,
    },
    TuningRow {
        tau: 0.90,
        rmax: 30.0,
    },
    TuningRow {
        tau: 0.80,
        rmax: 40.0,
    },
    TuningRow {
        tau: 0.70,
        rmax: 50.0,
    },
    TuningRow {
        tau: 0.60,
        rmax: 60.0,
    },
    TuningRow {
        tau: 0.50,
        rmax: 75.0,
    },
    TuningRow {
        tau: 0.30,
        rmax: 90.0,
    },
    TuningRow {
        tau: 0.2,
        rmax: 120.0,
    },
    TuningRow {
        tau: 0.15,
        rmax: 160.0,
    },
    TuningRow {
        tau: 0.1,
        rmax: 210.0,
    },
    TuningRow {
        tau: 0.1,
        rmax: 300.0,
    },
];

/// Target `tau`/`rmax` after the level lookup (not the step toward it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LevelTarget {
    /// Level 0: keep the current `RMAX` and `TCONST`.
    KeepExisting,
    /// Table row, with pitch `tau` already scaled.
    Table(TuningRow),
}

/// Clamp `AUTOTUNE_LEVEL` the way `update_rmax` does.
///
/// Upstream `constrain_int32(aparm.autotune_level, 0, ARRAY_SIZE(tuning_table))`.
#[must_use]
pub const fn constrain_autotune_level(level: i32) -> u8 {
    if level < AUTOTUNE_LEVEL_MIN as i32 {
        AUTOTUNE_LEVEL_MIN
    } else if level > AUTOTUNE_LEVEL_MAX as i32 {
        AUTOTUNE_LEVEL_MAX
    } else {
        level as u8
    }
}

const fn row(tau: f32, rmax: f32) -> TuningRow {
    TuningRow { tau, rmax }
}

/// Raw table row for `level` in 1..=11. Level 0 and out-of-range are [`None`].
#[must_use]
pub const fn tuning_row(level: u8) -> Option<TuningRow> {
    match level {
        1 => Some(row(1.00, 20.0)),
        2 => Some(row(0.90, 30.0)),
        3 => Some(row(0.80, 40.0)),
        4 => Some(row(0.70, 50.0)),
        5 => Some(row(0.60, 60.0)),
        6 => Some(row(0.50, 75.0)),
        7 => Some(row(0.30, 90.0)),
        8 => Some(row(0.2, 120.0)),
        9 => Some(row(0.15, 160.0)),
        10 => Some(row(0.1, 210.0)),
        11 => Some(row(0.1, 300.0)),
        _ => None,
    }
}

/// Starting aggressiveness for `AUTOTUNE_LEVEL` on `axis`.
///
/// Pitch multiplies table `tau` by [`PITCH_TAU_SCALE`]. Roll and yaw use
/// the row as stored. The FF/I inverse-tau raise and the per-loop slew
/// toward this target are not applied here.
#[must_use]
pub const fn aggressiveness_target(level: i32, axis: AtType) -> LevelTarget {
    let level = constrain_autotune_level(level);
    match tuning_row(level) {
        None => LevelTarget::KeepExisting,
        Some(table_row) => {
            let tau = match axis {
                AtType::Pitch => table_row.tau * PITCH_TAU_SCALE,
                AtType::Roll | AtType::Yaw => table_row.tau,
            };
            LevelTarget::Table(TuningRow {
                tau,
                rmax: table_row.rmax,
            })
        }
    }
}
