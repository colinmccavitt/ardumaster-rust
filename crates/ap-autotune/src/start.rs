//! Zero-FF floor on `AP_AutoTune::start`.
//!
//! Upstream refuses a zero feed-forward so the tuner never starts at
//! `current.FF == 0`: `if (current.FF < 0.01) { current.FF = 0.01; }`.
//! The PID `rpid.ff().set` write is deferred with the live AC_PID bind.

/// Minimum FF written on start, upstream `0.01`.
pub const AUTOTUNE_MIN_FF: f32 = 0.01;

/// Raise FF to [`AUTOTUNE_MIN_FF`] when it is below the floor.
///
/// Upstream `if (current.FF < 0.01) { current.FF = 0.01; rpid.ff().set(current.FF); }`
/// in `AP_AutoTune::start`, after the restore / last-save snapshot.
#[must_use]
pub fn floor_start_ff(ff: f32) -> f32 {
    if ff < AUTOTUNE_MIN_FF {
        AUTOTUNE_MIN_FF
    } else {
        ff
    }
}
