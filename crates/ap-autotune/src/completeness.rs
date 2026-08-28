//! FW-040 AutoTune completeness closer: `AP_AutoTune.cpp` surfaces
//! already on main vs remaining gaps.
//!
//! Catalogs the Plane AutoTune port. Items marked [`PortStatus::OnMain`]
//! landed in earlier slices and must not be redone. [`PortStatus::ThisSlice`]
//! includes Action / D-limit hunting (`RAISE_D` / `LOWER_D` /
//! `LOWER_PD` / `IDLE_LOWER_PD`) plus the earlier closer rows.
//! [`PortStatus::Remaining`] are still-open `AP_AutoTune.cpp` gaps
//! (slew limiter, ATRP log, EEPROM `save_*_if_changed`).
//!
//! This module does not rewrite [`crate::filters`] or [`crate::start`].

use crate::ff::{constrain_imax, AUTOTUNE_MAX_IMAX, AUTOTUNE_MIN_IMAX};
use crate::gains::should_save_on_stop;
use crate::state::{AtType, AutoTune};

/// Cycles without reducing P after both limits are set, upstream `done_count == 3`.
pub const DONE_COUNT_SAVE: u8 = 3;

/// Arbitrary yaw attitude limit, upstream `att_limit_deg = 20` for `AUTOTUNE_YAW`.
pub const YAW_ATT_LIMIT_DEG: f32 = 20.0;

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-040 completeness-closer slice.
    ThisSlice,
    /// Still deferred (`AP_AutoTune.cpp` leftover).
    Remaining,
}

/// One AutoTune surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutotunePortItem {
    /// Hookup or gap name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: hooked-up AutoTune vs remaining `AP_AutoTune.cpp` gaps.
pub const AUTOTUNE_COMPLETENESS: &[AutotunePortItem] = &[
    AutotunePortItem {
        name: "autotune_mode_hookup",
        status: PortStatus::OnMain,
        note: "ap-plane FBWA stick mapping — do not rewrite",
    },
    AutotunePortItem {
        name: "ATState Idle/DemandPos/DemandNeg",
        status: PortStatus::OnMain,
        note: "state.rs demand machine",
    },
    AutotunePortItem {
        name: "AUTOTUNE_LEVEL aggressiveness table",
        status: PortStatus::OnMain,
        note: "level.rs tuning_table tau/rmax",
    },
    AutotunePortItem {
        name: "save_gains / restore_gains",
        status: PortStatus::OnMain,
        note: "gains.rs stop snapshot when P/D limits positive",
    },
    AutotunePortItem {
        name: "update_gains saturation/overshoot",
        status: PortStatus::OnMain,
        note: "update.rs RAISE_P 1.3 / LOWER_P 0.35",
    },
    AutotunePortItem {
        name: "AUTOTUNE_OPTIONS FLTD/FLTT",
        status: PortStatus::OnMain,
        note: "options.rs has_option DISABLE_FLTD/FLTT",
    },
    AutotunePortItem {
        name: "AUTOTUNE_AXES start mask",
        status: PortStatus::OnMain,
        note: "options.rs roll/pitch/yaw bits including AUTOTUNE_YAW",
    },
    AutotunePortItem {
        name: "I-term / FF coupling",
        status: PortStatus::OnMain,
        note: "ff.rs INCREASE/DECREASE_FF_STEP, roll min(FF,P), IMAX helper",
    },
    AutotunePortItem {
        name: "FF estimate / ff_filter",
        status: PortStatus::OnMain,
        note: "ff_estimate.rs FF_single / ff_count 1/4 gates",
    },
    AutotunePortItem {
        name: "start() zero-FF floor 0.01",
        status: PortStatus::OnMain,
        note: "start.rs floor_start_ff — do not rewrite",
    },
    AutotunePortItem {
        name: "actuator/rate/target LPF cutoffs",
        status: PortStatus::OnMain,
        note: "filters.rs 0.75/0.75/4 Hz — do not rewrite",
    },
    AutotunePortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog of OnMain vs Remaining AP_AutoTune.cpp gaps",
    },
    AutotunePortItem {
        name: "IMAX constrain on start",
        status: PortStatus::ThisSlice,
        note: "constrain_float(kIMAX, AUTOTUNE_MIN_IMAX 0.4, AUTOTUNE_MAX_IMAX 0.9)",
    },
    AutotunePortItem {
        name: "yaw att_limit 20 deg",
        status: PortStatus::ThisSlice,
        note: "AUTOTUNE_YAW att_limit_deg = 20 (no yaw angle-limit param)",
    },
    AutotunePortItem {
        name: "save_gains after N stable cycles",
        status: PortStatus::ThisSlice,
        note: "done_count == 3 mid-tune save_gains after P_limit and D_limit",
    },
    AutotunePortItem {
        name: "Action / D-limit hunting",
        status: PortStatus::ThisSlice,
        note: "action.rs RAISE_D / LOWER_D / LOWER_PD / IDLE_LOWER_PD",
    },
    AutotunePortItem {
        name: "slew_limit / SlewLimiter",
        status: PortStatus::Remaining,
        note: "default slew_limit 150 deg/s, P/D slew rate tracking",
    },
    AutotunePortItem {
        name: "log_ATRP 25Hz",
        status: PortStatus::Remaining,
        note: "logger WriteBlock every 40 ms",
    },
    AutotunePortItem {
        name: "EEPROM save_*_if_changed",
        status: PortStatus::Remaining,
        note: "save_float_if_changed / save_int16_if_changed parameter persist",
    },
    AutotunePortItem {
        name: "update_rmax FF/I inverse-tau",
        status: PortStatus::Remaining,
        note: "target_tau = MAX(target_tau, 1/invtau)",
    },
    AutotunePortItem {
        name: "LOW_RATE / SHORT event rejects",
        status: PortStatus::Remaining,
        note: "max_rate < 0.01*rmax or event < 100 ms",
    },
    AutotunePortItem {
        name: "clipped actuator without I",
        status: PortStatus::Remaining,
        note: "constrain_float(FF+P+D+DFF+I, -45, 45) - I",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static AutotunePortItem> {
    AUTOTUNE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static AutotunePortItem> {
    AUTOTUNE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for later `AP_AutoTune.cpp` work.
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static AutotunePortItem> {
    AUTOTUNE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in AUTOTUNE_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    AUTOTUNE_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in AUTOTUNE_COMPLETENESS.iter().enumerate() {
        for other in AUTOTUNE_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

/// Attitude limit used for rate-threshold / in-demand checks.
///
/// Roll and pitch take the live `aparm` limits. Yaw is the hardcoded
/// upstream 20 deg (no yaw angle-limit param).
#[must_use]
pub const fn att_limit_deg(axis: AtType, roll_limit_deg: f32, pitch_limit_deg: f32) -> f32 {
    match axis {
        AtType::Roll => roll_limit_deg,
        AtType::Pitch => pitch_limit_deg,
        AtType::Yaw => YAW_ATT_LIMIT_DEG,
    }
}

/// Whether `done_count` has reached the mid-tune save.
#[must_use]
pub const fn should_save_after_cycles(done_count: u8) -> bool {
    done_count >= DONE_COUNT_SAVE
}

/// Apply the start() IMAX clamp, upstream `constrain_float(kIMAX, 0.4, 0.9)`.
#[must_use]
pub fn start_constrain_imax(imax: f32) -> f32 {
    constrain_imax(imax)
}

/// IMAX band written on start, matching [`AUTOTUNE_MIN_IMAX`]..=[`AUTOTUNE_MAX_IMAX`].
#[must_use]
pub const fn start_imax_band() -> (f32, f32) {
    (AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX)
}

impl AutoTune {
    /// Record one non-reducing cycle after both P and D limits are set.
    ///
    /// Upstream increments `done_count` in the "not oscillating, limits
    /// already found" branch and calls `save_gains` when it reaches 3.
    /// Returns true when this cycle persisted `last_save`.
    pub fn record_stable_cycle(&mut self) -> bool {
        if !self.running || !should_save_on_stop(self.p_limit, self.d_limit) {
            return false;
        }
        if self.done_count >= DONE_COUNT_SAVE {
            return false;
        }
        self.done_count += 1;
        if self.done_count == DONE_COUNT_SAVE {
            self.save_gains();
            true
        } else {
            false
        }
    }
}
