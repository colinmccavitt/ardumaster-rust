//! Port of `APM_Control/AP_AutoTune` — fixed-wing gain tuning. FW-040.
//!
//! Upstream sits next to `AP_FW_Controller` and is called from the rate
//! loop when AUTOTUNE mode is active. Mode glue (`autotune_mode_hookup`,
//! FBWA stick mapping) already lives in `ap-plane`; this crate is the
//! tuner itself, not that nav-demand path.
//!
//! Demand transitions (`ATState::{IDLE, DEMAND_POS, DEMAND_NEG}` plus
//! `start` / `stop`) live in [`state`]. The `AUTOTUNE_LEVEL` aggressiveness
//! table lives in [`level`]. This slice adds the roll/pitch `ATGains`
//! snapshot (`save_gains` / `restore_gains`). Saturation / overshoot PID
//! rewrite in `update` comes later. `ATGains` rate/tau fields already live
//! on `ap-control::RateGains` (FW-017); they are not rewritten here.

#![no_std]

pub mod gains;
pub mod level;
pub mod state;

pub use gains::{
    apply_stop_gains, should_save_on_stop, snapshot_gains, AtGains,
};
pub use level::{
    aggressiveness_target, constrain_autotune_level, tuning_row, AUTOTUNE_LEVEL_DEFAULT,
    AUTOTUNE_LEVEL_MAX, AUTOTUNE_LEVEL_MIN, LevelTarget, PITCH_TAU_SCALE, TUNING_TABLE, TuningRow,
};
pub use state::{
    in_att_demand, next_demand_state, rate_threshold1, rate_threshold2, AtState, AtType, AutoTune,
    ATT_DEMAND_FRAC, RATE_THRESHOLD1_FRAC, RATE_THRESHOLD2_FRAC,
};
