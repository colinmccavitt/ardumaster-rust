//! Port of `APM_Control/AP_AutoTune` — fixed-wing gain tuning. FW-040.
//!
//! Upstream sits next to `AP_FW_Controller` and is called from the rate
//! loop when AUTOTUNE mode is active. Mode glue (`autotune_mode_hookup`,
//! FBWA stick mapping) already lives in `ap-plane`; this crate is the
//! tuner itself, not that nav-demand path.
//!
//! Demand transitions (`ATState::{IDLE, DEMAND_POS, DEMAND_NEG}` plus
//! `start` / `stop`) live in [`state`]. This slice adds the
//! `AUTOTUNE_LEVEL` aggressiveness table (`tuning_table` in
//! `AP_AutoTune.cpp`). Gain save/restore and the PID rewrite in `update`
//! come later. `ATGains` rate/tau fields already live on
//! `ap-control::RateGains` (FW-017); they are not rewritten here.

#![no_std]

pub mod level;
pub mod state;

pub use level::{
    aggressiveness_target, constrain_autotune_level, tuning_row, AUTOTUNE_LEVEL_DEFAULT,
    AUTOTUNE_LEVEL_MAX, AUTOTUNE_LEVEL_MIN, LevelTarget, PITCH_TAU_SCALE, TUNING_TABLE, TuningRow,
};
pub use state::{
    in_att_demand, next_demand_state, rate_threshold1, rate_threshold2, AtState, AtType, AutoTune,
    ATT_DEMAND_FRAC, RATE_THRESHOLD1_FRAC, RATE_THRESHOLD2_FRAC,
};
