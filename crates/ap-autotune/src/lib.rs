//! Port of `APM_Control/AP_AutoTune` — fixed-wing gain tuning. FW-040.
//!
//! Upstream sits next to `AP_FW_Controller` and is called from the rate
//! loop when AUTOTUNE mode is active. Mode glue (`autotune_mode_hookup`,
//! FBWA stick mapping) already lives in `ap-plane`; this crate is the
//! tuner itself, not that nav-demand path.
//!
//! This slice is the demand state machine: `ATState::{IDLE, DEMAND_POS,
//! DEMAND_NEG}` plus `start` / `stop`. Gain save/restore, the
//! `AUTOTUNE_LEVEL` table, and the PID rewrite in `update` come later.
//! `ATGains` rate/tau fields already live on `ap-control::RateGains`
//! (FW-017); they are not rewritten here.

#![no_std]

pub mod state;

pub use state::{
    in_att_demand, next_demand_state, rate_threshold1, rate_threshold2, AtState, AtType, AutoTune,
    ATT_DEMAND_FRAC, RATE_THRESHOLD1_FRAC, RATE_THRESHOLD2_FRAC,
};
