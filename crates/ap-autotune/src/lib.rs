//! Port of `APM_Control/AP_AutoTune` — fixed-wing gain tuning. FW-040.
//!
//! Upstream sits next to `AP_FW_Controller` and is called from the rate
//! loop when AUTOTUNE mode is active. Mode glue (`autotune_mode_hookup`,
//! FBWA stick mapping) already lives in `ap-plane`; this crate is the
//! tuner itself, not that nav-demand path.
//!
//! Demand transitions (`ATState::{IDLE, DEMAND_POS, DEMAND_NEG}` plus
//! `start` / `stop`) live in [`state`]. The `AUTOTUNE_LEVEL` aggressiveness
//! table lives in [`level`]. The roll/pitch `ATGains` snapshot
//! (`save_gains` / `restore_gains`) lives in [`gains`]. This slice adds
//! the saturation / overshoot P rewrite and `update_rmax` tau/rmax slew
//! in [`update`]. D-limit hunting and FF median filter come later.
//! `ATGains` rate/tau fields already live on `ap-control::RateGains`
//! (FW-017); they are not rewritten here.

#![no_std]

pub mod gains;
pub mod level;
pub mod state;
pub mod update;

pub use gains::{apply_stop_gains, should_save_on_stop, snapshot_gains, AtGains};
pub use level::{
    aggressiveness_target, constrain_autotune_level, tuning_row, LevelTarget, TuningRow,
    AUTOTUNE_LEVEL_DEFAULT, AUTOTUNE_LEVEL_MAX, AUTOTUNE_LEVEL_MIN, PITCH_TAU_SCALE, TUNING_TABLE,
};
pub use state::{
    in_att_demand, next_demand_state, rate_threshold1, rate_threshold2, AtState, AtType, AutoTune,
    ATT_DEMAND_FRAC, RATE_THRESHOLD1_FRAC, RATE_THRESHOLD2_FRAC,
};
pub use update::{
    apply_p_step, couple_tau_rmax, gain_action, slew_rmax, slew_tau, update_gains, GainAction,
    LOWER_P_MUL, RAISE_P_MUL, RMAX_DEFAULT, RMAX_STEP, TAU_SLEW_DOWN, TAU_SLEW_UP,
};
