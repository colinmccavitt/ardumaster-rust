//! VTOL assistance in a forward flight mode. Upstream `ArduPlane/VTOL_Assist`.
//! Tracked as VT-002.
//!
//! This slice is the enable / check gate: whether assist *may* run given
//! `Q_ASSIST_SPEED`, `Q_ASSIST_ALT`, the Q-assist option bits, and the
//! three-position aux state. A small [`VtolAssist`] object, not QuadPlane.
//!
//! Speed / altitude trigger evaluation, angle-error hysteresis, spin
//! recovery, and the rest of `should_assist` are not here.

#![no_std]

pub mod assist;

pub use assist::{
    q_assist_force_enable_set, AssistOption, AssistState, AuxSwitchPos, VtolAssist,
    ASSIST_ALT_DEFAULT, ASSIST_ANGLE_DEFAULT, ASSIST_DELAY_DEFAULT, ASSIST_OPTIONS_DEFAULT,
    ASSIST_SPEED_DEFAULT, DISABLE_SYNTHETIC_AIRSPEED_ASSIST, Q_ASSIST_FORCE_ENABLE,
};
