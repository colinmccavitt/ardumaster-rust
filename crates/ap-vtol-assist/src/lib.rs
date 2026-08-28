//! VTOL assistance in a forward flight mode. Upstream `ArduPlane/VTOL_Assist`.
//! Tracked as VT-002.
//!
//! Enable / check lives in [`assist`]. The speed / altitude trigger
//! (`aspeed < Q_ASSIST_SPEED` or height AGL `< Q_ASSIST_ALT`, after the
//! enable / check gate is open) lives in [`speed_alt`]. A small
//! [`VtolAssist`] object, not QuadPlane.
//!
//! Angle-error hysteresis, spin recovery, and the rest of
//! `should_assist` are not here.

#![no_std]

pub mod assist;
pub mod speed_alt;

pub use assist::{
    q_assist_force_enable_set, AssistOption, AssistState, AuxSwitchPos, VtolAssist,
    ASSIST_ALT_DEFAULT, ASSIST_ANGLE_DEFAULT, ASSIST_DELAY_DEFAULT, ASSIST_OPTIONS_DEFAULT,
    ASSIST_SPEED_DEFAULT, DISABLE_SYNTHETIC_AIRSPEED_ASSIST, Q_ASSIST_FORCE_ENABLE,
};
pub use speed_alt::{evaluate_speed_alt, SpeedAltDecision, SpeedAltSample};
