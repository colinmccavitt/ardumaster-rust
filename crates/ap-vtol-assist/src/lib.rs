//! VTOL assistance in a forward flight mode. Upstream `ArduPlane/VTOL_Assist`.
//! Tracked as VT-002.
//!
//! Enable / check lives in [`assist`]. The speed / altitude trigger
//! (`aspeed < Q_ASSIST_SPEED` or height AGL `< Q_ASSIST_ALT`, after the
//! enable / check gate is open) lives in [`speed_alt`]. Force-assist
//! and the `Q_OPTIONS` bits that latch it (`Q_ASSIST_FORCE_ENABLE`,
//! spin-while-armed) live in [`force`]. The angle-error trigger
//! (outside the flight envelope *and* attitude error `>= Q_ASSIST_ANGLE`)
//! lives in [`angle`]. A small [`VtolAssist`] object, not QuadPlane.
//! [`completeness`] is the closer catalog: enable/check, speed/alt,
//! force, and angle-error are on main; leftover state-update tick,
//! assist-active latch, recovery, logging/GCS bits, and leftover
//! option paths stay documented as remaining.
//!
//! Hysteresis, spin recovery, and the rest of `should_assist` are not
//! here.

#![no_std]

pub mod angle;
pub mod assist;
pub mod completeness;
pub mod force;
pub mod speed_alt;

pub use angle::{
    angle_check_enabled, evaluate_angle, AngleDecision, AngleSample, ALLOWED_ENVELOPE_ERROR_DEG,
};
pub use assist::{
    q_assist_force_enable_set, AssistOption, AssistState, AuxSwitchPos, VtolAssist,
    ASSIST_ALT_DEFAULT, ASSIST_ANGLE_DEFAULT, ASSIST_DELAY_DEFAULT, ASSIST_OPTIONS_DEFAULT,
    ASSIST_SPEED_DEFAULT, DISABLE_SYNTHETIC_AIRSPEED_ASSIST, Q_ASSIST_FORCE_ENABLE,
};
pub use completeness::{AssistPortItem, PortStatus, ASSIST_COMPLETENESS};
pub use force::{
    disable_synthetic_airspeed_assist_set, evaluate_force, force_assist_latched,
    requested_overriding_speed_alt, synthetic_airspeed_assist_allowed, ForceDecision, ForceSample,
};
pub use speed_alt::{evaluate_speed_alt, SpeedAltDecision, SpeedAltSample};
