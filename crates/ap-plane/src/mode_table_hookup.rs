//! Mode table lookup and stabilize dispatch for the scheduler tick.
//!
//! Upstream `Plane::update_control_mode` resolves `control_mode` through
//! `mode_from_mode_num` and dispatches into the active mode's `run()`.

use crate::main_loop::StabilizeDispatch;
use crate::mode_run::{applies_fbw_stick_mixing, StickMixing};
use crate::mode_table::{BuildFeatures, ModeNumber};

/// Resolve `control_mode` through the build's mode table and return which
/// stabilization paths the active mode selected.
///
/// Upstream `Mode::run`'s prologue before mode-specific logic.
#[must_use]
pub fn dispatch_stabilize_from_mode(
    control_mode: u8,
    stick_mixing: Option<StickMixing>,
    features: &BuildFeatures,
) -> StabilizeDispatch {
    let Some(mode) = ModeNumber::from_number(control_mode, features) else {
        return StabilizeDispatch::default();
    };

    let fbw_stick_mixing = applies_fbw_stick_mixing(stick_mixing);

    match mode {
        ModeNumber::Manual | ModeNumber::Training => StabilizeDispatch {
            roll: false,
            pitch: false,
            yaw: false,
            fbw_stick_mixing: false,
        },
        ModeNumber::Acro | ModeNumber::QAcro => StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing: false,
        },
        ModeNumber::Initialising | ModeNumber::Circle => StabilizeDispatch::default(),
        _ => StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing,
        },
    }
}

/// Pilot-in-the-loop / assisted modes owned by FW-022.
///
/// Autonomous navigation and quadplane modes are excluded so a new
/// `ModeNumber` variant must be classified here instead of silently no-op'ing.
#[must_use]
pub fn is_assisted_or_manual_mode(mode: ModeNumber) -> bool {
    match mode {
        ModeNumber::Manual
        | ModeNumber::Circle
        | ModeNumber::Stabilize
        | ModeNumber::Training
        | ModeNumber::Acro
        | ModeNumber::FlyByWireA
        | ModeNumber::FlyByWireB
        | ModeNumber::Cruise
        | ModeNumber::Autotune
        | ModeNumber::Thermal => true,
        ModeNumber::Auto
        | ModeNumber::Rtl
        | ModeNumber::Loiter
        | ModeNumber::Takeoff
        | ModeNumber::AvoidAdsb
        | ModeNumber::Guided
        | ModeNumber::Initialising
        | ModeNumber::QStabilize
        | ModeNumber::QHover
        | ModeNumber::QLoiter
        | ModeNumber::QLand
        | ModeNumber::QRtl
        | ModeNumber::QAutotune
        | ModeNumber::QAcro
        | ModeNumber::LoiterAltQLand
        | ModeNumber::Autoland => false,
    }
}
