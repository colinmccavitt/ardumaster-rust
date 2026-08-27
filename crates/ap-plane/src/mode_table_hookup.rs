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
