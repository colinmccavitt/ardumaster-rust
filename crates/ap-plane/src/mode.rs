//! Plane's mode machine, upstream `ArduPlane/system.cpp:252`, `Plane::set_mode`.
//!
//! # It is not Copter's ladder with different names
//!
//! Copter checks everything and then commits. Plane commits first and rolls
//! back if the mode refuses to start, because a mode's `enter()` reads
//! `control_mode` — upstream's own TODO says so — and would otherwise be asked
//! to start while the vehicle still claims to be in the mode it is leaving.
//!
//! That is why this module exposes the change as two steps rather than one
//! predicate. A caller applies the state, runs `enter()`, and rolls back if it
//! fails. Collapsing it into "decide, then apply" would be tidier and would
//! change what `enter()` sees, which is the one thing the shape exists to get
//! right.
//!
//! The veto order differs too. Plane tests the fence *before* the GCS block;
//! Copter tests it after. Both are reproduced as written.
//!
//! # The rollback restores four things
//!
//! `control_mode`, `previous_mode`, and a reason for each. A rollback that
//! restored three of them would leave the vehicle in the right mode with a
//! wrong story about how it got there — and `in_fence_recovery` reads exactly
//! those reasons, so the next mode change would be judged on it.

/// Why a mode change was asked for.
///
/// Only the reasons the mode machine actually distinguishes are named; the
/// rest are carried through as [`ModeReason::Other`] with their number, so a
/// caller can store and compare them without this enum having to track every
/// value upstream defines.
///
/// The numbers are upstream's, from `AP_Vehicle/ModeReason.h`. They are logged
/// and sent over MAVLink, so they are part of what this port has to reproduce
/// rather than an internal choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeReason {
    /// Startup. Suppresses both the happy and the sad noise.
    Initialised,
    /// The ground station asked.
    GcsCommand,
    /// The fence put the vehicle here.
    FenceBreached,
    /// RTL finished and handed over to a fixed-wing autoland.
    RtlCompleteSwitchingToFixedwingAutoland,
    /// RTL finished and handed over to a VTOL land.
    RtlCompleteSwitchingToVtolLandRtl,
    /// A QRTL was substituted for an RTL.
    QrtlInsteadOfRtl,
    /// A QLAND was substituted for an RTL.
    QlandInsteadOfRtl,
    /// Any other reason, carrying upstream's number so it still compares
    /// correctly against itself.
    Other(u8),
}

impl ModeReason {
    /// Upstream's number for this reason.
    #[must_use]
    pub fn as_number(self) -> u8 {
        match self {
            Self::GcsCommand => 2,
            Self::FenceBreached => 10,
            Self::Initialised => 26,
            Self::RtlCompleteSwitchingToVtolLandRtl => 39,
            Self::RtlCompleteSwitchingToFixedwingAutoland => 40,
            Self::QrtlInsteadOfRtl => 44,
            Self::QlandInsteadOfRtl => 49,
            Self::Other(n) => n,
        }
    }

    /// The reason upstream's number denotes.
    ///
    /// Numbers this module does not name become [`Self::Other`], which
    /// compares correctly against itself and is never mistaken for one of the
    /// named reasons — the machine only ever asks whether a reason *is* one of
    /// them.
    #[must_use]
    pub fn from_number(number: u8) -> Self {
        match number {
            2 => Self::GcsCommand,
            10 => Self::FenceBreached,
            26 => Self::Initialised,
            39 => Self::RtlCompleteSwitchingToVtolLandRtl,
            40 => Self::RtlCompleteSwitchingToFixedwingAutoland,
            44 => Self::QrtlInsteadOfRtl,
            49 => Self::QlandInsteadOfRtl,
            other => Self::Other(other),
        }
    }
}

impl ModeReason {
    /// Whether this reason is an automatic change driven by landing
    /// sequencing, upstream `mode_reason_is_landing_sequence`.
    ///
    /// These four are the fence's own recovery completing. Treating them as
    /// ordinary mode changes would make the fence block the very handover it
    /// asked for.
    #[must_use]
    pub fn is_landing_sequence(self) -> bool {
        matches!(
            self,
            Self::RtlCompleteSwitchingToFixedwingAutoland
                | Self::RtlCompleteSwitchingToVtolLandRtl
                | Self::QrtlInsteadOfRtl
                | Self::QlandInsteadOfRtl
        )
    }
}

/// The four pieces of state a mode change moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeState {
    /// The mode running now.
    pub control_mode: u8,
    /// The mode before it.
    pub previous_mode: u8,
    /// Why the current mode was entered.
    pub control_mode_reason: ModeReason,
    /// Why the previous mode was entered.
    pub previous_mode_reason: ModeReason,
}

/// What a rollback needs to undo an [`apply`](ModeState::apply).
///
/// Deliberately opaque and deliberately not `Copy`-into-anything-else: it
/// exists so a caller cannot roll back by reconstructing three of the four
/// fields from memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeSnapshot {
    control_mode: u8,
    previous_mode: u8,
    control_mode_reason: ModeReason,
    previous_mode_reason: ModeReason,
}

impl ModeState {
    /// Move to `new_mode`, returning what a rollback would need.
    ///
    /// Upstream does this before calling `enter()`, so the new mode starts up
    /// while the vehicle already claims to be in it.
    pub fn apply(&mut self, new_mode: u8, reason: ModeReason) -> ModeSnapshot {
        let snapshot = ModeSnapshot {
            control_mode: self.control_mode,
            previous_mode: self.previous_mode,
            control_mode_reason: self.control_mode_reason,
            previous_mode_reason: self.previous_mode_reason,
        };

        self.previous_mode = self.control_mode;
        self.control_mode = new_mode;
        self.previous_mode_reason = self.control_mode_reason;
        self.control_mode_reason = reason;

        snapshot
    }

    /// Undo an [`apply`](Self::apply) after the new mode refused to start.
    ///
    /// # Not quite a restore
    ///
    /// Upstream does not put back what it saved. It writes
    /// `control_mode_reason = previous_mode_reason` — the value `apply` just
    /// overwrote it with, which is the same thing — and then
    /// `previous_mode_reason = old_previous_mode_reason`. The result is
    /// identical to a plain restore, and it is written that way here because
    /// the equivalence is worth being able to see rather than having to
    /// re-derive.
    pub fn roll_back(&mut self, snapshot: ModeSnapshot) {
        self.control_mode = snapshot.control_mode;
        self.previous_mode = snapshot.previous_mode;
        self.control_mode_reason = snapshot.control_mode_reason;
        self.previous_mode_reason = snapshot.previous_mode_reason;
    }

    /// Whether a fence breach recovery is still under way, upstream
    /// `Plane::in_fence_recovery`.
    ///
    /// The subtle half is the second clause. A breach sends the vehicle to
    /// RTL with reason `FENCE_BREACHED`; RTL then completes and hands over to
    /// a landing mode with its own reason, at which point the *current*
    /// reason is no longer the breach. The recovery is still in progress, and
    /// reading only the current reason would declare it finished exactly when
    /// the vehicle is closest to the ground.
    ///
    /// `auto_outside_landing_sequence` is upstream's first early return: in
    /// AUTO with no landing sequence flagged, the operator has retargeted the
    /// mission away from the landing, and holding them in recovery would be
    /// holding them to a plan they have abandoned.
    #[must_use]
    pub fn in_fence_recovery(&self, auto_outside_landing_sequence: bool) -> bool {
        if auto_outside_landing_sequence {
            return false;
        }

        let current_mode_breach = self.control_mode_reason == ModeReason::FenceBreached;
        let previous_mode_breach = self.previous_mode_reason == ModeReason::FenceBreached;
        let previous_mode_complete = self.control_mode_reason.is_landing_sequence();

        current_mode_breach || (previous_mode_breach && previous_mode_complete)
    }
}

/// Why a mode change was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeChangeVeto {
    /// A VTOL mode was asked for on a vehicle with no quadplane available.
    VtolUnavailable,
    /// A fence breach recovery is in progress.
    InFenceRecovery,
    /// `FLTMODE_GCSBLOCK` blocks this mode for ground-station entry.
    GcsEntryDisabled,
}

/// The fence's view, as the mode machine reads it.
#[derive(Debug, Clone, Copy)]
pub struct FenceState {
    /// `hal.util->get_soft_armed()`. Plane reads the arming state here rather
    /// than the motors, because a fixed-wing has no spool state to consult.
    pub soft_armed: bool,
    /// `fence.enabled()`.
    pub enabled: bool,
    /// `fence.option_enabled(DISABLE_MODE_CHANGE)`.
    pub disable_mode_change: bool,
    /// `fence.get_breaches()` is non-zero.
    pub breached: bool,
    /// `in_fence_recovery()`.
    pub recovering: bool,
}

/// Everything the pre-entry checks read.
#[derive(Debug, Clone, Copy)]
pub struct ModeChangeRequest {
    /// The requested mode's number.
    pub new_mode: u8,
    /// Why.
    pub reason: ModeReason,
    /// The requested mode is a VTOL mode.
    pub new_is_vtol: bool,
    /// A quadplane is compiled in, enabled and initialised.
    pub quadplane_available: bool,
    /// The fence's view.
    pub fence: FenceState,
    /// `gcs_mode_enabled(new_mode)`.
    pub gcs_entry_enabled: bool,
}

/// The checks that run before the mode is applied.
///
/// # The fence is tested before the GCS block
///
/// Copter tests them the other way round. The consequence here is that a
/// ground station blocked from a mode during a fence recovery is told it is
/// in fence recovery rather than that the mode is blocked — the more useful
/// of the two messages, since the block is a standing configuration and the
/// recovery is the thing that will pass.
#[must_use]
pub fn mode_change_veto(request: &ModeChangeRequest) -> Option<ModeChangeVeto> {
    if request.new_is_vtol && !request.quadplane_available {
        return Some(ModeChangeVeto::VtolUnavailable);
    }

    // Note the landing-sequence exemption: the fence's own recovery completing
    // must not be blocked by the fence.
    if request.fence.soft_armed
        && request.fence.enabled
        && request.fence.disable_mode_change
        && request.fence.breached
        && request.fence.recovering
        && !request.reason.is_landing_sequence()
    {
        return Some(ModeChangeVeto::InFenceRecovery);
    }

    if request.reason == ModeReason::GcsCommand && !request.gcs_entry_enabled {
        return Some(ModeChangeVeto::GcsEntryDisabled);
    }

    None
}

/// Whether asking for the mode already running should make the happy noise.
///
/// Upstream returns success either way. The noise is suppressed when the
/// reason matches the one already recorded, which stops a ground station
/// repeating a mode request from beeping every time, and suppressed at
/// startup because nobody asked for anything yet.
#[must_use]
pub fn already_in_mode_notifies(reason: ModeReason, control_mode_reason: ModeReason) -> bool {
    reason != control_mode_reason && reason != ModeReason::Initialised
}
