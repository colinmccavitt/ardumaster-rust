//! Whether a flight-mode change is allowed, upstream `Copter::set_mode`.
//!
//! The function is a ladder of vetoes followed by a commit. Only the ladder is
//! here; the commit — swapping the mode pointer, logging, notifying, setting
//! the rate time constants — belongs to the caller, which is the only thing
//! that owns the vehicle's mode.
//!
//! # Why the order is the content
//!
//! Every veto returns the same `false` to the caller, so from the outside the
//! ladder looks like one big condition. It is not, because each rung also
//! sends the pilot a different message, and a pilot deciding what to do about
//! a refused mode change acts on that message. "Requires position" sends them
//! looking for GPS; "throttle too high" sends them to the stick. Reordering
//! two rungs changes nothing about which changes are allowed and everything
//! about which explanation arrives.
//!
//! That is why this is modelled as a [`ModeEntry`] carrying the specific veto
//! rather than a bool, and why the parity recording intercepts upstream's
//! failure message rather than only its return value. Comparing return values
//! alone would let every veto in this ladder be permuted freely.

/// Why a mode change was refused.
///
/// The names are upstream's messages, which are what reaches the pilot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeEntryVeto {
    /// The mode is blocked for GCS entry by `FLTMODE_GCSBLOCK`.
    ///
    /// Only ever applies to a change the ground station asked for. A pilot
    /// with a mode switch is not affected, which is the point of the
    /// parameter: it stops a ground station from putting the aircraft
    /// somewhere the operator did not choose.
    GcsEntryDisabled,
    /// No mode carries this number.
    NoSuchMode,
    /// Upstream's "throttle too high".
    ThrottleTooHigh,
    /// Upstream's "requires position".
    RequiresPosition,
    /// Upstream's "need alt estimate".
    NeedAltEstimate,
    /// Upstream's "in fence recovery".
    InFenceRecovery,
    /// Upstream's "in RC failsafe".
    InRcFailsafe,
    /// The mode's own `init` refused.
    InitFailed,
}

/// What `set_mode` decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeEntry {
    /// Already in the requested mode. Upstream returns success without running
    /// a single check — the aircraft is already where it was asked to be, and
    /// a check that refused would be refusing the status quo.
    AlreadyInMode,
    /// `AUTO_RTL`, which is not a mode but AUTO carrying a mission jump. The
    /// answer comes from the mission logic, so the ladder does not decide it.
    DelegatedToAutoRtl,
    /// Every check passed. The caller commits the change.
    Entered,
    /// Refused, with the reason the pilot is told.
    Refused(ModeEntryVeto),
}

/// The fence conditions that can block a mode change.
///
/// All four must hold together, and only during a breach recovery the fence
/// itself started.
#[derive(Debug, Clone, Copy)]
pub struct FenceState {
    /// `fence.enabled()`.
    pub enabled: bool,
    /// `fence.option_enabled(DISABLE_MODE_CHANGE)`.
    pub disable_mode_change: bool,
    /// `fence.get_breaches()` is non-zero.
    pub breached: bool,
    /// The current mode was entered *because* of a fence breach.
    pub entered_for_breach: bool,
}

/// Everything the ladder reads.
///
/// The field count is upstream's, not an elaboration of it: `set_mode` really
/// does consult this many things about the vehicle before letting a mode
/// change through.
#[derive(Debug, Clone, Copy)]
pub struct ModeEntryRequest {
    /// The requested mode is the one already running.
    pub target_is_current: bool,
    /// The request is `AUTO_RTL`.
    pub target_is_auto_rtl: bool,
    /// The request came from the ground station.
    pub reason_is_gcs_command: bool,
    /// `gcs_mode_enabled(mode)` — false when `FLTMODE_GCSBLOCK` blocks it.
    pub gcs_entry_enabled: bool,
    /// A mode exists with this number.
    pub mode_exists: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// The requested mode flies on the pilot's throttle directly.
    pub new_has_manual_throttle: bool,
    /// The requested mode is DRIFT, which upstream treats as manual-throttle
    /// for this check even though it reports otherwise.
    pub new_is_drift: bool,
    /// The mode currently running flies on the pilot's throttle directly.
    pub current_has_manual_throttle: bool,
    /// What the requested mode would make of the throttle stick right now.
    pub pilot_throttle: f32,
    /// `copter.get_non_takeoff_throttle()`.
    pub non_takeoff_throttle: f32,
    /// The requested mode needs a position estimate.
    pub new_requires_position: bool,
    /// `copter.position_ok()`.
    pub position_ok: bool,
    /// `copter.ekf_alt_ok()`.
    pub ekf_alt_ok: bool,
    /// The fence's view.
    pub fence: FenceState,
    /// `rc().in_rc_failsafe()`.
    pub in_rc_failsafe: bool,
    /// The requested mode is willing to be entered during an RC failsafe.
    pub new_allows_entry_in_rc_failsafe: bool,
    /// What the requested mode's `init` returned.
    pub init_ok: bool,
}

/// Run the ladder, upstream `Copter::set_mode`.
///
/// # Disarmed skips almost everything
///
/// `ignore_checks` is simply `!armed`, and it suppresses every check from the
/// throttle test down to `init` — which still runs, but is told to ignore its
/// own checks too. A disarmed aircraft can be put into any mode, because
/// nothing it does in that mode can hurt anyone until it arms, and the arming
/// checks are where the real gate is. Loading a mission on the bench should
/// not require a GPS fix.
///
/// Two rungs are *not* suppressed by it: the GCS block, and the RC failsafe.
/// Neither is about whether flying in the new mode is safe.
#[must_use]
pub fn mode_entry(request: &ModeEntryRequest) -> ModeEntry {
    if request.target_is_current {
        return ModeEntry::AlreadyInMode;
    }

    // Not suppressed by being disarmed: this is about who is allowed to ask,
    // not about whether the aircraft could cope.
    if request.reason_is_gcs_command && !request.gcs_entry_enabled {
        return ModeEntry::Refused(ModeEntryVeto::GcsEntryDisabled);
    }

    if request.target_is_auto_rtl {
        return ModeEntry::DelegatedToAutoRtl;
    }

    if !request.mode_exists {
        return ModeEntry::Refused(ModeEntryVeto::NoSuchMode);
    }

    // Disarmed, allow switching to any mode: the arming checks are the gate.
    let ignore_checks = !request.armed;

    // Don't let the aircraft leap off the ground when a pilot switches into a
    // manual-throttle mode with the stick already raised. The case upstream
    // describes: armed in guided, throttle raised to 1300 — not enough to
    // trigger an auto takeoff — then switched to a manual mode, where 1300
    // suddenly means what it says.
    //
    // DRIFT is counted as manual-throttle here despite reporting otherwise,
    // because for this purpose what matters is whether the stick reaches the
    // motors, and in DRIFT it does.
    let user_throttle = request.new_has_manual_throttle || request.new_is_drift;
    if !ignore_checks
        && request.land_complete
        && user_throttle
        && !request.current_has_manual_throttle
        && request.pilot_throttle > request.non_takeoff_throttle
    {
        return ModeEntry::Refused(ModeEntryVeto::ThrottleTooHigh);
    }

    if !ignore_checks && request.new_requires_position && !request.position_ok {
        return ModeEntry::Refused(ModeEntryVeto::RequiresPosition);
    }

    // Only when the change would make things worse. Leaving a mode that never
    // needed an altitude estimate for one that does is the transition that can
    // hurt; staying put, or moving between two modes that both need it, is not
    // improved by refusing here.
    if !ignore_checks
        && !request.ekf_alt_ok
        && request.current_has_manual_throttle
        && !request.new_has_manual_throttle
    {
        return ModeEntry::Refused(ModeEntryVeto::NeedAltEstimate);
    }

    // Recovering from a fence breach. The vehicle put itself into the current
    // mode to get back inside, and a mode change would abandon that — but only
    // while it is still airborne, and only if the operator asked the fence to
    // hold on to control.
    //
    // Upstream also tests `motors->armed()` here, which `!ignore_checks`
    // already guarantees. Reproduced as written; the redundancy costs nothing
    // and removing it would make this rung read differently from the source.
    if !ignore_checks
        && request.fence.enabled
        && request.fence.disable_mode_change
        && request.fence.breached
        && request.armed
        && request.fence.entered_for_breach
        && !request.land_complete
    {
        return ModeEntry::Refused(ModeEntryVeto::InFenceRecovery);
    }

    // Not suppressed by being disarmed either, and deliberately so: a mode
    // that refuses entry during an RC failsafe is refusing because of what it
    // would do without a pilot, which does not become acceptable on the bench.
    if request.in_rc_failsafe && !request.new_allows_entry_in_rc_failsafe {
        return ModeEntry::Refused(ModeEntryVeto::InRcFailsafe);
    }

    if !request.init_ok {
        return ModeEntry::Refused(ModeEntryVeto::InitFailed);
    }

    ModeEntry::Entered
}
