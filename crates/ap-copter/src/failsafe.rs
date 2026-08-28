//! Radio and GCS failsafe check+action leftover, upstream `ArduCopter/events.cpp`.
//!
//! Tracked as **COP-019**. This is the first vehicle-level failsafe surface:
//! when a GCS heartbeat ages out, and which [`FailsafeAction`] a radio or GCS
//! loss would ask `do_failsafe_action` to run. The radio *check* that latches
//! `failsafe.radio` lives in `radio.cpp` (not this ticket). Crash detection
//! and `ModeBrake` are later leftovers in the same ticket.
//!
//! # The check is an edge, not a level
//!
//! `failsafe_gcs_check` compares `millis() - last_seen` against
//! `FS_GCS_TIMEOUT` with a strict `<` / `>` pair. Equality does nothing —
//! the latch stays where it was. A port that used `>=` would trip one
//! millisecond early and would not be this function.
//!
//! Disabled (`FS_GCS_ENABLE == 0`) and a GCS that has never spoken
//! (`last_seen == 0`) also do nothing. Heartbeat tracking only starts after
//! the first packet from `sysid_mygcs`.
//!
//! # The action tables are not the same
//!
//! Radio (`FS_THR_ENABLE`) and GCS (`FS_GCS_ENABLE`) share most of their
//! numbering, but an unrecognised parameter falls to LAND on the radio path
//! and RTL on the GCS path. The override ladders differ too: GCS can refuse
//! to act when the aircraft is already disarmed, and it has its own
//! continue-in-pilot-control option. Folding the two tables together would
//! hide both of those.

use ap_math::scalar::constrain_value;

/// Upstream `Copter::FailsafeAction`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailsafeAction {
    /// 0 — take no mode-change action.
    None = 0,
    /// 1 — `set_mode_land_with_pause`.
    Land = 1,
    /// 2 — `set_mode_RTL_or_land_with_pause`.
    Rtl = 2,
    /// 3 — `set_mode_SmartRTL_or_RTL`.
    SmartRtl = 3,
    /// 4 — `set_mode_SmartRTL_or_land_with_pause`.
    SmartRtlLand = 4,
    /// 5 — terminate / disarm. Not produced by the radio or GCS tables.
    Terminate = 5,
    /// 6 — `set_mode_auto_do_land_start_or_RTL`.
    AutoDoLandStart = 6,
    /// 7 — `set_mode_brake_or_land_with_pause`.
    BrakeLand = 7,
}

/// Upstream `Copter::FailsafeOption` bits on `FS_OPTIONS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum FailsafeOption {
    /// Bit 0 — radio failsafe continues a running AUTO mission.
    RcContinueIfAuto = 1 << 0,
    /// Bit 1 — GCS failsafe continues a running AUTO mission.
    GcsContinueIfAuto = 1 << 1,
    /// Bit 2 — radio failsafe continues Guided.
    RcContinueIfGuided = 1 << 2,
    /// Bit 3 — either failsafe continues a landing already in progress.
    ContinueIfLanding = 1 << 3,
    /// Bit 4 — GCS failsafe continues a pilot-controlled mode.
    GcsContinueIfPilotControl = 1 << 4,
    /// Bit 5 — `do_failsafe_action` also releases the gripper.
    ReleaseGripper = 1 << 5,
}

/// Upstream `FS_THR_ENABLE` / `defines.h` radio-failsafe values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FsThrEnable {
    /// 0 — `FS_THR_DISABLED`.
    Disabled = 0,
    /// 1 — `FS_THR_ENABLED_ALWAYS_RTL`. Parameter default.
    AlwaysRtl = 1,
    /// 2 — `FS_THR_ENABLED_CONTINUE_MISSION`. Removed in 4.0+; still maps to RTL.
    ContinueMission = 2,
    /// 3 — `FS_THR_ENABLED_ALWAYS_LAND`.
    AlwaysLand = 3,
    /// 4 — `FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL`.
    AlwaysSmartrtlOrRtl = 4,
    /// 5 — `FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND`.
    AlwaysSmartrtlOrLand = 5,
    /// 6 — `FS_THR_ENABLED_AUTO_RTL_OR_RTL`.
    AutoRtlOrRtl = 6,
    /// 7 — `FS_THR_ENABLED_BRAKE_OR_LAND`.
    BrakeOrLand = 7,
}

/// Upstream `FS_GCS_ENABLE` / `defines.h` GCS-failsafe values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FsGcsEnable {
    /// 0 — `FS_GCS_DISABLED`. Parameter default.
    Disabled = 0,
    /// 1 — `FS_GCS_ENABLED_ALWAYS_RTL`.
    AlwaysRtl = 1,
    /// 2 — `FS_GCS_ENABLED_CONTINUE_MISSION`. Removed in 4.0+; still maps to RTL.
    ContinueMission = 2,
    /// 3 — `FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL`.
    AlwaysSmartrtlOrRtl = 3,
    /// 4 — `FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND`.
    AlwaysSmartrtlOrLand = 4,
    /// 5 — `FS_GCS_ENABLED_ALWAYS_LAND`.
    AlwaysLand = 5,
    /// 6 — `FS_GCS_ENABLED_AUTO_RTL_OR_RTL`.
    AutoRtlOrRtl = 6,
    /// 7 — `FS_GCS_ENABLED_BRAKE_OR_LAND`.
    BrakeOrLand = 7,
}

/// Upstream `FS_GCS_TIMEOUT` default, seconds.
pub const FS_GCS_TIMEOUT_DEFAULT_S: f32 = 5.0;

/// `Mode::Number::STABILIZE`.
pub const MODE_STABILIZE: u8 = 0;
/// `Mode::Number::ACRO`.
pub const MODE_ACRO: u8 = 1;
/// `Mode::Number::AUTO`.
pub const MODE_AUTO: u8 = 3;
/// `Mode::Number::AUTO_RTL`.
pub const MODE_AUTO_RTL: u8 = 27;

impl FsThrEnable {
    /// Decode `FS_THR_ENABLE`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::AlwaysRtl),
            2 => Some(Self::ContinueMission),
            3 => Some(Self::AlwaysLand),
            4 => Some(Self::AlwaysSmartrtlOrRtl),
            5 => Some(Self::AlwaysSmartrtlOrLand),
            6 => Some(Self::AutoRtlOrRtl),
            7 => Some(Self::BrakeOrLand),
            _ => None,
        }
    }

    /// Upstream `FS_THR_ENABLE` default, `FS_THR_ENABLED_ALWAYS_RTL`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::AlwaysRtl
    }
}

impl FsGcsEnable {
    /// Decode `FS_GCS_ENABLE`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::AlwaysRtl),
            2 => Some(Self::ContinueMission),
            3 => Some(Self::AlwaysSmartrtlOrRtl),
            4 => Some(Self::AlwaysSmartrtlOrLand),
            5 => Some(Self::AlwaysLand),
            6 => Some(Self::AutoRtlOrRtl),
            7 => Some(Self::BrakeOrLand),
            _ => None,
        }
    }

    /// Upstream `FS_GCS_ENABLE` default, `FS_GCS_DISABLED`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::Disabled
    }

    /// Whether `failsafe_gcs_check` will look at the heartbeat age at all.
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        !matches!(self, Self::Disabled)
    }
}

/// Upstream `Copter::failsafe_option`.
#[must_use]
pub const fn failsafe_option(fs_options: u32, opt: FailsafeOption) -> bool {
    fs_options & (opt as u32) != 0
}

/// Radio `FS_THR_ENABLE` switch, before the override ladder.
///
/// An unrecognised value falls to [`FailsafeAction::Land`].
#[must_use]
pub const fn radio_param_action(failsafe_throttle: u8) -> FailsafeAction {
    match failsafe_throttle {
        0 => FailsafeAction::None,
        1 | 2 => FailsafeAction::Rtl,
        3 => FailsafeAction::Land,
        4 => FailsafeAction::SmartRtl,
        5 => FailsafeAction::SmartRtlLand,
        6 => FailsafeAction::AutoDoLandStart,
        7 => FailsafeAction::BrakeLand,
        _ => FailsafeAction::Land,
    }
}

/// GCS `FS_GCS_ENABLE` switch, before the override ladder.
///
/// An unrecognised value falls to [`FailsafeAction::Rtl`], not Land.
#[must_use]
pub const fn gcs_param_action(failsafe_gcs: u8) -> FailsafeAction {
    match failsafe_gcs {
        0 => FailsafeAction::None,
        1 | 2 => FailsafeAction::Rtl,
        3 => FailsafeAction::SmartRtl,
        4 => FailsafeAction::SmartRtlLand,
        5 => FailsafeAction::Land,
        6 => FailsafeAction::AutoDoLandStart,
        7 => FailsafeAction::BrakeLand,
        _ => FailsafeAction::Rtl,
    }
}

/// Inputs for `Copter::failsafe_gcs_check`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcsFailsafeInputs {
    /// `g.failsafe_gcs` / `FS_GCS_ENABLE`.
    pub enable: u8,
    /// `millis()`.
    pub now_ms: u32,
    /// `gcs().sysid_mygcs_last_seen_time_ms()`. Zero until the first heartbeat.
    pub last_seen_ms: u32,
    /// `g2.fs_gcs_timeout`, seconds.
    pub timeout_s: f32,
    /// `failsafe.gcs` already latched.
    pub already_gcs: bool,
}

impl Default for GcsFailsafeInputs {
    fn default() -> Self {
        Self {
            enable: FsGcsEnable::Disabled as u8,
            now_ms: 0,
            last_seen_ms: 0,
            timeout_s: FS_GCS_TIMEOUT_DEFAULT_S,
            already_gcs: false,
        }
    }
}

/// What `failsafe_gcs_check` would do to the latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GcsFailsafeEdge {
    /// Leave `failsafe.gcs` where it is. No on/off event.
    Hold,
    /// `set_failsafe_gcs(true)` then `failsafe_gcs_on_event`.
    Trigger,
    /// `set_failsafe_gcs(false)` then `failsafe_gcs_off_event`.
    Recover,
}

/// `uint32_t(constrain_float(fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX))`.
#[must_use]
pub fn gcs_timeout_ms(timeout_s: f32) -> u32 {
    let ms = constrain_value(timeout_s * 1000.0, 0.0, u32::MAX as f32);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "upstream casts the constrained float to uint32_t"
    )]
    {
        ms as u32
    }
}

/// GCS failsafe check leftover, upstream `Copter::failsafe_gcs_check`.
#[must_use]
pub fn failsafe_gcs_check(inp: &GcsFailsafeInputs) -> GcsFailsafeEdge {
    if inp.enable == FsGcsEnable::Disabled as u8 {
        return GcsFailsafeEdge::Hold;
    }
    if inp.last_seen_ms == 0 {
        return GcsFailsafeEdge::Hold;
    }
    let age_ms = inp.now_ms.wrapping_sub(inp.last_seen_ms);
    let timeout_ms = gcs_timeout_ms(inp.timeout_s);
    if age_ms < timeout_ms && inp.already_gcs {
        GcsFailsafeEdge::Recover
    } else if age_ms > timeout_ms && !inp.already_gcs {
        GcsFailsafeEdge::Trigger
    } else {
        // Healthy and clear, already latched, or exactly on the timeout.
        GcsFailsafeEdge::Hold
    }
}

/// Inputs for `Copter::failsafe_radio_on_event`'s action selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFailsafeInputs {
    /// `g.failsafe_throttle` / `FS_THR_ENABLE`.
    pub failsafe_throttle: u8,
    /// `should_disarm_on_failsafe()`.
    pub should_disarm: bool,
    /// `flightmode->is_landing()`.
    pub is_landing: bool,
    /// Battery has failsafed at LAND priority or higher.
    pub battery_requires_land: bool,
    /// `flightmode->mode_number() == AUTO`.
    pub mode_is_auto: bool,
    /// `flightmode->in_guided_mode()`.
    pub in_guided_mode: bool,
    /// `g2.fs_options`.
    pub fs_options: u32,
}

impl Default for RadioFailsafeInputs {
    fn default() -> Self {
        Self {
            failsafe_throttle: FsThrEnable::AlwaysRtl as u8,
            should_disarm: false,
            is_landing: false,
            battery_requires_land: false,
            mode_is_auto: false,
            in_guided_mode: false,
            fs_options: 0,
        }
    }
}

/// Radio failsafe action leftover, upstream `Copter::failsafe_radio_on_event`.
///
/// Logging and the GCS announce string are left to the caller. This returns
/// the `desired_action` handed to `do_failsafe_action`.
#[must_use]
pub fn failsafe_radio_on_event(inp: &RadioFailsafeInputs) -> FailsafeAction {
    let mut desired = radio_param_action(inp.failsafe_throttle);
    if inp.should_disarm {
        desired = FailsafeAction::None;
    } else if inp.is_landing && inp.battery_requires_land {
        desired = FailsafeAction::Land;
    } else if inp.is_landing && failsafe_option(inp.fs_options, FailsafeOption::ContinueIfLanding) {
        desired = FailsafeAction::Land;
    } else if inp.mode_is_auto && failsafe_option(inp.fs_options, FailsafeOption::RcContinueIfAuto)
    {
        desired = FailsafeAction::None;
    } else if inp.in_guided_mode
        && failsafe_option(inp.fs_options, FailsafeOption::RcContinueIfGuided)
    {
        desired = FailsafeAction::None;
    }
    desired
}

/// Inputs for `Copter::failsafe_gcs_on_event`'s action selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GcsFailsafeActionInputs {
    /// `g.failsafe_gcs` / `FS_GCS_ENABLE`.
    pub failsafe_gcs: u8,
    /// `motors->armed()`.
    pub armed: bool,
    /// `should_disarm_on_failsafe()`.
    pub should_disarm: bool,
    /// `flightmode->is_landing()`.
    pub is_landing: bool,
    /// Battery has failsafed at LAND priority or higher.
    pub battery_requires_land: bool,
    /// `flightmode->mode_number() == AUTO`.
    pub mode_is_auto: bool,
    /// `flightmode->is_autopilot()`.
    pub is_autopilot: bool,
    /// `g2.fs_options`.
    pub fs_options: u32,
}

impl Default for GcsFailsafeActionInputs {
    fn default() -> Self {
        Self {
            failsafe_gcs: FsGcsEnable::AlwaysRtl as u8,
            armed: true,
            should_disarm: false,
            is_landing: false,
            battery_requires_land: false,
            mode_is_auto: false,
            is_autopilot: false,
            fs_options: 0,
        }
    }
}

/// GCS failsafe action leftover, upstream `Copter::failsafe_gcs_on_event`.
#[must_use]
pub fn failsafe_gcs_on_event(inp: &GcsFailsafeActionInputs) -> FailsafeAction {
    let mut desired = gcs_param_action(inp.failsafe_gcs);
    if !inp.armed {
        desired = FailsafeAction::None;
    } else if inp.should_disarm {
        desired = FailsafeAction::None;
    } else if inp.is_landing && inp.battery_requires_land {
        desired = FailsafeAction::Land;
    } else if inp.is_landing && failsafe_option(inp.fs_options, FailsafeOption::ContinueIfLanding) {
        desired = FailsafeAction::Land;
    } else if inp.mode_is_auto && failsafe_option(inp.fs_options, FailsafeOption::GcsContinueIfAuto)
    {
        desired = FailsafeAction::None;
    } else if failsafe_option(inp.fs_options, FailsafeOption::GcsContinueIfPilotControl)
        && !inp.is_autopilot
    {
        desired = FailsafeAction::None;
    }
    desired
}

/// Upstream `Copter::should_disarm_on_failsafe`.
///
/// `mode` is `Mode::Number`. Stabilize/Acro disarm on zero throttle *or*
/// landed; AUTO / AUTO_RTL only when the mission has not started and the
/// aircraft is landed; every other mode disarms only when landed. An arming
/// delay short-circuits all of that.
#[must_use]
pub const fn should_disarm_on_failsafe(
    in_arming_delay: bool,
    mode: u8,
    throttle_zero: bool,
    land_complete: bool,
    auto_armed: bool,
) -> bool {
    if in_arming_delay {
        return true;
    }
    match mode {
        MODE_STABILIZE | MODE_ACRO => throttle_zero || land_complete,
        MODE_AUTO | MODE_AUTO_RTL => !auto_armed && land_complete,
        _ => land_complete,
    }
}
