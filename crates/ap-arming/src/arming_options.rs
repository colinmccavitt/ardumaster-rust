//! `ARMING_OPTIONS` bitfield. FW-026.
//!
//! Upstream `AP_Arming::Option` / `ARMING_OPTIONS`:
//! * bit 0 — disable periodic pre-arm failure display
//! * bit 1 — do not send status text on arm/disarm
//! * bit 2 — skip IMU consistency while an ICE motor is starting/running
//!
//! Default is 0 (every option off). Decode is [`option_enabled`]; apply is
//! whether `AP_Arming::update` reports pre-arm failures, whether
//! `send_arm_disarm_statustext` emits, and whether INS consistency
//! runs when the ICE is live. The ICE state machine itself is a later slice.

/// Default `ARMING_OPTIONS`, upstream `_arming_options` groupinfo.
pub const ARMING_OPTIONS_DEFAULT: u32 = 0;

/// Upstream `AP_Arming::Option` bits (`ARMING_OPTIONS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum ArmingOption {
    /// Bit 0 — `DISABLE_PREARM_DISPLAY`: never print pre-arm failures.
    DisablePrearmDisplay = 1 << 0,
    /// Bit 1 — `DISABLE_STATUSTEXT_ON_STATE_CHANGE`: mute arm/disarm text.
    DisableStatustextOnStateChange = 1 << 1,
    /// Bit 2 — `SKIP_IMU_CONSISTENCY_ICE_RUNNING`: skip IMU check while ICE is live.
    SkipImuConsistencyIceRunning = 1 << 2,
}

impl ArmingOption {
    /// Bit value stored in `ARMING_OPTIONS`.
    #[must_use]
    pub const fn bit(self) -> u32 {
        self as u32
    }
}

/// Decoded `ARMING_OPTIONS` bitmask, upstream `AP_Arming::_arming_options`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingOptions {
    /// Raw `ARMING_OPTIONS` integer.
    pub bits: u32,
}

impl Default for ArmingOptions {
    fn default() -> Self {
        Self {
            bits: ARMING_OPTIONS_DEFAULT,
        }
    }
}

impl ArmingOptions {
    /// Wrap a stored `ARMING_OPTIONS` value.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Self {
        Self { bits }
    }

    /// Upstream `AP_Arming::option_enabled`.
    #[must_use]
    pub const fn option_enabled(self, option: ArmingOption) -> bool {
        (self.bits & option.bit()) != 0
    }

    /// Bit 0: suppress the periodic pre-arm failure display.
    #[must_use]
    pub const fn disable_prearm_display(self) -> bool {
        self.option_enabled(ArmingOption::DisablePrearmDisplay)
    }

    /// Bit 1: mute GCS status text on arm/disarm.
    #[must_use]
    pub const fn disable_statustext_on_state_change(self) -> bool {
        self.option_enabled(ArmingOption::DisableStatustextOnStateChange)
    }

    /// Bit 2: skip IMU consistency while an ICE motor is starting/running.
    #[must_use]
    pub const fn skip_imu_consistency_ice_running(self) -> bool {
        self.option_enabled(ArmingOption::SkipImuConsistencyIceRunning)
    }
}

/// ICE motor states that gate the IMU-consistency skip.
///
/// Values match upstream `AP_ICEngine::ICE_State`. Only starting / running
/// suppress the check; every other state (or no ICE) still runs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IceState {
    /// 0 — engine off.
    Off = 0,
    /// 3 — cranking / starting.
    Starting = 3,
    /// 4 — running.
    Running = 4,
}

/// Whether `AP_Arming::update` should display pre-arm failures this tick.
///
/// Upstream sets `display_fail` from the 30 s period (or immediate
/// report), then forces it false when bit 0 is set.
#[must_use]
pub const fn apply_prearm_display(options: ArmingOptions, period_elapsed: bool) -> bool {
    period_elapsed && !options.disable_prearm_display()
}

/// Whether `send_arm_disarm_statustext` should emit.
///
/// Upstream returns early when bit 1 is set.
#[must_use]
pub const fn apply_statustext_on_state_change(options: ArmingOptions) -> bool {
    !options.disable_statustext_on_state_change()
}

/// Whether INS accel/gyro consistency should run.
///
/// Upstream starts from true. Bit 2 plus a live ICE (`Starting` or
/// `Running`) flips it false. No ICE, or ICE off, still runs the check.
#[must_use]
pub const fn apply_imu_consistency_check(options: ArmingOptions, ice: Option<IceState>) -> bool {
    if !options.skip_imu_consistency_ice_running() {
        return true;
    }
    match ice {
        Some(IceState::Starting) | Some(IceState::Running) => false,
        Some(IceState::Off) | None => true,
    }
}

/// Applied `ARMING_OPTIONS` view for one update / arm-disarm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingOptionsApplied {
    /// `display_fail` after bit 0.
    pub display_prearm_failures: bool,
    /// Emit arm/disarm status text after bit 1.
    pub send_statustext: bool,
    /// Run IMU consistency after bit 2 + ICE state.
    pub run_imu_consistency: bool,
}

/// Decode `ARMING_OPTIONS` and apply it to one update / ICE sample.
#[must_use]
pub const fn apply_arming_options(
    options: ArmingOptions,
    period_elapsed: bool,
    ice: Option<IceState>,
) -> ArmingOptionsApplied {
    ArmingOptionsApplied {
        display_prearm_failures: apply_prearm_display(options, period_elapsed),
        send_statustext: apply_statustext_on_state_change(options),
        run_imu_consistency: apply_imu_consistency_check(options, ice),
    }
}
