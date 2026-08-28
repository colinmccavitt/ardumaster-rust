//! `INITIAL_MODE` and boot-mode-from-switch.
//!
//! Upstream `Plane::init_ardupilot` starts in `INITIAL_MODE`
//! (`ModeReason::INITIALISED`), then calls `rc().reset_mode_switch()`.
//! A valid `FLTMODE_CH` pulse that decodes through `FLTMODE1`-`FLTMODE6`
//! overwrites that with `ModeReason::RC_COMMAND`. A disabled switch or
//! an invalid pulse leaves `INITIAL_MODE` in place -- useful for AUTO on
//! boot without a receiver.
//!
//! PWM edges and the `FLTMODE1`-`FLTMODE6` table stay in [`crate::fltmode`];
//! this module is only the boot choice.

use crate::fltmode::{
    decode_fltmode_from_channels, decode_fltmode_number, FltModeTable, MODE_NUMBER_MANUAL,
};

/// Upstream `INITIAL_MODE` default (`Mode::Number::MANUAL`).
pub const INITIAL_MODE_DEFAULT: i8 = MODE_NUMBER_MANUAL;
/// Upstream `Mode::Number::AUTO` -- typical `INITIAL_MODE` without a receiver.
pub const MODE_NUMBER_AUTO: i8 = 10;

/// Upstream `ModeReason::RC_COMMAND`.
pub const MODE_REASON_RC_COMMAND: u8 = 1;
/// Upstream `ModeReason::INITIALISED`.
pub const MODE_REASON_INITIALISED: u8 = 26;

/// Why the boot mode was chosen, upstream `ModeReason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootModeReason {
    /// `set_mode_by_number(g.initial_mode, ModeReason::INITIALISED)`.
    Initialised,
    /// Flight-mode switch after `reset_mode_switch` / `mode_switch_changed`.
    RcCommand,
}

impl BootModeReason {
    /// Upstream `ModeReason` discriminant.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::RcCommand => MODE_REASON_RC_COMMAND,
            Self::Initialised => MODE_REASON_INITIALISED,
        }
    }
}

/// Settled mode after the boot sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootMode {
    /// `Mode::Number` that is live after boot.
    pub mode: i8,
    /// `ModeReason` that selected it.
    pub reason: BootModeReason,
}

/// Mode applied from `INITIAL_MODE` before the switch is read.
#[must_use]
pub const fn boot_from_initial_mode(initial_mode: i8) -> BootMode {
    BootMode {
        mode: initial_mode,
        reason: BootModeReason::Initialised,
    }
}

/// Settled boot mode from a mode-switch PWM (no channel lookup).
///
/// Invalid PWM keeps `initial_mode`. Slot mapping is [`decode_fltmode_number`].
#[must_use]
pub fn boot_mode_from_switch_pwm(initial_mode: i8, table: &FltModeTable, pwm: u16) -> BootMode {
    if let Some(mode) = decode_fltmode_number(table, pwm) {
        BootMode {
            mode,
            reason: BootModeReason::RcCommand,
        }
    } else {
        boot_from_initial_mode(initial_mode)
    }
}

/// Settled boot mode: `INITIAL_MODE`, then the flight-mode switch if it decodes.
///
/// Upstream sequence is `set_mode(INITIAL_MODE)` then `reset_mode_switch`.
/// This returns the mode after switch debounce has completed. No valid
/// `FLTMODE_CH` / pulse keeps `INITIAL_MODE`.
#[must_use]
pub fn boot_mode_from_switch(
    initial_mode: i8,
    fltmode_ch: i8,
    table: &FltModeTable,
    channels: &[u16],
) -> BootMode {
    if let Some(mode) = decode_fltmode_from_channels(channels, fltmode_ch, table) {
        BootMode {
            mode,
            reason: BootModeReason::RcCommand,
        }
    } else {
        boot_from_initial_mode(initial_mode)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aux_switch::{RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM};
    use crate::fltmode::{FLTMODE_CH_DISABLED, MODE_NUMBER_RTL};

    #[test]
    fn default_initial_mode_is_manual() {
        assert_eq!(INITIAL_MODE_DEFAULT, MODE_NUMBER_MANUAL);
        assert_eq!(INITIAL_MODE_DEFAULT, 0);
        assert_eq!(MODE_NUMBER_AUTO, 10);
        assert_eq!(MODE_REASON_INITIALISED, 26);
        assert_eq!(MODE_REASON_RC_COMMAND, 1);
        let boot = boot_from_initial_mode(INITIAL_MODE_DEFAULT);
        assert_eq!(boot.mode, MODE_NUMBER_MANUAL);
        assert_eq!(boot.reason, BootModeReason::Initialised);
        assert_eq!(boot.reason.as_u8(), MODE_REASON_INITIALISED);
    }

    #[test]
    fn disabled_switch_keeps_initial_mode() {
        let table = FltModeTable::default();
        let channels = [1500_u16; 16];
        let boot = boot_mode_from_switch(MODE_NUMBER_AUTO, FLTMODE_CH_DISABLED, &table, &channels);
        assert_eq!(boot.mode, MODE_NUMBER_AUTO);
        assert_eq!(boot.reason, BootModeReason::Initialised);
    }

    #[test]
    fn valid_switch_pwm_overrides_initial_mode() {
        let table = FltModeTable::default();
        // Slot 0 PWM -> FLTMODE1 RTL, even if INITIAL_MODE is AUTO.
        let boot = boot_mode_from_switch_pwm(MODE_NUMBER_AUTO, &table, 1100);
        assert_eq!(boot.mode, MODE_NUMBER_RTL);
        assert_eq!(boot.reason, BootModeReason::RcCommand);
        assert_eq!(boot.reason.as_u8(), MODE_REASON_RC_COMMAND);
    }

    #[test]
    fn invalid_pwm_keeps_initial_mode() {
        let table = FltModeTable::default();
        let boot = boot_mode_from_switch_pwm(MODE_NUMBER_AUTO, &table, RC_MIN_LIMIT_PWM);
        assert_eq!(boot.mode, MODE_NUMBER_AUTO);
        assert_eq!(boot.reason, BootModeReason::Initialised);
        let boot = boot_mode_from_switch_pwm(MODE_NUMBER_AUTO, &table, RC_MAX_LIMIT_PWM);
        assert_eq!(boot.mode, MODE_NUMBER_AUTO);
        assert_eq!(boot.reason, BootModeReason::Initialised);
    }
}
