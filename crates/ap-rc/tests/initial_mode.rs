//! `INITIAL_MODE` / boot-mode-from-switch, upstream `Plane::init_ardupilot`.
//!
//! Boot applies `INITIAL_MODE` (`ModeReason::INITIALISED`), then
//! `rc().reset_mode_switch()`. A valid `FLTMODE_CH` decode overwrites
//! that with the `FLTMODE1`-`FLTMODE6` slot (`ModeReason::RC_COMMAND`).

use ap_rc::{
    boot_from_initial_mode, boot_mode_from_switch, boot_mode_from_switch_pwm, BootModeReason,
    FltModeTable, FLTMODE_CH_DEFAULT, FLTMODE_CH_DISABLED, INITIAL_MODE_DEFAULT, MODE_NUMBER_AUTO,
    MODE_NUMBER_MANUAL, MODE_NUMBER_RTL, MODE_REASON_INITIALISED, MODE_REASON_RC_COMMAND,
    RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM,
};

#[test]
fn initial_mode_default_is_manual() {
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
fn no_receiver_keeps_initial_mode() {
    // Param text: start in AUTO on boot without a receiver.
    let table = FltModeTable::default();
    let channels = [1500_u16; 16];
    let boot = boot_mode_from_switch(MODE_NUMBER_AUTO, FLTMODE_CH_DISABLED, &table, &channels);
    assert_eq!(boot.mode, MODE_NUMBER_AUTO);
    assert_eq!(boot.reason, BootModeReason::Initialised);

    // Invalid pulse on a live FLTMODE_CH also leaves INITIAL_MODE.
    let mut live = [1500_u16; 16];
    live[7] = RC_MIN_LIMIT_PWM;
    let boot = boot_mode_from_switch(MODE_NUMBER_AUTO, FLTMODE_CH_DEFAULT, &table, &live);
    assert_eq!(boot.mode, MODE_NUMBER_AUTO);
    assert_eq!(boot.reason, BootModeReason::Initialised);
    assert_eq!(
        boot_mode_from_switch_pwm(MODE_NUMBER_AUTO, &table, RC_MAX_LIMIT_PWM).mode,
        MODE_NUMBER_AUTO
    );
}

#[test]
fn flight_mode_switch_overrides_initial_mode_on_boot() {
    let table = FltModeTable::default();
    let mut channels = [1500_u16; 16];
    // Channel 8, slot 0 -> FLTMODE1 RTL.
    channels[7] = 1100;
    let boot = boot_mode_from_switch(MODE_NUMBER_AUTO, FLTMODE_CH_DEFAULT, &table, &channels);
    assert_eq!(boot.mode, MODE_NUMBER_RTL);
    assert_eq!(boot.reason, BootModeReason::RcCommand);
    assert_eq!(boot.reason.as_u8(), MODE_REASON_RC_COMMAND);

    // Mid-band slot 5 -> FLTMODE6 Manual.
    channels[7] = 1900;
    let boot = boot_mode_from_switch(INITIAL_MODE_DEFAULT, FLTMODE_CH_DEFAULT, &table, &channels);
    assert_eq!(boot.mode, MODE_NUMBER_MANUAL);
    assert_eq!(boot.reason, BootModeReason::RcCommand);

    // PWM helper: slot 2 -> FLTMODE3 FBWA default (5).
    let boot = boot_mode_from_switch_pwm(MODE_NUMBER_AUTO, &table, 1400);
    assert_eq!(boot.mode, 5);
    assert_eq!(boot.reason, BootModeReason::RcCommand);
}
