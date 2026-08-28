//! `FLTMODE1`–`FLTMODE6` slot mapping, upstream `Plane::flight_modes`.

use ap_rc::{
    decode_fltmode_from_channels, decode_fltmode_number, fltmode_for_slot, FltModeTable,
    FLTMODE1_DEFAULT, FLTMODE2_DEFAULT, FLTMODE3_DEFAULT, FLTMODE4_DEFAULT, FLTMODE5_DEFAULT,
    FLTMODE6_DEFAULT, FLTMODE_CH_DEFAULT, FLTMODE_CH_DISABLED, MODE_NUMBER_FLY_BY_WIRE_A,
    MODE_NUMBER_MANUAL, MODE_NUMBER_RTL, NUM_FLIGHT_MODES, RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM,
};

#[test]
fn default_fltmode_params_match_upstream_config() {
    assert_eq!(NUM_FLIGHT_MODES, 6);
    assert_eq!(FLTMODE1_DEFAULT, MODE_NUMBER_RTL);
    assert_eq!(FLTMODE2_DEFAULT, MODE_NUMBER_RTL);
    assert_eq!(FLTMODE3_DEFAULT, MODE_NUMBER_FLY_BY_WIRE_A);
    assert_eq!(FLTMODE4_DEFAULT, MODE_NUMBER_FLY_BY_WIRE_A);
    assert_eq!(FLTMODE5_DEFAULT, MODE_NUMBER_MANUAL);
    assert_eq!(FLTMODE6_DEFAULT, MODE_NUMBER_MANUAL);
    assert_eq!(MODE_NUMBER_RTL, 11);
    assert_eq!(MODE_NUMBER_FLY_BY_WIRE_A, 5);
    assert_eq!(MODE_NUMBER_MANUAL, 0);

    let table = FltModeTable::default();
    assert_eq!(table.modes, [11, 11, 5, 5, 0, 0]);
}

#[test]
fn slot_indexes_fltmode1_through_fltmode6() {
    // Custom table so each slot is a distinct mode number.
    let table = FltModeTable::from_params(11, 10, 5, 7, 0, 1);
    assert_eq!(fltmode_for_slot(&table, 0), Some(11)); // FLTMODE1 RTL
    assert_eq!(fltmode_for_slot(&table, 1), Some(10)); // FLTMODE2 AUTO
    assert_eq!(fltmode_for_slot(&table, 2), Some(5)); // FLTMODE3 FBWA
    assert_eq!(fltmode_for_slot(&table, 3), Some(7)); // FLTMODE4 CRUISE
    assert_eq!(fltmode_for_slot(&table, 4), Some(0)); // FLTMODE5 MANUAL
    assert_eq!(fltmode_for_slot(&table, 5), Some(1)); // FLTMODE6 CIRCLE
    assert_eq!(fltmode_for_slot(&table, 6), None);
}

#[test]
fn pwm_slot_maps_to_fltmoden_mode_number() {
    let table = FltModeTable::from_params(11, 10, 5, 7, 0, 1);
    // Mid-band PWM in each six-pos slot; edges stay in the FLTMODE_CH tests.
    assert_eq!(decode_fltmode_number(&table, 1100), Some(11));
    assert_eq!(decode_fltmode_number(&table, 1300), Some(10));
    assert_eq!(decode_fltmode_number(&table, 1400), Some(5));
    assert_eq!(decode_fltmode_number(&table, 1550), Some(7));
    assert_eq!(decode_fltmode_number(&table, 1680), Some(0));
    assert_eq!(decode_fltmode_number(&table, 1900), Some(1));
    assert_eq!(decode_fltmode_number(&table, RC_MIN_LIMIT_PWM), None);
    assert_eq!(decode_fltmode_number(&table, RC_MAX_LIMIT_PWM), None);
}

#[test]
fn frame_plus_table_maps_fltmode_ch_pwm_to_mode() {
    let table = FltModeTable::default();
    let mut channels = [1500_u16; 16];
    channels[7] = 1100; // channel 8, slot 0 → FLTMODE1 RTL
    assert_eq!(
        decode_fltmode_from_channels(&channels, FLTMODE_CH_DEFAULT, &table),
        Some(MODE_NUMBER_RTL)
    );
    channels[7] = 1900; // slot 5 → FLTMODE6 MANUAL
    assert_eq!(
        decode_fltmode_from_channels(&channels, FLTMODE_CH_DEFAULT, &table),
        Some(MODE_NUMBER_MANUAL)
    );
    assert_eq!(
        decode_fltmode_from_channels(&channels, FLTMODE_CH_DISABLED, &table),
        None
    );
}
