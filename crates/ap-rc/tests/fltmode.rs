//! Flight-mode switch / `FLTMODE_CH` decode, upstream `read_6pos_switch`.

use ap_rc::{
    decode_fltmode_ch, decode_fltmode_switch, flight_mode_channel_index,
    flight_mode_channel_pwm, fltmode_ch_valid, read_6pos_switch, FLTMODE_CH_DEFAULT,
    FLTMODE_CH_DISABLED, FLTMODE_POS0_MAX_PWM, FLTMODE_POS1_MAX_PWM, FLTMODE_POS2_MAX_PWM,
    FLTMODE_POS3_MAX_PWM, FLTMODE_POS4_MAX_PWM, NUM_RC_CHANNELS, RC_MAX_LIMIT_PWM,
    RC_MIN_LIMIT_PWM,
};

#[test]
fn fltmode_ch_default_is_channel_eight() {
    assert_eq!(FLTMODE_CH_DEFAULT, 8);
    assert_eq!(FLTMODE_CH_DISABLED, 0);
    assert_eq!(NUM_RC_CHANNELS, 16);
    assert!(fltmode_ch_valid(FLTMODE_CH_DEFAULT));
    assert_eq!(flight_mode_channel_index(FLTMODE_CH_DEFAULT), Some(7));
    assert!(!fltmode_ch_valid(FLTMODE_CH_DISABLED));
    assert!(!fltmode_ch_valid(NUM_RC_CHANNELS));
}

#[test]
fn six_pos_thresholds_match_upstream_read_6pos_switch() {
    assert_eq!(FLTMODE_POS0_MAX_PWM, 1231);
    assert_eq!(FLTMODE_POS1_MAX_PWM, 1361);
    assert_eq!(FLTMODE_POS2_MAX_PWM, 1491);
    assert_eq!(FLTMODE_POS3_MAX_PWM, 1621);
    assert_eq!(FLTMODE_POS4_MAX_PWM, 1750);

    assert_eq!(read_6pos_switch(FLTMODE_POS0_MAX_PWM - 1), Some(0));
    assert_eq!(read_6pos_switch(FLTMODE_POS0_MAX_PWM), Some(1));
    assert_eq!(read_6pos_switch(FLTMODE_POS1_MAX_PWM - 1), Some(1));
    assert_eq!(read_6pos_switch(FLTMODE_POS1_MAX_PWM), Some(2));
    assert_eq!(read_6pos_switch(FLTMODE_POS2_MAX_PWM - 1), Some(2));
    assert_eq!(read_6pos_switch(FLTMODE_POS2_MAX_PWM), Some(3));
    assert_eq!(read_6pos_switch(FLTMODE_POS3_MAX_PWM - 1), Some(3));
    assert_eq!(read_6pos_switch(FLTMODE_POS3_MAX_PWM), Some(4));
    assert_eq!(read_6pos_switch(FLTMODE_POS4_MAX_PWM - 1), Some(4));
    assert_eq!(read_6pos_switch(FLTMODE_POS4_MAX_PWM), Some(5));
}

#[test]
fn invalid_or_disabled_channel_does_not_decode() {
    assert_eq!(RC_MIN_LIMIT_PWM, 800);
    assert_eq!(RC_MAX_LIMIT_PWM, 2200);
    assert_eq!(decode_fltmode_ch(FLTMODE_CH_DISABLED, 1500), None);
    assert_eq!(decode_fltmode_ch(NUM_RC_CHANNELS, 1500), None);
    assert_eq!(decode_fltmode_ch(FLTMODE_CH_DEFAULT, RC_MIN_LIMIT_PWM), None);
    assert_eq!(decode_fltmode_ch(FLTMODE_CH_DEFAULT, RC_MAX_LIMIT_PWM), None);
    assert_eq!(decode_fltmode_ch(FLTMODE_CH_DEFAULT, 1500), Some(3));
}

#[test]
fn decode_reads_the_fltmode_channel_from_the_frame() {
    // 16-wide receiver frame; channel 8 (index 7) is the default mode switch.
    let mut channels = [1500_u16; 16];
    channels[7] = 1100;
    channels[4] = 1900;
    assert_eq!(
        flight_mode_channel_pwm(&channels, FLTMODE_CH_DEFAULT),
        Some(1100)
    );
    assert_eq!(decode_fltmode_switch(&channels, FLTMODE_CH_DEFAULT), Some(0));
    assert_eq!(decode_fltmode_switch(&channels, 5), Some(5));
    assert_eq!(decode_fltmode_switch(&channels, 0), None);
    // Channel 16 is rejected before the pulse is read.
    assert_eq!(decode_fltmode_switch(&channels, 16), None);
}
