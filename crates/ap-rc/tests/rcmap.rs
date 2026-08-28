//! RC channel map and trim persist, upstream `AP_RCMapper` / `set_and_save_trim`.

use ap_rc::{
    mapped_pwm, persist_stick_trims, rcmap_index, set_and_save_trim, RcChannel, RcMap,
    RCMAP_CHANNEL_MAX, RCMAP_PITCH_DEFAULT, RCMAP_ROLL_DEFAULT, RCMAP_THROTTLE_DEFAULT,
    RCMAP_YAW_DEFAULT,
};

#[test]
fn default_rcmap_is_roll_pitch_throttle_yaw() {
    let map = RcMap::default();
    assert_eq!(map.roll, RCMAP_ROLL_DEFAULT);
    assert_eq!(map.pitch, RCMAP_PITCH_DEFAULT);
    assert_eq!(map.throttle, RCMAP_THROTTLE_DEFAULT);
    assert_eq!(map.yaw, RCMAP_YAW_DEFAULT);
    assert_eq!(RCMAP_CHANNEL_MAX, 16);
}

#[test]
fn mapped_pwm_follows_one_based_rcmap() {
    let frame = [1111_u16, 1222, 1333, 1444, 1555];
    assert_eq!(mapped_pwm(&frame, 1), Some(1111));
    assert_eq!(mapped_pwm(&frame, 4), Some(1444));
    assert_eq!(mapped_pwm(&frame, 5), Some(1555));
    assert_eq!(mapped_pwm(&frame, 6), None);
    assert_eq!(rcmap_index(0), None);
}

#[test]
fn remapped_sticks_swap_receiver_order() {
    // JR/Spektrum-style: throttle on 1, roll on 2, pitch on 3, yaw on 4.
    let map = RcMap::from_params(2, 3, 1, 4);
    let frame = [1000_u16, 1600, 1400, 1500];
    let sticks = map.map_sticks(&frame);
    assert_eq!(sticks.throttle, Some(1000));
    assert_eq!(sticks.roll, Some(1600));
    assert_eq!(sticks.pitch, Some(1400));
    assert_eq!(sticks.yaw, Some(1500));
}

#[test]
fn set_and_save_trim_persists_radio_in_if_changed() {
    let mut ch = RcChannel::default();
    let same = ch.radio_trim;
    assert!(!set_and_save_trim(&mut ch, same));
    assert!(set_and_save_trim(&mut ch, 1475));
    assert_eq!(ch.radio_trim, 1475);
}

#[test]
fn persist_stick_trims_writes_roll_pitch_rudder_only() {
    let mut roll = RcChannel::default();
    let mut pitch = RcChannel::default();
    let mut yaw = RcChannel::default();
    assert!(persist_stick_trims(
        &mut roll, &mut pitch, &mut yaw, 1490, 1510, 1505
    ));
    assert_eq!(roll.radio_trim, 1490);
    assert_eq!(pitch.radio_trim, 1510);
    assert_eq!(yaw.radio_trim, 1505);
    assert!(!persist_stick_trims(
        &mut roll, &mut pitch, &mut yaw, 1490, 1510, 1505
    ));
}
