//! `ModeSystemId` init leftover, upstream `ArduCopter/mode_systemid.cpp`.

use ap_copter::mode_loiter::MODE_NUMBER_LOITER;
use ap_copter::mode_systemid::{
    is_poscontrol_axis_type, systemid_enabled, systemid_has_user_takeoff, systemid_init,
    systemid_mode_flags, SystemIdAxis, SystemIdInitFail, SystemIdInitView, SystemIdState,
    MODE_NUMBER_SYSTEMID, SYSTEMID_AXIS_DEFAULT, SYSTEMID_F_START_HZ_DEFAULT,
    SYSTEMID_F_STOP_HZ_DEFAULT, SYSTEMID_MAGNITUDE_DEFAULT, SYSTEMID_T_FADE_IN_DEFAULT,
    SYSTEMID_T_FADE_OUT_DEFAULT, SYSTEMID_T_REC_DEFAULT, SYSTEM_ID_DELAY_S,
};

#[test]
fn systemid_number_is_twenty_five() {
    assert_eq!(MODE_NUMBER_SYSTEMID, 25);
    assert_eq!(systemid_mode_flags().mode_number, MODE_NUMBER_SYSTEMID);
}

#[test]
fn systemid_flags_are_manual_throttle_no_arming() {
    let flags = systemid_mode_flags();
    assert!(!flags.requires_position);
    assert!(flags.has_manual_throttle);
    assert!(!flags.allows_arming);
    assert!(!flags.is_autopilot);
    assert!(flags.logs_attitude);
}

#[test]
fn user_takeoff_is_never_allowed() {
    assert!(!systemid_has_user_takeoff(false));
    assert!(!systemid_has_user_takeoff(true));
}

#[test]
fn enabled_is_nonzero_axis() {
    assert!(!systemid_enabled(0));
    assert!(systemid_enabled(1));
    assert!(systemid_enabled(19));
}

#[test]
fn constructor_defaults_match_upstream() {
    assert_eq!(SYSTEMID_AXIS_DEFAULT, 0);
    assert_eq!(SYSTEMID_MAGNITUDE_DEFAULT.to_bits(), 15.0f32.to_bits());
    assert_eq!(SYSTEMID_F_START_HZ_DEFAULT.to_bits(), 0.5f32.to_bits());
    assert_eq!(SYSTEMID_F_STOP_HZ_DEFAULT.to_bits(), 40.0f32.to_bits());
    assert_eq!(SYSTEMID_T_FADE_IN_DEFAULT.to_bits(), 15.0f32.to_bits());
    assert_eq!(SYSTEMID_T_REC_DEFAULT.to_bits(), 70.0f32.to_bits());
    assert_eq!(SYSTEMID_T_FADE_OUT_DEFAULT.to_bits(), 2.0f32.to_bits());
    assert_eq!(SYSTEM_ID_DELAY_S.to_bits(), 1.0f32.to_bits());
}

#[test]
fn poscontrol_axes_are_fourteen_through_nineteen() {
    for axis in 0..=13 {
        assert!(!is_poscontrol_axis_type(axis));
    }
    for axis in 14..=19 {
        assert!(is_poscontrol_axis_type(axis));
        assert_eq!(
            SystemIdAxis::from_i8(axis).map(is_poscontrol_via_enum),
            Some(true)
        );
    }
    assert!(!is_poscontrol_axis_type(20));
    assert!(SystemIdAxis::from_i8(20).is_none());
}

fn is_poscontrol_via_enum(axis: SystemIdAxis) -> bool {
    is_poscontrol_axis_type(axis as i8)
}

#[test]
fn axis_zero_fails_before_flying_or_chirp() {
    let mut view = SystemIdInitView::typical();
    view.axis = 0;
    view.armed = false;
    view.land_complete = true;
    let out = systemid_init(true, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(SystemIdInitFail::AxisNone));
    assert!(!out.chirp_init);
    assert!(!out.set_ne_max_speed_accel);
    assert!(!out.set_d_max_speed_accel);
    assert!(out.att_bf_feedforward.is_none());
    assert!(out.state.is_none());
}

#[test]
fn not_flying_fails_when_disarmed_or_landed() {
    let mut disarmed = SystemIdInitView::typical();
    disarmed.armed = false;
    let out = systemid_init(false, &disarmed);
    assert_eq!(out.fail, Some(SystemIdInitFail::NotFlying));
    assert!(!out.ok);
    assert!(!out.chirp_init);

    let mut not_auto = SystemIdInitView::typical();
    not_auto.auto_armed = false;
    assert_eq!(
        systemid_init(true, &not_auto).fail,
        Some(SystemIdInitFail::NotFlying)
    );

    let mut landed = SystemIdInitView::typical();
    landed.land_complete = true;
    assert_eq!(
        systemid_init(true, &landed).fail,
        Some(SystemIdInitFail::NotFlying)
    );
}

#[test]
fn attitude_axis_requires_manual_throttle() {
    let mut view = SystemIdInitView::typical();
    view.from_has_manual_throttle = false;
    view.from_mode_number = MODE_NUMBER_LOITER;
    let out = systemid_init(true, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(SystemIdInitFail::NeedsManualThrottle));
    assert!(!out.chirp_init);
    assert!(out.target_pos_ne_m.is_none());
}

#[test]
fn attitude_axis_starts_chirp_without_seating_ne_or_d() {
    let view = SystemIdInitView::typical();
    let out = systemid_init(false, &view);
    assert!(out.ok);
    assert!(out.fail.is_none());
    assert!(!out.init_ne_controller);
    assert!(!out.init_d_controller);
    assert!(!out.set_ne_max_speed_accel);
    assert!(!out.set_ne_correction_speed_accel);
    assert!(!out.set_d_max_speed_accel);
    assert!(!out.set_d_correction_speed_accel);
    assert!(out.speed_ne_ms.is_none());
    assert!(out.target_pos_ne_m.is_none());
    assert_eq!(out.att_bf_feedforward, Some(true));
    assert_eq!(out.waveform_time.unwrap().to_bits(), 0.0f32.to_bits());
    assert_eq!(
        out.time_const_freq.unwrap().to_bits(),
        (2.0f32 / 0.5).to_bits()
    );
    assert_eq!(out.state, Some(SystemIdState::Testing));
    assert_eq!(out.log_subsample, Some(0));
    assert!(out.chirp_init);
    assert_eq!(out.chirp_time_record.unwrap().to_bits(), 70.0f32.to_bits());
    assert_eq!(
        out.chirp_frequency_start.unwrap().to_bits(),
        0.5f32.to_bits()
    );
    assert_eq!(
        out.chirp_frequency_stop.unwrap().to_bits(),
        40.0f32.to_bits()
    );
    assert_eq!(out.chirp_time_fade_in.unwrap().to_bits(), 15.0f32.to_bits());
    assert_eq!(out.chirp_time_fade_out.unwrap().to_bits(), 2.0f32.to_bits());
}

#[test]
fn poscontrol_axis_requires_loiter() {
    let mut view = SystemIdInitView::typical_poscontrol();
    view.from_mode_number = 0;
    let out = systemid_init(true, &view);
    assert!(!out.ok);
    assert_eq!(out.fail, Some(SystemIdInitFail::NeedsLoiter));
    assert!(!out.set_ne_max_speed_accel);
    assert!(!out.set_d_max_speed_accel);
    assert!(!out.chirp_init);
    assert!(out.target_pos_ne_m.is_none());
}

#[test]
fn poscontrol_from_loiter_seats_ne_and_d_only_when_inactive() {
    let mut view = SystemIdInitView::typical_poscontrol();
    view.ne_is_active = false;
    view.d_is_active = false;
    let cold = systemid_init(false, &view);
    assert!(cold.ok);
    assert!(cold.init_ne_controller);
    assert!(cold.init_d_controller);
    assert!(cold.set_ne_max_speed_accel);
    assert!(cold.set_ne_correction_speed_accel);
    assert!(cold.set_d_max_speed_accel);
    assert!(cold.set_d_correction_speed_accel);
    assert_eq!(cold.speed_ne_ms.unwrap().to_bits(), 5.0f32.to_bits());
    assert_eq!(cold.accel_ne_mss.unwrap().to_bits(), 1.0f32.to_bits());
    assert_eq!(cold.speed_dn_ms.unwrap().to_bits(), 1.5f32.to_bits());
    assert_eq!(cold.speed_up_ms.unwrap().to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.accel_d_mss.unwrap().to_bits(), 2.5f32.to_bits());
    assert_eq!(cold.target_pos_ne_m, Some((12.5, -3.0)));
    assert_eq!(cold.state, Some(SystemIdState::Testing));
    assert!(cold.chirp_init);

    let mut hot = view;
    hot.ne_is_active = true;
    hot.d_is_active = true;
    let hot_out = systemid_init(true, &hot);
    assert!(hot_out.ok);
    assert!(!hot_out.init_ne_controller);
    assert!(!hot_out.init_d_controller);
    assert!(hot_out.set_ne_max_speed_accel);
    assert!(hot_out.set_d_max_speed_accel);
    assert_eq!(hot_out.target_pos_ne_m, Some((12.5, -3.0)));
}

#[test]
fn every_poscontrol_axis_accepts_loiter() {
    for axis in 14..=19 {
        let mut view = SystemIdInitView::typical_poscontrol();
        view.axis = axis;
        let out = systemid_init(false, &view);
        assert!(out.ok, "axis {axis}");
        assert!(out.set_ne_max_speed_accel);
        assert!(out.chirp_init);
    }
}

#[test]
fn ignore_checks_cannot_bypass_any_gate() {
    let mut view = SystemIdInitView::typical();
    view.axis = 0;
    let refused = systemid_init(false, &view);
    let ignored = systemid_init(true, &view);
    assert_eq!(refused, ignored);
    assert!(!ignored.ok);

    let mut landed = SystemIdInitView::typical();
    landed.land_complete = true;
    assert_eq!(systemid_init(false, &landed), systemid_init(true, &landed));

    let mut loiter_needed = SystemIdInitView::typical_poscontrol();
    loiter_needed.from_mode_number = 2;
    assert_eq!(
        systemid_init(false, &loiter_needed),
        systemid_init(true, &loiter_needed)
    );
}
