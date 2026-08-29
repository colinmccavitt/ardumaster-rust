//! AutoYaw `set_mode` / `get_heading` leftover, upstream `ArduCopter/autoyaw.cpp`.

use ap_copter::auto_yaw::{
    get_heading, set_mode, set_mode_to_default, GetHeadingContext, HeadingMode, WpYawBehaviour,
    YawAngleSource, YawMode, YawModeEntry, YawRateSource,
};

fn almost(got: f32, want: f32) {
    assert!((got - want).abs() < 1.0e-6, "got {got} want {want}");
}

#[test]
fn set_mode_is_a_no_op_when_unchanged() {
    let leftover = set_mode(YawMode::Rate, YawMode::Hold, YawMode::Rate, 1.25);
    assert!(!leftover.changed);
    assert_eq!(leftover.mode, YawMode::Rate);
    assert_eq!(
        leftover.last_mode,
        YawMode::Hold,
        "re-selecting RATE must not rewrite last_mode"
    );
    assert!(leftover.entry.is_none());
    assert!(!leftover.need_ahrs_yaw);
    assert!(leftover.look_ahead_yaw_rad.is_none());
    assert!(
        leftover.yaw_rate_rads.is_none(),
        "re-selecting RATE must not zero a commanded rate"
    );
}

#[test]
fn set_mode_look_ahead_seeds_from_ahrs() {
    let leftover = set_mode(YawMode::Hold, YawMode::Roi, YawMode::LookAhead, 0.75);
    assert!(leftover.changed);
    assert_eq!(leftover.mode, YawMode::LookAhead);
    assert_eq!(leftover.last_mode, YawMode::Hold);
    assert_eq!(
        leftover.entry,
        Some(YawModeEntry::SeedLookAheadFromCurrentYaw)
    );
    assert!(leftover.need_ahrs_yaw);
    assert_eq!(leftover.look_ahead_yaw_rad, Some(0.75));
    assert!(leftover.yaw_rate_rads.is_none());
}

#[test]
fn set_mode_rate_zeros_the_stored_rate() {
    let leftover = set_mode(YawMode::Hold, YawMode::Roi, YawMode::Rate, 0.0);
    assert!(leftover.changed);
    assert_eq!(leftover.mode, YawMode::Rate);
    assert_eq!(leftover.last_mode, YawMode::Hold);
    assert_eq!(leftover.entry, Some(YawModeEntry::ZeroYawRate));
    assert!(!leftover.need_ahrs_yaw);
    assert_eq!(leftover.yaw_rate_rads, Some(0.0));
}

#[test]
fn set_mode_other_entries_write_nothing() {
    for new_mode in [
        YawMode::Hold,
        YawMode::LookAtNextWp,
        YawMode::Roi,
        YawMode::Fixed,
        YawMode::ResetToArmedYaw,
        YawMode::AngleRate,
        YawMode::Circle,
        YawMode::PilotRate,
        YawMode::Weathervane,
    ] {
        let leftover = set_mode(YawMode::LookAhead, YawMode::Roi, new_mode, 2.0);
        assert!(leftover.changed, "{new_mode:?}");
        assert_eq!(leftover.entry, Some(YawModeEntry::Nothing), "{new_mode:?}");
        assert!(!leftover.need_ahrs_yaw, "{new_mode:?}");
        assert!(leftover.look_ahead_yaw_rad.is_none(), "{new_mode:?}");
        assert!(leftover.yaw_rate_rads.is_none(), "{new_mode:?}");
        assert_eq!(leftover.last_mode, YawMode::LookAhead, "{new_mode:?}");
    }
}

#[test]
fn set_mode_to_default_uses_wp_yaw_behavior() {
    let leftover = set_mode_to_default(
        YawMode::Weathervane,
        YawMode::Hold,
        WpYawBehaviour::LookAtNextWp,
        false,
        0.0,
    );
    assert_eq!(leftover.mode, YawMode::LookAtNextWp);
    assert_eq!(leftover.last_mode, YawMode::Weathervane);

    let rtl = set_mode_to_default(
        YawMode::Weathervane,
        YawMode::Hold,
        WpYawBehaviour::LookAtNextWpExceptRtl,
        true,
        0.0,
    );
    assert_eq!(
        rtl.mode,
        YawMode::Hold,
        "rtl true is HOLD for LOOK_AT_NEXT_WP_EXCEPT_RTL"
    );
}

#[test]
fn hold_falls_through_to_pos_control_yaw() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Hold,
        pos_control_yaw_rad: 0.4,
        pos_control_yaw_rate_rads: 0.9,
        yaw_angle_rad: 0.1,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.mode, YawMode::Hold);
    assert_eq!(leftover.heading_mode, HeadingMode::RateOnly);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::PosControl);
    assert!(leftover.yaw_angle_source.need_pos_control_yaw());
    almost(leftover.yaw_angle_rad, 0.4);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Zero);
    almost(leftover.yaw_rate_rads, 0.0);
    assert!(leftover.pilot_set_mode.is_none());
    assert!(leftover.weathervane_set_mode.is_none());
}

#[test]
fn look_at_next_wp_reads_pos_control_angle_and_rate() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::LookAtNextWp,
        pos_control_yaw_rad: 0.55,
        pos_control_yaw_rate_rads: 0.22,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.heading_mode, HeadingMode::AngleAndRate);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::PosControl);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::PositionController);
    almost(leftover.yaw_angle_rad, 0.55);
    almost(leftover.yaw_rate_rads, 0.22);
}

#[test]
fn pilot_stick_takes_the_axis() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Roi,
        has_valid_input: true,
        use_pilot_yaw: true,
        pilot_desired_yaw_rate_rads: 0.35,
        att_target_yaw_rad: 1.1,
        ..GetHeadingContext::default()
    });
    assert!(leftover.need_pilot_yaw_rate);
    assert_eq!(leftover.pilot_set_mode, Some(YawMode::PilotRate));
    assert_eq!(leftover.mode, YawMode::PilotRate);
    assert_eq!(leftover.last_mode, YawMode::Roi);
    assert_eq!(leftover.heading_mode, HeadingMode::RateOnly);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::AttitudeTarget);
    assert!(leftover.yaw_angle_source.need_att_target_yaw());
    almost(leftover.yaw_angle_rad, 1.1);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Pilot);
    almost(leftover.yaw_rate_rads, 0.35);
    almost(leftover.pilot_yaw_rate_rads, 0.35);
}

#[test]
fn rc_failsafe_from_pilot_rate_holds() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::PilotRate,
        last_mode: YawMode::Roi,
        has_valid_input: false,
        use_pilot_yaw: true,
        pilot_desired_yaw_rate_rads: 0.5,
        pos_control_yaw_rad: 0.2,
        ..GetHeadingContext::default()
    });
    assert!(!leftover.need_pilot_yaw_rate);
    assert_eq!(leftover.pilot_set_mode, Some(YawMode::Hold));
    assert_eq!(leftover.mode, YawMode::Hold);
    assert_eq!(leftover.last_mode, YawMode::PilotRate);
    almost(leftover.pilot_yaw_rate_rads, 0.0);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::PosControl);
    almost(leftover.yaw_angle_rad, 0.2);
}

#[test]
fn valid_zero_stick_leaves_pilot_rate_alone() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::PilotRate,
        last_mode: YawMode::Fixed,
        has_valid_input: true,
        use_pilot_yaw: true,
        pilot_desired_yaw_rate_rads: 0.0,
        att_target_yaw_rad: 0.8,
        ..GetHeadingContext::default()
    });
    assert!(leftover.need_pilot_yaw_rate);
    assert!(leftover.pilot_set_mode.is_none());
    assert_eq!(leftover.mode, YawMode::PilotRate);
    assert_eq!(leftover.last_mode, YawMode::Fixed);
    almost(leftover.yaw_rate_rads, 0.0);
}

#[test]
fn weathervane_engages_after_pilot() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Hold,
        has_valid_input: true,
        use_pilot_yaw: true,
        pilot_desired_yaw_rate_rads: 0.4,
        weathervane_enabled: true,
        allows_weathervaning: true,
        weathervane_wants_yaw: true,
        weathervane_rate_rads: 0.15,
        att_target_yaw_rad: 0.3,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.pilot_set_mode, Some(YawMode::PilotRate));
    assert_eq!(leftover.weathervane_set_mode, Some(YawMode::Weathervane));
    assert_eq!(leftover.mode, YawMode::Weathervane);
    assert_eq!(
        leftover.last_mode,
        YawMode::PilotRate,
        "weathervane took the axis from the pilot"
    );
    assert!(leftover.need_weathervane_yaw_out);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Unchanged);
    almost(leftover.yaw_rate_rads, 0.15);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::AttitudeTarget);
}

#[test]
fn weathervane_release_from_hold_goes_to_default() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Weathervane,
        last_mode: YawMode::Hold,
        weathervane_enabled: true,
        allows_weathervaning: false,
        wp_yaw_behavior: WpYawBehaviour::LookAtNextWp,
        pos_control_yaw_rad: 0.66,
        pos_control_yaw_rate_rads: 0.11,
        yaw_rate_rads: 0.4,
        ..GetHeadingContext::default()
    });
    assert!(!leftover.need_weathervane_yaw_out);
    assert_eq!(leftover.weathervane_set_mode, Some(YawMode::LookAtNextWp));
    assert_eq!(leftover.mode, YawMode::LookAtNextWp);
    assert_eq!(leftover.last_mode, YawMode::Weathervane);
    almost(leftover.yaw_angle_rad, 0.66);
    almost(leftover.yaw_rate_rads, 0.11);
}

#[test]
fn weathervane_release_restores_last_mode() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Weathervane,
        last_mode: YawMode::Roi,
        weathervane_enabled: true,
        allows_weathervaning: false,
        roi_ne_m: (10.0, 0.0),
        position_ne_m: Some((0.0, 0.0)),
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.weathervane_set_mode, Some(YawMode::Roi));
    assert_eq!(leftover.mode, YawMode::Roi);
    assert_eq!(leftover.last_mode, YawMode::Weathervane);
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::Roi);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Zero);
}

#[test]
fn weathervane_compiled_out_does_not_release() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Weathervane,
        last_mode: YawMode::Hold,
        weathervane_enabled: false,
        allows_weathervaning: false,
        weathervane_wants_yaw: false,
        yaw_rate_rads: 0.25,
        att_target_yaw_rad: 0.5,
        ..GetHeadingContext::default()
    });
    assert!(!leftover.need_update_weathervane);
    assert!(!leftover.need_weathervane_yaw_out);
    assert!(leftover.weathervane_set_mode.is_none());
    assert_eq!(leftover.mode, YawMode::Weathervane);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Unchanged);
    almost(leftover.yaw_rate_rads, 0.25);
}

#[test]
fn fixed_slew_consumes_offset_and_advances_millis() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Fixed,
        now_ms: 1_000,
        last_update_ms: 900,
        yaw_angle_rad: 0.0,
        fixed_yaw_offset_rad: 1.0,
        fixed_yaw_slewrate_rads: 2.0,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::FixedSlew);
    assert!(leftover.yaw_angle_source.need_millis());
    assert_eq!(leftover.last_update_ms, 1_000);
    almost(leftover.yaw_angle_rad, 0.2);
    almost(leftover.fixed_yaw_offset_rad, 0.8);
    assert_eq!(leftover.heading_mode, HeadingMode::AngleAndRate);
}

#[test]
fn angle_rate_integrates_and_leaves_rate_alone() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::AngleRate,
        now_ms: 1_000,
        last_update_ms: 900,
        yaw_angle_rad: 0.5,
        yaw_rate_rads: 1.0,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::AngleRate);
    assert!(leftover.yaw_angle_source.need_millis());
    assert_eq!(leftover.last_update_ms, 1_000);
    almost(leftover.yaw_angle_rad, 0.6);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Unchanged);
    almost(leftover.yaw_rate_rads, 1.0);
}

#[test]
fn look_ahead_uses_ground_course_when_fast_enough() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::LookAhead,
        look_ahead_yaw_rad: 0.1,
        position_ok: true,
        vel_n_ms: 0.0,
        vel_e_ms: 3.0,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::LookAhead);
    almost(leftover.yaw_angle_rad, core::f32::consts::FRAC_PI_2);
    almost(leftover.look_ahead_yaw_rad, leftover.yaw_angle_rad);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Zero);
}

#[test]
fn reset_to_armed_yaw_reads_the_bearing() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::ResetToArmedYaw,
        initial_armed_bearing_rad: 2.1,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::ArmedBearing);
    assert!(leftover.yaw_angle_source.need_armed_bearing());
    almost(leftover.yaw_angle_rad, 2.1);
}

#[test]
fn circle_active_reads_circle_nav() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Circle,
        circle_nav_enabled: true,
        circle_nav_active: true,
        circle_yaw_rad: 1.7,
        yaw_angle_rad: 0.2,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::CircleNav);
    assert!(leftover.yaw_angle_source.need_circle_yaw());
    almost(leftover.yaw_angle_rad, 1.7);
}

#[test]
fn circle_inactive_holds_the_stored_angle() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Circle,
        circle_nav_enabled: true,
        circle_nav_active: false,
        circle_yaw_rad: 1.7,
        yaw_angle_rad: 0.2,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::CircleHeld);
    assert!(!leftover.yaw_angle_source.need_circle_yaw());
    almost(leftover.yaw_angle_rad, 0.2);
}

#[test]
fn circle_compiled_out_holds_the_stored_angle() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Circle,
        circle_nav_enabled: false,
        circle_nav_active: true,
        circle_yaw_rad: 1.7,
        yaw_angle_rad: 0.2,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::CircleHeld);
    almost(leftover.yaw_angle_rad, 0.2);
}

#[test]
fn rate_mode_reads_att_target_and_keeps_the_rate() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Rate,
        yaw_rate_rads: -0.55,
        att_target_yaw_rad: 0.9,
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::AttitudeTarget);
    almost(leftover.yaw_angle_rad, 0.9);
    assert_eq!(leftover.yaw_rate_source, YawRateSource::Unchanged);
    almost(leftover.yaw_rate_rads, -0.55);
}

#[test]
fn roi_without_position_holds_the_attitude_target() {
    let leftover = get_heading(&GetHeadingContext {
        mode: YawMode::Roi,
        position_ne_m: None,
        att_target_yaw_rad: 0.33,
        roi_ne_m: (100.0, 0.0),
        ..GetHeadingContext::default()
    });
    assert_eq!(leftover.yaw_angle_source, YawAngleSource::Roi);
    almost(leftover.yaw_angle_rad, 0.33);
}

#[test]
fn heading_mode_never_produces_angle_only() {
    for mode in [
        YawMode::Hold,
        YawMode::LookAtNextWp,
        YawMode::Roi,
        YawMode::Fixed,
        YawMode::LookAhead,
        YawMode::ResetToArmedYaw,
        YawMode::AngleRate,
        YawMode::Rate,
        YawMode::Circle,
        YawMode::PilotRate,
        YawMode::Weathervane,
    ] {
        let leftover = get_heading(&GetHeadingContext {
            mode,
            ..GetHeadingContext::default()
        });
        assert_ne!(
            leftover.heading_mode,
            HeadingMode::AngleOnly,
            "{mode:?} must not produce Angle_Only"
        );
    }
}
