//! Simple / SuperSimple leftover, upstream `ArduCopter/Copter.cpp`.

use ap_copter::simple::{
    init_simple_bearing, update_simple_mode, update_super_simple_bearing, InitSimpleBearingInputs,
    SimpleMode, UpdateSimpleModeInputs, UpdateSuperSimpleBearingInputs, SUPER_SIMPLE_RADIUS_M,
};
use ap_copter::vehicle_loop::{DEFAULT_LOG_BITMASK, MASK_LOG_ANY, MASK_LOG_MOTBATT};
use ap_math::scalar::{radians, wrap_2pi};

fn north_simple(mode: SimpleMode, roll: i16, pitch: i16) -> UpdateSimpleModeInputs {
    UpdateSimpleModeInputs {
        simple_mode: mode,
        new_radio_frame: true,
        has_valid_input: true,
        roll_control_in: roll,
        pitch_control_in: pitch,
        simple_cos_yaw: 1.0,
        simple_sin_yaw: 0.0,
        super_simple_cos_yaw: 1.0,
        super_simple_sin_yaw: 0.0,
        ahrs_cos_yaw: 1.0,
        ahrs_sin_yaw: 0.0,
    }
}

fn echo_super(
    force: bool,
    mode: SimpleMode,
    home_distance_m: f32,
    home_bearing_rad: f32,
    last: f32,
) -> UpdateSuperSimpleBearingInputs {
    UpdateSuperSimpleBearingInputs {
        force_update: force,
        simple_mode: mode,
        home_distance_m,
        home_bearing_rad,
        super_simple_last_bearing_rad: last,
        super_simple_cos_yaw: 1.0,
        super_simple_sin_yaw: 0.0,
    }
}

#[test]
fn init_simple_bearing_captures_ahrs_and_seeds_super_simple_180_opposite() {
    let leftover = init_simple_bearing(InitSimpleBearingInputs {
        ahrs_cos_yaw: 1.0,
        ahrs_sin_yaw: 0.0,
        ahrs_yaw_rad: 0.0,
        ahrs_yaw_sensor: 0,
        log_bitmask: DEFAULT_LOG_BITMASK,
    });
    assert!((leftover.simple_cos_yaw - 1.0).abs() < 1e-6);
    assert!(leftover.simple_sin_yaw.abs() < 1e-6);
    assert!((leftover.super_simple_last_bearing_rad - wrap_2pi(radians(180.0))).abs() < 1e-5);
    // SuperSimple cos/sin copy the simple heading, not cos/sin of yaw+180.
    assert!((leftover.super_simple_cos_yaw - 1.0).abs() < 1e-6);
    assert!(leftover.super_simple_sin_yaw.abs() < 1e-6);
    assert!(leftover.log_init_simple_bearing);
    assert_eq!(leftover.logged_yaw_sensor, 0);
}

#[test]
fn init_simple_bearing_logs_only_for_mask_log_any() {
    let logged = init_simple_bearing(InitSimpleBearingInputs {
        ahrs_cos_yaw: 0.0,
        ahrs_sin_yaw: 1.0,
        ahrs_yaw_rad: radians(90.0),
        ahrs_yaw_sensor: 9_000,
        log_bitmask: MASK_LOG_ANY,
    });
    assert!(logged.log_init_simple_bearing);
    assert_eq!(logged.logged_yaw_sensor, 9_000);
    assert!(logged.simple_cos_yaw.abs() < 1e-6);
    assert!((logged.simple_sin_yaw - 1.0).abs() < 1e-6);
    assert!((logged.super_simple_last_bearing_rad - wrap_2pi(radians(270.0))).abs() < 1e-5);

    let motbatt_only = init_simple_bearing(InitSimpleBearingInputs {
        ahrs_cos_yaw: 1.0,
        ahrs_sin_yaw: 0.0,
        ahrs_yaw_rad: 0.0,
        ahrs_yaw_sensor: 0,
        log_bitmask: MASK_LOG_MOTBATT,
    });
    assert!(!motbatt_only.log_init_simple_bearing);
}

#[test]
fn update_simple_mode_refuses_none_or_stale_radio_without_consuming() {
    let none = update_simple_mode(UpdateSimpleModeInputs {
        simple_mode: SimpleMode::None,
        ..north_simple(SimpleMode::None, 1_000, -500)
    });
    assert!(!none.consumed_radio_frame);
    assert!(!none.rotated);
    assert_eq!(none.roll_control_in, 1_000);
    assert_eq!(none.pitch_control_in, -500);

    let stale = update_simple_mode(UpdateSimpleModeInputs {
        new_radio_frame: false,
        ..north_simple(SimpleMode::Simple, 1_000, -500)
    });
    assert!(!stale.consumed_radio_frame);
    assert!(!stale.rotated);
}

#[test]
fn update_simple_mode_consumes_the_frame_before_the_valid_input_refuse() {
    let leftover = update_simple_mode(UpdateSimpleModeInputs {
        has_valid_input: false,
        ..north_simple(SimpleMode::Simple, 1_000, -500)
    });
    assert!(leftover.consumed_radio_frame);
    assert!(!leftover.rotated);
    assert_eq!(leftover.roll_control_in, 1_000);
    assert_eq!(leftover.pitch_control_in, -500);
}

#[test]
fn update_simple_mode_is_identity_when_simple_heading_matches_ahrs() {
    let leftover = update_simple_mode(north_simple(SimpleMode::Simple, 1_500, -800));
    assert!(leftover.consumed_radio_frame);
    assert!(leftover.rotated);
    assert_eq!(leftover.roll_control_in, 1_500);
    assert_eq!(leftover.pitch_control_in, -800);
}

#[test]
fn update_simple_mode_rotates_forward_stick_into_roll_when_vehicle_faces_east() {
    // Simple heading still north; vehicle yaw 90° (cos=0, sin=1).
    // Forward (negative pitch) becomes left roll.
    let leftover = update_simple_mode(UpdateSimpleModeInputs {
        ahrs_cos_yaw: 0.0,
        ahrs_sin_yaw: 1.0,
        ..north_simple(SimpleMode::Simple, 0, -1_000)
    });
    assert!(leftover.rotated);
    assert_eq!(leftover.roll_control_in, -1_000);
    assert_eq!(leftover.pitch_control_in, 0);
}

#[test]
fn update_simple_mode_supersimple_uses_the_home_relative_heading() {
    // SuperSimple heading is east (cos=0, sin=1); AHRS still north.
    // A right-roll stick becomes a forward pitch in the vehicle frame.
    let leftover = update_simple_mode(UpdateSimpleModeInputs {
        super_simple_cos_yaw: 0.0,
        super_simple_sin_yaw: 1.0,
        ..north_simple(SimpleMode::SuperSimple, 1_000, 0)
    });
    assert!(leftover.rotated);
    assert_eq!(leftover.roll_control_in, 0);
    assert_eq!(leftover.pitch_control_in, 1_000);
}

#[test]
fn update_super_simple_bearing_refuses_when_not_supersimple_unless_forced() {
    let refused = update_super_simple_bearing(echo_super(
        false,
        SimpleMode::Simple,
        50.0,
        radians(90.0),
        0.0,
    ));
    assert!(!refused.updated);
    assert!((refused.super_simple_last_bearing_rad).abs() < 1e-6);
    assert!((refused.super_simple_cos_yaw - 1.0).abs() < 1e-6);

    let forced =
        update_super_simple_bearing(echo_super(true, SimpleMode::None, 1.0, radians(90.0), 0.0));
    assert!(forced.updated);
    assert!((forced.super_simple_last_bearing_rad - radians(90.0)).abs() < 1e-5);
    let angle: f32 = radians(90.0) + radians(180.0);
    assert!((forced.super_simple_cos_yaw - angle.cos()).abs() < 1e-5);
    assert!((forced.super_simple_sin_yaw - angle.sin()).abs() < 1e-5);
}

#[test]
fn update_super_simple_bearing_refuses_inside_the_home_radius() {
    let leftover = update_super_simple_bearing(echo_super(
        false,
        SimpleMode::SuperSimple,
        SUPER_SIMPLE_RADIUS_M - 0.1,
        radians(90.0),
        0.0,
    ));
    assert!(!leftover.updated);
}

#[test]
fn update_super_simple_bearing_refuses_a_sub_five_degree_change() {
    let leftover = update_super_simple_bearing(echo_super(
        true,
        SimpleMode::SuperSimple,
        50.0,
        radians(4.0),
        0.0,
    ));
    assert!(!leftover.updated);
}

#[test]
fn update_super_simple_bearing_rewrites_after_five_degrees() {
    let leftover = update_super_simple_bearing(echo_super(
        true,
        SimpleMode::SuperSimple,
        50.0,
        radians(6.0),
        0.0,
    ));
    assert!(leftover.updated);
    assert!((leftover.super_simple_last_bearing_rad - radians(6.0)).abs() < 1e-5);
    let angle: f32 = radians(6.0) + radians(180.0);
    assert!((leftover.super_simple_cos_yaw - angle.cos()).abs() < 1e-5);
    assert!((leftover.super_simple_sin_yaw - angle.sin()).abs() < 1e-5);
}

#[test]
fn update_super_simple_bearing_wraps_the_deadband_across_pi() {
    let leftover = update_super_simple_bearing(echo_super(
        true,
        SimpleMode::SuperSimple,
        50.0,
        -radians(179.0),
        radians(179.0),
    ));
    assert!(!leftover.updated);
}
