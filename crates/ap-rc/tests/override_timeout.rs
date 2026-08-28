//! GCS RC override timeout, upstream `RC_Channel::has_override`.
//!
//! `RC_OVERRIDE_TIME` seconds after the last `RC_CHANNELS_OVERRIDE` the
//! stored PWM expires and receiver input resumes.

use ap_rc::{
    apply_gcs_override_field, override_timeout_from_param, OverrideTimeout, RcOverride,
    RC_OVERRIDE_IGNORE, RC_OVERRIDE_RELEASE, RC_OVERRIDE_TIME_DEFAULT, RC_OVERRIDE_TIME_MAX,
};

#[test]
fn default_override_time_is_three_seconds() {
    assert!((RC_OVERRIDE_TIME_DEFAULT - 3.0).abs() < 1e-6);
    assert!((RC_OVERRIDE_TIME_MAX - 120.0).abs() < 1e-6);
    assert_eq!(
        override_timeout_from_param(RC_OVERRIDE_TIME_DEFAULT),
        OverrideTimeout::ExpireAfter(3000)
    );
}

#[test]
fn gcs_override_replaces_radio_in_until_timeout() {
    let mut ov = RcOverride::default();
    ov.set_override(1650, 2_000, 2_000, true);
    assert!(ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 2_000));
    assert_eq!(
        ov.read_input(1500, RC_OVERRIDE_TIME_DEFAULT, 2_000, false),
        1650
    );
    assert!(ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 4_999));
    assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 5_000));
    assert_eq!(
        ov.read_input(1500, RC_OVERRIDE_TIME_DEFAULT, 5_000, false),
        1500
    );
}

#[test]
fn zero_override_time_disables_gcs_overrides() {
    let mut ov = RcOverride::default();
    ov.set_override(1800, 100, 100, true);
    assert_eq!(override_timeout_from_param(0.0), OverrideTimeout::Disabled);
    assert!(!ov.has_override(0.0, 100));
    assert_eq!(ov.read_input(1400, 0.0, 100, false), 1400);
}

#[test]
fn negative_override_time_never_times_out() {
    let mut ov = RcOverride::default();
    ov.set_override(1720, 0, 250, true);
    assert_eq!(ov.last_override_time, 250);
    assert_eq!(override_timeout_from_param(-1.0), OverrideTimeout::Never);
    assert!(ov.has_override(-1.0, 250 + 120_000));
    assert_eq!(ov.read_input(1500, -1.0, 250 + 120_000, false), 1720);
}

#[test]
fn disabled_or_cleared_override_leaves_receiver() {
    let mut ov = RcOverride::default();
    ov.set_override(1600, 10, 10, false);
    assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 10));
    ov.set_override(1600, 10, 10, true);
    ov.clear_override();
    assert_eq!(
        ov.read_input(1510, RC_OVERRIDE_TIME_DEFAULT, 10, false),
        1510
    );
}

#[test]
fn mavlink_ignore_and_release_fields() {
    let mut ov = RcOverride::default();
    assert!(!apply_gcs_override_field(
        &mut ov,
        2,
        RC_OVERRIDE_IGNORE,
        40,
        true
    ));
    assert!(apply_gcs_override_field(&mut ov, 2, 1610, 40, true));
    assert_eq!(ov.override_value, 1610);
    assert!(apply_gcs_override_field(
        &mut ov,
        9,
        RC_OVERRIDE_RELEASE,
        50,
        true
    ));
    assert_eq!(ov.override_value, 0);
    assert!(!ov.has_override(RC_OVERRIDE_TIME_DEFAULT, 50));
}
