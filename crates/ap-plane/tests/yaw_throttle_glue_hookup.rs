use ap_plane::mode_run::{PilotThrottleSource, StickMixing};
use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
use ap_plane::stabilize_hookup::AP_PLANE_TRIM_THROTTLE_DEFAULT;
use ap_plane::yaw_throttle_glue_hookup::{
    apply_battery_compensation, apply_throttle_limits, map_pilot_throttle,
    pilot_throttle_glue_tick, vtol_yaw_stick_glue_tick, PilotThrottleGlueInputs,
    VtolYawStickGlueInputs,
};

fn throttle_cfg() -> RcChannelConfig {
    RcChannelConfig {
        radio_min: 1000,
        radio_max: 2000,
        ..RcChannelConfig::default()
    }
}

#[test]
fn direct_pilot_throttle_maps_percent() {
    let out = map_pilot_throttle(
        2000,
        &throttle_cfg(),
        PilotThrottleSource::Direct,
        AP_PLANE_TRIM_THROTTLE_DEFAULT,
    );
    assert!((out - 100.0).abs() < 0.1);
}

#[test]
fn trim_adjusted_maps_center_to_trim() {
    let out = map_pilot_throttle(
        1500,
        &throttle_cfg(),
        PilotThrottleSource::TrimAdjusted,
        AP_PLANE_TRIM_THROTTLE_DEFAULT,
    );
    assert!((out - AP_PLANE_TRIM_THROTTLE_DEFAULT).abs() < 0.1);
}

#[test]
fn throttle_limits_clamp_when_enabled() {
    assert_eq!(apply_throttle_limits(120.0, true, 0.0, 100.0), 100.0);
    assert_eq!(apply_throttle_limits(120.0, false, 0.0, 100.0), 120.0);
}

#[test]
fn battery_compensation_scales_up_on_sag() {
    let out = apply_battery_compensation(50.0, true, 0.8);
    assert!((out - 62.5).abs() < 0.1);
}

#[test]
fn pilot_throttle_glue_tick_applies_limits_and_comp() {
    let out = pilot_throttle_glue_tick(&PilotThrottleGlueInputs {
        throttle_pwm: Some(2000),
        throttle_cfg: throttle_cfg(),
        pilot_throttle_source: PilotThrottleSource::Direct,
        use_throttle_limits: true,
        use_battery_compensation: true,
        battery_voltage_ratio: 0.5,
        ..PilotThrottleGlueInputs::default()
    });
    assert_eq!(out, 100.0);
}

#[test]
fn vtol_yaw_stick_mixes_into_rudder() {
    let out = vtol_yaw_stick_glue_tick(
        0.0,
        &VtolYawStickGlueInputs {
            stick_mixing: Some(StickMixing::VtolYaw),
            yaw_norm_dz: 0.5,
            rudder_limit_scaled: 1000.0,
        },
    );
    assert!((out - 500.0).abs() < 0.1);
}

#[test]
fn vtol_yaw_stick_skipped_for_fbw_mixing() {
    let out = vtol_yaw_stick_glue_tick(
        100.0,
        &VtolYawStickGlueInputs {
            stick_mixing: Some(StickMixing::Fbw),
            yaw_norm_dz: 1.0,
            ..VtolYawStickGlueInputs::default()
        },
    );
    assert_eq!(out, 100.0);
}
