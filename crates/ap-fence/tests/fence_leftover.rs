//! AC_Fence type bits, enable leftover, circle / alt-max / alt-min checks,
//! and `check()` orchestration.
//!
//! Tracked as **COP-025**. Polygon EEPROM / `AC_PolyFence_loader` is not
//! in this slice.

use ap_fence::{
    Action, AutoEnable, CheckAltMaxContext, CheckAltMinContext, CheckCircleContext, CheckContext,
    Fence, MinAltState, ALT_MAX_BACKUP_DISTANCE_M, ALT_MAX_DEFAULT_M, ALT_MIN_BACKUP_DISTANCE_M,
    ALT_MIN_DEFAULT_M, ARMING_FENCES, CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M,
    CIRCLE_RADIUS_DEFAULT_M, FENCE_TYPE_DEFAULT_COPTER, FENCE_TYPE_DEFAULT_PLANE,
    FENCE_TYPE_DEFAULT_ROVER, MANUAL_RECOVERY_TIME_MIN_MS, MARGIN_DEFAULT_M, TYPE_ALL,
    TYPE_ALT_MAX, TYPE_ALT_MIN, TYPE_CIRCLE, TYPE_POLYGON,
};
use ap_math::location::AltFrame;
use ap_math::scalar::is_equal;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn enable_circle_and_alt(fence: &mut Fence) {
    fence.set_configured_fences(TYPE_ALT_MAX | TYPE_CIRCLE);
    let leftover = fence.enable(true, TYPE_ALT_MAX | TYPE_CIRCLE, true);
    assert_eq!(leftover.changed_mask, TYPE_ALT_MAX | TYPE_CIRCLE);
}

fn enable_alt_min(fence: &mut Fence) {
    fence.set_configured_fences(TYPE_ALT_MIN);
    fence.set_alt_min_m(20.0);
    let leftover = fence.enable(true, TYPE_ALT_MIN, true);
    assert_eq!(leftover.changed_mask, TYPE_ALT_MIN);
}

#[test]
fn type_bits_match_upstream() {
    assert_eq!(TYPE_ALT_MAX, 1);
    assert_eq!(TYPE_CIRCLE, 2);
    assert_eq!(TYPE_POLYGON, 4);
    assert_eq!(TYPE_ALT_MIN, 8);
    assert_eq!(ARMING_FENCES, TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON);
    assert_eq!(TYPE_ALL, ARMING_FENCES | TYPE_ALT_MIN);
    assert_eq!(
        FENCE_TYPE_DEFAULT_COPTER,
        TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON
    );
    assert_eq!(FENCE_TYPE_DEFAULT_PLANE, TYPE_POLYGON);
    assert_eq!(FENCE_TYPE_DEFAULT_ROVER, TYPE_CIRCLE | TYPE_POLYGON);
    assert_eq!(Action::from_param(0), Some(Action::ReportOnly));
    assert_eq!(Action::from_param(1), Some(Action::RtlAndLand));
    assert_eq!(Action::from_param(2), Some(Action::AlwaysLand));
    assert_eq!(Action::from_param(6), Some(Action::Guided));
    assert_eq!(Action::from_param(8), Some(Action::AutolandOrRtl));
    assert_eq!(Action::from_param(9), None);
    assert_eq!(Action::default_param(), Action::RtlAndLand);
}

#[test]
fn constructor_strips_alt_min_only_when_enable_param_is_set() {
    let off = Fence::new();
    assert!(!off.enabled());
    assert_eq!(off.configured_fences(), FENCE_TYPE_DEFAULT_COPTER);
    assert_eq!(off.get_enabled_fences(), 0);
    assert_eq!(off.present(), TYPE_ALT_MAX | TYPE_CIRCLE);
    assert_eq!(off.poly_fence_count(), 0);

    let on = Fence::from_params(true, FENCE_TYPE_DEFAULT_COPTER | TYPE_ALT_MIN);
    assert!(on.enabled());
    assert_eq!(on.enabled_fences_raw(), FENCE_TYPE_DEFAULT_COPTER);
    assert_eq!(on.get_enabled_fences(), TYPE_ALT_MAX | TYPE_CIRCLE);
}

#[test]
fn enable_returns_changed_mask_and_logs_per_type() {
    let mut fence = Fence::new();
    let first = fence.enable_configured(true);
    assert_eq!(first.changed_mask, FENCE_TYPE_DEFAULT_COPTER);
    assert_eq!(first.enabled_fences, FENCE_TYPE_DEFAULT_COPTER);
    assert_eq!(first.log_enable, Some(true));
    assert_eq!(first.log_alt_max, Some(true));
    assert_eq!(first.log_circle, Some(true));
    assert_eq!(first.log_polygon, Some(true));
    assert_eq!(first.log_alt_min, None);
    assert_eq!(first.clear_breach_mask, 0);
    assert!(!first.reset_manual_recovery);
    // Polygon is configured but not present until the loader leftover.
    assert_eq!(fence.get_enabled_fences(), TYPE_ALT_MAX | TYPE_CIRCLE);

    let again = fence.enable_configured(true);
    assert_eq!(again.changed_mask, 0);
    assert_eq!(again.log_enable, None);
}

#[test]
fn enable_min_alt_state_writes_before_the_no_change_return() {
    let mut fence = Fence::from_params(false, TYPE_ALL);
    let first = fence.enable(true, TYPE_ALT_MIN, true);
    assert_eq!(first.changed_mask, TYPE_ALT_MIN);
    assert_eq!(first.min_alt_state, MinAltState::ManuallyEnabled);
    assert_eq!(first.log_alt_min, Some(true));

    let again = fence.enable(true, TYPE_ALT_MIN, true);
    assert_eq!(again.changed_mask, 0);
    assert_eq!(again.min_alt_state, MinAltState::ManuallyEnabled);
    assert_eq!(fence.min_alt_state(), MinAltState::ManuallyEnabled);

    let off = fence.enable(false, TYPE_ALT_MIN, true);
    assert_eq!(off.changed_mask, TYPE_ALT_MIN);
    assert_eq!(off.min_alt_state, MinAltState::ManuallyDisabled);
    assert!(off.reset_manual_recovery);
}

#[test]
fn disable_clears_breach_and_resets_manual_recovery() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let hit = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((400.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(hit.newly_breached);
    assert_eq!(fence.get_breaches() & TYPE_CIRCLE, TYPE_CIRCLE);
    fence.set_manual_recovery_start_ms(500);

    let off = fence.enable(false, TYPE_CIRCLE, true);
    assert_eq!(off.changed_mask, TYPE_CIRCLE);
    assert_eq!(off.clear_breach_mask, TYPE_CIRCLE);
    assert!(off.reset_manual_recovery);
    assert_eq!(off.log_enable, Some(false));
    assert_eq!(off.log_circle, Some(false));
    assert_eq!(fence.get_breaches() & TYPE_CIRCLE, 0);
    assert_eq!(fence.manual_recovery_start_ms(), 0);
}

#[test]
fn unconfigured_types_do_not_enable() {
    let mut fence = Fence::from_params(false, TYPE_CIRCLE);
    let leftover = fence.enable(true, TYPE_ALL, true);
    assert_eq!(leftover.changed_mask, TYPE_CIRCLE);
    assert_eq!(fence.get_enabled_fences(), TYPE_CIRCLE);
}

#[test]
fn circle_inside_is_not_a_breach() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((10.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(leftover.enabled);
    assert!(leftover.need_ne_home);
    assert!(!leftover.newly_breached);
    assert!(!leftover.recorded_breach);
    assert!(!leftover.margin_breached);
    almost(leftover.home_distance_m, 10.0);
    almost(leftover.breach_distance_m, 10.0 - CIRCLE_RADIUS_DEFAULT_M);
    almost(leftover.breach_direction_ne_m.x, 290.0);
    almost(leftover.breach_direction_ne_m.y, 0.0);
}

#[test]
fn circle_outside_records_a_new_breach_and_backup() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((400.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    assert!(leftover.recorded_breach);
    assert!(leftover.need_gcs_fence_status);
    almost(leftover.home_distance_m, 400.0);
    almost(leftover.breach_distance_m, 100.0);
    almost(
        leftover.backup_radius_m,
        400.0 + CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M,
    );
    almost(leftover.breach_direction_ne_m.x, -100.0);
    assert_eq!(fence.get_breaches(), TYPE_CIRCLE);
    assert_eq!(fence.get_breach_count(), 1);
    assert_eq!(fence.get_breach_time(), 1_001);
}

#[test]
fn circle_already_breached_does_not_re_fire_until_backup() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let first = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((400.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(first.newly_breached);
    almost(first.backup_radius_m, 420.0);

    let still = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((410.0, 0.0)),
        now_ms: 2_000,
    });
    assert!(!still.newly_breached);
    assert!(!still.recorded_breach);
    assert_eq!(fence.get_breach_count(), 1);

    let backup = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((420.0, 0.0)),
        now_ms: 3_000,
    });
    assert!(backup.newly_breached);
    assert!(backup.recorded_breach);
    assert!(!backup.need_gcs_fence_status);
    almost(backup.backup_radius_m, 440.0);
    assert_eq!(fence.get_breach_count(), 2);
}

#[test]
fn circle_return_inside_clears_breach_and_backup() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    assert!(
        fence
            .check_fence_circle(CheckCircleContext {
                ne_home_m: Some((400.0, 0.0)),
                now_ms: 1_001,
            })
            .newly_breached
    );

    let back = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((50.0, 0.0)),
        now_ms: 2_000,
    });
    assert!(!back.newly_breached);
    assert!(back.cleared_breach);
    assert!(!back.margin_breached);
    almost(back.backup_radius_m, 0.0);
    assert_eq!(fence.get_breaches() & TYPE_CIRCLE, 0);
}

#[test]
fn circle_margin_is_inside_but_within_margin_ne() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    // Default radius 300, margin 2. 299 is inside the fence and inside the margin.
    let leftover = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((299.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(!leftover.newly_breached);
    assert!(leftover.margin_breached);
    assert_eq!(fence.get_margin_breaches() & TYPE_CIRCLE, TYPE_CIRCLE);
}

#[test]
fn circle_disabled_does_not_clear_or_record() {
    let mut fence = Fence::new();
    let leftover = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((400.0, 0.0)),
        now_ms: 1_001,
    });
    assert!(!leftover.enabled);
    assert!(!leftover.newly_breached);
    assert!(!leftover.need_ne_home);
    assert_eq!(fence.get_breaches(), 0);
}

#[test]
fn circle_missing_position_keeps_stale_home_distance() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    assert!(
        fence
            .check_fence_circle(CheckCircleContext {
                ne_home_m: Some((400.0, 0.0)),
                now_ms: 1_001,
            })
            .newly_breached
    );

    let stale = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: None,
        now_ms: 2_000,
    });
    assert!(!stale.need_ne_home);
    assert!(!stale.newly_breached);
    almost(stale.home_distance_m, 400.0);
}

#[test]
fn circle_at_home_points_east_along_the_radius() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_circle(CheckCircleContext {
        ne_home_m: Some((0.0, 0.0)),
        now_ms: 1_001,
    });
    almost(leftover.breach_direction_ne_m.x, CIRCLE_RADIUS_DEFAULT_M);
    almost(leftover.breach_direction_ne_m.y, 0.0);
}

#[test]
fn alt_max_below_ceiling_is_not_a_breach() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.enabled);
    assert!(leftover.need_alt_in_frame);
    assert!(!leftover.need_home_alt);
    assert!(!leftover.newly_breached);
    almost(leftover.breach_distance_m, 50.0 - ALT_MAX_DEFAULT_M);
    almost(leftover.safe_relhome_alt_max_m, 100.0 - MARGIN_DEFAULT_M);
}

#[test]
fn alt_max_above_ceiling_records_a_new_breach_and_backup() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: Some(120.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    assert!(leftover.recorded_breach);
    assert!(leftover.need_gcs_fence_status);
    almost(leftover.breach_distance_m, 20.0);
    almost(leftover.backup_alt_m, 120.0 + ALT_MAX_BACKUP_DISTANCE_M);
    assert_eq!(fence.get_breaches(), TYPE_ALT_MAX);
}

#[test]
fn alt_max_unavailable_is_a_fresh_breach_without_record() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: None,
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    assert!(leftover.alt_unavailable);
    assert!(!leftover.recorded_breach);
    assert!(!leftover.need_gcs_fence_status);
    assert_eq!(fence.get_breaches(), 0);
    assert_eq!(fence.get_breach_count(), 0);
}

#[test]
fn alt_max_absolute_frame_is_a_home_alt_leftover() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    fence.set_alt_max_type(AltFrame::Absolute);
    let leftover = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 10.0,
        now_ms: 1_001,
    });
    assert!(leftover.need_home_alt);
    almost(
        leftover.safe_relhome_alt_max_m,
        ALT_MAX_DEFAULT_M - 10.0 - MARGIN_DEFAULT_M,
    );
}

#[test]
fn alt_max_margin_and_clear_on_descent() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    assert!(
        fence
            .check_fence_alt_max(CheckAltMaxContext {
                alt_u_m: Some(120.0),
                home_alt_amsl_m: 0.0,
                now_ms: 1_001,
            })
            .newly_breached
    );

    let margin = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: Some(99.0),
        home_alt_amsl_m: 0.0,
        now_ms: 2_000,
    });
    assert!(!margin.newly_breached);
    assert!(margin.cleared_breach);
    assert!(margin.margin_breached);
    almost(margin.backup_alt_m, 0.0);
    assert_eq!(fence.get_breaches() & TYPE_ALT_MAX, 0);
    assert_eq!(fence.get_margin_breaches() & TYPE_ALT_MAX, TYPE_ALT_MAX);

    let clear = fence.check_fence_alt_max(CheckAltMaxContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 0.0,
        now_ms: 3_000,
    });
    assert!(!clear.margin_breached);
    assert_eq!(fence.get_margin_breaches() & TYPE_ALT_MAX, 0);
}

#[test]
fn get_margin_ne_uses_xy_only_when_positive() {
    let mut fence = Fence::new();
    almost(fence.get_margin_ne_m(), MARGIN_DEFAULT_M);
    fence.set_margin_ne_m(5.0);
    almost(fence.get_margin_ne_m(), 5.0);
    fence.set_margin_ne_m(0.0);
    almost(fence.get_margin_ne_m(), MARGIN_DEFAULT_M);
}

#[test]
fn alt_min_above_floor_is_not_a_breach() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    let leftover = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.enabled);
    assert!(leftover.need_alt_in_frame);
    assert!(!leftover.need_home_alt);
    assert!(!leftover.newly_breached);
    almost(leftover.breach_distance_m, 20.0 - 50.0);
    almost(leftover.safe_relhome_alt_min_m, 20.0 - MARGIN_DEFAULT_M);
    almost(fence.get_safe_alt_min_m(), 20.0 + MARGIN_DEFAULT_M);
}

#[test]
fn alt_min_below_floor_records_a_new_breach_and_backup() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    let leftover = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(10.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    assert!(leftover.recorded_breach);
    assert!(leftover.need_gcs_fence_status);
    almost(leftover.breach_distance_m, 10.0);
    almost(leftover.backup_alt_m, 10.0 - ALT_MIN_BACKUP_DISTANCE_M);
    assert_eq!(fence.get_breaches(), TYPE_ALT_MIN);
    assert_eq!(fence.get_breach_count(), 1);
    assert_eq!(fence.get_breach_time(), 1_001);
}

#[test]
fn alt_min_unavailable_is_a_fresh_breach_without_record() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    let leftover = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: None,
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    assert!(leftover.alt_unavailable);
    assert!(!leftover.recorded_breach);
    assert!(!leftover.need_gcs_fence_status);
    assert_eq!(fence.get_breaches(), 0);
    assert_eq!(fence.get_breach_count(), 0);
}

#[test]
fn alt_min_absolute_frame_is_a_home_alt_leftover() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    fence.set_alt_min_type(AltFrame::Absolute);
    let leftover = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 10.0,
        now_ms: 1_001,
    });
    assert!(leftover.need_home_alt);
    almost(leftover.safe_relhome_alt_min_m, 20.0 - 10.0 - MARGIN_DEFAULT_M);
}

#[test]
fn alt_min_already_breached_does_not_re_fire_until_backup() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    let first = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(10.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(first.newly_breached);
    almost(first.backup_alt_m, -10.0);

    let still = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(5.0),
        home_alt_amsl_m: 0.0,
        now_ms: 2_000,
    });
    assert!(!still.newly_breached);
    assert!(!still.recorded_breach);
    assert_eq!(fence.get_breach_count(), 1);

    let backup = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(-10.0),
        home_alt_amsl_m: 0.0,
        now_ms: 3_000,
    });
    assert!(backup.newly_breached);
    assert!(backup.recorded_breach);
    assert!(!backup.need_gcs_fence_status);
    almost(backup.backup_alt_m, -10.0 - ALT_MIN_BACKUP_DISTANCE_M);
    assert_eq!(fence.get_breach_count(), 2);
}

#[test]
fn alt_min_margin_and_clear_on_climb() {
    let mut fence = Fence::new();
    enable_alt_min(&mut fence);
    assert!(
        fence
            .check_fence_alt_min(CheckAltMinContext {
                alt_u_m: Some(10.0),
                home_alt_amsl_m: 0.0,
                now_ms: 1_001,
            })
            .newly_breached
    );

    let margin = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(21.0),
        home_alt_amsl_m: 0.0,
        now_ms: 2_000,
    });
    assert!(!margin.newly_breached);
    assert!(margin.cleared_breach);
    assert!(margin.margin_breached);
    almost(margin.backup_alt_m, 0.0);
    assert_eq!(fence.get_breaches() & TYPE_ALT_MIN, 0);
    assert_eq!(fence.get_margin_breaches() & TYPE_ALT_MIN, TYPE_ALT_MIN);

    let clear = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(50.0),
        home_alt_amsl_m: 0.0,
        now_ms: 3_000,
    });
    assert!(!clear.margin_breached);
    assert_eq!(fence.get_margin_breaches() & TYPE_ALT_MIN, 0);
}

#[test]
fn alt_min_disabled_does_not_clear_or_record() {
    let mut fence = Fence::new();
    fence.set_alt_min_m(20.0);
    let leftover = fence.check_fence_alt_min(CheckAltMinContext {
        alt_u_m: Some(10.0),
        home_alt_amsl_m: 0.0,
        now_ms: 1_001,
    });
    assert!(!leftover.enabled);
    assert!(!leftover.newly_breached);
    assert!(!leftover.need_alt_in_frame);
    assert_eq!(fence.get_breaches(), 0);
    almost(fence.alt_min_m(), 20.0);
    almost(ALT_MIN_DEFAULT_M, -10.0);
}


fn check_ctx(alt_max_u_m: f32, alt_min_u_m: f32, north_m: f32, east_m: f32) -> CheckContext {
    CheckContext {
        disable_auto_fences: false,
        now_ms: 1_001,
        location_valid: true,
        ne_home_m: Some((north_m, east_m)),
        alt_max_u_m: Some(alt_max_u_m),
        alt_min_u_m: Some(alt_min_u_m),
        home_alt_amsl_m: 0.0,
    }
}

#[test]
fn check_skips_when_disabled_and_floor_not_configured() {
    let mut fence = Fence::new();
    let leftover = fence.check(CheckContext::default());
    assert!(leftover.skipped);
    assert_eq!(leftover.new_breaches, 0);
    assert!(!leftover.need_location);
    assert!(!leftover.polygon_checked);
    assert!(!leftover.alt_max.enabled);
    assert!(!leftover.circle.enabled);
}

#[test]
fn check_skips_when_no_fence_types_are_configured() {
    let mut fence = Fence::from_params(true, TYPE_ALT_MAX | TYPE_CIRCLE);
    fence.set_configured_fences(0);
    let leftover = fence.check(check_ctx(200.0, 50.0, 1_000.0, 0.0));
    assert!(leftover.skipped);
    assert_eq!(leftover.new_breaches, 0);
    assert_eq!(fence.get_breaches(), 0);
}

#[test]
fn get_auto_disable_fences_matches_upstream_switch() {
    let mut fence = Fence::new();
    assert_eq!(fence.get_auto_disable_fences(), TYPE_ALT_MIN);
    fence.set_auto_enabled(AutoEnable::EnableOnAutoTakeoff);
    assert_eq!(fence.get_auto_disable_fences(), TYPE_ALL);
    fence.set_auto_enabled(AutoEnable::EnableDisableFloorOnly);
    assert_eq!(fence.get_auto_disable_fences(), TYPE_ALT_MIN);
    fence.set_auto_enabled(AutoEnable::OnlyWhenArmed);
    assert_eq!(fence.get_auto_disable_fences(), TYPE_ALT_MIN);

    fence.set_configured_fences(TYPE_ALT_MIN);
    let leftover = fence.enable(true, TYPE_ALT_MIN, true);
    assert_eq!(leftover.min_alt_state, MinAltState::ManuallyEnabled);
    assert_eq!(fence.get_auto_disable_fences(), 0);
}

#[test]
fn check_or_s_fresh_alt_max_and_circle_breaches() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    let leftover = fence.check(check_ctx(120.0, 50.0, CIRCLE_RADIUS_DEFAULT_M + 10.0, 0.0));
    assert!(!leftover.skipped);
    assert!(leftover.need_location);
    assert!(leftover.last_check_loc_valid);
    assert_eq!(leftover.new_breaches, TYPE_ALT_MAX | TYPE_CIRCLE);
    assert!(leftover.alt_max.newly_breached);
    assert!(leftover.alt_max.recorded_breach);
    assert!(leftover.circle.newly_breached);
    assert!(leftover.circle.recorded_breach);
    assert!(!leftover.polygon_checked);
    assert_eq!(fence.get_breaches(), TYPE_ALT_MAX | TYPE_CIRCLE);
    assert_eq!(fence.get_breach_count(), 2);
}

#[test]
fn check_clears_breach_from_a_type_that_is_no_longer_configured() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    assert!(
        fence
            .check_fence_circle(CheckCircleContext {
                ne_home_m: Some((CIRCLE_RADIUS_DEFAULT_M + 10.0, 0.0)),
                now_ms: 1_001,
            })
            .newly_breached
    );
    assert_eq!(fence.get_breaches() & TYPE_CIRCLE, TYPE_CIRCLE);

    fence.set_configured_fences(TYPE_ALT_MAX);
    let leftover = fence.check(check_ctx(50.0, 50.0, CIRCLE_RADIUS_DEFAULT_M + 10.0, 0.0));
    assert_eq!(leftover.cleared_unconfigured_breach, TYPE_CIRCLE);
    assert_eq!(fence.get_breaches() & TYPE_CIRCLE, 0);
    assert!(!leftover.skipped);
}

#[test]
fn check_auto_disables_the_floor_on_landing_unless_manually_enabled() {
    let mut fence = Fence::new();
    fence.set_configured_fences(TYPE_ALT_MIN | TYPE_ALT_MAX);
    fence.set_alt_min_m(20.0);
    let enabled = fence.enable(true, TYPE_ALT_MIN | TYPE_ALT_MAX, false);
    assert_eq!(enabled.changed_mask, TYPE_ALT_MIN | TYPE_ALT_MAX);
    assert_eq!(fence.min_alt_state(), MinAltState::Default);
    assert!(fence.floor_enabled());

    let mut ctx = check_ctx(50.0, 10.0, 0.0, 0.0);
    ctx.disable_auto_fences = true;
    let leftover = fence.check(ctx);
    assert_eq!(leftover.disabled_fences, TYPE_ALT_MIN);
    assert_eq!(leftover.fences_to_disable, TYPE_ALT_MIN);
    assert_eq!(leftover.auto_disabled_message, Some(TYPE_ALT_MIN));
    assert_eq!(leftover.disable_changed_mask, TYPE_ALT_MIN);
    assert!(!fence.floor_enabled());
    assert_eq!(leftover.new_breaches & TYPE_ALT_MIN, 0);
    assert!(!leftover.alt_min.enabled);
}

#[test]
fn check_records_a_breach_during_recovery_but_returns_zero() {
    let mut fence = Fence::new();
    enable_circle_and_alt(&mut fence);
    fence.set_manual_recovery_start_ms(1_000);

    let leftover = fence.check(check_ctx(120.0, 50.0, 0.0, 0.0));
    assert!(leftover.manual_recovery_active);
    assert!(!leftover.manual_recovery_expired);
    assert_eq!(leftover.new_breaches, 0);
    assert_eq!(fence.get_breaches() & TYPE_ALT_MAX, TYPE_ALT_MAX);
    assert_eq!(fence.manual_recovery_start_ms(), 1_000);

    let mut later = check_ctx(120.0, 50.0, 0.0, 0.0);
    later.now_ms = 1_000 + MANUAL_RECOVERY_TIME_MIN_MS;
    let expired = fence.check(later);
    assert!(!expired.manual_recovery_active);
    assert!(expired.manual_recovery_expired);
    assert_eq!(fence.manual_recovery_start_ms(), 0);
    assert_eq!(expired.new_breaches, 0);
    assert!(!expired.alt_max.newly_breached);
}

#[test]
fn check_auto_enables_the_floor_once_above_the_safe_min() {
    let mut fence = Fence::from_params(true, TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_ALT_MIN);
    fence.set_alt_min_m(20.0);
    assert!(fence.enable_param());
    assert!(!fence.floor_enabled());
    assert_eq!(fence.get_enabled_fences() & TYPE_ALT_MIN, 0);

    let leftover = fence.check(check_ctx(50.0, 30.0, 0.0, 0.0));
    assert!(!leftover.skipped);
    assert!(leftover.floor_auto_enabled);
    assert!(leftover.need_gcs_floor_notice);
    assert!(!leftover.floor_alt_unavailable);
    assert!(fence.floor_enabled());
    assert_eq!(fence.get_enabled_fences() & TYPE_ALT_MIN, TYPE_ALT_MIN);
    assert_eq!(fence.min_alt_state(), MinAltState::Default);
}

#[test]
fn check_does_not_auto_enable_floor_when_enable_param_is_off() {
    let mut fence = Fence::from_params(false, TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_ALT_MIN);
    fence.set_configured_fences(TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_ALT_MIN);
    fence.enable(true, TYPE_ALT_MAX | TYPE_CIRCLE, false);
    fence.set_alt_min_m(20.0);
    assert!(!fence.enable_param());
    assert_eq!(fence.auto_enabled(), AutoEnable::AlwaysDisabled);

    let leftover = fence.check(check_ctx(50.0, 30.0, 0.0, 0.0));
    assert!(!leftover.floor_auto_enabled);
    assert!(!fence.floor_enabled());
}

#[test]
fn check_polygon_bit_stays_a_loader_leftover() {
    let mut fence = Fence::new();
    fence.set_configured_fences(TYPE_POLYGON | TYPE_ALT_MAX);
    fence.enable(true, TYPE_POLYGON | TYPE_ALT_MAX, false);
    let leftover = fence.check(check_ctx(50.0, 50.0, 0.0, 0.0));
    assert!(!leftover.polygon_checked);
    assert_eq!(leftover.new_breaches & TYPE_POLYGON, 0);
}

#[test]
fn manual_recovery_time_is_ten_seconds() {
    assert_eq!(MANUAL_RECOVERY_TIME_MIN_MS, 10_000);
}
