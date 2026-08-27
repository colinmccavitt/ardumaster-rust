//! Mission scheduler hookup wiring.

use ap_landing::go_around::{LandingFlags, LandingType};
use ap_math::location::{AltFrame, Location};
use ap_plane::landing_loop::LandingContext;
use ap_plane::mission_scheduler_hookup::{
    mission_scheduler_tick, MissionContext, MissionSchedulerInputs,
};
use ap_plane::mode_table::ModeNumber;
use ap_plane::target_altitude::TargetAltitude;

fn wp(lat: i32, lng: i32) -> Location {
    Location::new_with_alt(lat, lng, 10_000, AltFrame::Absolute)
}

#[test]
fn mission_tick_skips_outside_auto() {
    let mut ctx = MissionContext::default();
    let landing = LandingContext::default();
    let out = mission_scheduler_tick(
        &mut ctx,
        &landing,
        &MissionSchedulerInputs {
            control_mode: ModeNumber::Manual.as_number(),
            waypoint_count: 2,
            waypoints: [wp(0, 0), wp(100, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0)],
            ..MissionSchedulerInputs::default()
        },
    );
    assert!(!out.ran);
    assert!(!out.advanced);
}

#[test]
fn mission_tick_advances_within_wp_radius() {
    let mut ctx = MissionContext::default();
    let landing = LandingContext::default();
    let target = wp(-35_000_000, 149_000_000);
    let mut next = target;
    next.offset(500.0, 0.0);
    let out = mission_scheduler_tick(
        &mut ctx,
        &landing,
        &MissionSchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            current_loc: next,
            waypoint_count: 2,
            waypoints: [target, next, wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0)],
            wp_radius_m: 100.0,
            ..MissionSchedulerInputs::default()
        },
    );
    assert!(out.ran);
    assert!(out.advanced);
    assert_eq!(ctx.current_index, 1);
}

#[test]
fn mission_tick_selects_landing_glide_slope_on_approach() {
    let mut ctx = MissionContext::default();
    let landing = LandingContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::StandardGlideSlope,
        machine: ap_landing::landing_state_machine::LandingMachineState {
            slope_stage: ap_landing::slope_stage::SlopeStage::Approach,
            ..Default::default()
        },
    };
    let out = mission_scheduler_tick(
        &mut ctx,
        &landing,
        &MissionSchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            current_loc: wp(-35_000_000, 149_000_000),
            waypoint_count: 1,
            waypoints: [wp(-35_000_000, 149_000_000), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0), wp(0, 0)],
            ..MissionSchedulerInputs::default()
        },
    );
    assert!(out.ran);
    assert_eq!(out.target, TargetAltitude::LandingGlideSlope);
}
