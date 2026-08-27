//! Landing loop scheduler hookup wiring.

use ap_landing::go_around::{LandingFlags, LandingType};
use ap_landing::landing_state_machine::LandingMachineState;
use ap_landing::slope_stage::{FlareConfig, SlopeStage};
use ap_math::location::Location;
use ap_math::vector2::Vector2f;
use ap_plane::landing_loop::{LandingContext, VerifyLandVehicleInputs};
use ap_plane::landing_loop_hookup::{landing_loop_scheduler_tick, LandingLoopSchedulerInputs};

fn landing_ctx() -> LandingContext {
    LandingContext {
        flags: LandingFlags {
            in_progress: true,
            commanded_go_around: false,
        },
        landing_type: LandingType::StandardGlideSlope,
        machine: LandingMachineState::default(),
    }
}

fn verify_inputs() -> VerifyLandVehicleInputs {
    VerifyLandVehicleInputs {
        height_above_target_m: 20.0,
        terrain_correction_m: 0.0,
        sink_rate_ms: 2.0,
        wp_proportion: 0.6,
        is_flying: true,
        rangefinder_in_range: true,
        bearing_error_cd: 500,
        crosstrack_error_m: 1.0,
        nav_data_is_stale: false,
        below_prev_wp: false,
        prev_cmd_is_loiter_to_alt: false,
        crash_detection_enable: false,
        flare_cfg: FlareConfig {
            flare_alt: 3.0,
            flare_sec: 2.0,
            pre_flare_alt: 8.0,
            pre_flare_sec: 0.0,
            pre_flare_airspeed: 12.0,
        },
        deepstall: ap_landing::deepstall_stage::DeepstallVerifyInputs {
            distance_to_landing_m: 50.0,
            distance_to_arc_entry_m: 150.0,
            loiter_radius_m: 100.0,
            loiter_ccw: false,
            reached_loiter: true,
            height_error_m: 1.0,
            target_bearing_cd: 500,
            heading_error_deg: 5.0,
            target_heading_deg: 0.0,
            groundspeed_ne: Vector2f::new(10.0, 0.0),
            current: Location::new(-35_000_000, 149_000_000),
            arc_exit: Location::new(-35_000_000, 149_000_000),
            arc_entry: Location::new(-35_000_000, 149_000_000),
            extended_approach: Location::new(-35_000_000, 149_000_000),
            entry_point: Location::new(-35_000_000, 149_000_000),
        },
    }
}

#[test]
fn scheduler_tick_advances_slope_stage_and_constrains_roll() {
    let mut ctx = landing_ctx();
    let out = landing_loop_scheduler_tick(
        &mut ctx,
        &LandingLoopSchedulerInputs {
            verify: verify_inputs(),
            nav_roll_cd: 6000,
            level_roll_limit_cd: 4500,
        },
    );
    assert!(out.ran);
    assert_eq!(ctx.machine.slope_stage, SlopeStage::Approach);
    assert_eq!(out.nav_roll_cd, 6000);
    assert!(!out.throttle_suppressed);

    ctx.machine.slope_stage = SlopeStage::Final;
    let out = landing_loop_scheduler_tick(
        &mut ctx,
        &LandingLoopSchedulerInputs {
            verify: verify_inputs(),
            nav_roll_cd: 6000,
            level_roll_limit_cd: 4500,
        },
    );
    assert_eq!(out.nav_roll_cd, 4500);
    assert!(out.throttle_suppressed);
}
