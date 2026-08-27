//! Vehicle loop rangefinder bump hookup.

use ap_landing::go_around::{LandingFlags, LandingType};
use ap_landing::rangefinder_bump::{RangefinderBumpConfig, RangefinderBumpInputs};
use ap_landing::slope_stage::RangefinderState;
use ap_landing::{SlopeConfig, SlopeInputs};
use ap_math::location::{AltContext, AltFrame, Location};
use ap_plane::rangefinder_bump_hookup::{
    rangefinder_bump_hookup, RangefinderBumpContext, RangefinderBumpHookupInputs,
};

fn hookup_inputs(rf: RangefinderState) -> RangefinderBumpHookupInputs {
    let prev = Location::new_with_alt(0, 0, 10_000, AltFrame::Absolute);
    let mut next = prev;
    next.offset(1000.0, 0.0);
    next.set_alt_cm(0, AltFrame::Absolute);

    RangefinderBumpHookupInputs {
        flight_stage_is_land: true,
        landing_type: LandingType::StandardGlideSlope,
        bump_cfg: RangefinderBumpConfig {
            shallow_threshold: 1.0,
            steep_threshold_deg: 1.0,
        },
        slope_cfg: SlopeConfig {
            flare_sec: 2.0,
            flare_alt: 3.0,
            flare_effectivness_pct: 50,
        },
        slope_inp: SlopeInputs {
            prev_wp: prev,
            next_wp: next,
            current: prev,
            groundspeed: 20.0,
            land_sinkrate: 1.0,
            alt_ctx: AltContext {
                home_alt_cm: Some(0),
                origin_alt_cm: Some(0),
                terrain_alt_cm: Some(0),
            },
        },
        bump: RangefinderBumpInputs {
            rf,
            prev_wp: prev,
            next_wp: next,
            current: prev,
            wp_distance_m: 300.0,
            adjusted_altitude_cm: 10_000,
            alt_ctx: AltContext {
                home_alt_cm: Some(0),
                origin_alt_cm: Some(0),
                terrain_alt_cm: Some(0),
            },
        },
    }
}

#[test]
fn hookup_skipped_outside_land_stage() {
    let mut ctx = RangefinderBumpContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        ..RangefinderBumpContext::default()
    };
    let mut inp = hookup_inputs(RangefinderState {
        in_use: true,
        correction: 6.0,
        last_stable_correction: 0.0,
    });
    inp.flight_stage_is_land = false;
    assert!(rangefinder_bump_hookup(&mut ctx, &inp).is_none());
}

#[test]
fn hookup_skipped_for_deepstall() {
    let mut ctx = RangefinderBumpContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        slope: 0.05,
        ..RangefinderBumpContext::default()
    };
    let mut inp = hookup_inputs(RangefinderState {
        in_use: true,
        correction: 6.0,
        last_stable_correction: 0.0,
    });
    inp.landing_type = LandingType::Deepstall;
    assert!(rangefinder_bump_hookup(&mut ctx, &inp).is_none());
}

#[test]
fn hookup_recalculates_slope_on_large_bump() {
    let mut ctx = RangefinderBumpContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        slope: 0.05,
        ..RangefinderBumpContext::default()
    };
    let inp = hookup_inputs(RangefinderState {
        in_use: true,
        correction: 6.0,
        last_stable_correction: 0.0,
    });
    let result = rangefinder_bump_hookup(&mut ctx, &inp).expect("recalculated");
    assert!(result.recalculated);
    assert!(!result.go_around);
    assert_eq!(ctx.rf.last_stable_correction, 6.0);
}

#[test]
fn hookup_abort_wires_go_around_flag() {
    let mut ctx = RangefinderBumpContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        slope: 0.05,
        initial_slope: 0.0,
        ..RangefinderBumpContext::default()
    };
    let inp = hookup_inputs(RangefinderState {
        in_use: true,
        correction: -40.0,
        last_stable_correction: 0.0,
    });
    let result = rangefinder_bump_hookup(&mut ctx, &inp).expect("recalculated");
    assert!(result.go_around);
    assert!(ctx.flags.commanded_go_around);
    assert!(ctx.slope_flags.has_aborted_due_to_slope_recalc);
}
