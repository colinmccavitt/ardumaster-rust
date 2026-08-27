//! Mission waypoint advancement and target altitude for the scheduler tick.
//!
//! Upstream `ModeAuto::run` advances the mission when the vehicle reaches each
//! waypoint and calls `Mode::update_target_altitude` for the active leg.

use ap_math::location::Location;

use crate::landing_loop::{target_altitude_landing_inputs, LandingContext};
use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::target_altitude::{target_altitude, TargetAltitude, TargetAltitudeInputs};

/// Persistent mission state the vehicle carries, upstream `AP_Mission` index.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MissionContext {
    pub current_index: u16,
    pub complete: bool,
}

/// HAL inputs for one mission scheduler tick.
#[derive(Debug, Clone, Copy)]
pub struct MissionSchedulerInputs {
    pub control_mode: u8,
    pub current_loc: Location,
    pub waypoints: [Location; 8],
    pub waypoint_count: u8,
    pub wp_radius_m: f32,
    pub offset_cm: i32,
    pub next_wp_is_terrain_alt: bool,
    pub past_interval_finish_line: bool,
    pub reached_loiter_target: bool,
    pub soaring_gliding: bool,
    pub terrain_proportion_ok: bool,
    pub landing_point: Location,
}

impl Default for MissionSchedulerInputs {
    fn default() -> Self {
        Self {
            control_mode: 0,
            current_loc: Location::new(0, 0),
            waypoints: [Location::new(0, 0); 8],
            waypoint_count: 0,
            wp_radius_m: 90.0,
            offset_cm: 0,
            next_wp_is_terrain_alt: false,
            past_interval_finish_line: false,
            reached_loiter_target: false,
            soaring_gliding: false,
            terrain_proportion_ok: false,
            landing_point: Location::new(0, 0),
        }
    }
}

/// Result of one mission scheduler tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MissionSchedulerOutput {
    pub prev_wp: Location,
    pub next_wp: Location,
    pub target: TargetAltitude,
    pub advanced: bool,
    pub complete: bool,
    pub ran: bool,
}

fn is_auto_mode(control_mode: u8) -> bool {
    ModeNumber::from_number(control_mode, &BuildFeatures::default()) == Some(ModeNumber::Auto)
}

fn waypoint_at(waypoints: &[Location; 8], count: u8, index: u16) -> Location {
    if count == 0 {
        return Location::new(0, 0);
    }
    let idx = (index as usize).min(count as usize - 1);
    waypoints[idx]
}

/// Advance the mission when AUTO and within WP_RADIUS, then evaluate target altitude.
#[must_use]
pub fn mission_scheduler_tick(
    ctx: &mut MissionContext,
    landing: &LandingContext,
    inp: &MissionSchedulerInputs,
) -> MissionSchedulerOutput {
    if !is_auto_mode(inp.control_mode) || inp.waypoint_count == 0 {
        return MissionSchedulerOutput {
            prev_wp: Location::new(0, 0),
            next_wp: Location::new(0, 0),
            target: TargetAltitude::FromNextWaypoint,
            advanced: false,
            complete: ctx.complete,
            ran: false,
        };
    }

    let next_wp = waypoint_at(&inp.waypoints, inp.waypoint_count, ctx.current_index);
    let prev_wp = if ctx.current_index == 0 {
        inp.current_loc
    } else {
        waypoint_at(&inp.waypoints, inp.waypoint_count, ctx.current_index - 1)
    };

    let mut advanced = false;
    if !ctx.complete {
        let dist_m = inp.current_loc.get_distance(next_wp);
        if dist_m <= inp.wp_radius_m {
            let next_index = ctx.current_index.saturating_add(1);
            if next_index >= u16::from(inp.waypoint_count) {
                ctx.complete = true;
            } else {
                ctx.current_index = next_index;
            }
            advanced = true;
        }
    }

    let mut alt_inp = target_altitude_landing_inputs(landing, inp.landing_point);
    alt_inp.offset_cm = inp.offset_cm;
    alt_inp.next_wp_is_terrain_alt = inp.next_wp_is_terrain_alt;
    alt_inp.past_interval_finish_line = inp.past_interval_finish_line;
    alt_inp.reached_loiter_target = inp.reached_loiter_target;
    alt_inp.soaring_gliding = inp.soaring_gliding;

    let terrain_ok = inp.terrain_proportion_ok;
    let target = target_altitude(&alt_inp, || terrain_ok);

    let out_next = if ctx.complete {
        next_wp
    } else {
        waypoint_at(&inp.waypoints, inp.waypoint_count, ctx.current_index)
    };

    MissionSchedulerOutput {
        prev_wp,
        next_wp: out_next,
        target,
        advanced,
        complete: ctx.complete,
        ran: true,
    }
}
