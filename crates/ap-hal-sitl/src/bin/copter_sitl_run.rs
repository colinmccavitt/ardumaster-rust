//! COP-034: standalone copter_sitl_run — arm, takeoff, fly 1 mile, RTL, land
//! on the real SIM_Multicopter Frame/Motor plant.
//!
//! Horizontal motion is original-source AC_PosControl NE + WPNav + ModeRTL
//! leftover mixed as differential PWM. Not leftover equal-collective.
//!
//! Counterpart of C++ `sitl/copter_sitl_run`. Not a `#[test]`, not MAVLink,
//! not Mission Planner.
//!
//! USAGE: copter_sitl_run [--help] [duration_seconds]
//! Default duration is 600 simulated seconds @ 400Hz (covers mile+return
//! at WP_SPD 10 m/s). Exits 0 if LANDED after climbing, 1 otherwise.

use ap_hal_sitl::{
    leftover_copter_sitl_step, leftover_mission_begin_takeoff, leftover_touchdown_miss_m,
    LeftoverCopter, LeftoverMission, MissionPhase, SitlCopterHarness, MILE_M,
};
use ap_sim::sim_multicopter::SimMulticopter;

fn print_usage(argv0: &str) {
    eprintln!(
        "usage: {argv0} [--help] [duration_seconds > 0]\n  COP-034: LeftoverCopter + PosControl NE/D + WPNav + RTL + SimMulticopter\n  Mission: arm, takeoff 10m, fly 1 mile North, RTL, land original spot.\n  Default duration: 600 simulated seconds @ 400Hz."
    );
}

fn main() {
    const DT: f32 = 0.0025;
    const TICKS_PER_SECOND: i32 = 400;

    let mut args = std::env::args();
    let argv0 = args.next().unwrap_or_else(|| "copter_sitl_run".into());
    let duration_s: i32 = match args.next() {
        Some(arg) if arg == "--help" || arg == "-h" => {
            print_usage(&argv0);
            std::process::exit(0);
        }
        Some(arg) => match arg.parse::<i32>() {
            Ok(n) if n > 0 => n,
            _ => {
                print_usage(&argv0);
                std::process::exit(1);
            }
        },
        None => 600,
    };
    let num_ticks = duration_s * TICKS_PER_SECOND;

    let mut copter = LeftoverCopter::default();
    let mut sim = SimMulticopter::new("x");
    let mut harness = SitlCopterHarness::new();
    let mut mission = LeftoverMission::default();

    println!(
        "COP-034 SITL: LeftoverCopter+PosControl NE/D+WPNav+RTL+SimMulticopter, {duration_s} simulated seconds ({num_ticks} ticks @ 400Hz)"
    );
    println!(
        "mission: arm, takeoff {:.3}m, fly {:.3}m heading_rad={:.3} (North=0), RTL, land  frame={} motors={}",
        mission.takeoff_alt_m,
        mission.fly_distance_m,
        mission.fly_heading_rad,
        sim.frame.name,
        sim.num_motors()
    );
    debug_assert!((mission.fly_distance_m - MILE_M).abs() < 0.01);

    let print_telemetry = |t_s: f32,
                           copter: &LeftoverCopter,
                           sim: &SimMulticopter,
                           mission: &LeftoverMission,
                           ticks: u32| {
        let (tr, tp, ty) = sim.true_euler_deg();
        let spd = sim.velocity_ef.x.hypot(sim.velocity_ef.y);
        println!(
            "t={t_s:7.3}s  phase={}  armed={}  land_complete={}  alt={:.3}m  baro={:.3}m  cmd={:.3}  pos_ned=({:.3}, {:.3}, {:.3})  spd_ne={spd:.3}m/s  true_rpy_deg=({tr:.3}, {tp:.3}, {ty:.3})  ticks={ticks}",
            mission.phase.name(),
            if copter.motors_armed { 1 } else { 0 },
            if copter.land_complete { 1 } else { 0 },
            -sim.position.z,
            copter.baro_altitude_m,
            mission.command,
            sim.position.x,
            sim.position.y,
            sim.position.z,
        );
    };

    print_telemetry(0.0, &copter, &sim, &mission, harness.tick_count());

    leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
    leftover_mission_begin_takeoff(&mut mission);
    print_telemetry(
        harness.tick_count() as f32 * DT,
        &copter,
        &sim,
        &mission,
        harness.tick_count(),
    );

    let mut max_alt_m = 0.0f32;
    let mut max_spd_ne = 0.0f32;
    let mut landed_tick: i32 = -1;
    let mut prev_phase = mission.phase;
    let mut t_takeoff = 0.0f32;
    let mut t_cruise = 0.0f32;
    let mut t_rtl = 0.0f32;
    let mut t_land = 0.0f32;
    let mut t_landed = 0.0f32;
    for i in 1..num_ticks {
        leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        let alt = -sim.position.z;
        if alt > max_alt_m {
            max_alt_m = alt;
        }
        let spd = sim.velocity_ef.x.hypot(sim.velocity_ef.y);
        if spd > max_spd_ne {
            max_spd_ne = spd;
        }
        if mission.phase != prev_phase {
            let t_s = (i + 1) as f32 * DT;
            match mission.phase {
                MissionPhase::Takeoff => t_takeoff = t_s,
                MissionPhase::Cruise => t_cruise = t_s,
                MissionPhase::Rtl => t_rtl = t_s,
                MissionPhase::Land => t_land = t_s,
                MissionPhase::Landed => t_landed = t_s,
                MissionPhase::Disarmed => {}
            }
            print_telemetry(t_s, &copter, &sim, &mission, harness.tick_count());
            prev_phase = mission.phase;
        }
        if landed_tick < 0 && mission.phase == MissionPhase::Landed {
            landed_tick = i;
            if t_landed == 0.0 {
                t_landed = (i + 1) as f32 * DT;
            }
        }
        if (i + 1) % TICKS_PER_SECOND == 0 {
            print_telemetry(
                (i + 1) as f32 * DT,
                &copter,
                &sim,
                &mission,
                harness.tick_count(),
            );
        }
        if landed_tick >= 0 && (i - landed_tick) >= TICKS_PER_SECOND {
            break;
        }
    }

    print_telemetry(
        harness.tick_count() as f32 * DT,
        &copter,
        &sim,
        &mission,
        harness.tick_count(),
    );
    let miss = leftover_touchdown_miss_m(&mission, &sim);
    let ok = mission.phase == MissionPhase::Landed
        && max_alt_m >= (mission.takeoff_alt_m * 0.9)
        && copter.land_complete
        && sim.on_ground();
    println!(
        "times_s: takeoff={t_takeoff:.3} cruise={t_cruise:.3} rtl={t_rtl:.3} land={t_land:.3} landed={t_landed:.3}"
    );
    println!(
        "touchdown: pos_ned=({:.3}, {:.3}, {:.3}) origin=({:.3}, {:.3}) miss_horiz_m={miss:.3} max_alt={:.3}m max_spd_ne={max_spd_ne:.3}m/s",
        sim.position.x,
        sim.position.y,
        sim.position.z,
        mission.origin_n,
        mission.origin_e,
        max_alt_m,
    );
    println!(
        "Done: ticks={} max_alt={:.3}m phase={} miss_horiz={miss:.3}m {}",
        harness.tick_count(),
        max_alt_m,
        mission.phase.name(),
        if ok { "SUCCESS" } else { "FAIL" }
    );
    std::process::exit(if ok { 0 } else { 1 });
}
