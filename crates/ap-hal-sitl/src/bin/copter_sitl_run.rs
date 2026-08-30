//! COP-031: standalone copter_sitl_run — arm, takeoff, hold, land on the
//! real SIM_Multicopter Frame/Motor plant (not leftover body-z / AttitudeSim).
//!
//! Counterpart of C++ `sitl/copter_sitl_run` (CCP-044/045). Not a `#[test]`,
//! not MAVLink, not Mission Planner.
//!
//! USAGE: copter_sitl_run [--help] [duration_seconds]
//! Default duration is 20 simulated seconds @ 400Hz. Exits 0 if LANDED after
//! climbing, 1 otherwise.

use ap_hal_sitl::{
    leftover_copter_sitl_step, leftover_mission_begin_takeoff, LeftoverCopter, LeftoverMission,
    MissionPhase, SitlCopterHarness,
};
use ap_sim::sim_multicopter::SimMulticopter;

fn print_usage(argv0: &str) {
    eprintln!(
        "usage: {argv0} [--help] [duration_seconds > 0]\n  COP-031: LeftoverCopter + SimMulticopter Frame/Motor plant\n  Mission: arm, takeoff to 10m, hold 2s, land.\n  Default duration: 20 simulated seconds @ 400Hz."
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
        None => 20,
    };
    let num_ticks = duration_s * TICKS_PER_SECOND;

    let mut copter = LeftoverCopter::default();
    let mut sim = SimMulticopter::new("x");
    let mut harness = SitlCopterHarness::new();
    let mut mission = LeftoverMission::default();

    println!(
        "COP-031 SITL: LeftoverCopter+SimMulticopter Frame/Motor, {duration_s} simulated seconds ({num_ticks} ticks @ 400Hz)"
    );
    println!(
        "mission: arm, takeoff {:.3}m, hold {:.3}s, land  frame={} motors={}",
        mission.takeoff_alt_m,
        mission.hold_s,
        sim.frame.name,
        sim.num_motors()
    );

    let print_telemetry = |t_s: f32,
                           copter: &LeftoverCopter,
                           sim: &SimMulticopter,
                           mission: &LeftoverMission,
                           ticks: u32| {
        let (tr, tp, ty) = sim.true_euler_deg();
        println!(
            "t={t_s:6.3}s  phase={}  armed={}  land_complete={}  alt={:.3}m  baro={:.3}m  cmd={:.3}  pos_ned=({:.3}, {:.3}, {:.3})  true_rpy_deg=({tr:.3}, {tp:.3}, {ty:.3})  ticks={ticks}",
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

    let mut max_alt_m = 0.0f32;
    let mut landed_tick: i32 = -1;
    for i in 1..num_ticks {
        leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        let alt = -sim.position.z;
        if alt > max_alt_m {
            max_alt_m = alt;
        }
        if landed_tick < 0 && mission.phase == MissionPhase::Landed {
            landed_tick = i;
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
    let ok = mission.phase == MissionPhase::Landed
        && max_alt_m >= (mission.takeoff_alt_m * 0.9)
        && copter.land_complete
        && sim.on_ground();
    println!(
        "Done: ticks={} max_alt={:.3}m phase={} {}",
        harness.tick_count(),
        max_alt_m,
        mission.phase.name(),
        if ok { "SUCCESS" } else { "FAIL" }
    );
    std::process::exit(if ok { 0 } else { 1 });
}
