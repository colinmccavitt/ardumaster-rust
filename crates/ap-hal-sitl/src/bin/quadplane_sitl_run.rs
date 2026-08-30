//! VT-010: standalone quadplane_sitl_run — leftover hover mission on
//! original-source SimQuadPlane (Plane aero + Frame motors).
//!
//! C++ sitl/quadplane_sitl_run is currently a 17-line plant smoke; this
//! binary is a real bounded 400 Hz loop with periodic stdout telemetry.

use ap_hal_sitl::{
    QuadLeftoverMission, QuadMissionPhase, SitlQuadPlaneHarness,
};
use ap_sim::sim_quadplane::SimQuadPlane;

fn main() {
    const DT: f32 = 0.0025;
    const TICKS_PER_SECOND: i32 = 400;
    let duration_s: i32 = match std::env::args().nth(1) {
        Some(arg) if arg == "--help" || arg == "-h" => {
            eprintln!("usage: quadplane_sitl_run [duration_seconds > 0]");
            std::process::exit(0);
        }
        Some(arg) => arg.parse().unwrap_or(20),
        None => 20,
    };
    if duration_s <= 0 {
        eprintln!("usage: quadplane_sitl_run [duration_seconds > 0]");
        std::process::exit(1);
    }
    let num_ticks = duration_s * TICKS_PER_SECOND;

    let mut qp = SimQuadPlane::new("quadplane");
    let mut harness = SitlQuadPlaneHarness::new();
    let mut mission = QuadLeftoverMission::default();
    mission.phase = QuadMissionPhase::Takeoff;

    println!(
        "VT-010 SITL: SimQuadPlane+SitlQuadPlaneHarness, {duration_s} simulated seconds ({} ticks @ 400Hz)",
        num_ticks
    );
    println!(
        "mission: takeoff {:.1}m, hold {:.1}s, land  frame={} motors={} offset={}",
        mission.takeoff_alt_m,
        mission.hold_s,
        qp.frame.name,
        qp.frame.num_motors,
        qp.frame.motor_offset
    );

    let print_tel = |t_s: f32, qp: &SimQuadPlane, mission: &QuadLeftoverMission, ticks: u32| {
        let (r, p, y) = qp.plane.true_euler_deg();
        println!(
            "t={t_s:6.3}s  phase={}  alt={:.3}m  as={:.3}  V={:.3}  true_rpy_deg=({r:.3}, {p:.3}, {y:.3})  ticks={ticks}",
            mission.phase.name(),
            -qp.plane.position.z,
            qp.plane.airspeed,
            qp.battery_voltage,
        );
    };

    print_tel(0.0, &qp, &mission, 0);
    let mut max_alt = 0.0f32;
    let mut landed_tick = -1i32;
    for i in 0..num_ticks {
        harness.step(&mut qp, &mut mission, DT);
        let alt = -qp.plane.position.z;
        if alt > max_alt {
            max_alt = alt;
        }
        if landed_tick < 0 && mission.phase == QuadMissionPhase::Landed {
            landed_tick = i;
        }
        if (i + 1) % TICKS_PER_SECOND == 0 {
            print_tel((i + 1) as f32 * DT, &qp, &mission, harness.tick_count());
        }
        if landed_tick >= 0 && (i - landed_tick) >= TICKS_PER_SECOND {
            break;
        }
    }
    print_tel(
        harness.tick_count() as f32 * DT,
        &qp,
        &mission,
        harness.tick_count(),
    );
    let ok = mission.phase == QuadMissionPhase::Landed
        && max_alt >= mission.takeoff_alt_m * 0.7
        && qp.plane.on_ground()
        && qp.battery_voltage.is_finite();
    println!(
        "Done: ticks={} max_alt={:.3}m phase={} {}",
        harness.tick_count(),
        max_alt,
        mission.phase.name(),
        if ok { "SUCCESS" } else { "FAIL" }
    );
    std::process::exit(if ok { 0 } else { 1 });
}
