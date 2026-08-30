//! VT-011: quadplane_sitl_run — SitlQuadPlaneHarness closed loop on the
//! original-source SimQuadPlane plant (VT-010). Thin hover then forward
//! transition scenario. Not MAVLink, not Mission Planner.
//!
//! USAGE: quadplane_sitl_run [--help] [duration_seconds]
//! Default 20 simulated seconds @ 400 Hz. Exit 0 if hover climbed and
//! transition produced finite airspeed; 1 otherwise.

use ap_hal_sitl::{set_sticks, setup_hover_transition, SitlQuadPlaneHarness};
use ap_plane::main_loop::PlaneMainLoop;
use ap_quadplane::QuadPlane;
use ap_sim::sim_quadplane::SimQuadPlane;

fn print_usage(argv0: &str) {
    eprintln!(
        "usage: {argv0} [--help] [duration_seconds > 0]\n  VT-011: Plane+QuadPlane+SimQuadPlane hover then transition\n  Default duration: 20 simulated seconds @ 400Hz."
    );
}

fn main() {
    const DT: f32 = 0.0025;
    const TICKS_PER_SECOND: i32 = 400;

    let argv0 = std::env::args()
        .next()
        .unwrap_or_else(|| "quadplane_sitl_run".into());
    let duration_s: i32 = match std::env::args().nth(1) {
        Some(arg) if arg == "--help" || arg == "-h" => {
            print_usage(&argv0);
            std::process::exit(0);
        }
        Some(arg) => match arg.parse() {
            Ok(v) if v > 0 => v,
            _ => {
                print_usage(&argv0);
                std::process::exit(1);
            }
        },
        None => 20,
    };
    let num_ticks = duration_s * TICKS_PER_SECOND;

    let mut plane = PlaneMainLoop::default();
    let mut sim = SimQuadPlane::new("quadplane");
    let mut qp = QuadPlane::with_enable(1);
    setup_hover_transition(&mut plane, &mut qp);
    let mut harness = SitlQuadPlaneHarness::new();

    println!(
        "VT-011 SITL: Plane+QuadPlane+SimQuadPlane, hover then transition, {duration_s} simulated seconds ({} ticks @ 400Hz)",
        num_ticks
    );
    println!(
        "frame={} motors={} motor_offset={} q_enable={} available={}",
        sim.frame.name,
        sim.frame.num_motors,
        sim.frame.motor_offset,
        qp.enable(),
        if qp.available() { 1 } else { 0 }
    );

    let print_telemetry = |t_s: f32, phase: &str, sim: &SimQuadPlane, ticks: u32| {
        let (r, p, y) = sim.plane.true_euler_deg();
        let pos = sim.plane.position;
        println!(
            "t={t_s:6.3}s  phase={phase}  alt={:.3}m  as={:.3}m/s  pos_ned=({:.3}, {:.3}, {:.3})  true_rpy_deg=({r:.3}, {p:.3}, {y:.3})  ticks={ticks}",
            -pos.z,
            sim.plane.airspeed,
            pos.x,
            pos.y,
            pos.z,
        );
    };

    print_telemetry(0.0, "HOVER", &sim, 0);

    let mut max_alt_m = 0.0f32;
    let mut max_as = 0.0f32;
    let mut saw_transition = false;
    let mut now_ms = 0_u32;
    let hover = sim.frame.hover_command();
    let climb = (hover + 0.20).clamp(0.0, 1.0);

    for i in 0..num_ticks {
        now_ms = now_ms.saturating_add(3); // 400 Hz -> 2.5 ms; keep integer ms
        let t_s = (i + 1) as f32 * DT;
        let mut phase = "HOVER";
        let mut vtol_cmd = climb;
        let mut thr_pwm: u16 = 1000;
        if t_s >= 8.0 {
            phase = "TRANSITION";
            saw_transition = true;
            vtol_cmd = hover;
            thr_pwm = 1700;
        } else if (-sim.plane.position.z) >= 8.0 {
            phase = "HOVER";
            vtol_cmd = hover;
            thr_pwm = 1000;
        }
        set_sticks(&mut plane, 1500, 1500, thr_pwm, 1500);
        harness.step(&mut plane, &mut qp, &mut sim, now_ms, DT, vtol_cmd, true);
        let alt = -sim.plane.position.z;
        if alt > max_alt_m {
            max_alt_m = alt;
        }
        if sim.plane.airspeed > max_as {
            max_as = sim.plane.airspeed;
        }
        if (i + 1) % TICKS_PER_SECOND == 0 {
            print_telemetry(t_s, phase, &sim, harness.tick_count());
        }
    }

    print_telemetry(
        num_ticks as f32 * DT,
        if saw_transition {
            "TRANSITION"
        } else {
            "HOVER"
        },
        &sim,
        harness.tick_count(),
    );
    let ok = max_alt_m.is_finite()
        && max_as.is_finite()
        && max_alt_m >= 5.0
        && qp.available()
        && harness.tick_count() > 0;
    println!(
        "Done: ticks={} max_alt={:.3}m max_as={:.3}m/s{}",
        harness.tick_count(),
        max_alt_m,
        max_as,
        if ok { " SUCCESS" } else { " FAIL" }
    );
    std::process::exit(if ok { 0 } else { 1 });
}
