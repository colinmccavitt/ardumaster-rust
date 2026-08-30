//! FW-046: the first standalone runnable in ardumaster-rust.
//!
//! Counterpart of C++ `sitl/sitl_run` (CPP-085). Not a `#[test]`, not MAVLink,
//! not Mission Planner. `fn main()` constructs `PlaneMainLoop` + `SimPlane` +
//! `SitlHarness`, ticks at 50 Hz, and prints one telemetry line per simulated
//! second.
//!
//! Scenario is the C++ sitl_run / vehicle_test FBWA recipe, reused verbatim:
//!   ModeFBWA, `soft_armed = true` (not `arm()`, same gate reason as C++),
//!   sticks 1650/1500/1700/1500 every tick, 50 Hz, default 60 simulated
//!   seconds. Plant is the original-source SimPlane aero port, not AttitudeSim.

use ap_hal_sitl::{set_sticks, SitlHarness, SITL_LOOP_HZ};
use ap_plane::main_loop::PlaneMainLoop;
use ap_sim::sim_plane::SimPlane;

fn main() {
    let duration_s: i32 = match std::env::args().nth(1) {
        Some(arg) => match arg.parse::<i32>() {
            Ok(n) if n > 0 => n,
            _ => {
                eprintln!("usage: sitl_run [duration_seconds > 0]");
                std::process::exit(1);
            }
        },
        None => 60,
    };

    let dt = 1.0 / f32::from(SITL_LOOP_HZ);
    let ticks_per_second = i32::from(SITL_LOOP_HZ);
    let num_ticks = duration_s * ticks_per_second;

    let mut plane = PlaneMainLoop::default();
    let mut sim = SimPlane::new();
    SitlHarness::setup_fbwa(&mut plane, &mut sim);
    let mut harness = SitlHarness::new();

    println!(
        "FW-046 SITL executable: PlaneMainLoop+SimPlane+SitlHarness, ModeFBWA, {duration_s} simulated seconds ({} ticks @ {}Hz)",
        num_ticks, SITL_LOOP_HZ
    );

    let print_telemetry = |now_ms: u32, plane: &PlaneMainLoop, sim: &SimPlane| {
        let (true_roll, true_pitch, true_yaw) = sim.true_euler_deg();
        let ahrs_roll = plane.roll_rad * 180.0 / core::f32::consts::PI;
        let ahrs_pitch = plane.pitch_rad * 180.0 / core::f32::consts::PI;
        let ahrs_yaw = plane.yaw_rad * 180.0 / core::f32::consts::PI;
        println!(
            "t={:6.3}s  mode=FBWA  pos_ned=({:.3}, {:.3}, {:.3})  alt={:.3}m  true_rpy_deg=({:.3}, {:.3}, {:.3})  ahrs_rpy_deg=({:.3}, {:.3}, {:.3})  true_airspeed={:.3}m/s",
            now_ms as f32 / 1000.0,
            sim.position.x,
            sim.position.y,
            sim.position.z,
            sim.altitude_m(),
            true_roll,
            true_pitch,
            true_yaw,
            ahrs_roll,
            ahrs_pitch,
            ahrs_yaw,
            sim.airspeed,
        );
    };

    let mut now_ms: u32 = 0;
    let mut prev_cal = plane.airspeed_offset_calibrated;
    print_telemetry(now_ms, &plane, &sim);

    for i in 0..num_ticks {
        now_ms = now_ms.saturating_add(20);
        set_sticks(&mut plane, 1650, 1500, 1700, 1500);
        harness.step(&mut plane, &mut sim, now_ms, dt);
        if plane.airspeed_offset_calibrated != prev_cal {
            println!(
                "  [airspeed_sensor] calibration_state: {} -> {} at t={:.3}s",
                if prev_cal { "Success" } else { "InProgress" },
                if plane.airspeed_offset_calibrated {
                    "Success"
                } else {
                    "InProgress"
                },
                now_ms as f32 / 1000.0
            );
            prev_cal = plane.airspeed_offset_calibrated;
        }

        if (i + 1) % ticks_per_second == 0 {
            print_telemetry(now_ms, &plane, &sim);
        }
    }

    println!("Done: {num_ticks} ticks ({duration_s} simulated seconds) completed.");
}
