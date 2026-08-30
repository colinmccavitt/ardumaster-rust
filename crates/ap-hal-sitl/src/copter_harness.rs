//! COP-031: Copter analogue of SitlHarness (FW-046) / C++ SitlCopterHarness
//! (CCP-043/045). Sensors from SimMulticopter truth, leftover Copter tick,
//! motor PWM back into the Frame/Motor plant.
//!
//! Leftover mission (arm / takeoff / hold / land) matches C++
//! `copter_sitl_run_leftover.hpp`. Collective leftover throttle becomes N
//! motor PWM values mixed by AP_MotorsMatrix (quad X) then SIM_Frame /
//! SIM_Motor. `leftover_hold_command` is leftover altitude-rate damping,
//! not AC_PosControl and not the plant.

#![allow(missing_docs)]

use ap_motors::armed::{output_armed_stabilizing, ArmedDemand};
use ap_motors::MotorMatrix;
use ap_sim::sim_motor::SitlInput;
use ap_sim::sim_multicopter::SimMulticopter;
use ap_sim::sim_plane::Vec3;

/// C++ leftover Copter vehicle shell for SitlCopterHarness.
#[derive(Debug, Clone)]
pub struct LeftoverCopter {
    pub gyro_buffer: Vec3,
    pub accel_buffer: Vec3,
    pub baro_altitude_m: f32,
    pub gps_lat: i32,
    pub gps_lng: i32,
    pub compass_field_bf: Vec3,
    pub gyro_injected: bool,
    pub accel_injected: bool,
    pub baro_injected: bool,
    pub gps_injected: bool,
    pub compass_injected: bool,
    pub motors_armed: bool,
    pub motors_armed_injected: bool,
    pub spool_unlimited: bool,
    pub spool_injected: bool,
    pub attitude_hold: bool,
    pub attitude_hold_injected: bool,
    pub motor_pwm: [u16; 32],
    pub home_lat: i32,
    pub home_lng: i32,
    pub tick_count: u32,
    pub land_complete: bool,
    mixer: MotorMatrix,
    mixer_inited: bool,
}

impl Default for LeftoverCopter {
    fn default() -> Self {
        Self {
            gyro_buffer: Vec3::zero(),
            accel_buffer: Vec3::zero(),
            baro_altitude_m: 0.0,
            gps_lat: 0,
            gps_lng: 0,
            compass_field_bf: Vec3::zero(),
            gyro_injected: false,
            accel_injected: false,
            baro_injected: false,
            gps_injected: false,
            compass_injected: false,
            motors_armed: false,
            motors_armed_injected: false,
            spool_unlimited: false,
            spool_injected: false,
            attitude_hold: false,
            attitude_hold_injected: false,
            motor_pwm: [0; 32],
            home_lat: -353_632_621,
            home_lng: 1_491_652_374,
            tick_count: 0,
            land_complete: false,
            mixer: MotorMatrix::new(),
            mixer_inited: false,
        }
    }
}

pub fn leftover_copter_tick(copter: &mut LeftoverCopter) {
    copter.tick_count = copter.tick_count.saturating_add(1);
}

/// Leftover mission phases, C++ `MissionPhase`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    Disarmed,
    Takeoff,
    Hold,
    Land,
    Landed,
}

impl MissionPhase {
    pub fn name(self) -> &'static str {
        match self {
            Self::Disarmed => "DISARMED",
            Self::Takeoff => "TAKEOFF",
            Self::Hold => "HOLD",
            Self::Land => "LAND",
            Self::Landed => "LANDED",
        }
    }
}

#[derive(Debug, Clone)]
pub struct LeftoverMission {
    pub phase: MissionPhase,
    pub takeoff_alt_m: f32,
    pub climb_command: f32,
    pub land_command: f32,
    pub hold_s: f32,
    pub hold_elapsed_s: f32,
    pub command: f32,
}

impl Default for LeftoverMission {
    fn default() -> Self {
        Self {
            phase: MissionPhase::Disarmed,
            takeoff_alt_m: 10.0,
            climb_command: 0.70,
            land_command: 0.20,
            hold_s: 2.0,
            hold_elapsed_s: 0.0,
            command: 0.0,
        }
    }
}

/// Leftover mission hold: hoverThrOut plus a thin vertical-rate damper.
/// NED +z down: positive vz (descending) -> more command.
pub fn leftover_hold_command(sim: &SimMulticopter) -> f32 {
    const VEL_GAIN: f32 = 0.08;
    (sim.hover_command() + VEL_GAIN * sim.velocity_ef.z).clamp(0.0, 1.0)
}

fn ensure_mixer(copter: &mut LeftoverCopter) {
    if copter.mixer_inited {
        return;
    }
    // FRAME_CLASS=1 QUAD, FRAME_TYPE=1 X — same as C++ leftover_apply_collective.
    let ok = copter.mixer.setup_motors(1, 1);
    debug_assert!(ok, "QUAD X");
    copter.mixer_inited = true;
}

/// Leftover collective command → per-motor PWM through AP_MotorsMatrix,
/// matching C++ `leftover_apply_collective` (curve_expo=0, spin 0..1,
/// PWM 1000..2000).
pub fn leftover_apply_collective(copter: &mut LeftoverCopter, sim: &SimMulticopter, command: f32) {
    ensure_mixer(copter);
    for slot in &mut copter.motor_pwm {
        *slot = 0;
    }
    if !copter.motors_armed {
        return;
    }
    let (roll, pitch, _yaw) = sim.dcm.to_euler();
    let roll_in = (-0.5 * roll).clamp(-1.0, 1.0);
    let pitch_in = (-0.5 * pitch).clamp(-1.0, 1.0);
    let yaw_in = (-0.2 * sim.gyro.z).clamp(-1.0, 1.0);
    let demand = ArmedDemand {
        roll: roll_in,
        pitch: pitch_in,
        yaw: yaw_in,
        throttle: command,
        throttle_avg_max: command,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 200,
        thrust_boost: false,
        thrust_boost_ratio: 0.0,
        motor_lost_index: 0,
    };
    let out = output_armed_stabilizing(&copter.mixer, &demand);
    let n = sim.frame.num_motors as usize;
    for i in 0..n {
        let pwm = (1000.0 + out.get_thrust_rpyt_out(i as u8) * 1000.0)
            .clamp(1000.0, 2000.0)
            .round() as u16;
        let servo = sim.frame.motors()[i].servo as usize;
        if let Some(slot) = copter.motor_pwm.get_mut(servo) {
            *slot = pwm;
        }
    }
}

pub fn leftover_mission_begin_takeoff(mission: &mut LeftoverMission) {
    mission.phase = MissionPhase::Takeoff;
    mission.hold_elapsed_s = 0.0;
}

pub fn leftover_mission_advance(
    copter: &mut LeftoverCopter,
    sim: &SimMulticopter,
    mission: &mut LeftoverMission,
    dt: f32,
) {
    let alt_m = -sim.position.z;
    match mission.phase {
        MissionPhase::Disarmed => {
            copter.motors_armed = false;
            copter.land_complete = true;
            mission.command = 0.0;
        }
        MissionPhase::Takeoff => {
            copter.motors_armed = true;
            if copter.land_complete {
                copter.land_complete = false;
            }
            mission.command = mission.climb_command;
            if alt_m >= mission.takeoff_alt_m {
                mission.phase = MissionPhase::Hold;
                mission.hold_elapsed_s = 0.0;
                mission.command = leftover_hold_command(sim);
            }
        }
        MissionPhase::Hold => {
            copter.motors_armed = true;
            mission.command = leftover_hold_command(sim);
            mission.hold_elapsed_s += dt;
            if mission.hold_elapsed_s >= mission.hold_s {
                mission.phase = MissionPhase::Land;
            }
        }
        MissionPhase::Land => {
            copter.motors_armed = true;
            mission.command = mission.land_command;
            if sim.on_ground() {
                copter.land_complete = true;
                copter.motors_armed = false;
                mission.command = 0.0;
                mission.phase = MissionPhase::Landed;
            }
        }
        MissionPhase::Landed => {
            copter.motors_armed = false;
            copter.land_complete = true;
            mission.command = 0.0;
        }
    }
    leftover_apply_collective(copter, sim, mission.command);
}

/// SitlCopterHarness: sensors from sim, leftover tick, PWM into plant.
pub struct SitlCopterHarness {
    tick_count: u32,
}

impl Default for SitlCopterHarness {
    fn default() -> Self {
        Self::new()
    }
}

impl SitlCopterHarness {
    pub fn new() -> Self {
        Self { tick_count: 0 }
    }

    pub fn tick_count(&self) -> u32 {
        self.tick_count
    }

    pub fn step(&mut self, copter: &mut LeftoverCopter, sim: &mut SimMulticopter, dt: f32) {
        copter.gyro_buffer = sim.gyro;
        copter.accel_buffer = sim.accel_body;
        copter.gyro_injected = true;
        copter.accel_injected = true;

        sim.update_position();
        sim.update_mag_field_bf();
        copter.baro_altitude_m = -sim.position.z;
        copter.baro_injected = true;

        copter.gps_lat = sim.location_lat_e7;
        copter.gps_lng = sim.location_lng_e7;
        copter.gps_injected = true;

        copter.compass_field_bf = sim.get_mag_field_bf();
        copter.compass_injected = true;

        copter.motors_armed_injected = true;
        if copter.motors_armed {
            copter.spool_unlimited = true;
            copter.attitude_hold = true;
        } else {
            copter.spool_unlimited = false;
            copter.attitude_hold = false;
        }
        copter.spool_injected = true;
        copter.attitude_hold_injected = true;

        leftover_copter_tick(copter);
        self.tick_count = copter.tick_count;

        let mut input = SitlInput::default();
        input.servos = copter.motor_pwm;
        sim.update(&input, dt);
    }
}

/// Mission leftover then harness (sensors + Frame/Motor plant).
pub fn leftover_copter_sitl_step(
    harness: &mut SitlCopterHarness,
    copter: &mut LeftoverCopter,
    sim: &mut SimMulticopter,
    mission: &mut LeftoverMission,
    dt: f32,
) {
    leftover_mission_advance(copter, sim, mission, dt);
    harness.step(copter, sim, dt);
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.0025;

    #[test]
    fn zero_command_stays_on_ground() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission::default();
        leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        assert!(sim.on_ground());
        assert!((-sim.position.z).abs() < 0.01);
    }

    #[test]
    fn climb_command_via_leftover_pwm_leaves_the_ground() {
        let mut copter = LeftoverCopter::default();
        copter.motors_armed = true;
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission::default();
        leftover_mission_begin_takeoff(&mut mission);
        for _ in 0..1200 {
            leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        }
        assert!(-sim.position.z > 2.0, "alt={}", -sim.position.z);
        assert!(!sim.on_ground());
    }

    #[test]
    fn hover_thr_out_holds_altitude_on_the_real_plant() {
        let mut sim = SimMulticopter::new("x");
        sim.position.z = -10.0;
        sim.velocity_ef = Vec3::zero();
        let mut copter = LeftoverCopter::default();
        copter.motors_armed = true;
        let mut harness = SitlCopterHarness::new();
        let hover = sim.hover_command();
        for _ in 0..400 {
            leftover_apply_collective(&mut copter, &sim, hover);
            harness.step(&mut copter, &mut sim, DT);
        }
        let alt = -sim.position.z;
        assert!((alt - 10.0).abs() < 1.5, "alt={alt}");
    }

    #[test]
    fn leftover_copter_sitl_mission_arm_takeoff_hold_land() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission::default();
        leftover_mission_begin_takeoff(&mut mission);

        let mut max_alt_m = 0.0f32;
        let mut saw_hold = false;
        let max_ticks = 20 * 400;
        for _ in 0..max_ticks {
            leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
            let alt = -sim.position.z;
            if alt > max_alt_m {
                max_alt_m = alt;
            }
            if mission.phase == MissionPhase::Hold {
                saw_hold = true;
            }
            if mission.phase == MissionPhase::Landed {
                break;
            }
        }

        assert!(saw_hold, "never reached HOLD, phase={:?}", mission.phase);
        assert!(max_alt_m >= 9.0, "max_alt={max_alt_m}");
        assert_eq!(mission.phase, MissionPhase::Landed);
        assert!(copter.land_complete);
        assert!(!copter.motors_armed);
        assert!(sim.on_ground());
        assert!(harness.tick_count() > 0);
        assert!(copter.gyro_injected);
        assert!(copter.baro_injected);
    }

    #[test]
    fn harness_synthesizes_gyro_accel_baro_gps_compass() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        sim.position.z = -5.0;
        sim.gyro = Vec3::new(0.1, 0.0, 0.0);
        let mut harness = SitlCopterHarness::new();
        harness.step(&mut copter, &mut sim, DT);
        assert!(copter.gyro_injected);
        assert!(copter.accel_injected);
        assert!(copter.baro_injected);
        assert!(copter.gps_injected);
        assert!(copter.compass_injected);
        assert!(copter.tick_count > 0);
    }
}
