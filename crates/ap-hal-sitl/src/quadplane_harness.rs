//! VT-010: SitlQuadPlaneHarness — leftover hover mission on original-source
//! SimQuadPlane (Plane aero + Frame motors). C++ sitl/quadplane_sitl_run is
//! currently a 17-line plant smoke; this is a real bounded binary harness.
//!
//! Collective leftover uses the same AP_MotorsMatrix P-hold as copter
//! leftover_apply_collective (attitude P on roll/pitch, rate P on yaw)
//! written onto Frame motors at motor_offset. FW throttle stays at PWM
//! 1000 so the hover is copter-axis, not a 50% body-X shove.

#![allow(missing_docs)]

use ap_motors::armed::{output_armed_stabilizing, ArmedDemand};
use ap_motors::MotorMatrix;
use ap_sim::sim_motor::SitlInput;
use ap_sim::sim_quadplane::SimQuadPlane;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuadMissionPhase {
    Disarmed,
    Takeoff,
    Hold,
    Land,
    Landed,
}

impl QuadMissionPhase {
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

pub struct QuadLeftoverMission {
    pub phase: QuadMissionPhase,
    pub takeoff_alt_m: f32,
    pub hold_s: f32,
    pub hold_elapsed_s: f32,
    pub fw_throttle_pwm: u16,
    pub climb_command: f32,
    pub land_command: f32,
}

impl Default for QuadLeftoverMission {
    fn default() -> Self {
        Self {
            phase: QuadMissionPhase::Disarmed,
            takeoff_alt_m: 10.0,
            hold_s: 2.0,
            hold_elapsed_s: 0.0,
            fw_throttle_pwm: 1000,
            climb_command: 0.70,
            land_command: 0.20,
        }
    }
}

pub struct SitlQuadPlaneHarness {
    tick_count: u32,
    mixer: MotorMatrix,
    mixer_inited: bool,
}

impl Default for SitlQuadPlaneHarness {
    fn default() -> Self {
        Self::new()
    }
}

impl SitlQuadPlaneHarness {
    pub fn new() -> Self {
        Self {
            tick_count: 0,
            mixer: MotorMatrix::new(),
            mixer_inited: false,
        }
    }
    pub fn tick_count(&self) -> u32 {
        self.tick_count
    }

    fn leftover_pwm(mixer: &MotorMatrix, qp: &SimQuadPlane, command: f32) -> SitlInput {
        let mut input = SitlInput::default();
        input.servos[2] = 1000;
        if command <= 0.0 {
            return input;
        }
        let (roll, pitch, _yaw) = qp.plane.dcm.to_euler();
        let demand = ArmedDemand {
            roll: (-0.5 * roll).clamp(-1.0, 1.0),
            pitch: (-0.5 * pitch).clamp(-1.0, 1.0),
            yaw: (-0.2 * qp.plane.gyro.z).clamp(-1.0, 1.0),
            throttle: command,
            throttle_avg_max: command,
            throttle_thrust_max: 1.0,
            compensation_gain: 1.0,
            yaw_headroom: 200,
            thrust_boost: false,
            thrust_boost_ratio: 0.0,
            motor_lost_index: 0,
        };
        let out = output_armed_stabilizing(mixer, &demand);
        let n = qp.frame.num_motors as usize;
        let offset = qp.frame.motor_offset as usize;
        for i in 0..n {
            let pwm = (1000.0 + out.get_thrust_rpyt_out(i as u8) * 1000.0)
                .clamp(1000.0, 2000.0)
                .round() as u16;
            let servo = qp.frame.motors()[i].servo as usize;
            if let Some(slot) = input.servos.get_mut(offset + servo) {
                *slot = pwm;
            }
        }
        input.servos[2] = 1000;
        input
    }

    pub fn step(&mut self, qp: &mut SimQuadPlane, mission: &mut QuadLeftoverMission, dt: f32) {
        if !self.mixer_inited {
            let ok = self.mixer.setup_motors(1, 1);
            debug_assert!(ok, "QUAD X");
            self.mixer_inited = true;
        }
        let alt = -qp.plane.position.z;
        let mut command;
        match mission.phase {
            QuadMissionPhase::Disarmed => {
                command = 0.0;
            }
            QuadMissionPhase::Takeoff => {
                command = mission.climb_command;
                if alt >= mission.takeoff_alt_m {
                    mission.phase = QuadMissionPhase::Hold;
                    mission.hold_elapsed_s = 0.0;
                }
            }
            QuadMissionPhase::Hold => {
                const VEL_GAIN: f32 = 0.08;
                command = (qp.frame.hover_command() + VEL_GAIN * qp.plane.velocity_ef.z).clamp(0.0, 1.0);
                mission.hold_elapsed_s += dt;
                if mission.hold_elapsed_s >= mission.hold_s {
                    mission.phase = QuadMissionPhase::Land;
                }
            }
            QuadMissionPhase::Land => {
                command = mission.land_command;
                if qp.plane.on_ground() {
                    mission.phase = QuadMissionPhase::Landed;
                    command = 0.0;
                }
            }
            QuadMissionPhase::Landed => {
                command = 0.0;
            }
        }
        let mut input = Self::leftover_pwm(&self.mixer, qp, command);
        input.servos[2] = mission.fw_throttle_pwm;
        qp.update(&input, dt);
        self.tick_count = self.tick_count.saturating_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_mission_climbs() {
        let mut qp = SimQuadPlane::new("quadplane");
        let mut harness = SitlQuadPlaneHarness::new();
        let mut mission = QuadLeftoverMission::default();
        mission.phase = QuadMissionPhase::Takeoff;
        let dt = 0.0025_f32;
        for _ in 0..2000 {
            harness.step(&mut qp, &mut mission, dt);
        }
        assert!(
            -qp.plane.position.z > 2.0,
            "alt={} roll={}",
            -qp.plane.position.z,
            qp.plane.true_euler_deg().0
        );
        assert!(qp.battery_voltage.is_finite());
        assert!(harness.tick_count() > 0);
    }

    #[test]
    fn leftover_quadplane_mission_takeoff_hold_land() {
        let mut qp = SimQuadPlane::new("quadplane");
        let mut harness = SitlQuadPlaneHarness::new();
        let mut mission = QuadLeftoverMission::default();
        mission.phase = QuadMissionPhase::Takeoff;
        let dt = 0.0025_f32;
        let mut max_alt = 0.0f32;
        for _ in 0..(20 * 400) {
            harness.step(&mut qp, &mut mission, dt);
            max_alt = max_alt.max(-qp.plane.position.z);
            if mission.phase == QuadMissionPhase::Landed {
                break;
            }
        }
        assert_eq!(mission.phase, QuadMissionPhase::Landed);
        assert!(max_alt >= 7.0, "max_alt={max_alt}");
        assert!(qp.plane.on_ground());
        assert!(qp.battery_voltage.is_finite());
    }
}
