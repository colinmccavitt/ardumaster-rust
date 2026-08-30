//! Port of libraries/SITL/SIM_QuadPlane.h/.cpp via C++ `sim_quadplane.hpp`.
//! Plane aero + Frame motors, motor_offset, tilt/firefly/tailsitter variants.

#![allow(missing_docs)]

use crate::sim_battery::{Battery, SitlParams};
use crate::sim_frame::Frame;
use crate::sim_motor::{SitlInput, ServoType};
use crate::sim_plane::{
    AirframeMix, GroundBehavior, Mat3, SimPlane, Vec3, GRAVITY_MSS,
};

fn servo_norm(pwm: u16) -> f32 {
    ((pwm as f32 - 1500.0) / 500.0).clamp(-1.0, 1.0)
}

fn radians(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

/// Ground-truth QuadPlane plant. Upstream `SITL::QuadPlane`.
pub struct SimQuadPlane {
    pub plane: SimPlane,
    pub frame: Frame,
    pub battery: Battery,
    pub sitl_params: SitlParams,
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub battery_temperature_degc: f32,
    pub time_now_us: u64,
    pub location_alt_cm: i32,
    thrust_scale_zero: bool,
    copter_tailsitter: bool,
}

impl SimQuadPlane {
    pub fn new(frame_str: &str) -> Self {
        let mut plane = SimPlane::new();
        let mut frame_type = "x";
        let mut motor_offset: u8 = 4;
        plane.ground_behavior = GroundBehavior::NoMovement;
        let mut thrust_scale_zero = false;
        let mut copter_tailsitter = false;

        if frame_str.contains("-octa-quad-cor") {
            frame_type = "octa-quad-cor";
        } else if frame_str.contains("-octa-quad-cw-cor") {
            frame_type = "octa-quad-cw-cor";
        } else if frame_str.contains("-octa-quad") || frame_str.contains("-octaquad") {
            frame_type = "octa-quad";
        } else if frame_str.contains("-octa") {
            frame_type = "octa";
        } else if frame_str.contains("-hexax") {
            frame_type = "hexax";
        } else if frame_str.contains("-hexa") {
            frame_type = "hexa";
        } else if frame_str.contains("-plus") {
            frame_type = "+";
        } else if frame_str.contains("-y6") {
            frame_type = "y6";
        } else if frame_str.contains("-tri") {
            frame_type = "tri";
        } else if frame_str.contains("-tilttrivec") {
            frame_type = "tilttrivec";
            thrust_scale_zero = true;
        } else if frame_str.contains("-tilthvec") {
            frame_type = "tilthvec";
        } else if frame_str.contains("-tilttri") {
            frame_type = "tilttri";
            thrust_scale_zero = true;
        } else if frame_str.contains("firefly") {
            frame_type = "firefly";
            plane.frame_config.mix = AirframeMix::Elevons;
            thrust_scale_zero = true;
            motor_offset = 2;
        } else if frame_str.contains("-tilt") {
            frame_type = "tilt";
            thrust_scale_zero = true;
        } else if frame_str.contains("cl84") {
            frame_type = "tilttri";
            thrust_scale_zero = true;
        } else if frame_str.contains("-copter_tailsitter") {
            frame_type = "+";
            copter_tailsitter = true;
            plane.ground_behavior = GroundBehavior::Tailsitter;
        }

        let mut frame = Frame::create_frame(frame_type);
        if !frame.valid() {
            frame = Frame::create_frame("x");
        }
        if frame_str.contains("cl84") && frame.num_motors >= 2 {
            frame.motors_mut()[0].servo_type = ServoType::Retract;
            frame.motors_mut()[0].servo_rate = 7.0 * 60.0 / 90.0;
            frame.motors_mut()[1].servo_type = ServoType::Retract;
            frame.motors_mut()[1].servo_rate = 7.0 * 60.0 / 90.0;
        }
        frame.motor_offset = motor_offset;
        frame.set_mass_scale(1.5);
        frame.init(frame_str);
        plane.mass = frame.get_mass();

        let mut battery = Battery::new(10.0);
        battery.setup(
            frame.get_model_batt_capacity_ah(),
            frame.get_model_batt_resistance_ohm(),
            frame.get_model_batt_max_voltage(),
            25.0,
        );
        let battery_voltage = battery.get_voltage();
        frame.set_battery_voltage(battery_voltage);

        Self {
            plane,
            frame,
            battery,
            sitl_params: SitlParams::default(),
            battery_voltage,
            battery_current: 0.0,
            battery_temperature_degc: 0.0,
            time_now_us: 0,
            location_alt_cm: 0,
            thrust_scale_zero,
            copter_tailsitter,
        }
    }

    pub fn update(&mut self, input: &SitlInput, dt: f32) {
        self.plane.mass = self.frame.get_mass();
        self.plane.update_wind();

        let aileron = servo_norm(input.servos[0]);
        let elevator = servo_norm(input.servos[1]);
        let mut throttle = (input.servos[2] as f32 - 1000.0) / 1000.0;
        throttle = throttle.clamp(0.0, 1.0);
        if self.thrust_scale_zero {
            throttle = 0.0;
        }
        let rudder = servo_norm(input.servos[3]);
        let mixed = self.plane.mix_surfaces(aileron, elevator, rudder, throttle);
        self.plane.angle_of_attack = self
            .plane
            .velocity_air_bf
            .z
            .atan2(self.plane.velocity_air_bf.x);
        self.plane.beta = self
            .plane
            .velocity_air_bf
            .y
            .atan2(self.plane.velocity_air_bf.x);
        let force = self.plane.get_force(
            mixed.aileron,
            mixed.elevator,
            mixed.rudder,
            self.plane.angle_of_attack,
            self.plane.beta,
            self.plane.airspeed,
            self.plane.gyro,
            self.plane.air_density,
        );
        let mut rot_accel = self.plane.get_torque(
            mixed.aileron,
            mixed.elevator,
            mixed.rudder,
            mixed.throttle,
            force,
            self.plane.angle_of_attack,
            self.plane.airspeed,
            self.plane.beta,
            self.plane.gyro,
            self.plane.air_density,
        );
        let thrust_scale = (self.plane.mass * GRAVITY_MSS) / self.plane.hover_throttle;
        let thrust_newtons = mixed.throttle * thrust_scale;
        self.plane.accel_body = Vec3::new(thrust_newtons, 0.0, 0.0)
            .plus(force)
            .scaled(1.0 / self.plane.mass);

        let alt_amsl = self.location_alt_cm as f32 * 0.01;
        self.frame.set_battery_voltage(self.battery_voltage);
        let (mut quad_rot, mut quad_accel) = self.frame.calculate_forces(
            self.plane.dcm,
            self.plane.velocity_air_ef,
            self.plane.gyro,
            alt_amsl,
            input,
            self.plane.mass,
            true,
            self.time_now_us,
        );
        if self.copter_tailsitter {
            let r = Mat3::from_euler(0.0, radians(270.0), 0.0);
            quad_rot = r.apply(quad_rot);
            quad_accel = r.apply(quad_accel);
        }

        if self.frame.battery_changed() {
            self.battery.setup(
                self.frame.get_model_batt_capacity_ah(),
                self.frame.get_model_batt_resistance_ohm(),
                self.frame.get_model_batt_max_voltage(),
                25.0,
            );
        }
        self.battery.maybe_reset(
            self.sitl_params.batt_voltage,
            self.sitl_params.batt_capacity_ah,
            self.sitl_params.batt_resistance,
        );
        self.battery_voltage = self.battery.get_voltage();
        self.battery_current = self.frame.get_current_amp();
        self.battery_temperature_degc = self.battery.get_temperature_degc();
        self.battery.consume_energy(self.battery_current, self.time_now_us);
        self.battery_current += 20.0 * throttle.abs();
        self.frame.set_battery_voltage(self.battery_voltage);

        rot_accel = rot_accel.plus(quad_rot);
        self.plane.accel_body = self.plane.accel_body.plus(quad_accel);
        self.plane.update_dynamics(rot_accel, dt);
        self.time_now_us = self.time_now_us.saturating_add((dt * 1.0e6) as u64);
        self.location_alt_cm = ((self.plane.home_alt_m - self.plane.position.z) * 100.0) as i32;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_quadplane_is_x_frame_offset_4() {
        let qp = SimQuadPlane::new("quadplane");
        assert!(qp.frame.valid());
        assert_eq!(qp.frame.num_motors, 4);
        assert_eq!(qp.frame.motor_offset, 4);
        assert!((qp.frame.get_mass() - 4.5).abs() < 1e-3, "mass={}", qp.frame.get_mass());
    }

    #[test]
    fn firefly_uses_elevons_and_motor_offset_2() {
        let qp = SimQuadPlane::new("firefly");
        assert_eq!(qp.frame.motor_offset, 2);
        assert_eq!(qp.plane.frame_config.mix, AirframeMix::Elevons);
    }

    #[test]
    fn hover_command_on_quad_motors_leaves_ground() {
        let mut qp = SimQuadPlane::new("quadplane");
        let mut input = SitlInput::default();
        let cmd = (qp.frame.hover_command() + 0.12).clamp(0.0, 1.0);
        qp.frame.set_equal_command(&mut input, cmd);
        input.servos[2] = 1000;
        qp.plane.position.z = -1.0;
        let dt = 0.0025_f32;
        for _ in 0..2000 {
            qp.update(&input, dt);
        }
        assert!(
            -qp.plane.position.z > 1.5,
            "alt={} on_ground={} mass={}",
            -qp.plane.position.z,
            qp.plane.on_ground(),
            qp.plane.mass
        );
        assert!(qp.battery_voltage.is_finite());
        assert!(qp.plane.gyro.x.is_finite());
    }
}
