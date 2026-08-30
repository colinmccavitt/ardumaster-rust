//! CCP-045 / COP-031: port of libraries/SITL/SIM_Motor.h + SIM_Motor.cpp
//! (Copter-4.7.0, same commit as Plane-4.7.0) via C++ `fwcpp::sim::Motor`.
//!
//! `Motor::calculate_forces` takes sitl_input PWM, converts to a 0..1 command
//! (`pwm_to_command`), slews, computes disc-actuator thrust (`calc_thrust`),
//! rotor yaw torque, optional tilt-servo rotation, optional momentum drag,
//! then `torque = position % thrust + rotor_torque`.
//!
//! ADR-0012: `time_us` and battery voltage are arguments (no HAL / Battery
//! singletons). Shares no arithmetic with the DCM estimator under test —
//! uses [`crate::sim_plane::Vec3`] / [`crate::sim_plane::Mat3`].

#![allow(missing_docs)]

use crate::sim_plane::{Mat3, Vec3};

/// Upstream `AP_MotorsMatrix.h` `AP_MOTORS_MATRIX_YAW_FACTOR_CW`.
pub const MOTORS_YAW_FACTOR_CW: f32 = -1.0;
/// Upstream `AP_MOTORS_MATRIX_YAW_FACTOR_CCW`.
pub const MOTORS_YAW_FACTOR_CCW: f32 = 1.0;

/// Upstream SITL `sitl_input.servos` length.
pub const SITL_SERVO_CHANNELS: usize = 32;

const FLT_EPSILON: f32 = 1.192_092_9e-7;

pub fn is_zero(x: f32) -> bool {
    x.abs() < FLT_EPSILON
}

pub fn is_negative(x: f32) -> bool {
    x < 0.0
}

pub fn constrain(v: f32, min: f32, max: f32) -> f32 {
    v.clamp(min, max)
}

pub fn radians(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

pub fn vec3_is_zero(v: Vec3) -> bool {
    is_zero(v.x) && is_zero(v.y) && is_zero(v.z)
}

/// Upstream `Vector3::projected`.
pub fn projected(this: Vec3, onto: Vec3) -> Vec3 {
    let denom = onto.dot(onto);
    if is_zero(denom) {
        Vec3::zero()
    } else {
        onto.scaled(this.dot(onto) / denom)
    }
}

/// PWM microseconds into the plant, matching C++ `SitlInput`.
#[derive(Debug, Clone, Copy)]
pub struct SitlInput {
    pub servos: [u16; SITL_SERVO_CHANNELS],
}

impl Default for SitlInput {
    fn default() -> Self {
        Self {
            servos: [0; SITL_SERVO_CHANNELS],
        }
    }
}

/// Upstream `Motor::ServoType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoType {
    Normal = 0,
    Retract = 1,
}

/// One rotor in a SIM_Frame, upstream `SITL::Motor`.
#[derive(Debug, Clone)]
pub struct Motor {
    pub angle: f32,
    pub yaw_factor: f32,
    pub servo: u8,
    pub display_order: u8,
    pub roll_servo: i8,
    pub roll_min: f32,
    pub roll_max: f32,
    pub pitch_servo: i8,
    pub pitch_min: f32,
    pub pitch_max: f32,
    pub servo_type: ServoType,
    pub servo_rate: f32,
    pub last_change_usec: u64,
    pub last_roll_value: f32,
    pub last_pitch_value: f32,
    mot_pwm_min: f32,
    mot_pwm_max: f32,
    mot_spin_min: f32,
    mot_spin_max: f32,
    mot_expo: f32,
    slew_max: f32,
    current: f32,
    power_factor: f32,
    voltage_max: f32,
    effective_prop_area: f32,
    max_outflow_velocity: f32,
    true_prop_area: f32,
    momentum_drag_coefficient: f32,
    diagonal_size: f32,
    last_command: f32,
    last_calc_us: u64,
    position: Vec3,
    thrust_vector: Vec3,
}

impl Default for Motor {
    fn default() -> Self {
        Self::new(0, 0.0, 0.0, 0)
    }
}

impl Motor {
    pub fn new(servo_idx: u8, angle_deg: f32, yaw: f32, order: u8) -> Self {
        Self::with_tilt(servo_idx, angle_deg, yaw, order, -1, 0.0, 0.0, -1, 0.0, 0.0)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn with_tilt(
        servo_idx: u8,
        angle_deg: f32,
        yaw: f32,
        order: u8,
        roll_srv: i8,
        rmin: f32,
        rmax: f32,
        pitch_srv: i8,
        pmin: f32,
        pmax: f32,
    ) -> Self {
        Self {
            angle: angle_deg,
            yaw_factor: yaw,
            servo: servo_idx,
            display_order: order,
            roll_servo: roll_srv,
            roll_min: rmin,
            roll_max: rmax,
            pitch_servo: pitch_srv,
            pitch_min: pmin,
            pitch_max: pmax,
            servo_type: ServoType::Normal,
            servo_rate: 0.24,
            last_change_usec: 0,
            last_roll_value: 0.0,
            last_pitch_value: 0.0,
            mot_pwm_min: 1000.0,
            mot_pwm_max: 2000.0,
            mot_spin_min: 0.15,
            mot_spin_max: 0.95,
            mot_expo: 0.65,
            slew_max: 150.0,
            current: 0.0,
            power_factor: 1.0,
            voltage_max: 12.6,
            effective_prop_area: 0.0,
            max_outflow_velocity: 0.0,
            true_prop_area: 0.0,
            momentum_drag_coefficient: 0.0,
            diagonal_size: 0.35,
            last_command: 0.0,
            last_calc_us: 0,
            position: Vec3::new(radians(angle_deg).cos(), radians(angle_deg).sin(), 0.0),
            thrust_vector: Vec3::new(0.0, 0.0, -1.0),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn calculate_forces(
        &mut self,
        input: &SitlInput,
        motor_offset: u8,
        velocity_air_bf: Vec3,
        gyro: Vec3,
        air_density: f32,
        voltage: f32,
        use_drag: bool,
        time_us: u64,
    ) -> (Vec3, Vec3) {
        let idx = (motor_offset as usize).saturating_add(self.servo as usize);
        let pwm = input.servos.get(idx).copied().unwrap_or(0) as f32;
        let mut command = self.pwm_to_command(pwm);
        let voltage_scale = voltage / self.voltage_max;

        if voltage_scale < 0.1 {
            self.current = 0.0;
            return (Vec3::zero(), Vec3::zero());
        }

        if self.last_calc_us != 0 && self.slew_max > 0.0 {
            let dt = (time_us.saturating_sub(self.last_calc_us)) as f32 * 1.0e-6;
            let slew_max_change = self.slew_max * dt;
            command = constrain(
                command,
                self.last_command - slew_max_change,
                self.last_command + slew_max_change,
            );
        }
        self.last_calc_us = time_us;
        self.last_command = command;

        let motor_vel = velocity_air_bf.plus(self.position.cross(gyro).scaled(-1.0));
        let velocity_in = (-projected(motor_vel, self.thrust_vector).z).max(0.0);
        let motor_thrust = self.calc_thrust(command, air_density, velocity_in, voltage_scale);

        let yaw_scale = 0.05 * self.diagonal_size * motor_thrust;
        let mut rotor_torque = self
            .thrust_vector
            .scaled(self.yaw_factor * command * yaw_scale * -1.0);
        let mut thrust = self.thrust_vector.scaled(motor_thrust);

        let mut roll = 0.0f32;
        let mut pitch = 0.0f32;
        if self.roll_servo >= 0 {
            let sidx = (self.roll_servo as u8 as usize).saturating_add(motor_offset as usize);
            let demand = input.servos.get(sidx).copied().unwrap_or(0);
            let last = self.last_roll_value;
            let (servoval, last_v) = self.update_servo(demand, time_us, last);
            self.last_roll_value = last_v;
            if self.roll_min < self.roll_max {
                roll = constrain(
                    self.roll_min
                        + (f32::from(servoval) - 1000.0) * 0.001 * (self.roll_max - self.roll_min),
                    self.roll_min,
                    self.roll_max,
                );
            } else {
                roll = constrain(
                    self.roll_max
                        + (2000.0 - f32::from(servoval)) * 0.001 * (self.roll_min - self.roll_max),
                    self.roll_max,
                    self.roll_min,
                );
            }
        }
        if self.pitch_servo >= 0 {
            let sidx = (self.pitch_servo as u8 as usize).saturating_add(motor_offset as usize);
            let demand = input.servos.get(sidx).copied().unwrap_or(0);
            let last = self.last_pitch_value;
            let (servoval, last_v) = self.update_servo(demand, time_us, last);
            self.last_pitch_value = last_v;
            if self.pitch_min < self.pitch_max {
                pitch = constrain(
                    self.pitch_min
                        + (f32::from(servoval) - 1000.0)
                            * 0.001
                            * (self.pitch_max - self.pitch_min),
                    self.pitch_min,
                    self.pitch_max,
                );
            } else {
                pitch = constrain(
                    self.pitch_max
                        + (2000.0 - f32::from(servoval))
                            * 0.001
                            * (self.pitch_min - self.pitch_max),
                    self.pitch_max,
                    self.pitch_min,
                );
            }
        }
        self.last_change_usec = time_us;

        if !is_zero(roll) || !is_zero(pitch) {
            let rotation = Mat3::from_euler(radians(roll), radians(pitch), 0.0);
            thrust = rotation.apply(thrust);
            rotor_torque = rotation.apply(rotor_torque);
        }

        if use_drag {
            let momentum_drag_factor =
                self.momentum_drag_coefficient * (air_density * self.true_prop_area).sqrt();
            let momentum_drag = Vec3::new(
                momentum_drag_factor
                    * motor_vel.x
                    * (thrust.y.abs().sqrt() + thrust.z.abs().sqrt()),
                momentum_drag_factor
                    * motor_vel.y
                    * (thrust.x.abs().sqrt() + thrust.z.abs().sqrt()),
                momentum_drag_factor
                    * motor_vel.z
                    * (thrust.x.abs().sqrt() + thrust.y.abs().sqrt() + thrust.z.abs().sqrt()),
            );
            thrust = thrust.minus(momentum_drag);
        }

        let torque = self.position.cross(thrust).plus(rotor_torque);
        let power = self.power_factor * motor_thrust.abs();
        self.current = power / voltage.max(0.1);
        (torque, thrust)
    }

    fn update_servo(&self, mut demand: u16, time_usec: u64, last_value: f32) -> (u16, f32) {
        if self.servo_rate <= 0.0 {
            return (demand, last_value);
        }
        if self.servo_type == ServoType::Retract {
            demand = if demand > 1700 {
                2000
            } else if demand < 1300 {
                1000
            } else {
                last_value as u16
            };
        }
        demand = constrain(f32::from(demand), 1000.0, 2000.0) as u16;
        let dt = (time_usec.saturating_sub(self.last_change_usec)) as f32 * 1.0e-6;
        let max_change = 1000.0 * (dt / self.servo_rate) * 60.0 / 90.0;
        let last_value = constrain(
            f32::from(demand),
            last_value - max_change,
            last_value + max_change,
        );
        ((last_value + 0.5) as u16, last_value)
    }

    pub fn get_current(&self) -> f32 {
        self.current
    }
    pub fn get_command(&self) -> f32 {
        self.last_command
    }
    pub fn get_position(&self) -> Vec3 {
        self.position
    }
    pub fn get_thrust_vector(&self) -> Vec3 {
        self.thrust_vector
    }

    #[allow(clippy::too_many_arguments)]
    pub fn setup_params(
        &mut self,
        pwm_min: u16,
        pwm_max: u16,
        spin_min: f32,
        spin_max: f32,
        expo: f32,
        slew: f32,
        diag_size: f32,
        pwr_factor: f32,
        volt_max: f32,
        eff_prop_area: f32,
        velocity_max: f32,
        pos: Vec3,
        thrust_vec: Vec3,
        yaw: f32,
        true_area: f32,
        mdrag: f32,
    ) {
        self.mot_pwm_min = f32::from(pwm_min);
        self.mot_pwm_max = f32::from(pwm_max);
        self.mot_spin_min = spin_min;
        self.mot_spin_max = spin_max;
        self.mot_expo = expo;
        self.slew_max = slew;
        self.power_factor = pwr_factor;
        self.voltage_max = volt_max;
        self.effective_prop_area = eff_prop_area;
        self.max_outflow_velocity = velocity_max;
        self.true_prop_area = true_area;
        self.momentum_drag_coefficient = mdrag;
        self.diagonal_size = diag_size;

        if !vec3_is_zero(pos) {
            self.position = pos;
        } else {
            self.position = Vec3::new(
                radians(self.angle).cos() * diag_size,
                radians(self.angle).sin() * diag_size,
                0.0,
            );
        }
        if !vec3_is_zero(thrust_vec) {
            self.thrust_vector = thrust_vec;
        }
        if !is_zero(yaw) {
            self.yaw_factor = yaw;
        }
    }

    pub fn pwm_to_command(&self, pwm: f32) -> f32 {
        let pwm_thrust_max =
            self.mot_pwm_min + self.mot_spin_max * (self.mot_pwm_max - self.mot_pwm_min);
        let pwm_thrust_min =
            self.mot_pwm_min + self.mot_spin_min * (self.mot_pwm_max - self.mot_pwm_min);
        let pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
        constrain((pwm - pwm_thrust_min) / pwm_thrust_range, 0.0, 1.0)
    }

    pub fn command_to_pwm(&self, command: f32) -> u16 {
        let pwm_thrust_max =
            self.mot_pwm_min + self.mot_spin_max * (self.mot_pwm_max - self.mot_pwm_min);
        let pwm_thrust_min =
            self.mot_pwm_min + self.mot_spin_min * (self.mot_pwm_max - self.mot_pwm_min);
        let pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
        let cmd = constrain(command, 0.0, 1.0);
        (pwm_thrust_min + cmd * pwm_thrust_range + 0.5) as u16
    }

    pub fn calc_thrust(
        &self,
        command: f32,
        air_density: f32,
        velocity_in: f32,
        voltage_scale: f32,
    ) -> f32 {
        let velocity_out = voltage_scale
            * self.max_outflow_velocity
            * ((1.0 - self.mot_expo) * command + self.mot_expo * command * command).sqrt();
        0.5 * air_density
            * self.effective_prop_area
            * (velocity_out * velocity_out - velocity_in * velocity_in)
    }
}
