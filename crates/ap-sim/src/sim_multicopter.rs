//! COP-031: port of libraries/SITL/SIM_Multicopter.h/.cpp via C++
//! `fwcpp::sim::SimMulticopter` (CCP-045).
//!
//! Inherits the Aircraft rigid-body integrator (`update_dynamics` /
//! `on_ground` / `hagl` / `NoMovement` ground behaviour) used by C++
//! `SimMulticopter : public Aircraft`. Plant is Frame/Motor mixing, not
//! kinematic [`crate::AttitudeSim`] and not [`crate::sim_plane::SimPlane`].

#![allow(missing_docs)]

use crate::sim_frame::Frame;
use crate::sim_motor::SitlInput;
use crate::sim_plane::{GroundBehavior, Mat3, Vec3, GRAVITY_MSS};

// `constrain` is crate-private in sim_plane; duplicate the clamp used by
// Aircraft::update_dynamics.
fn clamp(v: f32, min: f32, max: f32) -> f32 {
    v.clamp(min, max)
}

fn radians(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

/// Ground-truth multicopter plant. Upstream `SITL::MultiCopter`.
pub struct SimMulticopter {
    pub frame: Frame,
    pub dcm: Mat3,
    pub gyro: Vec3,
    pub accel_body: Vec3,
    pub velocity_ef: Vec3,
    pub velocity_air_ef: Vec3,
    pub velocity_air_bf: Vec3,
    pub position: Vec3,
    pub wind_ef: Vec3,
    pub mass: f32,
    pub ground_level: f32,
    pub frame_height: f32,
    pub home_alt_m: f32,
    pub home_alt_amsl_m: f32,
    pub ground_behavior: GroundBehavior,
    pub battery_voltage: f32,
    pub time_now_us: u64,
    pub mag_field_bf: Vec3,
    pub home_lat_e7: i32,
    pub home_lng_e7: i32,
    pub location_lat_e7: i32,
    pub location_lng_e7: i32,
    pub location_alt_cm: i32,
}

impl Default for SimMulticopter {
    fn default() -> Self {
        Self::new("x")
    }
}

impl SimMulticopter {
    pub fn new(frame_str: &str) -> Self {
        let mut frame = Frame::create_frame(frame_str);
        if !frame.valid() {
            frame = Frame::create_frame("x");
        }
        let name = frame.name;
        frame.init(frame_str);
        let mass = frame.get_mass();
        let batt = frame.battery_voltage();
        Self {
            frame,
            dcm: Mat3::identity(),
            gyro: Vec3::zero(),
            accel_body: Vec3::zero(),
            velocity_ef: Vec3::zero(),
            velocity_air_ef: Vec3::zero(),
            velocity_air_bf: Vec3::zero(),
            position: Vec3::zero(),
            wind_ef: Vec3::zero(),
            mass,
            ground_level: 0.0,
            frame_height: 0.0,
            home_alt_m: 0.0,
            home_alt_amsl_m: 0.0,
            ground_behavior: GroundBehavior::NoMovement,
            battery_voltage: batt,
            time_now_us: 0,
            mag_field_bf: Vec3::zero(),
            home_lat_e7: -353_632_621,
            home_lng_e7: 1_491_652_374,
            location_lat_e7: -353_632_621,
            location_lng_e7: 1_491_652_374,
            location_alt_cm: 0,
        }
        .with_name_note(name)
    }

    fn with_name_note(self, _name: &'static str) -> Self {
        self
    }

    pub fn num_motors(&self) -> u8 {
        self.frame.num_motors
    }
    pub fn hover_thr_out(&self) -> f32 {
        self.frame.hover_thr_out()
    }
    pub fn hover_command(&self) -> f32 {
        self.frame.hover_command()
    }
    pub fn command_to_pwm(&self, command: f32) -> u16 {
        self.frame.command_to_pwm(command)
    }
    pub fn set_equal_command(&self, input: &mut SitlInput, command: f32) {
        self.frame.set_equal_command(input, command);
    }

    pub fn hagl(&self) -> f32 {
        (-self.position.z) + self.home_alt_m - self.ground_level - self.frame_height
    }

    pub fn on_ground(&self) -> bool {
        self.hagl() <= 0.001
    }

    pub fn altitude_m(&self) -> f32 {
        -self.position.z
    }

    pub fn true_euler_deg(&self) -> (f32, f32, f32) {
        let (r, p, y) = self.dcm.to_euler();
        (
            r * 180.0 / core::f32::consts::PI,
            p * 180.0 / core::f32::consts::PI,
            y * 180.0 / core::f32::consts::PI,
        )
    }

    pub fn calculate_forces(&mut self, input: &SitlInput) -> (Vec3, Vec3) {
        self.mass = self.frame.get_mass();
        let alt_amsl = if self.home_alt_amsl_m != 0.0 {
            self.home_alt_amsl_m
        } else {
            self.location_alt_cm as f32 * 0.01
        };
        self.frame.set_battery_voltage(self.battery_voltage);
        let (rot_accel, body_acc) = self.frame.calculate_forces(
            self.dcm,
            self.velocity_air_ef,
            self.gyro,
            alt_amsl,
            input,
            self.mass,
            true,
            self.time_now_us,
        );
        (rot_accel, body_acc)
    }

    fn apply_ground_behavior(&mut self) {
        if !self.on_ground() {
            return;
        }
        self.position.z = -(self.ground_level + self.frame_height - self.home_alt_m);
        match self.ground_behavior {
            GroundBehavior::None | GroundBehavior::Tailsitter => {}
            GroundBehavior::NoMovement => {
                let (_r, _p, y) = self.dcm.to_euler();
                self.dcm = Mat3::from_euler(0.0, 0.0, y);
                self.velocity_ef.x = 0.0;
                self.velocity_ef.y = 0.0;
                if self.velocity_ef.z > 0.0 {
                    self.velocity_ef.z = 0.0;
                }
                self.gyro = Vec3::zero();
            }
            GroundBehavior::FwdOnly => {
                let (_r, mut p, y) = self.dcm.to_euler();
                if self.velocity_ef.length() < 5.0 {
                    p = 0.0;
                } else {
                    p = p.max(0.0);
                }
                self.dcm = Mat3::from_euler(0.0, p, y);
                let mut v_bf = self.dcm.transposed().apply(self.velocity_ef);
                v_bf.y = 0.0;
                if v_bf.x < 0.0 {
                    v_bf.x = 0.0;
                }
                self.velocity_ef = self.dcm.apply(v_bf);
                if self.velocity_ef.z > 0.0 {
                    self.velocity_ef.z = 0.0;
                }
                self.gyro = Vec3::zero();
            }
        }
    }

    /// Upstream `Aircraft::update_dynamics`.
    pub fn update_dynamics(&mut self, rot_accel: Vec3, dt: f32) {
        self.gyro = self.gyro.plus(rot_accel.scaled(dt));
        let gyro_lim = radians(2000.0);
        self.gyro.x = clamp(self.gyro.x, -gyro_lim, gyro_lim);
        self.gyro.y = clamp(self.gyro.y, -gyro_lim, gyro_lim);
        self.gyro.z = clamp(self.gyro.z, -gyro_lim, gyro_lim);

        let accel_limit = 64.0 * GRAVITY_MSS;
        self.accel_body.x = clamp(self.accel_body.x, -accel_limit, accel_limit);
        self.accel_body.y = clamp(self.accel_body.y, -accel_limit, accel_limit);
        self.accel_body.z = clamp(self.accel_body.z, -accel_limit, accel_limit);

        self.dcm.rotate(self.gyro.scaled(dt));
        self.dcm.normalize();

        let mut accel_earth = self.dcm.apply(self.accel_body);
        accel_earth = accel_earth.plus(Vec3::new(0.0, 0.0, GRAVITY_MSS));

        if self.on_ground() && accel_earth.z > 0.0 {
            accel_earth.z = 0.0;
        }

        self.accel_body =
            self.dcm
                .transposed()
                .apply(accel_earth.plus(Vec3::new(0.0, 0.0, -GRAVITY_MSS)));

        self.velocity_ef = self.velocity_ef.plus(accel_earth.scaled(dt));
        self.position = self.position.plus(self.velocity_ef.scaled(dt));

        self.velocity_air_ef = self.velocity_ef.minus(self.wind_ef);
        self.velocity_air_bf = self.dcm.transposed().apply(self.velocity_air_ef);

        if self.on_ground() {
            self.apply_ground_behavior();
            if self.on_ground() && self.velocity_ef.z > 0.0 {
                self.velocity_ef.z = 0.0;
            }
        }
    }

    pub fn update_position(&mut self) {
        // NED metres -> lat/lng e7 + alt cm. Flat-earth as in C++ SITL harness.
        const M_PER_DEG: f32 = 111_320.0;
        let north = self.position.x;
        let east = self.position.y;
        let dlat = north / M_PER_DEG;
        let lat = self.home_lat_e7 as f32 * 1.0e-7 + dlat;
        let cos_lat = (lat * core::f32::consts::PI / 180.0).cos().max(0.1);
        let dlng = east / (M_PER_DEG * cos_lat);
        self.location_lat_e7 = (lat * 1.0e7) as i32;
        self.location_lng_e7 = (self.home_lng_e7 as f32 + dlng * 1.0e7) as i32;
        self.location_alt_cm = ((self.home_alt_m - self.position.z) * 100.0) as i32;
    }

    pub fn update_mag_field_bf(&mut self) {
        // Earth field NED milligauss, rotated into body (C++ Aircraft role).
        let earth = Vec3::new(230.0, 50.0, 450.0);
        self.mag_field_bf = self.dcm.transposed().apply(earth);
    }

    pub fn get_mag_field_bf(&self) -> Vec3 {
        self.mag_field_bf
    }

    /// Upstream `MultiCopter::update`.
    pub fn update(&mut self, input: &SitlInput, dt: f32) {
        self.mass = self.frame.get_mass();
        // Wind left at zero unless a caller wrote `wind_ef` (C++ leftover
        // copter_sitl_run never sets SIM wind).
        let (rot_accel, body_acc) = self.calculate_forces(input);
        self.accel_body = body_acc;
        self.update_dynamics(rot_accel, dt);
        self.time_now_us = self.time_now_us.saturating_add((dt * 1.0e6) as u64);
        self.update_position();
        self.update_mag_field_bf();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim_frame::Frame;
    use crate::sim_motor::{MOTORS_YAW_FACTOR_CCW, MOTORS_YAW_FACTOR_CW};

    #[test]
    fn create_frame_matches_original_sitl_name_table() {
        let x = Frame::create_frame("x");
        assert!(x.valid());
        assert_eq!(x.num_motors, 4);
        assert_eq!(x.name, "x");

        let plus = Frame::create_frame("quad");
        assert!(plus.valid());
        assert_eq!(plus.name, "quad");
        assert_eq!(plus.num_motors, 4);

        let hexa = Frame::create_frame("hexa");
        assert!(hexa.valid());
        assert_eq!(hexa.num_motors, 6);

        let octa = Frame::create_frame("octa-quad");
        assert!(octa.valid());
        assert_eq!(octa.num_motors, 8);

        let missing = Frame::create_frame("not-a-frame");
        assert!(!missing.valid());
    }

    #[test]
    fn quad_x_motor_angles_and_yaw_factors_match_sim_frame() {
        let x = Frame::create_frame("x");
        let m = x.motors();
        assert!((m[0].angle - 45.0).abs() < 1e-4);
        assert!((m[0].yaw_factor - MOTORS_YAW_FACTOR_CCW).abs() < 1e-6);
        assert!((m[1].angle - (-135.0)).abs() < 1e-4);
        assert!((m[1].yaw_factor - MOTORS_YAW_FACTOR_CCW).abs() < 1e-6);
        assert!((m[2].angle - (-45.0)).abs() < 1e-4);
        assert!((m[2].yaw_factor - MOTORS_YAW_FACTOR_CW).abs() < 1e-6);
        assert!((m[3].angle - 135.0).abs() < 1e-4);
        assert!((m[3].yaw_factor - MOTORS_YAW_FACTOR_CW).abs() < 1e-6);
    }

    #[test]
    fn equal_hover_command_produces_near_1g_body_z_thrust() {
        let mut copter = SimMulticopter::new("x");
        copter.position.z = -10.0;
        copter.velocity_ef = Vec3::zero();
        copter.home_alt_amsl_m = copter.frame.get_model().ref_alt;
        let mut input = SitlInput::default();
        let hover = copter.hover_command();
        copter.set_equal_command(&mut input, hover);
        let (rot, body) = copter.calculate_forces(&input);
        assert!((body.z + GRAVITY_MSS).abs() < 1.5, "body.z={}", body.z);
        assert!(body.x.abs() < 0.5);
        assert!(body.y.abs() < 0.5);
        assert!(rot.x.abs() < 0.5);
        assert!(rot.y.abs() < 0.5);
    }

    #[test]
    fn zero_pwm_stays_on_the_ground() {
        let mut copter = SimMulticopter::new("x");
        assert!(copter.on_ground());
        let input = SitlInput::default();
        let dt = 0.0025_f32;
        for _ in 0..400 {
            copter.update(&input, dt);
        }
        assert!(copter.on_ground());
        assert!(copter.position.z.abs() < 0.05);
    }

    #[test]
    fn climb_command_leaves_the_ground() {
        let mut copter = SimMulticopter::new("x");
        let mut input = SitlInput::default();
        copter.set_equal_command(&mut input, 0.70);
        let dt = 0.0025_f32;
        for _ in 0..1200 {
            copter.update(&input, dt);
        }
        assert!(-copter.position.z > 2.0, "alt={}", -copter.position.z);
        assert!(!copter.on_ground());
    }

    #[test]
    fn differential_thrust_on_left_motors_produces_positive_roll() {
        let mut copter = SimMulticopter::new("x");
        copter.position.z = -10.0;
        let mut input = SitlInput::default();
        let high = copter.command_to_pwm(0.70);
        let low = copter.command_to_pwm(0.20);
        input.servos[0] = low;
        input.servos[1] = high;
        input.servos[2] = high;
        input.servos[3] = low;
        let (rot, _body) = copter.calculate_forces(&input);
        assert!(rot.x > 1.0, "rot.x={}", rot.x);
        assert!(rot.y.abs() < rot.x.abs());
    }

    #[test]
    fn differential_thrust_on_rear_motors_produces_positive_pitch() {
        let mut copter = SimMulticopter::new("x");
        copter.position.z = -10.0;
        let mut input = SitlInput::default();
        let high = copter.command_to_pwm(0.70);
        let low = copter.command_to_pwm(0.20);
        input.servos[0] = high;
        input.servos[1] = low;
        input.servos[2] = high;
        input.servos[3] = low;
        let (rot, _body) = copter.calculate_forces(&input);
        assert!(rot.y > 1.0, "rot.y={}", rot.y);
        assert!(rot.x.abs() < rot.y.abs());
    }

    #[test]
    fn ccw_vs_cw_command_imbalance_produces_yaw() {
        let mut copter = SimMulticopter::new("x");
        copter.position.z = -10.0;
        let mut input = SitlInput::default();
        let high = copter.command_to_pwm(0.70);
        let low = copter.command_to_pwm(0.20);
        input.servos[0] = high;
        input.servos[1] = high;
        input.servos[2] = low;
        input.servos[3] = low;
        let (rot, _body) = copter.calculate_forces(&input);
        assert!(rot.z.abs() > 0.5, "rot.z={}", rot.z);
    }

    #[test]
    fn integrated_differential_roll_banks_the_rigid_body() {
        let mut copter = SimMulticopter::new("x");
        copter.position.z = -20.0;
        let mut input = SitlInput::default();
        let high = copter.command_to_pwm(0.65);
        let low = copter.command_to_pwm(0.25);
        input.servos[0] = low;
        input.servos[1] = high;
        input.servos[2] = high;
        input.servos[3] = low;
        let dt = 0.0025_f32;
        for _ in 0..80 {
            copter.update(&input, dt);
        }
        let (r, _p, _y) = copter.dcm.to_euler();
        assert!(r > 0.05, "roll={r}");
    }

    #[test]
    fn plus_vs_x_frames_have_different_motor_angles() {
        let plus = Frame::create_frame("+");
        let x = Frame::create_frame("x");
        assert!((plus.motors()[0].angle - 90.0).abs() < 1e-4);
        assert!((x.motors()[0].angle - 45.0).abs() < 1e-4);
    }
}
