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

    /// Upstream `SITL::QuadPlane::copter_tailsitter` (real
    /// `SIM_QuadPlane.cpp` line 80-82: set true only for the
    /// `-copter_tailsitter` frame-string suffix). Drives the
    /// `ROTATION_PITCH_270`-equivalent rotation applied to `quad_rot`/
    /// `quad_accel` in [`Self::update`] (real lines 132-135).
    pub fn copter_tailsitter(&self) -> bool {
        self.copter_tailsitter
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

    // VT-013: prove `SimQuadPlane::new`'s own `frame.init(frame_str);`
    // call (this file, line 100) genuinely reaches `Frame::init`'s
    // already-fixed (COP-035) `:model.json` suffix mechanism with the
    // JSON suffix intact, end to end -- i.e. that composing a
    // `SimQuadPlane` with a real JSON-suffixed frame_str like
    // `"quadplane:<path-to-Callisto.json>"` loads the real, byte-for-byte
    // upstream `Tools/autotest/models/Callisto.json` fixture (reusing the
    // exact `fixtures/Callisto.json` path COP-035 already established --
    // not a copy) into the resulting quadplane's own `frame` field,
    // exactly as COP-035 already proved for a plain, standalone `Frame`.
    //
    // Traced directly rather than assumed: none of this file's own
    // frame-type substring checks (lines 45-86: `-octa*`, `-hexa*`,
    // `-plus`, `-y6`, `-tri*`, `-tilt*`, `firefly`, `cl84`,
    // `-copter_tailsitter`) match any text in `"quadplane:"` plus the
    // fixture's absolute path, so `frame_type` stays the default `"x"`
    // and `Frame::create_frame("x")` builds a valid quad-X frame that
    // `Frame::init` then loads the real Callisto model into -- this
    // composition already worked correctly before this ticket; no
    // production code changed.
    //
    // Does NOT forward `-heavy`/`-jet`: real upstream `SIM_QuadPlane.cpp`
    // (`QuadPlane::QuadPlane`, real lines 25-27) inherits `Plane`'s own
    // `-heavy`/`-jet` handling via the `Plane(frame_str)` base-class call,
    // but real line 107 then does `mass = frame->get_mass() * 1.5;`
    // unconditionally, right after `frame->init(frame_str, &battery)` at
    // real line 104 -- discarding whatever `-heavy`/`-jet` set on `mass`.
    // Forwarding those flags to this port's own embedded `plane` field
    // would have no observable effect on `mass`, matching real upstream's
    // own behavior, so this test (and this ticket) does not add that
    // forwarding.
    #[test]
    fn new_with_json_suffixed_frame_str_loads_real_callisto_fixture() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("fixtures/Callisto.json"))
            .expect("workspace root");
        let frame_str = format!("quadplane:{}", path.display());

        let qp = SimQuadPlane::new(&frame_str);

        let m = qp.frame.get_model();
        assert!((m.mass - 32.5).abs() < 1e-4, "mass={}", m.mass);
        assert!(
            (m.num_motors - 8.0).abs() < 1e-4,
            "num_motors={}",
            m.num_motors
        );
        assert!(
            (m.disc_area - 1.82).abs() < 1e-4,
            "disc_area={}",
            m.disc_area
        );
        assert!(
            (m.hover_thr_out - 0.36).abs() < 1e-4,
            "hoverThrOut={}",
            m.hover_thr_out
        );

        // The composed `mass_scale` (`SimQuadPlane::new`'s own
        // `frame.set_mass_scale(1.5);`, this file's line 99) is this
        // port's equivalent of real upstream's post-init
        // `mass = frame->get_mass() * 1.5;` -- so `get_mass()` (which
        // folds in `mass_scale`) is the real fixture's mass times 1.5,
        // not the raw fixture value asserted above via `get_model()`.
        assert!(
            (qp.frame.get_mass() - 32.5 * 1.5).abs() < 1e-3,
            "get_mass()={}",
            qp.frame.get_mass()
        );
        assert!(
            (qp.plane.mass - 32.5 * 1.5).abs() < 1e-3,
            "plane.mass={}",
            qp.plane.mass
        );
    }
}
