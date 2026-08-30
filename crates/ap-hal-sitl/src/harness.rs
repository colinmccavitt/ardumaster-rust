//! Per-tick SitlHarness: RC in, sensors from SimPlane, scheduler, actuators out.

use core::cell::Cell;

use ap_control::{PitchController, RateGains, RollController, YawController, YawGains};
use ap_hal::time::{Clock, Micros, Millis};
use ap_pid::PidGains;
use ap_plane::stabilize_hookup::StabilizeControllers;
use ap_ins::sitl::{SitlBodyState, SitlImuBackend, SitlInsCluster};
use ap_ins::SitlInsMotorRuntime;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::{plane_fast_tasks, run_scheduler_tick, PlaneMainLoop};
use ap_plane::mode_table::ModeNumber;
use ap_plane::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};
use ap_plane::sitl_gps_hookup::{SitlGpsHookup, SitlGpsTruth};
use ap_plane::sitl_ins_noise_hookup::SitlInsNoiseHookup;
use ap_scheduler::scheduler::{Scheduler, Task};
use ap_sim::sim_plane::{GroundBehavior, SimPlane, Vec3};

/// C++ `fwcpp::vehicle::kServoMax` (4500 centidegrees).
pub const SERVO_MAX: f32 = 4500.0;
/// Tick rate matching C++ sitl_run / the closed-loop test suite.
pub const SITL_LOOP_HZ: u16 = 50;
/// Default SITL home (CMAC), matching C++ `Aircraft` constructor lat/lng e7.
pub const SITL_HOME_LAT_DEG: f32 = -35.363_262;
pub const SITL_HOME_LNG_DEG: f32 = 149.165_24;

/// Simulated microseconds, matching the main_loop tests' `StepClock`.
pub struct StepClock {
    pub us: Cell<u32>,
}

impl StepClock {
    pub fn from_ms(now_ms: u32) -> Self {
        Self {
            us: Cell::new(now_ms.saturating_mul(1000)),
        }
    }
}

impl Clock for StepClock {
    fn millis(&self) -> Millis {
        Millis(self.us.get() / 1000)
    }
    fn micros(&self) -> Micros {
        Micros(self.us.get())
    }
    fn millis64(&self) -> u64 {
        u64::from(self.us.get()) / 1000
    }
    fn micros64(&self) -> u64 {
        u64::from(self.us.get())
    }
}

/// Set RC PWM like C++ `set_sticks(plane, roll, pitch, throttle, rudder)`.
///
/// Must run before [`SitlHarness::step`]: RC is not synthesized from sim
/// truth (a real receiver frame). `has_valid_input` is forced true so the
/// failsafe path does not zero the sticks.
pub fn set_sticks(
    plane: &mut PlaneMainLoop,
    roll_pwm: u16,
    pitch_pwm: u16,
    throttle_pwm: u16,
    rudder_pwm: u16,
) {
    let mut inp = plane.rc_failsafe_inputs;
    inp.roll_pwm = Some(roll_pwm);
    inp.pitch_pwm = Some(pitch_pwm);
    inp.throttle_pwm = Some(throttle_pwm);
    inp.yaw_pwm = Some(rudder_pwm);
    inp.has_valid_input = true;
    plane.rc_failsafe_inputs = inp;
}

fn ned_to_latlng(home_lat: f32, home_lng: f32, north_m: f32, east_m: f32) -> (f32, f32) {
    const M_PER_DEG: f32 = 111_320.0;
    let lat = home_lat + north_m / M_PER_DEG;
    let cos_lat = (lat * core::f32::consts::PI / 180.0).cos().max(0.1);
    let lng = home_lng + east_m / (M_PER_DEG * cos_lat);
    (lat, lng)
}

fn vec3_to_math(v: Vec3) -> Vector3f {
    Vector3f::new(v.x, v.y, v.z)
}

fn degrees(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

/// Reusable SITL closed-loop driver. Does not own Plane/SimPlane — the
/// caller constructs, arms, and picks the mode (same as C++ SitlHarness).
pub struct SitlHarness {
    tasks: [Task<PlaneMainLoop>; 4],
    last_run: [u16; 4],
    loop_rate_hz: u16,
    /// Added to true gyro before INS sees it. Default zero (true sensors).
    pub gyro_bias_rad_s: Vec3,
}

impl SitlHarness {
    pub fn new() -> Self {
        Self {
            tasks: plane_fast_tasks(),
            last_run: [0; 4],
            loop_rate_hz: SITL_LOOP_HZ,
            gyro_bias_rad_s: Vec3::zero(),
        }
    }


    /// ArduPlane-4.7.0 AP_Roll/PitchController AC_PID::Defaults plus
    /// RLL/PTCH2SRV_TCONST = 0.5. PlaneMainLoop default-constructs with
    /// zero PID gains (no AP_Param table), so sitl_run must load these
    /// or stabilize emits zero aileron/elevator forever.
    fn apply_default_fw_gains(plane: &mut PlaneMainLoop) {
        let roll_pid = PidGains {
            p: 0.08,
            i: 0.15,
            d: 0.0,
            ff: 0.345,
            imax: 0.666,
            filt_t_hz: 3.0,
            filt_e_hz: 0.0,
            filt_d_hz: 12.0,
            srmax: 150.0,
            srtau: 1.0,
            ..PidGains::default()
        };
        let pitch_pid = PidGains {
            p: 0.04,
            i: 0.15,
            d: 0.0,
            ff: 0.345,
            imax: 0.666,
            filt_t_hz: 3.0,
            filt_e_hz: 0.0,
            filt_d_hz: 12.0,
            srmax: 150.0,
            srtau: 1.0,
            ..PidGains::default()
        };
        let rate = RateGains {
            tau: 0.5,
            rmax_pos: 0.0,
            rmax_neg: 0.0,
        };
        plane.controllers = StabilizeControllers {
            roll: RollController::new(roll_pid, rate),
            pitch: PitchController::new(pitch_pid, rate, 1.0),
            yaw: YawController::new(YawGains::default(), PidGains::default()),
        };
    }

    /// Wire the SITL sensor hookups PlaneMainLoop already has, zero INS
    /// vibration, and apply the FBWA LIM_ROLL/LIM_PITCH defaults the C++
    /// vehicle carries as AP_Param defaults (4500 / +2000 / -2500 cd).
    pub fn configure_vehicle(plane: &mut PlaneMainLoop) {
        Self::apply_default_fw_gains(plane);
        // One IMU sample per vehicle tick (50 Hz), matching C++ SitlHarness
        // feeding a single GyroSample/accel per step rather than a 1 kHz
        // backend that would be starved at this loop rate.
        let mut cluster = SitlInsCluster::new();
        cluster
            .register(SitlImuBackend::new(u16::from(SITL_LOOP_HZ), u16::from(SITL_LOOP_HZ)))
            .expect("SITL IMU register");
        let mut noise = SitlInsNoiseHookup {
            cluster,
            noise_params: Default::default(),
            file_playback_params: Default::default(),
        };
        noise.noise_params.motor_accel_noise = 0.0;
        noise.noise_params.motor_gyro_noise_deg = 0.0;
        plane.sitl_ins_noise = Some(noise);
        plane.sitl_gps = Some(SitlGpsHookup::default());
        plane.sitl_baro = Some(SitlBaroHookup::default());
        plane.sitl_compass = Some(SitlCompassHookup::default());
        plane.sitl_airspeed = Some(SitlAirspeedHookup::default());
        plane.stabilize_demands.roll_limit_cd = 4500;
        plane.stabilize_demands.pitch_limit_max_cd = 2000;
        plane.stabilize_demands.pitch_limit_min_cd = -2500;
        // ArduPlane SCALING_SPEED / ARSPD_FBW_MIN / ARSPD_FBW_MAX defaults.
        // Pitch turn-coordination divides by max(V, airspeed_min); leaving
        // these at zero makes V_floor=1 m/s and demands ~50 deg/s nose-up
        // at a 17 deg FBWA bank.
        plane.speed_scaler_inputs.scaling_speed = 15.0;
        plane.speed_scaler_inputs.airspeed_min = 9.0;
        plane.speed_scaler_inputs.airspeed_max = 22.0;
        plane.speed_scaler_inputs.armed = true;
        plane.stabilize_ctx.airspeed_min = 9;
        plane.stabilize_ctx.airspeed_max = 22;
        plane.stabilize_ctx.roll_limit_deg = 45.0;
        plane.stabilize_ctx.eas2tas = 1.0;
        plane.loop_timing.delta_time = 1.0 / f32::from(SITL_LOOP_HZ);
        plane.loop_timing.loop_delta_t_max = 10.0 / f32::from(SITL_LOOP_HZ);
        plane.home_altitude_m = 0.0;
        if let Some(gps) = plane.sitl_gps.as_mut() {
            gps.set_lag_sec(0.0);
            gps.fly_forward = true;
            gps.truth.latitude_deg = SITL_HOME_LAT_DEG;
            gps.truth.longitude_deg = SITL_HOME_LNG_DEG;
        }
        if let Some(compass) = plane.sitl_compass.as_mut() {
            compass.truth.latitude_deg = SITL_HOME_LAT_DEG;
            compass.truth.longitude_deg = SITL_HOME_LNG_DEG;
        }
    }

    /// FBWA + armed + safety-equivalent of C++ sitl_run setup.
    ///
    /// C++ does not call `plane.arm()` (its rc_received gate would fail
    /// before set_sticks). We set `soft_armed` directly, matching
    /// `plane.armed = true` + `force_safety_off()`.
    pub fn setup_fbwa(plane: &mut PlaneMainLoop, sim: &mut SimPlane) {
        Self::configure_vehicle(plane);
        plane.mode.control_mode = ModeNumber::FlyByWireA.as_number();
        plane.soft_armed = true;
        plane.airspeed_calibrate_requested = true;
        // Match C++ sitl_run: SimPlane default ground_behavior is kNone.
        // FWD_ONLY zeros gyro and levels the wings on the runway, so the
        // FBWA aileron demand is held until liftoff and then applied as a
        // snap. C++ CPP-085 verification (17 deg bank, climb) used kNone.
        sim.ground_behavior = GroundBehavior::None;
    }

    fn publish_sensors(plane: &mut PlaneMainLoop, sim: &SimPlane, now_ms: u32, gyro_bias: Vec3) {
        let measured_gyro = sim.gyro.plus(gyro_bias);
        plane.sitl_body = SitlBodyState {
            roll_rate_dps: degrees(measured_gyro.x),
            pitch_rate_dps: degrees(measured_gyro.y),
            yaw_rate_dps: degrees(measured_gyro.z),
            x_accel: sim.accel_body.x,
            y_accel: sim.accel_body.y,
            z_accel: sim.accel_body.z,
            roll_accel_dps2: 0.0,
            pitch_accel_dps2: 0.0,
            yaw_accel_dps2: 0.0,
        };
        plane.sitl_now_us = u64::from(now_ms) * 1000;
        plane.loop_timing.delta_time = 1.0 / f32::from(SITL_LOOP_HZ);
        plane.yaw_ctx.now_ms = now_ms;
        plane.sitl_ins_motor = SitlInsMotorRuntime {
            motors_on: plane.servos.throttle_scaled > 0.0,
            throttle: plane.servos.throttle_scaled,
            motor_mask: 1,
            motor_rpm: [0.0; 8],
        };

        let alt = sim.altitude_m();
        let (lat, lng) = ned_to_latlng(
            SITL_HOME_LAT_DEG,
            SITL_HOME_LNG_DEG,
            sim.position.x,
            sim.position.y,
        );

        if let Some(gps) = plane.sitl_gps.as_mut() {
            gps.truth = SitlGpsTruth {
                velocity_ned: vec3_to_math(sim.velocity_ef),
                latitude_deg: lat,
                longitude_deg: lng,
                altitude_m: alt,
                now_ms,
            };
        }
        if let Some(baro) = plane.sitl_baro.as_mut() {
            baro.truth = SitlBaroTruth {
                sim_altitude_m: alt,
                airspeed_bf: vec3_to_math(sim.velocity_air_bf),
                now_ms,
                noise_sample: 0.0,
            };
        }
        if let Some(compass) = plane.sitl_compass.as_mut() {
            compass.truth = SitlCompassTruth {
                latitude_deg: lat,
                longitude_deg: lng,
                now_ms,
            };
            compass.body_attitude_override = Some(Matrix3f::from_rows(
                Vector3f::new(sim.dcm.a.x, sim.dcm.a.y, sim.dcm.a.z),
                Vector3f::new(sim.dcm.b.x, sim.dcm.b.y, sim.dcm.b.z),
                Vector3f::new(sim.dcm.c.x, sim.dcm.c.y, sim.dcm.c.z),
            ));
        }
        if let Some(airspeed) = plane.sitl_airspeed.as_mut() {
            airspeed.truth = SitlAirspeedTruth {
                airspeed_bf: vec3_to_math(sim.velocity_air_bf),
                now_ms,
            };
        }

        // Position/altitude the navigation modes read. C++ SitlHarness:
        // `in.position_ned = sim.position; in.current_altitude_m = -z`.
        plane.relative_altitude_m = alt;

        plane.speed_scaler_inputs.airspeed_eas = Some(sim.airspeed);
        plane.speed_scaler_inputs.armed = plane.soft_armed;
        plane.speed_scaler_inputs.throttle_scaled = plane.servos.throttle_scaled;
        plane.stabilize_ctx.ground_mode = sim.on_ground();
        plane.stabilize_ctx.airspeed_eas = Some(sim.airspeed);

    }

    fn read_actuators(plane: &PlaneMainLoop) -> (f32, f32, f32, f32) {
        let aileron = plane.servos.aileron_scaled / SERVO_MAX;
        let elevator = plane.stabilize_servos.elevator_scaled / SERVO_MAX;
        let rudder = plane.servos.rudder_scaled / SERVO_MAX;
        let throttle = plane.servos.throttle_scaled / 100.0;
        (
            aileron.clamp(-1.0, 1.0),
            elevator.clamp(-1.0, 1.0),
            rudder.clamp(-1.0, 1.0),
            throttle.clamp(0.0, 1.0),
        )
    }

    /// One closed-loop tick. Caller must have called [`set_sticks`] first.
    ///
    /// Order matches C++ `SitlHarness::step`: synthesize from the CURRENT
    /// sim state (previous update, or the initial rest state), run the
    /// vehicle scheduler, then `sim.update` with the new servos.
    pub fn step(&mut self, plane: &mut PlaneMainLoop, sim: &mut SimPlane, now_ms: u32, dt: f32) {
        Self::publish_sensors(plane, sim, now_ms, self.gyro_bias_rad_s);

        let clock = StepClock::from_ms(now_ms);
        let mut scheduler = Scheduler::new(&self.tasks, &[], &mut self.last_run, self.loop_rate_hz);
        let _stats = run_scheduler_tick(plane, &mut scheduler, &clock, 20_000);

        let (aileron, elevator, rudder, throttle) = Self::read_actuators(plane);
        sim.update(aileron, elevator, rudder, throttle, dt);
    }
}

impl Default for SitlHarness {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_sim::sim_plane::SimPlane;

    #[test]
    fn one_tick_is_finite() {
        let mut plane = PlaneMainLoop::default();
        let mut sim = SimPlane::new();
        sim.position = Vec3::new(0.0, 0.0, -100.0);
        SitlHarness::setup_fbwa(&mut plane, &mut sim);
        set_sticks(&mut plane, 1650, 1500, 1700, 1500);
        let mut harness = SitlHarness::new();
        harness.step(&mut plane, &mut sim, 20, 0.02);
        assert!(!sim.dcm.is_nan());
        assert!(!sim.airspeed.is_nan());
        assert!(!plane.roll_rad.is_nan());
    }

    #[test]
    fn set_sticks_marks_valid_input() {
        let mut plane = PlaneMainLoop::default();
        set_sticks(&mut plane, 1650, 1500, 1700, 1500);
        assert!(plane.rc_failsafe_inputs.has_valid_input);
        assert_eq!(plane.rc_failsafe_inputs.roll_pwm, Some(1650));
        assert_eq!(plane.rc_failsafe_inputs.throttle_pwm, Some(1700));
    }

    #[test]
    fn fbwa_closed_loop_takeoff_is_physically_plausible() {
        let mut plane = PlaneMainLoop::default();
        let mut sim = SimPlane::new();
        SitlHarness::setup_fbwa(&mut plane, &mut sim);
        assert_eq!(sim.ground_behavior, GroundBehavior::None);
        let mut harness = SitlHarness::new();
        let dt = 1.0 / f32::from(SITL_LOOP_HZ);
        let mut now_ms = 0_u32;
        for _ in 0..(10 * i32::from(SITL_LOOP_HZ)) {
            now_ms = now_ms.saturating_add(20);
            set_sticks(&mut plane, 1650, 1500, 1700, 1500);
            harness.step(&mut plane, &mut sim, now_ms, dt);
        }
        let (true_roll, true_pitch, _) = sim.true_euler_deg();
        let ahrs_roll = plane.roll_rad * 180.0 / core::f32::consts::PI;
        let ahrs_pitch = plane.pitch_rad * 180.0 / core::f32::consts::PI;
        assert!(
            sim.airspeed > 8.0,
            "airspeed {} m/s after 10s FBWA",
            sim.airspeed
        );
        assert!(
            sim.altitude_m() > 1.0,
            "alt {} m after 10s FBWA",
            sim.altitude_m()
        );
        assert!(
            true_roll.abs() < 40.0,
            "true roll {} deg (inverted?)",
            true_roll
        );
        assert!(
            (ahrs_roll - true_roll).abs() < 25.0,
            "AHRS roll {} vs true {}",
            ahrs_roll,
            true_roll
        );
        assert!(
            (ahrs_pitch - true_pitch).abs() < 25.0,
            "AHRS pitch {} vs true {}",
            ahrs_pitch,
            true_pitch
        );
    }
}
