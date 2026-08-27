//! Roll/pitch/yaw controller hookup for the main vehicle loop.
//!
//! Upstream `Plane::stabilize` calls the three attitude controllers when the
//! active mode's `run()` selected them; `set_servos` publishes the scaled
//! surface demands that result.

use ap_control::{
    PitchController, PitchInputs, RateGains, RollController, RollInputs, SideslipInputs,
    YawController, YawGains,
};
use ap_ins::ImuInstance;
use ap_math::scalar::cd_to_rad;
use ap_pid::PidGains;

use crate::ahrs_hookup::AhrsAttitude;
use crate::main_loop::{StabilizeDispatch, StabilizeRun};
use crate::{PitchDemand, RollDemand};

/// Attitude controllers the vehicle loop owns, upstream `rollController` etc.
#[derive(Debug, Clone, Copy)]
pub struct StabilizeControllers {
    /// Roll axis, upstream `rollController`.
    pub roll: RollController,
    /// Pitch axis, upstream `pitchController`.
    pub pitch: PitchController,
    /// Yaw axis, upstream `yawController`.
    pub yaw: YawController,
}

impl Default for StabilizeControllers {
    fn default() -> Self {
        let pid = PidGains::default();
        let rate = RateGains::default();
        Self {
            roll: RollController::new(pid, rate),
            pitch: PitchController::new(pid, rate, 1.0),
            yaw: YawController::new(YawGains::default(), pid),
        }
    }
}

/// Navigation and trim inputs the stabilize path reads.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StabilizeDemands {
    /// Upstream `Plane::nav_roll_cd`.
    pub nav_roll_cd: i32,
    /// Upstream `Plane::nav_pitch_cd`.
    pub nav_pitch_cd: i32,
    /// Upstream `pitch_trim_cd`.
    pub pitch_trim_cd: i32,
    /// Throttle output scaled 0..100, upstream `channel_throttle->get_servo_out()`.
    pub throttle_scaled: f32,
    /// Upstream `KFF_THR2PTCH`.
    pub kff_throttle_to_pitch: f32,
    /// Current roll limit, centidegrees. Upstream `roll_limit_cd`.
    pub roll_limit_cd: i32,
    /// Current pitch floor, centidegrees. Upstream `pitch_limit_min`.
    pub pitch_limit_min_cd: i32,
    /// Current pitch ceiling, centidegrees. Upstream `pitch_limit_max`.
    pub pitch_limit_max_cd: i32,
}

/// Vehicle context passed into the controllers each loop.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StabilizeContext {
    /// Airspeed scaling factor, upstream `get_speed_scaler()`.
    pub scaler: f32,
    /// Upstream `disable_integrator`.
    pub disable_integrator: bool,
    /// Upstream `ground_mode`.
    pub ground_mode: bool,
    /// Equivalent airspeed, m/s, or `None` when unavailable.
    pub airspeed_eas: Option<f32>,
    /// Upstream `aparm.airspeed_min`.
    pub airspeed_min: i16,
    /// Upstream `aparm.airspeed_max`.
    pub airspeed_max: i16,
    /// Upstream `aparm.roll_limit`.
    pub roll_limit_deg: f32,
    /// Upstream `AP::ahrs().get_EAS2TAS()`.
    pub eas2tas: f32,
    /// Lateral accelerometer bias, m/s². Upstream `ahrs.get_accel_bias().y`.
    pub accel_bias_y: f32,
    /// Milliseconds since boot.
    pub now_ms: u32,
}

/// Scaled surface demands from the last stabilize pass.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StabilizeServoDemands {
    /// Aileron demand, scaled −4500..4500 centidegrees.
    pub aileron_scaled: f32,
    /// Elevator demand, scaled −4500..4500 centidegrees.
    pub elevator_scaled: f32,
    /// Rudder demand, scaled −4500..4500 centidegrees.
    pub rudder_scaled: f32,
}

/// Result of one stabilize pass.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StabilizeOutputs {
    /// Which attitude paths ran.
    pub run: StabilizeRun,
    /// Surface demands written for `set_servos`.
    pub servos: StabilizeServoDemands,
}

/// One stabilize tick: honour dispatch flags and call `servo_out`.
#[must_use]
pub fn stabilize_controllers(
    controllers: &mut StabilizeControllers,
    attitude: &AhrsAttitude,
    imu: &ImuInstance,
    dispatch: StabilizeDispatch,
    demands: &StabilizeDemands,
    ctx: &StabilizeContext,
    dt: f32,
) -> StabilizeOutputs {
    let gyro = imu.gyro();
    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream promotes int32 attitude sensors to float the same way"
    )]
    let roll_rad = cd_to_rad(attitude.roll_sensor_cd as f32);
    let pitch_rad = cd_to_rad(attitude.pitch_sensor_cd as f32);

    let mut servos = StabilizeServoDemands::default();
    let mut run = StabilizeRun::default();

    if dispatch.roll {
        run.roll = true;
        let demand = RollDemand::from_navigation(demands.nav_roll_cd, demands.roll_limit_cd);
        let err = demand.angle_error_cd(attitude.roll_sensor_cd);
        let inp = RollInputs {
            scaler: ctx.scaler,
            disable_integrator: ctx.disable_integrator,
            ground_mode: ctx.ground_mode,
            roll_rate_rad: gyro.x,
            airspeed_eas: ctx.airspeed_eas,
            airspeed_min: ctx.airspeed_min,
            eas2tas: ctx.eas2tas,
            dt,
            now_ms: ctx.now_ms,
        };
        servos.aileron_scaled = controllers.roll.servo_out(err, &inp);
    }

    if dispatch.pitch {
        run.pitch = true;
        let demand = PitchDemand::from_tecs(
            demands.nav_pitch_cd,
            demands.pitch_limit_min_cd,
            demands.pitch_limit_max_cd,
        );
        let demanded = demand.demanded_pitch_cd(
            demands.pitch_trim_cd,
            demands.throttle_scaled,
            demands.kff_throttle_to_pitch,
        );
        let err = PitchDemand::angle_error_cd(demanded, attitude.pitch_sensor_cd);
        let inp = PitchInputs {
            scaler: ctx.scaler,
            disable_integrator: ctx.disable_integrator,
            ground_mode: ctx.ground_mode,
            pitch_rate_rad: gyro.y,
            airspeed_eas: ctx.airspeed_eas,
            airspeed_min: ctx.airspeed_min,
            airspeed_max: ctx.airspeed_max,
            roll_limit_deg: ctx.roll_limit_deg,
            roll_rad,
            pitch_rad,
            roll_sensor_cd: attitude.roll_sensor_cd,
            pitch_sensor_cd: attitude.pitch_sensor_cd,
            eas2tas: ctx.eas2tas,
            dt,
            now_ms: ctx.now_ms,
        };
        servos.elevator_scaled = controllers.pitch.servo_out(err, &inp);
    }

    if dispatch.yaw {
        run.yaw = true;
        let inp = SideslipInputs {
            scaler: ctx.scaler,
            disable_integrator: ctx.disable_integrator,
            now_ms: ctx.now_ms,
            roll_rad,
            airspeed_eas: ctx.airspeed_eas,
            airspeed_min: ctx.airspeed_min,
            airspeed_max: ctx.airspeed_max,
            yaw_rate_rad: gyro.z,
            accel_y: imu.accel().y,
            accel_bias_y: ctx.accel_bias_y,
        };
        servos.rudder_scaled = controllers.yaw.servo_out(&inp) as f32;
    }

    StabilizeOutputs { run, servos }
}

/// Convert a scaled centidegree demand to default PWM, upstream trim at 1500 µs.
#[must_use]
#[allow(
    clippy::cast_possible_truncation,
    reason = "pwm is clamped to 1000..2000 before truncation"
)]
pub fn scaled_to_pwm_trim(scaled: f32) -> u16 {
    let pwm = 1500.0 + scaled / 4500.0 * 500.0;
    pwm.clamp(1000.0, 2000.0) as u16
}

/// Publish stabilize demands into the vehicle servo output state.
pub fn apply_stabilize_to_servos(
    stabilize: &StabilizeServoDemands,
    servos: &mut crate::landing_hookup::ServoOutputState,
) {
    servos.aileron_scaled = stabilize.aileron_scaled;
    servos.rudder_scaled = stabilize.rudder_scaled;
    servos.elevator_pwm = scaled_to_pwm_trim(stabilize.elevator_scaled);
}
