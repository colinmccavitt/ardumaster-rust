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
use ap_math::scalar::{cd_to_rad, constrain_int32, constrain_value, Real};
use ap_pid::PidGains;

use crate::ahrs_hookup::AhrsAttitude;
use crate::main_loop::{StabilizeDispatch, StabilizeRun};
use crate::mode_run::StickMixing;
use crate::{PitchDemand, RollDemand};

/// Upstream `MIN_AIRSPEED_MIN`.
pub const MIN_AIRSPEED_MIN: f32 = 5.0;
/// Upstream `AP_PLANE_TRIM_THROTTLE_DEFAULT`.
pub const AP_PLANE_TRIM_THROTTLE_DEFAULT: f32 = 45.0;

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

/// Raw navigation outputs before limiting, upstream nav_controller/TECS.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NavCommandInputs {
    pub commanded_roll_cd: i32,
    pub commanded_pitch_cd: i32,
}

/// RC stick inputs for FBW mixing, upstream `norm_input_dz()`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RcStickInputs {
    pub roll_norm_dz: f32,
    pub pitch_norm_dz: f32,
}

/// Parameters for speed scaler computation, upstream `calc_speed_scaler`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SpeedScalerInputs {
    pub airspeed_eas: Option<f32>,
    pub scaling_speed: f32,
    pub airspeed_min: f32,
    pub airspeed_max: f32,
    pub armed: bool,
    pub throttle_scaled: f32,
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

/// Populate nav demands from navigation/TECS commands, upstream
/// `calc_nav_roll` / `calc_nav_pitch`.
pub fn calc_nav_demands(demands: &mut StabilizeDemands, nav: &NavCommandInputs) {
    demands.nav_roll_cd = RollDemand::from_navigation(
        nav.commanded_roll_cd,
        demands.roll_limit_cd,
    )
    .nav_roll_cd;
    demands.nav_pitch_cd = PitchDemand::from_tecs(
        nav.commanded_pitch_cd,
        demands.pitch_limit_min_cd,
        demands.pitch_limit_max_cd,
    )
    .nav_pitch_cd;
}

/// Airspeed-based PID gain scaling, upstream `Plane::calc_speed_scaler`.
#[must_use]
pub fn calc_speed_scaler(inp: &SpeedScalerInputs) -> f32 {
    if let Some(aspeed) = inp.airspeed_eas {
        let airspeed_min = inp.airspeed_min.max(MIN_AIRSPEED_MIN);
        let scale_min = (inp.scaling_speed / (2.0 * inp.airspeed_max)).min(0.5);
        let scale_max = (inp.scaling_speed / (0.7 * airspeed_min)).max(2.0);
        let speed_scaler = if aspeed > 0.000_1 {
            inp.scaling_speed / aspeed
        } else {
            scale_max
        };
        constrain_value(speed_scaler, scale_min, scale_max)
    } else if inp.armed {
        let throttle_out = inp.throttle_scaled.max(1.0);
        let speed_scaler = (AP_PLANE_TRIM_THROTTLE_DEFAULT / throttle_out).sqrt();
        constrain_value(speed_scaler, 0.6, 1.67)
    } else {
        1.0
    }
}

/// Non-linear stick shaping, upstream the roll/pitch prologue in
/// `stabilize_stick_mixing_fbw`.
#[must_use]
pub fn nonlinear_stick_input(norm_dz: f32) -> f32 {
    if norm_dz > 0.5 {
        3.0 * norm_dz - 1.0
    } else if norm_dz < -0.5 {
        3.0 * norm_dz + 1.0
    } else {
        norm_dz
    }
}

/// Whether the aircraft is flying inverted, upstream `fly_inverted()`.
#[must_use]
pub const fn fly_inverted(roll_sensor_cd: i32) -> bool {
    roll_sensor_cd < -9000 || roll_sensor_cd > 9000
}

/// FBW stick mixing into nav demands, upstream `stabilize_stick_mixing_fbw`.
pub fn stabilize_stick_mixing_fbw(
    demands: &mut StabilizeDemands,
    sticks: &RcStickInputs,
    mix_pitch: bool,
    inverted: bool,
) {
    let roll_input = nonlinear_stick_input(sticks.roll_norm_dz);
    demands.nav_roll_cd += (roll_input * demands.roll_limit_cd as f32) as i32;
    demands.nav_roll_cd = constrain_int32(
        demands.nav_roll_cd,
        -demands.roll_limit_cd,
        demands.roll_limit_cd,
    );

    if !mix_pitch {
        return;
    }

    let mut pitch_input = nonlinear_stick_input(sticks.pitch_norm_dz);
    if inverted {
        pitch_input = -pitch_input;
    }
    let pitch_range_cd = demands.pitch_limit_max_cd - demands.pitch_limit_min_cd;
    demands.nav_pitch_cd += (pitch_input * pitch_range_cd as f32 / 2.0) as i32;
    demands.nav_pitch_cd = constrain_int32(
        demands.nav_pitch_cd,
        demands.pitch_limit_min_cd,
        demands.pitch_limit_max_cd,
    );
}

/// Update demands and context before controller `servo_out`, upstream the
/// prologue to `Plane::stabilize`.
pub fn prepare_stabilize_path(
    demands: &mut StabilizeDemands,
    ctx: &mut StabilizeContext,
    nav: &NavCommandInputs,
    scaler_inp: &SpeedScalerInputs,
    dispatch: StabilizeDispatch,
    sticks: &RcStickInputs,
    stick_mixing: Option<StickMixing>,
    roll_sensor_cd: i32,
) {
    calc_nav_demands(demands, nav);
    if dispatch.fbw_stick_mixing {
        let mix_pitch = !matches!(stick_mixing, Some(StickMixing::FbwNoPitch));
        stabilize_stick_mixing_fbw(
            demands,
            sticks,
            mix_pitch,
            fly_inverted(roll_sensor_cd),
        );
    }
    ctx.scaler = calc_speed_scaler(scaler_inp);
    ctx.airspeed_eas = scaler_inp.airspeed_eas;
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
