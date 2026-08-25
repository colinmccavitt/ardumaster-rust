//! The attitude controller's state and input entry points, upstream
//! `AC_AttitudeControl`. COP-007.
//!
//! Everything before this module is a pure function. This is where the state
//! lives: the target attitude the controller is steering toward, and the rate
//! and acceleration it is feeding forward.
//!
//! # The target is not the command
//!
//! The distinction that makes the whole design work: a pilot's stick position
//! is not the attitude target. It is the attitude the target is *shaped
//! toward*, subject to rate and acceleration limits, and the target moves
//! there over several iterations. The aircraft chases the target; the target
//! chases the stick.
//!
//! That is why the entry points below advance the target before doing anything
//! else, and why the shaping is applied to the *difference* between stick and
//! target rather than to the stick directly. Skipping the indirection gives an
//! aircraft that snaps to stick inputs and cannot express a rate limit at all.

use ap_math::quaternion::Quaternion;
use ap_math::scalar::{radians, wrap_pi};
use ap_math::vector3::Vector3f;

use crate::attitude_error::{
    attitude_command_model, attitude_controller_run, update_attitude_target, AngleGains,
    CommandModel, ControllerInputs, ControllerOutput, YawLimitGains,
};
use crate::attitude_kinematics::{
    body_to_euler_derivative, body_to_euler_limit, euler_derivative_to_body,
};

/// The tunables the input shaping reads.
#[derive(Debug, Clone, Copy)]
pub struct ShapingConfig {
    /// `ATC_INPUT_TC`: how sharply roll and pitch follow the stick.
    pub input_tc: f32,
    /// `ATC_RATE_Y_TC`: the same for the yaw rate, separately tunable because
    /// yaw has far less authority.
    pub rate_y_tc: f32,
    /// `ATC_RATE_FF_ENAB`: whether to shape at all. With it off the target
    /// jumps straight to the stick and no feedforward is produced.
    pub rate_bf_ff_enabled: bool,
    /// Maximum roll rate, degrees per second.
    pub ang_vel_roll_max_degs: f32,
    /// Maximum pitch rate, degrees per second.
    pub ang_vel_pitch_max_degs: f32,
    /// Maximum yaw rate, degrees per second.
    pub ang_vel_yaw_max_degs: f32,
    /// Maximum roll acceleration, rad/s².
    pub accel_roll_max_radss: f32,
    /// Maximum pitch acceleration, rad/s².
    pub accel_pitch_max_radss: f32,
    /// Maximum yaw acceleration, rad/s².
    pub accel_yaw_max_radss: f32,
}

/// The controller's target state, upstream's `_attitude_target` and the
/// feedforward vectors beside it.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeController {
    attitude_target: Quaternion,
    euler_angle_target_rad: Vector3f,
    euler_rate_target_rads: Vector3f,
    ang_vel_target_rads: Vector3f,
    ang_accel_target_rads: Vector3f,
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self::new()
    }
}

impl AttitudeController {
    /// Level, north-facing, with no rate or acceleration feedforward.
    #[must_use]
    pub fn new() -> Self {
        Self {
            attitude_target: Quaternion::identity(),
            euler_angle_target_rad: Vector3f::new(0.0, 0.0, 0.0),
            euler_rate_target_rads: Vector3f::new(0.0, 0.0, 0.0),
            ang_vel_target_rads: Vector3f::new(0.0, 0.0, 0.0),
            ang_accel_target_rads: Vector3f::new(0.0, 0.0, 0.0),
        }
    }

    /// The attitude the controller is steering toward.
    #[must_use]
    pub fn attitude_target(&self) -> Quaternion {
        self.attitude_target
    }

    /// The target attitude as Euler angles.
    #[must_use]
    pub fn euler_angle_target_rad(&self) -> Vector3f {
        self.euler_angle_target_rad
    }

    /// The target's angular velocity, body frame.
    #[must_use]
    pub fn ang_vel_target_rads(&self) -> Vector3f {
        self.ang_vel_target_rads
    }

    /// Set the target directly, for resets and for tests.
    pub fn set_attitude_target(&mut self, target: Quaternion) {
        self.attitude_target = target;
        let (r, p, y) = target.to_euler();
        self.euler_angle_target_rad = Vector3f::new(r, p, y);
    }

    /// Roll and pitch as angles, yaw as a rate — upstream
    /// `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
    ///
    /// This is the stabilised-flight entry point: the stick commands a lean
    /// directly and a *rate* of turn, which is what a pilot expects from a
    /// multirotor.
    ///
    /// The limits are converted from body frame to Euler frame first, through
    /// [`body_to_euler_limit`]. That is not a formality: an aircraft leaning
    /// hard needs a much larger Euler yaw rate to achieve a given body yaw
    /// rate, and limiting in the wrong frame would either throttle it
    /// needlessly or let it exceed what the airframe can do.
    #[expect(
        clippy::too_many_arguments,
        reason = "one upstream entry point; its arguments are the vehicle's \
configuration and the pilot's command, and bundling them further would hide \
which is which"
    )]
    pub fn input_euler_angle_roll_pitch_euler_rate_yaw(
        &mut self,
        euler_roll_angle_rad: f32,
        euler_pitch_angle_rad: f32,
        euler_yaw_rate_rads: f32,
        attitude_body: Quaternion,
        shaping: &ShapingConfig,
        yaw_gains: &YawLimitGains,
        angle_gains: &AngleGains,
        gyro_rads: Vector3f,
        dt: f32,
    ) -> ControllerOutput {
        self.attitude_target =
            update_attitude_target(self.attitude_target, self.ang_vel_target_rads, dt);

        let (r, p, y) = self.attitude_target.to_euler();
        self.euler_angle_target_rad = Vector3f::new(r, p, y);

        if shaping.rate_bf_ff_enabled {
            let euler_accel_radss = body_to_euler_limit(
                self.attitude_target,
                Vector3f::new(
                    shaping.accel_roll_max_radss,
                    shaping.accel_pitch_max_radss,
                    shaping.accel_yaw_max_radss,
                ),
            );
            let euler_rate_max_rads = body_to_euler_limit(
                self.attitude_target,
                Vector3f::new(
                    radians(shaping.ang_vel_roll_max_degs),
                    radians(shaping.ang_vel_pitch_max_degs),
                    radians(shaping.ang_vel_yaw_max_degs),
                ),
            );

            // At gimbal lock there is no Euler acceleration to shape from.
            // Upstream leaves its output untouched, which means the previous
            // iteration's value carries; reproduced by falling back to zero
            // only where there was nothing to carry.
            let mut euler_accel_target_rads =
                body_to_euler_derivative(self.attitude_target, self.ang_accel_target_rads)
                    .unwrap_or(Vector3f::new(0.0, 0.0, 0.0));

            let roll = attitude_command_model(
                CommandModel {
                    target_ang_vel: self.euler_rate_target_rads.x,
                    target_ang_accel: euler_accel_target_rads.x,
                },
                wrap_pi(euler_roll_angle_rad - self.euler_angle_target_rad.x),
                0.0,
                libm::fabsf(euler_rate_max_rads.x),
                euler_accel_radss.x,
                shaping.input_tc,
                dt,
            );
            let pitch = attitude_command_model(
                CommandModel {
                    target_ang_vel: self.euler_rate_target_rads.y,
                    target_ang_accel: euler_accel_target_rads.y,
                },
                wrap_pi(euler_pitch_angle_rad - self.euler_angle_target_rad.y),
                0.0,
                libm::fabsf(euler_rate_max_rads.y),
                euler_accel_radss.y,
                shaping.input_tc,
                dt,
            );
            // Yaw takes a rate command, not an angle: the error is zero and
            // the desired velocity carries the stick. And it uses its own time
            // constant, because yaw authority is a fraction of roll's.
            let yaw = attitude_command_model(
                CommandModel {
                    target_ang_vel: self.euler_rate_target_rads.z,
                    target_ang_accel: euler_accel_target_rads.z,
                },
                0.0,
                euler_yaw_rate_rads,
                libm::fabsf(euler_rate_max_rads.z),
                euler_accel_radss.z,
                shaping.rate_y_tc,
                dt,
            );

            self.euler_rate_target_rads = Vector3f::new(
                roll.target_ang_vel,
                pitch.target_ang_vel,
                yaw.target_ang_vel,
            );
            euler_accel_target_rads = Vector3f::new(
                roll.target_ang_accel,
                pitch.target_ang_accel,
                yaw.target_ang_accel,
            );

            self.ang_vel_target_rads =
                euler_derivative_to_body(self.attitude_target, self.euler_rate_target_rads);
            self.ang_accel_target_rads =
                euler_derivative_to_body(self.attitude_target, euler_accel_target_rads);
        } else {
            self.euler_angle_target_rad.x = euler_roll_angle_rad;
            self.euler_angle_target_rad.y = euler_pitch_angle_rad;
            self.euler_angle_target_rad.z += euler_yaw_rate_rads * dt;

            self.attitude_target = Quaternion::from_euler(
                self.euler_angle_target_rad.x,
                self.euler_angle_target_rad.y,
                self.euler_angle_target_rad.z,
            );

            self.euler_rate_target_rads = Vector3f::new(0.0, 0.0, 0.0);
            self.ang_vel_target_rads = Vector3f::new(0.0, 0.0, 0.0);
            self.ang_accel_target_rads = Vector3f::new(0.0, 0.0, 0.0);
        }

        let out = attitude_controller_run(
            self.attitude_target,
            attitude_body,
            yaw_gains,
            angle_gains,
            &ControllerInputs {
                ang_vel_target_rads: self.ang_vel_target_rads,
                gyro_rads,
                ang_vel_roll_max_degs: shaping.ang_vel_roll_max_degs,
                ang_vel_pitch_max_degs: shaping.ang_vel_pitch_max_degs,
                ang_vel_yaw_max_degs: shaping.ang_vel_yaw_max_degs,
            },
            dt,
        );

        // The yaw cap may have rebuilt the target; keep it.
        self.attitude_target = out.attitude_target;
        out
    }
}
