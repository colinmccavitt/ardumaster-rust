//! The multicopter rate loop, upstream
//! `AC_AttitudeControl_Multi::rate_controller_run_dt`.
//!
//! The innermost loop of the whole stack, and the shortest. Everything above
//! it — modes, navigation, the attitude controller — exists to produce the
//! three numbers this hands to three PIDs.

use ap_math::vector3::Vector3f;
use ap_pid::{AcPid, Scaling};

use crate::throttle_mix::{GainBoost, ThrottleMix, ThrottleMixConfig, VehicleThrottleState};

/// The three rate PIDs and the scalings applied to them.
#[derive(Debug, Clone, Copy)]
pub struct RateLoop {
    /// Roll axis PID.
    pub roll: AcPid,
    /// Pitch axis PID.
    pub pitch: AcPid,
    /// Yaw axis PID.
    pub yaw: AcPid,

    pd_scale: Vector3f,
    i_scale: Vector3f,
    angle_p_scale: Vector3f,

    pd_scale_used: Vector3f,
    i_scale_used: Vector3f,
    angle_p_scale_used: Vector3f,

    sysid_ang_vel_body_rads: Vector3f,
    actuator_sysid: Vector3f,

    rate_gyro_rads: Vector3f,
    rate_gyro_time_us: u64,
}

/// Where the rate loop's output goes: the mixer's four inputs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorDemand {
    /// Roll demand, upstream `_motors.set_roll`.
    pub roll: f32,
    /// Pitch demand, upstream `_motors.set_pitch`.
    pub pitch: f32,
    /// Yaw demand, upstream `_motors.set_yaw`.
    pub yaw: f32,
    /// Roll feed-forward, upstream `set_roll_ff`.
    pub roll_ff: f32,
    /// Pitch feed-forward, upstream `set_pitch_ff`.
    pub pitch_ff: f32,
    /// Yaw feed-forward, upstream `set_yaw_ff`.
    ///
    /// The only one scaled by the feedforward blend. When the thrust vector is
    /// badly wrong the attitude controller gives up heading to save attitude,
    /// and the yaw *feed-forward* has to fade with it — otherwise the loop
    /// would keep driving toward a heading the controller has already
    /// abandoned.
    pub yaw_ff: f32,
}

/// Everything the rate loop needs that is not its own state.
#[derive(Debug, Clone, Copy)]
pub struct RateLoopInputs {
    /// The attitude controller's body-frame rate demand, upstream
    /// `_ang_vel_body_rads`.
    pub ang_vel_body_rads: Vector3f,
    /// Latest gyro.
    pub gyro_rads: Vector3f,
    /// How much feedforward survived the thrust-error blend, from the attitude
    /// controller's output.
    pub feedforward_scalar: f32,
    /// Whether the mixer reported each axis saturated, upstream
    /// `_motors.limit.*`. A saturated axis stops its integrator growing.
    pub limit_roll: bool,
    /// Pitch axis saturated.
    pub limit_pitch: bool,
    /// Yaw axis saturated.
    pub limit_yaw: bool,
    /// Monotonic time for the PIDs' slew limiter.
    pub now_ms: u32,
    /// Monotonic microseconds, recorded alongside the gyro.
    pub now_us: u64,
}

impl RateLoop {
    /// Build from three tuned PIDs, with every scaling at unity.
    #[must_use]
    pub fn new(roll: AcPid, pitch: AcPid, yaw: AcPid) -> Self {
        let ones = Vector3f::new(1.0, 1.0, 1.0);
        Self {
            roll,
            pitch,
            yaw,
            pd_scale: ones,
            i_scale: ones,
            angle_p_scale: ones,
            pd_scale_used: ones,
            i_scale_used: ones,
            angle_p_scale_used: ones,
            sysid_ang_vel_body_rads: Vector3f::new(0.0, 0.0, 0.0),
            actuator_sysid: Vector3f::new(0.0, 0.0, 0.0),
            rate_gyro_rads: Vector3f::new(0.0, 0.0, 0.0),
            rate_gyro_time_us: 0,
        }
    }

    /// The angle-P scaling the *last completed* cycle used, upstream
    /// `_angle_P_scale_used`.
    ///
    /// The attitude controller reads this rather than the live value. The two
    /// differ by exactly one cycle, and that is the point: the scale is set
    /// before the rate loop runs and cleared after, so a reader outside the
    /// loop that took the live value would see it mid-update.
    #[must_use]
    pub fn angle_p_scale_used(&self) -> Vector3f {
        self.angle_p_scale_used
    }

    /// The PD scaling the last completed cycle used.
    #[must_use]
    pub fn pd_scale_used(&self) -> Vector3f {
        self.pd_scale_used
    }

    /// The I scaling the last completed cycle used.
    #[must_use]
    pub fn i_scale_used(&self) -> Vector3f {
        self.i_scale_used
    }

    /// The gyro this loop last ran on, and when, upstream `_rate_gyro_rads`
    /// and `_rate_gyro_time_us`.
    ///
    /// Recorded so a separate rate thread can be the source of gyro data
    /// without the attitude side having to ask the AHRS again and get a
    /// different sample.
    #[must_use]
    pub fn latest_gyro(&self) -> (Vector3f, u64) {
        (self.rate_gyro_rads, self.rate_gyro_time_us)
    }

    /// Multiply this cycle's PD scale, upstream `set_PD_scale_mult`.
    pub fn set_pd_scale_mult(&mut self, scale: Vector3f) {
        self.pd_scale = Vector3f::new(
            self.pd_scale.x * scale.x,
            self.pd_scale.y * scale.y,
            self.pd_scale.z * scale.z,
        );
    }

    /// Multiply this cycle's angle-P scale, upstream `set_angle_P_scale_mult`.
    pub fn set_angle_p_scale_mult(&mut self, scale: Vector3f) {
        self.angle_p_scale = Vector3f::new(
            self.angle_p_scale.x * scale.x,
            self.angle_p_scale.y * scale.y,
            self.angle_p_scale.z * scale.z,
        );
    }

    /// Multiply this cycle's I scale, upstream `set_I_scale_mult`.
    pub fn set_i_scale_mult(&mut self, scale: Vector3f) {
        self.i_scale = Vector3f::new(
            self.i_scale.x * scale.x,
            self.i_scale.y * scale.y,
            self.i_scale.z * scale.z,
        );
    }

    /// Inject a system-identification rate, upstream
    /// `_sysid_ang_vel_body_rads`.
    pub fn set_sysid_ang_vel_body(&mut self, rate: Vector3f) {
        self.sysid_ang_vel_body_rads = rate;
    }

    /// Inject a system-identification actuator offset, upstream
    /// `_actuator_sysid`.
    pub fn set_actuator_sysid(&mut self, actuator: Vector3f) {
        self.actuator_sysid = actuator;
    }

    /// Clear the per-cycle scalings and sysid injections, upstream
    /// `rate_controller_target_reset`.
    pub fn target_reset(&mut self) {
        let ones = Vector3f::new(1.0, 1.0, 1.0);
        self.sysid_ang_vel_body_rads = Vector3f::new(0.0, 0.0, 0.0);
        self.actuator_sysid = Vector3f::new(0.0, 0.0, 0.0);
        self.pd_scale = ones;
        self.i_scale = ones;
        self.angle_p_scale = ones;
    }

    /// Clear all three integrators, upstream `reset_rate_controller_I_terms`.
    pub fn reset_i_terms(&mut self) {
        self.roll.reset_i();
        self.pitch.reset_i();
        self.yaw.reset_i();
    }

    /// Reset all three input filters, upstream's half of
    /// `relax_attitude_controllers` that touches the PIDs.
    pub fn reset_filters(&mut self) {
        self.roll.reset_filter();
        self.pitch.reset_filter();
        self.yaw.reset_filter();
    }

    /// The rate loop's half of `relax_attitude_controllers`.
    ///
    /// Filters first, then integrators. Upstream's order, and it is the right
    /// one: `reset_filter` reseeds from the next sample, so clearing the
    /// integrators afterwards cannot be undone by a filter that was still
    /// carrying the old error.
    pub fn relax(&mut self) {
        self.reset_filters();
        self.reset_i_terms();
    }

    /// One iteration, upstream `rate_controller_run_dt`.
    ///
    /// The throttle housekeeping runs from here because upstream puts it here,
    /// and upstream puts it here because this is the one function guaranteed
    /// to be called every iteration. It is not conceptually part of the rate
    /// loop; it is here to be sure it happens.
    ///
    /// The gain boost is applied *before* the PIDs read their scales, so a
    /// throttle slew affects the same cycle that detected it rather than the
    /// next one.
    pub fn run(
        &mut self,
        inputs: &RateLoopInputs,
        mix: &mut ThrottleMix,
        state: &VehicleThrottleState,
        config: &ThrottleMixConfig,
        dt: f32,
    ) -> MotorDemand {
        // Copied first, upstream's comment: "so that it can't be changed from
        // under us". Meaningful because a separate rate thread may be running
        // this while the attitude loop writes the target.
        let mut ang_vel_body = inputs.ang_vel_body_rads;

        if let Some(GainBoost {
            pd_scale,
            angle_p_scale,
        }) = ThrottleMix::update_throttle_gain_boost(state, config)
        {
            self.set_pd_scale_mult(pd_scale);
            self.set_angle_p_scale_mult(angle_p_scale);
        }

        mix.update_throttle_rpy_mix(state, dt);

        ang_vel_body += self.sysid_ang_vel_body_rads;

        self.rate_gyro_rads = inputs.gyro_rads;
        self.rate_gyro_time_us = inputs.now_us;

        let roll = self.roll.update_all(
            ang_vel_body.x,
            inputs.gyro_rads.x,
            dt,
            inputs.limit_roll,
            Scaling {
                pd: self.pd_scale.x,
                i: self.i_scale.x,
            },
            inputs.now_ms,
        ) + self.actuator_sysid.x;
        let roll_ff = self.roll.info().ff;

        let pitch = self.pitch.update_all(
            ang_vel_body.y,
            inputs.gyro_rads.y,
            dt,
            inputs.limit_pitch,
            Scaling {
                pd: self.pd_scale.y,
                i: self.i_scale.y,
            },
            inputs.now_ms,
        ) + self.actuator_sysid.y;
        let pitch_ff = self.pitch.info().ff;

        let yaw = self.yaw.update_all(
            ang_vel_body.z,
            inputs.gyro_rads.z,
            dt,
            inputs.limit_yaw,
            Scaling {
                pd: self.pd_scale.z,
                i: self.i_scale.z,
            },
            inputs.now_ms,
        ) + self.actuator_sysid.z;
        let yaw_ff = self.yaw.info().ff * inputs.feedforward_scalar;

        self.pd_scale_used = self.pd_scale;
        self.i_scale_used = self.i_scale;
        self.angle_p_scale_used = self.angle_p_scale;

        MotorDemand {
            roll,
            pitch,
            yaw,
            roll_ff,
            pitch_ff,
            yaw_ff,
        }
    }
}
