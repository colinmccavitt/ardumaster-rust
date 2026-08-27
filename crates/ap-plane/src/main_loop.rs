//! ArduPlane main vehicle loop skeleton, upstream the four fast scheduler
//! tasks in `ArduPlane/Plane.cpp` and `Mode::run` in `ArduPlane/mode.cpp`.
//!
//! `ap-scheduler` owns tick ordering; this module is where the vehicle wires
//! those tasks to mode dispatch and the attitude/servo paths that follow.

use ap_ahrs::{YawCompassSample, YawDriftContext, YawGpsSample};
use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_scheduler::scheduler::{LOOP_RATE, RunStats, Scheduler, Task};

use crate::ahrs_hookup::{drift_motion_inputs, yaw_update_inputs, AhrsAttitude, AhrsFeed};
use ap_landing::deepstall_override::DeepstallOverrideInputs;
use ap_landing::landing_state_machine::VerifyLandEffects;
use ap_landing::deepstall_stage::DeepstallStage;
use crate::go_around_hookup::apply_landing_go_around_latch;
use crate::landing_hookup::{landing_servo_hookup, LandingServoHookupInputs, ServoOutputState};
use crate::landing_loop::{LandingContext, VerifyLandVehicleInputs};
use crate::landing_loop_hookup::{landing_loop_scheduler_tick, LandingLoopSchedulerInputs};
use crate::nav_tecs_hookup::{feed_nav_commands, NavTecsPublish};
use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish};
use crate::mode::ModeState;
use crate::mode_table_hookup::dispatch_stabilize_from_mode;
use crate::stabilize_hookup::{
    apply_stabilize_to_servos, prepare_stabilize_path, stabilize_controllers, NavCommandInputs,
    RcStickInputs, SpeedScalerInputs, StabilizeContext, StabilizeControllers, StabilizeDemands,
    StabilizeServoDemands,
};
use crate::mode_run::{applies_fbw_stick_mixing, StickMixing};
use crate::mode_table::{BuildFeatures, ModeNumber};

/// Per-loop accounting for the four fast tasks.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FastTaskTicks {
    /// Upstream `Plane::ahrs_update`.
    pub ahrs_update: u32,
    /// Upstream `Plane::update_control_mode`.
    pub update_control_mode: u32,
    /// Upstream `Plane::stabilize`.
    pub stabilize: u32,
    /// Upstream `Plane::set_servos`.
    pub set_servos: u32,
}

/// Which stabilization paths the active mode's `run()` selected.
///
/// Upstream `Mode::run` decides whether to call the three `stabilize_*`
/// helpers and whether fly-by-wire stick mixing runs first.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StabilizeDispatch {
    pub roll: bool,
    pub pitch: bool,
    pub yaw: bool,
    pub fbw_stick_mixing: bool,
}

/// Which stabilization paths ran on the last `stabilize` call.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StabilizeRun {
    pub roll: bool,
    pub pitch: bool,
    pub yaw: bool,
}

/// Vehicle state the main loop carries between scheduler ticks.
#[derive(Debug, Clone)]
pub struct PlaneMainLoop {
    pub mode: ModeState,
    pub stick_mixing: Option<StickMixing>,
    pub features: BuildFeatures,
    pub ticks: FastTaskTicks,
    pub last_stabilize: StabilizeDispatch,
    pub last_stabilize_run: StabilizeRun,
    /// DCM estimator and drift correction, upstream `AP::ahrs()`.
    pub ahrs: AhrsFeed,
    /// INS frontend publishing primary IMU samples, upstream `AP::ins()`.
    pub ins: InertialSensorFrontend,
    /// Loop timing passed into INS and AHRS, upstream scheduler deltas.
    pub loop_timing: LoopTiming,
    /// Attitude sensors published by the latest `ahrs_update`.
    pub attitude: AhrsAttitude,
    /// Optional compass sample for yaw drift correction.
    pub compass: Option<YawCompassSample>,
    /// Optional GPS sample for yaw drift fallback.
    pub gps_yaw: Option<YawGpsSample>,
    /// Vehicle context for compass vs GPS yaw selection.
    pub yaw_ctx: YawDriftContext,
    /// True airspeed for no-GPS drift and wind estimation, m/s.
    pub airspeed_tas: f32,
    /// Optional SITL yaw publish source; when set, samples are refreshed each `ahrs_update`.
    pub sitl_yaw: Option<SitlYawPublish>,
    /// Roll/pitch/yaw controllers, upstream `rollController` et al.
    pub controllers: StabilizeControllers,
    /// L1/TECS navigation publish source refreshed before stabilize.
    pub nav_tecs: NavTecsPublish,
    /// Raw navigation commands before limiting, upstream nav_controller/TECS.
    pub nav_commands: NavCommandInputs,
    /// RC stick inputs for FBW mixing.
    pub rc_sticks: RcStickInputs,
    /// Inputs to the speed scaler, upstream `calc_speed_scaler`.
    pub speed_scaler_inputs: SpeedScalerInputs,
    /// Cached surface speed scaler, upstream `surface_speed_scaler`.
    pub surface_speed_scaler: f32,
    /// Navigation demands fed into stabilize.
    pub stabilize_demands: StabilizeDemands,
    /// Per-loop context for the attitude controllers.
    pub stabilize_ctx: StabilizeContext,
    /// Scaled demands from the latest `stabilize`.
    pub stabilize_servos: StabilizeServoDemands,
    /// Landing state machine and flags, upstream `AP_Landing`.
    pub landing: LandingContext,
    /// HAL measurements for verify_land, upstream `Plane::verify_command`.
    pub verify_land_inputs: VerifyLandVehicleInputs,
    /// Roll limit during LAND, upstream `roll_limit_cd`.
    pub level_roll_limit_cd: i32,
    /// Effects from the latest landing-loop scheduler tick.
    pub last_verify_land_effects: VerifyLandEffects,
    /// Whether landing suppressed throttle on the last tick.
    pub landing_throttle_suppressed: bool,
    /// Upstream `flight_stage == LAND`.
    pub flight_stage_is_land: bool,
    /// Deepstall servo override HAL inputs for the landing hookup.
    pub deepstall_override: DeepstallOverrideInputs,
    /// Whether landing overrode servos on the last `set_servos`.
    pub landing_servo_override_applied: bool,
    /// Go-around requested because deepstall elevator is missing.
    pub landing_request_go_around: bool,
    /// Servo outputs about to be published, upstream `set_servos` state.
    pub servos: ServoOutputState,
}

impl Default for PlaneMainLoop {
    fn default() -> Self {
        Self {
            mode: ModeState {
                control_mode: ModeNumber::Initialising.as_number(),
                previous_mode: ModeNumber::Manual.as_number(),
                control_mode_reason: crate::mode::ModeReason::Initialised,
                previous_mode_reason: crate::mode::ModeReason::Initialised,
            },
            stick_mixing: Some(StickMixing::Fbw),
            features: BuildFeatures::default(),
            ticks: FastTaskTicks::default(),
            last_stabilize: StabilizeDispatch::default(),
            last_stabilize_run: StabilizeRun::default(),
            ahrs: AhrsFeed::default(),
            ins: InertialSensorFrontend::default(),
            loop_timing: LoopTiming::new(1.0 / f32::from(LOOP_RATE)),
            attitude: AhrsAttitude::default(),
            compass: None,
            gps_yaw: None,
            yaw_ctx: YawDriftContext::default(),
            airspeed_tas: 0.0,
            sitl_yaw: None,
            controllers: StabilizeControllers::default(),
            nav_tecs: NavTecsPublish::default(),
            nav_commands: NavCommandInputs::default(),
            rc_sticks: RcStickInputs::default(),
            speed_scaler_inputs: SpeedScalerInputs::default(),
            surface_speed_scaler: 1.0,
            stabilize_demands: StabilizeDemands::default(),
            stabilize_ctx: StabilizeContext::default(),
            stabilize_servos: StabilizeServoDemands::default(),
            landing: LandingContext::default(),
            verify_land_inputs: VerifyLandVehicleInputs {
                height_above_target_m: 0.0,
                terrain_correction_m: 0.0,
                sink_rate_ms: 0.0,
                wp_proportion: 0.0,
                is_flying: false,
                rangefinder_in_range: false,
                bearing_error_cd: 0,
                crosstrack_error_m: 0.0,
                nav_data_is_stale: false,
                below_prev_wp: false,
                prev_cmd_is_loiter_to_alt: false,
                crash_detection_enable: false,
                flare_cfg: ap_landing::slope_stage::FlareConfig {
                    flare_alt: 3.0,
                    flare_sec: 2.0,
                    pre_flare_alt: 8.0,
                    pre_flare_sec: 0.0,
                    pre_flare_airspeed: 12.0,
                },
                deepstall: ap_landing::deepstall_stage::DeepstallVerifyInputs {
                    distance_to_landing_m: 0.0,
                    distance_to_arc_entry_m: 0.0,
                    loiter_radius_m: 0.0,
                    loiter_ccw: false,
                    reached_loiter: false,
                    height_error_m: 0.0,
                    target_bearing_cd: 0,
                    heading_error_deg: 0.0,
                    target_heading_deg: 0.0,
                    groundspeed_ne: ap_math::vector2::Vector2f::default(),
                    current: ap_math::location::Location::new(0, 0),
                    arc_exit: ap_math::location::Location::new(0, 0),
                    arc_entry: ap_math::location::Location::new(0, 0),
                    extended_approach: ap_math::location::Location::new(0, 0),
                    entry_point: ap_math::location::Location::new(0, 0),
                },
            },
            level_roll_limit_cd: 4500,
            last_verify_land_effects: VerifyLandEffects::default(),
            landing_throttle_suppressed: false,
            flight_stage_is_land: false,
            deepstall_override: DeepstallOverrideInputs {
                stage: DeepstallStage::FlyToLanding,
                stall_entry_ms: 0,
                now_ms: 0,
                slew_speed: 1.0,
                initial_elevator_pwm: 1500,
                target_elevator_pwm: 1500,
                airspeed_ms: None,
                handoff_airspeed_ms: 12.0,
                handoff_lower_limit_ms: 8.0,
                steering_pid: 0.0,
                aileron_scalar: 1.0,
                elevator_present: true,
            },
            landing_servo_override_applied: false,
            landing_request_go_around: false,
            servos: ServoOutputState::default(),
        }
    }
}

impl PlaneMainLoop {
    /// Upstream `Plane::ahrs_update`. Runs INS→DCM and publishes attitude sensors.
    pub fn ahrs_update(&mut self) {
        self.ticks.ahrs_update += 1;
        if let Some(source) = self.sitl_yaw {
            let samples = publish_sitl_yaw_samples(
                &source,
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            self.compass = samples.compass;
            self.gps_yaw = samples.gps_yaw;
            self.yaw_ctx = samples.yaw_ctx;
        }
        let yaw = yaw_update_inputs(self.compass, self.gps_yaw, self.yaw_ctx);
        let motion = drift_motion_inputs(
            self.yaw_ctx,
            self.gps_yaw,
            self.airspeed_tas,
            &mut self.ahrs.last_gps_fix_ms,
        );
        let (_health, attitude) = self.ahrs.update_from_ins(
            &self.ins,
            &self.loop_timing,
            yaw,
            motion,
        );
        self.attitude = attitude;
    }

    /// Upstream `Plane::update_control_mode`. Dispatches to the active mode.
    pub fn update_control_mode(&mut self) {
        self.ticks.update_control_mode += 1;
        self.last_stabilize = dispatch_stabilize_from_mode(
            self.mode.control_mode,
            self.stick_mixing,
            &self.features,
        );

        if self.flight_stage_is_land {
            let out = landing_loop_scheduler_tick(
                &mut self.landing,
                &LandingLoopSchedulerInputs {
                    verify: self.verify_land_inputs,
                    nav_roll_cd: self.nav_tecs.nav_roll_cd,
                    level_roll_limit_cd: self.level_roll_limit_cd,
                },
            );
            self.last_verify_land_effects = out.effects;
            self.landing_throttle_suppressed = out.throttle_suppressed;
            self.nav_tecs.nav_roll_cd = out.nav_roll_cd;
        }
    }

    /// Upstream `Plane::stabilize`. Calls roll/pitch/yaw controllers when the
    /// active mode selected them on the previous `update_control_mode`.
    pub fn stabilize(&mut self) {
        self.ticks.stabilize += 1;
        feed_nav_commands(&mut self.nav_commands, &self.nav_tecs);
        prepare_stabilize_path(
            &mut self.stabilize_demands,
            &mut self.stabilize_ctx,
            &self.nav_commands,
            &self.speed_scaler_inputs,
            self.last_stabilize,
            &self.rc_sticks,
            self.stick_mixing,
            self.attitude.roll_sensor_cd,
        );
        self.surface_speed_scaler = self.stabilize_ctx.scaler;

        let imu = self
            .ins
            .primary_imu()
            .unwrap_or(&self.ins.instances[0]);
        let out = stabilize_controllers(
            &mut self.controllers,
            &self.attitude,
            imu,
            self.last_stabilize,
            &self.stabilize_demands,
            &self.stabilize_ctx,
            self.loop_timing.delta_time,
        );
        self.last_stabilize_run = out.run;
        self.stabilize_servos = out.servos;
    }

    /// Upstream `Plane::set_servos`. Publishes scaled/PWM demands from stabilize,
    /// then applies landing servo overrides when in LAND.
    pub fn set_servos(&mut self) {
        self.ticks.set_servos += 1;
        apply_stabilize_to_servos(&self.stabilize_servos, &mut self.servos);

        let hookup_inp = LandingServoHookupInputs {
            flight_stage_is_land: self.flight_stage_is_land,
            landing_flags: self.landing.flags,
            landing_type: self.landing.landing_type,
            deepstall_stage: self.landing.machine.deepstall.stage,
            deepstall: self.deepstall_override,
        };
        let result = landing_servo_hookup(self.servos, &hookup_inp);
        self.landing_servo_override_applied = result.applied_override;
        self.landing_request_go_around = result.request_go_around;
        apply_landing_go_around_latch(&mut self.landing.flags, result.request_go_around);
        self.servos = result.outputs;
    }
}


/// Re-export for tests that import from `main_loop`.
pub use crate::mode_table_hookup::dispatch_stabilize_from_mode as mode_run_dispatch;

fn task_ahrs(v: &mut PlaneMainLoop) {
    v.ahrs_update();
}
fn task_update_control_mode(v: &mut PlaneMainLoop) {
    v.update_control_mode();
}
fn task_stabilize(v: &mut PlaneMainLoop) {
    v.stabilize();
}
fn task_set_servos(v: &mut PlaneMainLoop) {
    v.set_servos();
}

/// The four ArduPlane fast tasks, in upstream priority order.
#[must_use]
pub fn plane_fast_tasks() -> [Task<PlaneMainLoop>; 4] {
    [
        Task {
            function: task_ahrs,
            name: "ahrs_update",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 0,
        },
        Task {
            function: task_update_control_mode,
            name: "update_control_mode",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 1,
        },
        Task {
            function: task_stabilize,
            name: "stabilize",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 2,
        },
        Task {
            function: task_set_servos,
            name: "set_servos",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 3,
        },
    ]
}

/// Advance one scheduler tick and run the fast-task pass.
pub fn run_scheduler_tick(
    vehicle: &mut PlaneMainLoop,
    scheduler: &mut Scheduler<'_, PlaneMainLoop>,
    clock: &dyn ap_hal::time::Clock,
    time_available_us: u32,
) -> RunStats {
    scheduler.tick();
    scheduler.run(vehicle, clock, time_available_us)
}
