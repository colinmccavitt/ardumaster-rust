//! ArduPlane main vehicle loop skeleton, upstream the four fast scheduler
//! tasks in `ArduPlane/Plane.cpp` and `Mode::run` in `ArduPlane/mode.cpp`.
//!
//! `ap-scheduler` owns tick ordering; this module is where the vehicle wires
//! those tasks to mode dispatch and the attitude/servo paths that follow.

use ap_ahrs::{AhrsBackendKind, MatrixHealth, YawCompassSample, YawDriftContext, YawGpsSample};
use ap_ins::sitl::{SitlBodyState, SitlInsInstanceFiles, SITL_INS_MAX_INSTANCES};
use ap_ins::{InertialSensorFrontend, LoopTiming, SitlInsMotorRuntime};
use ap_scheduler::scheduler::{LOOP_RATE, RunStats, Scheduler, Task};

use crate::ahrs_hookup::{drift_motion_inputs, yaw_update_inputs, AhrsAttitude, AhrsFeed};
use ap_landing::deepstall_override::DeepstallOverrideInputs;
use ap_landing::landing_state_machine::VerifyLandEffects;
use ap_landing::deepstall_stage::DeepstallStage;
use crate::deepstall_override_scheduler_hookup::{
    deepstall_override_scheduler_tick, DeepstallOverrideSchedulerInputs,
};
use crate::landing_hookup::ServoOutputState;
use crate::landing_loop::{LandingContext, VerifyLandVehicleInputs};
use crate::landing_loop_hookup::{landing_loop_scheduler_tick, LandingLoopSchedulerInputs};
use crate::landing_throttle_scheduler_hookup::{
    landing_throttle_scheduler_tick, LandingThrottleSchedulerInputs,
};
use crate::rangefinder_bump_hookup::{RangefinderBumpContext, RangefinderBumpHookupInputs};
use crate::rangefinder_bump_scheduler_hookup::{rangefinder_bump_scheduler_tick, RangefinderBumpSchedulerInputs};
use crate::rc_failsafe_scheduler_hookup::{rc_failsafe_scheduler_tick, RcFailsafeSchedulerInputs};
use crate::mission_scheduler_hookup::{mission_scheduler_tick, MissionContext, MissionSchedulerInputs};
use crate::target_altitude::TargetAltitude;
use crate::nav_tecs_hookup::{feed_nav_commands, NavTecsPublish};
use crate::ins_hntch_scheduler_hookup::{
    ins_hntch_scheduler_tick, ins_hntch_scheduler_tick_cluster, InsHntchHookup,
    InsHntchSchedulerInputs,
};
use crate::sitl_ins_host_files::sitl_ins_host_files_fill;
use crate::sitl_ins_host_files::SitlInsHostFiles;
use crate::sitl_ins_noise_hookup::{
    sitl_ins_noise_scheduler_tick, SitlInsNoiseHookup, SitlInsNoiseSchedulerInputs,
};
use crate::sitl_ahrs_hookup::{publish_sitl_ahrs_samples, SitlAhrsPublish};
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
    /// DCM matrix health from the latest AHRS cycle.
    pub ahrs_matrix_health: MatrixHealth,
    /// Whether NavEKF3 reported healthy this cycle.
    pub ekf_healthy: bool,
    /// Active AHRS backend after health fallback.
    pub active_ahrs_backend: AhrsBackendKind,
    /// Dead-reckoning north offset from last GPS fix, metres.
    pub dead_reckoning_north_m: f32,
    /// Dead-reckoning east offset from last GPS fix, metres.
    pub dead_reckoning_east_m: f32,
    /// Whether a GPS fix anchor exists for dead reckoning.
    pub have_dead_reckoning_position: bool,
    /// Optional compass sample for yaw drift correction.
    pub compass: Option<YawCompassSample>,
    /// Optional GPS sample for yaw drift fallback.
    pub gps_yaw: Option<YawGpsSample>,
    /// Vehicle context for compass vs GPS yaw selection.
    pub yaw_ctx: YawDriftContext,
    /// True airspeed for no-GPS drift and wind estimation, m/s.
    pub airspeed_tas: f32,
    /// EAS to TAS scale for wind estimation, upstream `AP_Baro::get_EAS2TAS()`.
    pub eas2tas: f32,
    /// Optional wind vane sample; seeds AHRS wind before drift correction.
    pub wind_vane: Option<ap_ahrs::WindVaneSample>,
    /// Latest estimated wind in NED, upstream `AP_AHRS::wind_estimate`.
    pub estimated_wind: ap_math::vector3::Vector3f,
    /// Head-wind component along fuselage, upstream `AP_AHRS::head_wind`.
    pub head_wind_ms: f32,
    /// Optional SITL yaw publish source; when set, samples are refreshed each `ahrs_update`.
    pub sitl_yaw: Option<SitlYawPublish>,
    /// Optional SITL AHRS publish (yaw + airspeed TAS); takes precedence over `sitl_yaw`.
    pub sitl_ahrs: Option<SitlAhrsPublish>,
    /// Optional SITL INS noise cluster hookup; when set, runs before AHRS each tick.
    pub sitl_ins_noise: Option<SitlInsNoiseHookup>,
    /// Optional INS harmonic notch hookup; configures gyro filters each tick.
    pub ins_hntch: Option<InsHntchHookup>,
    /// Per-tick motor runtime for SIM_VIB noise injection.
    pub sitl_ins_motor: SitlInsMotorRuntime,
    /// Kinematic body state for the SITL INS cluster this tick.
    pub sitl_body: SitlBodyState,
    /// Monotonic time in microseconds for SITL INS timer_update.
    pub sitl_now_us: u64,
    pub sitl_ins_host_files: [SitlInsHostFiles; SITL_INS_MAX_INSTANCES],
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
    /// Persistent slope/rangefinder state for bump recalculation.
    pub rangefinder_bump: RangefinderBumpContext,
    /// HAL inputs for rangefinder bump during LAND.
    pub rangefinder_bump_inputs: RangefinderBumpHookupInputs,
    /// Whether the latest scheduler tick recalculated the glide slope.
    pub last_rangefinder_bump_recalculated: bool,
    /// Upstream `flight_stage == LAND`.
    pub flight_stage_is_land: bool,
    /// Deepstall servo override HAL inputs for the landing hookup.
    pub deepstall_override: DeepstallOverrideInputs,
    /// Whether landing overrode servos on the last `set_servos`.
    pub landing_servo_override_applied: bool,
    /// Whether landing zeroed throttle on the last `set_servos`.
    pub landing_throttle_applied: bool,
    /// Go-around requested because deepstall elevator is missing.
    pub landing_request_go_around: bool,
    /// Mission index and completion, upstream `AP_Mission`.
    pub mission: MissionContext,
    /// HAL inputs for mission advancement and target altitude each tick.
    pub mission_inputs: MissionSchedulerInputs,
    /// Target altitude source from the latest mission scheduler tick.
    pub last_target_altitude: TargetAltitude,
    /// Whether the latest scheduler tick advanced the mission index.
    pub mission_advanced: bool,
    /// HAL inputs for RC channel read and failsafe during the scheduler tick.
    pub rc_failsafe_inputs: RcFailsafeSchedulerInputs,
    /// Whether the latest scheduler tick saw an RC failsafe.
    pub in_rc_failsafe: bool,
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
            ahrs_matrix_health: MatrixHealth::Ok,
            ekf_healthy: false,
            active_ahrs_backend: AhrsBackendKind::default(),
            dead_reckoning_north_m: 0.0,
            dead_reckoning_east_m: 0.0,
            have_dead_reckoning_position: false,
            compass: None,
            gps_yaw: None,
            yaw_ctx: YawDriftContext::default(),
            airspeed_tas: 0.0,
            eas2tas: 1.0,
            wind_vane: None,
            estimated_wind: ap_math::vector3::Vector3f::zero(),
            head_wind_ms: 0.0,
            sitl_yaw: None,
            sitl_ahrs: None,
            sitl_ins_noise: None,
            ins_hntch: None,
            sitl_ins_motor: SitlInsMotorRuntime::default(),
            sitl_body: SitlBodyState::default(),
            sitl_now_us: 0,
            sitl_ins_host_files: [SitlInsHostFiles::default(); SITL_INS_MAX_INSTANCES],
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
            rangefinder_bump: RangefinderBumpContext::default(),
            rangefinder_bump_inputs: RangefinderBumpHookupInputs {
                flight_stage_is_land: false,
                landing_type: ap_landing::go_around::LandingType::StandardGlideSlope,
                bump_cfg: ap_landing::rangefinder_bump::RangefinderBumpConfig {
                    shallow_threshold: 1.0,
                    steep_threshold_deg: 1.0,
                },
                slope_cfg: ap_landing::SlopeConfig {
                    flare_sec: 2.0,
                    flare_alt: 3.0,
                    flare_effectivness_pct: 50,
                },
                slope_inp: ap_landing::SlopeInputs {
                    prev_wp: ap_math::location::Location::new(0, 0),
                    next_wp: ap_math::location::Location::new(0, 0),
                    current: ap_math::location::Location::new(0, 0),
                    groundspeed: 0.0,
                    land_sinkrate: 1.0,
                    alt_ctx: ap_math::location::AltContext::default(),
                },
                bump: ap_landing::rangefinder_bump::RangefinderBumpInputs {
                    rf: ap_landing::slope_stage::RangefinderState {
                        in_use: false,
                        correction: 0.0,
                        last_stable_correction: 0.0,
                    },
                    prev_wp: ap_math::location::Location::new(0, 0),
                    next_wp: ap_math::location::Location::new(0, 0),
                    current: ap_math::location::Location::new(0, 0),
                    wp_distance_m: 0.0,
                    adjusted_altitude_cm: 0,
                    alt_ctx: ap_math::location::AltContext::default(),
                },
            },
            last_rangefinder_bump_recalculated: false,
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
            landing_throttle_applied: false,
            landing_request_go_around: false,
            mission: MissionContext::default(),
            mission_inputs: MissionSchedulerInputs::default(),
            last_target_altitude: TargetAltitude::FromNextWaypoint,
            mission_advanced: false,
            rc_failsafe_inputs: RcFailsafeSchedulerInputs::default(),
            in_rc_failsafe: false,
            servos: ServoOutputState::default(),
        }
    }
}

impl PlaneMainLoop {
    /// Upstream `Plane::ahrs_update`. Runs INS→DCM and publishes attitude sensors.
    pub fn ahrs_update(&mut self) {
        self.ticks.ahrs_update += 1;
        if let Some(source) = self.sitl_ahrs {
            let samples = publish_sitl_ahrs_samples(
                &source,
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            self.compass = samples.yaw.compass;
            self.gps_yaw = samples.yaw.gps_yaw;
            self.yaw_ctx = samples.yaw.yaw_ctx;
            self.airspeed_tas = samples.airspeed_tas;
            self.eas2tas = samples.eas2tas;
        } else if let Some(source) = self.sitl_yaw {
            let samples = publish_sitl_yaw_samples(
                &source,
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            self.compass = samples.compass;
            self.gps_yaw = samples.gps_yaw;
            self.yaw_ctx = samples.yaw_ctx;
        }
        if let Some(hntch) = self.ins_hntch.as_mut() {
            let hntch_inp = InsHntchSchedulerInputs {
                throttle: self.sitl_ins_motor.throttle,
                motor_mask: self.sitl_ins_motor.motor_mask,
                motor_rpm: self.sitl_ins_motor.motor_rpm,
            };
            if let Some(noise) = self.sitl_ins_noise.as_mut() {
                let _ = ins_hntch_scheduler_tick_cluster(
                    &mut noise.cluster,
                    hntch,
                    &hntch_inp,
                );
            } else {
                let _ = ins_hntch_scheduler_tick(&mut self.ins, hntch, &hntch_inp);
            }
        }
                if let Some(hookup) = self.sitl_ins_noise.as_mut() {
            let mut file_views = [SitlInsInstanceFiles::default(); SITL_INS_MAX_INSTANCES];
            let files = sitl_ins_host_files_fill(
                &self.sitl_ins_host_files,
                hookup.cluster.instance_count(),
                &mut file_views,
            );
            let _ = sitl_ins_noise_scheduler_tick(
                hookup,
                &SitlInsNoiseSchedulerInputs {
                    body: self.sitl_body,
                    motor: self.sitl_ins_motor,
                    files,
                    now_us: self.sitl_now_us,
                },
            );
            self.ins = hookup.cluster.frontend.clone();
        }
        if let Some(vane) = self.wind_vane {
            self.ahrs.apply_wind_vane(vane);
        }
        let yaw = yaw_update_inputs(self.compass, self.gps_yaw, self.yaw_ctx);
        let motion = drift_motion_inputs(
            self.yaw_ctx,
            self.gps_yaw,
            self.airspeed_tas,
            self.eas2tas,
            &mut self.ahrs.last_gps_fix_ms,
        );
        let (health, attitude) = self.ahrs.update_from_ins(
            &self.ins,
            &self.loop_timing,
            yaw,
            motion,
        );
        self.attitude = attitude;
        self.ahrs_matrix_health = health;
        self.ekf_healthy = self.ahrs.ekf_healthy;
        self.active_ahrs_backend = self.ahrs.active_backend;
        let (n, e, have) = self.ahrs.dead_reckoning_offset();
        self.dead_reckoning_north_m = n;
        self.dead_reckoning_east_m = e;
        self.have_dead_reckoning_position = have;
        self.estimated_wind = self.ahrs.wind_estimate();
        self.head_wind_ms = self.ahrs.head_wind();
    }

    /// Upstream `Plane::update_control_mode`. Dispatches to the active mode.
    pub fn update_control_mode(&mut self) {
        self.ticks.update_control_mode += 1;
        let rc_out = rc_failsafe_scheduler_tick(&self.rc_failsafe_inputs);
        self.in_rc_failsafe = rc_out.in_rc_failsafe;
        self.rc_sticks = rc_out.rc_sticks;

        let mission_out = mission_scheduler_tick(
            &mut self.mission,
            &self.landing,
            &MissionSchedulerInputs {
                control_mode: self.mode.control_mode,
                ..self.mission_inputs
            },
        );
        self.last_target_altitude = mission_out.target;
        self.mission_advanced = mission_out.advanced;

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

            let bump_out = rangefinder_bump_scheduler_tick(
                &mut self.rangefinder_bump,
                &mut self.landing,
                self.flight_stage_is_land,
                &RangefinderBumpSchedulerInputs {
                    hookup: self.rangefinder_bump_inputs,
                },
            );
            self.last_rangefinder_bump_recalculated = bump_out
                .result
                .map(|r| r.recalculated)
                .unwrap_or(false);
            if self.rangefinder_bump.flags.commanded_go_around {
                self.landing_request_go_around = true;
            }
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

        let ds_out = deepstall_override_scheduler_tick(
            &mut self.landing,
            self.servos,
            &DeepstallOverrideSchedulerInputs {
                flight_stage_is_land: self.flight_stage_is_land,
                deepstall: self.deepstall_override,
            },
        );
        self.landing_servo_override_applied = ds_out.applied_override;
        self.landing_request_go_around = ds_out.request_go_around;
        self.servos = ds_out.servos;

        let thr_out = landing_throttle_scheduler_tick(
            self.servos,
            &LandingThrottleSchedulerInputs {
                flight_stage_is_land: self.flight_stage_is_land,
                throttle_suppressed: self.landing_throttle_suppressed,
            },
        );
        self.landing_throttle_applied = thr_out.applied;
        self.servos = thr_out.servos;
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
