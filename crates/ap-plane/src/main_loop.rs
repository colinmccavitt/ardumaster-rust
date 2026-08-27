//! ArduPlane main vehicle loop skeleton, upstream the four fast scheduler
//! tasks in `ArduPlane/Plane.cpp` and `Mode::run` in `ArduPlane/mode.cpp`.
//!
//! `ap-scheduler` owns tick ordering; this module is where the vehicle wires
//! those tasks to mode dispatch and the attitude/servo paths that follow.

use ap_ahrs::{YawCompassSample, YawDriftContext, YawGpsSample};
use ap_math::scalar::degrees;
use ap_ins::sitl::{SitlBodyState, SitlInsInstanceFiles, SITL_INS_MAX_INSTANCES};
use ap_ins::{InertialSensorFrontend, LoopTiming, SitlInsMotorRuntime};
use ap_scheduler::scheduler::{LOOP_RATE, RunStats, Scheduler, Task};

use crate::ahrs_hookup::{drift_motion_inputs, yaw_update_inputs, AhrsAttitude, AhrsFeed};
use crate::ahrs_pre_arm_hookup::plane_pre_arm_checks;
use crate::baro_pre_arm_hookup::{baro_pre_arm_check, plane_pre_arm_checks_baro};
use crate::gps_pre_arm_hookup::{gps_pre_arm_check, plane_pre_arm_checks_gps};
use crate::mode_run::{pre_arm_checks, PreArmResult};
use ap_landing::deepstall_override::DeepstallOverrideInputs;
use ap_landing::landing_state_machine::VerifyLandEffects;
use ap_landing::deepstall_stage::DeepstallStage;
use crate::deepstall_override_scheduler_hookup::{
    deepstall_override_scheduler_tick, DeepstallOverrideSchedulerInputs,
};
use crate::go_around_hookup::apply_landing_go_around_latch;
use crate::landing_hookup::ServoOutputState;
use crate::landing_loop::{LandingContext, VerifyLandVehicleInputs};
use crate::landing_loop_hookup::{landing_loop_scheduler_tick, LandingLoopSchedulerInputs};
use crate::landing_throttle_scheduler_hookup::{
    landing_throttle_scheduler_tick, LandingThrottleSchedulerInputs,
};
use crate::arming_scheduler_hookup::{
    arming_scheduler_tick, ArmingSchedulerInputs,
};
use crate::srv_output_scheduler_hookup::{
    srv_output_scheduler_tick, SrvOutputHookupState, SrvOutputSchedulerInputs,
};

use crate::srv_pwm_publish_hookup::{
    srv_pwm_publish_tick, sync_pwm_channels_from_registry, SrvPwmPublishInputs,
    SrvPwmPublishState,
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
use crate::sitl_baro_hookup::SitlBaroHookup;
use ap_baro::sitl::BaroHealthFlags;
use crate::sitl_gps_hookup::SitlGpsHookup;
use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish};
use crate::entry_state::ModeEntryState;
use crate::mode::ModeState;
use crate::mode_entry_scheduler_hookup::{
    mode_entry_scheduler_tick, ModeEntrySchedulerInputs,
};
use crate::mode_glue_hookup::{
    mode_glue_set_servos_tick, mode_glue_update_control_tick, ModeGlueSetServosInputs,
    ModeGlueUpdateControlInputs,
};
use crate::mode_transition_throttle_hookup::{
    mode_transition_throttle_tick, ModeTransitionThrottleInputs,
};
use crate::throttle_context_hookup::{
    throttle_context_tick, ThrottleContextInputs,
};
use crate::yaw_throttle_glue_hookup::{
    pilot_throttle_glue_tick, vtol_yaw_stick_glue_tick, PilotThrottleGlueInputs,
    VtolYawStickGlueInputs,
};
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
    /// State cleared on each mode change, upstream `Mode::enter`.
    pub mode_entry: ModeEntryState,
    /// Last `control_mode` seen by the mode-entry hookup.
    pub tracked_control_mode: u8,
    /// Whether the latest scheduler tick reset mode-entry state.
    pub mode_entry_reset: bool,
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
    /// Roll attitude radians, upstream `AP_AHRS::get_roll_rad()`.
    pub roll_rad: f32,
    /// Pitch attitude radians, upstream `AP_AHRS::get_pitch_rad()`.
    pub pitch_rad: f32,
    /// Yaw attitude radians, upstream `AP_AHRS::get_yaw_rad()`.
    pub yaw_rad: f32,
    /// Optional compass sample for yaw drift correction.
    pub compass: Option<YawCompassSample>,
    /// Optional GPS sample for yaw drift fallback.
    pub gps_yaw: Option<YawGpsSample>,
    /// Lag-buffered GPS status from the fix producer, upstream `AP_GPS::status()`.
    pub gps_status: Option<ap_gps::GpsStatus>,
    /// Lag-buffered NED velocity for drift, upstream `AP_GPS::state().velocity`.
    pub gps_velocity: Option<ap_gps::GpsVelocitySample>,
    /// GPS health flags from the fix producer, upstream `AP_GPS::isHealthy()`.
    pub gps_health: Option<ap_gps::GpsHealthFlags>,
    /// Whether GPS output is the blended virtual instance.
    pub gps_output_is_blended: bool,
    /// Active GPS instance index (0/1/blended=2).
    pub gps_active_instance: u8,
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
    /// Optional SITL GPS fix producer; takes precedence over manual GPS in `sitl_yaw`.
    pub sitl_gps: Option<SitlGpsHookup>,
    /// Optional SITL baro producer; publishes EAS2TAS each `ahrs_update`.
    pub sitl_baro: Option<SitlBaroHookup>,
    /// Latest baro sample from the SITL backend, upstream `AP_Baro` frontend.
    pub baro_sample: Option<ap_baro::sitl::BaroSampleState>,
    /// Whether the SITL baro backend is healthy, upstream `AP_Baro::healthy()`.
    pub baro_healthy: bool,
    /// Per-instance baro health flags, upstream `AP_Baro` frontend.
    pub baro_health: BaroHealthFlags,
    /// Filtered baro climb rate, upstream `AP_Baro::get_climb_rate()`.
    pub baro_climb_rate_mps: f32,
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
    /// Whether the latest `set_servos` tick latched go-around into landing flags.
    pub last_go_around_latched: bool,
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
    /// SRV registry hookup state for elevon/flap mixing.
    /// Last EKF healthy flag from AHRS update.
    pub ekf_healthy: bool,
    /// Whether NavEKF3 has completed its first update, upstream filter initialised.
    pub ekf3_initialized: bool,
    /// NavEKF3 update count since boot, upstream `_framesSincePredict`.
    pub ekf3_update_count: u32,
    /// Parameter-selected AHRS backend, upstream `AHRS_EKF_TYPE`.
    pub configured_ahrs_backend: ap_ahrs::AhrsBackendKind,
    /// Active AHRS backend kind after fallback resolution.
    pub active_ahrs_backend: ap_ahrs::AhrsBackendKind,
    /// Wind alignment with current yaw heading, upstream `AP_AHRS::wind_alignment`.
    pub wind_alignment: f32,
    /// DCM matrix health from last AHRS update.
    pub ahrs_matrix_health: ap_ahrs::MatrixHealth,
    /// Whether AHRS is healthy for arming, upstream `AP_AHRS::healthy()`.
    pub ahrs_healthy: bool,
    /// Whether GPS velocity is fused, upstream `AP_AHRS::using_gps()`.
    pub ahrs_using_gps: bool,
    /// Pre-arm AHRS gate, upstream `AP_AHRS::pre_arm_check(false)`.
    pub ahrs_pre_arm_ok: bool,
    /// Pre-arm GPS gate, upstream `AP_GPS::isHealthy()` when GPS configured.
    pub gps_pre_arm_ok: bool,
    /// Pre-arm baro gate when SITL baro configured.
    pub baro_pre_arm_ok: bool,
    /// Combined mode + AHRS + GPS + baro pre-arm result for arming.
    pub pre_arm_ok: bool,
    /// Dead-reckoning north offset (m) when GPS absent.
    pub dead_reckoning_north_m: f32,
    /// Dead-reckoning east offset (m) when GPS absent.
    pub dead_reckoning_east_m: f32,
    /// Whether dead-reckoning position is valid.
    pub have_dead_reckoning_position: bool,
    pub srv_output: SrvOutputHookupState,
    /// HAL inputs for SRV output mapping during `set_servos`.
    pub srv_output_inputs: SrvOutputSchedulerInputs,
    /// Output channels for registry PWM publish.
    pub srv_pwm: SrvPwmPublishState,
    pub srv_pwm_inputs: SrvPwmPublishInputs,
    /// Whether the latest `set_servos` tick published registry PWM.
    pub last_pwm_publish_ran: bool,
    /// Auto flap percent from the latest SRV output tick.
    pub last_auto_flap_percent: i8,
    /// `hal.util->get_soft_armed()`.
    pub soft_armed: bool,
    /// Whether disarm zeroed throttle on the last `set_servos`.
    pub disarm_throttle_applied: bool,
    /// Whether mode-entry suppress_throttle zeroed throttle.
    pub mode_entry_throttle_applied: bool,
    /// Whether mode-transition logic cleared throttle suppression.
    pub mode_transition_throttle_cleared: bool,
    /// Whether mode glue zeroed pilot throttle on mode entry.
    pub mode_glue_throttle_zeroed: bool,
    /// Whether set_servos restored pilot throttle after transition clear.
    pub mode_glue_throttle_restored: bool,
    /// Effective stick mixing after mode glue resolution.
    pub effective_stick_mixing: Option<StickMixing>,
    /// Whether configured throttle limits apply this tick.
    pub throttle_use_limits: bool,
    /// Whether battery voltage compensation applies this tick.
    pub throttle_use_battery_comp: bool,
    /// How pilot throttle is mapped this tick.
    pub pilot_throttle_source: crate::mode_run::PilotThrottleSource,
    /// `THR_PASS_STAB` parameter.
    pub throttle_passthru_stabilize: bool,
    /// Guided mode passthrough throttle flag.
    pub guided_throttle_passthru: bool,
    /// Quadplane forward throttle allowed in VTOL.
    pub allow_forward_throttle_in_vtol: bool,
    /// Whether a quadplane is compiled in.
    pub quadplane_available: bool,
    /// `IDLE_GOV_MANUAL` when quadplane present.
    pub idle_gov_manual: bool,
    /// Nav scripting active this tick.
    pub nav_scripting_active: bool,
    /// `TRIM_THROTTLE` parameter, percent.
    pub trim_throttle: f32,
    /// `THR_MIN` parameter, percent.
    pub throttle_min: f32,
    /// `THR_MAX` parameter, percent.
    pub throttle_max: f32,
    /// Pack voltage ratio for battery throttle compensation.
    pub battery_voltage_ratio: f32,
    /// Altitude above home/reference, metres. Upstream `relative_altitude`.
    pub relative_altitude_m: f32,
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
            mode_entry: ModeEntryState {
                auto: crate::entry_state::AutoState {
                    inverted_flight: false,
                    next_wp_crosstrack: false,
                    checked_for_autoland: false,
                    highest_airspeed: 0.0,
                    initial_pitch_cd: 0,
                    fbwa_tdrag_takeoff_mode: false,
                    rotation_complete: false,
                    vtol_mode: false,
                    vtol_loiter: false,
                    idle_mode: false,
                },
                steer: crate::entry_state::SteerState {
                    locked_course: false,
                    locked_course_err: 0.0,
                },
                crash: crate::entry_state::CrashState {
                    is_crashed: false,
                    impact_detected: false,
                },
                waiting_for_rudder_neutral: false,
                loiter_start_time_ms: 0,
                new_airspeed_cm: -1,
                long_failsafe_pending: false,
                throttle_suppressed: false,
            },
            tracked_control_mode: ModeNumber::Initialising.as_number(),
            mode_entry_reset: false,
            stick_mixing: Some(StickMixing::Fbw),
            features: BuildFeatures::default(),
            ticks: FastTaskTicks::default(),
            last_stabilize: StabilizeDispatch::default(),
            last_stabilize_run: StabilizeRun::default(),
            ahrs: AhrsFeed::default(),
            ins: InertialSensorFrontend::default(),
            loop_timing: LoopTiming::new(1.0 / f32::from(LOOP_RATE)),
            attitude: AhrsAttitude::default(),
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            compass: None,
            gps_yaw: None,
            gps_status: None,
            gps_velocity: None,
            gps_health: None,
            gps_output_is_blended: false,
            gps_active_instance: 0,
            yaw_ctx: YawDriftContext::default(),
            airspeed_tas: 0.0,
            eas2tas: 1.0,
            wind_vane: None,
            estimated_wind: ap_math::vector3::Vector3f::zero(),
            head_wind_ms: 0.0,
            sitl_yaw: None,
            sitl_ahrs: None,
            sitl_gps: None,
            sitl_baro: None,
            baro_sample: None,
            baro_healthy: false,
            baro_health: BaroHealthFlags::default(),
            baro_climb_rate_mps: 0.0,
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
            last_go_around_latched: false,
            mission: MissionContext::default(),
            mission_inputs: MissionSchedulerInputs::default(),
            last_target_altitude: TargetAltitude::FromNextWaypoint,
            mission_advanced: false,
            rc_failsafe_inputs: RcFailsafeSchedulerInputs::default(),
            in_rc_failsafe: false,
            ekf_healthy: false,
            ekf3_initialized: false,
            ekf3_update_count: 0,
            configured_ahrs_backend: ap_ahrs::AhrsBackendKind::Dcm,
            active_ahrs_backend: ap_ahrs::AhrsBackendKind::Dcm,
            wind_alignment: 0.0,
            ahrs_matrix_health: ap_ahrs::MatrixHealth::Ok,
            ahrs_healthy: false,
            ahrs_using_gps: false,
            ahrs_pre_arm_ok: false,
            gps_pre_arm_ok: false,
            baro_pre_arm_ok: false,
            pre_arm_ok: false,
            dead_reckoning_north_m: 0.0,
            dead_reckoning_east_m: 0.0,
            have_dead_reckoning_position: false,
            srv_output: SrvOutputHookupState::default(),
            srv_output_inputs: SrvOutputSchedulerInputs {
                dt: 0.02,
                ..SrvOutputSchedulerInputs::default()
            },
            srv_pwm: SrvPwmPublishState::default(),
            srv_pwm_inputs: SrvPwmPublishInputs::default(),
            last_pwm_publish_ran: false,
            last_auto_flap_percent: 0,
            soft_armed: false,
            disarm_throttle_applied: false,
            mode_entry_throttle_applied: false,
            mode_transition_throttle_cleared: false,
            mode_glue_throttle_zeroed: false,
            mode_glue_throttle_restored: false,
            effective_stick_mixing: Some(StickMixing::Fbw),
            throttle_use_limits: true,
            throttle_use_battery_comp: true,
            pilot_throttle_source: crate::mode_run::PilotThrottleSource::TrimAdjusted,
            throttle_passthru_stabilize: false,
            guided_throttle_passthru: false,
            allow_forward_throttle_in_vtol: true,
            quadplane_available: false,
            idle_gov_manual: false,
            nav_scripting_active: false,
            trim_throttle: crate::stabilize_hookup::AP_PLANE_TRIM_THROTTLE_DEFAULT,
            throttle_min: 0.0,
            throttle_max: 100.0,
            battery_voltage_ratio: 1.0,
            relative_altitude_m: 0.0,
            servos: ServoOutputState::default(),
        }
    }
}

impl PlaneMainLoop {
    /// Upstream `Plane::ahrs_update`. Runs INS→DCM and publishes attitude sensors.
    pub fn ahrs_update(&mut self) {
        self.ticks.ahrs_update += 1;
        if let Some(gps) = self.sitl_gps.as_mut() {
            let samples = gps.publish_yaw_samples(
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            self.compass = samples.compass;
            self.gps_yaw = samples.gps_yaw;
            self.yaw_ctx = samples.yaw_ctx;
            self.gps_status = Some(gps.gps_status_publish());
            self.gps_velocity = Some(gps.gps_velocity_publish());
            self.gps_health = Some(gps.gps_health_publish());
            self.gps_output_is_blended = gps.gps_output_is_blended();
            self.gps_active_instance = gps.gps_active_instance();
            if let Some(health) = self.gps_health {
                self.yaw_ctx.have_gps = health.usable_for_drift();
            }
        } else if let Some(source) = self.sitl_ahrs {
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
        if let Some(baro) = self.sitl_baro.as_mut() {
            let published = baro.publish();
            self.baro_sample = Some(published.sample);
            self.baro_healthy = published.healthy;
            self.baro_health = published.health;
            self.baro_climb_rate_mps = published.climb_rate_mps;
            self.eas2tas = published.eas2tas;
        }
        if let Some(vane) = self.wind_vane {
            self.ahrs.apply_wind_vane(vane);
        }
        let yaw = yaw_update_inputs(self.compass, self.gps_yaw, self.yaw_ctx);
        let motion = drift_motion_inputs(
            self.yaw_ctx,
            self.gps_yaw,
            self.gps_velocity,
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
        self.roll_rad = attitude.roll_rad();
        self.pitch_rad = attitude.pitch_rad();
        self.yaw_rad = attitude.yaw_rad();
        self.estimated_wind = self.ahrs.wind_estimate();
        self.ekf_healthy = self.ahrs.ekf_healthy;
        self.ekf3_initialized = self.ahrs.ekf3.initialized;
        self.ekf3_update_count = self.ahrs.ekf3.update_count;
        self.configured_ahrs_backend = self.ahrs.configured_backend;
        self.active_ahrs_backend = self.ahrs.active_backend;
        self.ahrs_matrix_health = health;
        self.ahrs_healthy = self.ahrs.healthy();
        self.ahrs_using_gps = self.ahrs.using_gps();
        self.ahrs_pre_arm_ok = self.ahrs.pre_arm_check(false);
        let (n, e, have) = self.ahrs.dead_reckoning_offset();
        self.dead_reckoning_north_m = n;
        self.dead_reckoning_east_m = e;
        self.have_dead_reckoning_position = have;

        self.head_wind_ms = self.ahrs.head_wind();
        self.wind_alignment = self.ahrs.wind_alignment(degrees(self.attitude.yaw_rad()));
    }

    /// Upstream `Plane::update_control_mode`. Dispatches to the active mode.
    pub fn update_control_mode(&mut self) {
        self.ticks.update_control_mode += 1;
        let entry_out = mode_entry_scheduler_tick(
            &mut self.mode_entry,
            &ModeEntrySchedulerInputs {
                control_mode: self.mode.control_mode,
                previous_tracked_mode: self.tracked_control_mode,
                current_pitch_cd: self.attitude.pitch_sensor_cd as i16,
                features: self.features,
            },
        );
        self.tracked_control_mode = entry_out.tracked_mode;
        self.mode_entry_reset = entry_out.mode_changed;
        let rc_out = rc_failsafe_scheduler_tick(&self.rc_failsafe_inputs);
        self.in_rc_failsafe = rc_out.in_rc_failsafe;
        self.rc_sticks = rc_out.rc_sticks;
        self.srv_output_inputs.manual_flap_percent = rc_out.manual_flap_percent;

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

        let thr_ctx = throttle_context_tick(&ThrottleContextInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            nav_scripting_active: self.nav_scripting_active,
            throttle_passthru_stabilize: self.throttle_passthru_stabilize,
            guided_throttle_passthru: self.guided_throttle_passthru,
            allow_forward_throttle_in_vtol: self.allow_forward_throttle_in_vtol,
            quadplane_available: self.quadplane_available,
            idle_gov_manual: self.idle_gov_manual,
        });
        self.throttle_use_limits = thr_ctx.use_throttle_limits;
        self.throttle_use_battery_comp = thr_ctx.use_battery_compensation;
        self.pilot_throttle_source = thr_ctx.pilot_throttle_source;

        let glue_out = mode_glue_update_control_tick(&ModeGlueUpdateControlInputs {
            pilot_throttle: PilotThrottleGlueInputs {
                throttle_pwm: self.rc_failsafe_inputs.throttle_pwm,
                throttle_cfg: self.rc_failsafe_inputs.throttle_cfg,
                pilot_throttle_source: self.pilot_throttle_source,
                trim_throttle: self.trim_throttle,
                throttle_min: self.throttle_min,
                throttle_max: self.throttle_max,
                use_throttle_limits: self.throttle_use_limits,
                use_battery_compensation: self.throttle_use_battery_comp,
                battery_voltage_ratio: self.battery_voltage_ratio,
            },
            control_mode: self.mode.control_mode,
            features: self.features,
            stick_mixing: self.stick_mixing,
            throttle_suppressed: self.mode_entry.throttle_suppressed,
        });
        self.effective_stick_mixing = glue_out.effective_stick_mixing;
        self.mode_glue_throttle_zeroed = glue_out.throttle_zeroed_by_mode_entry;

        self.last_stabilize = dispatch_stabilize_from_mode(
            self.mode.control_mode,
            self.effective_stick_mixing,
            &self.features,
        );

        self.stabilize_demands.throttle_scaled = glue_out.pilot_throttle;
        self.servos.throttle_scaled = glue_out.pilot_throttle;

        let mode_pre_arm = pre_arm_checks(true, "");
        let with_ahrs = plane_pre_arm_checks(mode_pre_arm, self.ahrs_pre_arm_ok);
        let require_gps = self.sitl_gps.is_some();
        self.gps_pre_arm_ok = if let Some(gps) = self.sitl_gps.as_mut() {
            gps.gps_dual_pre_arm_ok()
        } else {
            gps_pre_arm_check(self.gps_health, require_gps)
        };
        let with_gps = plane_pre_arm_checks_gps(with_ahrs, self.gps_pre_arm_ok);
        let require_baro = self.sitl_baro.is_some();
        self.baro_pre_arm_ok = baro_pre_arm_check(self.baro_health, require_baro);
        self.pre_arm_ok = matches!(
            plane_pre_arm_checks_baro(with_gps, self.baro_pre_arm_ok),
            PreArmResult::Allowed
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
                apply_landing_go_around_latch(
                    &mut self.landing.flags,
                    true,
                );
            }
        }
    }

    /// Upstream `Plane::stabilize`. Calls roll/pitch/yaw controllers when the
    /// active mode selected them on the previous `update_control_mode`.
    pub fn stabilize(&mut self) {
        self.ticks.stabilize += 1;
        feed_nav_commands(&mut self.nav_commands, &self.nav_tecs);
        self.stabilize_ctx.eas2tas = self.eas2tas;
        self.stabilize_ctx.accel_bias_y = self.ahrs.accel_bias().y;
        self.stabilize_ctx.now_ms = self.yaw_ctx.now_ms;
        prepare_stabilize_path(
            &mut self.stabilize_demands,
            &mut self.stabilize_ctx,
            &self.nav_commands,
            &self.speed_scaler_inputs,
            self.last_stabilize,
            &self.rc_sticks,
            self.effective_stick_mixing,
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
        self.stabilize_servos.rudder_scaled = vtol_yaw_stick_glue_tick(
            self.stabilize_servos.rudder_scaled,
            &VtolYawStickGlueInputs {
                stick_mixing: self.effective_stick_mixing,
                yaw_norm_dz: self.rc_sticks.yaw_norm_dz,
                rudder_limit_scaled: 4500.0,
            },
        );
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
        if ds_out.request_go_around {
            self.landing_request_go_around = true;
        }
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

        let srv_out = srv_output_scheduler_tick(
            self.servos,
            &mut self.srv_output,
            &self.srv_output_inputs,
        );
        self.last_auto_flap_percent = srv_out.auto_flap_percent;

        sync_pwm_channels_from_registry(&self.srv_output.registry, &mut self.srv_pwm);
        let pwm_out = srv_pwm_publish_tick(
            &mut self.srv_output.registry,
            &mut self.srv_pwm,
            &self.srv_pwm_inputs,
        );
        self.last_pwm_publish_ran = pwm_out.ran;

        let trans_out = mode_transition_throttle_tick(
            &mut self.mode_entry,
            &ModeTransitionThrottleInputs {
                control_mode: self.mode.control_mode,
                relative_altitude_m: self.relative_altitude_m,
                gps: self.gps_status,
                features: self.features,
            },
        );
        self.mode_transition_throttle_cleared = trans_out.cleared;

        let pilot_throttle = pilot_throttle_glue_tick(&PilotThrottleGlueInputs {
            throttle_pwm: self.rc_failsafe_inputs.throttle_pwm,
            throttle_cfg: self.rc_failsafe_inputs.throttle_cfg,
            pilot_throttle_source: self.pilot_throttle_source,
            trim_throttle: self.trim_throttle,
            throttle_min: self.throttle_min,
            throttle_max: self.throttle_max,
            use_throttle_limits: self.throttle_use_limits,
            use_battery_compensation: self.throttle_use_battery_comp,
            battery_voltage_ratio: self.battery_voltage_ratio,
        });
        let glue_servos_out = mode_glue_set_servos_tick(
            self.servos,
            &ModeGlueSetServosInputs {
                control_mode: self.mode.control_mode,
                features: self.features,
                transition_cleared: trans_out.cleared,
                throttle_suppressed: self.mode_entry.throttle_suppressed,
                current_throttle: self.servos.throttle_scaled,
                pilot_throttle,
            },
        );
        self.mode_glue_throttle_restored = glue_servos_out.throttle_restored;
        if glue_servos_out.clear_throttle_zeroed {
            self.mode_glue_throttle_zeroed = false;
        }
        if let Some(throttle) = glue_servos_out.stabilize_throttle {
            self.stabilize_demands.throttle_scaled = throttle;
        }
        self.mode_entry_throttle_applied = glue_servos_out.mode_entry_applied;
        self.servos = glue_servos_out.servos;

        let arm_out = arming_scheduler_tick(
            self.servos,
            &ArmingSchedulerInputs {
                soft_armed: self.soft_armed,
            },
        );
        self.disarm_throttle_applied = arm_out.applied;
        self.servos = arm_out.servos;

        self.last_go_around_latched = apply_landing_go_around_latch(
            &mut self.landing.flags,
            self.landing_request_go_around,
        );
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
