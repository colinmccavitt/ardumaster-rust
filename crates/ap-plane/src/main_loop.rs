//! ArduPlane main vehicle loop skeleton, upstream the four fast scheduler
//! tasks in `ArduPlane/Plane.cpp` and `Mode::run` in `ArduPlane/mode.cpp`.
//!
//! `ap-scheduler` owns tick ordering; this module is where the vehicle wires
//! those tasks to mode dispatch and the attitude/servo paths that follow.

use ap_ahrs::{YawCompassSample, YawDriftContext, YawGpsSample};
use ap_math::scalar::{degrees, safe_sqrt};
use ap_ins::sitl::{SitlBodyState, SitlInsInstanceFiles, SITL_INS_MAX_INSTANCES};
use ap_ins::{InertialSensorFrontend, LoopTiming, SitlInsMotorRuntime};
use ap_scheduler::scheduler::{LOOP_RATE, RunStats, Scheduler, Task};
use ap_tecs::params::FlightStage;
use ap_tecs::tecs::Tecs;

use crate::ahrs_hookup::{drift_motion_inputs, yaw_update_inputs, AhrsAttitude, AhrsFeed};
use crate::ahrs_pre_arm_hookup::plane_pre_arm_checks;
use crate::baro_arm_calibration_hookup::BaroArmCalibrationInputs;
use crate::baro_pre_arm_hookup::{baro_pre_arm_check, plane_pre_arm_checks_baro};
use crate::compass_pre_arm_hookup::{compass_pre_arm_check, plane_pre_arm_checks_compass};
use crate::airspeed_pre_arm_hookup::{airspeed_pre_arm_check, plane_pre_arm_checks_airspeed};
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
use crate::mission_alt_offset_glue_hookup::{mission_alt_offset_glue_tick, MissionAltOffsetGlueInputs};
use crate::rangefinder_correction_glue_hookup::{rangefinder_correction_glue_tick, rangefinder_correction_glue_inputs};
use crate::target_altitude::TargetAltitude;
use crate::altitude_glue_hookup::{altitude_glue_tick, AltitudeGlueInputs};
use crate::altitude_tecs_feed_hookup::{altitude_tecs_feed_tick, AltitudeTecsFeedInputs};
use crate::set_servos_glue_hookup::{set_servos_calc_throttle_tick, SetServosGlueInputs};
use crate::tecs_baro_hookup::{tecs_baro_feed_tick, TecsBaroFeed, TecsBaroInputs};
use crate::calc_throttle_glue_hookup::{calc_throttle_glue_tick, CalcThrottleGlueInputs};
use crate::nav_tecs_hookup::{feed_nav_commands, NavTecsPublish};
use crate::nav_tecs_scheduler_hookup::nav_tecs_scheduler_publish_tick;
use crate::navigation_scheduler_hookup::{
    navigation_scheduler_tick, NavigationSchedulerInputs, NavigationSchedulerOutput,
};
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
use crate::sitl_compass_hookup::SitlCompassHookup;
use crate::compass_health_scheduler_hookup::{
    compass_health_scheduler_tick, CompassHealthSchedulerInputs,
};
use crate::compass_offset_calibration_hookup::{
    compass_offset_calibration_tick, CompassOffsetCalibrationInputs,
};
use crate::compass_offset_persist_hookup::{
    compass_offset_persist_tick, CompassOffsetPersistInputs,
};
use crate::compass_motor_compensation_hookup::{
    compass_motor_compensation_tick, CompassMotorCompensationInputs,
};
use crate::airspeed_health_scheduler_hookup::{
    airspeed_health_scheduler_tick, AirspeedHealthSchedulerInputs,
};
use crate::airspeed_offset_calibration_hookup::{
    airspeed_offset_calibration_tick, AirspeedOffsetCalibrationInputs,
};
use crate::airspeed_analog_hookup::AirspeedAnalogHookup;
use crate::airspeed_type_hookup::select_airspeed_backend;
use crate::sitl_airspeed_hookup::SitlAirspeedHookup;
use crate::airspeed_tecs_health_hookup::publish_airspeed_for_tecs;
use ap_airspeed::backend::{AirspeedBackendKind, ARSPD_TYPE_SITL};
use ap_airspeed::sitl::AirspeedSampleState;
use ap_airspeed::sitl::AirspeedHealthFlags;
use ap_airspeed::sitl::{
    tas_for_nav, ARSPD_AUTOCAL_DEFAULT, ARSPD_RATIO_DEFAULT, ARSPD_SKIP_CAL_DEFAULT,
    ARSPD_TEMP_REF_C, ARSPD_USE_DEFAULT,
};
use ap_airspeed::bus::ARSPD_BUS_DEFAULT;
use ap_airspeed::devid::ARSPD_DEVID_DEFAULT;
use ap_airspeed::options::ARSPD_OPTIONS_DEFAULT;
use ap_airspeed::wind_max::ARSPD_WIND_MAX_DEFAULT;
use ap_airspeed::wind_warn::ARSPD_WIND_WARN_DEFAULT;
use ap_airspeed::primary::ARSPD_PRIMARY_DEFAULT;
use ap_airspeed::fbw::{ARSPD_FBW_MAX_DEFAULT, ARSPD_FBW_MIN_DEFAULT};
use ap_airspeed::psi_range::{clamp_psi_range, ARSPD_PSI_RANGE_DEFAULT};
use ap_airspeed::tube_order::ARSPD_TUBE_ORDER_DEFAULT;
use ap_baro::sitl::BaroHealthFlags;
use ap_compass::sitl::{CompassHealthFlags, MagSampleState};
use crate::sitl_gps_hookup::SitlGpsHookup;
use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish};
use crate::entry_state::ModeEntryState;
use crate::mode::ModeState;
use crate::mode_entry_scheduler_hookup::{
    mode_entry_scheduler_tick, ModeEntrySchedulerInputs,
};
use crate::fbwa_mode_hookup::{fbwa_mode_nav_tick, FbwaModeNavInputs};
use crate::manual_mode_hookup::{
    manual_mode_nav_tick, manual_mode_servos_tick, ManualModeNavInputs,
    ManualModeServosInputs,
};
use crate::stabilize_mode_hookup::{stabilize_mode_nav_tick, StabilizeModeNavInputs};
use crate::acro_mode_hookup::{acro_mode_nav_tick, AcroModeNavInputs};
use crate::training_mode_hookup::{training_mode_nav_tick, TrainingModeNavInputs};
use crate::fbwb_mode_hookup::{fbwb_mode_nav_tick, FbwbModeNavInputs};
use crate::cruise_mode_hookup::{cruise_mode_nav_tick, CruiseModeNavInputs};
use crate::autotune_mode_hookup::{autotune_mode_nav_tick, AutotuneModeNavInputs};
use crate::circle_mode_hookup::{circle_mode_nav_tick, CircleModeNavInputs};
use crate::thermal_mode_hookup::{thermal_mode_nav_tick, ThermalModeNavInputs};
use crate::auto_mode_hookup::{
    auto_mode_complete_tick, auto_mode_mission_tick, AutoModeCompleteInputs,
    AutoModeMissionInputs,
};
use crate::rtl_mode_hookup::{rtl_mode_climb_tick, rtl_mode_nav_tick, RtlModeClimbInputs, RtlModeNavInputs};
use crate::loiter_mode_hookup::{loiter_mode_nav_tick, LoiterModeNavInputs};
use crate::guided_mode_hookup::{guided_mode_nav_tick, GuidedModeNavInputs};
use crate::takeoff_mode_hookup::{takeoff_mode_nav_tick, TakeoffModeNavInputs};
use crate::autoland_mode_hookup::{autoland_mode_nav_tick, AutolandModeNavInputs};
use crate::avoid_adsb_mode_hookup::{avoid_adsb_mode_nav_tick, AvoidAdsbModeNavInputs};
use crate::mode_glue_hookup::{
    mode_glue_set_servos_tick, mode_glue_stabilize_tick, mode_glue_update_control_tick,
    ModeGlueSetServosInputs, ModeGlueStabilizeInputs, ModeGlueUpdateControlInputs,
};
use crate::mode_transition_throttle_hookup::{
    mode_transition_throttle_tick, ModeTransitionThrottleInputs,
};
use crate::throttle_context_hookup::{
    throttle_context_tick, ThrottleContextInputs,
};
use crate::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;
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
#[derive(Debug)]
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
    /// Optional SITL compass producer; publishes mag samples each `ahrs_update`.
    pub sitl_compass: Option<SitlCompassHookup>,
    /// Optional SITL airspeed producer; publishes pitot TAS each `ahrs_update`.
    pub sitl_airspeed: Option<SitlAirspeedHookup>,
    /// Latest pitot sample from the SITL backend, upstream `AP_Airspeed::get_airspeed()`.
    pub airspeed_sample: Option<AirspeedSampleState>,
    /// Whether the SITL airspeed backend is healthy, upstream `AP_Airspeed::healthy()`.
    pub airspeed_healthy: bool,
    /// Per-instance airspeed health flags, upstream `AP_Airspeed` frontend.
    pub airspeed_health: AirspeedHealthFlags,
    /// Request pitot offset calibration on the next `ahrs_update`.
    pub airspeed_calibrate_requested: bool,
    /// True after a successful pitot offset latch, upstream `calibrate()`.
    pub airspeed_offset_calibrated: bool,
    /// Primary pitot tube ratio, upstream `ARSPD_RATIO`.
    pub airspeed_ratio: f32,
    /// Primary `ARSPD_USE`, upstream `AP_Airspeed` use param.
    pub airspeed_use: u8,
    /// Whether TAS is used for TECS/nav, upstream `AP_Airspeed::use()`.
    pub airspeed_use_for_control: bool,
    /// Primary pitot / ISA temperature (deg C), upstream `get_temperature()`.
    pub airspeed_temperature_c: f32,
    /// Primary `ARSPD_AUTOCAL`, upstream automatic pitot-ratio calibration.
    pub airspeed_autocal: u8,
    /// Primary ARSPD_SKIP_CAL, skip startup / requested pitot offset cal.
    pub airspeed_skip_cal: bool,
    /// Pitot connector order, upstream `ARSPD_TUBE_ORDER`.
    pub airspeed_tube_order: u8,
    /// I2C bus, upstream `ARSPD_BUS`.
    pub airspeed_bus: u8,
    /// Sensor device ID, upstream `ARSPD_DEVID`.
    pub airspeed_devid: i32,
    /// Vehicle-level bitmask, upstream `ARSPD_OPTIONS`.
    pub airspeed_options: u32,
    /// Max |airspeed-groundspeed| (m/s), upstream `ARSPD_WIND_MAX`.
    pub airspeed_wind_max: f32,
    /// True when the enabled WIND_MAX check fails.
    pub airspeed_wind_max_exceeded: bool,
    /// Airspeed-vs-wind warning (m/s), upstream `ARSPD_WIND_WARN`.
    pub airspeed_wind_warn: f32,
    /// True when the enabled WIND_WARN threshold is exceeded.
    pub airspeed_wind_warn_exceeded: bool,
    /// Preferred instance, upstream `ARSPD_PRIMARY`.
    pub airspeed_primary: u8,
    /// FBW minimum airspeed (m/s), upstream `ARSPD_FBW_MIN` / `AIRSPEED_MIN`.
    pub airspeed_fbw_min: f32,
    /// FBW maximum airspeed (m/s), upstream `ARSPD_FBW_MAX` / `AIRSPEED_MAX`.
    pub airspeed_fbw_max: f32,
    /// Primary PSI full-scale (clamped), upstream `ARSPD_PSI_RANGE`.
    pub airspeed_psi_range: f32,
    /// Primary `ARSPD_TYPE`, upstream `AP_Airspeed` type param.
    pub airspeed_type: u8,
    /// Configured airspeed backend, upstream `AP_Airspeed::airspeed_type`.
    pub configured_airspeed_backend: AirspeedBackendKind,
    /// Active airspeed backend after unported-type fallback.
    pub active_airspeed_backend: AirspeedBackendKind,
    /// Optional analog airspeed backend, upstream `AP_Airspeed_Analog`.
    pub analog_airspeed: Option<AirspeedAnalogHookup>,
    /// Primary analog pin, upstream `ARSPD_PIN`.
    pub airspeed_pin: i8,
    /// Latest analog differential pressure (Pa).
    pub airspeed_diff_pressure_pa: f32,
    /// Whether the analog backend returned a pressure this tick.
    pub airspeed_analog_have_pressure: bool,
    /// Last TECS `use_airspeed` feed, upstream `TECS_controller.use_airspeed()`.
    pub last_tecs_use_airspeed: bool,
    /// Latest mag sample from the SITL backend, upstream `AP_Compass::get_field()`.
    pub mag_sample: Option<MagSampleState>,
    /// Whether the SITL compass backend is healthy, upstream `AP_Compass::healthy()`.
    pub compass_healthy: bool,
    /// Per-instance compass health flags, upstream `AP_Compass` frontend.
    pub compass_health: CompassHealthFlags,
    /// Request a `COMPASS_OFS` learn this `ahrs_update`, upstream `COMPASS_LEARN`.
    pub compass_learn_requested: bool,
    /// True after the last requested mag offset learn succeeded.
    pub compass_offsets_learned: bool,
    /// Request a `COMPASS_OFS` persist this `ahrs_update`, upstream `save_offsets`.
    pub compass_save_offsets_requested: bool,
    /// True after the last requested mag offset persist succeeded.
    pub compass_offsets_saved: bool,
    /// Battery current for `COMPASS_MOT`, upstream `AP_BattMonitor::current_amps`.
    pub compass_battery_current_amps: f32,
    /// Latest baro sample from the SITL backend, upstream `AP_Baro` frontend.
    pub baro_sample: Option<ap_baro::sitl::BaroSampleState>,
    /// Whether the SITL baro backend is healthy, upstream `AP_Baro::healthy()`.
    pub baro_healthy: bool,
    /// Per-instance baro health flags, upstream `AP_Baro` frontend.
    pub baro_health: BaroHealthFlags,
    /// Filtered baro climb rate, upstream `AP_Baro::get_climb_rate()`.
    pub baro_climb_rate_mps: f32,
    pub tecs_baro_feed: TecsBaroFeed,
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
    /// HAL inputs for navigation scheduler tick glue.
    pub navigation_scheduler_inputs: NavigationSchedulerInputs,
    /// TECS controller, upstream `TECS_controller`.
    pub tecs: Tecs,
    /// TECS throttle demand 0..100 for calc_throttle glue.
    pub tecs_throttle_demand: f32,
    /// Throttle nudge from mission/GCS.
    pub throttle_nudge: i16,
    pub target_airspeed_cm: f32,
    pub mission_alt_offset_cm: i32,
    pub rangefinder_correction_m: f32,
    pub next_wp_alt_m: f32,
    pub tecs_flight_stage: FlightStage,
    pub last_altitude_tecs_ran: bool,
    pub last_set_servos_calc_throttle: bool,
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
    /// Upstream AP::ahrs().home_is_set() — ModeAuto::navigate gates on this.
    pub home_is_set: bool,
    /// Whether AUTO mission start/advance glue ran this tick.
    pub auto_mode_mission_applied: bool,
    /// Whether ModeAuto::_enter start_or_resume armed the mission this tick.
    pub auto_mode_mission_started: bool,
    /// Whether AUTO navigate allowed a mission item advance this tick.
    pub auto_mode_mission_advanced: bool,
    /// Whether AUTO mission-complete / landing handoff glue ran this tick.
    pub auto_mode_complete_applied: bool,
    /// Whether exit_mission_callback / AUTO-without-mission would switch to RTL.
    pub auto_mode_switch_to_rtl: bool,
    /// Whether ModeAuto hands the current NAV_LAND command to landing.
    pub auto_mode_land_handoff: bool,
    /// Upstream current nav command is MAV_CMD_NAV_LAND.
    pub auto_current_nav_is_land: bool,
    /// Upstream RTL_RADIUS, metres. Negative is CCW; zero uses WP_LOITER_RAD.
    pub rtl_radius_m: i16,
    /// Whether RTL enter/navigate glue ran this tick.
    pub rtl_mode_nav_applied: bool,
    /// Whether ModeRTL::_enter armed do_RTL this tick.
    pub rtl_mode_started: bool,
    /// Whether RTL navigate is allowed to call update_loiter this tick.
    pub rtl_mode_loiter_allowed: bool,
    /// abs(RTL_RADIUS) applied this tick.
    pub rtl_loiter_radius_m: u16,
    /// RTL_RADIUS < 0 selects counterclockwise loiter.
    pub rtl_loiter_ccw: bool,
    /// Upstream FlightOptions::CLIMB_BEFORE_TURN. Overrides RTL_CLIMB_MIN.
    pub rtl_climb_before_turn: bool,
    /// Upstream `g2.rtl_climb_min`, metres. Zero disables the climb-min gate.
    pub rtl_climb_min_m: u16,
    /// Upstream `current_loc.alt` for the RTL climb gate, centimetres.
    pub rtl_current_alt_cm: i32,
    /// Upstream `next_WP_loc.alt` (RTL altitude), centimetres.
    pub rtl_next_wp_alt_cm: i32,
    /// Upstream `prev_WP_loc.alt` at RTL enter, centimetres.
    pub rtl_prev_wp_alt_cm: i32,
    /// Upstream `rtl.done_climb`.
    pub rtl_done_climb: bool,
    /// Whether RTL climb-then-home remaining-leg glue ran this tick.
    pub rtl_mode_climb_applied: bool,
    /// Climb gate is enabled this tick (`CLIMB_BEFORE_TURN` or RTL_CLIMB_MIN).
    pub rtl_climb_gated: bool,
    /// ModeRTL::update still constrains roll to LEVEL_ROLL_LIMIT.
    pub rtl_climb_constrain_roll: bool,
    /// Climb completed this tick: prev_WP = current, setup_alt_slope.
    pub rtl_setup_remaining_leg: bool,
    /// Upstream WP_LOITER_RAD (aparm.loiter_radius), metres. Negative is CCW.
    pub wp_loiter_rad_m: i16,
    /// Upstream FlightOptions::ENABLE_LOITER_ALT_CONTROL.
    pub loiter_alt_control_enabled: bool,
    /// Whether LOITER enter/navigate glue ran this tick.
    pub loiter_mode_nav_applied: bool,
    /// Whether ModeLoiter::_enter armed do_loiter_at_location this tick.
    pub loiter_mode_started: bool,
    /// Whether LOITER navigate is allowed to call update_loiter this tick.
    pub loiter_mode_loiter_allowed: bool,
    /// abs(WP_LOITER_RAD) applied this tick.
    pub loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise loiter.
    pub loiter_ccw: bool,
    /// Stick mixing plus ENABLE_LOITER_ALT_CONTROL: FBWB-style altitude.
    pub loiter_alt_control: bool,
    /// Upstream ModeGuided::active_radius_m. Zero uses WP_LOITER_RAD.
    pub guided_active_radius_m: u16,
    /// Whether GUIDED enter/navigate glue ran this tick.
    pub guided_mode_nav_applied: bool,
    /// Whether ModeGuided::_enter armed set_guided_WP this tick.
    pub guided_mode_started: bool,
    /// Whether GUIDED navigate is allowed to call update_loiter this tick.
    pub guided_mode_loiter_allowed: bool,
    /// active_radius_m applied this tick (0 after enter).
    pub guided_loiter_radius_m: u16,
    /// GUIDED loiter direction applied this tick.
    pub guided_loiter_ccw: bool,
    /// Whether AVOID_ADSB enter/navigate glue ran this tick.
    pub avoid_adsb_mode_nav_applied: bool,
    /// Whether ModeAvoidADSB::_enter armed ModeGuided::_enter this tick.
    pub avoid_adsb_mode_started: bool,
    /// Whether AVOID_ADSB navigate is allowed to call update_loiter this tick.
    pub avoid_adsb_mode_loiter_allowed: bool,
    /// Always 0 when applied: navigate calls update_loiter(0).
    pub avoid_adsb_loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise avoidance loiter.
    pub avoid_adsb_loiter_ccw: bool,
    /// Upstream TKOFF_ALT, metres.
    pub takeoff_target_alt_m: u16,
    /// Upstream TKOFF_DIST, metres.
    pub takeoff_target_dist_m: u16,
    /// Upstream `plane.current_loc.initialised()`.
    pub current_loc_initialised: bool,
    /// Whether TAKEOFF enter/navigate glue ran this tick.
    pub takeoff_mode_nav_applied: bool,
    /// Whether ModeTakeoff::_enter cleared takeoff_mode_setup this tick.
    pub takeoff_mode_started: bool,
    /// Whether TAKEOFF update may place the climb/loiter waypoint this tick.
    pub takeoff_mode_setup_allowed: bool,
    /// Whether TAKEOFF navigate is allowed to call update_loiter this tick.
    pub takeoff_mode_loiter_allowed: bool,
    /// Always 0 when applied: navigate calls update_loiter(0).
    pub takeoff_loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise takeoff loiter.
    pub takeoff_loiter_ccw: bool,
    /// Upstream plane.is_flying() — ModeAutoLand::_enter requires it.
    pub is_flying: bool,
    /// Upstream 	akeoff_state.initial_direction.initialized.
    pub takeoff_direction_initialized: bool,
    /// Upstream AUTOLAND_WP_ALT, metres.
    pub autoland_wp_alt_m: u16,
    /// Upstream AUTOLAND_WP_DIST, metres.
    pub autoland_wp_dist_m: u16,
    /// Upstream AUTOLAND_CLIMB (terrain_alt_min), metres.
    pub autoland_terrain_alt_min_m: u16,
    /// True when climb-above-terrain is still required.
    pub autoland_need_climb: bool,
    /// Landing type is deepstall (skips loiter-to-alt).
    pub autoland_landing_is_deepstall: bool,
    /// Current AutoLandStage (0 climb, 1 loiter, 2 landing).
    pub autoland_stage: u8,
    /// Climb stage finished this tick.
    pub autoland_climb_complete: bool,
    /// verify_loiter_to_alt completed this tick.
    pub autoland_loiter_to_alt_complete: bool,
    /// Whether AUTOLAND enter/navigate glue ran this tick.
    pub autoland_mode_nav_applied: bool,
    /// Whether ModeAutoLand::_enter succeeded this tick.
    pub autoland_mode_started: bool,
    /// Whether ModeAutoLand::_enter refused this tick.
    pub autoland_mode_refused: bool,
    /// Whether AUTOLAND navigate may call update_loiter this tick.
    pub autoland_mode_loiter_allowed: bool,
    /// Whether AUTOLAND navigate verifies NAV_LAND this tick.
    pub autoland_mode_land_allowed: bool,
    /// update() applies LEVEL_ROLL_LIMIT during CLIMB.
    pub autoland_apply_level_roll: bool,
    /// _enter / climb-complete set next_wp_crosstrack.
    pub autoland_next_wp_crosstrack: bool,
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
    /// Pre-arm compass gate when SITL compass configured.
    pub compass_pre_arm_ok: bool,
    /// Pre-arm airspeed gate, upstream AP_Airspeed::healthy().
    pub airspeed_pre_arm_ok: bool,
    /// Ground pressure latched on the latest arm rising edge.
    pub baro_arm_calibration_latched: bool,
    /// Previous soft_armed for baro arm-calibration edge detect.
    baro_was_soft_armed: bool,
    /// Combined mode + AHRS + GPS + baro + compass pre-arm result for arming.
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
    /// Whether manual-mode nav mirror ran this tick.
    pub manual_mode_nav_applied: bool,
    /// Whether FBWA nav stick mapping ran this tick.
    pub fbwa_mode_nav_applied: bool,
    /// Whether Stabilize nav zeroing ran this tick.
    pub stabilize_mode_nav_applied: bool,
    /// Whether Acro nav lock/mirror ran this tick.
    pub acro_mode_nav_applied: bool,
    /// Whether Training envelope-limit nav ran this tick.
    pub training_mode_nav_applied: bool,
    /// Whether FBWB cruise-assisted nav roll mapping ran this tick.
    pub fbwb_mode_nav_applied: bool,
    /// Whether CRUISE heading-lock nav roll mapping ran this tick.
    pub cruise_mode_nav_applied: bool,
    /// Whether AUTOTUNE FBWA-delegated nav stick mapping ran this tick.
    pub autotune_mode_nav_applied: bool,
    /// Whether CIRCLE loiter-assisted nav roll ran this tick.
    pub circle_mode_nav_applied: bool,
    /// Whether THERMAL soaring-assisted nav roll ran this tick.
    pub thermal_mode_nav_applied: bool,
    /// Upstream `SOAR_THML_BANK`, degrees.
    pub thermal_bank_deg: f32,
    /// Upstream `ModeCruise::locked_heading`.
    pub cruise_locked_heading: bool,
    /// Upstream `training_manual_roll`.
    pub training_manual_roll: bool,
    /// Upstream `training_manual_pitch`.
    pub training_manual_pitch: bool,
    /// Upstream `acro_state.locked_roll`.
    pub acro_locked_roll: bool,
    /// Upstream `acro_state.locked_pitch`.
    pub acro_locked_pitch: bool,
    /// Upstream `acro_state.locked_roll_err`.
    pub acro_locked_roll_err: f32,
    /// Upstream `acro_state.locked_pitch_cd`.
    pub acro_locked_pitch_cd: i32,
    /// Whether manual-mode RC passthrough ran in set_servos.
    pub manual_mode_servos_applied: bool,
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
    /// Home/reference AMSL altitude for baro fallback, upstream `home.alt`.
    pub home_altitude_m: f32,
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
            sitl_compass: None,
            sitl_airspeed: None,
            airspeed_sample: None,
            airspeed_healthy: false,
            airspeed_health: AirspeedHealthFlags::default(),
            airspeed_calibrate_requested: false,
            airspeed_offset_calibrated: false,
            airspeed_ratio: ARSPD_RATIO_DEFAULT,
            airspeed_use: ARSPD_USE_DEFAULT,
            airspeed_use_for_control: true,
            airspeed_temperature_c: ARSPD_TEMP_REF_C,
            airspeed_autocal: ARSPD_AUTOCAL_DEFAULT,
            airspeed_skip_cal: ARSPD_SKIP_CAL_DEFAULT,
            airspeed_tube_order: ARSPD_TUBE_ORDER_DEFAULT,
            airspeed_bus: ARSPD_BUS_DEFAULT,
            airspeed_devid: ARSPD_DEVID_DEFAULT,
            airspeed_options: ARSPD_OPTIONS_DEFAULT,
            airspeed_wind_max: ARSPD_WIND_MAX_DEFAULT,
            airspeed_wind_max_exceeded: false,
            airspeed_wind_warn: ARSPD_WIND_WARN_DEFAULT,
            airspeed_wind_warn_exceeded: false,
            airspeed_primary: ARSPD_PRIMARY_DEFAULT,
            airspeed_fbw_min: ARSPD_FBW_MIN_DEFAULT,
            airspeed_fbw_max: ARSPD_FBW_MAX_DEFAULT,
            airspeed_psi_range: ARSPD_PSI_RANGE_DEFAULT,
            airspeed_type: ARSPD_TYPE_SITL,
            configured_airspeed_backend: AirspeedBackendKind::Sitl,
            active_airspeed_backend: AirspeedBackendKind::Sitl,
            analog_airspeed: None,
            airspeed_pin: 0,
            airspeed_diff_pressure_pa: 0.0,
            airspeed_analog_have_pressure: false,
            last_tecs_use_airspeed: false,
            baro_sample: None,
            mag_sample: None,
            compass_healthy: false,
            compass_health: CompassHealthFlags::default(),
            compass_learn_requested: false,
            compass_offsets_learned: false,
            compass_save_offsets_requested: false,
            compass_offsets_saved: false,
            compass_battery_current_amps: 0.0,
            baro_healthy: false,
            baro_health: BaroHealthFlags::default(),
            baro_climb_rate_mps: 0.0,
            tecs_baro_feed: TecsBaroFeed::default(),
            sitl_ins_noise: None,
            ins_hntch: None,
            sitl_ins_motor: SitlInsMotorRuntime::default(),
            sitl_body: SitlBodyState::default(),
            sitl_now_us: 0,
            sitl_ins_host_files: [SitlInsHostFiles::default(); SITL_INS_MAX_INSTANCES],
            controllers: StabilizeControllers::default(),
            navigation_scheduler_inputs: NavigationSchedulerInputs::default(),
            tecs: Tecs::default(),
            tecs_throttle_demand: 0.0,
            throttle_nudge: 0,
            target_airspeed_cm: 1500.0,
            mission_alt_offset_cm: 0,
            rangefinder_correction_m: 0.0,
            next_wp_alt_m: 0.0,
            tecs_flight_stage: FlightStage::Normal,
            last_altitude_tecs_ran: false,
            last_set_servos_calc_throttle: false,
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
            home_is_set: true,
            auto_mode_mission_applied: false,
            auto_mode_mission_started: false,
            auto_mode_mission_advanced: false,
            auto_mode_complete_applied: false,
            auto_mode_switch_to_rtl: false,
            auto_mode_land_handoff: false,
            auto_current_nav_is_land: false,
            rtl_radius_m: 0,
            rtl_mode_nav_applied: false,
            rtl_mode_started: false,
            rtl_mode_loiter_allowed: false,
            rtl_loiter_radius_m: 0,
            rtl_loiter_ccw: false,
            rtl_climb_before_turn: false,
            rtl_climb_min_m: 0,
            rtl_current_alt_cm: 0,
            rtl_next_wp_alt_cm: 0,
            rtl_prev_wp_alt_cm: 0,
            rtl_done_climb: false,
            rtl_mode_climb_applied: false,
            rtl_climb_gated: false,
            rtl_climb_constrain_roll: false,
            rtl_setup_remaining_leg: false,
            wp_loiter_rad_m: 0,
            loiter_alt_control_enabled: false,
            loiter_mode_nav_applied: false,
            loiter_mode_started: false,
            loiter_mode_loiter_allowed: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            loiter_alt_control: false,
            guided_active_radius_m: 0,
            guided_mode_nav_applied: false,
            guided_mode_started: false,
            guided_mode_loiter_allowed: false,
            guided_loiter_radius_m: 0,
            guided_loiter_ccw: false,
            avoid_adsb_mode_nav_applied: false,
            avoid_adsb_mode_started: false,
            avoid_adsb_mode_loiter_allowed: false,
            avoid_adsb_loiter_radius_m: 0,
            avoid_adsb_loiter_ccw: false,
            takeoff_target_alt_m: crate::takeoff_mode_hookup::TKOFF_ALT_DEFAULT_M,
            takeoff_target_dist_m: crate::takeoff_mode_hookup::TKOFF_DIST_DEFAULT_M,
            current_loc_initialised: true,
            takeoff_mode_nav_applied: false,
            takeoff_mode_started: false,
            takeoff_mode_setup_allowed: false,
            takeoff_mode_loiter_allowed: false,
            takeoff_loiter_radius_m: 0,
            takeoff_loiter_ccw: false,
            is_flying: false,
            takeoff_direction_initialized: false,
            autoland_wp_alt_m: crate::autoland_mode_hookup::AUTOLAND_WP_ALT_DEFAULT_M,
            autoland_wp_dist_m: crate::autoland_mode_hookup::AUTOLAND_WP_DIST_DEFAULT_M,
            autoland_terrain_alt_min_m: 0,
            autoland_need_climb: false,
            autoland_landing_is_deepstall: false,
            autoland_stage: crate::autoland_mode_hookup::STAGE_LOITER,
            autoland_climb_complete: false,
            autoland_loiter_to_alt_complete: false,
            autoland_mode_nav_applied: false,
            autoland_mode_started: false,
            autoland_mode_refused: false,
            autoland_mode_loiter_allowed: false,
            autoland_mode_land_allowed: false,
            autoland_apply_level_roll: false,
            autoland_next_wp_crosstrack: false,
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
            compass_pre_arm_ok: false,
            airspeed_pre_arm_ok: false,
            baro_arm_calibration_latched: false,
            baro_was_soft_armed: false,
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
            manual_mode_nav_applied: false,
            fbwa_mode_nav_applied: false,
            stabilize_mode_nav_applied: false,
            acro_mode_nav_applied: false,
            training_mode_nav_applied: false,
            fbwb_mode_nav_applied: false,
            cruise_mode_nav_applied: false,
            autotune_mode_nav_applied: false,
            circle_mode_nav_applied: false,
            thermal_mode_nav_applied: false,
            thermal_bank_deg: crate::thermal_mode_hookup::SOAR_THML_BANK_DEFAULT_DEG,
            cruise_locked_heading: false,
            training_manual_roll: false,
            training_manual_pitch: false,
            acro_locked_roll: false,
            acro_locked_pitch: false,
            acro_locked_roll_err: 0.0,
            acro_locked_pitch_cd: 0,
            manual_mode_servos_applied: false,
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
            home_altitude_m: 0.0,
            servos: ServoOutputState::default(),
        }
    }
}

impl PlaneMainLoop {
    /// Whether TECS should use the airspeed throttle path, upstream `use_airspeed()`.
    fn tecs_use_airspeed(&self) -> bool {
        let healthy = if self.sitl_airspeed.is_some() {
            self.airspeed_healthy
        } else {
            true
        };
        let gated = publish_airspeed_for_tecs(
            self.airspeed_tas,
            healthy,
            self.airspeed_use_for_control,
        );
        gated.use_for_tecs && gated.tas_for_tecs > 1.0
    }

    /// Upstream `Plane::ahrs_update`. Runs INS→DCM and publishes attitude sensors.
    pub fn ahrs_update(&mut self) {
        self.ticks.ahrs_update += 1;
        if let Some(gps) = self.sitl_gps.as_mut() {
            let _ = gps.gps_status_publish();
        }
        let gps_declination_fix = self.sitl_gps.as_ref().map(|gps| {
            let fix = gps.current_fix();
            ap_compass::GpsDeclinationFix {
                latitude_deg: fix.latitude_deg,
                longitude_deg: fix.longitude_deg,
                have_fix: fix.have_fix,
            }
        });
        if let Some(compass) = self.sitl_compass.as_mut() {
            let _ = compass_motor_compensation_tick(
                compass,
                CompassMotorCompensationInputs {
                    thr_or_curr: self.compass_battery_current_amps,
                },
            );
            let out = compass_health_scheduler_tick(
                compass,
                &CompassHealthSchedulerInputs {
                    attitude: self.ahrs.dcm.matrix,
                    loop_dt: self.loop_timing.delta_time,
                    gps: gps_declination_fix,
                },
            );
            self.mag_sample = Some(out.sample);
            self.compass_healthy = out.healthy;
            self.compass_health = out.health;
            self.compass = out.yaw_compass;
            if self.compass_learn_requested {
                let cal = compass_offset_calibration_tick(
                    compass,
                    CompassOffsetCalibrationInputs {
                        request_learn: true,
                    },
                );
                self.compass_offsets_learned = cal.learned;
                self.compass_learn_requested = false;
                if cal.learned {
                    self.compass_save_offsets_requested = true;
                }
            }
            if self.compass_save_offsets_requested {
                let persist = compass_offset_persist_tick(
                    compass,
                    CompassOffsetPersistInputs {
                        request_save: true,
                    },
                );
                self.compass_offsets_saved = persist.saved;
                self.compass_save_offsets_requested = false;
            }
        }
        if let Some(gps) = self.sitl_gps.as_mut() {
            let samples = gps.publish_yaw_samples(
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            if self.sitl_compass.is_none() {
                self.compass = samples.compass;
            }
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
            if self.sitl_airspeed.is_none() {
                self.airspeed_tas = samples.airspeed_tas;
            }
            self.eas2tas = samples.eas2tas;
        } else if let Some(source) = self.sitl_yaw {
            let samples = publish_sitl_yaw_samples(
                &source,
                self.ahrs.dcm.matrix,
                self.loop_timing.delta_time,
            );
            if self.sitl_compass.is_none() {
                self.compass = samples.compass;
            }
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
            self.relative_altitude_m = altitude_glue_tick(AltitudeGlueInputs {
                baro_altitude_m: published.sample.altitude_m,
                baro_relative_m: published.relative_altitude_m,
                home_altitude_m: self.home_altitude_m,
                have_baro_sample: published.sample.have_sample,
            });
            self.tecs_baro_feed = tecs_baro_feed_tick(TecsBaroInputs {
                relative_altitude_m: self.relative_altitude_m,
                baro_climb_rate_mps: published.climb_rate_mps,
                have_baro_sample: published.sample.have_sample,
                baro_healthy: published.healthy,
            });
            let arm_cal = baro.arm_calibration_tick(BaroArmCalibrationInputs {
                soft_armed: self.soft_armed,
                was_soft_armed: self.baro_was_soft_armed,
            });
            self.baro_arm_calibration_latched = arm_cal.latched;
            self.baro_was_soft_armed = arm_cal.was_soft_armed;
        }
        if let Some(airspeed) = self.sitl_airspeed.as_mut() {
            if let Some(vel) = self.gps_velocity {
                if vel.have_velocity {
                    airspeed.gps_groundspeed_mps = safe_sqrt(
                        vel.velocity_ned.x * vel.velocity_ned.x + vel.velocity_ned.y * vel.velocity_ned.y,
                    );
                }
            }
            let out = airspeed_health_scheduler_tick(
                airspeed,
                &AirspeedHealthSchedulerInputs { eas2tas: self.eas2tas },
            );
            self.airspeed_sample = Some(out.sample);
            self.airspeed_healthy = out.healthy;
            self.airspeed_health = out.health;
            self.airspeed_ratio = airspeed
                .backend()
                .map(|backend| backend.config().ratio)
                .unwrap_or(ARSPD_RATIO_DEFAULT);
            self.airspeed_use = airspeed
                .backend()
                .map(|backend| backend.config().use_airspeed)
                .unwrap_or(ARSPD_USE_DEFAULT);
            self.airspeed_use_for_control = out.use_airspeed;
            self.airspeed_temperature_c = out.sample.temperature_c;
            self.airspeed_autocal = airspeed
                .backend()
                .map(|backend| backend.config().autocal)
                .unwrap_or(ARSPD_AUTOCAL_DEFAULT);
            self.airspeed_skip_cal = airspeed
                .backend()
                .map(|backend| backend.config().skip_cal)
                .unwrap_or(ARSPD_SKIP_CAL_DEFAULT);
            self.airspeed_tube_order = airspeed.airspeed_params().primary_tube_order();
            self.airspeed_bus = airspeed.airspeed_params().primary_bus();
            self.airspeed_devid = airspeed.airspeed_params().primary_devid();
            self.airspeed_options = airspeed.airspeed_params().options;
            self.airspeed_wind_max = airspeed.airspeed_params().wind_max;
            self.airspeed_wind_max_exceeded = out.wind_max_exceeded;
            self.airspeed_wind_warn = airspeed.airspeed_params().wind_warn;
            self.airspeed_wind_warn_exceeded = out.wind_warn_exceeded;
            self.airspeed_primary = out.health.primary;
            self.airspeed_fbw_min = airspeed.airspeed_params().fbw_min;
            self.airspeed_fbw_max = airspeed.airspeed_params().fbw_max;
            self.airspeed_psi_range = clamp_psi_range(airspeed.airspeed_params().primary_psi_range());
            if out.healthy {
                self.airspeed_tas = out.sample.tas_mps;
            }
            if self.airspeed_calibrate_requested {
                let cal = airspeed_offset_calibration_tick(
                    airspeed,
                    AirspeedOffsetCalibrationInputs {
                        request_calibrate: true,
                    },
                );
                self.airspeed_offset_calibrated = cal.calibrated;
                self.airspeed_calibrate_requested = false;
            }
        }
        if let Some(analog) = self.analog_airspeed.as_mut() {
            let out = analog.publish();
            self.airspeed_pin = out.pin;
            self.airspeed_diff_pressure_pa = out.pressure_pa;
            self.airspeed_analog_have_pressure = out.have_pressure;
            self.airspeed_tube_order = out.tube_order;
            self.airspeed_bus = out.bus;
            self.airspeed_devid = out.devid;
            self.airspeed_options = out.options;
            self.airspeed_wind_max = out.wind_max;
            self.airspeed_wind_warn = out.wind_warn;
            self.airspeed_psi_range = clamp_psi_range(out.psi_range);
        }
        let sensor_type = self
            .sitl_airspeed
            .as_ref()
            .map(|a| a.airspeed_params().primary_sensor_type())
            .or_else(|| {
                self.analog_airspeed
                    .as_ref()
                    .map(|a| a.airspeed_params().primary_sensor_type())
            })
            .unwrap_or(self.airspeed_type);
        let typed = select_airspeed_backend(sensor_type);
        self.airspeed_type = typed.sensor_type;
        self.configured_airspeed_backend = typed.configured;
        self.active_airspeed_backend = typed.active;
        if !typed.enabled {
            self.airspeed_use_for_control = false;
        }
        if let Some(vane) = self.wind_vane {
            self.ahrs.apply_wind_vane(vane);
        }
        let yaw = yaw_update_inputs(self.compass, self.gps_yaw, self.yaw_ctx);
        let motion = drift_motion_inputs(
            self.yaw_ctx,
            self.gps_yaw,
            self.gps_velocity,
            tas_for_nav(self.airspeed_tas, self.airspeed_use_for_control),
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

    /// Build pilot-throttle glue inputs shared by update_control and set_servos.
    fn pilot_throttle_glue_inputs(&self) -> PilotThrottleGlueInputs {
        PilotThrottleGlueInputs {
            throttle_pwm: self.rc_failsafe_inputs.throttle_pwm,
            throttle_cfg: self.rc_failsafe_inputs.throttle_cfg,
            pilot_throttle_source: self.pilot_throttle_source,
            trim_throttle: self.trim_throttle,
            throttle_min: self.throttle_min,
            throttle_max: self.throttle_max,
            use_throttle_limits: self.throttle_use_limits,
            use_battery_compensation: self.throttle_use_battery_comp,
            battery_voltage_ratio: self.battery_voltage_ratio,
        }
    }
    /// Build SRV output glue inputs for the set_servos flap/auto-flap path.
    fn srv_output_glue_inputs(&self) -> SrvOutputSchedulerInputs {
        SrvOutputSchedulerInputs {
            mixing: self.srv_output.mixing,
            flap_params: self.srv_output.flap_params,
            manual_flap_percent: self.srv_output_inputs.manual_flap_percent,
            flap_speed_source_ms: self.airspeed_tas,
            has_auto_flap_schedule: self.srv_output.has_auto_flap_schedule,
            flight_stage_is_takeoff: self.srv_output.flight_stage_is_takeoff,
            flight_stage_is_land: self.flight_stage_is_land,
            apply_elevon_mixing: self.srv_output.apply_elevon_mixing,
            apply_vtail_mixing: self.srv_output.apply_vtail_mixing,
            apply_dspoiler_mixing: self.srv_output.apply_dspoiler_mixing,
            dspoiler: self.srv_output.dspoiler,
            dt: self.loop_timing.delta_time,
            elevator_scaled: self.stabilize_servos.elevator_scaled,
        }
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

        let auto_mission = auto_mode_mission_tick(&AutoModeMissionInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            mission_running: self.mission.running,
            home_is_set: self.home_is_set,
            waypoint_count: self.mission_inputs.waypoint_count,
            current_index: self.mission.current_index,
        });
        self.auto_mode_mission_applied = auto_mission.applied;
        self.auto_mode_mission_started = auto_mission.started;
        if auto_mission.applied {
            self.mission.running = auto_mission.mission_running;
            if auto_mission.started {
                self.mission.current_index = auto_mission.current_index;
                self.mission.complete = false;
            }
        }

        let mission_out = mission_scheduler_tick(
            &mut self.mission,
            &self.landing,
            &MissionSchedulerInputs {
                control_mode: self.mode.control_mode,
                ..self.mission_inputs
            },
        );
        self.last_target_altitude = mission_out.target;
        self.mission_alt_offset_cm = mission_alt_offset_glue_tick(MissionAltOffsetGlueInputs {
            offset_cm: self.mission_inputs.offset_cm,
            target: mission_out.target,
        });
        self.rangefinder_correction_m = rangefinder_correction_glue_tick(
            rangefinder_correction_glue_inputs(
                self.flight_stage_is_land,
                self.rangefinder_bump.rf,
            ),
        );
        self.mission_advanced = mission_out.advanced;
        self.auto_mode_mission_advanced = auto_mission.applied
            && auto_mission.allow_advance
            && mission_out.advanced;
        if mission_out.ran {
            self.next_wp_alt_m = mission_out.next_wp.alt as f32 * 0.01;
        }

        let auto_complete = auto_mode_complete_tick(&AutoModeCompleteInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mission_running: self.mission.running,
            mission_complete: self.mission.complete,
            current_nav_is_land: self.auto_current_nav_is_land,
        });
        self.auto_mode_complete_applied = auto_complete.applied;
        self.auto_mode_switch_to_rtl = auto_complete.switch_to_rtl;
        self.auto_mode_land_handoff = auto_complete.allow_land;

        let rtl_nav = rtl_mode_nav_tick(&RtlModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            home_is_set: self.home_is_set,
            rtl_radius_m: self.rtl_radius_m,
        });
        self.rtl_mode_nav_applied = rtl_nav.applied;
        self.rtl_mode_started = rtl_nav.started;
        self.rtl_mode_loiter_allowed = rtl_nav.allow_loiter;
        if rtl_nav.applied {
            self.rtl_loiter_radius_m = rtl_nav.loiter_radius_m;
            if rtl_nav.direction_set {
                self.rtl_loiter_ccw = rtl_nav.loiter_ccw;
            }
        }
        if rtl_nav.started {
            self.rtl_done_climb = false;
        }

        let rtl_climb = rtl_mode_climb_tick(&RtlModeClimbInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            done_climb: self.rtl_done_climb,
            climb_before_turn: self.rtl_climb_before_turn,
            rtl_climb_min_m: self.rtl_climb_min_m,
            current_alt_cm: self.rtl_current_alt_cm,
            next_wp_alt_cm: self.rtl_next_wp_alt_cm,
            prev_wp_alt_cm: self.rtl_prev_wp_alt_cm,
        });
        self.rtl_mode_climb_applied = rtl_climb.applied;
        self.rtl_climb_gated = rtl_climb.climb_gated;
        self.rtl_climb_constrain_roll = rtl_climb.constrain_roll;
        self.rtl_setup_remaining_leg = rtl_climb.setup_remaining_leg;
        if rtl_climb.applied {
            self.rtl_done_climb = rtl_climb.done_climb;
        }

        let loiter_nav = loiter_mode_nav_tick(&LoiterModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            wp_loiter_rad_m: self.wp_loiter_rad_m,
            stick_mixing_enabled: applies_fbw_stick_mixing(self.stick_mixing),
            loiter_alt_control: self.loiter_alt_control_enabled,
        });
        self.loiter_mode_nav_applied = loiter_nav.applied;
        self.loiter_mode_started = loiter_nav.started;
        self.loiter_mode_loiter_allowed = loiter_nav.allow_loiter;
        self.loiter_alt_control = loiter_nav.alt_control;
        if loiter_nav.applied {
            self.loiter_radius_m = loiter_nav.loiter_radius_m;
            if loiter_nav.direction_set {
                self.loiter_ccw = loiter_nav.loiter_ccw;
            }
        }

        let guided_nav = guided_mode_nav_tick(&GuidedModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            active_radius_m: self.guided_active_radius_m,
            wp_loiter_rad_m: self.wp_loiter_rad_m,
            guided_ccw: self.guided_loiter_ccw,
        });
        self.guided_mode_nav_applied = guided_nav.applied;
        self.guided_mode_started = guided_nav.started;
        self.guided_mode_loiter_allowed = guided_nav.allow_loiter;
        if guided_nav.applied {
            self.guided_loiter_radius_m = guided_nav.loiter_radius_m;
            self.guided_active_radius_m = guided_nav.loiter_radius_m;
            if guided_nav.direction_set {
                self.guided_loiter_ccw = guided_nav.loiter_ccw;
            }
            if guided_nav.clear_throttle_passthru {
                self.guided_throttle_passthru = false;
            }
        }

        let avoid_adsb_nav = avoid_adsb_mode_nav_tick(&AvoidAdsbModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            wp_loiter_rad_m: self.wp_loiter_rad_m,
        });
        self.avoid_adsb_mode_nav_applied = avoid_adsb_nav.applied;
        self.avoid_adsb_mode_started = avoid_adsb_nav.started;
        self.avoid_adsb_mode_loiter_allowed = avoid_adsb_nav.allow_loiter;
        if avoid_adsb_nav.applied {
            self.avoid_adsb_loiter_radius_m = avoid_adsb_nav.loiter_radius_m;
            if avoid_adsb_nav.direction_set {
                self.avoid_adsb_loiter_ccw = avoid_adsb_nav.loiter_ccw;
            }
            if avoid_adsb_nav.clear_throttle_passthru {
                self.guided_throttle_passthru = false;
                self.guided_active_radius_m = 0;
            }
        }

        let takeoff_nav = takeoff_mode_nav_tick(&TakeoffModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            home_is_set: self.home_is_set,
            current_loc_initialised: self.current_loc_initialised,
            target_alt_m: self.takeoff_target_alt_m,
            target_dist_m: self.takeoff_target_dist_m,
            wp_loiter_rad_m: self.wp_loiter_rad_m,
        });
        self.takeoff_mode_nav_applied = takeoff_nav.applied;
        self.takeoff_mode_started = takeoff_nav.started;
        self.takeoff_mode_setup_allowed = takeoff_nav.allow_setup;
        self.takeoff_mode_loiter_allowed = takeoff_nav.allow_loiter;
        if takeoff_nav.applied {
            self.takeoff_loiter_radius_m = takeoff_nav.loiter_radius_m;
            self.takeoff_target_alt_m = takeoff_nav.target_alt_m;
            self.takeoff_target_dist_m = takeoff_nav.target_dist_m;
            if takeoff_nav.direction_set {
                self.takeoff_loiter_ccw = takeoff_nav.loiter_ccw;
            }
        }

        let autoland_nav = autoland_mode_nav_tick(&AutolandModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            mode_just_entered: self.mode_entry_reset,
            is_flying: self.is_flying,
            takeoff_direction_initialized: self.takeoff_direction_initialized,
            quadplane_available: self.quadplane_available,
            landing_is_deepstall: self.autoland_landing_is_deepstall,
            terrain_alt_min_m: self.autoland_terrain_alt_min_m,
            need_climb: self.autoland_need_climb,
            current_stage: self.autoland_stage,
            climb_complete: self.autoland_climb_complete,
            loiter_to_alt_complete: self.autoland_loiter_to_alt_complete,
            wp_alt_m: self.autoland_wp_alt_m,
            wp_dist_m: self.autoland_wp_dist_m,
        });
        self.autoland_mode_nav_applied = autoland_nav.applied;
        self.autoland_mode_started = autoland_nav.started;
        self.autoland_mode_refused = autoland_nav.refused;
        self.autoland_mode_loiter_allowed = autoland_nav.allow_loiter;
        self.autoland_mode_land_allowed = autoland_nav.allow_land;
        self.autoland_apply_level_roll = autoland_nav.apply_level_roll;
        self.autoland_next_wp_crosstrack = autoland_nav.next_wp_crosstrack;
        if autoland_nav.applied {
            self.autoland_stage = autoland_nav.stage;
            if !autoland_nav.refused {
                self.autoland_wp_alt_m = autoland_nav.wp_alt_m;
                self.autoland_wp_dist_m = autoland_nav.wp_dist_m;
            }
        }

        let have_baro = self
            .baro_sample
            .map(|s| s.have_sample)
            .unwrap_or(self.baro_healthy);
        let use_airspeed = self.tecs_use_airspeed();
        self.last_tecs_use_airspeed = use_airspeed;
        let tecs_out = altitude_tecs_feed_tick(
            &mut self.tecs,
            &AltitudeTecsFeedInputs {
                baro_feed: self.tecs_baro_feed,
                have_baro_sample: have_baro,
                relative_altitude_m: self.relative_altitude_m,
                home_altitude_m: self.home_altitude_m,
                next_wp_alt_m: self.next_wp_alt_m,
                mission_alt_offset_cm: self.mission_alt_offset_cm,
                rangefinder_correction_m: self.rangefinder_correction_m,
                terrain_offset_m: self.rangefinder_correction_m,
                target: self.last_target_altitude,
                throttle_suppressed: self.mode_entry.throttle_suppressed,
                throttle_nudge: self.throttle_nudge,
                target_airspeed_cm: self.target_airspeed_cm,
                flight_stage: self.tecs_flight_stage,
                pitch_rad: self.pitch_rad,
                cos_roll: ap_math::scalar::Real::cos(self.attitude.roll_rad()),
                use_airspeed,
                pitch_trim_deg: 0.0,
                now_ms: ap_hal::time::Millis(self.yaw_ctx.now_ms),
                dt: self.loop_timing.delta_time,
            },
        );
        self.last_altitude_tecs_ran = tecs_out.ran;
        if tecs_out.ran {
            self.tecs_throttle_demand = tecs_out.tecs_throttle_demand;
            self.nav_tecs.tecs_pitch_demand_rad = tecs_out.tecs_pitch_demand_rad;
            self.navigation_scheduler_inputs.commanded_pitch_cd =
                ap_math::scalar::rad_to_cd(tecs_out.tecs_pitch_demand_rad) as i32;
        }


        let nav_out = if self.navigation_scheduler_inputs.commanded_roll_cd != 0
            || self.navigation_scheduler_inputs.commanded_pitch_cd != 0
        {
            navigation_scheduler_tick(&NavigationSchedulerInputs {
                commanded_roll_cd: self.navigation_scheduler_inputs.commanded_roll_cd,
                commanded_pitch_cd: self.navigation_scheduler_inputs.commanded_pitch_cd,
                roll_limit_cd: self.stabilize_demands.roll_limit_cd,
                pitch_limit_min_cd: self.stabilize_demands.pitch_limit_min_cd,
                pitch_limit_max_cd: self.stabilize_demands.pitch_limit_max_cd,
            })
        } else {
            NavigationSchedulerOutput {
                nav_roll_cd: self.nav_tecs.nav_roll_cd,
                tecs_pitch_demand_rad: self.nav_tecs.tecs_pitch_demand_rad,
            }
        };
        self.nav_tecs = nav_tecs_scheduler_publish_tick(nav_out);

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
            pilot_throttle: self.pilot_throttle_glue_inputs(),
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

        let manual_nav = manual_mode_nav_tick(&ManualModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_sensor_cd: self.attitude.roll_sensor_cd,
            pitch_sensor_cd: self.attitude.pitch_sensor_cd,
        });
        self.manual_mode_nav_applied = manual_nav.applied;
        if manual_nav.applied {
            self.nav_tecs.nav_roll_cd = manual_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = manual_nav.nav_pitch_cd;
        }

        let fbwa_nav = fbwa_mode_nav_tick(&FbwaModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_norm: self.rc_sticks.roll_norm_dz,
            pitch_norm: self.rc_sticks.pitch_norm_dz,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
            pitch_limit_min_cd: self.stabilize_demands.pitch_limit_min_cd,
            pitch_limit_max_cd: self.stabilize_demands.pitch_limit_max_cd,
            roll_sensor_cd: self.attitude.roll_sensor_cd,
        });
        self.fbwa_mode_nav_applied = fbwa_nav.applied;
        if fbwa_nav.applied {
            self.nav_tecs.nav_roll_cd = fbwa_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = fbwa_nav.nav_pitch_cd;
        }

        let stabilize_nav = stabilize_mode_nav_tick(&StabilizeModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
        });
        self.stabilize_mode_nav_applied = stabilize_nav.applied;
        if stabilize_nav.applied {
            self.nav_tecs.nav_roll_cd = stabilize_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = stabilize_nav.nav_pitch_cd;
        }

        let acro_nav = acro_mode_nav_tick(&AcroModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            locked_roll: self.acro_locked_roll,
            locked_pitch: self.acro_locked_pitch,
            locked_roll_err: self.acro_locked_roll_err,
            locked_pitch_cd: self.acro_locked_pitch_cd,
            roll_sensor_cd: self.attitude.roll_sensor_cd,
            pitch_sensor_cd: self.attitude.pitch_sensor_cd,
        });
        self.acro_mode_nav_applied = acro_nav.applied;
        if acro_nav.applied {
            self.nav_tecs.nav_roll_cd = acro_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = acro_nav.nav_pitch_cd;
        }

        let training_nav = training_mode_nav_tick(&TrainingModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_sensor_cd: self.attitude.roll_sensor_cd,
            pitch_sensor_cd: self.attitude.pitch_sensor_cd,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
            pitch_limit_min_cd: self.stabilize_demands.pitch_limit_min_cd,
            pitch_limit_max_cd: self.stabilize_demands.pitch_limit_max_cd,
        });
        self.training_mode_nav_applied = training_nav.applied;
        self.training_manual_roll = training_nav.training_manual_roll;
        self.training_manual_pitch = training_nav.training_manual_pitch;
        if training_nav.applied {
            self.nav_tecs.nav_roll_cd = training_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = training_nav.nav_pitch_cd;
        }

        let fbwb_nav = fbwb_mode_nav_tick(&FbwbModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_norm: self.rc_sticks.roll_norm_dz,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
        });
        self.fbwb_mode_nav_applied = fbwb_nav.applied;
        if fbwb_nav.applied {
            self.nav_tecs.nav_roll_cd = fbwb_nav.nav_roll_cd;
        }

        let cruise_nav = cruise_mode_nav_tick(&CruiseModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_norm: self.rc_sticks.roll_norm_dz,
            rudder_norm: self.rc_sticks.yaw_norm_dz,
            locked_heading: self.cruise_locked_heading,
            nav_scripting_active: self.nav_scripting_active,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
            commanded_roll_cd: self.nav_tecs.nav_roll_cd,
        });
        self.cruise_mode_nav_applied = cruise_nav.applied;
        if cruise_nav.applied {
            self.nav_tecs.nav_roll_cd = cruise_nav.nav_roll_cd;
            self.cruise_locked_heading = cruise_nav.locked_heading;
        }

        let autotune_nav = autotune_mode_nav_tick(&AutotuneModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_norm: self.rc_sticks.roll_norm_dz,
            pitch_norm: self.rc_sticks.pitch_norm_dz,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
            pitch_limit_min_cd: self.stabilize_demands.pitch_limit_min_cd,
            pitch_limit_max_cd: self.stabilize_demands.pitch_limit_max_cd,
            roll_sensor_cd: self.attitude.roll_sensor_cd,
        });
        self.autotune_mode_nav_applied = autotune_nav.applied;
        if autotune_nav.applied {
            self.nav_tecs.nav_roll_cd = autotune_nav.nav_roll_cd;
            self.navigation_scheduler_inputs.commanded_pitch_cd = autotune_nav.nav_pitch_cd;
        }

        let circle_nav = circle_mode_nav_tick(&CircleModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
        });
        self.circle_mode_nav_applied = circle_nav.applied;
        if circle_nav.applied {
            self.nav_tecs.nav_roll_cd = circle_nav.nav_roll_cd;
        }

        let thermal_nav = thermal_mode_nav_tick(&ThermalModeNavInputs {
            control_mode: self.mode.control_mode,
            features: self.features,
            thermal_bank_deg: self.thermal_bank_deg,
            roll_limit_cd: self.stabilize_demands.roll_limit_cd,
        });
        self.thermal_mode_nav_applied = thermal_nav.applied;
        if thermal_nav.applied {
            self.nav_tecs.nav_roll_cd = thermal_nav.nav_roll_cd;
        }

        let throttle = if glue_out.throttle_zeroed_by_mode_entry {
            0.0
        } else {
            calc_throttle_glue_tick(&CalcThrottleGlueInputs {
                control_mode: self.mode.control_mode,
                features: self.features,
                tecs_throttle_demand: self.tecs_throttle_demand,
                throttle_nudge: self.throttle_nudge,
                pilot_throttle: self.pilot_throttle_glue_inputs(),
            })
        };
        self.stabilize_demands.throttle_scaled = throttle;
        self.servos.throttle_scaled = throttle;

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
        let with_baro = plane_pre_arm_checks_baro(with_gps, self.baro_pre_arm_ok);
        let require_compass = self.sitl_compass.is_some();
        self.compass_pre_arm_ok =
            compass_pre_arm_check(self.compass_health, require_compass);
        let with_compass =
            plane_pre_arm_checks_compass(with_baro, self.compass_pre_arm_ok);
        let require_airspeed = self.sitl_airspeed.is_some();
        self.airspeed_pre_arm_ok =
            airspeed_pre_arm_check(self.airspeed_health, require_airspeed);
        self.pre_arm_ok = matches!(
            plane_pre_arm_checks_airspeed(with_compass, self.airspeed_pre_arm_ok),
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
        let glue_stab_out = mode_glue_stabilize_tick(
            self.stabilize_servos.rudder_scaled,
            &ModeGlueStabilizeInputs {
                stick_mixing: self.effective_stick_mixing,
                yaw_norm_dz: self.rc_sticks.yaw_norm_dz,
                rudder_limit_scaled: 4500.0,
            },
        );
        self.stabilize_servos.rudder_scaled = glue_stab_out.rudder_scaled;
    }

    /// Upstream `Plane::set_servos`. Publishes scaled/PWM demands from stabilize,
    /// then applies landing servo overrides when in LAND.
    pub fn set_servos(&mut self) {
        self.ticks.set_servos += 1;
        apply_stabilize_to_servos(&self.stabilize_servos, &mut self.servos);

        let manual_servos = manual_mode_servos_tick(
            self.servos,
            &ManualModeServosInputs {
                control_mode: self.mode.control_mode,
                features: self.features,
                rc_sticks: self.rc_sticks,
            },
        );
        self.manual_mode_servos_applied = manual_servos.applied;
        self.servos = manual_servos.servos;

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

        let srv_inp = self.srv_output_glue_inputs();
        let srv_out = srv_output_scheduler_tick(
            self.servos,
            &mut self.srv_output,
            &srv_inp,
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

        let glue_servos_out = mode_glue_set_servos_tick(
            self.servos,
            &ModeGlueSetServosInputs {
                control_mode: self.mode.control_mode,
                features: self.features,
                transition_cleared: trans_out.cleared,
                throttle_suppressed: self.mode_entry.throttle_suppressed,
                current_throttle: self.servos.throttle_scaled,
                pilot_throttle: self.pilot_throttle_glue_inputs(),
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

        let calc_out = set_servos_calc_throttle_tick(
            self.servos,
            &SetServosGlueInputs {
                control_mode: self.mode.control_mode,
                features: self.features,
                tecs_throttle_demand: self.tecs_throttle_demand,
                throttle_nudge: self.throttle_nudge,
                landing_throttle_applied: self.landing_throttle_applied,
                disarm_throttle_applied: self.disarm_throttle_applied,
                mode_entry_applied: self.mode_entry_throttle_applied,
                mode_glue_throttle_restored: self.mode_glue_throttle_restored,
                pilot_throttle: self.pilot_throttle_glue_inputs(),
            },
        );
        self.last_set_servos_calc_throttle = calc_out.applied;
        if calc_out.applied {
            self.stabilize_demands.throttle_scaled = calc_out.stabilize_throttle;
        }
        self.servos = calc_out.servos;

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
