//! Main vehicle loop scheduler wiring and mode dispatch.

use ap_hal::time::{Clock, Micros, Millis};
use ap_plane::main_loop::{plane_fast_tasks, run_scheduler_tick, PlaneMainLoop, StabilizeDispatch};
use ap_plane::mode_table_hookup::dispatch_stabilize_from_mode;
use ap_plane::mode_run::StickMixing;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_scheduler::scheduler::Scheduler;
use core::cell::Cell;

struct StepClock {
    us: Cell<u32>,
}

impl StepClock {
    fn new() -> Self {
        Self {
            us: Cell::new(0),
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
        self.us.get() as u64 / 1000
    }
    fn micros64(&self) -> u64 {
        self.us.get() as u64
    }
}

#[test]
fn fast_tasks_run_in_scheduler_order() {
    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();

    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert_eq!(vehicle.ticks.ahrs_update, 1);
    assert_eq!(vehicle.ticks.update_control_mode, 1);
    assert_eq!(vehicle.ticks.stabilize, 1);
    assert_eq!(vehicle.ticks.set_servos, 1);
}

#[test]
fn stabilize_mode_enables_attitude_paths_and_stick_mixing() {
    let dispatch = dispatch_stabilize_from_mode(
        ModeNumber::Stabilize.as_number(),
        Some(StickMixing::Fbw),
        &BuildFeatures::default(),
    );
    assert_eq!(
        dispatch,
        StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing: true,
        }
    );
}

#[test]
fn manual_mode_skips_stabilization() {
    let dispatch = dispatch_stabilize_from_mode(
        ModeNumber::Manual.as_number(),
        Some(StickMixing::Fbw),
        &BuildFeatures::default(),
    );
    assert_eq!(dispatch, StabilizeDispatch::default());
}

#[test]
fn update_control_mode_records_mode_dispatch() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::FlyByWireA.as_number();
    vehicle.stick_mixing = Some(StickMixing::None);

    vehicle.update_control_mode();

    assert_eq!(vehicle.ticks.update_control_mode, 1);
    assert_eq!(
        vehicle.last_stabilize,
        StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing: false,
        }
    );
}

#[test]
fn stabilize_records_active_attitude_paths() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.last_stabilize = StabilizeDispatch {
        roll: true,
        pitch: false,
        yaw: true,
        fbw_stick_mixing: false,
    };

    vehicle.stabilize();

    assert_eq!(
        vehicle.last_stabilize_run,
        ap_plane::main_loop::StabilizeRun {
            roll: true,
            pitch: false,
            yaw: true,
        }
    );
    assert_eq!(vehicle.stabilize_servos.elevator_scaled, 0.0);
}

#[test]
fn set_servos_applies_deepstall_landing_override() {
    use ap_landing::deepstall_override::DeepstallOverrideInputs;
    use ap_landing::deepstall_stage::DeepstallStage;
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_plane::stabilize_hookup::StabilizeServoDemands;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        ..LandingFlags::default()
    };
    vehicle.landing.landing_type = LandingType::Deepstall;
    vehicle.landing.machine.deepstall.stage = DeepstallStage::Land;
    vehicle.deepstall_override = DeepstallOverrideInputs {
        stage: DeepstallStage::Land,
        stall_entry_ms: 0,
        now_ms: 5000,
        slew_speed: 1.0,
        initial_elevator_pwm: 1500,
        target_elevator_pwm: 1900,
        airspeed_ms: Some(10.0),
        handoff_airspeed_ms: 12.0,
        handoff_lower_limit_ms: 8.0,
        steering_pid: 0.5,
        aileron_scalar: 1.0,
        elevator_present: true,
    };
    vehicle.stabilize_servos = StabilizeServoDemands {
        aileron_scaled: 0.0,
        elevator_scaled: 0.0,
        rudder_scaled: 0.0,
    };

    vehicle.set_servos();

    assert!(vehicle.landing_servo_override_applied);
    assert_eq!(vehicle.servos.elevator_pwm, 1900);
    assert!((vehicle.servos.aileron_scaled - 2250.0).abs() < 1.0);
    assert_eq!(vehicle.servos.throttle_scaled, 0.0);
}

#[test]
fn set_servos_skips_landing_override_outside_land_stage() {
    use ap_landing::deepstall_stage::DeepstallStage;
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_plane::stabilize_hookup::{scaled_to_pwm_trim, StabilizeServoDemands};

    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = false;
    vehicle.landing.flags.in_progress = true;
    vehicle.landing.landing_type = LandingType::Deepstall;
    vehicle.landing.machine.deepstall.stage = DeepstallStage::Land;
    vehicle.stabilize_servos = StabilizeServoDemands {
        aileron_scaled: 500.0,
        elevator_scaled: -250.0,
        rudder_scaled: 0.0,
    };

    vehicle.set_servos();

    assert!(!vehicle.landing_servo_override_applied);
    assert_eq!(vehicle.servos.aileron_scaled, 500.0);
    assert_eq!(vehicle.servos.elevator_pwm, scaled_to_pwm_trim(-250.0));
}

#[test]
fn set_servos_latches_go_around_from_missing_elevator() {
    use ap_landing::deepstall_override::DeepstallOverrideInputs;
    use ap_landing::deepstall_stage::DeepstallStage;
    use ap_landing::go_around::{LandingFlags, LandingType};

    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    vehicle.landing.landing_type = LandingType::Deepstall;
    vehicle.landing.machine.deepstall.stage = DeepstallStage::Land;
    vehicle.deepstall_override = DeepstallOverrideInputs {
        stage: DeepstallStage::Land,
        stall_entry_ms: 0,
        now_ms: 5000,
        slew_speed: 1.0,
        initial_elevator_pwm: 1500,
        target_elevator_pwm: 1900,
        airspeed_ms: Some(10.0),
        handoff_airspeed_ms: 12.0,
        handoff_lower_limit_ms: 8.0,
        steering_pid: 0.5,
        aileron_scalar: 1.0,
        elevator_present: false,
    };

    vehicle.set_servos();

    assert!(vehicle.landing_request_go_around);
    assert!(vehicle.landing.flags.commanded_go_around);
}

#[test]
fn scheduler_tick_advances_landing_in_land_stage() {
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_landing::slope_stage::SlopeStage;
    use ap_plane::landing_loop::VerifyLandVehicleInputs;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        ..LandingFlags::default()
    };
    vehicle.landing.landing_type = LandingType::StandardGlideSlope;
    vehicle.verify_land_inputs = VerifyLandVehicleInputs {
        height_above_target_m: 20.0,
        terrain_correction_m: 0.0,
        sink_rate_ms: 2.0,
        wp_proportion: 0.6,
        is_flying: true,
        rangefinder_in_range: true,
        bearing_error_cd: 500,
        crosstrack_error_m: 1.0,
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
            distance_to_landing_m: 50.0,
            distance_to_arc_entry_m: 150.0,
            loiter_radius_m: 100.0,
            loiter_ccw: false,
            reached_loiter: true,
            height_error_m: 1.0,
            target_bearing_cd: 500,
            heading_error_deg: 5.0,
            target_heading_deg: 0.0,
            groundspeed_ne: ap_math::vector2::Vector2f::new(10.0, 0.0),
            current: ap_math::location::Location::new(-35_000_000, 149_000_000),
            arc_exit: ap_math::location::Location::new(-35_000_000, 149_000_000),
            arc_entry: ap_math::location::Location::new(-35_000_000, 149_000_000),
            extended_approach: ap_math::location::Location::new(-35_000_000, 149_000_000),
            entry_point: ap_math::location::Location::new(-35_000_000, 149_000_000),
        },
    };
    vehicle.nav_tecs.nav_roll_cd = 6000;
    vehicle.level_roll_limit_cd = 4500;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();

    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert_eq!(vehicle.landing.machine.slope_stage, SlopeStage::Approach);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 6000);
    assert!(!vehicle.landing_throttle_suppressed);
}

#[test]
fn scheduler_tick_recalculates_slope_on_rangefinder_bump() {
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_landing::rangefinder_bump::{RangefinderBumpConfig, RangefinderBumpInputs};
    use ap_landing::slope_stage::RangefinderState;
    use ap_landing::{SlopeConfig, SlopeInputs};
    use ap_math::location::{AltContext, AltFrame, Location};
    use ap_plane::rangefinder_bump_hookup::RangefinderBumpHookupInputs;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        ..LandingFlags::default()
    };
    vehicle.landing.landing_type = LandingType::StandardGlideSlope;
    vehicle.rangefinder_bump.flags.in_progress = true;
    vehicle.rangefinder_bump.slope = 0.05;

    let prev = Location::new_with_alt(-35_000_000, 149_000_000, 10_000, AltFrame::Absolute);
    let mut next = prev;
    next.offset(1000.0, 0.0);
    next.set_alt_cm(0, AltFrame::Absolute);
    let alt_ctx = AltContext {
        home_alt_cm: Some(0),
        origin_alt_cm: Some(0),
        terrain_alt_cm: Some(0),
    };

    vehicle.rangefinder_bump_inputs = RangefinderBumpHookupInputs {
        flight_stage_is_land: true,
        landing_type: LandingType::StandardGlideSlope,
        bump_cfg: RangefinderBumpConfig {
            shallow_threshold: 1.0,
            steep_threshold_deg: 1.0,
        },
        slope_cfg: SlopeConfig {
            flare_sec: 2.0,
            flare_alt: 3.0,
            flare_effectivness_pct: 50,
        },
        slope_inp: SlopeInputs {
            prev_wp: prev,
            next_wp: next,
            current: prev,
            groundspeed: 20.0,
            land_sinkrate: 1.0,
            alt_ctx,
        },
        bump: RangefinderBumpInputs {
            rf: RangefinderState {
                in_use: true,
                correction: 6.0,
                last_stable_correction: 0.0,
            },
            prev_wp: prev,
            next_wp: next,
            current: prev,
            wp_distance_m: 300.0,
            adjusted_altitude_cm: 10_000,
            alt_ctx,
        },
    };

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(vehicle.last_rangefinder_bump_recalculated);
    assert_eq!(vehicle.rangefinder_bump.rf.last_stable_correction, 6.0);
}

#[test]
fn scheduler_tick_applies_deepstall_servo_override() {
    use ap_landing::deepstall_override::DeepstallOverrideInputs;
    use ap_landing::deepstall_stage::DeepstallStage;
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_plane::landing_hookup::ServoOutputState;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        ..LandingFlags::default()
    };
    vehicle.landing.landing_type = LandingType::Deepstall;
    vehicle.landing.machine.deepstall.stage = DeepstallStage::Land;
    vehicle.deepstall_override = DeepstallOverrideInputs {
        stage: DeepstallStage::Land,
        stall_entry_ms: 0,
        now_ms: 5000,
        slew_speed: 1.0,
        initial_elevator_pwm: 1500,
        target_elevator_pwm: 1900,
        airspeed_ms: Some(10.0),
        handoff_airspeed_ms: 12.0,
        handoff_lower_limit_ms: 8.0,
        steering_pid: 0.5,
        aileron_scalar: 1.0,
        elevator_present: true,
    };
    vehicle.servos = ServoOutputState {
        elevator_pwm: 1500,
        throttle_scaled: 50.0,
        ..ServoOutputState::default()
    };

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(vehicle.landing_servo_override_applied);
    assert_eq!(vehicle.servos.elevator_pwm, 1900);
    assert_eq!(vehicle.servos.throttle_scaled, 0.0);
}

#[test]
fn scheduler_tick_reads_rc_channels() {
    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };
    vehicle.rc_failsafe_inputs = RcFailsafeSchedulerInputs {
        has_valid_input: true,
        roll_pwm: Some(1700),
        pitch_pwm: Some(1300),
        throttle_pwm: Some(1100),
        roll_cfg: RcChannelConfig::default(),
        pitch_cfg: RcChannelConfig::default(),
        flap_pwm: None,
        flap_cfg: RcChannelConfig::default(),
        failsafe_cfg: RcFailsafeConfig {
            throttle_failsafe_enabled: true,
            throttle_failsafe_pwm: 975,
        },
    };

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(!vehicle.in_rc_failsafe);
    assert!(vehicle.rc_sticks.roll_norm_dz > 0.4);
    assert!(vehicle.rc_sticks.pitch_norm_dz < -0.4);
}

#[test]
fn scheduler_tick_advances_mission_in_auto() {
    use ap_math::location::{AltFrame, Location};
    use ap_plane::mode_table::ModeNumber;
    use ap_plane::target_altitude::TargetAltitude;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    let target = Location::new_with_alt(-35_000_000, 149_000_000, 10_000, AltFrame::Absolute);
    let mut near = target;
    near.offset(50.0, 0.0);
    vehicle.mission_inputs = ap_plane::mission_scheduler_hookup::MissionSchedulerInputs {
        control_mode: ModeNumber::Auto.as_number(),
        current_loc: near,
        waypoint_count: 2,
        waypoints: [
            target,
            near,
            Location::new(0, 0),
            Location::new(0, 0),
            Location::new(0, 0),
            Location::new(0, 0),
            Location::new(0, 0),
            Location::new(0, 0),
        ],
        wp_radius_m: 100.0,
        ..Default::default()
    };

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(vehicle.mission_advanced);
    assert_eq!(vehicle.last_target_altitude, TargetAltitude::FromNextWaypoint);
}

#[test]
fn set_servos_zeros_throttle_when_landing_suppressed() {
    use ap_plane::landing_hookup::ServoOutputState;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing_throttle_suppressed = true;
    vehicle.servos = ServoOutputState {
        throttle_scaled: 60.0,
        ..ServoOutputState::default()
    };

    vehicle.set_servos();

    assert!(vehicle.landing_throttle_applied);
    assert_eq!(vehicle.servos.throttle_scaled, 0.0);
}

#[test]
fn set_servos_applies_elevon_mixing_via_srv_output() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::srv_output_hookup::MixingParams;
    use ap_plane::srv_output_scheduler_hookup::SrvOutputSchedulerInputs;
    use ap_servo::function::Function;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.stabilize_servos.aileron_scaled = 1000.0;
    vehicle.servos = ServoOutputState {
        aileron_scaled: 1000.0,
        ..ServoOutputState::default()
    };
    vehicle.srv_output.apply_elevon_mixing = true;
    vehicle.srv_output_inputs = SrvOutputSchedulerInputs {
        mixing: MixingParams {
            mixing_gain: 1.0,
            mixing_offset: 0,
        },
        apply_elevon_mixing: true,
        elevator_scaled: 500.0,
        dt: 0.02,
        ..SrvOutputSchedulerInputs::default()
    };
    vehicle.srv_output.registry.assign(Function::AILERON, 1 << 0);
    vehicle.srv_output.registry.assign(Function::ELEVATOR, 1 << 1);
    vehicle.srv_output.registry.assign(Function::ELEVON_LEFT, 1 << 2);
    vehicle.srv_output.registry.assign(Function::ELEVON_RIGHT, 1 << 3);

    vehicle.set_servos();

    assert_eq!(
        vehicle.srv_output.registry.output_scaled(Function::ELEVON_LEFT),
        -500.0
    );
    assert_eq!(
        vehicle.srv_output.registry.output_scaled(Function::ELEVON_RIGHT),
        1500.0
    );
}

#[test]
fn set_servos_zeros_throttle_when_disarmed() {
    use ap_plane::landing_hookup::ServoOutputState;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.soft_armed = false;
    vehicle.servos = ServoOutputState {
        throttle_scaled: 55.0,
        ..ServoOutputState::default()
    };

    vehicle.set_servos();

    assert!(vehicle.disarm_throttle_applied);
    assert_eq!(vehicle.servos.throttle_scaled, 0.0);
}

#[test]
fn set_servos_keeps_throttle_when_armed() {
    use ap_plane::landing_hookup::ServoOutputState;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.soft_armed = true;
    vehicle.servos = ServoOutputState {
        throttle_scaled: 55.0,
        ..ServoOutputState::default()
    };

    vehicle.set_servos();

    assert!(!vehicle.disarm_throttle_applied);
    assert_eq!(vehicle.servos.throttle_scaled, 55.0);
}

#[test]
fn update_control_mode_resets_mode_entry_on_change() {
    use ap_plane::mode_table::ModeNumber;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Manual.as_number();
    vehicle.tracked_control_mode = ModeNumber::Auto.as_number();
    vehicle.mode_entry.auto.inverted_flight = true;
    vehicle.mode_entry.new_airspeed_cm = 500;
    vehicle.attitude.pitch_sensor_cd = 800;

    vehicle.update_control_mode();

    assert!(vehicle.mode_entry_reset);
    assert!(!vehicle.mode_entry.auto.inverted_flight);
    assert_eq!(vehicle.mode_entry.auto.initial_pitch_cd, 800);
    assert_eq!(vehicle.mode_entry.new_airspeed_cm, -1);
    assert_eq!(vehicle.tracked_control_mode, ModeNumber::Manual.as_number());
}

#[test]
fn set_servos_zeros_throttle_on_mode_entry_suppression() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::mode_table::ModeNumber;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.soft_armed = true;
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.mode_entry.throttle_suppressed = true;
    vehicle.servos = ServoOutputState {
        throttle_scaled: 80.0,
        ..ServoOutputState::default()
    };

    vehicle.set_servos();

    assert!(vehicle.mode_entry_throttle_applied);
    assert_eq!(vehicle.servos.throttle_scaled, 0.0);
}

#[test]
fn set_servos_publishes_throttle_pwm_from_registry() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::srv_pwm_publish_hookup::channel_pwm;
    use ap_servo::function::Function;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.servos = ServoOutputState {
        throttle_scaled: 1000.0,
        ..ServoOutputState::default()
    };
    vehicle.srv_output.registry.assign(Function::THROTTLE, 1 << 0);

    vehicle.set_servos();

    assert!(vehicle.last_pwm_publish_ran);
    let pwm = channel_pwm(&vehicle.srv_pwm, Function::THROTTLE).expect("throttle pwm");
    assert!(pwm > 1900, "full throttle pwm expected, got {pwm}");
}

#[test]
fn scheduler_tick_publishes_throttle_pwm_after_set_servos() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::srv_pwm_publish_hookup::channel_pwm;
    use ap_servo::function::Function;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.servos = ServoOutputState {
        throttle_scaled: 1000.0,
        ..ServoOutputState::default()
    };
    vehicle.srv_output.registry.assign(Function::THROTTLE, 1 << 0);

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(vehicle.last_pwm_publish_ran);
    let pwm = channel_pwm(&vehicle.srv_pwm, Function::THROTTLE).expect("throttle pwm");
    assert!(pwm > 1900, "full throttle pwm expected, got {pwm}");
}

#[test]
fn scheduler_tick_latches_go_around_from_rangefinder_bump_abort() {
    use ap_landing::go_around::{LandingFlags, LandingType};
    use ap_landing::rangefinder_bump::{RangefinderBumpConfig, RangefinderBumpInputs};
    use ap_landing::slope_stage::RangefinderState;
    use ap_landing::{SlopeConfig, SlopeInputs};
    use ap_math::location::{AltContext, AltFrame, Location};
    use ap_plane::rangefinder_bump_hookup::RangefinderBumpHookupInputs;

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.flight_stage_is_land = true;
    vehicle.landing.flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    vehicle.landing.landing_type = LandingType::StandardGlideSlope;
    vehicle.rangefinder_bump.flags.in_progress = true;
    vehicle.rangefinder_bump.slope = 0.05;

    let prev = Location::new_with_alt(-35_000_000, 149_000_000, 10_000, AltFrame::Absolute);
    let mut next = prev;
    next.offset(1000.0, 0.0);
    next.set_alt_cm(0, AltFrame::Absolute);
    let alt_ctx = AltContext {
        home_alt_cm: Some(0),
        origin_alt_cm: Some(0),
        terrain_alt_cm: Some(0),
    };

    vehicle.rangefinder_bump_inputs = RangefinderBumpHookupInputs {
        flight_stage_is_land: true,
        landing_type: LandingType::StandardGlideSlope,
        bump_cfg: RangefinderBumpConfig {
            shallow_threshold: 1.0,
            steep_threshold_deg: 1.0,
        },
        slope_cfg: SlopeConfig {
            flare_sec: 2.0,
            flare_alt: 3.0,
            flare_effectivness_pct: 50,
        },
        slope_inp: SlopeInputs {
            prev_wp: prev,
            next_wp: next,
            current: prev,
            groundspeed: 20.0,
            land_sinkrate: 1.0,
            alt_ctx,
        },
        bump: RangefinderBumpInputs {
            rf: RangefinderState {
                in_use: true,
                correction: -40.0,
                last_stable_correction: 0.0,
            },
            prev_wp: prev,
            next_wp: next,
            current: prev,
            wp_distance_m: 300.0,
            adjusted_altitude_cm: 10_000,
            alt_ctx,
        },
    };

    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();
    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

    assert!(vehicle.landing_request_go_around);
    assert!(vehicle.last_go_around_latched);
    assert!(vehicle.landing.flags.commanded_go_around);
}

#[test]
fn set_servos_clears_mode_entry_throttle_on_altitude() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::mode_table::ModeNumber;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.soft_armed = true;
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.mode_entry.throttle_suppressed = true;
    vehicle.relative_altitude_m = 15.0;
    vehicle.servos = ServoOutputState {
        throttle_scaled: 80.0,
        ..ServoOutputState::default()
    };

    vehicle.set_servos();

    assert!(vehicle.mode_transition_throttle_cleared);
    assert!(!vehicle.mode_entry.throttle_suppressed);
    assert!(!vehicle.mode_entry_throttle_applied);
    assert_eq!(vehicle.servos.throttle_scaled, 80.0);
}



#[test]
fn update_control_mode_sets_throttle_rules_for_stabilize() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ap_plane::mode_table::ModeNumber::Stabilize.as_number();
    vehicle.update_control_mode();
    assert!(vehicle.throttle_use_limits);
    assert!(!vehicle.throttle_use_battery_comp);
}

#[test]
fn update_control_mode_sets_battery_comp_in_fbwb() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ap_plane::mode_table::ModeNumber::FlyByWireB.as_number();
    vehicle.update_control_mode();
    assert!(vehicle.throttle_use_limits);
    assert!(vehicle.throttle_use_battery_comp);
}
