//! Vehicle-loop scheduler leftover, upstream `ArduCopter/Copter.cpp`.

use ap_copter::radio::ReadRadioLeftover;
use ap_copter::vehicle_loop::{
    always_on_tasks, ap_value, auto_disarm_check, auto_disarm_delay_ms, check_ekf_reset,
    copter_first_fast_tasks, copter_first_scheduled_tasks, copter_logging_tasks,
    copter_next_fast_tasks, copter_next_scheduled_tasks, copter_periodic_loop_tasks,
    copter_rc_loop_task, first_scheduled_task, get_scheduler_tasks, get_wp_distance_m,
    allocate_motors, init_ardupilot, startup_ins_ground,
    loop_rate_logging, lost_vehicle_check, motors_output, motors_output_main, one_hz_loop, rc_loop,
    read_ahrs, read_inertia, read_mode_switch, run_nav_updates, run_scheduler_tick, set_home,
    set_home_to_current_location, set_home_to_current_location_inflight, should_log,
    standby_update, takeoff_check, takeoff_check_load_adequate, ten_hz_logging_loop, three_hz_loop,
    throttle_loop, twentyfive_hz_logging, update_altitude, update_auto_armed, update_batt_compass,
    update_flight_mode, update_home_from_ekf, update_land_and_crash_detectors,
    update_rangefinder_terrain_offset, ApState, AutoDisarmCheckInputs, AutoDisarmCheckPath,
    AllocateMotorsInputs, AllocatedAttitudeKind, AllocatedMotorsKind,
    CheckEkfResetInputs, CopterVehicleLoop, EkfResetMethod, InitArdupilotInputs, InitArdupilotPath,
    InterlockEdge, LoopRateLoggingInputs,
    LostVehicleCheckInputs, ModeSwitchReadInputs, ModeSwitchReadLeftover, MotorsOutputDrive,
    MotorsOutputMainLeftover, MotorsOutputPush, OneHzLoopInputs, RangefinderTerrainState,
    SetHomeInputs, SetHomeToCurrentLocationInflightInputs, SetHomeToCurrentLocationInputs,
    StartupInsGroundInputs,
    TakeoffCheckInputs, TakeoffCheckPath, TaskKind, TenHzLoggingInputs, TwentyfiveHzLoggingInputs,
    UpdateAltitudeInputs, UpdateAutoArmedInputs, UpdateBattCompassInputs, UpdateFlightModeInputs,
    UpdateHomeFromEkfInputs, UpdateHomeFromEkfPath, UpdateRangefinderTerrainOffsetInputs,
    ALLOCATE_MOTORS_BRUSHED_RC_SPEED_HZ, ALLOCATE_MOTORS_TRI_YAW_FILT_D_HZ,
    ALLOCATE_MOTORS_Y6_RATE_RP_KD, ALLOCATE_MOTORS_Y6_RATE_RP_KP,
    ALLOCATE_MOTORS_Y6_RATE_YAW_KI, ALLOCATE_MOTORS_Y6_RATE_YAW_KP,
    AP_PARAM_FRAME_TRICOPTER, ARMING_DELAY_MS, AUTO_DISARMING_DELAY,
    AUTO_DISARM_CHECK_MAX_TIME_MICROS,
    AUTO_DISARM_CHECK_PRIORITY, AUTO_DISARM_CHECK_RATE_HZ, COPTER_LOOP_RATE_HZ,
    DEFAULT_LOG_BITMASK, DISARM_DELAY_MAX_S, FAST_TASK_PRI0, INIT_ARDUPILOT_FAILSAFE_US,
    LOOP_RATE_LOGGING_MAX_TIME_MICROS,
    LOOP_RATE_LOGGING_PRIORITY, LOST_VEHICLE_CHECK_MAX_TIME_MICROS, LOST_VEHICLE_CHECK_PRIORITY,
    LOST_VEHICLE_CHECK_RATE_HZ, LOST_VEHICLE_DELAY, LOST_VEHICLE_STICK_MAX, MASK_LOG_ANY,
    MASK_LOG_ATTITUDE_FAST, MASK_LOG_ATTITUDE_MED, MASK_LOG_IMU, MASK_LOG_IMU_FAST,
    MASK_LOG_MOTBATT, MASK_LOG_NTUN, MASK_LOG_PM, MODE_REASON_INITIALISED, MODE_REASON_UNAVAILABLE,
    MODE_STABILIZE, MODE_THROW, MOTOR_FRAME_6DOF_SCRIPTING, MOTOR_FRAME_COAX,
    MOTOR_FRAME_DECA, MOTOR_FRAME_DODECAHEXA, MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX,
    MOTOR_FRAME_HELI, MOTOR_FRAME_HELI_DUAL, MOTOR_FRAME_HELI_QUAD,
    MOTOR_FRAME_HEXA, MOTOR_FRAME_OCTA, MOTOR_FRAME_OCTAQUAD, MOTOR_FRAME_QUAD,
    MOTOR_FRAME_SCRIPTING_MATRIX, MOTOR_FRAME_SINGLE, MOTOR_FRAME_TAILSITTER,
    MOTOR_FRAME_TRI, MOTOR_FRAME_UNDEFINED, MOTOR_FRAME_Y6,
    ONE_HZ_LOOP_MAX_TIME_MICROS,
    ONE_HZ_LOOP_PRIORITY, ONE_HZ_LOOP_RATE_HZ, RC_LOOP_MAX_TIME_MICROS, RC_LOOP_PRIORITY,
    RC_LOOP_RATE_HZ, REMAINING, RUN_NAV_UPDATES_MAX_TIME_MICROS, RUN_NAV_UPDATES_PRIORITY,
    RUN_NAV_UPDATES_RATE_HZ, SCHEDULER_TASKS, STANDBY_UPDATE_MAX_TIME_MICROS,
    STANDBY_UPDATE_PRIORITY, STANDBY_UPDATE_RATE_HZ, SURFTRAK_TC_DEFAULT,
    TAKEOFF_CHECK_AVG_LOAD_MAX, TAKEOFF_CHECK_MAX_TIME_MICROS, TAKEOFF_CHECK_PEAK_LOAD_MAX,
    TAKEOFF_CHECK_PRIORITY, TAKEOFF_CHECK_RATE_HZ, TAKEOFF_CHECK_WARNING_MS,
    TEN_HZ_LOGGING_MAX_TIME_MICROS, TEN_HZ_LOGGING_PRIORITY, TEN_HZ_LOGGING_RATE_HZ,
    THREE_HZ_LOOP_MAX_TIME_MICROS, THREE_HZ_LOOP_PRIORITY, THREE_HZ_LOOP_RATE_HZ,
    THROTTLE_LOOP_MAX_TIME_MICROS, THROTTLE_LOOP_PRIORITY, THROTTLE_LOOP_RATE_HZ,
    THR_BEHAVE_FEEDBACK_FROM_MID_STICK, TWENTYFIVE_HZ_LOGGING_MAX_TIME_MICROS,
    TWENTYFIVE_HZ_LOGGING_PRIORITY, TWENTYFIVE_HZ_LOGGING_RATE_HZ, UPDATE_ALTITUDE_MAX_TIME_MICROS,
    UPDATE_ALTITUDE_PRIORITY, UPDATE_ALTITUDE_RATE_HZ, UPDATE_BATT_COMPASS_MAX_TIME_MICROS,
    UPDATE_BATT_COMPASS_PRIORITY, UPDATE_BATT_COMPASS_RATE_HZ, VEHICLE_CLASS_COPTER,
};
use ap_hal::time::{Clock, Micros, Millis};
use ap_math::location::AltFrame;
use ap_scheduler::scheduler::{Scheduler, LOOP_RATE};
use core::cell::Cell;

struct StepClock {
    us: Cell<u32>,
}

impl StepClock {
    fn new() -> Self {
        Self { us: Cell::new(0) }
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

#[test]
fn table_starts_with_ins_update_fast_task() {
    let first = SCHEDULER_TASKS[0];
    assert_eq!(first.name, "AP_InertialSensor::update");
    assert_eq!(first.kind, TaskKind::Fast);
    assert_eq!(first.priority, FAST_TASK_PRI0);
    assert!(first.rate_hz == LOOP_RATE);
    assert_eq!(first.max_time_micros, 0);
    assert!(first.gate.is_none());
}

#[test]
fn first_scheduled_row_is_rc_loop() {
    let task = first_scheduled_task().expect("rc_loop is always compiled");
    assert_eq!(task.name, "rc_loop");
    assert_eq!(task.kind, TaskKind::Scheduled);
    assert!(task.rate_hz == RC_LOOP_RATE_HZ);
    assert_eq!(task.max_time_micros, RC_LOOP_MAX_TIME_MICROS);
    assert_eq!(task.priority, RC_LOOP_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn get_scheduler_tasks_hands_pm_log_bit() {
    let view = get_scheduler_tasks();
    assert_eq!(view.log_bit, MASK_LOG_PM);
    assert_eq!(view.log_bit, 8);
    assert_eq!(view.task_count, SCHEDULER_TASKS.len());
    assert!(view.task_count > always_on_tasks().count());
}

#[test]
fn throttle_loop_is_the_fifty_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "throttle_loop")
        .expect("throttle_loop");
    assert!(task.rate_hz == THROTTLE_LOOP_RATE_HZ);
    assert_eq!(task.max_time_micros, THROTTLE_LOOP_MAX_TIME_MICROS);
    assert_eq!(task.priority, THROTTLE_LOOP_PRIORITY);
}

#[test]
fn remaining_leftovers_keep_later_callbacks() {
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_auto_armed"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::init_ardupilot"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::rc_loop"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::read_AHRS"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::motors_output_main"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::read_inertia"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::throttle_loop"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_flight_mode"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_land_and_crash_detectors"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_batt_compass"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::check_ekf_reset"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_home_from_EKF"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::get_wp_distance_m"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::run_nav_updates"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_rangefinder_terrain_offset"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::auto_disarm_check"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::standby_update"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::lost_vehicle_check"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::takeoff_check"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::startup_INS_ground"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::allocate_motors"));
    assert!(REMAINING.is_empty());
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::init_simple_bearing"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_simple_mode"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_super_simple_bearing"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::update_altitude"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::loop_rate_logging"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::ten_hz_logging_loop"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::twentyfive_hz_logging"));
    assert!(!REMAINING
        .iter()
        .any(|name| *name == "Copter::three_hz_loop"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::ap_value"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::one_hz_loop"));
}

#[test]
fn rc_loop_always_calls_radio_then_mode_switch() {
    let vehicle = CopterVehicleLoop::typical();
    let out = rc_loop(&vehicle.radio, vehicle.mode_switch);
    assert!(matches!(out.radio, ReadRadioLeftover::Frame { .. }));
    assert_eq!(out.mode_switch, ModeSwitchReadLeftover::Read);
}

#[test]
fn late_radio_frame_still_calls_mode_switch() {
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.radio.got_input = false;
    vehicle.radio.last_radio_update_ms = 0;
    vehicle.radio.now_ms = 2_000;
    vehicle.radio.failsafe.radio = false;
    let out = rc_loop(&vehicle.radio, vehicle.mode_switch);
    assert_eq!(out.radio, ReadRadioLeftover::LateFrame);
    assert_eq!(out.mode_switch, ModeSwitchReadLeftover::Read);
}

#[test]
fn mode_switch_refuses_invalid_input_and_missing_channel() {
    assert_eq!(
        read_mode_switch(ModeSwitchReadInputs {
            has_valid_input: false,
            flight_mode_channel: Some(4),
        }),
        ModeSwitchReadLeftover::NoValidInput
    );
    assert_eq!(
        read_mode_switch(ModeSwitchReadInputs {
            has_valid_input: true,
            flight_mode_channel: None,
        }),
        ModeSwitchReadLeftover::NoChannel
    );
}

#[test]
fn scheduler_runs_rc_loop_on_the_first_tick() {
    let tasks = [copter_rc_loop_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.rc_loop, 1);
    let leftover = vehicle.last_rc.expect("rc_loop ran");
    assert!(matches!(leftover.radio, ReadRadioLeftover::Frame { .. }));
    assert_eq!(leftover.mode_switch, ModeSwitchReadLeftover::Read);
}

#[test]
fn read_ahrs_always_skips_ins_update() {
    let leftover = read_ahrs();
    assert!(leftover.skip_ins_update);
}

#[test]
fn motors_output_main_skips_when_the_rate_thread_owns_it() {
    let vehicle = CopterVehicleLoop::typical();
    assert_eq!(
        motors_output_main(true, &vehicle.motors),
        MotorsOutputMainLeftover::Skipped
    );
}

#[test]
fn motors_output_main_forces_full_push() {
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.motors.full_push = false;
    vehicle.motors.armed = true;
    let MotorsOutputMainLeftover::Ran(out) = motors_output_main(false, &vehicle.motors) else {
        panic!("main thread must run motors_output");
    };
    assert_eq!(out.push, MotorsOutputPush::Srv);
    assert!(out.interlock);
    assert_eq!(out.drive, MotorsOutputDrive::FlightMode);
}

#[test]
fn motors_output_clears_arming_delay_on_disarm_timeout_or_throw() {
    let mut inputs = CopterVehicleLoop::typical().motors;
    inputs.in_arming_delay = true;
    inputs.armed = true;
    inputs.arm_time_ms = 0;
    inputs.now_ms = ARMING_DELAY_MS;
    // `>` not `>=` — exactly 2.0 s still holds the delay.
    let held = motors_output(&inputs);
    assert!(held.in_arming_delay);
    assert!(!held.interlock);

    inputs.now_ms = ARMING_DELAY_MS + 1;
    assert!(!motors_output(&inputs).in_arming_delay);

    inputs.now_ms = 0;
    inputs.armed = false;
    assert!(!motors_output(&inputs).in_arming_delay);

    inputs.armed = true;
    inputs.mode_number = MODE_THROW;
    assert!(!motors_output(&inputs).in_arming_delay);
}

#[test]
fn motors_output_interlock_needs_armed_cleared_delay_and_no_estop() {
    let mut inputs = CopterVehicleLoop::typical().motors;
    inputs.armed = true;
    inputs.using_interlock = true;
    inputs.motor_interlock_switch = false;
    let blocked = motors_output(&inputs);
    assert!(!blocked.interlock);
    assert_eq!(blocked.interlock_edge, InterlockEdge::None);

    inputs.motor_interlock_switch = true;
    inputs.emergency_stop = true;
    assert!(!motors_output(&inputs).interlock);

    inputs.emergency_stop = false;
    let on = motors_output(&inputs);
    assert!(on.interlock);
    assert_eq!(on.interlock_edge, InterlockEdge::Enabled);
    assert!(on.calc_pwm && on.cork && on.output_ch_all);

    inputs.motors_interlock = true;
    inputs.armed = false;
    let off = motors_output(&inputs);
    assert!(!off.interlock);
    assert_eq!(off.interlock_edge, InterlockEdge::Disabled);
}

#[test]
fn motors_output_motor_test_beats_flight_mode_and_rcout_when_not_full_push() {
    let mut inputs = CopterVehicleLoop::typical().motors;
    inputs.motor_test = true;
    inputs.full_push = false;
    let out = motors_output(&inputs);
    assert_eq!(out.drive, MotorsOutputDrive::MotorTest);
    assert_eq!(out.push, MotorsOutputPush::Rcout);
}

#[test]
fn scheduler_runs_first_fast_tasks_every_loop() {
    let tasks = copter_first_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.motors.armed = true;
    vehicle.motors.in_arming_delay = true;
    vehicle.motors.arm_time_ms = 0;
    vehicle.motors.now_ms = ARMING_DELAY_MS + 1;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 4);
    assert_eq!(vehicle.ticks.run_rate_controller_main, 1);
    assert_eq!(vehicle.ticks.motors_output_main, 1);
    assert_eq!(vehicle.ticks.read_ahrs, 1);
    assert_eq!(vehicle.ticks.read_inertia, 1);
    let rate = vehicle.last_rate.expect("rate leftover");
    assert!(rate.set_pos_control_dt && rate.set_attitude_control_dt);
    assert!(rate.set_motors_dt && rate.run_rate_controller && rate.reset_rate_target);
    assert!((rate.dt_s - 0.0025).abs() < 1e-6);
    let MotorsOutputMainLeftover::Ran(motors) = vehicle.last_motors.expect("motors leftover")
    else {
        panic!("motors_output_main ran");
    };
    assert!(!motors.in_arming_delay);
    assert!(motors.interlock);
    assert_eq!(motors.interlock_edge, InterlockEdge::Enabled);
    assert!(vehicle.last_ahrs.expect("ahrs leftover").skip_ins_update);
    let inertia = vehicle.last_inertia.expect("inertia leftover");
    assert!(inertia.update_estimates);
    assert!(inertia.altitude_updated);
    assert!(!inertia.used_home_fallback);
    assert_eq!(inertia.current_loc.lat, vehicle.inertia.ahrs_lat);
    assert_eq!(inertia.current_loc.alt_frame(), AltFrame::AboveHome);
    assert_eq!(inertia.current_loc.alt, 1_000);
}

#[test]
fn scheduler_fast_tasks_respect_the_rate_thread() {
    let tasks = copter_first_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.using_rate_thread = true;
    vehicle.motors.armed = true;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 4);
    let rate = vehicle.last_rate.expect("rate leftover");
    assert!(!rate.set_motors_dt);
    assert!(!rate.run_rate_controller);
    assert!(rate.reset_rate_target);
    assert_eq!(
        vehicle.last_motors.expect("motors leftover"),
        MotorsOutputMainLeftover::Skipped
    );
    assert!(vehicle.last_ahrs.expect("ahrs leftover").skip_ins_update);
    assert!(
        vehicle
            .last_inertia
            .expect("inertia leftover")
            .wrote_lat_lng
    );
}

#[test]
fn read_inertia_writes_lat_lng_before_the_altitude_refuse() {
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.current_loc = ap_math::location::Location::new(1, 2);
    vehicle.current_loc.set_alt_m(3.0, AltFrame::AboveHome);
    vehicle.inertia.pos_d_m = None;
    vehicle.inertia.high_vibes = true;
    let leftover = read_inertia(vehicle.current_loc, &vehicle.inertia);
    assert!(leftover.update_estimates);
    assert!(leftover.high_vibes);
    assert!(leftover.follow_update_estimates);
    assert!(leftover.wrote_lat_lng);
    assert!(!leftover.altitude_updated);
    assert!(!leftover.used_home_fallback);
    assert_eq!(leftover.current_loc.lat, vehicle.inertia.ahrs_lat);
    assert_eq!(leftover.current_loc.lng, vehicle.inertia.ahrs_lng);
    assert_eq!(leftover.current_loc.alt, 300);
    assert_eq!(leftover.current_loc.alt_frame(), AltFrame::AboveHome);
}

#[test]
fn read_inertia_falls_back_when_home_is_unset_or_the_frame_change_fails() {
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.inertia.home_is_set = false;
    let no_home = read_inertia(vehicle.current_loc, &vehicle.inertia);
    assert!(no_home.altitude_updated);
    assert!(no_home.used_home_fallback);
    assert_eq!(no_home.current_loc.alt_frame(), AltFrame::AboveHome);
    assert_eq!(no_home.current_loc.alt, 1_000);

    vehicle.inertia.home_is_set = true;
    vehicle.inertia.origin_alt_cm = None;
    let no_origin = read_inertia(vehicle.current_loc, &vehicle.inertia);
    assert!(no_origin.used_home_fallback);
    assert_eq!(no_origin.current_loc.alt_frame(), AltFrame::AboveHome);
    assert_eq!(no_origin.current_loc.alt, 1_000);
}

#[test]
fn read_inertia_converts_origin_metres_to_above_home() {
    let vehicle = CopterVehicleLoop::typical();
    let leftover = read_inertia(vehicle.current_loc, &vehicle.inertia);
    assert!(leftover.altitude_updated);
    assert!(!leftover.used_home_fallback);
    assert_eq!(leftover.current_loc.alt_frame(), AltFrame::AboveHome);
    assert_eq!(leftover.current_loc.alt, 1_000);
    assert_eq!(leftover.current_loc.lat, vehicle.inertia.ahrs_lat);
}

#[test]
fn throttle_loop_always_runs_the_stock_multicopter_callees() {
    let leftover = throttle_loop();
    assert!(leftover.update_throttle_mix);
    assert!(leftover.update_auto_armed);
    assert!(!leftover.heli_update_rotor_speed_targets);
    assert!(!leftover.heli_update_landing_swash);
    assert!(leftover.update_ground_effect_detector);
    assert!(leftover.update_ekf_terrain_height_stable);
}

#[test]
fn scheduler_runs_throttle_loop_every_eighth_tick() {
    let tasks = copter_first_scheduled_tasks();
    let mut last = [0u16; 2];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..7 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 1);
        assert_eq!(vehicle.ticks.throttle_loop, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 2);
    assert_eq!(vehicle.ticks.rc_loop, 8);
    assert_eq!(vehicle.ticks.throttle_loop, 1);
    let leftover = vehicle.last_throttle.expect("throttle_loop ran");
    assert!(leftover.update_throttle_mix);
    assert!(leftover.update_auto_armed);
    assert!(!leftover.heli_update_rotor_speed_targets);
    assert_eq!(vehicle.ticks.update_auto_armed, 1);
    let auto = vehicle.last_auto_armed.expect("update_auto_armed ran");
    assert!(!auto.auto_armed);
}

#[test]
fn update_batt_compass_is_the_ten_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "update_batt_compass")
        .expect("update_batt_compass");
    assert!(task.rate_hz == UPDATE_BATT_COMPASS_RATE_HZ);
    assert_eq!(task.max_time_micros, UPDATE_BATT_COMPASS_MAX_TIME_MICROS);
    assert_eq!(task.priority, UPDATE_BATT_COMPASS_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn update_flight_mode_reduces_gains_then_sets_reset_then_runs() {
    let leftover = update_flight_mode(UpdateFlightModeInputs {
        land_complete: true,
        move_vehicle_on_ekf_reset: false,
    });
    assert!(leftover.invalidate_for_logging);
    assert!(leftover.landed_gain_reduction);
    assert!(leftover.land_complete);
    assert_eq!(leftover.reset_handling, EkfResetMethod::MoveTarget);
    assert!(leftover.flightmode_run);
}

#[test]
fn update_flight_mode_moves_vehicle_when_the_mode_asks() {
    let leftover = update_flight_mode(UpdateFlightModeInputs {
        land_complete: false,
        move_vehicle_on_ekf_reset: true,
    });
    assert!(!leftover.land_complete);
    assert_eq!(leftover.reset_handling, EkfResetMethod::MoveVehicle);
    assert!(leftover.flightmode_run);
}

#[test]
fn update_land_and_crash_detectors_runs_stock_multicopter_callees() {
    let leftover = update_land_and_crash_detectors();
    assert!(leftover.apply_land_accel_filter);
    assert!(leftover.gravity_added_to_z);
    assert!(leftover.update_land_detector);
    assert!(!leftover.parachute_check);
    assert!(leftover.crash_check);
    assert!(leftover.thrust_loss_check);
    assert!(leftover.yaw_imbalance_check);
}

#[test]
fn update_batt_compass_reads_battery_before_compass() {
    let leftover = update_batt_compass(UpdateBattCompassInputs {
        compass_available: true,
    });
    assert!(leftover.battery_read);
    assert!(leftover.compass_set_throttle);
    assert!(leftover.compass_set_voltage);
    assert!(leftover.compass_read);
}

#[test]
fn update_batt_compass_still_reads_battery_when_compass_is_missing() {
    let leftover = update_batt_compass(UpdateBattCompassInputs {
        compass_available: false,
    });
    assert!(leftover.battery_read);
    assert!(!leftover.compass_set_throttle);
    assert!(!leftover.compass_set_voltage);
    assert!(!leftover.compass_read);
}

#[test]
fn scheduler_runs_next_fast_tasks_every_loop() {
    let tasks = copter_next_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.flight_mode.land_complete = true;
    vehicle.flight_mode.move_vehicle_on_ekf_reset = true;
    vehicle.ekf_reset.new_ekf_yaw_reset_ms = 12;
    vehicle.home_from_ekf.home_is_set = false;
    vehicle.home_from_ekf.motors_armed = true;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 4);
    assert_eq!(vehicle.ticks.check_ekf_reset, 1);
    assert_eq!(vehicle.ticks.update_flight_mode, 1);
    assert_eq!(vehicle.ticks.update_home_from_ekf, 1);
    assert_eq!(vehicle.ticks.update_land_and_crash_detectors, 1);
    let ekf = vehicle.last_ekf_reset.expect("check_ekf_reset ran");
    assert!(ekf.inertial_frame_reset_yaw);
    assert_eq!(ekf.ekf_yaw_reset_ms, 12);
    let home = vehicle
        .last_home_from_ekf
        .expect("update_home_from_EKF ran");
    assert_eq!(home.path, UpdateHomeFromEkfPath::ArmedInflight);
    assert!(home.copy_alt_from_origin);
    assert!(home.success);
    let mode = vehicle.last_flight_mode.expect("update_flight_mode ran");
    assert!(mode.landed_gain_reduction);
    assert!(mode.land_complete);
    assert_eq!(mode.reset_handling, EkfResetMethod::MoveVehicle);
    assert!(mode.flightmode_run);
    let land = vehicle
        .last_land_crash
        .expect("update_land_and_crash_detectors ran");
    assert!(land.update_land_detector);
    assert!(land.crash_check);
    assert!(!land.parachute_check);
}

#[test]
fn scheduler_runs_update_batt_compass_every_fortieth_tick() {
    let tasks = copter_next_scheduled_tasks();
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..39 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.update_batt_compass, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.update_batt_compass, 1);
    let leftover = vehicle.last_batt_compass.expect("update_batt_compass ran");
    assert!(leftover.battery_read);
    assert!(leftover.compass_read);
}

#[test]
fn loop_rate_logging_is_the_loop_rate_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "loop_rate_logging")
        .expect("loop_rate_logging");
    assert!(task.rate_hz == LOOP_RATE);
    assert_eq!(task.max_time_micros, LOOP_RATE_LOGGING_MAX_TIME_MICROS);
    assert_eq!(task.priority, LOOP_RATE_LOGGING_PRIORITY);
    assert_eq!(task.gate, Some("HAL_LOGGING_ENABLED"));
}

#[test]
fn ten_hz_logging_is_the_ten_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "ten_hz_logging_loop")
        .expect("ten_hz_logging_loop");
    assert!(task.rate_hz == TEN_HZ_LOGGING_RATE_HZ);
    assert_eq!(task.max_time_micros, TEN_HZ_LOGGING_MAX_TIME_MICROS);
    assert_eq!(task.priority, TEN_HZ_LOGGING_PRIORITY);
}

#[test]
fn twentyfive_hz_logging_is_the_twentyfive_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "twentyfive_hz_logging")
        .expect("twentyfive_hz_logging");
    assert!(task.rate_hz == TWENTYFIVE_HZ_LOGGING_RATE_HZ);
    assert_eq!(task.max_time_micros, TWENTYFIVE_HZ_LOGGING_MAX_TIME_MICROS);
    assert_eq!(task.priority, TWENTYFIVE_HZ_LOGGING_PRIORITY);
}

#[test]
fn three_hz_loop_is_the_three_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "three_hz_loop")
        .expect("three_hz_loop");
    assert!(task.rate_hz == THREE_HZ_LOOP_RATE_HZ);
    assert_eq!(task.max_time_micros, THREE_HZ_LOOP_MAX_TIME_MICROS);
    assert_eq!(task.priority, THREE_HZ_LOOP_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn one_hz_loop_is_the_one_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "one_hz_loop")
        .expect("one_hz_loop");
    assert!(task.rate_hz == ONE_HZ_LOOP_RATE_HZ);
    assert_eq!(task.max_time_micros, ONE_HZ_LOOP_MAX_TIME_MICROS);
    assert_eq!(task.priority, ONE_HZ_LOOP_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn loop_rate_logging_always_writes_spol_and_skips_att_on_default_bitmask() {
    let leftover = loop_rate_logging(LoopRateLoggingInputs {
        log_bitmask: DEFAULT_LOG_BITMASK,
        logs_attitude: false,
        using_rate_thread: false,
    });
    assert!(leftover.write_spol);
    assert!(!leftover.write_attitude);
    assert!(!leftover.write_rate);
    assert!(!leftover.write_pids);
    assert!(!leftover.write_notch);
    assert!(!leftover.write_imu);
}

#[test]
fn loop_rate_logging_writes_att_rate_pid_when_fast_and_mode_does_not() {
    let leftover = loop_rate_logging(LoopRateLoggingInputs {
        log_bitmask: MASK_LOG_ATTITUDE_FAST | MASK_LOG_IMU_FAST,
        logs_attitude: false,
        using_rate_thread: false,
    });
    assert!(leftover.write_attitude);
    assert!(leftover.write_rate);
    assert!(leftover.write_pids);
    assert!(leftover.write_imu);
    assert!(leftover.write_spol);
}

#[test]
fn loop_rate_logging_skips_rate_and_pid_on_the_rate_thread() {
    let leftover = loop_rate_logging(LoopRateLoggingInputs {
        log_bitmask: MASK_LOG_ATTITUDE_FAST,
        logs_attitude: false,
        using_rate_thread: true,
    });
    assert!(leftover.write_attitude);
    assert!(!leftover.write_rate);
    assert!(!leftover.write_pids);
    assert!(leftover.write_spol);
}

#[test]
fn loop_rate_logging_skips_att_when_the_mode_already_logs_it() {
    let leftover = loop_rate_logging(LoopRateLoggingInputs {
        log_bitmask: MASK_LOG_ATTITUDE_FAST,
        logs_attitude: true,
        using_rate_thread: false,
    });
    assert!(!leftover.write_attitude);
    assert!(!leftover.write_rate);
    assert!(!leftover.write_pids);
    assert!(leftover.write_spol);
}

#[test]
fn ten_hz_logging_always_writes_ahrs_attitude() {
    let leftover = ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: 0,
        logs_attitude: true,
        using_rate_thread: true,
        requires_position: false,
        landing_with_gps: false,
        has_manual_throttle: true,
    });
    assert!(leftover.write_ahrs_attitude);
    assert!(!leftover.write_attitude);
    assert!(!leftover.write_rate);
    assert!(!leftover.write_pids);
    assert!(leftover.write_ekf_pos);
    assert!(!leftover.write_motors);
    assert!(!leftover.write_rcin);
    assert!(!leftover.write_rssi);
    assert!(!leftover.write_ntun);
    assert!(!leftover.write_proximity);
}

#[test]
fn ten_hz_logging_default_bitmask_writes_med_att_and_ntun_for_position_modes() {
    let leftover = ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: DEFAULT_LOG_BITMASK,
        logs_attitude: false,
        using_rate_thread: false,
        requires_position: true,
        landing_with_gps: false,
        has_manual_throttle: false,
    });
    assert!(leftover.write_ahrs_attitude);
    assert!(leftover.write_attitude);
    assert!(leftover.write_rate);
    assert!(leftover.write_pids);
    assert!(leftover.write_ekf_pos);
    assert!(leftover.write_motors);
    assert!(leftover.write_rcin);
    assert!(leftover.write_rcout);
    assert!(leftover.write_ntun);
    assert!(leftover.write_vibration);
    assert!(!leftover.write_rssi);
    assert!(!leftover.write_mount);
}

#[test]
fn ten_hz_logging_skips_med_att_when_fast_is_set() {
    let leftover = ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: DEFAULT_LOG_BITMASK | MASK_LOG_ATTITUDE_FAST,
        logs_attitude: false,
        using_rate_thread: false,
        requires_position: true,
        landing_with_gps: false,
        has_manual_throttle: false,
    });
    assert!(!leftover.write_attitude);
    assert!(!leftover.write_rate);
    assert!(!leftover.write_pids);
    assert!(!leftover.write_ekf_pos);
}

#[test]
fn ten_hz_logging_ntun_refuses_manual_throttle_without_position() {
    let leftover = ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: MASK_LOG_NTUN,
        logs_attitude: false,
        using_rate_thread: false,
        requires_position: false,
        landing_with_gps: false,
        has_manual_throttle: true,
    });
    assert!(!leftover.write_ntun);

    let landing = ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: MASK_LOG_NTUN,
        logs_attitude: false,
        using_rate_thread: false,
        requires_position: false,
        landing_with_gps: true,
        has_manual_throttle: true,
    });
    assert!(landing.write_ntun);
}

#[test]
fn twentyfive_hz_logging_moves_ekf_pos_when_att_fast() {
    let leftover = twentyfive_hz_logging(TwentyfiveHzLoggingInputs {
        log_bitmask: MASK_LOG_ATTITUDE_FAST | MASK_LOG_IMU,
    });
    assert!(leftover.write_ekf_pos);
    assert!(leftover.write_imu);
    assert!(!leftover.write_gyro_fft);
}

#[test]
fn twentyfive_hz_logging_skips_imu_when_fast_already_wrote_it() {
    let leftover = twentyfive_hz_logging(TwentyfiveHzLoggingInputs {
        log_bitmask: MASK_LOG_IMU | MASK_LOG_IMU_FAST,
    });
    assert!(!leftover.write_ekf_pos);
    assert!(!leftover.write_imu);
}

#[test]
fn three_hz_loop_runs_stock_multicopter_callees() {
    let leftover = three_hz_loop();
    assert!(leftover.failsafe_gcs_check);
    assert!(leftover.failsafe_terrain_check);
    assert!(leftover.failsafe_deadreckon_check);
    assert!(!leftover.tuning);
    assert!(leftover.low_alt_avoidance);
}

#[test]
fn ap_value_walks_packed_bools_in_declaration_order() {
    assert_eq!(ap_value(ApState::default()), 0);

    let mut ap = ApState::default();
    ap.land_complete = true;
    assert_eq!(ap_value(ap), 1 << 7);

    ap.auto_armed = true;
    assert_eq!(ap_value(ap), (1 << 5) | (1 << 7));

    ap.prec_land_active = true;
    assert_eq!(ap_value(ap), (1 << 5) | (1 << 7) | (1 << 26));
}

#[test]
fn one_hz_loop_logs_ap_state_only_for_low_sixteen_bits() {
    let any = one_hz_loop(OneHzLoopInputs {
        log_bitmask: MASK_LOG_ATTITUDE_MED,
        motors_armed: false,
        using_rate_thread: false,
        land_complete: false,
    });
    assert!(any.log_ap_state);
    assert!(any.update_using_interlock);
    assert!(any.set_frame_class_and_type);
    assert!(any.update_throttle_range);
    assert!(any.enable_aux_servos);
    assert!(any.terrain_logging);
    assert!(!any.adsb_set_is_flying);
    assert!(any.notify_flying);
    assert!(any.flying);
    assert!(any.attitude_notch_sample_rate);
    assert!(any.pos_control_notch_sample_rate);
    assert!(!any.start_rate_thread);

    let motbatt_only = one_hz_loop(OneHzLoopInputs {
        log_bitmask: MASK_LOG_MOTBATT,
        motors_armed: true,
        using_rate_thread: true,
        land_complete: true,
    });
    assert!(!motbatt_only.log_ap_state);
    assert!(!motbatt_only.update_using_interlock);
    assert!(!motbatt_only.set_frame_class_and_type);
    assert!(!motbatt_only.update_throttle_range);
    assert!(motbatt_only.enable_aux_servos);
    assert!(!motbatt_only.flying);
    assert!(!motbatt_only.attitude_notch_sample_rate);
    assert!(motbatt_only.pos_control_notch_sample_rate);
    assert_eq!(should_log(MASK_LOG_MOTBATT, MASK_LOG_ANY), false);
}

#[test]
fn scheduler_runs_loop_rate_logging_every_loop() {
    let tasks = copter_logging_tasks();
    let mut last = [0u16; 3];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.log_bitmask = MASK_LOG_ATTITUDE_FAST;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.loop_rate_logging, 1);
    assert_eq!(vehicle.ticks.ten_hz_logging_loop, 0);
    assert_eq!(vehicle.ticks.twentyfive_hz_logging, 0);
    let leftover = vehicle
        .last_loop_rate_logging
        .expect("loop_rate_logging ran");
    assert!(leftover.write_attitude);
    assert!(leftover.write_spol);
}

#[test]
fn scheduler_runs_twentyfive_hz_logging_every_sixteenth_tick() {
    let tasks = copter_logging_tasks();
    let mut last = [0u16; 3];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.log_bitmask = MASK_LOG_ATTITUDE_FAST | MASK_LOG_IMU;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..15 {
        let _ = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(vehicle.ticks.twentyfive_hz_logging, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert!(stats.tasks_run >= 2);
    assert_eq!(vehicle.ticks.loop_rate_logging, 16);
    assert_eq!(vehicle.ticks.twentyfive_hz_logging, 1);
    let leftover = vehicle
        .last_twentyfive_hz_logging
        .expect("twentyfive_hz_logging ran");
    assert!(leftover.write_ekf_pos);
    assert!(leftover.write_imu);
}

#[test]
fn scheduler_runs_ten_hz_logging_every_fortieth_tick() {
    let tasks = copter_logging_tasks();
    let mut last = [0u16; 3];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..39 {
        let _ = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(vehicle.ticks.ten_hz_logging_loop, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert!(stats.tasks_run >= 2);
    assert_eq!(vehicle.ticks.loop_rate_logging, 40);
    assert_eq!(vehicle.ticks.ten_hz_logging_loop, 1);
    let leftover = vehicle
        .last_ten_hz_logging
        .expect("ten_hz_logging_loop ran");
    assert!(leftover.write_ahrs_attitude);
    assert!(leftover.write_attitude);
    assert!(leftover.write_ntun);
}

#[test]
fn scheduler_runs_three_hz_then_one_hz() {
    let tasks = copter_periodic_loop_tasks();
    let mut last = [0u16; 2];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.flight_mode.land_complete = true;
    vehicle.motors.armed = true;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..132 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.three_hz_loop, 0);
        assert_eq!(vehicle.ticks.one_hz_loop, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.three_hz_loop, 1);
    assert_eq!(vehicle.ticks.one_hz_loop, 0);
    let three = vehicle.last_three_hz.expect("three_hz_loop ran");
    assert!(three.failsafe_gcs_check);
    assert!(three.low_alt_avoidance);

    for _ in 0..266 {
        let _ = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(vehicle.ticks.one_hz_loop, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(vehicle.ticks.one_hz_loop, 1);
    assert!(stats.tasks_run >= 1);
    let one = vehicle.last_one_hz.expect("one_hz_loop ran");
    assert!(one.log_ap_state);
    assert!(!one.update_using_interlock);
    assert!(!one.flying);
}

#[test]
fn update_altitude_is_the_ten_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "update_altitude")
        .expect("update_altitude");
    assert!(task.rate_hz == UPDATE_ALTITUDE_RATE_HZ);
    assert_eq!(task.max_time_micros, UPDATE_ALTITUDE_MAX_TIME_MICROS);
    assert_eq!(task.priority, UPDATE_ALTITUDE_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn update_altitude_always_reads_baro_and_writes_ctun_on_default_bitmask() {
    let leftover = update_altitude(UpdateAltitudeInputs {
        log_bitmask: DEFAULT_LOG_BITMASK,
    });
    assert!(leftover.read_barometer);
    assert!(leftover.write_control_tuning);
    assert!(!leftover.write_notch);
    assert!(!leftover.write_gyro_fft);
}

#[test]
fn update_altitude_skips_ctun_when_that_bit_is_clear() {
    let leftover = update_altitude(UpdateAltitudeInputs { log_bitmask: 0 });
    assert!(leftover.read_barometer);
    assert!(!leftover.write_control_tuning);
    assert!(!leftover.write_notch);
    assert!(!leftover.write_gyro_fft);
}

#[test]
fn scheduler_runs_update_altitude_every_fortieth_tick() {
    use ap_copter::vehicle_loop::copter_update_altitude_task;
    let tasks = [copter_update_altitude_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..39 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.update_altitude, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.update_altitude, 1);
    let leftover = vehicle.last_update_altitude.expect("update_altitude ran");
    assert!(leftover.read_barometer);
    assert!(leftover.write_control_tuning);
}

#[test]
fn check_ekf_reset_is_silent_when_timestamps_and_core_match() {
    let leftover = check_ekf_reset(CheckEkfResetInputs {
        ekf_yaw_reset_ms: 40,
        new_ekf_yaw_reset_ms: 40,
        ekf_primary_core: 0,
        primary_core_index: 0,
    });
    assert!(!leftover.inertial_frame_reset_yaw);
    assert!(!leftover.log_ekf_yaw_reset);
    assert_eq!(leftover.ekf_yaw_reset_ms, 40);
    assert!(!leftover.inertial_frame_reset_primary);
    assert!(!leftover.log_ekf_primary);
    assert!(!leftover.gcs_ekf_primary_changed);
    assert_eq!(leftover.ekf_primary_core, 0);
}

#[test]
fn check_ekf_reset_yaw_uses_timestamp_not_angle() {
    let leftover = check_ekf_reset(CheckEkfResetInputs {
        ekf_yaw_reset_ms: 0,
        new_ekf_yaw_reset_ms: 7,
        ekf_primary_core: 1,
        primary_core_index: 1,
    });
    assert!(leftover.inertial_frame_reset_yaw);
    assert!(leftover.log_ekf_yaw_reset);
    assert_eq!(leftover.ekf_yaw_reset_ms, 7);
    assert!(!leftover.inertial_frame_reset_primary);
}

#[test]
fn check_ekf_reset_ignores_primary_core_minus_one() {
    let leftover = check_ekf_reset(CheckEkfResetInputs {
        ekf_yaw_reset_ms: 1,
        new_ekf_yaw_reset_ms: 1,
        ekf_primary_core: 0,
        primary_core_index: -1,
    });
    assert!(!leftover.inertial_frame_reset_primary);
    assert_eq!(leftover.ekf_primary_core, 0);
    assert!(!leftover.log_ekf_primary);
    assert!(!leftover.gcs_ekf_primary_changed);
}

#[test]
fn check_ekf_reset_can_fire_yaw_and_primary_on_the_same_tick() {
    let leftover = check_ekf_reset(CheckEkfResetInputs {
        ekf_yaw_reset_ms: 3,
        new_ekf_yaw_reset_ms: 9,
        ekf_primary_core: 0,
        primary_core_index: 1,
    });
    assert!(leftover.inertial_frame_reset_yaw);
    assert!(leftover.inertial_frame_reset_primary);
    assert_eq!(leftover.ekf_yaw_reset_ms, 9);
    assert_eq!(leftover.ekf_primary_core, 1);
    assert!(leftover.log_ekf_primary);
    assert!(leftover.gcs_ekf_primary_changed);
}

#[test]
fn update_home_from_ekf_returns_when_home_is_already_set() {
    let leftover = update_home_from_ekf(UpdateHomeFromEkfInputs {
        home_is_set: true,
        motors_armed: true,
        got_location: true,
        got_origin: true,
        ahrs_set_home_ok: true,
    });
    assert_eq!(leftover.path, UpdateHomeFromEkfPath::HomeAlreadySet);
    assert!(!leftover.copy_alt_from_origin);
    assert!(!leftover.set_home);
    assert!(!leftover.smart_rtl_set_home);
}

#[test]
fn update_home_from_ekf_inflight_copies_origin_alt_before_set_home() {
    let leftover = update_home_from_ekf(UpdateHomeFromEkfInputs {
        home_is_set: false,
        motors_armed: true,
        got_location: true,
        got_origin: true,
        ahrs_set_home_ok: true,
    });
    assert_eq!(leftover.path, UpdateHomeFromEkfPath::ArmedInflight);
    assert!(leftover.copy_alt_from_origin);
    assert!(leftover.set_home);
    assert!(!leftover.lock_home);
    assert!(leftover.smart_rtl_set_home);
    assert!(leftover.success);
}

#[test]
fn update_home_from_ekf_inflight_refuses_without_origin() {
    let leftover = update_home_from_ekf(UpdateHomeFromEkfInputs {
        home_is_set: false,
        motors_armed: true,
        got_location: true,
        got_origin: false,
        ahrs_set_home_ok: true,
    });
    assert_eq!(leftover.path, UpdateHomeFromEkfPath::ArmedInflight);
    assert!(!leftover.copy_alt_from_origin);
    assert!(!leftover.set_home);
    assert!(!leftover.smart_rtl_set_home);
}

#[test]
fn update_home_from_ekf_disarmed_ignores_set_home_failure() {
    let leftover = update_home_from_ekf(UpdateHomeFromEkfInputs {
        home_is_set: false,
        motors_armed: false,
        got_location: true,
        got_origin: true,
        ahrs_set_home_ok: false,
    });
    assert_eq!(leftover.path, UpdateHomeFromEkfPath::DisarmedGround);
    assert!(!leftover.copy_alt_from_origin);
    assert!(leftover.set_home);
    assert!(!leftover.smart_rtl_set_home);
    assert!(!leftover.success);
}

#[test]
fn set_home_requires_origin_and_locks_only_after_success() {
    assert!(
        !set_home(SetHomeInputs {
            got_origin: false,
            ahrs_set_home_ok: true,
            lock: true,
        })
        .set_ahrs_home
    );
    let refused = set_home(SetHomeInputs {
        got_origin: true,
        ahrs_set_home_ok: false,
        lock: true,
    });
    assert!(refused.set_ahrs_home);
    assert!(!refused.lock_home);
    let locked = set_home(SetHomeInputs {
        got_origin: true,
        ahrs_set_home_ok: true,
        lock: true,
    });
    assert!(locked.success);
    assert!(locked.lock_home);
}

#[test]
fn set_home_to_current_location_seeds_smartrtl_only_on_success() {
    let ok = set_home_to_current_location(SetHomeToCurrentLocationInputs {
        got_location: true,
        set_home_ok: true,
        lock: true,
    });
    assert!(ok.set_home);
    assert!(ok.set_home_lock);
    assert!(ok.smart_rtl_set_home);
    assert!(ok.success);
    let no_loc = set_home_to_current_location(SetHomeToCurrentLocationInputs {
        got_location: false,
        set_home_ok: true,
        lock: true,
    });
    assert!(!no_loc.set_home);
    assert!(!no_loc.smart_rtl_set_home);
}

#[test]
fn set_home_to_current_location_inflight_needs_both_location_and_origin() {
    let leftover = set_home_to_current_location_inflight(SetHomeToCurrentLocationInflightInputs {
        got_location: true,
        got_origin: false,
        set_home_ok: true,
    });
    assert!(!leftover.copy_alt_from_origin);
    assert!(!leftover.set_home);
    let ok = set_home_to_current_location_inflight(SetHomeToCurrentLocationInflightInputs {
        got_location: true,
        got_origin: true,
        set_home_ok: false,
    });
    assert!(ok.copy_alt_from_origin);
    assert!(ok.set_home);
    assert!(!ok.smart_rtl_set_home);
}

#[test]
fn get_wp_distance_m_always_returns_the_mode_distance() {
    let leftover = get_wp_distance_m(0.0);
    assert!(leftover.ok);
    assert_eq!(leftover.distance_m, 0.0);
    let leftover = get_wp_distance_m(42.5);
    assert!(leftover.ok);
    assert_eq!(leftover.distance_m, 42.5);
}

#[test]
fn run_nav_updates_always_calls_super_simple_without_force() {
    let leftover = run_nav_updates();
    assert!(leftover.update_super_simple_bearing);
    assert!(!leftover.force_update);
}

#[test]
fn run_nav_updates_is_the_fifty_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "run_nav_updates")
        .expect("run_nav_updates");
    assert!(task.rate_hz == RUN_NAV_UPDATES_RATE_HZ);
    assert_eq!(task.max_time_micros, RUN_NAV_UPDATES_MAX_TIME_MICROS);
    assert_eq!(task.priority, RUN_NAV_UPDATES_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn scheduler_runs_run_nav_updates_every_eighth_tick() {
    use ap_copter::vehicle_loop::copter_run_nav_updates_task;
    let tasks = [copter_run_nav_updates_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..7 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.run_nav_updates, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.run_nav_updates, 1);
    let leftover = vehicle.last_run_nav_updates.expect("run_nav_updates ran");
    assert!(leftover.update_super_simple_bearing);
    assert!(!leftover.force_update);
}

#[test]
fn update_rangefinder_terrain_offset_filters_even_when_unhealthy() {
    let leftover = update_rangefinder_terrain_offset(UpdateRangefinderTerrainOffsetInputs {
        down: RangefinderTerrainState {
            ref_pos_u_m: 10.0,
            alt_glitch_protected_m: 0.0,
            terrain_u_m: 0.0,
            enabled: true,
            alt_healthy: false,
            data_stale: false,
        },
        up: RangefinderTerrainState {
            ref_pos_u_m: 4.0,
            alt_glitch_protected_m: 0.0,
            terrain_u_m: 0.0,
            enabled: false,
            alt_healthy: false,
            data_stale: false,
        },
        g_dt: 0.1,
        surftrak_tc: SURFTRAK_TC_DEFAULT,
        wp_nav_rangefinder_used: true,
    });
    assert!((leftover.down_terrain_u_m - 1.0).abs() < 1.0e-6);
    assert!((leftover.up_terrain_u_m - 0.4).abs() < 1.0e-6);
    assert!(!leftover.publish_wp_nav);
    assert!(!leftover.publish_circle_nav);
}

#[test]
fn update_rangefinder_terrain_offset_down_subtracts_and_up_adds() {
    let leftover = update_rangefinder_terrain_offset(UpdateRangefinderTerrainOffsetInputs {
        down: RangefinderTerrainState {
            ref_pos_u_m: 12.0,
            alt_glitch_protected_m: 2.0,
            terrain_u_m: 10.0,
            enabled: true,
            alt_healthy: true,
            data_stale: false,
        },
        up: RangefinderTerrainState {
            ref_pos_u_m: 3.0,
            alt_glitch_protected_m: 1.0,
            terrain_u_m: 4.0,
            enabled: true,
            alt_healthy: true,
            data_stale: false,
        },
        g_dt: 0.1,
        surftrak_tc: 0.0,
        wp_nav_rangefinder_used: false,
    });
    assert!((leftover.down_terrain_u_m - 10.0).abs() < 1.0e-6);
    assert!((leftover.up_terrain_u_m - 4.0).abs() < 1.0e-6);
    assert!(leftover.publish_wp_nav);
    assert!(leftover.publish_circle_nav);
    assert!(leftover.wp_nav_enabled);
    assert!(!leftover.circle_nav_enabled);
}

#[test]
fn update_rangefinder_terrain_offset_publishes_when_stale_even_if_unhealthy() {
    let leftover = update_rangefinder_terrain_offset(UpdateRangefinderTerrainOffsetInputs {
        down: RangefinderTerrainState {
            ref_pos_u_m: 5.0,
            alt_glitch_protected_m: 0.0,
            terrain_u_m: 5.0,
            enabled: true,
            alt_healthy: false,
            data_stale: true,
        },
        up: RangefinderTerrainState {
            ref_pos_u_m: 0.0,
            alt_glitch_protected_m: 0.0,
            terrain_u_m: 0.0,
            enabled: false,
            alt_healthy: true,
            data_stale: false,
        },
        g_dt: 0.1,
        surftrak_tc: SURFTRAK_TC_DEFAULT,
        wp_nav_rangefinder_used: true,
    });
    assert!(leftover.publish_wp_nav);
    assert!(leftover.publish_circle_nav);
    assert!(!leftover.wp_nav_healthy);
    assert!(leftover.circle_nav_enabled);
    assert!(!leftover.circle_nav_healthy);
}

#[test]
fn scheduler_runs_rangefinder_terrain_offset_every_loop() {
    use ap_copter::vehicle_loop::copter_later_fast_tasks;
    let tasks = copter_later_fast_tasks();
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.rangefinder_terrain.down.ref_pos_u_m = 10.0;
    vehicle.rangefinder_terrain.down.enabled = true;
    vehicle.rangefinder_terrain.down.alt_healthy = true;
    vehicle.rangefinder_terrain.g_dt = 0.1;
    vehicle.rangefinder_terrain.surftrak_tc = 1.0;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.update_rangefinder_terrain_offset, 1);
    let leftover = vehicle
        .last_rangefinder_terrain
        .expect("update_rangefinder_terrain_offset ran");
    assert!((leftover.down_terrain_u_m - 1.0).abs() < 1.0e-6);
    assert!(leftover.publish_wp_nav);
}

fn landed_auto_disarm() -> AutoDisarmCheckInputs {
    AutoDisarmCheckInputs {
        now_ms: 11_000,
        auto_disarm_begin_ms: 1_000,
        disarm_delay_s: AUTO_DISARMING_DELAY,
        motors_armed: true,
        throw_mode: false,
        desired_spool_above_ground_idle: false,
        spool_above_ground_idle: false,
        using_interlock: false,
        motors_interlock: false,
        emergency_stop: false,
        throttle_behavior: 0,
        has_manual_throttle: false,
        throttle_zero: true,
        throttle_control_in: 0,
        throttle_mid: 500,
        throttle_deadzone: 100,
        land_complete: true,
    }
}

#[test]
fn auto_disarm_delay_ms_constrains_to_int8_max() {
    assert_eq!(auto_disarm_delay_ms(AUTO_DISARMING_DELAY), 10_000);
    assert_eq!(auto_disarm_delay_ms(0), 0);
    assert_eq!(auto_disarm_delay_ms(-3), 0);
    assert_eq!(auto_disarm_delay_ms(200), 1_000 * DISARM_DELAY_MAX_S as u32);
}

#[test]
fn auto_disarm_check_resets_when_disarmed_disabled_or_throw() {
    let mut inputs = landed_auto_disarm();
    inputs.motors_armed = false;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::ResetDisarmedOrDisabled);
    assert_eq!(leftover.auto_disarm_begin_ms, inputs.now_ms);
    assert!(!leftover.disarm);

    inputs = landed_auto_disarm();
    inputs.disarm_delay_s = 0;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::ResetDisarmedOrDisabled);
    assert!(!leftover.disarm);

    inputs = landed_auto_disarm();
    inputs.throw_mode = true;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::ResetDisarmedOrDisabled);
    assert!(!leftover.disarm);
}

#[test]
fn auto_disarm_check_desired_or_actual_spool_inhibits() {
    let mut inputs = landed_auto_disarm();
    inputs.desired_spool_above_ground_idle = true;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::ResetSpooling);
    assert_eq!(leftover.auto_disarm_begin_ms, inputs.now_ms);
    assert!(!leftover.disarm);

    inputs = landed_auto_disarm();
    inputs.spool_above_ground_idle = true;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::ResetSpooling);
    assert!(!leftover.disarm);
}

#[test]
fn auto_disarm_check_disarms_after_delay_when_landed_and_throttle_low() {
    let leftover = auto_disarm_check(landed_auto_disarm());
    assert_eq!(leftover.path, AutoDisarmCheckPath::LandedThrottle);
    assert!(leftover.disarm);
    assert_eq!(leftover.auto_disarm_begin_ms, 11_000);
    assert_eq!(leftover.disarm_delay_ms, 10_000);
}

#[test]
fn auto_disarm_check_resets_when_not_landed_or_throttle_not_low() {
    let mut inputs = landed_auto_disarm();
    inputs.land_complete = false;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::LandedThrottle);
    assert_eq!(leftover.auto_disarm_begin_ms, inputs.now_ms);
    assert!(!leftover.disarm);

    inputs = landed_auto_disarm();
    inputs.throttle_zero = false;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.auto_disarm_begin_ms, inputs.now_ms);
    assert!(!leftover.disarm);
}

#[test]
fn auto_disarm_check_sprung_stick_uses_deadband_not_throttle_zero() {
    let mut inputs = landed_auto_disarm();
    inputs.throttle_behavior = THR_BEHAVE_FEEDBACK_FROM_MID_STICK;
    inputs.has_manual_throttle = false;
    inputs.throttle_zero = false;
    inputs.throttle_control_in = 600;
    let leftover = auto_disarm_check(inputs);
    assert!(leftover.disarm);

    inputs.throttle_control_in = 601;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.auto_disarm_begin_ms, inputs.now_ms);
    assert!(!leftover.disarm);
}

#[test]
fn auto_disarm_check_interlock_halves_delay_and_keeps_timer() {
    let mut inputs = landed_auto_disarm();
    inputs.using_interlock = true;
    inputs.motors_interlock = false;
    inputs.now_ms = 6_000;
    inputs.auto_disarm_begin_ms = 1_000;
    inputs.land_complete = false;
    let leftover = auto_disarm_check(inputs);
    assert_eq!(leftover.path, AutoDisarmCheckPath::InterlockOrEstop);
    assert_eq!(leftover.disarm_delay_ms, 5_000);
    assert!(leftover.disarm);
    assert_eq!(leftover.auto_disarm_begin_ms, 6_000);
}

#[test]
fn auto_disarm_check_is_the_ten_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "auto_disarm_check")
        .expect("auto_disarm_check");
    assert!(task.rate_hz == AUTO_DISARM_CHECK_RATE_HZ);
    assert_eq!(task.max_time_micros, AUTO_DISARM_CHECK_MAX_TIME_MICROS);
    assert_eq!(task.priority, AUTO_DISARM_CHECK_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn scheduler_runs_auto_disarm_check_every_fortieth_tick() {
    use ap_copter::vehicle_loop::copter_auto_disarm_check_task;
    let tasks = [copter_auto_disarm_check_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.auto_disarm = landed_auto_disarm();
    vehicle.auto_disarm.now_ms = 12_000;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..39 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.auto_disarm_check, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.auto_disarm_check, 1);
    let leftover = vehicle.last_auto_disarm.expect("auto_disarm_check ran");
    assert!(leftover.disarm);
}

#[test]
fn standby_update_is_noop_until_active() {
    let leftover = standby_update(false);
    assert!(!leftover.reset_rate_i_terms);
    assert!(!leftover.reset_yaw_target_and_rate);
    assert!(!leftover.ned_standby_reset);

    let leftover = standby_update(true);
    assert!(leftover.reset_rate_i_terms);
    assert!(leftover.reset_yaw_target_and_rate);
    assert!(leftover.ned_standby_reset);
}

#[test]
fn standby_update_is_the_hundred_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "standby_update")
        .expect("standby_update");
    assert!(task.rate_hz == STANDBY_UPDATE_RATE_HZ);
    assert_eq!(task.max_time_micros, STANDBY_UPDATE_MAX_TIME_MICROS);
    assert_eq!(task.priority, STANDBY_UPDATE_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn scheduler_runs_standby_update_every_fourth_tick() {
    use ap_copter::vehicle_loop::copter_standby_update_task;
    let tasks = [copter_standby_update_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.standby_active = true;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..3 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.standby_update, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.standby_update, 1);
    let leftover = vehicle.last_standby.expect("standby_update ran");
    assert!(leftover.reset_rate_i_terms);
    assert!(leftover.ned_standby_reset);
}

#[test]
fn lost_vehicle_check_aux_assigned_is_a_full_refuse() {
    let leftover = lost_vehicle_check(LostVehicleCheckInputs {
        lost_vehicle_sound_aux_assigned: true,
        throttle_zero: true,
        motors_armed: false,
        roll_control_in: LOST_VEHICLE_STICK_MAX + 1,
        pitch_control_in: LOST_VEHICLE_STICK_MAX + 1,
        soundalarm_counter: 3,
        vehicle_lost: true,
    });
    assert_eq!(leftover.soundalarm_counter, 3);
    assert!(leftover.vehicle_lost);
    assert!(!leftover.gcs_locate_copter_alarm);
}

#[test]
fn lost_vehicle_check_requires_sticks_above_4000() {
    let leftover = lost_vehicle_check(LostVehicleCheckInputs {
        lost_vehicle_sound_aux_assigned: false,
        throttle_zero: true,
        motors_armed: false,
        roll_control_in: LOST_VEHICLE_STICK_MAX,
        pitch_control_in: LOST_VEHICLE_STICK_MAX + 1,
        soundalarm_counter: 9,
        vehicle_lost: false,
    });
    assert_eq!(leftover.soundalarm_counter, 0);
    assert!(!leftover.vehicle_lost);
}

#[test]
fn lost_vehicle_check_counts_then_latches_on_rising_edge() {
    let mut inputs = LostVehicleCheckInputs {
        lost_vehicle_sound_aux_assigned: false,
        throttle_zero: true,
        motors_armed: false,
        roll_control_in: LOST_VEHICLE_STICK_MAX + 1,
        pitch_control_in: LOST_VEHICLE_STICK_MAX + 1,
        soundalarm_counter: 0,
        vehicle_lost: false,
    };
    for expected in 1..=LOST_VEHICLE_DELAY {
        let leftover = lost_vehicle_check(inputs);
        assert_eq!(leftover.soundalarm_counter, expected);
        assert!(!leftover.vehicle_lost);
        assert!(!leftover.gcs_locate_copter_alarm);
        inputs.soundalarm_counter = leftover.soundalarm_counter;
    }
    let leftover = lost_vehicle_check(inputs);
    assert_eq!(leftover.soundalarm_counter, LOST_VEHICLE_DELAY);
    assert!(leftover.vehicle_lost);
    assert!(leftover.gcs_locate_copter_alarm);

    inputs.soundalarm_counter = leftover.soundalarm_counter;
    inputs.vehicle_lost = leftover.vehicle_lost;
    let leftover = lost_vehicle_check(inputs);
    assert!(leftover.vehicle_lost);
    assert!(!leftover.gcs_locate_copter_alarm);
}

#[test]
fn lost_vehicle_check_clears_immediately_when_sticks_leave() {
    let leftover = lost_vehicle_check(LostVehicleCheckInputs {
        lost_vehicle_sound_aux_assigned: false,
        throttle_zero: true,
        motors_armed: false,
        roll_control_in: 0,
        pitch_control_in: 0,
        soundalarm_counter: LOST_VEHICLE_DELAY,
        vehicle_lost: true,
    });
    assert_eq!(leftover.soundalarm_counter, 0);
    assert!(!leftover.vehicle_lost);
    assert!(!leftover.gcs_locate_copter_alarm);
}

#[test]
fn lost_vehicle_check_is_the_ten_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "lost_vehicle_check")
        .expect("lost_vehicle_check");
    assert!(task.rate_hz == LOST_VEHICLE_CHECK_RATE_HZ);
    assert_eq!(task.max_time_micros, LOST_VEHICLE_CHECK_MAX_TIME_MICROS);
    assert_eq!(task.priority, LOST_VEHICLE_CHECK_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn scheduler_runs_lost_vehicle_check_every_fortieth_tick() {
    use ap_copter::vehicle_loop::copter_lost_vehicle_check_task;
    let tasks = [copter_lost_vehicle_check_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.lost_vehicle.roll_control_in = LOST_VEHICLE_STICK_MAX + 1;
    vehicle.lost_vehicle.pitch_control_in = LOST_VEHICLE_STICK_MAX + 1;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..39 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.lost_vehicle_check, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.lost_vehicle_check, 1);
    let leftover = vehicle.last_lost_vehicle.expect("lost_vehicle_check ran");
    assert_eq!(leftover.soundalarm_counter, 1);
    assert!(!leftover.vehicle_lost);
}

fn landed_blocked_takeoff() -> TakeoffCheckInputs {
    TakeoffCheckInputs {
        now_ms: 3_000,
        spoolup_block: true,
        land_complete: true,
        motor_check_passed: false,
        system_load_available: true,
        avg_load: 96.0,
        peak_load: 80.0,
        warning_ms: 500,
    }
}

#[test]
fn takeoff_check_unblocked_resets_warning_timers() {
    let leftover = takeoff_check(TakeoffCheckInputs {
        now_ms: 4_000,
        spoolup_block: false,
        land_complete: true,
        motor_check_passed: false,
        system_load_available: true,
        avg_load: 99.0,
        peak_load: 100.0,
        warning_ms: 0,
    });
    assert_eq!(leftover.path, TakeoffCheckPath::Unblocked);
    assert!(!leftover.spoolup_block);
    assert_eq!(leftover.warning_ms, 4_000);
    assert!(!leftover.gcs_cpu_overload);
}

#[test]
fn takeoff_check_airborne_clears_block_without_waiting_for_checks() {
    let leftover = takeoff_check(TakeoffCheckInputs {
        now_ms: 4_000,
        spoolup_block: true,
        land_complete: false,
        motor_check_passed: false,
        system_load_available: true,
        avg_load: 99.0,
        peak_load: 100.0,
        warning_ms: 100,
    });
    assert_eq!(leftover.path, TakeoffCheckPath::NotLanded);
    assert!(!leftover.spoolup_block);
    assert_eq!(leftover.warning_ms, 100);
    assert!(!leftover.gcs_cpu_overload);
}

#[test]
fn takeoff_check_clears_block_when_motor_and_load_pass() {
    let leftover = takeoff_check(TakeoffCheckInputs {
        now_ms: 4_000,
        spoolup_block: true,
        land_complete: true,
        motor_check_passed: true,
        system_load_available: true,
        avg_load: 95.0,
        peak_load: 99.5,
        warning_ms: 100,
    });
    assert_eq!(leftover.path, TakeoffCheckPath::ChecksPassed);
    assert!(!leftover.spoolup_block);
    assert!(takeoff_check_load_adequate(
        true,
        TAKEOFF_CHECK_AVG_LOAD_MAX,
        TAKEOFF_CHECK_PEAK_LOAD_MAX
    ));
}

#[test]
fn takeoff_check_missing_load_reading_is_adequate() {
    assert!(takeoff_check_load_adequate(false, 100.0, 100.0));
    let leftover = takeoff_check(TakeoffCheckInputs {
        now_ms: 4_000,
        spoolup_block: true,
        land_complete: true,
        motor_check_passed: true,
        system_load_available: false,
        avg_load: 100.0,
        peak_load: 100.0,
        warning_ms: 100,
    });
    assert_eq!(leftover.path, TakeoffCheckPath::ChecksPassed);
}

#[test]
fn takeoff_check_warns_cpu_overload_after_two_seconds_strict() {
    let leftover = takeoff_check(landed_blocked_takeoff());
    assert_eq!(leftover.path, TakeoffCheckPath::Blocked);
    assert!(leftover.spoolup_block);
    assert!(leftover.gcs_cpu_overload);
    assert_eq!(leftover.warning_ms, 3_000);

    let mut inputs = landed_blocked_takeoff();
    inputs.now_ms = inputs.warning_ms + TAKEOFF_CHECK_WARNING_MS;
    let leftover = takeoff_check(inputs);
    assert!(leftover.spoolup_block);
    assert!(!leftover.gcs_cpu_overload);
    assert_eq!(leftover.warning_ms, inputs.warning_ms);
}

#[test]
fn takeoff_check_motor_fail_does_not_send_cpu_warning() {
    let leftover = takeoff_check(TakeoffCheckInputs {
        now_ms: 3_000,
        spoolup_block: true,
        land_complete: true,
        motor_check_passed: false,
        system_load_available: true,
        avg_load: 10.0,
        peak_load: 10.0,
        warning_ms: 500,
    });
    assert_eq!(leftover.path, TakeoffCheckPath::Blocked);
    assert!(leftover.spoolup_block);
    assert!(!leftover.gcs_cpu_overload);
}

#[test]
fn takeoff_check_is_the_fifty_hz_row() {
    let task = SCHEDULER_TASKS
        .iter()
        .find(|row| row.name == "takeoff_check")
        .expect("takeoff_check");
    assert!(task.rate_hz == TAKEOFF_CHECK_RATE_HZ);
    assert_eq!(task.max_time_micros, TAKEOFF_CHECK_MAX_TIME_MICROS);
    assert_eq!(task.priority, TAKEOFF_CHECK_PRIORITY);
    assert!(task.gate.is_none());
}

#[test]
fn scheduler_runs_takeoff_check_every_eighth_tick() {
    use ap_copter::vehicle_loop::copter_takeoff_check_task;
    let tasks = [copter_takeoff_check_task()];
    let mut last = [0u16; 1];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.takeoff_check = landed_blocked_takeoff();
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    for _ in 0..7 {
        let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
        assert_eq!(stats.tasks_run, 0);
        assert_eq!(vehicle.ticks.takeoff_check, 0);
    }

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);
    assert_eq!(stats.tasks_run, 1);
    assert_eq!(vehicle.ticks.takeoff_check, 1);
    let leftover = vehicle.last_takeoff_check.expect("takeoff_check ran");
    assert_eq!(leftover.path, TakeoffCheckPath::Blocked);
    assert!(leftover.gcs_cpu_overload);
}

fn armed_manual() -> UpdateAutoArmedInputs {
    UpdateAutoArmedInputs {
        auto_armed: true,
        motors_armed: true,
        has_manual_throttle: true,
        throttle_zero: false,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: true,
        throw_mode: false,
    }
}

#[test]
fn update_auto_armed_disarm_clears_immediately() {
    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: true,
        motors_armed: false,
        has_manual_throttle: true,
        throttle_zero: true,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: false,
        throw_mode: false,
    });
    assert!(!leftover.auto_armed);
}

#[test]
fn update_auto_armed_manual_zero_needs_valid_rc() {
    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: true,
        motors_armed: true,
        has_manual_throttle: true,
        throttle_zero: true,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: true,
        throw_mode: false,
    });
    assert!(!leftover.auto_armed);

    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: true,
        motors_armed: true,
        has_manual_throttle: true,
        throttle_zero: true,
        has_valid_input: false,
        using_interlock: false,
        spool_throttle_unlimited: true,
        throw_mode: false,
    });
    assert!(leftover.auto_armed);
}

#[test]
fn update_auto_armed_interlock_needs_unlimited_spool() {
    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: false,
        has_valid_input: true,
        using_interlock: true,
        spool_throttle_unlimited: false,
        throw_mode: false,
    });
    assert!(!leftover.auto_armed);

    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: false,
        has_valid_input: true,
        using_interlock: true,
        spool_throttle_unlimited: true,
        throw_mode: false,
    });
    assert!(leftover.auto_armed);
}

#[test]
fn update_auto_armed_throw_only_on_non_interlock_path() {
    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: true,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: false,
        throw_mode: true,
    });
    assert!(leftover.auto_armed);

    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: true,
        has_valid_input: true,
        using_interlock: true,
        spool_throttle_unlimited: false,
        throw_mode: true,
    });
    assert!(!leftover.auto_armed);
}

#[test]
fn update_auto_armed_non_interlock_needs_throttle() {
    let leftover = update_auto_armed(armed_manual());
    assert!(leftover.auto_armed);

    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: true,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: false,
        throw_mode: false,
    });
    assert!(!leftover.auto_armed);

    let leftover = update_auto_armed(UpdateAutoArmedInputs {
        auto_armed: false,
        motors_armed: true,
        has_manual_throttle: false,
        throttle_zero: false,
        has_valid_input: true,
        using_interlock: false,
        spool_throttle_unlimited: false,
        throw_mode: false,
    });
    assert!(leftover.auto_armed);
}

fn typical_init_ardupilot() -> InitArdupilotInputs {
    InitArdupilotInputs {
        initial_mode: MODE_STABILIZE,
        initial_mode_ok: true,
    }
}

#[test]
fn init_ardupilot_uses_initial_mode_when_available() {
    let leftover = init_ardupilot(InitArdupilotInputs {
        initial_mode: MODE_THROW,
        initial_mode_ok: true,
    });
    assert_eq!(leftover.path, InitArdupilotPath::InitialMode);
    assert_eq!(leftover.mode, MODE_THROW);
    assert_eq!(leftover.mode_reason, MODE_REASON_INITIALISED);
    assert!(leftover.allocate_motors);
    assert!(leftover.startup_ins_ground);
    assert!(leftover.initialised);
    assert!(leftover.initialised_params);
}

#[test]
fn init_ardupilot_falls_back_to_stabilize_when_initial_unavailable() {
    let leftover = init_ardupilot(InitArdupilotInputs {
        initial_mode: MODE_THROW,
        initial_mode_ok: false,
    });
    assert_eq!(leftover.path, InitArdupilotPath::StabilizeFallback);
    assert_eq!(leftover.mode, MODE_STABILIZE);
    assert_eq!(leftover.mode_reason, MODE_REASON_UNAVAILABLE);
    assert_ne!(leftover.mode_reason, MODE_REASON_INITIALISED);
}

#[test]
fn init_ardupilot_sets_landed_before_failsafe_and_initialised_last() {
    let leftover = init_ardupilot(typical_init_ardupilot());
    assert!(leftover.land_complete);
    assert!(leftover.land_complete_maybe);
    assert!(leftover.failsafe_enable);
    assert!(leftover.motors_output_min);
    assert!(leftover.register_timer_failsafe);
    assert_eq!(leftover.failsafe_period_us, INIT_ARDUPILOT_FAILSAFE_US);
    assert!(leftover.initialised);
}

#[test]
fn init_ardupilot_heli_and_optional_inits_compiled_out() {
    let leftover = init_ardupilot(typical_init_ardupilot());
    assert!(!leftover.heli_init);
    assert!(!leftover.winch_init);
    assert!(!leftover.custom_control_init);
    assert!(leftover.surface_tracking_init);
    assert!(leftover.mission_init);
    assert!(leftover.smart_rtl_init);
}

#[test]
fn startup_ins_ground_sets_copter_class_then_resets_ahrs() {
    let leftover = startup_ins_ground(StartupInsGroundInputs {
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
    });
    assert!(leftover.ahrs_init);
    assert_eq!(leftover.vehicle_class, VEHICLE_CLASS_COPTER);
    assert!(leftover.ins_init);
    assert_eq!(leftover.ins_loop_rate_hz, COPTER_LOOP_RATE_HZ);
    assert!(leftover.ahrs_reset);
}

#[test]
fn startup_ins_ground_hands_scheduler_loop_rate_to_ins() {
    let leftover = startup_ins_ground(StartupInsGroundInputs { loop_rate_hz: 200 });
    assert_eq!(leftover.ins_loop_rate_hz, 200);
    assert_eq!(leftover.vehicle_class, VEHICLE_CLASS_COPTER);
}


fn typical_allocate_motors() -> AllocateMotorsInputs {
    AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_QUAD,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: false,
    }
}

#[test]
fn allocate_motors_frame_class_enum_matches_upstream() {
    assert_eq!(MOTOR_FRAME_UNDEFINED, 0);
    assert_eq!(MOTOR_FRAME_QUAD, 1);
    assert_eq!(MOTOR_FRAME_HEXA, 2);
    assert_eq!(MOTOR_FRAME_OCTA, 3);
    assert_eq!(MOTOR_FRAME_OCTAQUAD, 4);
    assert_eq!(MOTOR_FRAME_Y6, 5);
    assert_eq!(MOTOR_FRAME_HELI, 6);
    assert_eq!(MOTOR_FRAME_TRI, 7);
    assert_eq!(MOTOR_FRAME_SINGLE, 8);
    assert_eq!(MOTOR_FRAME_COAX, 9);
    assert_eq!(MOTOR_FRAME_TAILSITTER, 10);
    assert_eq!(MOTOR_FRAME_HELI_DUAL, 11);
    assert_eq!(MOTOR_FRAME_DODECAHEXA, 12);
    assert_eq!(MOTOR_FRAME_HELI_QUAD, 13);
    assert_eq!(MOTOR_FRAME_DECA, 14);
    assert_eq!(MOTOR_FRAME_SCRIPTING_MATRIX, 15);
    assert_eq!(MOTOR_FRAME_6DOF_SCRIPTING, 16);
    assert_eq!(MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX, 17);
    assert_eq!(AP_PARAM_FRAME_TRICOPTER, 1 << 4);
    assert_eq!(ALLOCATE_MOTORS_Y6_RATE_RP_KP, 0.1);
    assert_eq!(ALLOCATE_MOTORS_Y6_RATE_RP_KD, 0.006);
    assert_eq!(ALLOCATE_MOTORS_Y6_RATE_YAW_KP, 0.15);
    assert_eq!(ALLOCATE_MOTORS_Y6_RATE_YAW_KI, 0.015);
    assert_eq!(ALLOCATE_MOTORS_TRI_YAW_FILT_D_HZ, 100.0);
    assert_eq!(ALLOCATE_MOTORS_BRUSHED_RC_SPEED_HZ, 16_000);
}

#[test]
fn allocate_motors_quad_builds_matrix_and_multi_attitude() {
    let leftover = allocate_motors(typical_allocate_motors());
    assert_eq!(leftover.motors_kind, AllocatedMotorsKind::Matrix);
    assert!(leftover.motors_allocated);
    assert!(!leftover.allocation_error);
    assert_eq!(leftover.motors_loop_rate_hz, COPTER_LOOP_RATE_HZ);
    assert_eq!(leftover.frame_type_flags, 0);
    assert_eq!(leftover.attitude_kind, AllocatedAttitudeKind::Multi);
    assert!(leftover.load_motors_eeprom);
    assert!(leftover.ahrs_view);
    assert!(leftover.pos_control);
    assert!(leftover.wp_nav);
    assert!(leftover.loiter_nav);
    assert!(leftover.circle_nav);
    assert!(leftover.reload_defaults_file);
    assert!(!leftover.y6_rate_defaults);
    assert!(!leftover.tri_yaw_filt_d);
    assert!(!leftover.brushed_rc_speed);
    assert!(leftover.convert_pid_parameters);
    assert!(leftover.invalidate_count);
}

#[test]
fn allocate_motors_matrix_classes_share_motors_matrix() {
    for class in [
        MOTOR_FRAME_QUAD,
        MOTOR_FRAME_HEXA,
        MOTOR_FRAME_Y6,
        MOTOR_FRAME_OCTA,
        MOTOR_FRAME_OCTAQUAD,
        MOTOR_FRAME_DODECAHEXA,
        MOTOR_FRAME_DECA,
        MOTOR_FRAME_SCRIPTING_MATRIX,
    ] {
        let leftover = allocate_motors(AllocateMotorsInputs {
            frame_class: class,
            loop_rate_hz: COPTER_LOOP_RATE_HZ,
            brushed_pwm: false,
        });
        assert_eq!(leftover.motors_kind, AllocatedMotorsKind::Matrix);
        assert_eq!(leftover.frame_type_flags, 0);
        assert_eq!(leftover.y6_rate_defaults, class == MOTOR_FRAME_Y6);
    }
}

#[test]
fn allocate_motors_default_and_heli_class_fall_through_to_matrix() {
    for class in [
        MOTOR_FRAME_UNDEFINED,
        MOTOR_FRAME_HELI,
        MOTOR_FRAME_HELI_DUAL,
        MOTOR_FRAME_HELI_QUAD,
        99,
    ] {
        let leftover = allocate_motors(AllocateMotorsInputs {
            frame_class: class,
            loop_rate_hz: COPTER_LOOP_RATE_HZ,
            brushed_pwm: false,
        });
        assert_eq!(leftover.motors_kind, AllocatedMotorsKind::Matrix);
        assert_eq!(leftover.frame_type_flags, 0);
        assert!(!leftover.heli_motors_param_conversions);
    }
}

#[test]
fn allocate_motors_tri_sets_tricopter_frame_flag() {
    let leftover = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_TRI,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: false,
    });
    assert_eq!(leftover.motors_kind, AllocatedMotorsKind::Tri);
    assert_eq!(leftover.frame_type_flags, AP_PARAM_FRAME_TRICOPTER);
    assert!(leftover.tri_yaw_filt_d);
    assert!(!leftover.y6_rate_defaults);
    assert_eq!(leftover.attitude_kind, AllocatedAttitudeKind::Multi);
}

#[test]
fn allocate_motors_single_coax_tailsitter_pick_dedicated_classes() {
    let single = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_SINGLE,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: false,
    });
    assert_eq!(single.motors_kind, AllocatedMotorsKind::Single);
    assert_eq!(single.frame_type_flags, 0);

    let coax = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_COAX,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: false,
    });
    assert_eq!(coax.motors_kind, AllocatedMotorsKind::Coax);

    let tailsitter = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_TAILSITTER,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: false,
    });
    assert_eq!(tailsitter.motors_kind, AllocatedMotorsKind::Tailsitter);
    assert_eq!(tailsitter.attitude_kind, AllocatedAttitudeKind::Multi);
}

#[test]
fn allocate_motors_scripting_classes_fail_without_scripting() {
    for class in [
        MOTOR_FRAME_6DOF_SCRIPTING,
        MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX,
    ] {
        let leftover = allocate_motors(AllocateMotorsInputs {
            frame_class: class,
            loop_rate_hz: COPTER_LOOP_RATE_HZ,
            brushed_pwm: true,
        });
        assert_eq!(leftover.motors_kind, AllocatedMotorsKind::None);
        assert!(!leftover.motors_allocated);
        assert!(leftover.allocation_error);
        assert_eq!(leftover.motors_loop_rate_hz, 0);
        assert_eq!(leftover.attitude_kind, AllocatedAttitudeKind::None);
        assert!(!leftover.pos_control);
        assert!(!leftover.wp_nav);
        assert!(!leftover.convert_pid_parameters);
        assert!(!leftover.brushed_rc_speed);
        assert!(!leftover.invalidate_count);
    }
}

#[test]
fn allocate_motors_stock_follow_ons_skip_heli_oa_and_proximity() {
    let leftover = allocate_motors(typical_allocate_motors());
    assert!(!leftover.wp_nav_oa);
    assert!(!leftover.heli_motors_param_conversions);
    assert!(!leftover.convert_prx_parameters);
    assert!(leftover.convert_attitude_parameters);
    assert!(leftover.convert_pos_parameters);
    assert!(leftover.convert_wp_nav_parameters);
    assert!(leftover.convert_loiter_parameters);
    assert!(leftover.convert_circle_parameters);
    assert!(leftover.load_attitude_eeprom);
}

#[test]
fn allocate_motors_brushed_pwm_defaults_rc_speed() {
    let leftover = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_QUAD,
        loop_rate_hz: COPTER_LOOP_RATE_HZ,
        brushed_pwm: true,
    });
    assert!(leftover.brushed_rc_speed);
    assert_eq!(ALLOCATE_MOTORS_BRUSHED_RC_SPEED_HZ, 16_000);
}

#[test]
fn allocate_motors_hands_scheduler_loop_rate_to_motors() {
    let leftover = allocate_motors(AllocateMotorsInputs {
        frame_class: MOTOR_FRAME_HEXA,
        loop_rate_hz: 200,
        brushed_pwm: false,
    });
    assert_eq!(leftover.motors_loop_rate_hz, 200);
    assert_eq!(leftover.motors_kind, AllocatedMotorsKind::Matrix);
}
