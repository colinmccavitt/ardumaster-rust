//! Vehicle-loop scheduler leftover, upstream `ArduCopter/Copter.cpp`.

use ap_copter::radio::ReadRadioLeftover;
use ap_copter::vehicle_loop::{
    always_on_tasks, ap_value, copter_first_fast_tasks, copter_first_scheduled_tasks,
    copter_logging_tasks, copter_next_fast_tasks, copter_next_scheduled_tasks,
    copter_periodic_loop_tasks, copter_rc_loop_task, first_scheduled_task, get_scheduler_tasks,
    loop_rate_logging, motors_output, motors_output_main, one_hz_loop, rc_loop, read_ahrs,
    read_inertia, read_mode_switch, run_scheduler_tick, should_log, ten_hz_logging_loop,
    three_hz_loop, throttle_loop, twentyfive_hz_logging, update_batt_compass, update_flight_mode,
    update_land_and_crash_detectors, ApState, CopterVehicleLoop, EkfResetMethod, InterlockEdge,
    LoopRateLoggingInputs, ModeSwitchReadInputs, ModeSwitchReadLeftover, MotorsOutputDrive,
    MotorsOutputMainLeftover, MotorsOutputPush, OneHzLoopInputs, TaskKind, TenHzLoggingInputs,
    TwentyfiveHzLoggingInputs, UpdateBattCompassInputs, UpdateFlightModeInputs, ARMING_DELAY_MS,
    COPTER_LOOP_RATE_HZ, DEFAULT_LOG_BITMASK, FAST_TASK_PRI0, LOOP_RATE_LOGGING_MAX_TIME_MICROS,
    LOOP_RATE_LOGGING_PRIORITY, MASK_LOG_ANY, MASK_LOG_ATTITUDE_FAST, MASK_LOG_ATTITUDE_MED,
    MASK_LOG_IMU, MASK_LOG_IMU_FAST, MASK_LOG_MOTBATT, MASK_LOG_NTUN, MASK_LOG_PM, MODE_THROW,
    ONE_HZ_LOOP_MAX_TIME_MICROS, ONE_HZ_LOOP_PRIORITY, ONE_HZ_LOOP_RATE_HZ,
    RC_LOOP_MAX_TIME_MICROS, RC_LOOP_PRIORITY, RC_LOOP_RATE_HZ, REMAINING, SCHEDULER_TASKS,
    TEN_HZ_LOGGING_MAX_TIME_MICROS, TEN_HZ_LOGGING_PRIORITY, TEN_HZ_LOGGING_RATE_HZ,
    THREE_HZ_LOOP_MAX_TIME_MICROS, THREE_HZ_LOOP_PRIORITY, THREE_HZ_LOOP_RATE_HZ,
    THROTTLE_LOOP_MAX_TIME_MICROS, THROTTLE_LOOP_PRIORITY, THROTTLE_LOOP_RATE_HZ,
    TWENTYFIVE_HZ_LOGGING_MAX_TIME_MICROS, TWENTYFIVE_HZ_LOGGING_PRIORITY,
    TWENTYFIVE_HZ_LOGGING_RATE_HZ, UPDATE_BATT_COMPASS_MAX_TIME_MICROS,
    UPDATE_BATT_COMPASS_PRIORITY, UPDATE_BATT_COMPASS_RATE_HZ,
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
    assert!(REMAINING.contains(&"Copter::update_auto_armed"));
    assert!(REMAINING.contains(&"Copter::init_ardupilot"));
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
    assert!(REMAINING.contains(&"Copter::check_ekf_reset"));
    assert!(REMAINING.contains(&"Copter::update_home_from_EKF"));
    assert!(REMAINING.contains(&"Copter::init_simple_bearing"));
    assert!(REMAINING.contains(&"Copter::update_altitude"));
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
    let mut last = [0u16; 2];
    let mut vehicle = CopterVehicleLoop::typical();
    vehicle.flight_mode.land_complete = true;
    vehicle.flight_mode.move_vehicle_on_ekf_reset = true;
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, COPTER_LOOP_RATE_HZ);
    let clock = StepClock::new();

    let stats = run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2_500);

    assert_eq!(stats.tasks_run, 2);
    assert_eq!(vehicle.ticks.update_flight_mode, 1);
    assert_eq!(vehicle.ticks.update_land_and_crash_detectors, 1);
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
