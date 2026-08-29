//! Vehicle-loop scheduler leftover, upstream `ArduCopter/Copter.cpp`.

use ap_copter::radio::ReadRadioLeftover;
use ap_copter::vehicle_loop::{
    always_on_tasks, copter_first_fast_tasks, copter_first_scheduled_tasks, copter_rc_loop_task,
    first_scheduled_task, get_scheduler_tasks, motors_output, motors_output_main, rc_loop,
    read_ahrs, read_inertia, read_mode_switch, run_scheduler_tick, throttle_loop,
    CopterVehicleLoop, InterlockEdge, ModeSwitchReadInputs, ModeSwitchReadLeftover,
    MotorsOutputDrive, MotorsOutputMainLeftover, MotorsOutputPush, TaskKind, ARMING_DELAY_MS,
    COPTER_LOOP_RATE_HZ, FAST_TASK_PRI0, MASK_LOG_PM, MODE_THROW, RC_LOOP_MAX_TIME_MICROS,
    RC_LOOP_PRIORITY, RC_LOOP_RATE_HZ, REMAINING, SCHEDULER_TASKS, THROTTLE_LOOP_MAX_TIME_MICROS,
    THROTTLE_LOOP_PRIORITY, THROTTLE_LOOP_RATE_HZ,
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
