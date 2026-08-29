//! Vehicle-loop scheduler leftover, upstream `ArduCopter/Copter.cpp`.

use ap_copter::radio::ReadRadioLeftover;
use ap_copter::vehicle_loop::{
    always_on_tasks, copter_rc_loop_task, first_scheduled_task, get_scheduler_tasks, rc_loop,
    read_mode_switch, run_scheduler_tick, CopterVehicleLoop, ModeSwitchReadInputs,
    ModeSwitchReadLeftover, TaskKind, COPTER_LOOP_RATE_HZ, FAST_TASK_PRI0, MASK_LOG_PM,
    RC_LOOP_MAX_TIME_MICROS, RC_LOOP_PRIORITY, RC_LOOP_RATE_HZ, REMAINING, SCHEDULER_TASKS,
};
use ap_hal::time::{Clock, Micros, Millis};
use ap_scheduler::scheduler::{LOOP_RATE, Scheduler};
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
    assert!(task.rate_hz == 50.0);
    assert_eq!(task.max_time_micros, 75);
    assert_eq!(task.priority, 6);
}

#[test]
fn remaining_leftovers_keep_later_callbacks() {
    assert!(REMAINING.contains(&"Copter::throttle_loop"));
    assert!(REMAINING.contains(&"Copter::read_AHRS"));
    assert!(REMAINING.contains(&"Copter::init_ardupilot"));
    assert!(!REMAINING.iter().any(|name| *name == "Copter::rc_loop"));
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
