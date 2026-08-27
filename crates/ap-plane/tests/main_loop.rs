//! Main vehicle loop scheduler wiring and mode dispatch.

use ap_hal::time::{Clock, Micros, Millis};
use ap_plane::main_loop::{
    mode_run_dispatch, plane_fast_tasks, run_scheduler_tick, PlaneMainLoop, StabilizeDispatch,
};
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
    let dispatch = mode_run_dispatch(
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
    let dispatch = mode_run_dispatch(
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
}
