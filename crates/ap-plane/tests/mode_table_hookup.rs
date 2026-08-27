//! Mode table dispatch into stabilize paths.

use ap_plane::main_loop::{PlaneMainLoop, StabilizeDispatch};
use ap_plane::mode_run::StickMixing;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::mode_table_hookup::dispatch_stabilize_from_mode;

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
fn invalid_mode_number_yields_no_stabilize_paths() {
    let dispatch = dispatch_stabilize_from_mode(9, Some(StickMixing::Fbw), &BuildFeatures::default());
    assert_eq!(dispatch, StabilizeDispatch::default());
}

#[test]
fn without_adsb_avoidance_number_dispatches_guided_stabilize() {
    let features = BuildFeatures {
        adsb: false,
        quadplane: true,
        qautotune: true,
        soaring: true,
        autoland: true,
    };
    let dispatch = dispatch_stabilize_from_mode(14, Some(StickMixing::Fbw), &features);
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
fn scheduler_tick_dispatches_mode_table_into_stabilize() {
    use ap_hal::time::{Clock, Micros, Millis};
    use ap_plane::main_loop::{plane_fast_tasks, run_scheduler_tick};
    use ap_scheduler::scheduler::Scheduler;
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
            self.us.get() as u64 / 1000
        }
        fn micros64(&self) -> u64 {
            self.us.get() as u64
        }
    }

    let tasks = plane_fast_tasks();
    let mut last = [0u16; 4];
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::FlyByWireA.as_number();
    vehicle.stick_mixing = Some(StickMixing::None);
    let mut scheduler = Scheduler::new(&tasks, &[], &mut last, 400);
    let clock = StepClock::new();

    run_scheduler_tick(&mut vehicle, &mut scheduler, &clock, 2500);

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
