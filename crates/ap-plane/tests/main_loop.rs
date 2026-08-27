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
