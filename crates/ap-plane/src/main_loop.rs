//! ArduPlane main vehicle loop skeleton, upstream the four fast scheduler
//! tasks in `ArduPlane/Plane.cpp` and `Mode::run` in `ArduPlane/mode.cpp`.
//!
//! `ap-scheduler` owns tick ordering; this module is where the vehicle wires
//! those tasks to mode dispatch and the attitude/servo paths that follow.

use ap_ahrs::{YawCompassSample, YawDriftContext, YawGpsSample};
use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_scheduler::scheduler::{LOOP_RATE, RunStats, Scheduler, Task};

use crate::ahrs_hookup::{yaw_update_inputs, AhrsAttitude, AhrsFeed};
use crate::landing_hookup::ServoOutputState;
use crate::mode::ModeState;
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
    /// Optional compass sample for yaw drift correction.
    pub compass: Option<YawCompassSample>,
    /// Optional GPS sample for yaw drift fallback.
    pub gps_yaw: Option<YawGpsSample>,
    /// Vehicle context for compass vs GPS yaw selection.
    pub yaw_ctx: YawDriftContext,
    /// Roll/pitch/yaw controllers, upstream `rollController` et al.
    pub controllers: StabilizeControllers,
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
            stick_mixing: Some(StickMixing::Fbw),
            features: BuildFeatures::default(),
            ticks: FastTaskTicks::default(),
            last_stabilize: StabilizeDispatch::default(),
            last_stabilize_run: StabilizeRun::default(),
            ahrs: AhrsFeed::default(),
            ins: InertialSensorFrontend::default(),
            loop_timing: LoopTiming::new(1.0 / f32::from(LOOP_RATE)),
            attitude: AhrsAttitude::default(),
            compass: None,
            gps_yaw: None,
            yaw_ctx: YawDriftContext::default(),
            controllers: StabilizeControllers::default(),
            nav_commands: NavCommandInputs::default(),
            rc_sticks: RcStickInputs::default(),
            speed_scaler_inputs: SpeedScalerInputs::default(),
            surface_speed_scaler: 1.0,
            stabilize_demands: StabilizeDemands::default(),
            stabilize_ctx: StabilizeContext::default(),
            stabilize_servos: StabilizeServoDemands::default(),
            servos: ServoOutputState::default(),
        }
    }
}

impl PlaneMainLoop {
    /// Upstream `Plane::ahrs_update`. Runs INS→DCM and publishes attitude sensors.
    pub fn ahrs_update(&mut self) {
        self.ticks.ahrs_update += 1;
        let yaw = yaw_update_inputs(self.compass, self.gps_yaw, self.yaw_ctx);
        let (_health, attitude) = self.ahrs.update_from_ins(
            &self.ins,
            &self.loop_timing,
            yaw,
        );
        self.attitude = attitude;
    }

    /// Upstream `Plane::update_control_mode`. Dispatches to the active mode.
    pub fn update_control_mode(&mut self) {
        self.ticks.update_control_mode += 1;
        self.last_stabilize = mode_run_dispatch(
            self.mode.control_mode,
            self.stick_mixing,
            &self.features,
        );
    }

    /// Upstream `Plane::stabilize`. Calls roll/pitch/yaw controllers when the
    /// active mode selected them on the previous `update_control_mode`.
    pub fn stabilize(&mut self) {
        self.ticks.stabilize += 1;
        prepare_stabilize_path(
            &mut self.stabilize_demands,
            &mut self.stabilize_ctx,
            &self.nav_commands,
            &self.speed_scaler_inputs,
            self.last_stabilize,
            &self.rc_sticks,
            self.stick_mixing,
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
    }

    /// Upstream `Plane::set_servos`. Publishes scaled/PWM demands from stabilize.
    pub fn set_servos(&mut self) {
        self.ticks.set_servos += 1;
        apply_stabilize_to_servos(&self.stabilize_servos, &mut self.servos);
    }
}

/// The stabilization dispatch for one mode iteration, upstream `Mode::run`'s
/// body before the mode-specific logic.
#[must_use]
pub fn mode_run_dispatch(
    control_mode: u8,
    stick_mixing: Option<StickMixing>,
    features: &BuildFeatures,
) -> StabilizeDispatch {
    let Some(mode) = ModeNumber::from_number(control_mode, features) else {
        return StabilizeDispatch::default();
    };

    let fbw_stick_mixing = applies_fbw_stick_mixing(stick_mixing);

    match mode {
        ModeNumber::Manual | ModeNumber::Training => StabilizeDispatch {
            roll: false,
            pitch: false,
            yaw: false,
            fbw_stick_mixing: false,
        },
        ModeNumber::Acro | ModeNumber::QAcro => StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing: false,
        },
        ModeNumber::Initialising | ModeNumber::Circle => StabilizeDispatch::default(),
        _ => StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing,
        },
    }
}

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
