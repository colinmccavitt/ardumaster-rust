//! Vehicle-loop leftover, upstream `ArduCopter/Copter.cpp` scheduler table.
//!
//! Tracked as **COP-012**. The table is the leftover: every `FAST_TASK` and
//! `SCHED_TASK` row with its rate, budget, priority, and compile gate.
//! [`rc_loop`] is the first scheduled callback and [`throttle_loop`] is the
//! next; [`update_batt_compass`] is the next always-on Copter-owned scheduled
//! leftover after GPS (which lives on `ap-gps`). The first Copter-owned
//! fast-loop leftovers are [`run_rate_controller_main`],
//! [`motors_output_main`], [`read_ahrs`], and [`read_inertia`] — INS `update`
//! lives on `ap-ins`. [`update_flight_mode`] and
//! [`update_land_and_crash_detectors`] are the next Copter-owned FAST_TASK
//! leftovers after `check_ekf_reset` / `update_home_from_EKF`, which stay
//! later. [`loop_rate_logging`], [`ten_hz_logging_loop`], and
//! [`twentyfive_hz_logging`] are the `HAL_LOGGING_ENABLED` scheduled
//! leftovers; [`three_hz_loop`], [`ap_value`], and [`one_hz_loop`] sit
//! next to them in `Copter.cpp`. Simple-mode, `update_altitude`, and
//! `system.cpp` stay later leftovers.

use ap_hal::time::Clock;
use ap_math::location::{AltContext, AltFrame, Location};
use ap_scheduler::scheduler::{RunStats, Scheduler, Task, LOOP_RATE};

use crate::attitude::RateControllerMainLeftover;
use crate::aux::AirMode;
use crate::ground::ekf_reset_method;
use crate::radio::{
    read_radio, ReadRadioInputs, ReadRadioLeftover, ThrottleFailsafeInputs, ThrottleZeroInputs,
    FS_THR_VALUE_COPTER_DEFAULT,
};

pub use crate::attitude::run_rate_controller_main;
pub use crate::ground::EkfResetMethod;

/// `AP_Scheduler::FAST_TASK_PRI0` — every `FAST_TASK_CLASS` row uses this.
pub const FAST_TASK_PRI0: u8 = 0;

/// Copter default scheduler loop rate, Hz.
///
/// Upstream `SCHED_LOOP_RATE` / `g.scheduler_loop_rate` default for
/// multicopter. Plane's four fast tasks also sit on 400 Hz.
pub const COPTER_LOOP_RATE_HZ: u16 = 400;

/// `MASK_LOG_PM` — `Copter::get_scheduler_tasks` writes this as `log_bit`.
pub const MASK_LOG_PM: u32 = 1 << 3;

/// `rc_loop` rate, Hz. Upstream `SCHED_TASK(rc_loop, 250, 130, 3)`.
pub const RC_LOOP_RATE_HZ: f32 = 250.0;

/// `rc_loop` expected budget, microseconds.
pub const RC_LOOP_MAX_TIME_MICROS: u16 = 130;

/// `rc_loop` scheduler priority (lower is higher priority).
pub const RC_LOOP_PRIORITY: u8 = 3;

/// `throttle_loop` rate, Hz. Upstream `SCHED_TASK(throttle_loop, 50, 75, 6)`.
pub const THROTTLE_LOOP_RATE_HZ: f32 = 50.0;

/// `throttle_loop` expected budget, microseconds.
pub const THROTTLE_LOOP_MAX_TIME_MICROS: u16 = 75;

/// `throttle_loop` scheduler priority (lower is higher priority).
pub const THROTTLE_LOOP_PRIORITY: u8 = 6;

/// `update_batt_compass` rate, Hz. Upstream `SCHED_TASK(update_batt_compass, 10, 120, 15)`.
pub const UPDATE_BATT_COMPASS_RATE_HZ: f32 = 10.0;

/// `update_batt_compass` expected budget, microseconds.
pub const UPDATE_BATT_COMPASS_MAX_TIME_MICROS: u16 = 120;

/// `update_batt_compass` scheduler priority (lower is higher priority).
pub const UPDATE_BATT_COMPASS_PRIORITY: u8 = 15;

/// `loop_rate_logging` rate, Hz. Upstream `SCHED_TASK(..., LOOP_RATE, 50, 75)`.
pub const LOOP_RATE_LOGGING_RATE_HZ: f32 = LOOP_RATE;

/// `loop_rate_logging` expected budget, microseconds.
pub const LOOP_RATE_LOGGING_MAX_TIME_MICROS: u16 = 50;

/// `loop_rate_logging` scheduler priority (lower is higher priority).
pub const LOOP_RATE_LOGGING_PRIORITY: u8 = 75;

/// `ten_hz_logging_loop` rate, Hz. Upstream `SCHED_TASK(..., 10, 350, 114)`.
pub const TEN_HZ_LOGGING_RATE_HZ: f32 = 10.0;

/// `ten_hz_logging_loop` expected budget, microseconds.
pub const TEN_HZ_LOGGING_MAX_TIME_MICROS: u16 = 350;

/// `ten_hz_logging_loop` scheduler priority (lower is higher priority).
pub const TEN_HZ_LOGGING_PRIORITY: u8 = 114;

/// `twentyfive_hz_logging` rate, Hz. Upstream `SCHED_TASK(..., 25, 110, 117)`.
pub const TWENTYFIVE_HZ_LOGGING_RATE_HZ: f32 = 25.0;

/// `twentyfive_hz_logging` expected budget, microseconds.
pub const TWENTYFIVE_HZ_LOGGING_MAX_TIME_MICROS: u16 = 110;

/// `twentyfive_hz_logging` scheduler priority (lower is higher priority).
pub const TWENTYFIVE_HZ_LOGGING_PRIORITY: u8 = 117;

/// `three_hz_loop` rate, Hz. Upstream `SCHED_TASK(three_hz_loop, 3, 75, 57)`.
pub const THREE_HZ_LOOP_RATE_HZ: f32 = 3.0;

/// `three_hz_loop` expected budget, microseconds.
pub const THREE_HZ_LOOP_MAX_TIME_MICROS: u16 = 75;

/// `three_hz_loop` scheduler priority (lower is higher priority).
pub const THREE_HZ_LOOP_PRIORITY: u8 = 57;

/// `one_hz_loop` rate, Hz. Upstream `SCHED_TASK(one_hz_loop, 1, 100, 81)`.
pub const ONE_HZ_LOOP_RATE_HZ: f32 = 1.0;

/// `one_hz_loop` expected budget, microseconds.
pub const ONE_HZ_LOOP_MAX_TIME_MICROS: u16 = 100;

/// `one_hz_loop` scheduler priority (lower is higher priority).
pub const ONE_HZ_LOOP_PRIORITY: u8 = 81;

/// `MASK_LOG_ATTITUDE_FAST` — Copter `defines.h`.
pub const MASK_LOG_ATTITUDE_FAST: u32 = 1 << 0;

/// `MASK_LOG_ATTITUDE_MED` — Copter `defines.h`.
pub const MASK_LOG_ATTITUDE_MED: u32 = 1 << 1;

/// `MASK_LOG_GPS` — Copter `defines.h`.
pub const MASK_LOG_GPS: u32 = 1 << 2;

/// `MASK_LOG_CTUN` — Copter `defines.h`.
pub const MASK_LOG_CTUN: u32 = 1 << 4;

/// `MASK_LOG_NTUN` — Copter `defines.h`.
pub const MASK_LOG_NTUN: u32 = 1 << 5;

/// `MASK_LOG_RCIN` — Copter `defines.h`.
pub const MASK_LOG_RCIN: u32 = 1 << 6;

/// `MASK_LOG_IMU` — Copter `defines.h`.
pub const MASK_LOG_IMU: u32 = 1 << 7;

/// `MASK_LOG_CMD` — Copter `defines.h`.
pub const MASK_LOG_CMD: u32 = 1 << 8;

/// `MASK_LOG_CURRENT` — Copter `defines.h`.
pub const MASK_LOG_CURRENT: u32 = 1 << 9;

/// `MASK_LOG_RCOUT` — Copter `defines.h`.
pub const MASK_LOG_RCOUT: u32 = 1 << 10;

/// `MASK_LOG_OPTFLOW` — Copter `defines.h`.
pub const MASK_LOG_OPTFLOW: u32 = 1 << 11;

/// `MASK_LOG_PID` — Copter `defines.h`.
pub const MASK_LOG_PID: u32 = 1 << 12;

/// `MASK_LOG_COMPASS` — Copter `defines.h`.
pub const MASK_LOG_COMPASS: u32 = 1 << 13;

/// `MASK_LOG_CAMERA` — Copter `defines.h`.
pub const MASK_LOG_CAMERA: u32 = 1 << 15;

/// `MASK_LOG_MOTBATT` — Copter `defines.h`.
pub const MASK_LOG_MOTBATT: u32 = 1 << 17;

/// `MASK_LOG_IMU_FAST` — Copter `defines.h`.
pub const MASK_LOG_IMU_FAST: u32 = 1 << 18;

/// `MASK_LOG_IMU_RAW` — Copter `defines.h`.
pub const MASK_LOG_IMU_RAW: u32 = 1 << 19;

/// `MASK_LOG_FTN_FAST` — Copter `defines.h`.
pub const MASK_LOG_FTN_FAST: u32 = 1 << 21;

/// `MASK_LOG_ANY` — low 16 bits only. `MOTBATT` / `IMU_FAST` sit above it.
pub const MASK_LOG_ANY: u32 = 0xFFFF;

/// Packed `Copter::ap` bool count. Upstream `sizeof(ap)` on a 1-byte `bool`.
pub const AP_STATE_BOOL_COUNT: usize = 27;

/// `DEFAULT_LOG_BITMASK` from Copter `config.h` (stock multicopter).
pub const DEFAULT_LOG_BITMASK: u32 = MASK_LOG_ATTITUDE_MED
    | MASK_LOG_GPS
    | MASK_LOG_PM
    | MASK_LOG_CTUN
    | MASK_LOG_NTUN
    | MASK_LOG_RCIN
    | MASK_LOG_IMU
    | MASK_LOG_CMD
    | MASK_LOG_CURRENT
    | MASK_LOG_RCOUT
    | MASK_LOG_OPTFLOW
    | MASK_LOG_PID
    | MASK_LOG_COMPASS
    | MASK_LOG_CAMERA
    | MASK_LOG_MOTBATT;

/// `ARMING_DELAY_SEC` — motors stay interlocked-off this long after arm.
pub const ARMING_DELAY_SEC: f32 = 2.0;

/// `ARMING_DELAY_SEC * 1.0e3f` as the leftover compares it to `millis()`.
pub const ARMING_DELAY_MS: u32 = 2_000;

/// `Mode::Number::THROW` — clears the arming delay so a toss can spool.
pub const MODE_THROW: u8 = 18;

/// Fast versus rate-limited scheduler row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskKind {
    /// `FAST_TASK` / `FAST_TASK_CLASS` — rate 0, budget 0, priority 0.
    Fast,
    /// `SCHED_TASK` / `SCHED_TASK_CLASS` — rate-limited.
    Scheduled,
}

/// One `Copter::scheduler_tasks[]` row.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SchedulerTaskSpec {
    /// Upstream function / method name as the scheduler would print it.
    pub name: &'static str,
    /// Requested rate in Hz. [`LOOP_RATE`] (0) means every loop.
    pub rate_hz: f32,
    /// Expected worst-case runtime, microseconds.
    pub max_time_micros: u16,
    /// Ordering key, ascending.
    pub priority: u8,
    /// Fast versus scheduled.
    pub kind: TaskKind,
    /// Compile gate, or `None` when the row is always in the table.
    pub gate: Option<&'static str>,
}

const fn fast(name: &'static str, gate: Option<&'static str>) -> SchedulerTaskSpec {
    SchedulerTaskSpec {
        name,
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
        kind: TaskKind::Fast,
        gate,
    }
}

const fn sched(
    name: &'static str,
    rate_hz: f32,
    max_time_micros: u16,
    priority: u8,
    gate: Option<&'static str>,
) -> SchedulerTaskSpec {
    SchedulerTaskSpec {
        name,
        rate_hz,
        max_time_micros,
        priority,
        kind: TaskKind::Scheduled,
        gate,
    }
}

/// `Copter::scheduler_tasks[]` leftover catalog.
///
/// Gated rows stay in the catalog so a later slice can turn a `#if` on
/// without inventing the rate / priority. [`always_on_tasks`] is the
/// `#else`-stripped walk `ARRAY_SIZE` would see on a stock multicopter.
pub const SCHEDULER_TASKS: &[SchedulerTaskSpec] = &[
    fast("AP_InertialSensor::update", None),
    fast("run_rate_controller_main", None),
    fast(
        "run_custom_controller",
        Some("AC_CUSTOMCONTROL_MULTI_ENABLED"),
    ),
    fast("heli_update_autorotation", Some("HELI_FRAME")),
    fast("motors_output_main", None),
    fast("read_AHRS", None),
    fast("update_heli_control_dynamics", Some("HELI_FRAME")),
    fast("read_inertia", None),
    fast("check_ekf_reset", None),
    fast("update_flight_mode", None),
    fast("update_home_from_EKF", None),
    fast("update_land_and_crash_detectors", None),
    fast("update_rangefinder_terrain_offset", None),
    fast("AP_Mount::update_fast", Some("HAL_MOUNT_ENABLED")),
    fast("Log_Video_Stabilisation", Some("HAL_LOGGING_ENABLED")),
    sched(
        "rc_loop",
        RC_LOOP_RATE_HZ,
        RC_LOOP_MAX_TIME_MICROS,
        RC_LOOP_PRIORITY,
        None,
    ),
    sched("throttle_loop", 50.0, 75, 6, None),
    sched("fence_check", 25.0, 100, 7, Some("AP_FENCE_ENABLED")),
    sched("AP_GPS::update", 50.0, 200, 9, None),
    sched(
        "AP_OpticalFlow::update",
        200.0,
        160,
        12,
        Some("AP_OPTICALFLOW_ENABLED"),
    ),
    sched("update_batt_compass", 10.0, 120, 15, None),
    sched("RC_Channels::read_aux_all", 10.0, 50, 18, None),
    sched("ToyMode::update", 10.0, 50, 24, Some("TOY_MODE_ENABLED")),
    sched("auto_disarm_check", 10.0, 50, 27, None),
    sched(
        "RC_Channels_Copter::auto_trim_run",
        10.0,
        75,
        30,
        Some("AP_COPTER_AHRS_AUTO_TRIM_ENABLED"),
    ),
    sched(
        "read_rangefinder",
        20.0,
        100,
        33,
        Some("AP_RANGEFINDER_ENABLED"),
    ),
    sched(
        "AP_Proximity::update",
        200.0,
        50,
        36,
        Some("HAL_PROXIMITY_ENABLED"),
    ),
    sched(
        "AP_Beacon::update",
        400.0,
        50,
        39,
        Some("AP_BEACON_ENABLED"),
    ),
    sched("update_altitude", 10.0, 100, 42, None),
    sched("run_nav_updates", 50.0, 100, 45, None),
    sched("update_throttle_hover", 100.0, 90, 48, None),
    sched(
        "ModeSmartRTL::save_position",
        3.0,
        100,
        51,
        Some("MODE_SMARTRTL_ENABLED"),
    ),
    sched(
        "AC_Sprayer::update",
        3.0,
        90,
        54,
        Some("HAL_SPRAYER_ENABLED"),
    ),
    sched("three_hz_loop", 3.0, 75, 57, None),
    sched(
        "AP_ServoRelayEvents::update_events",
        50.0,
        75,
        60,
        Some("AP_SERVORELAYEVENTS_ENABLED"),
    ),
    sched(
        "update_precland",
        400.0,
        50,
        69,
        Some("AC_PRECLAND_ENABLED"),
    ),
    sched("check_dynamic_flight", 50.0, 75, 72, Some("HELI_FRAME")),
    sched(
        "loop_rate_logging",
        LOOP_RATE,
        50,
        75,
        Some("HAL_LOGGING_ENABLED"),
    ),
    sched("one_hz_loop", 1.0, 100, 81, None),
    sched("ekf_check", 10.0, 75, 84, None),
    sched("check_vibration", 10.0, 50, 87, None),
    sched("gpsglitch_check", 10.0, 50, 90, None),
    sched("takeoff_check", 50.0, 50, 91, None),
    sched(
        "landinggear_update",
        10.0,
        75,
        93,
        Some("AP_LANDINGGEAR_ENABLED"),
    ),
    sched("standby_update", 100.0, 75, 96, None),
    sched("lost_vehicle_check", 10.0, 50, 99, None),
    sched("GCS::update_receive", 400.0, 180, 102, None),
    sched("GCS::update_send", 400.0, 550, 105, None),
    sched("AP_Mount::update", 50.0, 75, 108, Some("HAL_MOUNT_ENABLED")),
    sched(
        "AP_Camera::update",
        50.0,
        75,
        111,
        Some("AP_CAMERA_ENABLED"),
    ),
    sched(
        "ten_hz_logging_loop",
        10.0,
        350,
        114,
        Some("HAL_LOGGING_ENABLED"),
    ),
    sched(
        "twentyfive_hz_logging",
        25.0,
        110,
        117,
        Some("HAL_LOGGING_ENABLED"),
    ),
    sched(
        "AP_Logger::periodic_tasks",
        400.0,
        300,
        120,
        Some("HAL_LOGGING_ENABLED"),
    ),
    sched("AP_InertialSensor::periodic", 400.0, 50, 123, None),
    sched(
        "AP_Scheduler::update_logging",
        0.1,
        75,
        126,
        Some("HAL_LOGGING_ENABLED"),
    ),
    sched(
        "AP_TempCalibration::update",
        10.0,
        100,
        135,
        Some("AP_TEMPCALIBRATION_ENABLED"),
    ),
    sched(
        "avoidance_adsb_update",
        10.0,
        100,
        138,
        Some("HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED"),
    ),
    sched(
        "afs_fs_check",
        10.0,
        100,
        141,
        Some("AP_COPTER_ADVANCED_FAILSAFE_ENABLED"),
    ),
    sched(
        "terrain_update",
        10.0,
        100,
        144,
        Some("AP_TERRAIN_AVAILABLE"),
    ),
    sched("AP_Winch::update", 50.0, 50, 150, Some("AP_WINCH_ENABLED")),
    sched(
        "userhook_FastLoop",
        100.0,
        75,
        153,
        Some("USERHOOK_FASTLOOP"),
    ),
    sched("userhook_50Hz", 50.0, 75, 156, Some("USERHOOK_50HZLOOP")),
    sched(
        "userhook_MediumLoop",
        10.0,
        75,
        159,
        Some("USERHOOK_MEDIUMLOOP"),
    ),
    sched("userhook_SlowLoop", 3.3, 75, 162, Some("USERHOOK_SLOWLOOP")),
    sched(
        "userhook_SuperSlowLoop",
        1.0,
        75,
        165,
        Some("USERHOOK_SUPERSLOWLOOP"),
    ),
    sched(
        "AP_Button::update",
        5.0,
        100,
        168,
        Some("HAL_BUTTON_ENABLED"),
    ),
    sched(
        "update_dynamic_notch_at_specified_rate_main",
        LOOP_RATE,
        200,
        215,
        Some("AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED"),
    ),
];

/// Remaining `Copter.cpp` / `Copter.h` / `system.cpp` leftovers after the
/// table, `rc_loop`, `throttle_loop`, `update_batt_compass`, the first
/// Copter FAST_TASK bodies including `update_flight_mode` and
/// `update_land_and_crash_detectors`, and the logging / 3 Hz / 1 Hz
/// leftovers.
pub const REMAINING: &[&str] = &[
    "Copter::init_simple_bearing",
    "Copter::update_simple_mode",
    "Copter::update_super_simple_bearing",
    "Copter::update_altitude",
    "Copter::get_wp_distance_m",
    "Copter::check_ekf_reset",
    "Copter::update_home_from_EKF",
    "Copter::update_rangefinder_terrain_offset",
    "Copter::run_nav_updates",
    "Copter::auto_disarm_check",
    "Copter::standby_update",
    "Copter::lost_vehicle_check",
    "Copter::takeoff_check",
    "Copter::init_ardupilot",
    "Copter::startup_INS_ground",
    "Copter::update_auto_armed",
    "Copter::allocate_motors",
];

/// What `Copter::get_scheduler_tasks` hands the vehicle scheduler.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SchedulerTasksView {
    /// The leftover catalog, gated rows included.
    pub tasks: &'static [SchedulerTaskSpec],
    /// `ARRAY_SIZE` of the catalog (gated rows stay visible).
    pub task_count: usize,
    /// `MASK_LOG_PM`.
    pub log_bit: u32,
}

/// `Copter::get_scheduler_tasks`.
#[must_use]
pub const fn get_scheduler_tasks() -> SchedulerTasksView {
    SchedulerTasksView {
        tasks: SCHEDULER_TASKS,
        task_count: SCHEDULER_TASKS.len(),
        log_bit: MASK_LOG_PM,
    }
}

/// Rows compiled on a stock multicopter (no `gate`).
#[must_use]
pub fn always_on_tasks() -> impl Iterator<Item = &'static SchedulerTaskSpec> {
    SCHEDULER_TASKS.iter().filter(|task| task.gate.is_none())
}

/// First scheduled always-on row — `rc_loop`.
#[must_use]
pub fn first_scheduled_task() -> Option<&'static SchedulerTaskSpec> {
    always_on_tasks().find(|task| task.kind == TaskKind::Scheduled)
}

/// Inputs to `RC_Channels::read_mode_switch`, the call `rc_loop` always makes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeSwitchReadInputs {
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `rc().flight_mode_channel()` receiver index, or `None`.
    pub flight_mode_channel: Option<usize>,
}

/// What `RC_Channels::read_mode_switch` asked the channel leftover to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSwitchReadLeftover {
    /// Lost RC — do not walk the six-position switch.
    NoValidInput,
    /// `flight_mode_channel()` was null (`FLTMODE_CH` out of range).
    NoChannel,
    /// Hand the PWM to `RC_Channel::read_mode_switch`.
    Read,
}

/// `RC_Channels::read_mode_switch` leftover.
///
/// The debounce and [`crate::aux::mode_switch_changed`] stay on the
/// channel. This leftover is the two refuses `rc_loop` still *calls*
/// into: invalid input, then a missing mode channel.
#[must_use]
pub const fn read_mode_switch(inputs: ModeSwitchReadInputs) -> ModeSwitchReadLeftover {
    if !inputs.has_valid_input {
        return ModeSwitchReadLeftover::NoValidInput;
    }
    if inputs.flight_mode_channel.is_none() {
        return ModeSwitchReadLeftover::NoChannel;
    }
    ModeSwitchReadLeftover::Read
}

/// What `Copter::rc_loop` asked radio + mode-switch leftovers to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcLoopLeftover {
    /// After `read_radio()`.
    pub radio: ReadRadioLeftover,
    /// After `rc().read_mode_switch()`.
    pub mode_switch: ModeSwitchReadLeftover,
}

/// `Copter::rc_loop`.
///
/// Always `read_radio()` then `rc().read_mode_switch()`. A late radio
/// frame does not skip the mode switch — that refuse is inside the
/// callee. Folding the skip up here would drop a valid switch edge
/// on the same tick the receiver recovered.
#[must_use]
pub fn rc_loop(radio: &ReadRadioInputs, mode: ModeSwitchReadInputs) -> RcLoopLeftover {
    RcLoopLeftover {
        radio: read_radio(radio),
        mode_switch: read_mode_switch(mode),
    }
}

/// What `Copter::read_AHRS` asked AHRS to do.
///
/// The whole leftover is the `true`: INS already ran as the first
/// `FAST_TASK`, so a second `ins.update()` here would consume the
/// same samples twice. Passing `false` would be a different function.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadAhrsLeftover {
    /// Always `ahrs.update(true)`.
    pub skip_ins_update: bool,
}

/// `Copter::read_AHRS`.
#[must_use]
pub const fn read_ahrs() -> ReadAhrsLeftover {
    ReadAhrsLeftover {
        skip_ins_update: true,
    }
}

/// Inputs to `Copter::read_inertia`.
///
/// `ahrs.get_location` is called for its out-parameter, not its bool.
/// A failed location still writes whatever `loc` held — usually zeros —
/// into `current_loc.lat` / `lng`. Folding a missing fix into an early
/// return would leave last-tick coordinates on a vehicle that has lost
/// AHRS, which is not what the leftover does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReadInertiaInputs {
    /// `vibration_check.high_vibes` handed to `pos_control->update_estimates`.
    pub high_vibes: bool,
    /// `loc.lat` after `ahrs.get_location(loc)`.
    pub ahrs_lat: i32,
    /// `loc.lng` after `ahrs.get_location(loc)`.
    pub ahrs_lng: i32,
    /// `AP::ahrs().get_relative_position_D_origin_float`. `None` is the
    /// refuse that returns after lat/lng are already written.
    pub pos_d_m: Option<f32>,
    /// `ahrs.home_is_set()`.
    pub home_is_set: bool,
    /// Home altitude, cm AMSL. Needed for ABOVE_ORIGIN to ABOVE_HOME.
    pub home_alt_cm: Option<i32>,
    /// EKF origin altitude, cm AMSL. Same conversion.
    pub origin_alt_cm: Option<i32>,
}

/// What `Copter::read_inertia` asked pos-control / follow / `current_loc` to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReadInertiaLeftover {
    /// Always: `pos_control->update_estimates(high_vibes)`.
    pub update_estimates: bool,
    /// The vibe flag those estimates saw.
    pub high_vibes: bool,
    /// `MODE_FOLLOW_ENABLED` — `g2.follow.update_estimates()`.
    pub follow_update_estimates: bool,
    /// Lat/lng copied from AHRS even when the altitude refuse fires.
    pub wrote_lat_lng: bool,
    /// `current_loc` after this tick.
    pub current_loc: Location,
    /// False when `get_relative_position_D_origin_float` failed.
    pub altitude_updated: bool,
    /// Home unset, or `change_alt_frame(ABOVE_HOME)` failed: origin metres
    /// were stamped as ABOVE_HOME.
    pub used_home_fallback: bool,
}

/// `Copter::read_inertia`.
///
/// Lat/lng land before the D-origin refuse. A port that returned first
/// would keep last-tick coordinates on a vehicle that lost its origin.
///
/// The home conversion is `!home_is_set() || !change_alt_frame(...)`.
/// Either side is enough to treat origin-relative metres as home-relative
/// — the leftover that lets a vehicle without a home still report an
/// altitude above "home".
#[must_use]
pub fn read_inertia(current_loc: Location, inputs: &ReadInertiaInputs) -> ReadInertiaLeftover {
    let mut loc = current_loc;
    loc.lat = inputs.ahrs_lat;
    loc.lng = inputs.ahrs_lng;

    let Some(pos_d_m) = inputs.pos_d_m else {
        return ReadInertiaLeftover {
            update_estimates: true,
            high_vibes: inputs.high_vibes,
            follow_update_estimates: true,
            wrote_lat_lng: true,
            current_loc: loc,
            altitude_updated: false,
            used_home_fallback: false,
        };
    };

    let alt_above_origin_m = -pos_d_m;
    loc.set_alt_m(alt_above_origin_m, AltFrame::AboveOrigin);
    let alt_ctx = AltContext {
        home_alt_cm: inputs.home_alt_cm,
        origin_alt_cm: inputs.origin_alt_cm,
        terrain_alt_cm: None,
    };
    let used_home_fallback =
        !inputs.home_is_set || !loc.change_alt_frame(AltFrame::AboveHome, &alt_ctx);
    if used_home_fallback {
        loc.set_alt_m(alt_above_origin_m, AltFrame::AboveHome);
    }

    ReadInertiaLeftover {
        update_estimates: true,
        high_vibes: inputs.high_vibes,
        follow_update_estimates: true,
        wrote_lat_lng: true,
        current_loc: loc,
        altitude_updated: true,
        used_home_fallback,
    }
}

/// What `Copter::throttle_loop` asked later leftovers to do.
///
/// Stock multicopter (`FRAME_CONFIG != HELI_FRAME`): the two heli calls
/// stay compiled out. The four always-on callees (`update_throttle_mix`,
/// `update_auto_armed`, `update_ground_effect_detector`,
/// `update_ekf_terrain_height_stable`) remain later leftovers — this
/// leftover is the call order, not their bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThrottleLoopLeftover {
    /// Always: `update_throttle_mix()`.
    pub update_throttle_mix: bool,
    /// Always: `update_auto_armed()`.
    pub update_auto_armed: bool,
    /// `heli_update_rotor_speed_targets` — heli only.
    pub heli_update_rotor_speed_targets: bool,
    /// `heli_update_landing_swash` — heli only.
    pub heli_update_landing_swash: bool,
    /// Always: `update_ground_effect_detector()`.
    pub update_ground_effect_detector: bool,
    /// Always: `update_ekf_terrain_height_stable()`.
    pub update_ekf_terrain_height_stable: bool,
}

/// `Copter::throttle_loop`.
///
/// Mix, then auto-armed, then ground-effect / EKF terrain. Folding
/// auto-armed ahead of mix would flip the 50 Hz order those two leftovers
/// see. The heli pair is compiled out of this leftover, not skipped at
/// runtime — a runtime `if heli` would be a different function.
#[must_use]
pub const fn throttle_loop() -> ThrottleLoopLeftover {
    ThrottleLoopLeftover {
        update_throttle_mix: true,
        update_auto_armed: true,
        heli_update_rotor_speed_targets: false,
        heli_update_landing_swash: false,
        update_ground_effect_detector: true,
        update_ekf_terrain_height_stable: true,
    }
}

/// Inputs to `Copter::motors_output`.
///
/// Advanced-failsafe `should_crash_vehicle` is compiled out of this
/// leftover (`AP_COPTER_ADVANCED_FAILSAFE_ENABLED`). Stock multicopter
/// walks the arming-delay / interlock / drive / push path every tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorsOutputInputs {
    /// `full_push` — the default `motors_output()` argument is `true`.
    pub full_push: bool,
    /// `ap.in_arming_delay` on entry.
    pub in_arming_delay: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `millis()`.
    pub now_ms: u32,
    /// `arm_time_ms` — when the vehicle last armed.
    pub arm_time_ms: u32,
    /// `flightmode->mode_number()`.
    pub mode_number: u8,
    /// `ap.using_interlock`.
    pub using_interlock: bool,
    /// `ap.motor_interlock_switch`.
    pub motor_interlock_switch: bool,
    /// `SRV_Channels::get_emergency_stop()`.
    pub emergency_stop: bool,
    /// `motors->get_interlock()` on entry.
    pub motors_interlock: bool,
    /// `ap.motor_test`.
    pub motor_test: bool,
}

/// Which PWM push `motors_output` took.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorsOutputPush {
    /// `full_push` — `AP::srv().push()`, servos included.
    Srv,
    /// Rate-thread path — `hal.rcout->push()`, motors only.
    Rcout,
}

/// Who wrote the motor demands this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorsOutputDrive {
    /// `ap.motor_test` — `motor_test_output()`.
    MotorTest,
    /// `flightmode->output_to_motors()`.
    FlightMode,
}

/// Interlock edge that wants a `LogEvent`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterlockEdge {
    /// No change.
    None,
    /// Off → on. `LogEvent::MOTORS_INTERLOCK_ENABLED`.
    Enabled,
    /// On → off. `LogEvent::MOTORS_INTERLOCK_DISABLED`.
    Disabled,
}

/// What `Copter::motors_output` asked motors / SRV leftovers to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorsOutputLeftover {
    /// After the delay-clear test.
    pub in_arming_delay: bool,
    /// Always: `SRV_Channels::calc_pwm()`.
    pub calc_pwm: bool,
    /// Always: `AP::srv().cork()`.
    pub cork: bool,
    /// Always: `SRV_Channels::output_ch_all()`.
    pub output_ch_all: bool,
    /// Armed, not in delay, interlock switch (if used) on, e-stop off.
    pub interlock: bool,
    /// Log the interlock transition, if any.
    pub interlock_edge: InterlockEdge,
    /// Motor-test versus flight-mode output.
    pub drive: MotorsOutputDrive,
    /// `srv.push()` versus `hal.rcout->push()`.
    pub push: MotorsOutputPush,
}

/// `Copter::motors_output`.
///
/// The interlock formula uses the *cleared* arming-delay flag. A port
/// that computed interlock from the entry flag would keep motors locked
/// out for one extra loop after the two-second delay expired — and a
/// THROW mode that is supposed to spool immediately would wait too.
///
/// `calc_pwm` / cork / `output_ch_all` always run, even when the
/// interlock is off. Folding them behind the interlock would drop
/// passthrough aux channels on a disarmed vehicle.
#[must_use]
pub fn motors_output(inputs: &MotorsOutputInputs) -> MotorsOutputLeftover {
    let mut in_arming_delay = inputs.in_arming_delay;
    if in_arming_delay
        && (!inputs.armed
            || inputs.now_ms.wrapping_sub(inputs.arm_time_ms) > ARMING_DELAY_MS
            || inputs.mode_number == MODE_THROW)
    {
        in_arming_delay = false;
    }

    let interlock = inputs.armed
        && !in_arming_delay
        && (!inputs.using_interlock || inputs.motor_interlock_switch)
        && !inputs.emergency_stop;

    let interlock_edge = if !inputs.motors_interlock && interlock {
        InterlockEdge::Enabled
    } else if inputs.motors_interlock && !interlock {
        InterlockEdge::Disabled
    } else {
        InterlockEdge::None
    };

    MotorsOutputLeftover {
        in_arming_delay,
        calc_pwm: true,
        cork: true,
        output_ch_all: true,
        interlock,
        interlock_edge,
        drive: if inputs.motor_test {
            MotorsOutputDrive::MotorTest
        } else {
            MotorsOutputDrive::FlightMode
        },
        push: if inputs.full_push {
            MotorsOutputPush::Srv
        } else {
            MotorsOutputPush::Rcout
        },
    }
}

/// What `Copter::motors_output_main` asked `motors_output` to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorsOutputMainLeftover {
    /// Rate thread owns `motors_output`.
    Skipped,
    /// Main thread ran `motors_output()` (`full_push = true`).
    Ran(MotorsOutputLeftover),
}

/// `Copter::motors_output_main`.
///
/// The default `full_push` is forced on here. The rate thread is the
/// only caller that passes `false`, and it does not go through this
/// leftover.
#[must_use]
pub fn motors_output_main(
    using_rate_thread: bool,
    inputs: &MotorsOutputInputs,
) -> MotorsOutputMainLeftover {
    if using_rate_thread {
        return MotorsOutputMainLeftover::Skipped;
    }
    let mut main = *inputs;
    main.full_push = true;
    MotorsOutputMainLeftover::Ran(motors_output(&main))
}

/// Inputs to `Copter::update_flight_mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateFlightModeInputs {
    /// `copter.ap.land_complete` — handed to `landed_gain_reduction`.
    pub land_complete: bool,
    /// `flightmode->move_vehicle_on_ekf_reset()`.
    pub move_vehicle_on_ekf_reset: bool,
}

/// What `Copter::update_flight_mode` asked later leftovers to do.
///
/// Mode `run()` bodies stay on their own leftovers. This leftover is the
/// call order: invalidate surface-tracking, reduce landed gains, pick the
/// EKF reset method, then `flightmode->run()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateFlightModeLeftover {
    /// `AP_RANGEFINDER_ENABLED` — `surface_tracking.invalidate_for_logging()`.
    ///
    /// Stock multicopter compiles this in. The mode `run()` that follows
    /// may set `valid_for_logging` again if it actually uses the rangefinder.
    pub invalidate_for_logging: bool,
    /// Always: `attitude_control->landed_gain_reduction(land_complete)`.
    pub landed_gain_reduction: bool,
    /// The `land_complete` those gains saw.
    pub land_complete: bool,
    /// `pos_control->set_reset_handling_method(...)`.
    pub reset_handling: EkfResetMethod,
    /// Always: `flightmode->run()`.
    pub flightmode_run: bool,
}

/// `Copter::update_flight_mode`.
///
/// Gains and the EKF reset method are chosen *before* `run()`. Folding
/// `run()` first would let a mode that changes its own submode this tick
/// pick a reset method the leftover has not yet published, and would let
/// motors see last-tick landed gains for one loop after touchdown.
///
/// The reset method is [`ekf_reset_method`] — default modes return false
/// (`MoveTarget`). Guided/auto position legs return true (`MoveVehicle`).
#[must_use]
pub fn update_flight_mode(inputs: UpdateFlightModeInputs) -> UpdateFlightModeLeftover {
    UpdateFlightModeLeftover {
        invalidate_for_logging: true,
        landed_gain_reduction: true,
        land_complete: inputs.land_complete,
        reset_handling: ekf_reset_method(inputs.move_vehicle_on_ekf_reset),
        flightmode_run: true,
    }
}

/// What `Copter::update_land_and_crash_detectors` asked later leftovers to do.
///
/// `update_land_detector` is COP-021 and `crash_check` is COP-019. Thrust-loss,
/// yaw-imbalance, and parachute stay later leftovers — this leftover is the
/// call order and the gravity add the 1 Hz accel filter sees.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateLandAndCrashLeftover {
    /// Always: `z += GRAVITY_MSS` then `land_accel_ef_filter.apply`.
    pub apply_land_accel_filter: bool,
    /// Gravity was added to earth-frame Z before the filter.
    ///
    /// A port that filtered raw AHRS accel would treat 1 G hover as motion
    /// — `crash_check` compares the *filtered* vector after this add.
    pub gravity_added_to_z: bool,
    /// Always: `update_land_detector()`.
    pub update_land_detector: bool,
    /// `HAL_PARACHUTE_ENABLED` — compiled out of this leftover.
    pub parachute_check: bool,
    /// Always: `crash_check()`.
    pub crash_check: bool,
    /// Always: `thrust_loss_check()`.
    pub thrust_loss_check: bool,
    /// Always: `yaw_imbalance_check()`.
    pub yaw_imbalance_check: bool,
}

/// `Copter::update_land_and_crash_detectors`.
///
/// Land detector runs before crash / thrust-loss / yaw-imbalance. Folding
/// crash first would let `crash_check` see last-tick `ap.land_complete` on
/// the same loop the detector raised it. The parachute call is compiled
/// out, not skipped at runtime — a runtime `if parachute` would be a
/// different function.
#[must_use]
pub const fn update_land_and_crash_detectors() -> UpdateLandAndCrashLeftover {
    UpdateLandAndCrashLeftover {
        apply_land_accel_filter: true,
        gravity_added_to_z: true,
        update_land_detector: true,
        parachute_check: false,
        crash_check: true,
        thrust_loss_check: true,
        yaw_imbalance_check: true,
    }
}

/// Inputs to `Copter::update_batt_compass`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateBattCompassInputs {
    /// `AP::compass().available()`.
    pub compass_available: bool,
}

/// What `Copter::update_batt_compass` asked battery / compass leftovers to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateBattCompassLeftover {
    /// Always: `battery.read()`.
    pub battery_read: bool,
    /// Compassmot throttle — only when the compass is available.
    pub compass_set_throttle: bool,
    /// Compassmot voltage — only when the compass is available.
    pub compass_set_voltage: bool,
    /// `compass.read()` — only when the compass is available.
    pub compass_read: bool,
}

/// `Copter::update_batt_compass`.
///
/// Battery is read first, even when the compass is missing. Compassmot
/// compensation uses that voltage; folding `compass.read()` first would
/// compensate with last-tick throttle and voltage. A missing compass
/// still reads the battery — the 10 Hz current integration is not a
/// compass helper.
#[must_use]
pub const fn update_batt_compass(inputs: UpdateBattCompassInputs) -> UpdateBattCompassLeftover {
    UpdateBattCompassLeftover {
        battery_read: true,
        compass_set_throttle: inputs.compass_available,
        compass_set_voltage: inputs.compass_available,
        compass_read: inputs.compass_available,
    }
}

/// `Copter::should_log` / `AP_Logger::should_log` first reject.
///
/// Armed / download / backend-count checks stay later. This leftover is
/// the bitmask test the logging loops still *call* into: a zero overlap
/// must not emit.
#[must_use]
pub const fn should_log(log_bitmask: u32, mask: u32) -> bool {
    (mask & log_bitmask) != 0
}

/// Packed `Copter::ap` bools.
///
/// Upstream comments the bit index of each field. [`ap_value`] walks
/// them in that order — reordering a field would change the logged
/// `AP_STATE` bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ApState {
    /// Bit 0.
    pub unused1: bool,
    /// Bit 1. Was `simple_mode` byte 1.
    pub unused_was_simple_mode_byte1: bool,
    /// Bit 2. Was `simple_mode` byte 2.
    pub unused_was_simple_mode_byte2: bool,
    /// Bit 3. RC input pre-arm checks passed.
    pub pre_arm_rc_check: bool,
    /// Bit 4. All pre-arm checks passed.
    pub pre_arm_check: bool,
    /// Bit 5. Auto missions wait for throttle.
    pub auto_armed: bool,
    /// Bit 6. Was `log_started`.
    pub unused_log_started: bool,
    /// Bit 7. Land detector has landed.
    pub land_complete: bool,
    /// Bit 8. Fresh PWM this radio frame.
    pub new_radio_frame: bool,
    /// Bit 9. Was `usb_connected`.
    pub unused_usb_connected: bool,
    /// Bit 10. Was `receiver_present`.
    pub unused_receiver_present: bool,
    /// Bit 11. Compassmot calibration running.
    pub compass_mot: bool,
    /// Bit 12. Motor test running.
    pub motor_test: bool,
    /// Bit 13. `init_ardupilot` finished.
    pub initialised: bool,
    /// Bit 14. Softer land detector.
    pub land_complete_maybe: bool,
    /// Bit 15. Throttle stick at zero, debounced.
    pub throttle_zero: bool,
    /// Bit 16. Was `system_time_set`.
    pub system_time_set_unused: bool,
    /// Bit 17. GPS glitch affecting nav.
    pub gps_glitching: bool,
    /// Bit 18. Aux motor-interlock in use.
    pub using_interlock: bool,
    /// Bit 19. Pilot overriding land position.
    pub land_repo_active: bool,
    /// Bit 20. Pilot requesting interlock enable.
    pub motor_interlock_switch: bool,
    /// Bit 21. Armed but waiting to spool.
    pub in_arming_delay: bool,
    /// Bit 22. Parameters finished initialising.
    pub initialised_params: bool,
    /// Bit 23. Was `compass_init_location`.
    pub unused_compass_init_location: bool,
    /// Bit 24. Was aux-switch RC override.
    pub unused2_aux_switch_rc_override_allowed: bool,
    /// Bit 25. Armed from the airmode switch.
    pub armed_with_airmode_switch: bool,
    /// Bit 26. PrecLand active.
    pub prec_land_active: bool,
}

impl ApState {
    /// `sizeof(ap)` bools in declaration order.
    #[must_use]
    pub const fn bits(self) -> [bool; AP_STATE_BOOL_COUNT] {
        [
            self.unused1,
            self.unused_was_simple_mode_byte1,
            self.unused_was_simple_mode_byte2,
            self.pre_arm_rc_check,
            self.pre_arm_check,
            self.auto_armed,
            self.unused_log_started,
            self.land_complete,
            self.new_radio_frame,
            self.unused_usb_connected,
            self.unused_receiver_present,
            self.compass_mot,
            self.motor_test,
            self.initialised,
            self.land_complete_maybe,
            self.throttle_zero,
            self.system_time_set_unused,
            self.gps_glitching,
            self.using_interlock,
            self.land_repo_active,
            self.motor_interlock_switch,
            self.in_arming_delay,
            self.initialised_params,
            self.unused_compass_init_location,
            self.unused2_aux_switch_rc_override_allowed,
            self.armed_with_airmode_switch,
            self.prec_land_active,
        ]
    }
}

/// `Copter::ap_value`.
///
/// Walks the packed `ap` bools in declaration order. A port that packed
/// the live flags into a hand-built mask would drift the moment a
/// reserved / unused slot flipped — the log message is the byte walk.
#[must_use]
pub const fn ap_value(ap: ApState) -> u32 {
    let bits = ap.bits();
    let mut ret = 0u32;
    let mut i = 0;
    while i < AP_STATE_BOOL_COUNT {
        if bits[i] {
            ret |= 1u32 << i;
        }
        i += 1;
    }
    ret
}

/// Inputs to `Copter::loop_rate_logging`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopRateLoggingInputs {
    /// `LOG_BITMASK`.
    pub log_bitmask: u32,
    /// `flightmode->logs_attitude()` — Stabilize / Acro write ATT themselves.
    pub logs_attitude: bool,
    /// `using_rate_thread` — Rate / PID / notch live on the rate thread.
    pub using_rate_thread: bool,
}

/// What `Copter::loop_rate_logging` asked later leftovers to write.
///
/// Harmonic-notch FTN is compiled out of this leftover
/// (`AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED`). Stock multicopter still
/// always writes SPOL — that is not a `should_log` branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopRateLoggingLeftover {
    /// `Log_Write_Attitude()` — ATT_FAST and the mode is not already logging.
    pub write_attitude: bool,
    /// `Log_Write_Rate()` — same gate, and no rate thread.
    pub write_rate: bool,
    /// `Log_Write_PIDS()` — same gate, and no rate thread.
    pub write_pids: bool,
    /// `ins.write_notch_log_messages()` — compiled out.
    pub write_notch: bool,
    /// `ins.Write_IMU()` — IMU_FAST.
    pub write_imu: bool,
    /// Always: `motors->Log_Write_SPOL()`.
    pub write_spol: bool,
}

/// `Copter::loop_rate_logging`.
///
/// ATT_FAST attitude / rate / PID share one gate. Folding Rate/PID
/// behind a second bitmask test would drop them on a vehicle that set
/// ATT_FAST but not PID — `Log_Write_PIDS` does that check itself.
/// SPOL is not gated; a port that hid it behind ATT_FAST would lose
/// spool traces on a MED-only bitmask.
#[must_use]
pub const fn loop_rate_logging(inputs: LoopRateLoggingInputs) -> LoopRateLoggingLeftover {
    let att_fast = should_log(inputs.log_bitmask, MASK_LOG_ATTITUDE_FAST) && !inputs.logs_attitude;
    LoopRateLoggingLeftover {
        write_attitude: att_fast,
        write_rate: att_fast && !inputs.using_rate_thread,
        write_pids: att_fast && !inputs.using_rate_thread,
        write_notch: false,
        write_imu: should_log(inputs.log_bitmask, MASK_LOG_IMU_FAST),
        write_spol: true,
    }
}

/// Inputs to `Copter::ten_hz_logging_loop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TenHzLoggingInputs {
    /// `LOG_BITMASK`.
    pub log_bitmask: u32,
    /// `flightmode->logs_attitude()`.
    pub logs_attitude: bool,
    /// `using_rate_thread`.
    pub using_rate_thread: bool,
    /// `flightmode->requires_position()`.
    pub requires_position: bool,
    /// `landing_with_GPS()`.
    pub landing_with_gps: bool,
    /// `flightmode->has_manual_throttle()`.
    pub has_manual_throttle: bool,
}

/// What `Copter::ten_hz_logging_loop` asked later leftovers to write.
///
/// Heli always-write motors, RSSI, proximity, beacon, winch, and mount
/// are compiled out. Stock multicopter still always writes AHRS
/// attitude — that is not a `should_log` branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TenHzLoggingLeftover {
    /// Always: `ahrs.Write_Attitude(att_target_euler_rad * RAD_TO_DEG)`.
    pub write_ahrs_attitude: bool,
    /// `Log_Write_Attitude()` — ATT_MED, not ATT_FAST, mode not already logging.
    pub write_attitude: bool,
    /// `Log_Write_Rate()` — same ATT_MED gate, and no rate thread.
    pub write_rate: bool,
    /// `Log_Write_PIDS()` — not ATT_FAST, mode not already logging, no rate thread.
    pub write_pids: bool,
    /// `Log_Write_EKF_POS()` — not ATT_FAST (the 25 Hz leftover owns that).
    pub write_ekf_pos: bool,
    /// `motors->Log_Write()` — MOTBATT (heli always-write compiled out).
    pub write_motors: bool,
    /// `logger.Write_RCIN()` — RCIN.
    pub write_rcin: bool,
    /// `logger.Write_RSSI()` — compiled out (`AP_RSSI_ENABLED`).
    pub write_rssi: bool,
    /// `logger.Write_RCOUT()` — RCOUT.
    pub write_rcout: bool,
    /// `pos_control->write_log()` — NTUN and a position / auto-throttle mode.
    pub write_ntun: bool,
    /// `ins.Write_Vibration()` — IMU or IMU_FAST or IMU_RAW.
    pub write_vibration: bool,
    /// `g2.proximity.log()` — compiled out.
    pub write_proximity: bool,
    /// `g2.beacon.log()` — compiled out.
    pub write_beacon: bool,
    /// `g2.winch.write_log()` — compiled out.
    pub write_winch: bool,
    /// `camera_mount.write_log()` — compiled out.
    pub write_mount: bool,
}

/// `Copter::ten_hz_logging_loop`.
///
/// AHRS attitude is written first, even when every bitmask is clear.
/// Folding it behind ATT_MED would drop the 10 Hz target on a FAST-only
/// vehicle — FAST already wrote ATT at loop rate, but the AHRS view of
/// the target still belongs here.
///
/// PID at 10 Hz is the *not*-FAST path. FAST + PID logs at loop rate
/// instead; a port that also wrote PID here would double the rate.
///
/// NTUN needs a position or auto-throttle mode. Stabilize with NTUN
/// set still refuses — the leftover is `requires_position ||
/// landing_with_GPS || !has_manual_throttle`.
#[must_use]
pub const fn ten_hz_logging_loop(inputs: TenHzLoggingInputs) -> TenHzLoggingLeftover {
    let att_fast = should_log(inputs.log_bitmask, MASK_LOG_ATTITUDE_FAST);
    let att_med = should_log(inputs.log_bitmask, MASK_LOG_ATTITUDE_MED);
    let write_attitude = att_med && !att_fast && !inputs.logs_attitude;
    let write_ntun = should_log(inputs.log_bitmask, MASK_LOG_NTUN)
        && (inputs.requires_position || inputs.landing_with_gps || !inputs.has_manual_throttle);
    TenHzLoggingLeftover {
        write_ahrs_attitude: true,
        write_attitude,
        write_rate: write_attitude && !inputs.using_rate_thread,
        write_pids: !att_fast && !inputs.logs_attitude && !inputs.using_rate_thread,
        write_ekf_pos: !att_fast,
        write_motors: should_log(inputs.log_bitmask, MASK_LOG_MOTBATT),
        write_rcin: should_log(inputs.log_bitmask, MASK_LOG_RCIN),
        write_rssi: false,
        write_rcout: should_log(inputs.log_bitmask, MASK_LOG_RCOUT),
        write_ntun,
        write_vibration: should_log(inputs.log_bitmask, MASK_LOG_IMU)
            || should_log(inputs.log_bitmask, MASK_LOG_IMU_FAST)
            || should_log(inputs.log_bitmask, MASK_LOG_IMU_RAW),
        write_proximity: false,
        write_beacon: false,
        write_winch: false,
        write_mount: false,
    }
}

/// Inputs to `Copter::twentyfive_hz_logging`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TwentyfiveHzLoggingInputs {
    /// `LOG_BITMASK`.
    pub log_bitmask: u32,
}

/// What `Copter::twentyfive_hz_logging` asked later leftovers to write.
///
/// Gyro-FFT FTN is compiled out (`HAL_GYROFFT_ENABLED`). EKF-POS moves
/// here only when ATT_FAST already claimed the 10 Hz slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TwentyfiveHzLoggingLeftover {
    /// `Log_Write_EKF_POS()` — ATT_FAST.
    pub write_ekf_pos: bool,
    /// `ins.Write_IMU()` — IMU and not IMU_FAST (loop-rate leftover owns FAST).
    pub write_imu: bool,
    /// `gyro_fft.write_log_messages()` — compiled out.
    pub write_gyro_fft: bool,
}

/// `Copter::twentyfive_hz_logging`.
///
/// IMU_FAST already wrote IMU at loop rate. Folding the 25 Hz IMU
/// write in anyway would double the rate. EKF-POS is the ATT_FAST
/// counterpart of the 10 Hz leftover — a port that wrote it in both
/// would double that too.
#[must_use]
pub const fn twentyfive_hz_logging(
    inputs: TwentyfiveHzLoggingInputs,
) -> TwentyfiveHzLoggingLeftover {
    TwentyfiveHzLoggingLeftover {
        write_ekf_pos: should_log(inputs.log_bitmask, MASK_LOG_ATTITUDE_FAST),
        write_imu: should_log(inputs.log_bitmask, MASK_LOG_IMU)
            && !should_log(inputs.log_bitmask, MASK_LOG_IMU_FAST),
        write_gyro_fft: false,
    }
}

/// What `Copter::three_hz_loop` asked later leftovers to do.
///
/// Transmitter tuning is compiled out (`AP_RC_TRANSMITTER_TUNING_ENABLED`).
/// The three failsafe checks and low-alt avoidance stay later leftovers
/// — this leftover is the call order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThreeHzLoopLeftover {
    /// Always: `failsafe_gcs_check()`.
    pub failsafe_gcs_check: bool,
    /// Always: `failsafe_terrain_check()`.
    pub failsafe_terrain_check: bool,
    /// Always: `failsafe_deadreckon_check()`.
    pub failsafe_deadreckon_check: bool,
    /// `tuning()` — compiled out.
    pub tuning: bool,
    /// Always: `low_alt_avoidance()`.
    pub low_alt_avoidance: bool,
}

/// `Copter::three_hz_loop`.
///
/// GCS, then terrain, then dead-reckon, then low-alt avoidance.
/// Folding dead-reckon ahead of terrain would let a missing-terrain
/// vehicle also raise the dead-reckon flag on the same 3 Hz tick the
/// terrain leftover had not yet published.
#[must_use]
pub const fn three_hz_loop() -> ThreeHzLoopLeftover {
    ThreeHzLoopLeftover {
        failsafe_gcs_check: true,
        failsafe_terrain_check: true,
        failsafe_deadreckon_check: true,
        tuning: false,
        low_alt_avoidance: true,
    }
}

/// Inputs to `Copter::one_hz_loop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OneHzLoopInputs {
    /// `LOG_BITMASK`.
    pub log_bitmask: u32,
    /// `motors->armed()`.
    pub motors_armed: bool,
    /// `using_rate_thread`.
    pub using_rate_thread: bool,
    /// `ap.land_complete`.
    pub land_complete: bool,
}

/// What `Copter::one_hz_loop` asked later leftovers to do.
///
/// ADS-B flying, custom-control notch, and the rate-thread spawn are
/// compiled out. Disarmed-only frame / throttle-range updates stay
/// behind `!armed` — folding them onto an armed vehicle would let a
/// mid-flight `FRAME_CLASS` write retune the mixer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OneHzLoopLeftover {
    /// `Log_Write_Data(AP_STATE, ap_value())` — `MASK_LOG_ANY`.
    pub log_ap_state: bool,
    /// `update_using_interlock()` — disarmed only.
    pub update_using_interlock: bool,
    /// `motors->set_frame_class_and_type` — disarmed only.
    pub set_frame_class_and_type: bool,
    /// `motors->update_throttle_range()` — disarmed, not heli.
    pub update_throttle_range: bool,
    /// Always: `AP::srv().enable_aux_servos()`.
    pub enable_aux_servos: bool,
    /// Always: `terrain_logging()` (`HAL_LOGGING_ENABLED`).
    pub terrain_logging: bool,
    /// `adsb.set_is_flying` — compiled out.
    pub adsb_set_is_flying: bool,
    /// Always: `AP_Notify::flags.flying = !land_complete`.
    pub notify_flying: bool,
    /// The `flying` flag that notify saw.
    pub flying: bool,
    /// `attitude_control->set_notch_sample_rate` — no rate thread.
    pub attitude_notch_sample_rate: bool,
    /// Always: `pos_control` D-accel PID notch sample rate.
    pub pos_control_notch_sample_rate: bool,
    /// `custom_control.set_notch_sample_rate` — compiled out.
    pub custom_control_notch_sample_rate: bool,
    /// Rate-thread spawn — compiled out.
    pub start_rate_thread: bool,
}

/// `Copter::one_hz_loop`.
///
/// AP_STATE is logged from [`ap_value`] when any low-16 bitmask bit is
/// set. `MOTBATT` (bit 17) alone is not `MASK_LOG_ANY` — a port that
/// treated "any log bit" as the full 32-bit mask would emit AP_STATE
/// on a MOTBATT-only vehicle.
///
/// Frame class / throttle range run only while disarmed. Aux servos
/// and the flying notify still run armed — folding those behind
/// `!armed` would freeze aux assignment and the notify flag in flight.
#[must_use]
pub const fn one_hz_loop(inputs: OneHzLoopInputs) -> OneHzLoopLeftover {
    let disarmed = !inputs.motors_armed;
    OneHzLoopLeftover {
        log_ap_state: should_log(inputs.log_bitmask, MASK_LOG_ANY),
        update_using_interlock: disarmed,
        set_frame_class_and_type: disarmed,
        update_throttle_range: disarmed,
        enable_aux_servos: true,
        terrain_logging: true,
        adsb_set_is_flying: false,
        notify_flying: true,
        flying: !inputs.land_complete,
        attitude_notch_sample_rate: !inputs.using_rate_thread,
        pos_control_notch_sample_rate: true,
        custom_control_notch_sample_rate: false,
        start_rate_thread: false,
    }
}

/// Per-callback accounting for the leftovers this slice wires.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VehicleLoopTicks {
    /// Upstream `Copter::run_rate_controller_main`.
    pub run_rate_controller_main: u32,
    /// Upstream `Copter::motors_output_main`.
    pub motors_output_main: u32,
    /// Upstream `Copter::read_AHRS`.
    pub read_ahrs: u32,
    /// Upstream `Copter::read_inertia`.
    pub read_inertia: u32,
    /// Upstream `Copter::rc_loop`.
    pub rc_loop: u32,
    /// Upstream `Copter::throttle_loop`.
    pub throttle_loop: u32,
    /// Upstream `Copter::update_flight_mode`.
    pub update_flight_mode: u32,
    /// Upstream `Copter::update_land_and_crash_detectors`.
    pub update_land_and_crash_detectors: u32,
    /// Upstream `Copter::update_batt_compass`.
    pub update_batt_compass: u32,
    /// Upstream `Copter::loop_rate_logging`.
    pub loop_rate_logging: u32,
    /// Upstream `Copter::ten_hz_logging_loop`.
    pub ten_hz_logging_loop: u32,
    /// Upstream `Copter::twentyfive_hz_logging`.
    pub twentyfive_hz_logging: u32,
    /// Upstream `Copter::three_hz_loop`.
    pub three_hz_loop: u32,
    /// Upstream `Copter::one_hz_loop`.
    pub one_hz_loop: u32,
}

/// Vehicle state the wired leftovers carry between ticks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CopterVehicleLoop {
    /// Per-callback tick counts.
    pub ticks: VehicleLoopTicks,
    /// `using_rate_thread` — skips the rate run and `motors_output_main`.
    pub using_rate_thread: bool,
    /// `AP::scheduler().get_last_loop_time_s()`.
    pub last_loop_time_s: f32,
    /// Leftover from the latest `run_rate_controller_main` tick.
    pub last_rate: Option<RateControllerMainLeftover>,
    /// Inputs for `motors_output`.
    pub motors: MotorsOutputInputs,
    /// Leftover from the latest `motors_output_main` tick.
    pub last_motors: Option<MotorsOutputMainLeftover>,
    /// Leftover from the latest `read_AHRS` tick.
    pub last_ahrs: Option<ReadAhrsLeftover>,
    /// Inputs for `read_inertia`.
    pub inertia: ReadInertiaInputs,
    /// `current_loc` carried between inertia ticks.
    pub current_loc: Location,
    /// Leftover from the latest `read_inertia` tick.
    pub last_inertia: Option<ReadInertiaLeftover>,
    /// Inputs for the `read_radio` half of `rc_loop`.
    pub radio: ReadRadioInputs,
    /// Inputs for the `read_mode_switch` half of `rc_loop`.
    pub mode_switch: ModeSwitchReadInputs,
    /// Leftover from the latest `rc_loop` tick.
    pub last_rc: Option<RcLoopLeftover>,
    /// Leftover from the latest `throttle_loop` tick.
    pub last_throttle: Option<ThrottleLoopLeftover>,
    /// Inputs for `update_flight_mode`.
    pub flight_mode: UpdateFlightModeInputs,
    /// Leftover from the latest `update_flight_mode` tick.
    pub last_flight_mode: Option<UpdateFlightModeLeftover>,
    /// Leftover from the latest `update_land_and_crash_detectors` tick.
    pub last_land_crash: Option<UpdateLandAndCrashLeftover>,
    /// Inputs for `update_batt_compass`.
    pub batt_compass: UpdateBattCompassInputs,
    /// Leftover from the latest `update_batt_compass` tick.
    pub last_batt_compass: Option<UpdateBattCompassLeftover>,
    /// `LOG_BITMASK` for the logging leftovers.
    pub log_bitmask: u32,
    /// `flightmode->logs_attitude()`.
    pub logs_attitude: bool,
    /// `flightmode->requires_position()`.
    pub requires_position: bool,
    /// `landing_with_GPS()`.
    pub landing_with_gps: bool,
    /// `flightmode->has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// Packed `ap` bools for [`ap_value`].
    pub ap: ApState,
    /// Leftover from the latest `loop_rate_logging` tick.
    pub last_loop_rate_logging: Option<LoopRateLoggingLeftover>,
    /// Leftover from the latest `ten_hz_logging_loop` tick.
    pub last_ten_hz_logging: Option<TenHzLoggingLeftover>,
    /// Leftover from the latest `twentyfive_hz_logging` tick.
    pub last_twentyfive_hz_logging: Option<TwentyfiveHzLoggingLeftover>,
    /// Leftover from the latest `three_hz_loop` tick.
    pub last_three_hz: Option<ThreeHzLoopLeftover>,
    /// Leftover from the latest `one_hz_loop` tick.
    pub last_one_hz: Option<OneHzLoopLeftover>,
}

impl CopterVehicleLoop {
    /// Healthy frame + mode channel 5 (`CH_MODE_DEFAULT`).
    #[must_use]
    pub fn typical() -> Self {
        Self {
            ticks: VehicleLoopTicks::default(),
            using_rate_thread: false,
            last_loop_time_s: 1.0 / f32::from(COPTER_LOOP_RATE_HZ),
            last_rate: None,
            motors: typical_motors_output(),
            last_motors: None,
            last_ahrs: None,
            inertia: typical_read_inertia(),
            current_loc: Location::new(0, 0),
            last_inertia: None,
            radio: typical_radio_frame(),
            mode_switch: ModeSwitchReadInputs {
                has_valid_input: true,
                flight_mode_channel: Some(4),
            },
            last_rc: None,
            last_throttle: None,
            flight_mode: UpdateFlightModeInputs {
                land_complete: false,
                move_vehicle_on_ekf_reset: false,
            },
            last_flight_mode: None,
            last_land_crash: None,
            batt_compass: UpdateBattCompassInputs {
                compass_available: true,
            },
            last_batt_compass: None,
            log_bitmask: DEFAULT_LOG_BITMASK,
            logs_attitude: false,
            requires_position: true,
            landing_with_gps: false,
            has_manual_throttle: false,
            ap: ApState::default(),
            last_loop_rate_logging: None,
            last_ten_hz_logging: None,
            last_twentyfive_hz_logging: None,
            last_three_hz: None,
            last_one_hz: None,
        }
    }
}

/// A just-arrived RC frame with a resting throttle above `FS_THR_VALUE`.
#[must_use]
pub fn typical_radio_frame() -> ReadRadioInputs {
    ReadRadioInputs {
        got_input: true,
        now_ms: 1_000,
        last_radio_update_ms: 995,
        fs_timeout_s: 1.0,
        failsafe: ThrottleFailsafeInputs {
            failsafe_throttle: 1,
            failsafe_throttle_value: FS_THR_VALUE_COPTER_DEFAULT,
            throttle_pwm: 1_500,
            radio: false,
            radio_counter: 0,
            has_ever_seen_rc_input: true,
            armed: false,
        },
        throttle_zero: ThrottleZeroInputs {
            throttle_control: 500,
            using_interlock: false,
            emergency_stop: false,
            motor_interlock: false,
            armed_with_airmode_switch: false,
            air_mode: AirMode::None,
            last_nonzero_throttle_ms: 1_000,
            now_ms: 1_000,
            throttle_zero: false,
        },
    }
}

/// Origin 10 m below the vehicle, home and origin at the same AMSL.
#[must_use]
pub fn typical_read_inertia() -> ReadInertiaInputs {
    ReadInertiaInputs {
        high_vibes: false,
        ahrs_lat: 377_749_000,
        ahrs_lng: -1_224_194_000,
        pos_d_m: Some(-10.0),
        home_is_set: true,
        home_alt_cm: Some(10_000),
        origin_alt_cm: Some(10_000),
    }
}

/// Disarmed, not in the arming delay, flight-mode drive, full push.
#[must_use]
pub fn typical_motors_output() -> MotorsOutputInputs {
    MotorsOutputInputs {
        full_push: true,
        in_arming_delay: false,
        armed: false,
        now_ms: 1_000,
        arm_time_ms: 0,
        mode_number: 0,
        using_interlock: false,
        motor_interlock_switch: false,
        emergency_stop: false,
        motors_interlock: false,
        motor_test: false,
    }
}

fn task_run_rate_controller_main(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.run_rate_controller_main =
        vehicle.ticks.run_rate_controller_main.saturating_add(1);
    vehicle.last_rate = Some(run_rate_controller_main(
        vehicle.last_loop_time_s,
        vehicle.using_rate_thread,
    ));
}

fn task_motors_output_main(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.motors_output_main = vehicle.ticks.motors_output_main.saturating_add(1);
    let leftover = motors_output_main(vehicle.using_rate_thread, &vehicle.motors);
    if let MotorsOutputMainLeftover::Ran(out) = leftover {
        vehicle.motors.in_arming_delay = out.in_arming_delay;
        vehicle.motors.motors_interlock = out.interlock;
        vehicle.motors.full_push = true;
    }
    vehicle.last_motors = Some(leftover);
}

fn task_read_ahrs(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.read_ahrs = vehicle.ticks.read_ahrs.saturating_add(1);
    vehicle.last_ahrs = Some(read_ahrs());
}

fn task_read_inertia(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.read_inertia = vehicle.ticks.read_inertia.saturating_add(1);
    let leftover = read_inertia(vehicle.current_loc, &vehicle.inertia);
    vehicle.current_loc = leftover.current_loc;
    vehicle.last_inertia = Some(leftover);
}

fn task_rc_loop(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.rc_loop = vehicle.ticks.rc_loop.saturating_add(1);
    vehicle.last_rc = Some(rc_loop(&vehicle.radio, vehicle.mode_switch));
}

fn task_throttle_loop(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.throttle_loop = vehicle.ticks.throttle_loop.saturating_add(1);
    vehicle.last_throttle = Some(throttle_loop());
}

fn task_update_flight_mode(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.update_flight_mode = vehicle.ticks.update_flight_mode.saturating_add(1);
    vehicle.last_flight_mode = Some(update_flight_mode(vehicle.flight_mode));
}

fn task_update_land_and_crash_detectors(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.update_land_and_crash_detectors = vehicle
        .ticks
        .update_land_and_crash_detectors
        .saturating_add(1);
    vehicle.last_land_crash = Some(update_land_and_crash_detectors());
}

fn task_update_batt_compass(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.update_batt_compass = vehicle.ticks.update_batt_compass.saturating_add(1);
    vehicle.last_batt_compass = Some(update_batt_compass(vehicle.batt_compass));
}

fn task_loop_rate_logging(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.loop_rate_logging = vehicle.ticks.loop_rate_logging.saturating_add(1);
    vehicle.last_loop_rate_logging = Some(loop_rate_logging(LoopRateLoggingInputs {
        log_bitmask: vehicle.log_bitmask,
        logs_attitude: vehicle.logs_attitude,
        using_rate_thread: vehicle.using_rate_thread,
    }));
}

fn task_ten_hz_logging_loop(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.ten_hz_logging_loop = vehicle.ticks.ten_hz_logging_loop.saturating_add(1);
    vehicle.last_ten_hz_logging = Some(ten_hz_logging_loop(TenHzLoggingInputs {
        log_bitmask: vehicle.log_bitmask,
        logs_attitude: vehicle.logs_attitude,
        using_rate_thread: vehicle.using_rate_thread,
        requires_position: vehicle.requires_position,
        landing_with_gps: vehicle.landing_with_gps,
        has_manual_throttle: vehicle.has_manual_throttle,
    }));
}

fn task_twentyfive_hz_logging(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.twentyfive_hz_logging = vehicle.ticks.twentyfive_hz_logging.saturating_add(1);
    vehicle.last_twentyfive_hz_logging = Some(twentyfive_hz_logging(TwentyfiveHzLoggingInputs {
        log_bitmask: vehicle.log_bitmask,
    }));
}

fn task_three_hz_loop(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.three_hz_loop = vehicle.ticks.three_hz_loop.saturating_add(1);
    vehicle.last_three_hz = Some(three_hz_loop());
}

fn task_one_hz_loop(vehicle: &mut CopterVehicleLoop) {
    vehicle.ticks.one_hz_loop = vehicle.ticks.one_hz_loop.saturating_add(1);
    vehicle.last_one_hz = Some(one_hz_loop(OneHzLoopInputs {
        log_bitmask: vehicle.log_bitmask,
        motors_armed: vehicle.motors.armed,
        using_rate_thread: vehicle.using_rate_thread,
        land_complete: vehicle.flight_mode.land_complete,
    }));
}

/// First Copter-owned FAST_TASK, in upstream table form.
#[must_use]
pub fn copter_run_rate_controller_main_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_run_rate_controller_main,
        name: "run_rate_controller_main",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// `motors_output_main` FAST_TASK row.
#[must_use]
pub fn copter_motors_output_main_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_motors_output_main,
        name: "motors_output_main",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// `read_AHRS` FAST_TASK row.
#[must_use]
pub fn copter_read_ahrs_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_read_ahrs,
        name: "read_AHRS",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// `read_inertia` FAST_TASK row.
#[must_use]
pub fn copter_read_inertia_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_read_inertia,
        name: "read_inertia",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// The first scheduled Copter callback, in upstream table form.
#[must_use]
pub fn copter_rc_loop_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_rc_loop,
        name: "rc_loop",
        rate_hz: RC_LOOP_RATE_HZ,
        max_time_micros: RC_LOOP_MAX_TIME_MICROS,
        priority: RC_LOOP_PRIORITY,
    }
}

/// `throttle_loop` scheduled row.
#[must_use]
pub fn copter_throttle_loop_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_throttle_loop,
        name: "throttle_loop",
        rate_hz: THROTTLE_LOOP_RATE_HZ,
        max_time_micros: THROTTLE_LOOP_MAX_TIME_MICROS,
        priority: THROTTLE_LOOP_PRIORITY,
    }
}

/// First Copter-owned always-on FAST_TASK leftovers, table order.
#[must_use]
pub fn copter_first_fast_tasks() -> [Task<CopterVehicleLoop>; 4] {
    [
        copter_run_rate_controller_main_task(),
        copter_motors_output_main_task(),
        copter_read_ahrs_task(),
        copter_read_inertia_task(),
    ]
}

/// `update_flight_mode` FAST_TASK row.
#[must_use]
pub fn copter_update_flight_mode_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_update_flight_mode,
        name: "update_flight_mode",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// `update_land_and_crash_detectors` FAST_TASK row.
#[must_use]
pub fn copter_update_land_and_crash_detectors_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_update_land_and_crash_detectors,
        name: "update_land_and_crash_detectors",
        rate_hz: LOOP_RATE,
        max_time_micros: 0,
        priority: FAST_TASK_PRI0,
    }
}

/// `update_batt_compass` scheduled row.
#[must_use]
pub fn copter_update_batt_compass_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_update_batt_compass,
        name: "update_batt_compass",
        rate_hz: UPDATE_BATT_COMPASS_RATE_HZ,
        max_time_micros: UPDATE_BATT_COMPASS_MAX_TIME_MICROS,
        priority: UPDATE_BATT_COMPASS_PRIORITY,
    }
}

/// First two always-on scheduled leftovers, table order.
#[must_use]
pub fn copter_first_scheduled_tasks() -> [Task<CopterVehicleLoop>; 2] {
    [copter_rc_loop_task(), copter_throttle_loop_task()]
}

/// Next Copter-owned FAST_TASK leftovers after `read_inertia`.
///
/// Table order still has `check_ekf_reset` and `update_home_from_EKF`
/// between / around these; those stay later leftovers. This pair is the
/// attitude-run and land/crash wrapper.
#[must_use]
pub fn copter_next_fast_tasks() -> [Task<CopterVehicleLoop>; 2] {
    [
        copter_update_flight_mode_task(),
        copter_update_land_and_crash_detectors_task(),
    ]
}

/// Next always-on Copter-owned scheduled leftover after `throttle_loop`.
///
/// `fence_check` is gated and `AP_GPS::update` lives on `ap-gps`.
#[must_use]
pub fn copter_next_scheduled_tasks() -> [Task<CopterVehicleLoop>; 1] {
    [copter_update_batt_compass_task()]
}

/// `loop_rate_logging` scheduled row (`LOOP_RATE`).
#[must_use]
pub fn copter_loop_rate_logging_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_loop_rate_logging,
        name: "loop_rate_logging",
        rate_hz: LOOP_RATE_LOGGING_RATE_HZ,
        max_time_micros: LOOP_RATE_LOGGING_MAX_TIME_MICROS,
        priority: LOOP_RATE_LOGGING_PRIORITY,
    }
}

/// `ten_hz_logging_loop` scheduled row.
#[must_use]
pub fn copter_ten_hz_logging_loop_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_ten_hz_logging_loop,
        name: "ten_hz_logging_loop",
        rate_hz: TEN_HZ_LOGGING_RATE_HZ,
        max_time_micros: TEN_HZ_LOGGING_MAX_TIME_MICROS,
        priority: TEN_HZ_LOGGING_PRIORITY,
    }
}

/// `twentyfive_hz_logging` scheduled row.
#[must_use]
pub fn copter_twentyfive_hz_logging_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_twentyfive_hz_logging,
        name: "twentyfive_hz_logging",
        rate_hz: TWENTYFIVE_HZ_LOGGING_RATE_HZ,
        max_time_micros: TWENTYFIVE_HZ_LOGGING_MAX_TIME_MICROS,
        priority: TWENTYFIVE_HZ_LOGGING_PRIORITY,
    }
}

/// `three_hz_loop` scheduled row.
#[must_use]
pub fn copter_three_hz_loop_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_three_hz_loop,
        name: "three_hz_loop",
        rate_hz: THREE_HZ_LOOP_RATE_HZ,
        max_time_micros: THREE_HZ_LOOP_MAX_TIME_MICROS,
        priority: THREE_HZ_LOOP_PRIORITY,
    }
}

/// `one_hz_loop` scheduled row.
#[must_use]
pub fn copter_one_hz_loop_task() -> Task<CopterVehicleLoop> {
    Task {
        function: task_one_hz_loop,
        name: "one_hz_loop",
        rate_hz: ONE_HZ_LOOP_RATE_HZ,
        max_time_micros: ONE_HZ_LOOP_MAX_TIME_MICROS,
        priority: ONE_HZ_LOOP_PRIORITY,
    }
}

/// `HAL_LOGGING_ENABLED` scheduled leftovers, table order.
#[must_use]
pub fn copter_logging_tasks() -> [Task<CopterVehicleLoop>; 3] {
    [
        copter_loop_rate_logging_task(),
        copter_ten_hz_logging_loop_task(),
        copter_twentyfive_hz_logging_task(),
    ]
}

/// Next always-on Copter-owned scheduled leftovers after the logging trio.
#[must_use]
pub fn copter_periodic_loop_tasks() -> [Task<CopterVehicleLoop>; 2] {
    [copter_three_hz_loop_task(), copter_one_hz_loop_task()]
}

/// Advance one scheduler tick and run the leftover pass.
pub fn run_scheduler_tick(
    vehicle: &mut CopterVehicleLoop,
    scheduler: &mut Scheduler<'_, CopterVehicleLoop>,
    clock: &dyn Clock,
    time_available_us: u32,
) -> RunStats {
    scheduler.tick();
    scheduler.run(vehicle, clock, time_available_us)
}
