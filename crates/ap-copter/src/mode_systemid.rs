//! `ModeSystemId` init / run leftover, upstream `ArduCopter/mode_systemid.cpp`.
//!
//! Tracked as **COP-024**. SystemID injects a chirp into an attitude,
//! mixer, or position-control axis. The chirp generator stays a later
//! leftover — `run` takes the already-computed sample. What this file
//! owns is `init` (the axis / flying / from-mode gates, optional NE/D
//! seating, and the chirp start) and `run` (stop gates, axis inject,
//! Stabilize spool leftover, and the NE / D pos-control leftover).
//!
//! # `init` is three gates, then a chirp start
//!
//! `ModeSystemId::init` never reads `ignore_checks`. It returns false
//! when `SID_AXIS` is 0 (`enabled()` is `axis != 0`), when the aircraft
//! is not flying (`!armed || !auto_armed || land_complete`), or when
//! the from-mode does not match the axis class. Attitude / mixer axes
//! (1-13) require a from-mode with manual throttle. Position-control
//! axes (14-19) require a switch from Loiter, then seat the NE and D
//! controllers the same way ModeLoiter does — max and correction get
//! the same wp_nav numbers, and each controller is initialised only
//! when it is inactive.
//!
//! On the passing path the leftover captures the current body-frame
//! feedforward flag (so `exit` can restore it), zeros `waveform_time`,
//! sets `time_const_freq` to two cycles of `SID_F_START_HZ`, and
//! starts the chirp in `SYSTEMID_STATE_TESTING`. The tradheli
//! `set_use_stab_col` write is compiled out of this multicopter port.

use crate::mode_loiter::MODE_NUMBER_LOITER;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{is_negative, is_positive, radians};
use ap_math::vector2::Vector2f;
use ap_motors::spool::{DesiredSpoolState, SpoolState};

/// `Mode::Number::SYSTEMID`.
pub const MODE_NUMBER_SYSTEMID: u8 = 25;

/// Seconds waited after the mode change before the chirp injects.
/// Upstream `SYSTEM_ID_DELAY`.
pub const SYSTEM_ID_DELAY_S: f32 = 1.0;

/// Default `SID_AXIS`. Upstream constructor / `AP_GROUPINFO` value.
pub const SYSTEMID_AXIS_DEFAULT: i8 = 0;

/// Default `SID_MAGNITUDE`. Upstream constructor value.
pub const SYSTEMID_MAGNITUDE_DEFAULT: f32 = 15.0;

/// Default `SID_F_START_HZ`.
pub const SYSTEMID_F_START_HZ_DEFAULT: f32 = 0.5;

/// Default `SID_F_STOP_HZ`.
pub const SYSTEMID_F_STOP_HZ_DEFAULT: f32 = 40.0;

/// Default `SID_T_FADE_IN`, s.
pub const SYSTEMID_T_FADE_IN_DEFAULT: f32 = 15.0;

/// Default `SID_T_REC`, s.
pub const SYSTEMID_T_REC_DEFAULT: f32 = 70.0;

/// Default `SID_T_FADE_OUT`, s.
pub const SYSTEMID_T_FADE_OUT_DEFAULT: f32 = 2.0;

/// `ModeSystemId` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemIdModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. False: the mode itself does not need GPS.
    pub requires_position: bool,
    /// `has_manual_throttle()`. True: attitude axes fly like Stabilize.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`. False: must already be flying.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
    /// `logs_attitude()`.
    pub logs_attitude: bool,
}

/// Upstream `ModeSystemId` flags.
#[must_use]
pub const fn systemid_mode_flags() -> SystemIdModeFlags {
    SystemIdModeFlags {
        mode_number: MODE_NUMBER_SYSTEMID,
        requires_position: false,
        has_manual_throttle: true,
        allows_arming: false,
        is_autopilot: false,
        logs_attitude: true,
    }
}

/// Upstream `ModeSystemId` does not override `has_user_takeoff`.
///
/// The base `Mode` leftover is `false`. SystemID cannot start on the
/// ground — `init` already requires a flying aircraft.
#[must_use]
pub const fn systemid_has_user_takeoff(_must_navigate: bool) -> bool {
    false
}

/// Upstream `ModeSystemId::enabled`.
///
/// Listing the mode is `SID_AXIS != 0`. `init` still requires a flying
/// aircraft and a matching from-mode on top.
#[must_use]
pub const fn systemid_enabled(axis: i8) -> bool {
    axis != 0
}

/// Upstream `ModeSystemId::AxisType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i8)]
pub enum SystemIdAxis {
    /// `NONE` — mode is listed as disabled.
    None = 0,
    /// `INPUT_ROLL`.
    InputRoll = 1,
    /// `INPUT_PITCH`.
    InputPitch = 2,
    /// `INPUT_YAW`.
    InputYaw = 3,
    /// `RECOVER_ROLL`.
    RecoverRoll = 4,
    /// `RECOVER_PITCH`.
    RecoverPitch = 5,
    /// `RECOVER_YAW`.
    RecoverYaw = 6,
    /// `RATE_ROLL`.
    RateRoll = 7,
    /// `RATE_PITCH`.
    RatePitch = 8,
    /// `RATE_YAW`.
    RateYaw = 9,
    /// `MIX_ROLL`.
    MixRoll = 10,
    /// `MIX_PITCH`.
    MixPitch = 11,
    /// `MIX_YAW`.
    MixYaw = 12,
    /// `MIX_THROTTLE`.
    MixThrottle = 13,
    /// `DISTURB_POS_LAT`.
    DisturbPosLat = 14,
    /// `DISTURB_POS_LONG`.
    DisturbPosLong = 15,
    /// `DISTURB_VEL_LAT`.
    DisturbVelLat = 16,
    /// `DISTURB_VEL_LONG`.
    DisturbVelLong = 17,
    /// `INPUT_VEL_LAT`.
    InputVelLat = 18,
    /// `INPUT_VEL_LONG`.
    InputVelLong = 19,
}

impl SystemIdAxis {
    /// Parse a `SID_AXIS` integer. `None` for values outside 0..=19.
    #[must_use]
    pub const fn from_i8(axis: i8) -> Option<Self> {
        match axis {
            0 => Some(Self::None),
            1 => Some(Self::InputRoll),
            2 => Some(Self::InputPitch),
            3 => Some(Self::InputYaw),
            4 => Some(Self::RecoverRoll),
            5 => Some(Self::RecoverPitch),
            6 => Some(Self::RecoverYaw),
            7 => Some(Self::RateRoll),
            8 => Some(Self::RatePitch),
            9 => Some(Self::RateYaw),
            10 => Some(Self::MixRoll),
            11 => Some(Self::MixPitch),
            12 => Some(Self::MixYaw),
            13 => Some(Self::MixThrottle),
            14 => Some(Self::DisturbPosLat),
            15 => Some(Self::DisturbPosLong),
            16 => Some(Self::DisturbVelLat),
            17 => Some(Self::DisturbVelLong),
            18 => Some(Self::InputVelLat),
            19 => Some(Self::InputVelLong),
            _ => None,
        }
    }
}

/// Upstream `ModeSystemId::is_poscontrol_axis_type`.
///
/// Axes 14-19 excite a measured or commanded NE leftover. Axes 1-13
/// excite an attitude / mixer leftover. Axis 0 is not pos-control
/// either — `init` rejects it at the `enabled()` gate first.
#[must_use]
pub const fn is_poscontrol_axis_type(axis: i8) -> bool {
    matches!(axis, 14 | 15 | 16 | 17 | 18 | 19)
}

/// `ModeSystemId::SystemIDModeState` after `init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemIdState {
    /// `SYSTEMID_STATE_STOPPED`.
    Stopped,
    /// `SYSTEMID_STATE_TESTING` — `init` always starts here.
    Testing,
}

/// Why `ModeSystemId::init` returned false.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemIdInitFail {
    /// `SID_AXIS == 0`. GCS: "No axis selected, SID_AXIS = 0".
    AxisNone,
    /// `!armed || !auto_armed || land_complete`. GCS: "Aircraft must be flying".
    NotFlying,
    /// Attitude / mixer axis entered from a mode without manual throttle.
    NeedsManualThrottle,
    /// Pos-control axis entered from a mode other than Loiter.
    NeedsLoiter,
}

/// What `ModeSystemId::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct SystemIdInitView {
    /// `SID_AXIS` / `axis.get()`.
    pub axis: i8,
    /// `copter.motors->armed()`.
    pub armed: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.flightmode->has_manual_throttle()`.
    pub from_has_manual_throttle: bool,
    /// `copter.flightmode->mode_number()`.
    pub from_mode_number: u8,
    /// `pos_control->NE_is_active()`.
    pub ne_is_active: bool,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_NE_ms()`.
    pub speed_ne_ms: f32,
    /// `wp_nav->get_wp_acceleration_mss()`.
    pub accel_ne_mss: f32,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub speed_dn_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `pos_control->get_pos_estimate_NED_m().xy().x`.
    pub pos_estimate_n_m: f32,
    /// `pos_control->get_pos_estimate_NED_m().xy().y`.
    pub pos_estimate_e_m: f32,
    /// `attitude_control->get_bf_feedforward()`.
    pub att_bf_feedforward: bool,
    /// `SID_F_START_HZ`.
    pub frequency_start: f32,
    /// `SID_F_STOP_HZ`.
    pub frequency_stop: f32,
    /// `SID_T_FADE_IN`.
    pub time_fade_in: f32,
    /// `SID_T_REC`.
    pub time_record: f32,
    /// `SID_T_FADE_OUT`.
    pub time_fade_out: f32,
    /// `SID_MAGNITUDE`.
    pub waveform_magnitude: f32,
}

impl SystemIdInitView {
    /// Flying, `SID_AXIS = INPUT_ROLL`, from a manual-throttle mode.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: SystemIdAxis::InputRoll as i8,
            armed: true,
            auto_armed: true,
            land_complete: false,
            from_has_manual_throttle: true,
            from_mode_number: 0,
            ne_is_active: true,
            d_is_active: true,
            speed_ne_ms: 5.0,
            accel_ne_mss: 1.0,
            speed_dn_ms: 1.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            pos_estimate_n_m: 0.0,
            pos_estimate_e_m: 0.0,
            att_bf_feedforward: true,
            frequency_start: SYSTEMID_F_START_HZ_DEFAULT,
            frequency_stop: SYSTEMID_F_STOP_HZ_DEFAULT,
            time_fade_in: SYSTEMID_T_FADE_IN_DEFAULT,
            time_record: SYSTEMID_T_REC_DEFAULT,
            time_fade_out: SYSTEMID_T_FADE_OUT_DEFAULT,
            waveform_magnitude: SYSTEMID_MAGNITUDE_DEFAULT,
        }
    }

    /// Flying in Loiter with a pos-control axis selected.
    #[must_use]
    pub const fn typical_poscontrol() -> Self {
        let mut view = Self::typical();
        view.axis = SystemIdAxis::DisturbPosLat as i8;
        view.from_has_manual_throttle = false;
        view.from_mode_number = MODE_NUMBER_LOITER;
        view.pos_estimate_n_m = 12.5;
        view.pos_estimate_e_m = -3.0;
        view
    }
}

/// Leftover of one `ModeSystemId::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SystemIdInit {
    /// `NE_init_controller()` — only on a passing pos-control path
    /// when the controller was inactive.
    pub init_ne_controller: bool,
    /// `D_init_controller()` — only on a passing pos-control path
    /// when the controller was inactive.
    pub init_d_controller: bool,
    /// `NE_set_max_speed_accel_m` ran.
    pub set_ne_max_speed_accel: bool,
    /// `NE_set_correction_speed_accel_m` ran, same two numbers.
    pub set_ne_correction_speed_accel: bool,
    /// `D_set_max_speed_accel_m` ran.
    pub set_d_max_speed_accel: bool,
    /// `D_set_correction_speed_accel_m` ran, same three numbers.
    pub set_d_correction_speed_accel: bool,
    /// Horizontal speed written to both NE setters. `None` unless
    /// a pos-control path passed.
    pub speed_ne_ms: Option<f32>,
    /// Horizontal accel written to both NE setters.
    pub accel_ne_mss: Option<f32>,
    /// Descent speed written to both D setters.
    pub speed_dn_ms: Option<f32>,
    /// Climb speed written to both D setters.
    pub speed_up_ms: Option<f32>,
    /// Vertical accel written to both D setters.
    pub accel_d_mss: Option<f32>,
    /// `target_pos_ne_m` after `init`. `None` unless a pos-control
    /// path passed.
    pub target_pos_ne_m: Option<(f32, f32)>,
    /// Captured `attitude_control->get_bf_feedforward()`.
    pub att_bf_feedforward: Option<bool>,
    /// `waveform_time` after `init`. `Some(0.0)` on the passing path.
    pub waveform_time: Option<f32>,
    /// `2.0 / frequency_start` — two cycles at the start frequency.
    pub time_const_freq: Option<f32>,
    /// `systemid_state` after `init`. Always [`SystemIdState::Testing`]
    /// on the passing path.
    pub state: Option<SystemIdState>,
    /// `log_subsample` after `init`. `Some(0)` on the passing path.
    pub log_subsample: Option<i8>,
    /// `chirp_input.init(time_record, f_start, f_stop, fade_in, fade_out,
    /// time_const_freq)` ran.
    pub chirp_init: bool,
    /// Chirp record length handed to `chirp_input.init`.
    pub chirp_time_record: Option<f32>,
    /// Chirp start frequency handed to `chirp_input.init`.
    pub chirp_frequency_start: Option<f32>,
    /// Chirp stop frequency handed to `chirp_input.init`.
    pub chirp_frequency_stop: Option<f32>,
    /// Chirp fade-in handed to `chirp_input.init`.
    pub chirp_time_fade_in: Option<f32>,
    /// Chirp fade-out handed to `chirp_input.init`.
    pub chirp_time_fade_out: Option<f32>,
    /// Gate that fired, if any. `None` on the passing path.
    pub fail: Option<SystemIdInitFail>,
    /// `true` only when every gate passed. `ignore_checks` cannot
    /// bypass any of them.
    pub ok: bool,
}

fn failed(fail: SystemIdInitFail) -> SystemIdInit {
    SystemIdInit {
        init_ne_controller: false,
        init_d_controller: false,
        set_ne_max_speed_accel: false,
        set_ne_correction_speed_accel: false,
        set_d_max_speed_accel: false,
        set_d_correction_speed_accel: false,
        speed_ne_ms: None,
        accel_ne_mss: None,
        speed_dn_ms: None,
        speed_up_ms: None,
        accel_d_mss: None,
        target_pos_ne_m: None,
        att_bf_feedforward: None,
        waveform_time: None,
        time_const_freq: None,
        state: None,
        log_subsample: None,
        chirp_init: false,
        chirp_time_record: None,
        chirp_frequency_start: None,
        chirp_frequency_stop: None,
        chirp_time_fade_in: None,
        chirp_time_fade_out: None,
        fail: Some(fail),
        ok: false,
    }
}

/// Upstream `ModeSystemId::init`. `ignore_checks` is unread.
///
/// A zero axis, a landed / disarmed aircraft, or a from-mode that
/// does not match the axis class fails before any NE / D / chirp
/// leftover is written. The passing attitude path captures the
/// feedforward flag and starts the chirp. The passing pos-control
/// path seats NE and D first, then does the same chirp start.
#[must_use]
pub fn systemid_init(_ignore_checks: bool, view: &SystemIdInitView) -> SystemIdInit {
    if !systemid_enabled(view.axis) {
        return failed(SystemIdInitFail::AxisNone);
    }
    if !view.armed || !view.auto_armed || view.land_complete {
        return failed(SystemIdInitFail::NotFlying);
    }

    let mut init_ne_controller = false;
    let mut init_d_controller = false;
    let mut set_ne_max_speed_accel = false;
    let mut set_ne_correction_speed_accel = false;
    let mut set_d_max_speed_accel = false;
    let mut set_d_correction_speed_accel = false;
    let mut speed_ne_ms = None;
    let mut accel_ne_mss = None;
    let mut speed_dn_ms = None;
    let mut speed_up_ms = None;
    let mut accel_d_mss = None;
    let mut target_pos_ne_m = None;

    if !is_poscontrol_axis_type(view.axis) {
        if !view.from_has_manual_throttle {
            return failed(SystemIdInitFail::NeedsManualThrottle);
        }
    } else {
        if view.from_mode_number != MODE_NUMBER_LOITER {
            return failed(SystemIdInitFail::NeedsLoiter);
        }
        set_ne_max_speed_accel = true;
        set_ne_correction_speed_accel = true;
        speed_ne_ms = Some(view.speed_ne_ms);
        accel_ne_mss = Some(view.accel_ne_mss);
        init_ne_controller = !view.ne_is_active;
        set_d_max_speed_accel = true;
        set_d_correction_speed_accel = true;
        speed_dn_ms = Some(view.speed_dn_ms);
        speed_up_ms = Some(view.speed_up_ms);
        accel_d_mss = Some(view.accel_d_mss);
        init_d_controller = !view.d_is_active;
        target_pos_ne_m = Some((view.pos_estimate_n_m, view.pos_estimate_e_m));
    }

    let time_const_freq = 2.0 / view.frequency_start;
    SystemIdInit {
        init_ne_controller,
        init_d_controller,
        set_ne_max_speed_accel,
        set_ne_correction_speed_accel,
        set_d_max_speed_accel,
        set_d_correction_speed_accel,
        speed_ne_ms,
        accel_ne_mss,
        speed_dn_ms,
        speed_up_ms,
        accel_d_mss,
        target_pos_ne_m,
        att_bf_feedforward: Some(view.att_bf_feedforward),
        waveform_time: Some(0.0),
        time_const_freq: Some(time_const_freq),
        state: Some(SystemIdState::Testing),
        log_subsample: Some(0),
        chirp_init: true,
        chirp_time_record: Some(view.time_record),
        chirp_frequency_start: Some(view.frequency_start),
        chirp_frequency_stop: Some(view.frequency_stop),
        chirp_time_fade_in: Some(view.time_fade_in),
        chirp_time_fade_out: Some(view.time_fade_out),
        fail: None,
        ok: true,
    }
}

/// Why `ModeSystemId::run` moved `systemid_state` to STOPPED.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemIdStopReason {
    /// `SID_F_*` / fade / record leftover failed the positivity check.
    ParameterError,
    /// `copter.ap.land_complete`.
    Landed,
    /// `lean_angle_rad() > lean_angle_max_rad()`.
    LeanLimit,
    /// Waveform time past delay + fade-in + const-freq + record + fade-out.
    Finished,
    /// `SID_AXIS == 0` while TESTING.
    AxisNone,
}

/// Where the chirp sample was injected this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemIdInject {
    /// STOPPED, or TESTING that stopped before the axis switch.
    None,
    /// `target_roll_rad += radians(sample)`.
    InputRoll,
    /// `target_pitch_rad += radians(sample)`.
    InputPitch,
    /// `target_yaw_rate_rads += radians(sample)`.
    InputYaw,
    /// Same as InputRoll, plus `bf_feedforward(false)`.
    RecoverRoll,
    /// Same as InputPitch, plus `bf_feedforward(false)`.
    RecoverPitch,
    /// Same as InputYaw, plus `bf_feedforward(false)`.
    RecoverYaw,
    /// `rate_bf_roll_sysid_rads`.
    RateRoll,
    /// `rate_bf_pitch_sysid_rads`.
    RatePitch,
    /// `rate_bf_yaw_sysid_rads`.
    RateYaw,
    /// `actuator_roll_sysid`.
    MixRoll,
    /// `actuator_pitch_sysid`.
    MixPitch,
    /// `actuator_yaw_sysid`.
    MixYaw,
    /// `pilot_throttle_scaled += sample`.
    MixThrottle,
    /// `set_disturb_pos_NE_m` lateral.
    DisturbPosLat,
    /// `set_disturb_pos_NE_m` longitudinal.
    DisturbPosLong,
    /// `set_disturb_vel_NE_ms` lateral.
    DisturbVelLat,
    /// `set_disturb_vel_NE_ms` longitudinal.
    DisturbVelLong,
    /// Body-frame lateral velocity command, rotated to NE.
    InputVelLat,
    /// Body-frame longitudinal velocity command, rotated to NE.
    InputVelLong,
}

/// What `ModeSystemId::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct SystemIdRunView {
    /// `SID_AXIS` / `axis.get()`.
    pub axis: i8,
    /// `systemid_state` before this tick.
    pub state: SystemIdState,
    /// `waveform_time` before `+= G_Dt`.
    pub waveform_time: f32,
    /// `G_Dt`.
    pub dt: f32,
    /// Already-computed `chirp_input.update(time - delay, magnitude)`.
    pub waveform_sample: f32,
    /// Already-computed `chirp_input.get_frequency_rads()`.
    pub waveform_freq_rads: f32,
    /// `SID_MAGNITUDE`. Recorded as the chirp-update leftover argument.
    pub waveform_magnitude: f32,
    /// `SID_F_START_HZ`.
    pub frequency_start: f32,
    /// `SID_F_STOP_HZ`.
    pub frequency_stop: f32,
    /// `SID_T_FADE_IN`.
    pub time_fade_in: f32,
    /// `SID_T_REC`.
    pub time_record: f32,
    /// `SID_T_FADE_OUT`.
    pub time_fade_out: f32,
    /// `time_const_freq` after `init` (`2 / f_start`).
    pub time_const_freq: f32,
    /// Captured `attitude_control->get_bf_feedforward()`.
    pub att_bf_feedforward: bool,
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`.
    pub lean_angle_max_rad: f32,
    /// `attitude_control->lean_angle_rad()`.
    pub lean_angle_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// Already-converted `get_pilot_desired_throttle()`.
    pub pilot_throttle: f32,
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.throttle_zero`.
    pub throttle_zero: bool,
    /// `copter.is_tradheli()`. This multicopter leftover is false.
    pub tradheli: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `motors->init_targets_on_arming()`. True for multicopter.
    pub init_targets_on_arming: bool,
    /// `motors->limit.throttle_lower`.
    pub throttle_lower_limited: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.land_complete_maybe`.
    pub land_complete_maybe: bool,
    /// `attitude_control->get_att_target_euler_rad().z`.
    pub att_target_yaw_rad: f32,
    /// `target_pos_ne_m` before integrating `input_vel * dt`.
    pub target_pos_ne_m: (f32, f32),
    /// `input_vel_last_ne_ms` before this tick.
    pub input_vel_last_ne_ms: (f32, f32),
    /// `log_subsample` before this tick.
    pub log_subsample: i8,
    /// `MASK_LOG_ATTITUDE_FAST`.
    pub log_fast: bool,
    /// `MASK_LOG_ATTITUDE_MED`.
    pub log_med: bool,
}

impl SystemIdRunView {
    /// Flying, TESTING, INPUT_ROLL, first tick after `init`.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: SystemIdAxis::InputRoll as i8,
            state: SystemIdState::Testing,
            waveform_time: 0.0,
            dt: 0.0025,
            waveform_sample: 0.0,
            waveform_freq_rads: 0.0,
            waveform_magnitude: SYSTEMID_MAGNITUDE_DEFAULT,
            frequency_start: SYSTEMID_F_START_HZ_DEFAULT,
            frequency_stop: SYSTEMID_F_STOP_HZ_DEFAULT,
            time_fade_in: SYSTEMID_T_FADE_IN_DEFAULT,
            time_record: SYSTEMID_T_REC_DEFAULT,
            time_fade_out: SYSTEMID_T_FADE_OUT_DEFAULT,
            time_const_freq: 2.0 / SYSTEMID_F_START_HZ_DEFAULT,
            att_bf_feedforward: true,
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            lean_angle_max_rad: 0.523_598_8,
            lean_angle_rad: 0.0,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            pilot_throttle: 0.5,
            armed: true,
            throttle_zero: false,
            tradheli: false,
            spool_state: SpoolState::ThrottleUnlimited,
            init_targets_on_arming: true,
            throttle_lower_limited: false,
            land_complete: false,
            land_complete_maybe: false,
            att_target_yaw_rad: 0.0,
            target_pos_ne_m: (0.0, 0.0),
            input_vel_last_ne_ms: (0.0, 0.0),
            log_subsample: 0,
            log_fast: false,
            log_med: false,
        }
    }

    /// Flying in a pos-control axis, TESTING.
    #[must_use]
    pub const fn typical_poscontrol() -> Self {
        let mut view = Self::typical();
        view.axis = SystemIdAxis::InputVelLong as i8;
        view.target_pos_ne_m = (12.5, -3.0);
        view
    }
}

/// Leftover of one `ModeSystemId::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SystemIdRun {
    /// `systemid_state` after the stop gates and the axis switch.
    pub state: SystemIdState,
    /// Why TESTING became STOPPED, if it did.
    pub stop_reason: Option<SystemIdStopReason>,
    /// `waveform_time` after `+= G_Dt`.
    pub waveform_time: f32,
    /// Chirp sample this tick (already computed; recorded as the leftover).
    pub waveform_sample: f32,
    /// Chirp frequency this tick.
    pub waveform_freq_rads: f32,
    /// Always true: `chirp_input.update(time - SYSTEM_ID_DELAY, magnitude)`.
    pub chirp_update: bool,
    /// Always true: `chirp_input.get_frequency_rads()`.
    pub chirp_frequency: bool,
    /// Time handed to `chirp_input.update`.
    pub chirp_time: f32,
    /// Where the sample was injected.
    pub inject: SystemIdInject,
    /// `bf_feedforward` write. STOPPED restores the captured flag;
    /// Recover axes write `false`; others leave it alone.
    pub bf_feedforward: Option<bool>,
    /// Roll demand after the optional chirp add, rad.
    pub target_roll_rad: f32,
    /// Pitch demand after the optional chirp add, rad.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand after the optional chirp add, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Throttle after the optional MIX_THROTTLE add.
    pub pilot_throttle: f32,
    /// Desired spool on the attitude path. `None` on pos-control axes.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Rate-controller I-term reset on the attitude path.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` on the attitude path.
    pub reset_yaw_target_and_rate: bool,
    /// `set_land_complete(false)` on unlimited above the lower limit.
    pub clear_land_complete: bool,
    /// Always true on the attitude path: Euler roll/pitch + yaw-rate.
    pub input_euler_angle: bool,
    /// Always true on the attitude path: `set_throttle_out`.
    pub set_throttle_out: bool,
    /// `NE_soften_for_landing` on the pos-control path.
    pub soften_for_landing: bool,
    /// `target_pos_ne_m` after integrating `input_vel * dt`.
    pub target_pos_ne_m: (f32, f32),
    /// Commanded NE velocity after the body-to-earth rotate.
    pub input_vel_ne_ms: (f32, f32),
    /// `input_vel_last_ne_ms` after this tick.
    pub input_vel_last_ne_ms: (f32, f32),
    /// Accel from the velocity delta when `G_Dt` is positive.
    pub accel_ne_mss: (f32, f32),
    /// `set_pos_vel_accel_NE_m` ran.
    pub set_pos_vel_accel: bool,
    /// `set_disturb_pos_NE_m` leftover, earth-frame.
    pub disturb_pos_ne_m: Option<(f32, f32)>,
    /// `set_disturb_vel_NE_ms` leftover, earth-frame.
    pub disturb_vel_ne_ms: Option<(f32, f32)>,
    /// `NE_update_controller` ran.
    pub update_ne_controller: bool,
    /// `input_thrust_vector_rate_heading_rads` ran (`slew_yaw = false`).
    pub input_thrust_vector: bool,
    /// `D_set_pos_target_from_climb_rate_ms` ran (climb rate is 0 here).
    pub set_climb_rate: bool,
    /// `D_update_controller` ran.
    pub update_d_controller: bool,
    /// `log_data()` ran because `log_subsample <= 0`.
    pub logged: bool,
    /// `log_subsample` after the decrement.
    pub log_subsample: i8,
}

/// Upstream `ModeSystemId` parameter-error leftover.
///
/// TESTING is refused when the start/stop frequencies or the record
/// length are not strictly positive, when either fade is negative, or
/// when the record is not longer than the two-cycle constant-freq pad.
#[must_use]
pub fn systemid_parameter_error(view: &SystemIdRunView) -> bool {
    !is_positive(view.frequency_start)
        || !is_positive(view.frequency_stop)
        || is_negative(view.time_fade_in)
        || !is_positive(view.time_record)
        || is_negative(view.time_fade_out)
        || view.time_record <= view.time_const_freq
}

fn rotate_ne(body: (f32, f32), yaw_rad: f32) -> (f32, f32) {
    let mut v = Vector2f::new(body.0, body.1);
    v.rotate(yaw_rad);
    (v.x, v.y)
}

/// Upstream `ModeSystemId::run`.
///
/// The chirp generator stays a later leftover — this takes the already-
/// computed sample, the way Sport takes an already-converted climb rate.
/// Attitude axes convert the pilot, run the Stabilize spool leftover,
/// then inject. Pos-control axes skip the stick conversion, integrate
/// the commanded NE velocity, and run the NE / D controllers. Every
/// tick advances `waveform_time` and the log subsample, even after a
/// stop gate has already flipped the state.
#[must_use]
pub fn systemid_run(view: &SystemIdRunView) -> SystemIdRun {
    let poscontrol = is_poscontrol_axis_type(view.axis);

    let mut target_roll_rad = 0.0;
    let mut target_pitch_rad = 0.0;
    let mut target_yaw_rate_rads = 0.0;
    let mut pilot_throttle = 0.0;
    let mut desired_spool = None;
    let mut reset_rate_i = RateIReset::None;
    let mut reset_yaw_target_and_rate = false;
    let mut clear_land_complete = false;
    let mut input_euler_angle = false;
    let mut set_throttle_out = false;

    if !poscontrol {
        let (roll, pitch) = pilot_desired_lean_angles_rad(
            view.roll_in_norm,
            view.pitch_in_norm,
            view.lean_angle_max_rad,
            view.lean_angle_max_rad,
            view.has_valid_input,
        );
        target_roll_rad = roll;
        target_pitch_rad = pitch;
        target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
            view.yaw_in_norm,
            view.yaw_rate_degs,
            view.yaw_expo,
            view.has_valid_input,
        );
        desired_spool = Some(if !view.armed {
            DesiredSpoolState::ShutDown
        } else if view.throttle_zero && !view.tradheli {
            DesiredSpoolState::GroundIdle
        } else {
            DesiredSpoolState::ThrottleUnlimited
        });
        match view.spool_state {
            SpoolState::ShutDown => {
                reset_yaw_target_and_rate = true;
                reset_rate_i = RateIReset::Hard;
            }
            SpoolState::GroundIdle => {
                if view.init_targets_on_arming {
                    reset_yaw_target_and_rate = true;
                    reset_rate_i = RateIReset::Smooth;
                }
            }
            SpoolState::ThrottleUnlimited => {
                clear_land_complete = !view.throttle_lower_limited;
            }
            SpoolState::SpoolingUp | SpoolState::SpoolingDown => {}
        }
        pilot_throttle = view.pilot_throttle;
    }

    let mut state = view.state;
    let mut stop_reason = None;
    if state == SystemIdState::Testing && systemid_parameter_error(view) {
        state = SystemIdState::Stopped;
        stop_reason = Some(SystemIdStopReason::ParameterError);
    }

    let waveform_time = view.waveform_time + view.dt;
    let chirp_time = waveform_time - SYSTEM_ID_DELAY_S;
    let waveform_sample = view.waveform_sample;
    let waveform_freq_rads = view.waveform_freq_rads;

    let mut inject = SystemIdInject::None;
    let mut bf_feedforward = None;
    let mut input_vel_ne_ms = (0.0, 0.0);
    let mut disturb_pos_ne_m = None;
    let mut disturb_vel_ne_ms = None;

    match state {
        SystemIdState::Stopped => {
            bf_feedforward = Some(view.att_bf_feedforward);
        }
        SystemIdState::Testing => {
            if view.land_complete {
                state = SystemIdState::Stopped;
                stop_reason = Some(SystemIdStopReason::Landed);
            } else if view.lean_angle_rad > view.lean_angle_max_rad {
                state = SystemIdState::Stopped;
                stop_reason = Some(SystemIdStopReason::LeanLimit);
            } else if waveform_time
                > SYSTEM_ID_DELAY_S
                    + view.time_fade_in
                    + view.time_const_freq
                    + view.time_record
                    + view.time_fade_out
            {
                state = SystemIdState::Stopped;
                stop_reason = Some(SystemIdStopReason::Finished);
            } else {
                match SystemIdAxis::from_i8(view.axis) {
                    Some(SystemIdAxis::None) | None => {
                        state = SystemIdState::Stopped;
                        stop_reason = Some(SystemIdStopReason::AxisNone);
                    }
                    Some(SystemIdAxis::InputRoll) => {
                        target_roll_rad += radians(waveform_sample);
                        inject = SystemIdInject::InputRoll;
                    }
                    Some(SystemIdAxis::InputPitch) => {
                        target_pitch_rad += radians(waveform_sample);
                        inject = SystemIdInject::InputPitch;
                    }
                    Some(SystemIdAxis::InputYaw) => {
                        target_yaw_rate_rads += radians(waveform_sample);
                        inject = SystemIdInject::InputYaw;
                    }
                    Some(SystemIdAxis::RecoverRoll) => {
                        target_roll_rad += radians(waveform_sample);
                        bf_feedforward = Some(false);
                        inject = SystemIdInject::RecoverRoll;
                    }
                    Some(SystemIdAxis::RecoverPitch) => {
                        target_pitch_rad += radians(waveform_sample);
                        bf_feedforward = Some(false);
                        inject = SystemIdInject::RecoverPitch;
                    }
                    Some(SystemIdAxis::RecoverYaw) => {
                        target_yaw_rate_rads += radians(waveform_sample);
                        bf_feedforward = Some(false);
                        inject = SystemIdInject::RecoverYaw;
                    }
                    Some(SystemIdAxis::RateRoll) => inject = SystemIdInject::RateRoll,
                    Some(SystemIdAxis::RatePitch) => inject = SystemIdInject::RatePitch,
                    Some(SystemIdAxis::RateYaw) => inject = SystemIdInject::RateYaw,
                    Some(SystemIdAxis::MixRoll) => inject = SystemIdInject::MixRoll,
                    Some(SystemIdAxis::MixPitch) => inject = SystemIdInject::MixPitch,
                    Some(SystemIdAxis::MixYaw) => inject = SystemIdInject::MixYaw,
                    Some(SystemIdAxis::MixThrottle) => {
                        pilot_throttle += waveform_sample;
                        inject = SystemIdInject::MixThrottle;
                    }
                    Some(SystemIdAxis::DisturbPosLat) => {
                        disturb_pos_ne_m =
                            Some(rotate_ne((0.0, waveform_sample), view.att_target_yaw_rad));
                        inject = SystemIdInject::DisturbPosLat;
                    }
                    Some(SystemIdAxis::DisturbPosLong) => {
                        disturb_pos_ne_m =
                            Some(rotate_ne((waveform_sample, 0.0), view.att_target_yaw_rad));
                        inject = SystemIdInject::DisturbPosLong;
                    }
                    Some(SystemIdAxis::DisturbVelLat) => {
                        disturb_vel_ne_ms =
                            Some(rotate_ne((0.0, waveform_sample), view.att_target_yaw_rad));
                        inject = SystemIdInject::DisturbVelLat;
                    }
                    Some(SystemIdAxis::DisturbVelLong) => {
                        disturb_vel_ne_ms =
                            Some(rotate_ne((waveform_sample, 0.0), view.att_target_yaw_rad));
                        inject = SystemIdInject::DisturbVelLong;
                    }
                    Some(SystemIdAxis::InputVelLat) => {
                        input_vel_ne_ms =
                            rotate_ne((0.0, waveform_sample), view.att_target_yaw_rad);
                        inject = SystemIdInject::InputVelLat;
                    }
                    Some(SystemIdAxis::InputVelLong) => {
                        input_vel_ne_ms =
                            rotate_ne((waveform_sample, 0.0), view.att_target_yaw_rad);
                        inject = SystemIdInject::InputVelLong;
                    }
                }
            }
        }
    }

    let mut soften_for_landing = false;
    let mut target_pos_ne_m = view.target_pos_ne_m;
    let mut input_vel_last_ne_ms = view.input_vel_last_ne_ms;
    let mut accel_ne_mss = (0.0, 0.0);
    let mut set_pos_vel_accel = false;
    let mut update_ne_controller = false;
    let mut input_thrust_vector = false;
    let mut set_climb_rate = false;
    let mut update_d_controller = false;

    if !poscontrol {
        input_euler_angle = true;
        set_throttle_out = true;
    } else {
        soften_for_landing = view.land_complete_maybe;
        target_pos_ne_m = (
            view.target_pos_ne_m.0 + input_vel_ne_ms.0 * view.dt,
            view.target_pos_ne_m.1 + input_vel_ne_ms.1 * view.dt,
        );
        if is_positive(view.dt) {
            accel_ne_mss = (
                (input_vel_ne_ms.0 - view.input_vel_last_ne_ms.0) / view.dt,
                (input_vel_ne_ms.1 - view.input_vel_last_ne_ms.1) / view.dt,
            );
            input_vel_last_ne_ms = input_vel_ne_ms;
        }
        set_pos_vel_accel = true;
        update_ne_controller = true;
        input_thrust_vector = true;
        set_climb_rate = true;
        update_d_controller = true;
    }

    let logged = view.log_subsample <= 0;
    let mut log_subsample = if logged {
        if view.log_fast && view.log_med {
            1
        } else if view.log_fast {
            2
        } else if view.log_med {
            4
        } else {
            8
        }
    } else {
        view.log_subsample
    };
    log_subsample -= 1;

    SystemIdRun {
        state,
        stop_reason,
        waveform_time,
        waveform_sample,
        waveform_freq_rads,
        chirp_update: true,
        chirp_frequency: true,
        chirp_time,
        inject,
        bf_feedforward,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        pilot_throttle,
        desired_spool,
        reset_rate_i,
        reset_yaw_target_and_rate,
        clear_land_complete,
        input_euler_angle,
        set_throttle_out,
        soften_for_landing,
        target_pos_ne_m,
        input_vel_ne_ms,
        input_vel_last_ne_ms,
        accel_ne_mss,
        set_pos_vel_accel,
        disturb_pos_ne_m,
        disturb_vel_ne_ms,
        update_ne_controller,
        input_thrust_vector,
        set_climb_rate,
        update_d_controller,
        logged,
        log_subsample,
    }
}
