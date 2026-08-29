//! `ModeSystemId` init leftover, upstream `ArduCopter/mode_systemid.cpp`.
//!
//! Tracked as **COP-024**. SystemID injects a chirp into an attitude,
//! mixer, or position-control axis. The chirp generator and `run`
//! leftovers stay for a later slice. What this file owns is `init`:
//! the axis / flying / from-mode gates, the optional NE/D seating for
//! axes 14-19, and the waveform / chirp start leftover.
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
