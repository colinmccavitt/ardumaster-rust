//! `ModeFlowHold` init leftover, upstream `ArduCopter/mode_flowhold.cpp`.
//!
//! Tracked as **COP-024**. FlowHold is AltHold's vertical machine with an
//! optical-flow PI holding the horizontal — no GPS, no rangefinder. The
//! PI / filter / height-estimate leftovers stay for a later slice. What
//! this file owns is `init`: the optflow gate, the D seating, and the
//! reset of the flow filter / I-term / height offset.
//!
//! # `init` is an optflow gate, then AltHold's D start
//!
//! `ModeFlowHold::init` does not read `ignore_checks`. It returns false
//! when `optflow.enabled()` is false **or** `optflow.healthy()` is
//! false — a GCS that can list the mode (`enabled()` is only the first
//! of those) still cannot enter it on a bad sensor. On the passing
//! path it writes the same pilot speed / accel limits to both the max
//! and the correction setters, then inits the vertical position
//! controller only when it is inactive.
//!
//! The flow filter cutoff and PI `dt` are seeded from the scheduler
//! loop rate. `quality_filtered` and `height_offset_m` start at zero;
//! `last_ins_height_m` is the current Up estimate; the I-term is
//! reset and `limited` is cleared so a previous hold cannot wind up
//! the first tick.

/// `Mode::Number::FLOWHOLD`.
pub const MODE_NUMBER_FLOWHOLD: u8 = 22;

/// Minimum assumed height, m. Upstream `ModeFlowHold::height_min_m`.
pub const FLOWHOLD_HEIGHT_MIN_M: f32 = 0.1;

/// Maximum scaling height, m. Upstream `ModeFlowHold::height_max`.
pub const FLOWHOLD_HEIGHT_MAX_M: f32 = 3.0;

/// Default `FHLD_FLOW_MAX`. Upstream `flow_max` constructor value.
pub const FLOWHOLD_FLOW_MAX_DEFAULT: f32 = 0.6;

/// Default `FHLD_FILT_HZ`. Upstream `flow_filter_hz` constructor value.
pub const FLOWHOLD_FILTER_HZ_DEFAULT: f32 = 5.0;

/// Default `FHLD_QUAL_MIN`. Upstream `flow_min_quality` constructor value.
pub const FLOWHOLD_QUAL_MIN_DEFAULT: i8 = 10;

/// Default `FHLD_BRAKE_RATE`, deg/s. Upstream `brake_rate_dps` constructor.
pub const FLOWHOLD_BRAKE_RATE_DPS_DEFAULT: i8 = 8;

/// `ModeFlowHold` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlowHoldModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. FlowHold uses optical flow, not GPS.
    pub requires_position: bool,
    /// `has_manual_throttle()`. False: the D controller owns throttle.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`.
    pub allows_arming: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
    /// `allows_flip()`.
    pub allows_flip: bool,
}

/// Upstream `ModeFlowHold` flags.
#[must_use]
pub const fn flowhold_mode_flags() -> FlowHoldModeFlags {
    FlowHoldModeFlags {
        mode_number: MODE_NUMBER_FLOWHOLD,
        requires_position: false,
        has_manual_throttle: false,
        allows_arming: true,
        is_autopilot: false,
        allows_flip: true,
    }
}

/// Upstream `ModeFlowHold::has_user_takeoff`.
///
/// FlowHold can climb in place. A caller that needs the takeoff to
/// navigate (`must_navigate`) is refused.
#[must_use]
pub const fn flowhold_has_user_takeoff(must_navigate: bool) -> bool {
    !must_navigate
}

/// Upstream `ModeFlowHold::enabled`.
///
/// This is `copter.optflow.enabled()` only. `init` still requires
/// `optflow.healthy()` on top — listing the mode is not the same as
/// being allowed to enter it.
#[must_use]
pub const fn flowhold_enabled(optflow_enabled: bool) -> bool {
    optflow_enabled
}

/// What `ModeFlowHold::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct FlowHoldInitView {
    /// `copter.optflow.enabled()`.
    pub optflow_enabled: bool,
    /// `copter.optflow.healthy()`.
    pub optflow_healthy: bool,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `copter.scheduler.get_loop_rate_hz()`.
    pub loop_rate_hz: f32,
    /// `flow_filter_hz.get()`.
    pub flow_filter_hz: f32,
    /// `pos_control->get_pos_estimate_U_m()`.
    pub pos_estimate_u_m: f32,
}

impl FlowHoldInitView {
    /// Optflow healthy, D already running, default filter / loop rate.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            optflow_enabled: true,
            optflow_healthy: true,
            d_is_active: true,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            loop_rate_hz: 400.0,
            flow_filter_hz: FLOWHOLD_FILTER_HZ_DEFAULT,
            pos_estimate_u_m: 0.0,
        }
    }
}

/// Leftover of one `ModeFlowHold::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowHoldInit {
    /// `D_init_controller()` — only when the controller was inactive
    /// **and** the optflow gate passed.
    pub init_d_controller: bool,
    /// Speed written to both limit setters. `None` on the failing path.
    pub speed_dn_ms: Option<f32>,
    /// Climb speed written to both limit setters. `None` on the failing path.
    pub speed_up_ms: Option<f32>,
    /// Vertical accel written to both limit setters. `None` on the failing path.
    pub accel_d_mss: Option<f32>,
    /// `D_set_max_speed_accel_m` ran.
    pub set_max_speed_accel: bool,
    /// `D_set_correction_speed_accel_m` ran, same three numbers.
    pub set_correction_speed_accel: bool,
    /// `flow_filter.set_cutoff_frequency(loop_rate, flow_filter_hz)` ran.
    pub set_filter_cutoff: bool,
    /// Cutoff handed to the filter. `None` on the failing path.
    pub flow_filter_hz: Option<f32>,
    /// `quality_filtered` after `init`. `Some(0.0)` on the passing path.
    pub quality_filtered: Option<f32>,
    /// `flow_pi_xy.reset_I()` ran.
    pub reset_i: bool,
    /// `limited` after `init`. `Some(false)` on the passing path.
    pub limited: Option<bool>,
    /// `flow_pi_xy.set_dt(1 / loop_rate)` ran.
    pub set_dt: bool,
    /// `dt` handed to the PI. `None` on the failing path.
    pub dt: Option<f32>,
    /// `last_ins_height_m` after `init`. `None` on the failing path.
    pub last_ins_height_m: Option<f32>,
    /// `height_offset_m` after `init`. `Some(0.0)` on the passing path.
    pub height_offset_m: Option<f32>,
    /// `true` only when optflow is enabled **and** healthy.
    /// `ignore_checks` cannot bypass the gate.
    pub ok: bool,
}

/// Upstream `ModeFlowHold::init`. `ignore_checks` is unread.
///
/// A disabled or unhealthy optical-flow sensor fails before any D /
/// filter leftover is written. The passing path seats the D limits,
/// optionally inits the D controller, then resets the flow filter,
/// I-term, quality, limited flag, PI `dt`, and height offset.
#[must_use]
pub fn flowhold_init(_ignore_checks: bool, view: &FlowHoldInitView) -> FlowHoldInit {
    if !view.optflow_enabled || !view.optflow_healthy {
        return FlowHoldInit {
            init_d_controller: false,
            speed_dn_ms: None,
            speed_up_ms: None,
            accel_d_mss: None,
            set_max_speed_accel: false,
            set_correction_speed_accel: false,
            set_filter_cutoff: false,
            flow_filter_hz: None,
            quality_filtered: None,
            reset_i: false,
            limited: None,
            set_dt: false,
            dt: None,
            last_ins_height_m: None,
            height_offset_m: None,
            ok: false,
        };
    }

    FlowHoldInit {
        init_d_controller: !view.d_is_active,
        speed_dn_ms: Some(view.speed_dn_ms),
        speed_up_ms: Some(view.speed_up_ms),
        accel_d_mss: Some(view.accel_d_mss),
        set_max_speed_accel: true,
        set_correction_speed_accel: true,
        set_filter_cutoff: true,
        flow_filter_hz: Some(view.flow_filter_hz),
        quality_filtered: Some(0.0),
        reset_i: true,
        limited: Some(false),
        set_dt: true,
        dt: Some(1.0 / view.loop_rate_hz),
        last_ins_height_m: Some(view.pos_estimate_u_m),
        height_offset_m: Some(0.0),
        ok: true,
    }
}
