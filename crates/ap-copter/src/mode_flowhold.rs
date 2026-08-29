//! `ModeFlowHold` init / run leftover, upstream `ArduCopter/mode_flowhold.cpp`.
//!
//! Tracked as **COP-024**. FlowHold is AltHold's vertical machine with an
//! optical-flow PI holding the horizontal — no GPS, no rangefinder. The
//! PI / height-estimate leftovers stay later. What this file owns is
//! `init` (the optflow gate, D seating, and filter / I-term reset) and
//! `run` (the quality filter, the 3 s arm gate, the AltHold machine,
//! and the lean-max clamp after the flow add).
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

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_althold::AltHoldVertical;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::{constrain_value, is_equal};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

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

/// Seconds the quality filter uses as its complementary weight.
/// Upstream `filter_constant = 0.95`.
pub const FLOWHOLD_QUALITY_FILTER: f32 = 0.95;

/// Milliseconds after arming before flow-to-angle is allowed.
/// Upstream `AP_HAL::millis() - copter.arm_time_ms > 3000`.
pub const FLOWHOLD_ARM_DELAY_MS: u32 = 3_000;

/// Pilot / vehicle view `ModeFlowHold::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct FlowHoldRunView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `channel_roll->get_control_in()`. Used only for the stick-input gate.
    pub roll_control_in: i16,
    /// `channel_pitch->get_control_in()`.
    pub pitch_control_in: i16,
    /// `rc().has_valid_input()`. Consulted only for yaw.
    pub has_valid_input: bool,
    /// `attitude_control->lean_angle_max_rad()`.
    pub lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// Already-converted climb rate, m/s, up positive.
    pub target_climb_rate_ms: f32,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`. Written every run tick.
    pub accel_d_mss: f32,
    /// `motors->armed()`.
    pub armed: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `takeoff.running()`.
    pub takeoff_running: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.using_interlock`.
    pub using_interlock: bool,
    /// `g2.pilot_takeoff_alt_m`.
    pub takeoff_alt_m: f32,
    /// `copter.optflow.healthy()`.
    pub optflow_healthy: bool,
    /// `copter.optflow.quality()`.
    pub optflow_quality: f32,
    /// `quality_filtered` before this tick.
    pub quality_filtered: f32,
    /// `flow_min_quality.get()`.
    pub flow_min_quality: i8,
    /// `flow_filter.get_cutoff_freq()`.
    pub flow_filter_cutoff_hz: f32,
    /// `flow_filter_hz.get()`.
    pub flow_filter_hz: f32,
    /// `AP_HAL::millis() - copter.arm_time_ms`.
    pub time_since_arm_ms: u32,
    /// Already-computed flow-to-angle leftover, body-frame rad.
    /// The PI / brake / I-term leftover stays later; this is the
    /// `flow_angles` that `flowhold_flow_to_angle` would have written.
    pub flow_angles_rad: (f32, f32),
}

impl FlowHoldRunView {
    /// Armed, auto-armed, airborne, healthy flow, past the arm delay.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            roll_control_in: 0,
            pitch_control_in: 0,
            has_valid_input: true,
            lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            target_climb_rate_ms: 0.0,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            armed: true,
            spool_state: SpoolState::ThrottleUnlimited,
            takeoff_running: false,
            auto_armed: true,
            land_complete: false,
            using_interlock: false,
            takeoff_alt_m: 2.5,
            optflow_healthy: true,
            optflow_quality: 200.0,
            quality_filtered: 200.0,
            flow_min_quality: FLOWHOLD_QUAL_MIN_DEFAULT,
            flow_filter_cutoff_hz: FLOWHOLD_FILTER_HZ_DEFAULT,
            flow_filter_hz: FLOWHOLD_FILTER_HZ_DEFAULT,
            time_since_arm_ms: 5_000,
            flow_angles_rad: (0.0, 0.0),
        }
    }
}

/// Attitude / throttle leftover of one `ModeFlowHold::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowHoldRun {
    /// Always true: `update_height_estimate()` runs first. The estimator
    /// leftover itself stays later.
    pub update_height_estimate: bool,
    /// Always true: `D_set_max_speed_accel_m`.
    pub set_max_speed_accel: bool,
    /// Always true: `update_simple_mode`.
    pub update_simple_mode: bool,
    /// `flow_filter.set_cutoff_frequency` because the param moved.
    pub set_filter_cutoff: bool,
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command. MotorStopped / Takeoff / Flying write one;
    /// landed states leave the machine's leftover (usually none).
    pub desired_spool: Option<DesiredSpoolState>,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` this iteration.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. FlowHold uses the default `true`.
    pub reset_yaw_rate: bool,
    /// Vertical-controller leftover.
    pub vertical: AltHoldVertical,
    /// `flow_pi_xy.reset_I()` on MotorStopped.
    pub reset_flow_i: bool,
    /// `takeoff.start_m` should run (Takeoff and the helper is idle).
    pub start_takeoff: bool,
    /// Altitude handed to `takeoff.start_m`, clamped to `[0, 10]`.
    pub takeoff_start_alt_m: f32,
    /// Climb rate after the speed clamp (and, on takeoff / flying, after
    /// the avoidance identity).
    pub target_climb_rate_ms: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// `quality_filtered` after the complementary filter.
    pub quality_filtered: f32,
    /// Pilot lean from the sticks, before the flow add.
    pub pilot_roll_rad: f32,
    /// Pilot lean from the sticks, before the flow add.
    pub pilot_pitch_rad: f32,
    /// Flow-to-angle ran (`quality >= min` and arm time `> 3000`).
    pub flow_to_angle: bool,
    /// `stick_input` handed to `flowhold_flow_to_angle`.
    pub stick_input: bool,
    /// Body-frame roll demand after the flow add and the lean-max clamp.
    pub bf_roll_rad: f32,
    /// Body-frame pitch demand after the flow add and the lean-max clamp.
    pub bf_pitch_rad: f32,
    /// Always true: `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
    pub input_euler_angle: bool,
    /// Always true: `D_update_controller` after the attitude call.
    pub update_d_controller: bool,
}

/// Upstream `ModeFlowHold::run`.
///
/// Height-estimate and the flow PI stay later leftovers; this owns the
/// tick around them. Quality is a 0.95 complementary filter that drops
/// to zero on an unhealthy sensor. Flow-to-angle is gated on that
/// filtered quality and a 3 s arm delay so the first climb cannot
/// wind the hold from a bad takeoff sample. Avoidance is compiled out
/// — the lean demand after the flow add is the path this leftover
/// records.
#[must_use]
pub fn flowhold_run(view: &FlowHoldRunView) -> FlowHoldRun {
    let set_filter_cutoff = !is_equal(view.flow_filter_cutoff_hz, view.flow_filter_hz);
    let target_climb_rate_ms = constrain_value(
        view.target_climb_rate_ms,
        -view.speed_dn_ms,
        view.speed_up_ms,
    );
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );
    let decision = alt_hold_state(&AltHoldInputs {
        armed: view.armed,
        spool_state: view.spool_state,
        takeoff_running: view.takeoff_running,
        auto_armed: view.auto_armed,
        land_complete: view.land_complete,
        using_interlock: view.using_interlock,
        target_climb_rate_ms,
    });
    let quality_filtered = if view.optflow_healthy {
        FLOWHOLD_QUALITY_FILTER * view.quality_filtered
            + (1.0 - FLOWHOLD_QUALITY_FILTER) * view.optflow_quality
    } else {
        0.0
    };

    let (
        reset_rate_i,
        reset_yaw,
        reset_yaw_rate,
        vertical,
        start_takeoff,
        reset_flow_i,
        desired_spool,
    ) = match decision.state {
        AltHoldModeState::MotorStopped => (
            RateIReset::Hard,
            true,
            true,
            AltHoldVertical::Relax,
            false,
            true,
            Some(DesiredSpoolState::ShutDown),
        ),
        AltHoldModeState::LandedGroundIdle => (
            RateIReset::Smooth,
            true,
            true,
            AltHoldVertical::Relax,
            false,
            false,
            decision.desired_spool,
        ),
        AltHoldModeState::LandedPreTakeoff => (
            RateIReset::Smooth,
            false,
            false,
            AltHoldVertical::Relax,
            false,
            false,
            decision.desired_spool,
        ),
        AltHoldModeState::Takeoff => (
            RateIReset::None,
            false,
            false,
            AltHoldVertical::Takeoff,
            !view.takeoff_running,
            false,
            Some(DesiredSpoolState::ThrottleUnlimited),
        ),
        AltHoldModeState::Flying => (
            RateIReset::None,
            false,
            false,
            AltHoldVertical::ClimbRate,
            false,
            false,
            Some(DesiredSpoolState::ThrottleUnlimited),
        ),
    };

    let (pilot_roll_rad, pilot_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        view.lean_angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let stick_input = view.roll_control_in != 0 || view.pitch_control_in != 0;
    let flow_to_angle = quality_filtered >= f32::from(view.flow_min_quality)
        && view.time_since_arm_ms > FLOWHOLD_ARM_DELAY_MS;
    let angle_max_rad = view.lean_angle_max_rad;
    let mut bf_roll_rad = pilot_roll_rad;
    let mut bf_pitch_rad = pilot_pitch_rad;
    if flow_to_angle {
        let half = angle_max_rad * 0.5;
        bf_roll_rad += constrain_value(view.flow_angles_rad.0, -half, half);
        bf_pitch_rad += constrain_value(view.flow_angles_rad.1, -half, half);
    }
    bf_roll_rad = constrain_value(bf_roll_rad, -angle_max_rad, angle_max_rad);
    bf_pitch_rad = constrain_value(bf_pitch_rad, -angle_max_rad, angle_max_rad);

    FlowHoldRun {
        update_height_estimate: true,
        set_max_speed_accel: true,
        update_simple_mode: true,
        set_filter_cutoff,
        state: decision.state,
        desired_spool,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        reset_flow_i,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        target_climb_rate_ms,
        target_yaw_rate_rads,
        quality_filtered,
        pilot_roll_rad,
        pilot_pitch_rad,
        flow_to_angle,
        stick_input,
        bf_roll_rad,
        bf_pitch_rad,
        input_euler_angle: true,
        update_d_controller: true,
    }
}
