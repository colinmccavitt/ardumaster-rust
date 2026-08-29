//! AutoYaw `set_mode` / `get_heading` leftover, upstream `ArduCopter/autoyaw.cpp`.
//!
//! Tracked as **COP-021**. The mode machine and its setters live in the
//! parent module. This is the leftover catalog of the two functions that
//! still talk to the vehicle: [`set_mode`] seeds look-ahead from AHRS and
//! zeroes a rate, and [`get_heading`] may call `set_mode` twice (pilot,
//! then weathervane) before reading `yaw_rad` / `rate_rads` from
//! pos-control, the attitude controller, circle-nav, or the armed bearing.
//!
//! # `HOLD` is not a yaw-angle case
//!
//! `yaw_rad`'s switch has no `HOLD` arm. It falls through to
//! `LOOK_AT_NEXT_WP` / `default` and reads `pos_control->get_yaw_rad()`.
//! The heading command is still `Rate_Only` — the leftover writes the
//! angle, then the attitude controller is told to ignore it. A port that
//! skipped the pos-control read for rate-only modes would disagree on
//! `_yaw_angle_rad` even though the flown command would look the same.
//!
//! # Weathervane is compiled out with the library
//!
//! `update_weathervane` is behind `#if WEATHERVANE_ENABLED`. When that is
//! off the leftover does not run, so a vehicle that somehow entered
//! `WEATHERVANE` would stay there. The `weathervane_enabled` input is that
//! compile switch, not a runtime parameter.
//!
//! # `get_yaw_out` is short-circuited
//!
//! Upstream only calls the weathervane library when
//! `allows_weathervaning()` is already true. A false permission must not
//! be recorded as a library leftover.

use super::{
    angle_rate_step, default_yaw_mode, fixed_yaw_step, heading_mode, pilot_yaw_override,
    roi_yaw_rad, weathervane_action, yaw_mode_entry, yaw_rate_source, HeadingMode,
    PilotYawOverride, WeathervaneAction, WpYawBehaviour, YawMode, YawModeEntry, YawRateSource,
};

/// What `Mode::AutoYaw::set_mode` stored and asked the vehicle for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetModeLeftover {
    /// `_mode` after the call.
    pub mode: YawMode,
    /// `_last_mode` after the call. Unchanged when the mode did not change.
    pub last_mode: YawMode,
    /// Whether `_last_mode` / `_mode` were written.
    pub changed: bool,
    /// The entry effect, or `None` when the call returned immediately.
    pub entry: Option<YawModeEntry>,
    /// `LOOK_AHEAD` leftover of `ahrs.get_yaw_rad()`.
    pub need_ahrs_yaw: bool,
    /// Seeded look-ahead heading. `None` unless [`Self::need_ahrs_yaw`].
    pub look_ahead_yaw_rad: Option<f32>,
    /// Zeroed stored rate. `Some(0.0)` only on a real `RATE` entry.
    pub yaw_rate_rads: Option<f32>,
}

/// AutoYaw `set_mode` leftover.
///
/// Re-selecting the current mode is not a transition: `_last_mode` is
/// left alone and the `LOOK_AHEAD` / `RATE` initialisation does not re-run.
/// Asking for `RATE` twice must not wipe a rate `set_rate_rad` just wrote.
#[must_use]
pub fn set_mode(
    current: YawMode,
    last_mode: YawMode,
    new_mode: YawMode,
    ahrs_yaw_rad: f32,
) -> SetModeLeftover {
    match yaw_mode_entry(current, new_mode) {
        None => SetModeLeftover {
            mode: current,
            last_mode,
            changed: false,
            entry: None,
            need_ahrs_yaw: false,
            look_ahead_yaw_rad: None,
            yaw_rate_rads: None,
        },
        Some(entry) => {
            let need_ahrs_yaw = entry == YawModeEntry::SeedLookAheadFromCurrentYaw;
            SetModeLeftover {
                mode: new_mode,
                last_mode: current,
                changed: true,
                entry: Some(entry),
                need_ahrs_yaw,
                look_ahead_yaw_rad: if need_ahrs_yaw {
                    Some(ahrs_yaw_rad)
                } else {
                    None
                },
                yaw_rate_rads: if entry == YawModeEntry::ZeroYawRate {
                    Some(0.0)
                } else {
                    None
                },
            }
        }
    }
}

/// `Mode::AutoYaw::set_mode_to_default` leftover.
///
/// `rtl` is forwarded to [`default_yaw_mode`]. The weathervane release
/// path hard-codes it false even when the aircraft is returning.
#[must_use]
pub fn set_mode_to_default(
    current: YawMode,
    last_mode: YawMode,
    behaviour: WpYawBehaviour,
    rtl: bool,
    ahrs_yaw_rad: f32,
) -> SetModeLeftover {
    set_mode(
        current,
        last_mode,
        default_yaw_mode(behaviour, rtl),
        ahrs_yaw_rad,
    )
}

/// Where `yaw_rad` took the heading from this iteration.
///
/// This is the leftover catalog of that switch, not a heading-mode
/// synonym. [`YawAngleSource::PosControl`] covers both `LOOK_AT_NEXT_WP`
/// and the `HOLD` fall-through.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawAngleSource {
    /// `roi_yaw_rad()`.
    Roi,
    /// Fixed-yaw slew. Leftover of `millis()` for `dt`.
    FixedSlew,
    /// `look_ahead_yaw_rad()`.
    LookAhead,
    /// `copter.initial_armed_bearing_rad`.
    ArmedBearing,
    /// `circle_nav->get_yaw_rad()` while circle-nav is compiled in and active.
    CircleNav,
    /// `CIRCLE` but the nav is compiled out or inactive: stored angle held.
    CircleHeld,
    /// `attitude_control->get_att_target_euler_rad().z`.
    AttitudeTarget,
    /// `pos_control->get_yaw_rad()`. `LOOK_AT_NEXT_WP` and `HOLD`.
    PosControl,
    /// Angle-rate integration. Leftover of `millis()` for `dt`.
    AngleRate,
}

impl YawAngleSource {
    /// Leftover of `pos_control->get_yaw_rad()`.
    #[must_use]
    pub const fn need_pos_control_yaw(self) -> bool {
        matches!(self, Self::PosControl)
    }

    /// Leftover of `attitude_control->get_att_target_euler_rad().z`.
    #[must_use]
    pub const fn need_att_target_yaw(self) -> bool {
        matches!(self, Self::AttitudeTarget)
    }

    /// Leftover of `circle_nav->get_yaw_rad()`.
    #[must_use]
    pub const fn need_circle_yaw(self) -> bool {
        matches!(self, Self::CircleNav)
    }

    /// Leftover of `initial_armed_bearing_rad`.
    #[must_use]
    pub const fn need_armed_bearing(self) -> bool {
        matches!(self, Self::ArmedBearing)
    }

    /// Leftover of `millis()` for the two time-stepped arms.
    #[must_use]
    pub const fn need_millis(self) -> bool {
        matches!(self, Self::FixedSlew | Self::AngleRate)
    }
}

/// Inputs `Mode::AutoYaw::get_heading` reads from the vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GetHeadingContext {
    /// `_mode` at the start of the call.
    pub mode: YawMode,
    /// `_last_mode` at the start of the call.
    pub last_mode: YawMode,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `flightmode->use_pilot_yaw()`.
    pub use_pilot_yaw: bool,
    /// Leftover of `get_pilot_desired_yaw_rate_rads`. Ignored unless RC
    /// is valid and the mode uses pilot yaw.
    pub pilot_desired_yaw_rate_rads: f32,
    /// `WEATHERVANE_ENABLED`. Not a runtime parameter.
    pub weathervane_enabled: bool,
    /// `flightmode->allows_weathervaning()`.
    pub allows_weathervaning: bool,
    /// Leftover of `weathervane.get_yaw_out`. Only consumed when
    /// [`Self::allows_weathervaning`] is already true.
    pub weathervane_wants_yaw: bool,
    /// Rate `get_yaw_out` produced, already in rad/s.
    pub weathervane_rate_rads: f32,
    /// `g.wp_yaw_behavior`, for a weathervane release from `HOLD`.
    pub wp_yaw_behavior: WpYawBehaviour,
    /// `ahrs.get_yaw_rad()`, for a `LOOK_AHEAD` `set_mode` leftover.
    pub ahrs_yaw_rad: f32,
    /// `millis()` this iteration.
    pub now_ms: u32,
    /// `_last_update_ms` from the previous iteration.
    pub last_update_ms: u32,
    /// `_yaw_angle_rad` from the previous iteration.
    pub yaw_angle_rad: f32,
    /// `_yaw_rate_rads` from the previous iteration.
    pub yaw_rate_rads: f32,
    /// `_look_ahead_yaw_rad` from the previous iteration.
    pub look_ahead_yaw_rad: f32,
    /// `_fixed_yaw_offset_rad`.
    pub fixed_yaw_offset_rad: f32,
    /// `_fixed_yaw_slewrate_rads`.
    pub fixed_yaw_slewrate_rads: f32,
    /// `copter.initial_armed_bearing_rad`.
    pub initial_armed_bearing_rad: f32,
    /// `MODE_CIRCLE_ENABLED`.
    pub circle_nav_enabled: bool,
    /// `circle_nav->is_active()`.
    pub circle_nav_active: bool,
    /// `circle_nav->get_yaw_rad()`.
    pub circle_yaw_rad: f32,
    /// `attitude_control->get_att_target_euler_rad().z`.
    pub att_target_yaw_rad: f32,
    /// `pos_control->get_yaw_rad()`.
    pub pos_control_yaw_rad: f32,
    /// `pos_control->get_yaw_rate_rads()`.
    pub pos_control_yaw_rate_rads: f32,
    /// Position estimate is usable, for look-ahead and ROI.
    pub position_ok: bool,
    /// North velocity, m/s, for look-ahead.
    pub vel_n_ms: f32,
    /// East velocity, m/s, for look-ahead.
    pub vel_e_ms: f32,
    /// ROI north-east of EKF origin, m.
    pub roi_ne_m: (f32, f32),
    /// Vehicle north-east of EKF origin, or `None` when the estimate is
    /// unavailable. ROI then holds the attitude target.
    pub position_ne_m: Option<(f32, f32)>,
}

impl Default for GetHeadingContext {
    fn default() -> Self {
        // No RC, weathervane compiled in but not allowed: the heading
        // fixture's situation. Starting in HOLD stays in HOLD.
        Self {
            mode: YawMode::Hold,
            last_mode: YawMode::Hold,
            has_valid_input: false,
            use_pilot_yaw: true,
            pilot_desired_yaw_rate_rads: 0.0,
            weathervane_enabled: true,
            allows_weathervaning: false,
            weathervane_wants_yaw: false,
            weathervane_rate_rads: 0.0,
            wp_yaw_behavior: WpYawBehaviour::LookAtNextWp,
            ahrs_yaw_rad: 0.0,
            now_ms: 1_000,
            last_update_ms: 990,
            yaw_angle_rad: 0.0,
            yaw_rate_rads: 0.0,
            look_ahead_yaw_rad: 0.0,
            fixed_yaw_offset_rad: 0.0,
            fixed_yaw_slewrate_rads: 1.0,
            initial_armed_bearing_rad: 0.0,
            circle_nav_enabled: true,
            circle_nav_active: false,
            circle_yaw_rad: 0.0,
            att_target_yaw_rad: 0.0,
            pos_control_yaw_rad: 0.0,
            pos_control_yaw_rate_rads: 0.0,
            position_ok: true,
            vel_n_ms: 0.0,
            vel_e_ms: 0.0,
            roi_ne_m: (0.0, 0.0),
            position_ne_m: Some((0.0, 0.0)),
        }
    }
}

/// What `Mode::AutoYaw::get_heading` stored and asked the vehicle for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GetHeadingLeftover {
    /// `HeadingCommand.heading_mode` from the mode the vehicle ended in.
    pub heading_mode: HeadingMode,
    /// `HeadingCommand.yaw_angle_rad` / new `_yaw_angle_rad`.
    pub yaw_angle_rad: f32,
    /// `HeadingCommand.yaw_rate_rads` / new `_yaw_rate_rads`.
    pub yaw_rate_rads: f32,
    /// `_mode` after the call.
    pub mode: YawMode,
    /// `_last_mode` after the call.
    pub last_mode: YawMode,
    /// `_pilot_yaw_rate_rads` after the call.
    pub pilot_yaw_rate_rads: f32,
    /// `_last_update_ms` after the call.
    pub last_update_ms: u32,
    /// `_look_ahead_yaw_rad` after the call.
    pub look_ahead_yaw_rad: f32,
    /// `_fixed_yaw_offset_rad` after the call.
    pub fixed_yaw_offset_rad: f32,
    /// Leftover of `get_pilot_desired_yaw_rate_rads`.
    pub need_pilot_yaw_rate: bool,
    /// The compile switch was on, so `update_weathervane` ran.
    pub need_update_weathervane: bool,
    /// Leftover of `weathervane.get_yaw_out`. Short-circuited when
    /// weathervaning is not allowed.
    pub need_weathervane_yaw_out: bool,
    /// `set_mode` the pilot branch asked for, if any.
    pub pilot_set_mode: Option<YawMode>,
    /// `set_mode` the weathervane branch asked for, if any.
    pub weathervane_set_mode: Option<YawMode>,
    /// The last `set_mode` leftover, or the no-op if neither branch fired.
    pub set_mode: SetModeLeftover,
    /// Where `yaw_rad` took the heading.
    pub yaw_angle_source: YawAngleSource,
    /// Where `rate_rads` took the rate.
    pub yaw_rate_source: YawRateSource,
}

/// AutoYaw `get_heading` leftover.
///
/// Pilot override runs first and may `set_mode(PILOT_RATE)` or
/// `set_mode(HOLD)`. Weathervane then runs and may take the axis or hand
/// it back. `yaw_rad` and `rate_rads` read the *final* mode.
#[must_use]
pub fn get_heading(ctx: &GetHeadingContext) -> GetHeadingLeftover {
    let mut mode = ctx.mode;
    let mut last_mode = ctx.last_mode;
    let mut yaw_rate_rads = ctx.yaw_rate_rads;
    let mut look_ahead_yaw_rad = ctx.look_ahead_yaw_rad;
    let mut set_mode_leftover = set_mode(mode, last_mode, mode, ctx.ahrs_yaw_rad);

    let need_pilot_yaw_rate = ctx.has_valid_input && ctx.use_pilot_yaw;
    let pilot_yaw_rate_rads = if need_pilot_yaw_rate {
        ctx.pilot_desired_yaw_rate_rads
    } else {
        0.0
    };

    let pilot_set_mode = match pilot_yaw_override(
        mode,
        ctx.has_valid_input,
        ctx.use_pilot_yaw,
        pilot_yaw_rate_rads,
    ) {
        PilotYawOverride::TakeControl => Some(YawMode::PilotRate),
        PilotYawOverride::ReleaseToHold => Some(YawMode::Hold),
        PilotYawOverride::None => None,
    };
    if let Some(requested) = pilot_set_mode {
        set_mode_leftover = set_mode(mode, last_mode, requested, ctx.ahrs_yaw_rad);
        apply_set_mode(
            &mut mode,
            &mut last_mode,
            &mut yaw_rate_rads,
            &mut look_ahead_yaw_rad,
            set_mode_leftover,
        );
    }

    let need_update_weathervane = ctx.weathervane_enabled;
    let need_weathervane_yaw_out = need_update_weathervane && ctx.allows_weathervaning;
    let weathervane_set_mode = if need_update_weathervane {
        match weathervane_action(
            mode,
            last_mode,
            ctx.allows_weathervaning,
            ctx.weathervane_wants_yaw,
        ) {
            WeathervaneAction::Engage => {
                set_mode_leftover =
                    set_mode(mode, last_mode, YawMode::Weathervane, ctx.ahrs_yaw_rad);
                apply_set_mode(
                    &mut mode,
                    &mut last_mode,
                    &mut yaw_rate_rads,
                    &mut look_ahead_yaw_rad,
                    set_mode_leftover,
                );
                // After set_mode: WEATHERVANE initialises nothing, so this
                // assignment is what actually arms the rate.
                yaw_rate_rads = ctx.weathervane_rate_rads;
                Some(YawMode::Weathervane)
            }
            WeathervaneAction::ReleaseToDefault => {
                // Zero first: set_mode_to_default does not touch the rate.
                yaw_rate_rads = 0.0;
                set_mode_leftover = set_mode_to_default(
                    mode,
                    last_mode,
                    ctx.wp_yaw_behavior,
                    false,
                    ctx.ahrs_yaw_rad,
                );
                apply_set_mode(
                    &mut mode,
                    &mut last_mode,
                    &mut yaw_rate_rads,
                    &mut look_ahead_yaw_rad,
                    set_mode_leftover,
                );
                Some(set_mode_leftover.mode)
            }
            WeathervaneAction::ReleaseTo(restored) => {
                yaw_rate_rads = 0.0;
                set_mode_leftover = set_mode(mode, last_mode, restored, ctx.ahrs_yaw_rad);
                apply_set_mode(
                    &mut mode,
                    &mut last_mode,
                    &mut yaw_rate_rads,
                    &mut look_ahead_yaw_rad,
                    set_mode_leftover,
                );
                Some(restored)
            }
            WeathervaneAction::None => None,
        }
    } else {
        None
    };

    let yaw = yaw_rad_leftover(mode, ctx, look_ahead_yaw_rad, yaw_rate_rads);
    let yaw_rate_source = yaw_rate_source(mode);
    let yaw_rate_rads = match yaw_rate_source {
        YawRateSource::Zero => 0.0,
        YawRateSource::PositionController => ctx.pos_control_yaw_rate_rads,
        YawRateSource::Pilot => pilot_yaw_rate_rads,
        YawRateSource::Unchanged => yaw_rate_rads,
    };

    GetHeadingLeftover {
        heading_mode: heading_mode(mode),
        yaw_angle_rad: yaw.angle_rad,
        yaw_rate_rads,
        mode,
        last_mode,
        pilot_yaw_rate_rads,
        last_update_ms: yaw.last_update_ms,
        look_ahead_yaw_rad: yaw.look_ahead_yaw_rad,
        fixed_yaw_offset_rad: yaw.fixed_yaw_offset_rad,
        need_pilot_yaw_rate,
        need_update_weathervane,
        need_weathervane_yaw_out,
        pilot_set_mode,
        weathervane_set_mode,
        set_mode: set_mode_leftover,
        yaw_angle_source: yaw.source,
        yaw_rate_source,
    }
}

fn apply_set_mode(
    mode: &mut YawMode,
    last_mode: &mut YawMode,
    yaw_rate_rads: &mut f32,
    look_ahead_yaw_rad: &mut f32,
    leftover: SetModeLeftover,
) {
    *mode = leftover.mode;
    *last_mode = leftover.last_mode;
    if let Some(rate) = leftover.yaw_rate_rads {
        *yaw_rate_rads = rate;
    }
    if let Some(yaw) = leftover.look_ahead_yaw_rad {
        *look_ahead_yaw_rad = yaw;
    }
}

struct YawRadLeftover {
    angle_rad: f32,
    last_update_ms: u32,
    look_ahead_yaw_rad: f32,
    fixed_yaw_offset_rad: f32,
    source: YawAngleSource,
}

fn yaw_rad_leftover(
    mode: YawMode,
    ctx: &GetHeadingContext,
    look_ahead_yaw_rad: f32,
    yaw_rate_rads: f32,
) -> YawRadLeftover {
    let held = YawRadLeftover {
        angle_rad: ctx.yaw_angle_rad,
        last_update_ms: ctx.last_update_ms,
        look_ahead_yaw_rad,
        fixed_yaw_offset_rad: ctx.fixed_yaw_offset_rad,
        source: YawAngleSource::PosControl,
    };

    match mode {
        YawMode::Roi => YawRadLeftover {
            angle_rad: roi_yaw_rad(ctx.position_ne_m, ctx.roi_ne_m, ctx.att_target_yaw_rad),
            source: YawAngleSource::Roi,
            ..held
        },
        YawMode::Fixed => {
            let (angle_rad, offset) = fixed_yaw_step(
                ctx.yaw_angle_rad,
                ctx.fixed_yaw_offset_rad,
                ctx.fixed_yaw_slewrate_rads,
                dt_s(ctx.now_ms, ctx.last_update_ms),
            );
            YawRadLeftover {
                angle_rad,
                last_update_ms: ctx.now_ms,
                fixed_yaw_offset_rad: offset,
                source: YawAngleSource::FixedSlew,
                ..held
            }
        }
        YawMode::LookAhead => {
            let angle_rad = super::look_ahead_yaw_rad(
                look_ahead_yaw_rad,
                ctx.position_ok,
                ctx.vel_n_ms,
                ctx.vel_e_ms,
            );
            YawRadLeftover {
                angle_rad,
                look_ahead_yaw_rad: angle_rad,
                source: YawAngleSource::LookAhead,
                ..held
            }
        }
        YawMode::ResetToArmedYaw => YawRadLeftover {
            angle_rad: ctx.initial_armed_bearing_rad,
            source: YawAngleSource::ArmedBearing,
            ..held
        },
        YawMode::Circle => {
            if ctx.circle_nav_enabled && ctx.circle_nav_active {
                YawRadLeftover {
                    angle_rad: ctx.circle_yaw_rad,
                    source: YawAngleSource::CircleNav,
                    ..held
                }
            } else {
                YawRadLeftover {
                    source: YawAngleSource::CircleHeld,
                    ..held
                }
            }
        }
        YawMode::AngleRate => YawRadLeftover {
            angle_rad: angle_rate_step(
                ctx.yaw_angle_rad,
                yaw_rate_rads,
                dt_s(ctx.now_ms, ctx.last_update_ms),
            ),
            last_update_ms: ctx.now_ms,
            source: YawAngleSource::AngleRate,
            ..held
        },
        YawMode::Rate | YawMode::Weathervane | YawMode::PilotRate => YawRadLeftover {
            angle_rad: ctx.att_target_yaw_rad,
            source: YawAngleSource::AttitudeTarget,
            ..held
        },
        // HOLD falls through to the default arm — leftover of the missing case.
        YawMode::Hold | YawMode::LookAtNextWp => YawRadLeftover {
            angle_rad: ctx.pos_control_yaw_rad,
            source: YawAngleSource::PosControl,
            ..held
        },
    }
}

#[allow(
    clippy::cast_precision_loss,
    reason = "reproduces upstream's (now_ms - last_ms) * 0.001"
)]
fn dt_s(now_ms: u32, last_update_ms: u32) -> f32 {
    now_ms.wrapping_sub(last_update_ms) as f32 * 0.001
}
