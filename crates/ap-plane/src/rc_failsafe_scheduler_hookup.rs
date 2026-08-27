//! RC channel read and failsafe hookup for the scheduler tick.
//!
//! Upstream `Plane::read_radio` feeds `channel_roll->norm_input_dz()` et al.
//! and sets `rc().in_rc_failsafe()` when pulses are lost or throttle is low.

use crate::stabilize_hookup::RcStickInputs;

/// Per-channel calibration, upstream `RC_Channel` min/max/deadzone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannelConfig {
    pub radio_min: u16,
    pub radio_max: u16,
    pub deadzone: u16,
    /// Upstream `RC_Channel::reversed` for percent_input.
    pub reversed: bool,
}

impl Default for RcChannelConfig {
    fn default() -> Self {
        Self {
            radio_min: 1100,
            radio_max: 1900,
            deadzone: 30,
            reversed: false,
        }
    }
}

/// Throttle failsafe parameters, upstream `FS_THR_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcFailsafeConfig {
    pub throttle_failsafe_enabled: bool,
    pub throttle_failsafe_pwm: u16,
}

impl Default for RcFailsafeConfig {
    fn default() -> Self {
        Self {
            throttle_failsafe_enabled: true,
            throttle_failsafe_pwm: 975,
        }
    }
}

/// HAL inputs for one RC/failsafe scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcFailsafeSchedulerInputs {
    pub roll_pwm: Option<u16>,
    pub pitch_pwm: Option<u16>,
    pub throttle_pwm: Option<u16>,
    pub has_valid_input: bool,
    pub roll_cfg: RcChannelConfig,
    pub pitch_cfg: RcChannelConfig,
    pub failsafe_cfg: RcFailsafeConfig,
    /// Flap aux channel PWM, upstream `channel_flap->get_radio_in()`.
    pub flap_pwm: Option<u16>,
    pub flap_cfg: RcChannelConfig,
}

impl Default for RcFailsafeSchedulerInputs {
    fn default() -> Self {
        Self {
            roll_pwm: None,
            pitch_pwm: None,
            throttle_pwm: None,
            has_valid_input: false,
            roll_cfg: RcChannelConfig::default(),
            pitch_cfg: RcChannelConfig::default(),
            failsafe_cfg: RcFailsafeConfig::default(),
            flap_pwm: None,
            flap_cfg: RcChannelConfig::default(),
        }
    }
}

/// Result of one RC/failsafe scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcFailsafeSchedulerOutput {
    pub rc_sticks: RcStickInputs,
    /// Manual flap percent from the flap aux channel.
    pub manual_flap_percent: i8,
    pub in_rc_failsafe: bool,
    pub ran: bool,
}

/// Normalized stick with deadzone, upstream `RC_Channel::norm_input_dz`.
#[must_use]
pub fn norm_input_dz(pwm: u16, cfg: &RcChannelConfig) -> f32 {
    let range = cfg.radio_max.saturating_sub(cfg.radio_min);
    if range == 0 {
        return 0.0;
    }
    let norm = if pwm <= cfg.radio_min {
        -1.0
    } else if pwm >= cfg.radio_max {
        1.0
    } else {
        let center = (f32::from(cfg.radio_min) + f32::from(cfg.radio_max)) / 2.0;
        (f32::from(pwm) - center) * 2.0 / f32::from(range)
    };
    let dz = f32::from(cfg.deadzone) * 0.001;
    if (if norm < 0.0 { -norm } else { norm }) < dz {
        0.0
    } else {
        norm
    }
}


/// Unsigned 0..100 stick percent, upstream `RC_Channel::percent_input`.
#[must_use]
pub fn percent_input(pwm: u16, cfg: &RcChannelConfig) -> i8 {
    let range = cfg.radio_max.saturating_sub(cfg.radio_min);
    if range == 0 {
        return 0;
    }
    let ret = if pwm <= cfg.radio_min {
        if cfg.reversed { 100 } else { 0 }
    } else if pwm >= cfg.radio_max {
        if cfg.reversed { 0 } else { 100 }
    } else {
        let pct = (100.0 * f32::from(pwm.saturating_sub(cfg.radio_min))
            / f32::from(range)) as u8;
        if cfg.reversed {
            100_u8.saturating_sub(pct)
        } else {
            pct
        }
    };
    i8::try_from(ret).unwrap_or(100)
}

/// Whether the receiver is in failsafe, upstream `RC_Channels::in_rc_failsafe`.
#[must_use]
pub fn detect_rc_failsafe(inp: &RcFailsafeSchedulerInputs) -> bool {
    if !inp.has_valid_input {
        return true;
    }
    if inp.failsafe_cfg.throttle_failsafe_enabled {
        if let Some(thr) = inp.throttle_pwm {
            if thr < inp.failsafe_cfg.throttle_failsafe_pwm {
                return true;
            }
        }
    }
    false
}

/// Read RC channels and publish stick inputs for stabilize, upstream `read_radio`.
#[must_use]
pub fn rc_failsafe_scheduler_tick(inp: &RcFailsafeSchedulerInputs) -> RcFailsafeSchedulerOutput {
    let in_rc_failsafe = detect_rc_failsafe(inp);
    if in_rc_failsafe {
        return RcFailsafeSchedulerOutput {
            rc_sticks: RcStickInputs::default(),
            manual_flap_percent: 0,
            in_rc_failsafe: true,
            ran: true,
        };
    }

    let roll_norm_dz = inp
        .roll_pwm
        .map(|pwm| norm_input_dz(pwm, &inp.roll_cfg))
        .unwrap_or(0.0);
    let pitch_norm_dz = inp
        .pitch_pwm
        .map(|pwm| norm_input_dz(pwm, &inp.pitch_cfg))
        .unwrap_or(0.0);

    let manual_flap_percent = inp
        .flap_pwm
        .map(|pwm| percent_input(pwm, &inp.flap_cfg))
        .unwrap_or(0);

    RcFailsafeSchedulerOutput {
        rc_sticks: RcStickInputs {
            roll_norm_dz,
            pitch_norm_dz,
        },
        manual_flap_percent,
        in_rc_failsafe: false,
        ran: true,
    }
}
