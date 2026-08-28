//! Demand state machine, upstream `AP_AutoTune::ATState` / `update`.
//!
//! Plane-4.7.0 uses three states, not the older Copter Idle / Demanding /
//! Waiting / Updating names. `update` leaves `IDLE` when the filtered
//! target rate and attitude error both look like a stick demand, and
//! returns to `IDLE` when the rate falls back through a lower threshold.

use crate::gains::{apply_stop_gains, snapshot_gains, AtGains};
use crate::start::floor_start_ff;

/// Fraction of `min(att_limit/tau, rmax_pos)` that starts a demand event.
///
/// Upstream `0.4 * MIN(att_limit_deg / current.tau, current.rmax_pos)`.
pub const RATE_THRESHOLD1_FRAC: f32 = 0.4;

/// Exit threshold as a fraction of [`RATE_THRESHOLD1_FRAC`]'s result.
///
/// Upstream `0.25 * rate_threshold1`.
pub const RATE_THRESHOLD2_FRAC: f32 = 0.25;

/// Attitude-error fraction of the axis limit that counts as "in demand".
///
/// Upstream `fabsf(angle_err_deg) >= 0.3 * att_limit_deg`.
pub const ATT_DEMAND_FRAC: f32 = 0.3;

/// Upstream `AP_AutoTune::ATType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AtType {
    /// `AUTOTUNE_ROLL = 0`.
    Roll = 0,
    /// `AUTOTUNE_PITCH = 1`.
    Pitch = 1,
    /// `AUTOTUNE_YAW = 2`.
    Yaw = 2,
}

impl AtType {
    /// Decode an upstream `ATType` discriminant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Roll),
            1 => Some(Self::Pitch),
            2 => Some(Self::Yaw),
            _ => None,
        }
    }

    /// The stored discriminant.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Upstream `AP_AutoTune::axis_string`.
    #[must_use]
    pub const fn axis_string(self) -> &'static str {
        match self {
            Self::Roll => "Roll",
            Self::Pitch => "Pitch",
            Self::Yaw => "Yaw",
        }
    }
}

/// Upstream `AP_AutoTune::ATState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AtState {
    /// Waiting for a stick demand that exceeds the rate threshold.
    Idle = 0,
    /// Positive-rate demand event in progress.
    DemandPos = 1,
    /// Negative-rate demand event in progress.
    DemandNeg = 2,
}

impl AtState {
    /// Decode an upstream `ATState` discriminant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Idle),
            1 => Some(Self::DemandPos),
            2 => Some(Self::DemandNeg),
            _ => None,
        }
    }

    /// The stored discriminant, matching the `log_ATRP.state` field.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Entry rate threshold, upstream `rate_threshold1`.
///
/// `tau` must be positive — the same assumption `update` makes when it
/// divides `att_limit_deg` by `current.tau`.
#[must_use]
pub fn rate_threshold1(att_limit_deg: f32, tau: f32, rmax_pos: f32) -> f32 {
    let att_over_tau = att_limit_deg / tau;
    let capped = if att_over_tau < rmax_pos {
        att_over_tau
    } else {
        rmax_pos
    };
    RATE_THRESHOLD1_FRAC * capped
}

/// Exit rate threshold, upstream `rate_threshold2`.
#[must_use]
pub fn rate_threshold2(threshold1: f32) -> f32 {
    RATE_THRESHOLD2_FRAC * threshold1
}

/// Whether attitude error is large enough to count as a demand.
///
/// Upstream `in_att_demand`.
#[must_use]
pub fn in_att_demand(angle_err_deg: f32, att_limit_deg: f32) -> bool {
    angle_err_deg.abs() >= ATT_DEMAND_FRAC * att_limit_deg
}

/// Next demand state from the `switch (state)` in `AP_AutoTune::update`.
///
/// Gain rewrite, FF estimation, and the idle-oscillation `IDLE_LOWER_PD`
/// path are not part of this slice.
#[must_use]
pub fn next_demand_state(
    state: AtState,
    desired_rate: f32,
    threshold1: f32,
    threshold2: f32,
    in_att_demand: bool,
) -> AtState {
    match state {
        AtState::Idle => {
            if desired_rate > threshold1 && in_att_demand {
                AtState::DemandPos
            } else if desired_rate < -threshold1 && in_att_demand {
                AtState::DemandNeg
            } else {
                AtState::Idle
            }
        }
        AtState::DemandPos => {
            if desired_rate < threshold2 {
                AtState::Idle
            } else {
                AtState::DemandPos
            }
        }
        AtState::DemandNeg => {
            if desired_rate > -threshold2 {
                AtState::Idle
            } else {
                AtState::DemandNeg
            }
        }
    }
}

/// One-axis tuner session, upstream `AP_AutoTune` running/state fields.
///
/// `start` / `stop` match the mode-enter / mode-leave calls. Demand
/// transitions are applied with [`AutoTune::update_demand`] so tests can
/// drive the `switch (state)` without the PID/filter body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTune {
    /// Upstream `AP_AutoTune::running`.
    pub running: bool,
    /// Upstream `AP_AutoTune::state`.
    pub state: AtState,
    /// Which axis this session tunes.
    pub axis: AtType,
    /// Live axis gains, upstream `AP_AutoTune::current`.
    pub current: AtGains,
    /// Snapshot restored on abort, upstream `AP_AutoTune::restore`.
    pub restore: AtGains,
    /// Last persisted snapshot, upstream `AP_AutoTune::last_save`.
    pub last_save: AtGains,
    /// Highest accepted P this session, upstream `P_limit`.
    pub p_limit: f32,
    /// Highest accepted D this session, upstream `D_limit`.
    pub d_limit: f32,
    /// Live feed-forward, upstream `ATGains::FF` / `rpid.ff()`.
    ///
    /// Not stored on [`AtGains`] (see [`crate::ff`]); [`AutoTune::start`]
    /// floors values below [`crate::start::AUTOTUNE_MIN_FF`].
    pub ff: f32,
}

impl AutoTune {
    /// Construct a stopped tuner on `axis`, matching the C++ constructor
    /// leaving `running` false and `state` at its default (`IDLE` = 0).
    #[must_use]
    pub const fn new(axis: AtType) -> Self {
        Self::with_gains(axis, AtGains::ZERO)
    }

    /// Construct a stopped tuner holding `current` as the live gains.
    #[must_use]
    pub const fn with_gains(axis: AtType, current: AtGains) -> Self {
        Self {
            running: false,
            state: AtState::Idle,
            axis,
            current,
            restore: AtGains::ZERO,
            last_save: AtGains::ZERO,
            p_limit: 0.0,
            d_limit: 0.0,
            ff: 0.0,
        }
    }

    /// Live gains, upstream `AP_AutoTune::get_gains` without the PID merge.
    #[must_use]
    pub const fn get_gains(&self) -> AtGains {
        self.current
    }

    /// Persist `current` into `last_save`, upstream `save_gains`.
    pub fn save_gains(&mut self) {
        self.last_save = self.get_gains();
    }

    /// Put the start-of-session snapshot back, upstream `restore_gains`.
    pub fn restore_gains(&mut self) {
        self.current = self.restore;
    }

    /// Upstream `AP_AutoTune::start` — enter AUTOTUNE on this axis.
    ///
    /// Sets `running`, forces `IDLE`, snapshots `current` into
    /// `restore` / `last_save`, then floors FF below 0.01 so the tuner
    /// never starts at zero feed-forward.
    pub fn start(&mut self) {
        self.running = true;
        self.state = AtState::Idle;
        let (restore, last_save) = snapshot_gains(self.get_gains());
        self.restore = restore;
        self.last_save = last_save;
        self.p_limit = 0.0;
        self.d_limit = 0.0;
        self.ff = floor_start_ff(self.ff);
    }

    /// Upstream `AP_AutoTune::stop` — leave AUTOTUNE on this axis.
    ///
    /// Saves when both P and D limits are positive; otherwise restores
    /// the start-of-session snapshot.
    pub fn stop(&mut self) {
        if self.running {
            self.running = false;
            let (current, last_save) = apply_stop_gains(
                self.current,
                self.restore,
                self.last_save,
                self.p_limit,
                self.d_limit,
            );
            self.current = current;
            self.last_save = last_save;
        }
    }

    /// Apply the demand-state `switch` from `AP_AutoTune::update`.
    ///
    /// No-op when not running, matching the early return at the top of
    /// `update`.
    pub fn update_demand(
        &mut self,
        desired_rate: f32,
        threshold1: f32,
        threshold2: f32,
        in_att_demand: bool,
    ) {
        if !self.running {
            return;
        }
        self.state = next_demand_state(
            self.state,
            desired_rate,
            threshold1,
            threshold2,
            in_att_demand,
        );
    }
}
