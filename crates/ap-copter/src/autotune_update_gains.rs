//! Multi `Step::UPDATE_GAINS` leftover, upstream `AC_AutoTune_Multi`.
//!
//! Tracked as **COP-027**. The Copter wrapper used to catalogue
//! UPDATE_GAINS as a flag and fall through into ABORT. This leftover
//! is the tune-type switch and the Multi `updating_*` gain math that
//! the flag skipped. Sequencing lives in [`crate::autotune_next`].
//! Heli `RATE_FF_UP` /
//! `MAX_GAINS` trip flow-of-control. `TUNE_CHECK` only forces
//! [`AUTOTUNE_SUCCESS_COUNT`]; `TUNE_COMPLETE` is a no-op.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::{AutoTuneRunView, AxisType, TuneMode, TuneType, AUTOTUNE_SUCCESS_COUNT};

/// `AUTOTUNE_RD_STEP`.
pub const AUTOTUNE_RD_STEP: f32 = 0.05;

/// `AUTOTUNE_RP_STEP`.
pub const AUTOTUNE_RP_STEP: f32 = 0.05;

/// `AUTOTUNE_SP_STEP`.
pub const AUTOTUNE_SP_STEP: f32 = 0.05;

/// `AUTOTUNE_RD_MAX`.
pub const AUTOTUNE_RD_MAX: f32 = 0.200;

/// `AUTOTUNE_RP_MIN`.
pub const AUTOTUNE_RP_MIN: f32 = 0.01;

/// `AUTOTUNE_RP_MAX`.
pub const AUTOTUNE_RP_MAX: f32 = 2.0;

/// `AUTOTUNE_SP_MIN`.
pub const AUTOTUNE_SP_MIN: f32 = 0.5;

/// Copter `AUTOTUNE_SP_MAX`. Plane uses 10.0.
pub const AUTOTUNE_SP_MAX: f32 = 40.0;

/// `AUTOTUNE_D_UP_DOWN_MARGIN`.
pub const AUTOTUNE_D_UP_DOWN_MARGIN: f32 = 0.2;

/// `AUTOTUNE_RLPF_MIN` — yaw Rate-D leftover uses the error-filter as D.
pub const AUTOTUNE_RLPF_MIN: f32 = 1.0;

/// `AUTOTUNE_RLPF_MAX`.
pub const AUTOTUNE_RLPF_MAX: f32 = 5.0;

/// Default `AUTOTUNE_MIN_D`.
pub const AUTOTUNE_MIN_D_DEFAULT: f32 = 0.0005;

/// Typical rate-P seed used by leftover views.
pub const AUTOTUNE_TUNE_P_DEFAULT: f32 = 0.15;

/// Typical rate-D seed used by leftover views.
pub const AUTOTUNE_TUNE_D_DEFAULT: f32 = 0.004;

/// Typical angle-P seed used by leftover views.
pub const AUTOTUNE_TUNE_SP_DEFAULT: f32 = 4.5;

/// Inputs for one Multi UPDATE_GAINS leftover tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateGainsView {
    /// Current Multi [`TuneType`].
    pub tune_type: TuneType,
    /// Current axis. Yaw Rate-D leftover treats `tune_d` as rLPF.
    pub axis: AxisType,
    /// `tune_*_rp` or `tune_*_sp` for the active axis / type.
    pub tune_p: f32,
    /// `tune_*_rd` or `tune_*_rLPF` for the active axis / type.
    pub tune_d: f32,
    /// `success_counter` before this tick. Upstream is `int8_t`.
    pub success_counter: i8,
    /// `ignore_next` before this tick.
    pub ignore_next: bool,
    /// `aggressiveness` after backup constrain.
    pub aggressiveness: f32,
    /// `target_rate` from `test_init`.
    pub target_rate: f32,
    /// `target_angle` from `test_init`.
    pub target_angle: f32,
    /// `test_rate_min` after the twitch.
    pub meas_rate_min: f32,
    /// `test_rate_max` after the twitch.
    pub meas_rate_max: f32,
    /// `test_angle_max` after the twitch.
    pub meas_angle_max: f32,
    /// `min_d` param.
    pub min_d: f32,
}

impl UpdateGainsView {
    /// Rate-D-up roll, mid-range P/D, no bounce recorded yet.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            tune_type: TuneType::RateDUp,
            axis: AxisType::Roll,
            tune_p: AUTOTUNE_TUNE_P_DEFAULT,
            tune_d: AUTOTUNE_TUNE_D_DEFAULT,
            success_counter: 0,
            ignore_next: false,
            aggressiveness: crate::mode_autotune::AUTOTUNE_AGGR_DEFAULT,
            target_rate: crate::mode_autotune::AUTOTUNE_TARGET_RATE_RLLPIT_CDS,
            target_angle: 2_000.0,
            meas_rate_min: 0.0,
            meas_rate_max: 0.0,
            meas_angle_max: 0.0,
            min_d: AUTOTUNE_MIN_D_DEFAULT,
        }
    }

    /// Lift the gain-math inputs off a run view.
    #[must_use]
    pub const fn from_run(view: &AutoTuneRunView) -> Self {
        Self {
            tune_type: view.tune_type,
            axis: view.axis,
            tune_p: view.tune_p,
            tune_d: view.tune_d,
            success_counter: view.success_counter,
            ignore_next: view.ignore_next,
            aggressiveness: view.aggressiveness,
            target_rate: view.target_rate,
            target_angle: view.target_angle,
            meas_rate_min: view.test_rate_min,
            meas_rate_max: view.test_rate_max,
            meas_angle_max: view.test_angle_max,
            min_d: view.min_d,
        }
    }
}

/// Leftover of one Multi UPDATE_GAINS tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateGains {
    /// `tune_*_rp` / `tune_*_sp` after the updater.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` after the updater.
    pub tune_d: f32,
    /// `success_counter` after the updater.
    pub success_counter: i8,
    /// `ignore_next` after the updater.
    pub ignore_next: bool,
    /// `LogEvent::AUTOTUNE_REACHED_LIMIT` this tick.
    pub reached_limit: bool,
    /// `mode` was written to [`TuneMode::Failed`].
    pub failed: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — Heli type on Multi.
    pub flow_of_control: bool,
    /// GCS "Min Rate D limit reached".
    pub min_rate_d_limit: bool,
    /// GCS "Rate D Gain Determination Failed".
    pub rate_d_failed: bool,
    /// GCS "Rate P Gain Determination Failed".
    pub rate_p_failed: bool,
    /// GCS "Angle P Gain Determination Failed".
    pub angle_p_failed: bool,
    /// `success_counter >= AUTOTUNE_SUCCESS_COUNT`. Sequencing is [`crate::autotune_next`].
    pub tune_type_complete: bool,
    /// Tuner `mode` after this leftover. FINISHED is [`crate::autotune_next`].
    pub mode: TuneMode,
}

fn empty_gains(view: &UpdateGainsView, mode: TuneMode) -> UpdateGains {
    UpdateGains {
        tune_p: view.tune_p,
        tune_d: view.tune_d,
        success_counter: view.success_counter,
        ignore_next: view.ignore_next,
        reached_limit: false,
        failed: false,
        flow_of_control: false,
        min_rate_d_limit: false,
        rate_d_failed: false,
        rate_p_failed: false,
        angle_p_failed: false,
        tune_type_complete: view.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8,
        mode,
    }
}

/// Rate-P / Rate-D step limits for the active axis.
#[must_use]
pub const fn rate_d_limits(axis: AxisType, min_d: f32) -> (f32, f32) {
    match axis {
        AxisType::Yaw => (AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX),
        AxisType::Roll | AxisType::Pitch | AxisType::YawD => (min_d, AUTOTUNE_RD_MAX),
    }
}

/// Yaw `RATE_P_UP` passes `fail_min_d = false`.
#[must_use]
pub const fn rate_p_fail_min_d(axis: AxisType) -> bool {
    !matches!(axis, AxisType::Yaw)
}

/// Upstream `AC_AutoTune::control_attitude` UPDATE_GAINS switch plus
/// Multi `updating_*_all` / `updating_*`.
///
/// A complete tune-type is reported on [`UpdateGains::tune_type_complete`];
/// [`crate::autotune_next`] walks backoff / next type / next axis.
#[must_use]
pub fn autotune_update_gains(view: &UpdateGainsView) -> UpdateGains {
    match view.tune_type {
        TuneType::RateDUp => updating_rate_d_up_all(view),
        TuneType::RateDDown => updating_rate_d_down_all(view),
        TuneType::RatePUp => updating_rate_p_up_all(view),
        TuneType::AnglePDown => updating_angle_p_down_all(view),
        TuneType::AnglePUp => updating_angle_p_up_all(view),
        TuneType::RateFfUp | TuneType::MaxGains => {
            let mut out = empty_gains(view, TuneMode::Tuning);
            out.flow_of_control = true;
            out
        }
        TuneType::TuneCheck => {
            let mut out = empty_gains(view, TuneMode::Tuning);
            out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
            out.tune_type_complete = true;
            out
        }
        TuneType::TuneComplete => empty_gains(view, TuneMode::Tuning),
    }
}

fn updating_rate_d_up_all(view: &UpdateGainsView) -> UpdateGains {
    let (d_min, d_max) = rate_d_limits(view.axis, view.min_d);
    updating_rate_d_up(view, d_min, d_max)
}

fn updating_rate_d_down_all(view: &UpdateGainsView) -> UpdateGains {
    let (d_min, _) = rate_d_limits(view.axis, view.min_d);
    updating_rate_d_down(view, d_min)
}

fn updating_rate_p_up_all(view: &UpdateGainsView) -> UpdateGains {
    let (d_min, _) = rate_d_limits(view.axis, view.min_d);
    updating_rate_p_up_d_down(view, d_min, rate_p_fail_min_d(view.axis))
}

fn updating_angle_p_down_all(view: &UpdateGainsView) -> UpdateGains {
    updating_angle_p_down(view)
}

fn updating_angle_p_up_all(view: &UpdateGainsView) -> UpdateGains {
    updating_angle_p_up(view)
}

/// Upstream `AC_AutoTune_Multi::updating_rate_d_up`.
#[must_use]
pub fn updating_rate_d_up(view: &UpdateGainsView, tune_d_min: f32, tune_d_max: f32) -> UpdateGains {
    let mut out = empty_gains(view, TuneMode::Tuning);
    if view.meas_rate_max > view.target_rate {
        out.tune_p -= out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p < AUTOTUNE_RP_MIN {
            out.tune_p = AUTOTUNE_RP_MIN;
            out.tune_d -= out.tune_d * AUTOTUNE_RD_STEP;
            if out.tune_d <= tune_d_min {
                out.tune_d = tune_d_min;
                out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
                out.reached_limit = true;
                out.min_rate_d_limit = true;
            }
        }
    } else if view.meas_rate_max < view.target_rate * (1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)
        && out.tune_p <= AUTOTUNE_RP_MAX
    {
        out.tune_p += out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p >= AUTOTUNE_RP_MAX {
            out.tune_p = AUTOTUNE_RP_MAX;
            out.reached_limit = true;
        }
    } else if view.meas_rate_max - view.meas_rate_min > view.meas_rate_max * view.aggressiveness {
        out.ignore_next = true;
        out.success_counter += 1;
    } else if !out.ignore_next {
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_d += out.tune_d * AUTOTUNE_RD_STEP * 2.0;
        if out.tune_d >= tune_d_max {
            out.tune_d = tune_d_max;
            out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
            out.reached_limit = true;
        }
    } else {
        out.ignore_next = false;
    }
    out.tune_type_complete = out.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8;
    out
}

/// Upstream `AC_AutoTune_Multi::updating_rate_d_down`.
#[must_use]
pub fn updating_rate_d_down(view: &UpdateGainsView, tune_d_min: f32) -> UpdateGains {
    let mut out = empty_gains(view, TuneMode::Tuning);
    if view.meas_rate_max > view.target_rate {
        out.tune_p -= out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p < AUTOTUNE_RP_MIN {
            out.tune_p = AUTOTUNE_RP_MIN;
            out.tune_d -= out.tune_d * AUTOTUNE_RD_STEP;
            if out.tune_d <= tune_d_min {
                out.tune_d = tune_d_min;
                out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
                out.reached_limit = true;
                out.min_rate_d_limit = true;
            }
        }
    } else if view.meas_rate_max < view.target_rate * (1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)
        && out.tune_p <= AUTOTUNE_RP_MAX
    {
        out.tune_p += out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p >= AUTOTUNE_RP_MAX {
            out.tune_p = AUTOTUNE_RP_MAX;
            out.reached_limit = true;
        }
    } else if view.meas_rate_max - view.meas_rate_min < view.meas_rate_max * view.aggressiveness {
        if !out.ignore_next {
            out.success_counter += 1;
        } else {
            out.ignore_next = false;
        }
    } else {
        out.ignore_next = true;
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_d -= out.tune_d * AUTOTUNE_RD_STEP;
        if out.tune_d <= tune_d_min {
            out.tune_d = tune_d_min;
            out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
            out.reached_limit = true;
            out.min_rate_d_limit = true;
        }
    }
    out.tune_type_complete = out.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8;
    out
}

/// Upstream `AC_AutoTune_Multi::updating_rate_p_up_d_down`.
#[must_use]
pub fn updating_rate_p_up_d_down(
    view: &UpdateGainsView,
    tune_d_min: f32,
    fail_min_d: bool,
) -> UpdateGains {
    let mut out = empty_gains(view, TuneMode::Tuning);
    if view.meas_rate_max > view.target_rate * (1.0 + 0.5 * view.aggressiveness) {
        out.ignore_next = true;
        out.success_counter += 1;
    } else if view.meas_rate_max < view.target_rate
        && view.meas_rate_max > view.target_rate * (1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)
        && view.meas_rate_max - view.meas_rate_min > view.meas_rate_max * view.aggressiveness
        && out.tune_d > tune_d_min
    {
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_d -= out.tune_d * AUTOTUNE_RD_STEP;
        if out.tune_d <= tune_d_min {
            out.tune_d = tune_d_min;
            out.reached_limit = true;
            if fail_min_d {
                out.rate_d_failed = true;
                out.failed = true;
                out.mode = TuneMode::Failed;
            }
        }
        out.tune_p -= out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p <= AUTOTUNE_RP_MIN {
            out.tune_p = AUTOTUNE_RP_MIN;
            out.rate_p_failed = true;
            out.failed = true;
            out.mode = TuneMode::Failed;
        }
    } else if !out.ignore_next {
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_p += out.tune_p * AUTOTUNE_RP_STEP;
        if out.tune_p >= AUTOTUNE_RP_MAX {
            out.tune_p = AUTOTUNE_RP_MAX;
            out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
            out.reached_limit = true;
        }
    } else {
        out.ignore_next = false;
    }
    out.tune_type_complete = out.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8;
    out
}

/// Upstream `AC_AutoTune_Multi::updating_angle_p_down`.
///
/// `meas_rate_min` / `meas_rate_max` are in the C++ signature and unused.
#[must_use]
pub fn updating_angle_p_down(view: &UpdateGainsView) -> UpdateGains {
    let mut out = empty_gains(view, TuneMode::Tuning);
    if view.meas_angle_max < view.target_angle * (1.0 + 0.5 * view.aggressiveness) {
        if !out.ignore_next {
            out.success_counter += 1;
        } else {
            out.ignore_next = false;
        }
    } else {
        out.ignore_next = true;
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_p -= out.tune_p * AUTOTUNE_SP_STEP;
        if out.tune_p <= AUTOTUNE_SP_MIN {
            out.tune_p = AUTOTUNE_SP_MIN;
            out.reached_limit = true;
            out.angle_p_failed = true;
            out.failed = true;
            out.mode = TuneMode::Failed;
        }
    }
    out.tune_type_complete = out.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8;
    out
}

/// Upstream `AC_AutoTune_Multi::updating_angle_p_up`.
#[must_use]
pub fn updating_angle_p_up(view: &UpdateGainsView) -> UpdateGains {
    let mut out = empty_gains(view, TuneMode::Tuning);
    if view.meas_angle_max > view.target_angle * (1.0 + 0.5 * view.aggressiveness)
        || (view.meas_angle_max > view.target_angle
            && view.meas_rate_min < -view.meas_rate_max * view.aggressiveness)
    {
        out.ignore_next = true;
        out.success_counter += 1;
    } else if !out.ignore_next {
        if out.success_counter > 0 {
            out.success_counter -= 1;
        }
        out.tune_p += out.tune_p * AUTOTUNE_SP_STEP;
        if out.tune_p >= AUTOTUNE_SP_MAX {
            out.tune_p = AUTOTUNE_SP_MAX;
            out.success_counter = AUTOTUNE_SUCCESS_COUNT as i8;
            out.reached_limit = true;
        }
    } else {
        out.ignore_next = false;
    }
    out.tune_type_complete = out.success_counter >= AUTOTUNE_SUCCESS_COUNT as i8;
    out
}
