//! Multi `next_tune_type` / next-axis / backoff leftover, upstream `AC_AutoTune`.
//!
//! Tracked as **COP-027**. After UPDATE_GAINS freezes a tune-type
//! (`success_counter >= AUTOTUNE_SUCCESS_COUNT`), upstream applies
//! [`set_tuning_gains_with_backoff`], walks [`next_tune_type`], and
//! either starts the next Multi step or the next enabled axis. Multi
//! `reset_update_gain_variables` is a no-op. Heli sequence / backoff
//! stay out of scope. The rest of `AC_AutoTune_Multi` (load/save
//! gains, `test_init`, poshold lean, GCS report) stays leftover.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::{
    pitch_enabled, yaw_d_enabled, yaw_enabled, AxisType, GainType, TuneMode, TuneType,
    AUTOTUNE_AGGR_DEFAULT, AUTOTUNE_AXIS_BITMASK_DEFAULT, AUTOTUNE_AXIS_BITMASK_PITCH,
    AUTOTUNE_AXIS_BITMASK_ROLL, AUTOTUNE_AXIS_BITMASK_YAW, AUTOTUNE_AXIS_BITMASK_YAW_D,
    AUTOTUNE_MESSAGE_SUCCESS,
};
use ap_math::scalar::{cd_to_rad, constrain_value};

/// Length of Multi `tune_seq[]`.
pub const AUTOTUNE_TUNE_SEQ_LEN: usize = 6;

/// Default `AUTOTUNE_GMBK`.
pub const AUTOTUNE_GAIN_BACKOFF_DEFAULT: f32 = 0.25;

/// Upper clamp on `gain_backoff` (`set_and_save_ifchanged`).
pub const AUTOTUNE_GAIN_BACKOFF_MAX: f32 = 0.5;

/// `AUTOTUNE_RP_ACCEL_MIN`.
pub const AUTOTUNE_RP_ACCEL_MIN: f32 = 4_000.0;

/// `AUTOTUNE_Y_ACCEL_MIN`.
pub const AUTOTUNE_Y_ACCEL_MIN: f32 = 1_000.0;

/// `AUTOTUNE_ACCEL_RP_BACKOFF`.
pub const AUTOTUNE_ACCEL_RP_BACKOFF: f32 = 1.0;

/// `AUTOTUNE_ACCEL_Y_BACKOFF`.
pub const AUTOTUNE_ACCEL_Y_BACKOFF: f32 = 1.0;

/// Multi `AC_AutoTune_Multi::set_tune_sequence`.
#[must_use]
pub const fn multi_tune_sequence() -> [TuneType; AUTOTUNE_TUNE_SEQ_LEN] {
    [
        TuneType::RateDUp,
        TuneType::RateDDown,
        TuneType::RatePUp,
        TuneType::AnglePDown,
        TuneType::AnglePUp,
        TuneType::TuneComplete,
    ]
}

/// Leftover of `AC_AutoTune::next_tune_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NextTuneType {
    /// `curr_tune_type` after the walk.
    pub tune_type: TuneType,
    /// `tune_seq_index` after the walk.
    pub tune_seq_index: u8,
    /// `set_tune_sequence` ran (`reset == true`).
    pub sequence_reset: bool,
}

/// Upstream `AC_AutoTune::next_tune_type`.
///
/// `reset` rebuilds the Multi sequence and starts at index 0.
/// `TUNE_COMPLETE` without reset is left alone so the caller can
/// start the next axis or finish. Otherwise the index advances and
/// the Multi table is read.
#[must_use]
pub fn next_tune_type(curr: TuneType, reset: bool, seq_index: u8) -> NextTuneType {
    let seq = multi_tune_sequence();
    if reset {
        return NextTuneType {
            tune_type: seq[0],
            tune_seq_index: 0,
            sequence_reset: true,
        };
    }
    if matches!(curr, TuneType::TuneComplete) {
        return NextTuneType {
            tune_type: TuneType::TuneComplete,
            tune_seq_index: seq_index,
            sequence_reset: false,
        };
    }
    let next_i = (seq_index as usize + 1).min(seq.len() - 1);
    NextTuneType {
        tune_type: seq[next_i],
        tune_seq_index: next_i as u8,
        sequence_reset: false,
    }
}

/// Leftover of the UPDATE_GAINS next-axis switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NextAxis {
    /// Axis after the switch. Unchanged when this was the last axis.
    pub axis: AxisType,
    /// `axes_completed` with the finished axis bit or-ed in.
    pub axes_completed: u8,
    /// No further enabled axis — caller writes [`TuneMode::Finished`].
    pub complete: bool,
}

/// Axis-complete bitmask for `axis`.
#[must_use]
pub const fn axis_complete_bit(axis: AxisType) -> u8 {
    match axis {
        AxisType::Roll => AUTOTUNE_AXIS_BITMASK_ROLL,
        AxisType::Pitch => AUTOTUNE_AXIS_BITMASK_PITCH,
        AxisType::Yaw => AUTOTUNE_AXIS_BITMASK_YAW,
        AxisType::YawD => AUTOTUNE_AXIS_BITMASK_YAW_D,
    }
}

/// UPDATE_GAINS next-axis switch after `TUNE_COMPLETE`.
#[must_use]
pub const fn next_axis(axis: AxisType, axis_bitmask: u8, axes_completed: u8) -> NextAxis {
    let completed = axes_completed | axis_complete_bit(axis);
    match axis {
        AxisType::Roll => {
            if pitch_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::Pitch,
                    axes_completed: completed,
                    complete: false,
                }
            } else if yaw_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::Yaw,
                    axes_completed: completed,
                    complete: false,
                }
            } else if yaw_d_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::YawD,
                    axes_completed: completed,
                    complete: false,
                }
            } else {
                NextAxis {
                    axis,
                    axes_completed: completed,
                    complete: true,
                }
            }
        }
        AxisType::Pitch => {
            if yaw_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::Yaw,
                    axes_completed: completed,
                    complete: false,
                }
            } else if yaw_d_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::YawD,
                    axes_completed: completed,
                    complete: false,
                }
            } else {
                NextAxis {
                    axis,
                    axes_completed: completed,
                    complete: true,
                }
            }
        }
        AxisType::Yaw => {
            if yaw_d_enabled(axis_bitmask) {
                NextAxis {
                    axis: AxisType::YawD,
                    axes_completed: completed,
                    complete: false,
                }
            } else {
                NextAxis {
                    axis,
                    axes_completed: completed,
                    complete: true,
                }
            }
        }
        AxisType::YawD => NextAxis {
            axis,
            axes_completed: completed,
            complete: true,
        },
    }
}

/// Inputs for Multi `set_tuning_gains_with_backoff`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackoffView {
    /// Tune-type that just completed.
    pub tune_type: TuneType,
    /// Axis that just completed.
    pub axis: AxisType,
    /// `tune_*_rp` or `tune_*_sp` before backoff.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` before backoff.
    pub tune_d: f32,
    /// `tune_*_accel_radss` before backoff.
    pub tune_accel_radss: f32,
    /// `gain_backoff` param before the [0, 0.5] clamp.
    pub gain_backoff: f32,
    /// `aggressiveness` after backup constrain.
    pub aggressiveness: f32,
    /// `test_accel_max_cdss` from the twitch.
    pub test_accel_max_cdss: f32,
}

impl BackoffView {
    /// Rate-P-up roll, default GMBK / AGGR.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            tune_type: TuneType::RatePUp,
            axis: AxisType::Roll,
            tune_p: 0.15,
            tune_d: 0.004,
            tune_accel_radss: 0.0,
            gain_backoff: AUTOTUNE_GAIN_BACKOFF_DEFAULT,
            aggressiveness: AUTOTUNE_AGGR_DEFAULT,
            test_accel_max_cdss: 0.0,
        }
    }
}

/// Leftover of one Multi `set_tuning_gains_with_backoff`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Backoff {
    /// `tune_*_rp` / `tune_*_sp` after backoff.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` after backoff.
    pub tune_d: f32,
    /// `tune_*_accel_radss` after ANGLE_P_UP.
    pub tune_accel_radss: f32,
    /// `gain_backoff` after the [0, 0.5] clamp.
    pub gain_backoff: f32,
    /// Rate-P or angle-P backoff wrote the gains.
    pub applied: bool,
    /// Heli type on Multi (`RATE_FF_UP` / `MAX_GAINS` / `TUNE_CHECK`).
    pub flow_of_control: bool,
}

/// Multi `AC_AutoTune_Multi::set_tuning_gains_with_backoff`.
#[must_use]
pub fn set_tuning_gains_with_backoff(view: &BackoffView) -> Backoff {
    let gain_backoff = constrain_value(view.gain_backoff, 0.0, AUTOTUNE_GAIN_BACKOFF_MAX);
    let scale = 1.0 - gain_backoff;
    match view.tune_type {
        TuneType::RateDUp | TuneType::RateDDown | TuneType::AnglePDown | TuneType::TuneComplete => {
            Backoff {
                tune_p: view.tune_p,
                tune_d: view.tune_d,
                tune_accel_radss: view.tune_accel_radss,
                gain_backoff,
                applied: false,
                flow_of_control: false,
            }
        }
        TuneType::RatePUp => {
            let (tune_p, tune_d) = match view.axis {
                AxisType::Yaw => (view.tune_p * scale, view.tune_d),
                AxisType::Roll | AxisType::Pitch | AxisType::YawD => {
                    (view.tune_p * scale, view.tune_d * scale)
                }
            };
            Backoff {
                tune_p,
                tune_d,
                tune_accel_radss: view.tune_accel_radss,
                gain_backoff,
                applied: true,
                flow_of_control: false,
            }
        }
        TuneType::AnglePUp => {
            let tune_p = view.tune_p * scale * (1.0 - view.aggressiveness);
            let (min_cd, accel_backoff) = match view.axis {
                AxisType::Roll | AxisType::Pitch => {
                    (AUTOTUNE_RP_ACCEL_MIN, AUTOTUNE_ACCEL_RP_BACKOFF)
                }
                AxisType::Yaw | AxisType::YawD => (AUTOTUNE_Y_ACCEL_MIN, AUTOTUNE_ACCEL_Y_BACKOFF),
            };
            let accel_cd = view.test_accel_max_cdss * accel_backoff;
            let tune_accel_radss = cd_to_rad(if accel_cd > min_cd { accel_cd } else { min_cd });
            Backoff {
                tune_p,
                tune_d: view.tune_d,
                tune_accel_radss,
                gain_backoff,
                applied: true,
                flow_of_control: false,
            }
        }
        TuneType::RateFfUp | TuneType::MaxGains | TuneType::TuneCheck => Backoff {
            tune_p: view.tune_p,
            tune_d: view.tune_d,
            tune_accel_radss: view.tune_accel_radss,
            gain_backoff,
            applied: false,
            flow_of_control: true,
        },
    }
}

/// Inputs for the UPDATE_GAINS success walk (backoff + next type + next axis).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdvanceView {
    /// Current Multi [`TuneType`].
    pub tune_type: TuneType,
    /// Current axis.
    pub axis: AxisType,
    /// `AUTOTUNE_AXES` mask.
    pub axis_bitmask: u8,
    /// `axes_completed` before this walk.
    pub axes_completed: u8,
    /// `tune_seq_index` before this walk.
    pub tune_seq_index: u8,
    /// `tune_*_rp` / `tune_*_sp` after UPDATE_GAINS leftover.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` after UPDATE_GAINS leftover.
    pub tune_d: f32,
    /// `tune_*_accel_radss` before backoff.
    pub tune_accel_radss: f32,
    /// `gain_backoff` before the clamp.
    pub gain_backoff: f32,
    /// `aggressiveness` after backup constrain.
    pub aggressiveness: f32,
    /// `test_accel_max_cdss` from the twitch.
    pub test_accel_max_cdss: f32,
    /// `success_counter >= AUTOTUNE_SUCCESS_COUNT`.
    pub tune_type_complete: bool,
}

impl AdvanceView {
    /// Rate-D-up roll, default mask, just completed.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            tune_type: TuneType::RateDUp,
            axis: AxisType::Roll,
            axis_bitmask: AUTOTUNE_AXIS_BITMASK_DEFAULT,
            axes_completed: 0,
            tune_seq_index: 0,
            tune_p: 0.15,
            tune_d: 0.004,
            tune_accel_radss: 0.0,
            gain_backoff: AUTOTUNE_GAIN_BACKOFF_DEFAULT,
            aggressiveness: AUTOTUNE_AGGR_DEFAULT,
            test_accel_max_cdss: 0.0,
            tune_type_complete: true,
        }
    }

    /// Lift the walk off a run view plus UPDATE_GAINS leftover gains.
    #[must_use]
    pub const fn from_run(
        view: &crate::mode_autotune::AutoTuneRunView,
        tune_p: f32,
        tune_d: f32,
        tune_type_complete: bool,
    ) -> Self {
        Self {
            tune_type: view.tune_type,
            axis: view.axis,
            axis_bitmask: view.axis_bitmask,
            axes_completed: view.axes_completed,
            tune_seq_index: view.tune_seq_index,
            tune_p,
            tune_d,
            tune_accel_radss: view.tune_accel_radss,
            gain_backoff: view.gain_backoff,
            aggressiveness: view.aggressiveness,
            test_accel_max_cdss: view.test_accel_max_cdss,
            tune_type_complete,
        }
    }
}

/// Leftover of the UPDATE_GAINS success walk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Advance {
    /// `success_counter` after a freeze (`0`) or unchanged.
    pub success_counter: i8,
    /// `step_scaler` after a freeze (`1.0`) or unchanged.
    pub step_scaler: f32,
    /// `tune_type` after [`next_tune_type`].
    pub tune_type: TuneType,
    /// `tune_seq_index` after the walk.
    pub tune_seq_index: u8,
    /// Axis after a completed sequence, else unchanged.
    pub axis: AxisType,
    /// `axes_completed` after a completed sequence, else unchanged.
    pub axes_completed: u8,
    /// `tune_*_rp` / `tune_*_sp` after backoff.
    pub tune_p: f32,
    /// `tune_*_rd` / `tune_*_rLPF` after backoff.
    pub tune_d: f32,
    /// `tune_*_accel_radss` after ANGLE_P_UP backoff.
    pub tune_accel_radss: f32,
    /// `gain_backoff` after the clamp.
    pub gain_backoff: f32,
    /// Rate-P or angle-P backoff wrote the gains.
    pub backoff_applied: bool,
    /// Heli type on Multi backoff.
    pub flow_of_control: bool,
    /// `report_final_gains` ran (sequence hit `TUNE_COMPLETE`).
    pub reported_final_gains: bool,
    /// `AP_Notify::events.autotune_next_axis`.
    pub next_axis: bool,
    /// Last enabled axis finished.
    pub complete: bool,
    /// Tuner `mode` after the walk.
    pub mode: TuneMode,
    /// `load_gains(ORIGINAL)` on the complete path (ABORT then
    /// overwrites with INTRA_TEST on the same tick).
    pub loaded_gains: Option<GainType>,
    /// `update_gcs(AUTOTUNE_MESSAGE_SUCCESS)` on the complete path.
    pub gcs_message: Option<u8>,
    /// `AP_Notify::events.autotune_complete`.
    pub autotune_complete: bool,
}

fn idle_advance(view: &AdvanceView) -> Advance {
    Advance {
        success_counter: 0,
        step_scaler: 1.0,
        tune_type: view.tune_type,
        tune_seq_index: view.tune_seq_index,
        axis: view.axis,
        axes_completed: view.axes_completed,
        tune_p: view.tune_p,
        tune_d: view.tune_d,
        tune_accel_radss: view.tune_accel_radss,
        gain_backoff: constrain_value(view.gain_backoff, 0.0, AUTOTUNE_GAIN_BACKOFF_MAX),
        backoff_applied: false,
        flow_of_control: false,
        reported_final_gains: false,
        next_axis: false,
        complete: false,
        mode: TuneMode::Tuning,
        loaded_gains: None,
        gcs_message: None,
        autotune_complete: false,
    }
}

/// UPDATE_GAINS success walk: backoff, next type, then next axis.
///
/// A non-complete tick is a no-op (callers skip this leftover).
#[must_use]
pub fn autotune_advance(view: &AdvanceView) -> Advance {
    if !view.tune_type_complete {
        let mut out = idle_advance(view);
        out.success_counter = 0;
        return out;
    }

    let backed = set_tuning_gains_with_backoff(&BackoffView {
        tune_type: view.tune_type,
        axis: view.axis,
        tune_p: view.tune_p,
        tune_d: view.tune_d,
        tune_accel_radss: view.tune_accel_radss,
        gain_backoff: view.gain_backoff,
        aggressiveness: view.aggressiveness,
        test_accel_max_cdss: view.test_accel_max_cdss,
    });
    let stepped = next_tune_type(view.tune_type, false, view.tune_seq_index);

    let mut out = Advance {
        success_counter: 0,
        step_scaler: 1.0,
        tune_type: stepped.tune_type,
        tune_seq_index: stepped.tune_seq_index,
        axis: view.axis,
        axes_completed: view.axes_completed,
        tune_p: backed.tune_p,
        tune_d: backed.tune_d,
        tune_accel_radss: backed.tune_accel_radss,
        gain_backoff: backed.gain_backoff,
        backoff_applied: backed.applied,
        flow_of_control: backed.flow_of_control,
        reported_final_gains: false,
        next_axis: false,
        complete: false,
        mode: TuneMode::Tuning,
        loaded_gains: None,
        gcs_message: None,
        autotune_complete: false,
    };

    if stepped.tune_type == TuneType::TuneComplete {
        let reset = next_tune_type(TuneType::TuneComplete, true, stepped.tune_seq_index);
        out.tune_type = reset.tune_type;
        out.tune_seq_index = reset.tune_seq_index;
        out.reported_final_gains = true;
        let axis = next_axis(view.axis, view.axis_bitmask, view.axes_completed);
        out.axis = axis.axis;
        out.axes_completed = axis.axes_completed;
        out.complete = axis.complete;
        if axis.complete {
            out.mode = TuneMode::Finished;
            out.loaded_gains = Some(GainType::Original);
            out.gcs_message = Some(AUTOTUNE_MESSAGE_SUCCESS);
            out.autotune_complete = true;
        } else {
            out.next_axis = true;
        }
    }
    out
}
