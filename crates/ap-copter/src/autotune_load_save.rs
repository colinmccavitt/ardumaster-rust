//! Multi load / save leftover, upstream `AC_AutoTune_Multi`.
//!
//! Tracked as **COP-027**. The Copter wrapper already catalogues
//! which [`GainType`] `load_gains` selected. This leftover is the
//! Multi backup / load / save math that writes the rate and angle
//! PIDs: `backup_gains_and_initialise` after the base sequencer,
//! `load_orig_gains` / `load_tuned_gains` / `load_intra_test_gains` /
//! `load_test_gains`, the `loaded_gains` skip in `load_gains`,
//! `save_tuning_gains`, and the `stop` / `disarmed` gates that
//! choose original vs save-or-reset.
//!
//! EEPROM `PID::save_gains` / `save_accel_*` stay a write flag.
//! GCS leftover lives in [`crate::autotune_gcs`] — this tick still records
//! the message id. Heli load/save is out of scope.
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::{
    pitch_enabled, roll_enabled, yaw_d_enabled, yaw_enabled, AxisType, GainType,
    AUTOTUNE_AXIS_BITMASK_DEFAULT, AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_ROLL,
    AUTOTUNE_AXIS_BITMASK_YAW, AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_SAVED_GAINS,
    AUTOTUNE_MESSAGE_STOPPED,
};
use ap_math::scalar::{constrain_value, is_zero};

/// `AUTOTUNE_PI_RATIO_FOR_TESTING` — intra-test I is 10× smaller than P.
pub const AUTOTUNE_PI_RATIO_FOR_TESTING: f32 = 0.1;

/// `AUTOTUNE_PI_RATIO_FINAL` — roll/pitch I equals P after the tune.
pub const AUTOTUNE_PI_RATIO_FINAL: f32 = 1.0;

/// `AUTOTUNE_YAW_PI_RATIO_FINAL` — yaw I is 10× smaller than P after the tune.
pub const AUTOTUNE_YAW_PI_RATIO_FINAL: f32 = 0.1;

/// `AUTOTUNE_FLTE_MIN` — yaw error-filter floor when backup finds zero.
pub const AUTOTUNE_FLTE_MIN: f32 = 2.5;

/// Backup constrain low end for `aggressiveness`.
pub const AUTOTUNE_AGGR_MIN: f32 = 0.05;

/// Backup constrain high end for `aggressiveness`.
pub const AUTOTUNE_AGGR_MAX: f32 = 0.2;

/// `load_test_gains` I is `tune_rp * 0.01`.
pub const AUTOTUNE_TEST_I_RATIO: f32 = 0.01;

/// Typical live rate-P used by leftover views.
pub const AUTOTUNE_LIVE_RP_DEFAULT: f32 = 0.15;

/// Typical live rate-I used by leftover views. Kept off `rp` so tests
/// can tell a restored I from a ratio rewrite.
pub const AUTOTUNE_LIVE_RI_DEFAULT: f32 = 0.135;

/// Typical live rate-D used by leftover views.
pub const AUTOTUNE_LIVE_RD_DEFAULT: f32 = 0.004;

/// Typical live angle-P used by leftover views.
pub const AUTOTUNE_LIVE_SP_DEFAULT: f32 = 4.5;

/// Typical live target-filter Hz.
pub const AUTOTUNE_LIVE_FLTT_DEFAULT: f32 = 20.0;

/// Typical live yaw error-filter Hz (above [`AUTOTUNE_FLTE_MIN`]).
pub const AUTOTUNE_LIVE_FLTE_DEFAULT: f32 = 5.0;

/// Typical live accel limit, rad/s/s.
pub const AUTOTUNE_LIVE_ACCEL_DEFAULT: f32 = 1.2;

/// One axis of live attitude-control PID reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiveAxis {
    /// Rate `kP()`.
    pub rp: f32,
    /// Rate `kI()`.
    pub ri: f32,
    /// Rate `kD()`.
    pub rd: f32,
    /// Rate `ff()`.
    pub rff: f32,
    /// Rate `kDff()`.
    pub dff: f32,
    /// Rate `filt_T_hz()`.
    pub fltt: f32,
    /// Rate `slew_limit()`.
    pub smax: f32,
    /// Rate `filt_E_hz()` — yaw rLPF.
    pub flte: f32,
    /// Angle `kP()`.
    pub sp: f32,
    /// `get_accel_*_max_radss()`.
    pub accel_radss: f32,
}

impl LiveAxis {
    /// Mid-range live axis used by leftover views.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            rp: AUTOTUNE_LIVE_RP_DEFAULT,
            ri: AUTOTUNE_LIVE_RI_DEFAULT,
            rd: AUTOTUNE_LIVE_RD_DEFAULT,
            rff: 0.0,
            dff: 0.0,
            fltt: AUTOTUNE_LIVE_FLTT_DEFAULT,
            smax: 0.0,
            flte: AUTOTUNE_LIVE_FLTE_DEFAULT,
            sp: AUTOTUNE_LIVE_SP_DEFAULT,
            accel_radss: AUTOTUNE_LIVE_ACCEL_DEFAULT,
        }
    }
}

/// Backup `orig_*` snapshot for one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrigAxis {
    /// `orig_*_rp`.
    pub rp: f32,
    /// `orig_*_ri`.
    pub ri: f32,
    /// `orig_*_rd`.
    pub rd: f32,
    /// `orig_*_rff`.
    pub rff: f32,
    /// `orig_*_dff`.
    pub dff: f32,
    /// `orig_*_fltt`.
    pub fltt: f32,
    /// `orig_*_smax`.
    pub smax: f32,
    /// `orig_yaw_rLPF` (unused on roll/pitch).
    pub r_lpf: f32,
    /// `orig_*_sp`.
    pub sp: f32,
    /// `orig_*_accel_radss`.
    pub accel_radss: f32,
}

impl OrigAxis {
    /// Copy a live axis into the orig bank. `r_lpf` is `flte`.
    #[must_use]
    pub const fn from_live(live: LiveAxis) -> Self {
        Self {
            rp: live.rp,
            ri: live.ri,
            rd: live.rd,
            rff: live.rff,
            dff: live.dff,
            fltt: live.fltt,
            smax: live.smax,
            r_lpf: live.flte,
            sp: live.sp,
            accel_radss: live.accel_radss,
        }
    }
}

/// Backup `tune_*` seed for one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TuneAxis {
    /// `tune_*_rp`.
    pub rp: f32,
    /// `tune_*_rd`.
    pub rd: f32,
    /// `tune_yaw_rLPF` (unused on roll/pitch).
    pub r_lpf: f32,
    /// `tune_*_sp`.
    pub sp: f32,
    /// `tune_*_accel_radss`.
    pub accel_radss: f32,
}

impl TuneAxis {
    /// Seed tune values from the live axis. Yaw floors are applied
    /// by [`backup_multi_gains`], not here.
    #[must_use]
    pub const fn from_live(live: LiveAxis) -> Self {
        Self {
            rp: live.rp,
            rd: live.rd,
            r_lpf: live.flte,
            sp: live.sp,
            accel_radss: live.accel_radss,
        }
    }
}

/// What one load/save path writes onto an axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisWrite {
    /// This axis received at least one setter.
    pub written: bool,
    /// Rate `kP`.
    pub rp: f32,
    /// Rate `kI`.
    pub ri: f32,
    /// Rate `kD`.
    pub rd: f32,
    /// Rate `kD` was assigned (yaw RATE vs YAW_D differs).
    pub rd_written: bool,
    /// Rate `ff`.
    pub rff: f32,
    /// Rate `kDff`.
    pub dff: f32,
    /// Rate `filt_T_hz` when written.
    pub fltt: Option<f32>,
    /// Rate `slew_limit` when written.
    pub smax: Option<f32>,
    /// Rate `filt_E_hz` when written.
    pub flte: Option<f32>,
    /// Angle `kP`.
    pub sp: f32,
    /// Accel max when written.
    pub accel_radss: Option<f32>,
    /// `PID::save_gains` + `save_accel_*` on the save path.
    pub saved: bool,
}

impl AxisWrite {
    /// Empty write — axis was skipped.
    #[must_use]
    pub const fn skipped() -> Self {
        Self {
            written: false,
            rp: 0.0,
            ri: 0.0,
            rd: 0.0,
            rd_written: false,
            rff: 0.0,
            dff: 0.0,
            fltt: None,
            smax: None,
            flte: None,
            sp: 0.0,
            accel_radss: None,
            saved: false,
        }
    }
}

/// Inputs for Multi `backup_gains_and_initialise` after the base leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackupView {
    /// `aggressiveness` before the 0.05..0.2 constrain.
    pub aggressiveness: f32,
    /// `axis_bitmask`.
    pub axis_bitmask: u8,
    /// `min_d` param.
    pub min_d: f32,
    /// `attitude_control->get_bf_feedforward()`.
    pub bf_feedforward: bool,
    /// Live roll PIDs.
    pub roll: LiveAxis,
    /// Live pitch PIDs.
    pub pitch: LiveAxis,
    /// Live yaw PIDs.
    pub yaw: LiveAxis,
}

impl BackupView {
    /// Default-axes live PIDs, mid aggressiveness.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            aggressiveness: crate::mode_autotune::AUTOTUNE_AGGR_DEFAULT,
            axis_bitmask: AUTOTUNE_AXIS_BITMASK_DEFAULT,
            min_d: crate::autotune_update_gains::AUTOTUNE_MIN_D_DEFAULT,
            bf_feedforward: true,
            roll: LiveAxis::typical(),
            pitch: LiveAxis::typical(),
            yaw: LiveAxis::typical(),
        }
    }
}

/// Multi backup leftover after the base sequencer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackupGains {
    /// Constrained `aggressiveness`.
    pub aggressiveness: f32,
    /// `orig_bf_feedforward`.
    pub orig_bf_feedforward: bool,
    /// `orig_roll_*`.
    pub orig_roll: OrigAxis,
    /// `orig_pitch_*`.
    pub orig_pitch: OrigAxis,
    /// `orig_yaw_*`.
    pub orig_yaw: OrigAxis,
    /// `tune_roll_*`.
    pub tune_roll: TuneAxis,
    /// `tune_pitch_*`.
    pub tune_pitch: TuneAxis,
    /// `tune_yaw_*`.
    pub tune_yaw: TuneAxis,
    /// `yaw_d_enabled && is_zero(tune_yaw_rd)` seeded `min_d`.
    pub yaw_rd_seeded: bool,
    /// `yaw_enabled && is_zero(tune_yaw_rLPF)` seeded [`AUTOTUNE_FLTE_MIN`].
    pub yaw_rlpf_seeded: bool,
}

/// Inputs shared by Multi `load_*` / `save_tuning_gains`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadView {
    /// `axis_bitmask`.
    pub axis_bitmask: u8,
    /// `axes_completed`.
    pub axes_completed: u8,
    /// Current twitch axis — `load_test_gains` only.
    pub axis: AxisType,
    /// `orig_bf_feedforward`.
    pub orig_bf_feedforward: bool,
    /// Live `get_bf_feedforward()` for tuned/save.
    pub live_bf_feedforward: bool,
    /// `orig_roll_*`.
    pub orig_roll: OrigAxis,
    /// `orig_pitch_*`.
    pub orig_pitch: OrigAxis,
    /// `orig_yaw_*`.
    pub orig_yaw: OrigAxis,
    /// `tune_roll_*`.
    pub tune_roll: TuneAxis,
    /// `tune_pitch_*`.
    pub tune_pitch: TuneAxis,
    /// `tune_yaw_*`.
    pub tune_yaw: TuneAxis,
}

impl LoadView {
    /// Bank from a typical backup of default live PIDs.
    #[must_use]
    pub fn typical() -> Self {
        let backup = backup_multi_gains(&BackupView::typical());
        Self {
            axis_bitmask: AUTOTUNE_AXIS_BITMASK_DEFAULT,
            axes_completed: AUTOTUNE_AXIS_BITMASK_ROLL
                | AUTOTUNE_AXIS_BITMASK_PITCH
                | AUTOTUNE_AXIS_BITMASK_YAW,
            axis: AxisType::Roll,
            orig_bf_feedforward: backup.orig_bf_feedforward,
            live_bf_feedforward: true,
            orig_roll: backup.orig_roll,
            orig_pitch: backup.orig_pitch,
            orig_yaw: backup.orig_yaw,
            tune_roll: backup.tune_roll,
            tune_pitch: backup.tune_pitch,
            tune_yaw: backup.tune_yaw,
        }
    }
}

/// What a Multi `load_*` leftover writes onto the attitude controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadedGains {
    /// `use_sqrt_controller`.
    pub use_sqrt_controller: bool,
    /// `bf_feedforward` when written.
    pub bf_feedforward: Option<bool>,
    /// Tuned/save path zeroed roll accel because live FF was off.
    pub accel_roll_forced_zero: bool,
    /// Tuned/save path zeroed pitch accel because live FF was off.
    pub accel_pitch_forced_zero: bool,
    /// Roll setters.
    pub roll: AxisWrite,
    /// Pitch setters.
    pub pitch: AxisWrite,
    /// Yaw setters.
    pub yaw: AxisWrite,
}

/// `load_gains` leftover including the already-loaded skip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadGains {
    /// `loaded_gains == gain_type` — no Multi load ran.
    pub skipped: bool,
    /// Requested type, even on a skip.
    pub gain_type: GainType,
    /// Multi load writes. `None` on a skip.
    pub loaded: Option<LoadedGains>,
}

/// `save_tuning_gains` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SaveTuning {
    /// `axes_completed == 0` early return.
    pub skipped: bool,
    /// `bf_feedforward_save(true)` because live FF was off.
    pub bf_feedforward_saved: bool,
    /// Roll/pitch accel-0 save because live FF was off.
    pub accel_rp_saved_zero: bool,
    /// Roll setters + `save_gains`.
    pub roll: AxisWrite,
    /// Pitch setters + `save_gains`.
    pub pitch: AxisWrite,
    /// Yaw setters + `save_gains`.
    pub yaw: AxisWrite,
    /// `orig_roll_*` after the resave, when roll saved.
    pub orig_roll: Option<OrigAxis>,
    /// `orig_pitch_*` after the resave, when pitch saved.
    pub orig_pitch: Option<OrigAxis>,
    /// `orig_yaw_*` after the resave, when yaw saved.
    pub orig_yaw: Option<OrigAxis>,
    /// `update_gcs` id. `None` on skip.
    pub gcs_message: Option<u8>,
    /// `reset()` after a successful save.
    pub reset: bool,
}

/// `AC_AutoTune::disarmed` leftover — save vs `reset()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisarmAction {
    /// `save_tuning_gains()`.
    Save,
    /// `reset()`.
    Reset,
}

/// `AC_AutoTune::stop` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTuneStop {
    /// `load_gains(ORIGINAL)`.
    pub load: LoadGains,
    /// `AUTOTUNE_MESSAGE_STOPPED`.
    pub gcs_message: u8,
}

/// `aggressiveness.set(constrain_float(aggressiveness, 0.05, 0.2))`.
#[must_use]
pub fn constrain_aggressiveness(aggressiveness: f32) -> f32 {
    constrain_value(aggressiveness, AUTOTUNE_AGGR_MIN, AUTOTUNE_AGGR_MAX)
}

/// `load_gains` no-op when `loaded_gains` already matches.
#[must_use]
pub fn load_gains_already(loaded_gains: GainType, gain_type: GainType) -> bool {
    loaded_gains == gain_type
}

/// Multi `backup_gains_and_initialise` after the base leftover.
///
/// The base sequencer (first axis, `axes_completed = 0`,
/// `next_tune_type`, WAITING_FOR_LEVEL) already lives in
/// [`crate::mode_autotune::mode_autotune_init`] /
/// [`crate::autotune_next`]. This is the orig/tune PID snapshot
/// and the aggressiveness constrain.
#[must_use]
pub fn backup_multi_gains(view: &BackupView) -> BackupGains {
    let mut tune_yaw = TuneAxis::from_live(view.yaw);
    let mut yaw_rd_seeded = false;
    let mut yaw_rlpf_seeded = false;
    if yaw_d_enabled(view.axis_bitmask) && is_zero(tune_yaw.rd) {
        tune_yaw.rd = view.min_d;
        yaw_rd_seeded = true;
    }
    if yaw_enabled(view.axis_bitmask) && is_zero(tune_yaw.r_lpf) {
        tune_yaw.r_lpf = AUTOTUNE_FLTE_MIN;
        yaw_rlpf_seeded = true;
    }
    BackupGains {
        aggressiveness: constrain_aggressiveness(view.aggressiveness),
        orig_bf_feedforward: view.bf_feedforward,
        orig_roll: OrigAxis::from_live(view.roll),
        orig_pitch: OrigAxis::from_live(view.pitch),
        orig_yaw: OrigAxis::from_live(view.yaw),
        tune_roll: TuneAxis::from_live(view.roll),
        tune_pitch: TuneAxis::from_live(view.pitch),
        tune_yaw,
        yaw_rd_seeded,
        yaw_rlpf_seeded,
    }
}

/// Multi `load_orig_gains`.
#[must_use]
pub fn load_orig_gains(view: &LoadView) -> LoadedGains {
    let mut out = LoadedGains {
        use_sqrt_controller: true,
        bf_feedforward: Some(view.orig_bf_feedforward),
        accel_roll_forced_zero: false,
        accel_pitch_forced_zero: false,
        roll: AxisWrite::skipped(),
        pitch: AxisWrite::skipped(),
        yaw: AxisWrite::skipped(),
    };
    if roll_enabled(view.axis_bitmask) && !is_zero(view.orig_roll.rp) {
        out.roll = orig_full(&view.orig_roll, false);
    }
    if pitch_enabled(view.axis_bitmask) && !is_zero(view.orig_pitch.rp) {
        out.pitch = orig_full(&view.orig_pitch, false);
    }
    if (yaw_enabled(view.axis_bitmask) || yaw_d_enabled(view.axis_bitmask))
        && !is_zero(view.orig_yaw.rp)
    {
        out.yaw = orig_full(&view.orig_yaw, true);
    }
    out
}

/// Multi `load_tuned_gains`.
#[must_use]
pub fn load_tuned_gains(view: &LoadView) -> LoadedGains {
    let mut out = LoadedGains {
        use_sqrt_controller: true,
        bf_feedforward: None,
        accel_roll_forced_zero: false,
        accel_pitch_forced_zero: false,
        roll: AxisWrite::skipped(),
        pitch: AxisWrite::skipped(),
        yaw: AxisWrite::skipped(),
    };
    if !view.live_bf_feedforward {
        out.bf_feedforward = Some(true);
        out.accel_roll_forced_zero = true;
        out.accel_pitch_forced_zero = true;
    }
    if (view.axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) != 0
        && roll_enabled(view.axis_bitmask)
        && !is_zero(view.tune_roll.rp)
    {
        out.roll = tuned_rp(&view.tune_roll, &view.orig_roll, AUTOTUNE_PI_RATIO_FINAL);
    }
    if (view.axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) != 0
        && pitch_enabled(view.axis_bitmask)
        && !is_zero(view.tune_pitch.rp)
    {
        out.pitch = tuned_rp(&view.tune_pitch, &view.orig_pitch, AUTOTUNE_PI_RATIO_FINAL);
    }
    let yaw_done = ((view.axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) != 0
        && yaw_enabled(view.axis_bitmask))
        || ((view.axes_completed & AUTOTUNE_AXIS_BITMASK_YAW_D) != 0
            && yaw_d_enabled(view.axis_bitmask));
    if yaw_done && !is_zero(view.tune_yaw.rp) {
        out.yaw = tuned_yaw(view);
    }
    out
}

/// Multi `load_intra_test_gains`.
#[must_use]
pub fn load_intra_test_gains(view: &LoadView) -> LoadedGains {
    let mut out = LoadedGains {
        use_sqrt_controller: true,
        bf_feedforward: Some(true),
        accel_roll_forced_zero: false,
        accel_pitch_forced_zero: false,
        roll: AxisWrite::skipped(),
        pitch: AxisWrite::skipped(),
        yaw: AxisWrite::skipped(),
    };
    if roll_enabled(view.axis_bitmask) {
        out.roll = intra_rp(&view.orig_roll, false);
    }
    if pitch_enabled(view.axis_bitmask) {
        out.pitch = intra_rp(&view.orig_pitch, false);
    }
    if yaw_enabled(view.axis_bitmask) || yaw_d_enabled(view.axis_bitmask) {
        out.yaw = intra_rp(&view.orig_yaw, true);
    }
    out
}

/// Multi `load_test_gains`. Relies on `view.axis`.
#[must_use]
pub fn load_test_gains(view: &LoadView) -> LoadedGains {
    let mut out = LoadedGains {
        use_sqrt_controller: false,
        bf_feedforward: None,
        accel_roll_forced_zero: false,
        accel_pitch_forced_zero: false,
        roll: AxisWrite::skipped(),
        pitch: AxisWrite::skipped(),
        yaw: AxisWrite::skipped(),
    };
    match view.axis {
        AxisType::Roll => out.roll = test_rp(&view.tune_roll),
        AxisType::Pitch => out.pitch = test_rp(&view.tune_pitch),
        AxisType::Yaw | AxisType::YawD => out.yaw = test_yaw(view),
    }
    out
}

/// Base `AC_AutoTune::load_gains` leftover.
#[must_use]
pub fn load_gains(loaded_gains: GainType, gain_type: GainType, view: &LoadView) -> LoadGains {
    if load_gains_already(loaded_gains, gain_type) {
        return LoadGains {
            skipped: true,
            gain_type,
            loaded: None,
        };
    }
    let loaded = match gain_type {
        GainType::Original => load_orig_gains(view),
        GainType::IntraTest => load_intra_test_gains(view),
        GainType::Test => load_test_gains(view),
        GainType::Tuned => load_tuned_gains(view),
    };
    LoadGains {
        skipped: false,
        gain_type,
        loaded: Some(loaded),
    }
}

/// Multi `save_tuning_gains`.
#[must_use]
pub fn save_tuning_gains(view: &LoadView) -> SaveTuning {
    if view.axes_completed == 0 {
        return SaveTuning {
            skipped: true,
            bf_feedforward_saved: false,
            accel_rp_saved_zero: false,
            roll: AxisWrite::skipped(),
            pitch: AxisWrite::skipped(),
            yaw: AxisWrite::skipped(),
            orig_roll: None,
            orig_pitch: None,
            orig_yaw: None,
            gcs_message: None,
            reset: false,
        };
    }

    let mut out = SaveTuning {
        skipped: false,
        bf_feedforward_saved: false,
        accel_rp_saved_zero: false,
        roll: AxisWrite::skipped(),
        pitch: AxisWrite::skipped(),
        yaw: AxisWrite::skipped(),
        orig_roll: None,
        orig_pitch: None,
        orig_yaw: None,
        gcs_message: Some(AUTOTUNE_MESSAGE_SAVED_GAINS),
        reset: true,
    };
    if !view.live_bf_feedforward {
        out.bf_feedforward_saved = true;
        out.accel_rp_saved_zero = true;
    }
    if (view.axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) != 0
        && roll_enabled(view.axis_bitmask)
        && !is_zero(view.tune_roll.rp)
    {
        out.roll = save_rp(&view.tune_roll, &view.orig_roll, AUTOTUNE_PI_RATIO_FINAL);
        out.orig_roll = Some(orig_after_save_rp(&out.roll, &view.orig_roll));
    }
    if (view.axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) != 0
        && pitch_enabled(view.axis_bitmask)
        && !is_zero(view.tune_pitch.rp)
    {
        out.pitch = save_rp(&view.tune_pitch, &view.orig_pitch, AUTOTUNE_PI_RATIO_FINAL);
        out.orig_pitch = Some(orig_after_save_rp(&out.pitch, &view.orig_pitch));
    }
    let yaw_done = ((view.axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) != 0
        && yaw_enabled(view.axis_bitmask))
        || ((view.axes_completed & AUTOTUNE_AXIS_BITMASK_YAW_D) != 0
            && yaw_d_enabled(view.axis_bitmask));
    if yaw_done && !is_zero(view.tune_yaw.rp) {
        out.yaw = save_yaw(view);
        out.orig_yaw = Some(orig_after_save_yaw(&out.yaw, &view.orig_yaw));
    }
    out
}

/// `AC_AutoTune::disarmed` leftover.
#[must_use]
pub fn autotune_disarmed(
    in_autotune_mode: bool,
    testing_switch_used: bool,
    loaded_gains: GainType,
) -> DisarmAction {
    let testing_tuned = testing_switch_used && loaded_gains == GainType::Tuned;
    let tune_complete_no_testing = !testing_switch_used && in_autotune_mode;
    if tune_complete_no_testing || testing_tuned {
        DisarmAction::Save
    } else {
        DisarmAction::Reset
    }
}

/// `AC_AutoTune::stop` leftover — original gains + GCS STOPPED.
#[must_use]
pub fn autotune_stop(loaded_gains: GainType, view: &LoadView) -> AutoTuneStop {
    AutoTuneStop {
        load: load_gains(loaded_gains, GainType::Original, view),
        gcs_message: AUTOTUNE_MESSAGE_STOPPED,
    }
}

fn orig_full(orig: &OrigAxis, yaw: bool) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: orig.rp,
        ri: orig.ri,
        rd: orig.rd,
        rd_written: true,
        rff: orig.rff,
        dff: orig.dff,
        fltt: Some(orig.fltt),
        smax: Some(orig.smax),
        flte: if yaw { Some(orig.r_lpf) } else { None },
        sp: orig.sp,
        accel_radss: Some(orig.accel_radss),
        saved: false,
    }
}

fn tuned_rp(tune: &TuneAxis, orig: &OrigAxis, i_ratio: f32) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: tune.rp,
        ri: tune.rp * i_ratio,
        rd: tune.rd,
        rd_written: true,
        rff: orig.rff,
        dff: orig.dff,
        fltt: None,
        smax: None,
        flte: None,
        sp: tune.sp,
        accel_radss: Some(tune.accel_radss),
        saved: false,
    }
}

fn tuned_yaw(view: &LoadView) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: view.tune_yaw.rp,
        ri: view.tune_yaw.rp * AUTOTUNE_YAW_PI_RATIO_FINAL,
        rd: view.tune_yaw.rd,
        rd_written: yaw_d_enabled(view.axis_bitmask),
        rff: view.orig_yaw.rff,
        dff: view.orig_yaw.dff,
        fltt: None,
        smax: None,
        flte: if yaw_enabled(view.axis_bitmask) {
            Some(view.tune_yaw.r_lpf)
        } else {
            None
        },
        sp: view.tune_yaw.sp,
        accel_radss: Some(view.tune_yaw.accel_radss),
        saved: false,
    }
}

fn intra_rp(orig: &OrigAxis, yaw: bool) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: orig.rp,
        ri: orig.rp * AUTOTUNE_PI_RATIO_FOR_TESTING,
        rd: orig.rd,
        rd_written: true,
        rff: orig.rff,
        dff: orig.dff,
        fltt: Some(orig.fltt),
        smax: Some(orig.smax),
        flte: if yaw { Some(orig.r_lpf) } else { None },
        sp: orig.sp,
        accel_radss: None,
        saved: false,
    }
}

fn test_rp(tune: &TuneAxis) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: tune.rp,
        ri: tune.rp * AUTOTUNE_TEST_I_RATIO,
        rd: tune.rd,
        rd_written: true,
        rff: 0.0,
        dff: 0.0,
        fltt: Some(0.0),
        smax: Some(0.0),
        flte: None,
        sp: tune.sp,
        accel_radss: None,
        saved: false,
    }
}

fn test_yaw(view: &LoadView) -> AxisWrite {
    let yaw_d = view.axis == AxisType::YawD;
    AxisWrite {
        written: true,
        rp: view.tune_yaw.rp,
        ri: view.tune_yaw.rp * AUTOTUNE_TEST_I_RATIO,
        rd: if yaw_d { view.tune_yaw.rd } else { 0.0 },
        rd_written: true,
        rff: 0.0,
        dff: 0.0,
        fltt: Some(0.0),
        smax: Some(0.0),
        flte: if yaw_d {
            None
        } else {
            Some(view.tune_yaw.r_lpf)
        },
        sp: view.tune_yaw.sp,
        accel_radss: None,
        saved: false,
    }
}

fn save_rp(tune: &TuneAxis, orig: &OrigAxis, i_ratio: f32) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: tune.rp,
        ri: tune.rp * i_ratio,
        rd: tune.rd,
        rd_written: true,
        rff: orig.rff,
        dff: orig.dff,
        fltt: Some(orig.fltt),
        smax: Some(orig.smax),
        flte: None,
        sp: tune.sp,
        accel_radss: Some(tune.accel_radss),
        saved: true,
    }
}

fn save_yaw(view: &LoadView) -> AxisWrite {
    AxisWrite {
        written: true,
        rp: view.tune_yaw.rp,
        ri: view.tune_yaw.rp * AUTOTUNE_YAW_PI_RATIO_FINAL,
        rd: view.tune_yaw.rd,
        rd_written: yaw_d_enabled(view.axis_bitmask),
        rff: view.orig_yaw.rff,
        dff: view.orig_yaw.dff,
        fltt: Some(view.orig_yaw.fltt),
        smax: Some(view.orig_yaw.smax),
        flte: if yaw_enabled(view.axis_bitmask) {
            Some(view.tune_yaw.r_lpf)
        } else {
            None
        },
        sp: view.tune_yaw.sp,
        accel_radss: Some(view.tune_yaw.accel_radss),
        saved: true,
    }
}

fn orig_after_save_rp(written: &AxisWrite, prev: &OrigAxis) -> OrigAxis {
    OrigAxis {
        rp: written.rp,
        ri: written.ri,
        rd: written.rd,
        rff: written.rff,
        dff: written.dff,
        fltt: prev.fltt,
        smax: prev.smax,
        r_lpf: prev.r_lpf,
        sp: written.sp,
        accel_radss: written.accel_radss.unwrap_or(prev.accel_radss),
    }
}

fn orig_after_save_yaw(written: &AxisWrite, prev: &OrigAxis) -> OrigAxis {
    OrigAxis {
        rp: written.rp,
        ri: written.ri,
        rd: if written.rd_written {
            written.rd
        } else {
            prev.rd
        },
        rff: written.rff,
        dff: written.dff,
        fltt: prev.fltt,
        smax: prev.smax,
        r_lpf: written.flte.unwrap_or(prev.r_lpf),
        sp: written.sp,
        accel_radss: written.accel_radss.unwrap_or(prev.accel_radss),
    }
}
