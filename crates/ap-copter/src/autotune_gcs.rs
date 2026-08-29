//! AutoTune GCS leftover, upstream `AC_AutoTune::update_gcs` /
//! `send_step_string` / `get_axis_name` / `get_tune_type_name` and
//! Multi `do_gcs_announcements` / `report_final_gains`.
//!
//! Tracked as **COP-027**. Earlier leftovers record message ids and a
//! `do_gcs_announcements` flag. This leftover is the text: severity,
//! the 2 s announce interval, axis / tune-type names, the
//! `success_counter * 25` percent, the Saved / Pilot Testing axis
//! suffix, and the three-line Multi final-gain report.
//!
//! Multi `do_post_test_gcs_announcements` is a no-op. Heli GCS is out
//! of scope. Logging leftover lives in [`crate::autotune_log`].
//!
//! This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::autotune_load_save::{AUTOTUNE_PI_RATIO_FINAL, AUTOTUNE_YAW_PI_RATIO_FINAL};
use crate::mode_autotune::{
    AxisType, Step, TuneType, AUTOTUNE_AXIS_BITMASK_PITCH, AUTOTUNE_AXIS_BITMASK_ROLL,
    AUTOTUNE_AXIS_BITMASK_YAW, AUTOTUNE_AXIS_BITMASK_YAW_D, AUTOTUNE_MESSAGE_FAILED,
    AUTOTUNE_MESSAGE_SAVED_GAINS, AUTOTUNE_MESSAGE_STARTED, AUTOTUNE_MESSAGE_STOPPED,
    AUTOTUNE_MESSAGE_SUCCESS, AUTOTUNE_MESSAGE_TESTING, AUTOTUNE_MESSAGE_TESTING_END,
    AUTOTUNE_SUCCESS_COUNT,
};
use ap_math::scalar::rad_to_cd;

/// `AUTOTUNE_ANNOUNCE_INTERVAL_MS`.
pub const AUTOTUNE_ANNOUNCE_INTERVAL_MS: u32 = 2000;

/// `MAV_SEVERITY_CRITICAL`.
pub const MAV_SEVERITY_CRITICAL: u8 = 2;
/// `MAV_SEVERITY_NOTICE`.
pub const MAV_SEVERITY_NOTICE: u8 = 5;
/// `MAV_SEVERITY_INFO`.
pub const MAV_SEVERITY_INFO: u8 = 6;

/// Integer percent step: `100 / AUTOTUNE_SUCCESS_COUNT`.
pub const AUTOTUNE_ANNOUNCE_PERCENT_STEP: u8 = 100 / AUTOTUNE_SUCCESS_COUNT;

/// `update_gcs(AUTOTUNE_MESSAGE_STARTED)`.
pub const TEXT_STARTED: &str = "AutoTune: Started";
/// `update_gcs(AUTOTUNE_MESSAGE_STOPPED)`.
pub const TEXT_STOPPED: &str = "AutoTune: Stopped";
/// `update_gcs(AUTOTUNE_MESSAGE_SUCCESS)`.
pub const TEXT_SUCCESS: &str = "AutoTune: Success";
/// `update_gcs(AUTOTUNE_MESSAGE_FAILED)`.
pub const TEXT_FAILED: &str = "AutoTune: Failed";
/// `update_gcs(AUTOTUNE_MESSAGE_TESTING_END)`.
pub const TEXT_TESTING_END: &str = "AutoTune: original gains restored";
/// Verb used when `message_id == AUTOTUNE_MESSAGE_SAVED_GAINS`.
pub const TEXT_SAVED_VERB: &str = "Saved";
/// Verb used when `message_id == AUTOTUNE_MESSAGE_TESTING`.
pub const TEXT_TESTING_VERB: &str = "Pilot Testing";

/// `send_step_string` while the pilot is flying the aircraft.
pub const TEXT_STEP_PILOT_OVERRIDE: &str = "AutoTune: Paused: Pilot Override Active";
/// `Step::WAITING_FOR_LEVEL`.
pub const TEXT_STEP_LEVELING: &str = "AutoTune: Leveling";
/// `Step::UPDATE_GAINS`.
pub const TEXT_STEP_UPDATING: &str = "AutoTune: Updating Gains";
/// `Step::ABORT`.
pub const TEXT_STEP_ABORTING: &str = "AutoTune: Aborting Test";
/// `Step::EXECUTING_TEST`.
pub const TEXT_STEP_TESTING: &str = "AutoTune: Testing";
/// Fallback after the C++ switch. Exhaustive Rust cannot hit it.
pub const TEXT_STEP_UNKNOWN: &str = "AutoTune: unknown step";

/// `GCS_SEND_TEXT` while override stays active past the warn interval.
pub const TEXT_PILOT_OVERRIDES_ACTIVE: &str = "AutoTune: pilot overrides active";
/// Level-wait timed out.
pub const TEXT_FAILED_TO_LEVEL: &str = "AutoTune: Failed to level, please tune manually";
/// Aux-switch test while the tune is not finished.
pub const TEXT_MUST_BE_COMPLETE: &str = "AutoTune: must be complete to test gains";
/// Twitch scaler hit the floor.
pub const TEXT_TWITCH_SIZE_FAILED: &str = "AutoTune: Twitch Size Determination Failed";
/// Rate-D walk hit `min_d`.
pub const TEXT_MIN_RATE_D: &str = "AutoTune: Min Rate D limit reached";
/// Rate-D determination failed.
pub const TEXT_RATE_D_FAILED: &str = "AutoTune: Rate D Gain Determination Failed";
/// Rate-P determination failed.
pub const TEXT_RATE_P_FAILED: &str = "AutoTune: Rate P Gain Determination Failed";
/// Angle-P determination failed.
pub const TEXT_ANGLE_P_FAILED: &str = "AutoTune: Angle P Gain Determination Failed";

/// Upstream `AC_AutoTune::get_axis_name`.
#[must_use]
pub const fn get_axis_name(axis: AxisType) -> &'static str {
    match axis {
        AxisType::Roll => "Roll",
        AxisType::Pitch => "Pitch",
        AxisType::Yaw => "Yaw(E)",
        AxisType::YawD => "Yaw(D)",
    }
}

/// Upstream `AC_AutoTune::get_tune_type_name`.
#[must_use]
pub const fn get_tune_type_name(tune_type: TuneType) -> &'static str {
    match tune_type {
        TuneType::RateDUp => "Rate D Up",
        TuneType::RateDDown => "Rate D Down",
        TuneType::RatePUp => "Rate P Up",
        TuneType::RateFfUp => "Rate FF Up",
        TuneType::AnglePUp => "Angle P Up",
        TuneType::AnglePDown => "Angle P Down",
        TuneType::MaxGains => "Find Max Gains",
        TuneType::TuneCheck => "Check Tune Frequency Response",
        TuneType::TuneComplete => "Tune Complete",
    }
}

/// `success_counter * (100 / AUTOTUNE_SUCCESS_COUNT)` — integer math.
#[must_use]
pub const fn announce_percent(success_counter: i8) -> u8 {
    if success_counter <= 0 {
        0
    } else {
        (success_counter as u8).saturating_mul(AUTOTUNE_ANNOUNCE_PERCENT_STEP)
    }
}

/// What `update_gcs` would send. `None` on an unknown id.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateGcs {
    /// `MAV_SEVERITY`.
    pub severity: u8,
    /// Which `AUTOTUNE_MESSAGE_*` body.
    pub kind: UpdateGcsKind,
    /// `axes_completed & ROLL` on the Testing / Saved line.
    pub roll: bool,
    /// `axes_completed & PITCH`.
    pub pitch: bool,
    /// `axes_completed & YAW`.
    pub yaw: bool,
    /// `axes_completed & YAW_D`.
    pub yaw_d: bool,
}

/// `update_gcs` body. Testing / Saved share the axis-suffix line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateGcsKind {
    /// `AUTOTUNE_MESSAGE_STARTED` — INFO.
    Started,
    /// `AUTOTUNE_MESSAGE_STOPPED` — INFO.
    Stopped,
    /// `AUTOTUNE_MESSAGE_SUCCESS` — NOTICE.
    Success,
    /// `AUTOTUNE_MESSAGE_FAILED` — NOTICE.
    Failed,
    /// `AUTOTUNE_MESSAGE_TESTING` — NOTICE, "Pilot Testing gains for …".
    Testing,
    /// `AUTOTUNE_MESSAGE_SAVED_GAINS` — NOTICE, "Saved gains for …".
    SavedGains,
    /// `AUTOTUNE_MESSAGE_TESTING_END` — NOTICE.
    TestingEnd,
}

impl UpdateGcs {
    /// Static body for the non-axis messages. `None` on Testing / Saved.
    #[must_use]
    pub const fn text(self) -> Option<&'static str> {
        match self.kind {
            UpdateGcsKind::Started => Some(TEXT_STARTED),
            UpdateGcsKind::Stopped => Some(TEXT_STOPPED),
            UpdateGcsKind::Success => Some(TEXT_SUCCESS),
            UpdateGcsKind::Failed => Some(TEXT_FAILED),
            UpdateGcsKind::TestingEnd => Some(TEXT_TESTING_END),
            UpdateGcsKind::Testing | UpdateGcsKind::SavedGains => None,
        }
    }

    /// `"Saved"` or `"Pilot Testing"` on the axis-suffix line.
    #[must_use]
    pub const fn gains_verb(self) -> Option<&'static str> {
        match self.kind {
            UpdateGcsKind::SavedGains => Some(TEXT_SAVED_VERB),
            UpdateGcsKind::Testing => Some(TEXT_TESTING_VERB),
            _ => None,
        }
    }
}

/// Upstream `AC_AutoTune::update_gcs`.
///
/// Unknown ids fall off the C++ switch and send nothing.
#[must_use]
pub const fn update_gcs(message_id: u8, axes_completed: u8) -> Option<UpdateGcs> {
    let roll = axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL != 0;
    let pitch = axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH != 0;
    let yaw = axes_completed & AUTOTUNE_AXIS_BITMASK_YAW != 0;
    let yaw_d = axes_completed & AUTOTUNE_AXIS_BITMASK_YAW_D != 0;
    let (severity, kind) = match message_id {
        AUTOTUNE_MESSAGE_STARTED => (MAV_SEVERITY_INFO, UpdateGcsKind::Started),
        AUTOTUNE_MESSAGE_STOPPED => (MAV_SEVERITY_INFO, UpdateGcsKind::Stopped),
        AUTOTUNE_MESSAGE_SUCCESS => (MAV_SEVERITY_NOTICE, UpdateGcsKind::Success),
        AUTOTUNE_MESSAGE_FAILED => (MAV_SEVERITY_NOTICE, UpdateGcsKind::Failed),
        AUTOTUNE_MESSAGE_TESTING => (MAV_SEVERITY_NOTICE, UpdateGcsKind::Testing),
        AUTOTUNE_MESSAGE_SAVED_GAINS => (MAV_SEVERITY_NOTICE, UpdateGcsKind::SavedGains),
        AUTOTUNE_MESSAGE_TESTING_END => (MAV_SEVERITY_NOTICE, UpdateGcsKind::TestingEnd),
        _ => return None,
    };
    Some(UpdateGcs {
        severity,
        kind,
        roll,
        pitch,
        yaw,
        yaw_d,
    })
}

/// Axis suffix pieces for the Testing / Saved line, including the
/// trailing spaces on Roll / Pitch that upstream prints.
#[must_use]
pub const fn testing_axis_piece(bit: u8, axes_completed: u8) -> &'static str {
    match bit {
        AUTOTUNE_AXIS_BITMASK_ROLL if axes_completed & bit != 0 => "Roll ",
        AUTOTUNE_AXIS_BITMASK_PITCH if axes_completed & bit != 0 => "Pitch ",
        AUTOTUNE_AXIS_BITMASK_YAW if axes_completed & bit != 0 => "Yaw(E)",
        AUTOTUNE_AXIS_BITMASK_YAW_D if axes_completed & bit != 0 => "Yaw(D)",
        _ => "",
    }
}

/// Upstream `AC_AutoTune::send_step_string`.
#[must_use]
pub const fn send_step_string(pilot_override: bool, step: Step) -> &'static str {
    if pilot_override {
        return TEXT_STEP_PILOT_OVERRIDE;
    }
    match step {
        Step::WaitingForLevel => TEXT_STEP_LEVELING,
        Step::UpdateGains => TEXT_STEP_UPDATING,
        Step::Abort => TEXT_STEP_ABORTING,
        Step::ExecutingTest => TEXT_STEP_TESTING,
    }
}

/// What Multi `do_gcs_announcements` reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GcsAnnounceView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `last_announce_ms` before this call.
    pub last_announce_ms: u32,
    /// Current Multi axis.
    pub axis: AxisType,
    /// Current Multi tune type.
    pub tune_type: TuneType,
    /// `success_counter` — upstream `int8_t`.
    pub success_counter: i8,
}

impl GcsAnnounceView {
    /// Mid-tune leftover view: roll, RATE_D_UP, counter 0, interval elapsed.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            now_ms: 10_000,
            last_announce_ms: 0,
            axis: AxisType::Roll,
            tune_type: TuneType::RateDUp,
            success_counter: 0,
        }
    }
}

/// Leftover of one Multi `do_gcs_announcements`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GcsAnnounce {
    /// Interval elapsed — STATUSTEXT would go out.
    pub sent: bool,
    /// `last_announce_ms` after this call.
    pub last_announce_ms: u32,
    /// `MAV_SEVERITY_INFO` when sent.
    pub severity: u8,
    /// `get_axis_name()`.
    pub axis_name: &'static str,
    /// `get_tune_type_name()`.
    pub tune_type_name: &'static str,
    /// `success_counter * 25`.
    pub percent: u8,
}

/// Upstream `AC_AutoTune_Multi::do_gcs_announcements`.
///
/// Quiet until `AUTOTUNE_ANNOUNCE_INTERVAL_MS` has elapsed. Multi
/// `do_post_test_gcs_announcements` stays a no-op.
#[must_use]
pub const fn do_gcs_announcements(view: &GcsAnnounceView) -> GcsAnnounce {
    let elapsed = view.now_ms.wrapping_sub(view.last_announce_ms);
    if elapsed < AUTOTUNE_ANNOUNCE_INTERVAL_MS {
        return GcsAnnounce {
            sent: false,
            last_announce_ms: view.last_announce_ms,
            severity: MAV_SEVERITY_INFO,
            axis_name: get_axis_name(view.axis),
            tune_type_name: get_tune_type_name(view.tune_type),
            percent: announce_percent(view.success_counter),
        };
    }
    GcsAnnounce {
        sent: true,
        last_announce_ms: view.now_ms,
        severity: MAV_SEVERITY_INFO,
        axis_name: get_axis_name(view.axis),
        tune_type_name: get_tune_type_name(view.tune_type),
        percent: announce_percent(view.success_counter),
    }
}

/// Multi `do_post_test_gcs_announcements` — empty override.
#[must_use]
pub const fn do_post_test_gcs_announcements() -> bool {
    false
}

/// What Multi `report_final_gains` reads for one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReportGainsView {
    /// Axis that just finished.
    pub axis: AxisType,
    /// `tune_*_rp`.
    pub tune_rp: f32,
    /// `tune_*_rd`. Yaw(E) report forces D to 0.
    pub tune_rd: f32,
    /// `tune_*_sp`.
    pub tune_sp: f32,
    /// `tune_*_accel_radss`.
    pub tune_accel_radss: f32,
}

impl ReportGainsView {
    /// Mid-range leftover view: roll, typical tuned gains.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: AxisType::Roll,
            tune_rp: 0.15,
            tune_rd: 0.004,
            tune_sp: 4.5,
            tune_accel_radss: 0.0,
        }
    }
}

/// Three NOTICE lines from Multi `report_axis_gains`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReportAxisGains {
    /// `"Roll"` / `"Pitch"` / `"Yaw(E)"` / `"Yaw(D)"`.
    pub axis_string: &'static str,
    /// Rate P.
    pub rate_p: f32,
    /// Rate I — `rp * PI_RATIO_FINAL` or yaw `* 0.1`.
    pub rate_i: f32,
    /// Rate D. Forced to 0 on Yaw(E).
    pub rate_d: f32,
    /// Angle P.
    pub angle_p: f32,
    /// `rad_to_cd(max_accel_radss)` — `%0.0f` on the Angle line.
    pub max_accel_cd: f32,
    /// Every report line is NOTICE.
    pub severity: u8,
}

/// Upstream `AC_AutoTune_Multi::report_final_gains`.
#[must_use]
pub fn report_final_gains(view: &ReportGainsView) -> ReportAxisGains {
    match view.axis {
        AxisType::Roll => report_axis_gains(
            "Roll",
            view.tune_rp,
            view.tune_rp * AUTOTUNE_PI_RATIO_FINAL,
            view.tune_rd,
            view.tune_sp,
            view.tune_accel_radss,
        ),
        AxisType::Pitch => report_axis_gains(
            "Pitch",
            view.tune_rp,
            view.tune_rp * AUTOTUNE_PI_RATIO_FINAL,
            view.tune_rd,
            view.tune_sp,
            view.tune_accel_radss,
        ),
        AxisType::Yaw => report_axis_gains(
            "Yaw(E)",
            view.tune_rp,
            view.tune_rp * AUTOTUNE_YAW_PI_RATIO_FINAL,
            0.0,
            view.tune_sp,
            view.tune_accel_radss,
        ),
        AxisType::YawD => report_axis_gains(
            "Yaw(D)",
            view.tune_rp,
            view.tune_rp * AUTOTUNE_YAW_PI_RATIO_FINAL,
            view.tune_rd,
            view.tune_sp,
            view.tune_accel_radss,
        ),
    }
}

/// Upstream `AC_AutoTune_Multi::report_axis_gains`.
#[must_use]
pub fn report_axis_gains(
    axis_string: &'static str,
    rate_p: f32,
    rate_i: f32,
    rate_d: f32,
    angle_p: f32,
    max_accel_radss: f32,
) -> ReportAxisGains {
    ReportAxisGains {
        axis_string,
        rate_p,
        rate_i,
        rate_d,
        angle_p,
        max_accel_cd: rad_to_cd(max_accel_radss),
        severity: MAV_SEVERITY_NOTICE,
    }
}
