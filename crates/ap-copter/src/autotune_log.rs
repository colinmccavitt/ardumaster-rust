//! Multi AutoTune logging leftover, upstream `AC_AutoTune_Multi::Log_AutoTune`
//! / `Log_AutoTuneDetails` / `Log_Write_AutoTune` / `Log_Write_AutoTuneDetails`
//! and Copter `AutoTune::log_pids`.
//!
//! Tracked as **COP-027**. `AP::logger().Write` / `WriteStreaming` /
//! `Write_PID` / `Write_Rate` stay logger leftovers — this leftover
//! packs the ATUN / ATDE fields and catalogues the three rate-PID
//! writes plus the attitude-control rate packet.
//!
//! Multi `Log_AutoTuneSweep` is a no-op. Heli sweep logging is out of
//! scope. This is not Plane `AP_AutoTune` (the `ap-autotune` crate).

use crate::mode_autotune::{twitch_is_angle_p, AxisType, TuneType};

/// `ATUN` message name.
pub const ATUN_MSG: &str = "ATUN";
/// `ATUN` field labels.
pub const ATUN_LABELS: &str = "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt";
/// `ATUN` units string.
pub const ATUN_UNITS: &str = "s--ddd---o";
/// `ATUN` multipliers string.
pub const ATUN_MULTS: &str = "F--000---0";
/// `ATUN` format string (`QBBfffffff`).
pub const ATUN_FMT: &str = "QBBfffffff";

/// `ATDE` message name.
pub const ATDE_MSG: &str = "ATDE";
/// `ATDE` field labels.
pub const ATDE_LABELS: &str = "TimeUS,Angle,Rate";
/// `ATDE` units string.
pub const ATDE_UNITS: &str = "sdk";
/// `ATDE` multipliers string.
pub const ATDE_MULTS: &str = "F00";
/// `ATDE` format string (`Qff`).
pub const ATDE_FMT: &str = "Qff";

/// cd / cd/s values are written as deg / deg/s (`* 0.01`).
pub const AUTOTUNE_LOG_CD_TO_DEG: f32 = 0.01;

/// Copter `Write_PID(LOG_PIDR_MSG, ...)` name.
pub const LOG_PIDR_MSG_NAME: &str = "PIDR";
/// Copter `Write_PID(LOG_PIDP_MSG, ...)` name.
pub const LOG_PIDP_MSG_NAME: &str = "PIDP";
/// Copter `Write_PID(LOG_PIDY_MSG, ...)` name.
pub const LOG_PIDY_MSG_NAME: &str = "PIDY";

/// Packed `ATUN` payload. `AP::logger().Write` stays a leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogAtun {
    /// `AP_HAL::micros64()`.
    pub time_us: u64,
    /// Member `axis` — C++ takes `_axis` but writes `axis`.
    pub axis: u8,
    /// `tune_step` / `tune_type`.
    pub tune_step: u8,
    /// `meas_target * 0.01`.
    pub targ: f32,
    /// `meas_min * 0.01`.
    pub min: f32,
    /// `meas_max * 0.01`.
    pub max: f32,
    /// Rate-P (or yaw P) being logged.
    pub rp: f32,
    /// Rate-D, or yaw `rLPF` on [`AxisType::Yaw`].
    pub rd: f32,
    /// Angle-P being logged.
    pub sp: f32,
    /// `test_accel_max_cdss` (`ddt`).
    pub ddt: f32,
}

/// Packed `ATDE` payload. `AP::logger().WriteStreaming` stays a leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogAtde {
    /// `AP_HAL::micros64()`.
    pub time_us: u64,
    /// `angle_cd * 0.01`.
    pub angle: f32,
    /// `rate_cds * 0.01`.
    pub rate: f32,
}

/// Inputs `Log_AutoTune` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogAutoTuneView {
    /// Member `axis`.
    pub axis: AxisType,
    /// Member `tune_type`.
    pub tune_type: TuneType,
    /// Angle-P twitch target, centidegrees.
    pub target_angle: f32,
    /// Rate twitch target, centidegrees/s.
    pub target_rate: f32,
    /// Measured min angle, centidegrees.
    pub test_angle_min: f32,
    /// Measured max angle, centidegrees.
    pub test_angle_max: f32,
    /// Measured min rate, centidegrees/s.
    pub test_rate_min: f32,
    /// Measured max rate, centidegrees/s.
    pub test_rate_max: f32,
    /// Roll rate-P.
    pub tune_roll_rp: f32,
    /// Roll rate-D.
    pub tune_roll_rd: f32,
    /// Roll angle-P.
    pub tune_roll_sp: f32,
    /// Pitch rate-P.
    pub tune_pitch_rp: f32,
    /// Pitch rate-D.
    pub tune_pitch_rd: f32,
    /// Pitch angle-P.
    pub tune_pitch_sp: f32,
    /// Yaw rate-P (shared by `YAW` and `YAW_D`).
    pub tune_yaw_rp: f32,
    /// Yaw rate-D (`YAW_D` RD column).
    pub tune_yaw_rd: f32,
    /// Yaw rate LPF (`YAW` RD column).
    pub tune_yaw_r_lpf: f32,
    /// Yaw angle-P.
    pub tune_yaw_sp: f32,
    /// `test_accel_max_cdss`.
    pub test_accel_max_cdss: f32,
}

impl LogAutoTuneView {
    /// Distinct per-axis gains so the ATUN switch is testable.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            axis: AxisType::Roll,
            tune_type: TuneType::RatePUp,
            target_angle: 1_500.0,
            target_rate: 18_000.0,
            test_angle_min: -200.0,
            test_angle_max: 1_400.0,
            test_rate_min: -500.0,
            test_rate_max: 16_000.0,
            tune_roll_rp: 0.15,
            tune_roll_rd: 0.004,
            tune_roll_sp: 4.5,
            tune_pitch_rp: 0.16,
            tune_pitch_rd: 0.005,
            tune_pitch_sp: 4.6,
            tune_yaw_rp: 0.18,
            tune_yaw_rd: 0.006,
            tune_yaw_r_lpf: 2.5,
            tune_yaw_sp: 4.8,
            test_accel_max_cdss: 80_000.0,
        }
    }
}

/// What `AutoTune::log_pids` writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogPidsLeftover {
    /// `Write_PID(LOG_PIDR_MSG, rate_roll)`.
    pub write_pidr: bool,
    /// `Write_PID(LOG_PIDP_MSG, rate_pitch)`.
    pub write_pidp: bool,
    /// `Write_PID(LOG_PIDY_MSG, rate_yaw)`.
    pub write_pidy: bool,
}

/// What the `Step::EXECUTING_TEST` logging block writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExecutingTestLogs {
    /// `Log_AutoTuneDetails()`.
    pub details: LogAtde,
    /// `attitude_control->Write_Rate(*pos_control)`.
    pub need_write_rate: bool,
    /// `log_pids()`.
    pub pids: LogPidsLeftover,
}

/// `Log_Write_AutoTune` — packs ATUN. `Write` stays a leftover.
///
/// Upstream takes `_axis` but writes the member `axis`. Callers pass
/// the member, so the packed axis is always [`LogAutoTuneView::axis`].
#[must_use]
pub fn log_write_autotune(
    axis: AxisType,
    tune_step: TuneType,
    meas_target: f32,
    meas_min: f32,
    meas_max: f32,
    new_gain_rp: f32,
    new_gain_rd: f32,
    new_gain_sp: f32,
    new_ddt: f32,
    time_us: u64,
) -> LogAtun {
    LogAtun {
        time_us,
        axis: axis as u8,
        tune_step: tune_step as u8,
        targ: meas_target * AUTOTUNE_LOG_CD_TO_DEG,
        min: meas_min * AUTOTUNE_LOG_CD_TO_DEG,
        max: meas_max * AUTOTUNE_LOG_CD_TO_DEG,
        rp: new_gain_rp,
        rd: new_gain_rd,
        sp: new_gain_sp,
        ddt: new_ddt,
    }
}

/// `Log_Write_AutoTuneDetails` — packs ATDE. `WriteStreaming` stays a leftover.
#[must_use]
pub fn log_write_autotune_details(angle_cd: f32, rate_cds: f32, time_us: u64) -> LogAtde {
    LogAtde {
        time_us,
        angle: angle_cd * AUTOTUNE_LOG_CD_TO_DEG,
        rate: rate_cds * AUTOTUNE_LOG_CD_TO_DEG,
    }
}

/// Axis gains `Log_AutoTune` feeds `Log_Write_AutoTune`.
///
/// Yaw writes `rLPF` in the RD column; `YAW_D` writes `tune_yaw_rd`.
#[must_use]
pub fn log_autotune_gains(view: &LogAutoTuneView) -> (f32, f32, f32) {
    match view.axis {
        AxisType::Roll => (view.tune_roll_rp, view.tune_roll_rd, view.tune_roll_sp),
        AxisType::Pitch => (view.tune_pitch_rp, view.tune_pitch_rd, view.tune_pitch_sp),
        AxisType::Yaw => (view.tune_yaw_rp, view.tune_yaw_r_lpf, view.tune_yaw_sp),
        AxisType::YawD => (view.tune_yaw_rp, view.tune_yaw_rd, view.tune_yaw_sp),
    }
}

/// Measurements `Log_AutoTune` feeds `Log_Write_AutoTune`.
///
/// `ANGLE_P_DOWN` / `ANGLE_P_UP` use the angle trio; every other
/// Multi tune type uses the rate trio.
#[must_use]
pub fn log_autotune_meas(view: &LogAutoTuneView) -> (f32, f32, f32) {
    if twitch_is_angle_p(view.tune_type) {
        (view.target_angle, view.test_angle_min, view.test_angle_max)
    } else {
        (view.target_rate, view.test_rate_min, view.test_rate_max)
    }
}

/// `AC_AutoTune_Multi::Log_AutoTune`.
#[must_use]
pub fn log_autotune(view: &LogAutoTuneView, time_us: u64) -> LogAtun {
    let (meas_target, meas_min, meas_max) = log_autotune_meas(view);
    let (rp, rd, sp) = log_autotune_gains(view);
    log_write_autotune(
        view.axis,
        view.tune_type,
        meas_target,
        meas_min,
        meas_max,
        rp,
        rd,
        sp,
        view.test_accel_max_cdss,
        time_us,
    )
}

/// `AC_AutoTune_Multi::Log_AutoTuneDetails`.
#[must_use]
pub fn log_autotune_details(lean_angle: f32, rotation_rate: f32, time_us: u64) -> LogAtde {
    log_write_autotune_details(lean_angle, rotation_rate, time_us)
}

/// Multi `Log_AutoTuneSweep` — empty override.
#[must_use]
pub const fn log_autotune_sweep() -> bool {
    false
}

/// Copter `AutoTune::log_pids`.
#[must_use]
pub const fn log_pids() -> LogPidsLeftover {
    LogPidsLeftover {
        write_pidr: true,
        write_pidp: true,
        write_pidy: true,
    }
}

/// `Step::EXECUTING_TEST` logging block: details, `Write_Rate`, PIDs.
#[must_use]
pub fn executing_test_logs(lean_angle: f32, rotation_rate: f32, time_us: u64) -> ExecutingTestLogs {
    ExecutingTestLogs {
        details: log_autotune_details(lean_angle, rotation_rate, time_us),
        need_write_rate: true,
        pids: log_pids(),
    }
}

/// `Step::UPDATE_GAINS` logging block: `Log_AutoTune()`.
#[must_use]
pub fn update_gains_logs(view: &LogAutoTuneView, time_us: u64) -> LogAtun {
    log_autotune(view, time_us)
}
