//! Compass calibration start/cancel stub, upstream `CompassCalibrator`. FW-014.
//!
//! This is the GCS MAG_CAL start/cancel gate, not inflight `COMPASS_OFS`
//! learning. `start_calibration_all` only starts instances that are healthy
//! and marked `COMPASS_USE`. `cancel_calibration_all` returns every
//! calibrator to `NOT_STARTED`. The sphere-fit / geodesic-grid solver is
//! not in this slice.

/// Upstream `CompassCalibrator::Status`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompassCalStatus {
    /// `Status::NOT_STARTED`.
    #[default]
    NotStarted = 0,
    /// `Status::WAITING_TO_START` — start accepted, delay not elapsed.
    WaitingToStart = 1,
    /// `Status::RUNNING_STEP_ONE`.
    RunningStepOne = 2,
    /// `Status::RUNNING_STEP_TWO`.
    RunningStepTwo = 3,
    /// `Status::SUCCESS`.
    Success = 4,
    /// `Status::FAILED`.
    Failed = 5,
}

impl CompassCalStatus {
    /// True while a calibrator is waiting or running, upstream `is_calibrating`.
    #[must_use]
    pub const fn is_calibrating(self) -> bool {
        matches!(
            self,
            Self::WaitingToStart | Self::RunningStepOne | Self::RunningStepTwo
        )
    }

    /// True during the two running steps, upstream `CompassCalibrator::running`.
    #[must_use]
    pub const fn running(self) -> bool {
        matches!(self, Self::RunningStepOne | Self::RunningStepTwo)
    }
}

/// Per-instance calibrator, upstream `CompassCalibrator`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CompassCalibrator {
    /// Current `CompassCalibrator::Status`.
    pub status: CompassCalStatus,
}

impl CompassCalibrator {
    /// Request start, upstream `CompassCalibrator::start`.
    ///
    /// Already-running calibrators are left untouched.
    pub fn start(&mut self) {
        if self.running() {
            return;
        }
        self.status = CompassCalStatus::WaitingToStart;
    }

    /// Request stop, upstream `CompassCalibrator::stop`.
    pub fn stop(&mut self) {
        self.status = CompassCalStatus::NotStarted;
    }

    /// Upstream `CompassCalibrator::running`.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.status.running()
    }

    /// Waiting or running, used by `Compass::is_calibrating`.
    #[must_use]
    pub const fn is_calibrating(&self) -> bool {
        self.status.is_calibrating()
    }
}

/// Start one instance, upstream `Compass::_start_calibration`.
///
/// Returns false when the instance is unhealthy or `COMPASS_USE` is off.
#[must_use]
pub fn start_calibration(
    cal: &mut CompassCalibrator,
    healthy: bool,
    use_for_yaw: bool,
) -> bool {
    if !healthy || !use_for_yaw {
        return false;
    }
    cal.start();
    true
}

/// Start every healthy `COMPASS_USE` instance, upstream `start_calibration_all`.
///
/// Returns false only when no instance started.
#[must_use]
pub fn start_calibration_all(
    cals: &mut [CompassCalibrator],
    healthy: &[bool],
    use_for_yaw: &[bool],
) -> bool {
    let n = cals.len().min(healthy.len()).min(use_for_yaw.len());
    let mut started = false;
    for i in 0..n {
        if start_calibration(&mut cals[i], healthy[i], use_for_yaw[i]) {
            started = true;
        }
    }
    started
}

/// Stop every instance, upstream `Compass::cancel_calibration_all`.
pub fn cancel_calibration_all(cals: &mut [CompassCalibrator]) {
    for cal in cals {
        cal.stop();
    }
}

/// True when any instance is waiting or running, upstream `Compass::is_calibrating`.
#[must_use]
pub fn is_calibrating(cals: &[CompassCalibrator]) -> bool {
    cals.iter().any(CompassCalibrator::is_calibrating)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_requires_healthy_and_use() {
        let mut cal = CompassCalibrator::default();
        assert!(!start_calibration(&mut cal, false, true));
        assert_eq!(cal.status, CompassCalStatus::NotStarted);
        assert!(!start_calibration(&mut cal, true, false));
        assert_eq!(cal.status, CompassCalStatus::NotStarted);
        assert!(start_calibration(&mut cal, true, true));
        assert_eq!(cal.status, CompassCalStatus::WaitingToStart);
        assert!(cal.is_calibrating());
        assert!(!cal.running());
    }

    #[test]
    fn cancel_returns_to_not_started() {
        let mut cals = [CompassCalibrator::default(), CompassCalibrator::default()];
        assert!(start_calibration_all(
            &mut cals,
            &[true, true],
            &[true, false]
        ));
        assert!(is_calibrating(&cals));
        assert_eq!(cals[0].status, CompassCalStatus::WaitingToStart);
        assert_eq!(cals[1].status, CompassCalStatus::NotStarted);
        cancel_calibration_all(&mut cals);
        assert!(!is_calibrating(&cals));
        assert_eq!(cals[0].status, CompassCalStatus::NotStarted);
    }

    #[test]
    fn start_all_false_when_none_usable() {
        let mut cals = [CompassCalibrator::default()];
        assert!(!start_calibration_all(&mut cals, &[true], &[false]));
        assert!(!is_calibrating(&cals));
    }
}
