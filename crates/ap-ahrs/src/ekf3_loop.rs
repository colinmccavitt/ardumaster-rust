//! NavEKF3 full update stub, upstream `NavEKF3::UpdateFilter`.
//!
//! Full EKF is not ported; this stub mirrors the DCM update wiring so Plane
//! can dispatch EKF3 and fall back to DCM when unhealthy.

use ap_ins::{InertialSensorFrontend, LoopTiming};

use crate::dcm_drift_loop::{
    dcm_step_with_drift_from_ins_yaw, DcmDriftLoop, DriftMotionInputs, YawUpdateInputs,
};
use crate::{Dcm, MatrixHealth};

/// Outcome of one EKF3 filter cycle, upstream `NavEKF3::UpdateFilter` status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ekf3UpdateOutcome {
    /// Matrix health from the delegated DCM step.
    pub health: MatrixHealth,
    /// Whether `ekfHealthy()` would report true this cycle.
    pub healthy: bool,
}

/// Running EKF3 filter state, upstream `NavEKF3` core instance.
#[derive(Debug, Clone, Default)]
pub struct Ekf3Loop {
    /// Whether the filter reported healthy this cycle, upstream `ekfHealthy()`.
    pub healthy: bool,
    /// True after the first update attempt, upstream filter initialised.
    pub initialized: bool,
    /// Filter update count since boot, upstream `_framesSincePredict`.
    pub update_count: u32,
}

/// Full EKF3 update from INS, yaw, and motion fusion inputs.
///
/// Delegates attitude to DCM until NavEKF3 lands; marks unhealthy when the
/// matrix needs reset so [`active_backend_kind`](crate::active_backend_kind)
/// can fall back to DCM.
#[must_use]
pub fn ekf3_full_update_from_ins(
    ekf: &mut Ekf3Loop,
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    ins: &InertialSensorFrontend,
    timing: &LoopTiming,
    yaw: Option<YawUpdateInputs>,
    motion: DriftMotionInputs,
) -> Ekf3UpdateOutcome {
    let health = dcm_step_with_drift_from_ins_yaw(dcm, drift, ins, timing, yaw, motion);
    ekf.update_count = ekf.update_count.wrapping_add(1);
    ekf.initialized = true;
    ekf.healthy = health == MatrixHealth::Ok;
    Ekf3UpdateOutcome {
        health,
        healthy: ekf.healthy,
    }
}

/// One EKF3 filter step from INS, upstream `NavEKF3_core::UpdateFilter`.
///
/// Delegates to [`ekf3_full_update_from_ins`] and returns matrix health only.
#[must_use]
pub fn ekf3_step_from_ins(
    ekf: &mut Ekf3Loop,
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    ins: &InertialSensorFrontend,
    timing: &LoopTiming,
    yaw: Option<YawUpdateInputs>,
    motion: DriftMotionInputs,
) -> MatrixHealth {
    ekf3_full_update_from_ins(ekf, dcm, drift, ins, timing, yaw, motion).health
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_ins::LoopTiming;

    #[test]
    fn ekf3_full_update_marks_healthy_on_ok_matrix() {
        let mut ekf = Ekf3Loop::default();
        let mut dcm = Dcm::new();
        let mut drift = DcmDriftLoop::default();
        let ins = InertialSensorFrontend::default();
        let timing = LoopTiming::new(1.0 / 400.0);

        let outcome = ekf3_full_update_from_ins(
            &mut ekf,
            &mut dcm,
            &mut drift,
            &ins,
            &timing,
            None,
            DriftMotionInputs::default(),
        );

        assert_eq!(outcome.health, MatrixHealth::Ok);
        assert!(outcome.healthy);
        assert!(ekf.initialized);
        assert!(ekf.healthy);
        assert_eq!(ekf.update_count, 1);
    }

    #[test]
    fn ekf3_full_update_increments_update_count() {
        let mut ekf = Ekf3Loop::default();
        let mut dcm = Dcm::new();
        let mut drift = DcmDriftLoop::default();
        let ins = InertialSensorFrontend::default();
        let timing = LoopTiming::new(1.0 / 400.0);

        for expected in 1..=3 {
            ekf3_full_update_from_ins(
                &mut ekf,
                &mut dcm,
                &mut drift,
                &ins,
                &timing,
                None,
                DriftMotionInputs::default(),
            );
            assert_eq!(ekf.update_count, expected);
        }
    }

    #[test]
    fn ekf3_marks_unhealthy_on_needs_reset() {
        let mut ekf = Ekf3Loop::default();
        let mut dcm = Dcm::new();
        dcm.matrix.a.x = f32::NAN;
        let mut drift = DcmDriftLoop::default();
        let ins = InertialSensorFrontend::default();
        let timing = LoopTiming::new(1.0 / 400.0);

        let outcome = ekf3_full_update_from_ins(
            &mut ekf,
            &mut dcm,
            &mut drift,
            &ins,
            &timing,
            None,
            DriftMotionInputs::default(),
        );

        assert_eq!(outcome.health, MatrixHealth::NeedsReset);
        assert!(!outcome.healthy);
        assert!(!ekf.healthy);
    }
}
