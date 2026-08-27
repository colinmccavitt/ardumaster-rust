//! NavEKF3 update hook stub, upstream `NavEKF3::UpdateFilter`.
//!
//! Full EKF is not ported; this stub mirrors the DCM update wiring so Plane
//! can dispatch EKF3 and fall back to DCM when unhealthy.

use ap_ins::{InertialSensorFrontend, LoopTiming};

use crate::dcm_drift_loop::{
    dcm_step_with_drift_from_ins_yaw, DcmDriftLoop, DriftMotionInputs, YawUpdateInputs,
};
use crate::{Dcm, MatrixHealth};

/// Running EKF3 filter state, upstream `NavEKF3` core instance.
#[derive(Debug, Clone, Default)]
pub struct Ekf3Loop {
    /// Whether the filter reported healthy this cycle, upstream `ekfHealthy()`.
    pub healthy: bool,
    /// True after the first update attempt, upstream filter initialised.
    pub initialized: bool,
}

/// One EKF3 filter step from INS, upstream `NavEKF3_core::UpdateFilter`.
///
/// Delegates attitude to DCM until NavEKF3 lands; health follows matrix status.
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
    let health = dcm_step_with_drift_from_ins_yaw(dcm, drift, ins, timing, yaw, motion);
    ekf.initialized = true;
    ekf.healthy = health == MatrixHealth::Ok;
    health
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_ins::LoopTiming;

    #[test]
    fn ekf3_stub_marks_healthy_on_ok_matrix() {
        let mut ekf = Ekf3Loop::default();
        let mut dcm = Dcm::new();
        let mut drift = DcmDriftLoop::default();
        let ins = InertialSensorFrontend::default();
        let timing = LoopTiming::new(1.0 / 400.0);

        let health = ekf3_step_from_ins(
            &mut ekf,
            &mut dcm,
            &mut drift,
            &ins,
            &timing,
            None,
            DriftMotionInputs::default(),
        );

        assert_eq!(health, MatrixHealth::Ok);
        assert!(ekf.initialized);
        assert!(ekf.healthy);
    }
}
