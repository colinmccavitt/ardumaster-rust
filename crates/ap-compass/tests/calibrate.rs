//! Compass calibration start/cancel: `Compass::start_calibration_all`.

use ap_compass::calibrate::{
    cancel_calibration_all, is_calibrating, start_calibration_all, CompassCalStatus,
    CompassCalibrator,
};

#[test]
fn start_all_skips_unused_instance() {
    let mut cals = [CompassCalibrator::default(), CompassCalibrator::default()];
    assert!(start_calibration_all(
        &mut cals,
        &[true, true],
        &[false, true]
    ));
    assert_eq!(cals[0].status, CompassCalStatus::NotStarted);
    assert_eq!(cals[1].status, CompassCalStatus::WaitingToStart);
    assert!(is_calibrating(&cals));
}

#[test]
fn cancel_all_clears_waiting_calibrators() {
    let mut cals = [CompassCalibrator::default(), CompassCalibrator::default()];
    assert!(start_calibration_all(
        &mut cals,
        &[true, true],
        &[true, true]
    ));
    cancel_calibration_all(&mut cals);
    assert!(!is_calibrating(&cals));
    assert!(cals.iter().all(|c| c.status == CompassCalStatus::NotStarted));
}
