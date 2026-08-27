//! AHRS pre-arm gate hookup tests.

use ap_plane::ahrs_pre_arm_hookup::{ahrs_pre_arm_gate, plane_pre_arm_checks, AHRS_REFUSAL};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

#[test]
fn plane_pre_arm_refuses_when_ahrs_unhealthy() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks(mode, false),
        PreArmResult::Refused(AHRS_REFUSAL)
    );
}

#[test]
fn plane_pre_arm_preserves_mode_refusal() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks(mode, true),
        PreArmResult::Refused("not armable here")
    );
}

#[test]
fn ahrs_pre_arm_gate_honours_force() {
    assert!(ahrs_pre_arm_gate(false, true));
    assert!(!ahrs_pre_arm_gate(false, false));
    assert!(ahrs_pre_arm_gate(true, false));
}
