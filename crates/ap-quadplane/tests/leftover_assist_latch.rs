//! Leftover assisted-flight latch extras — force_fw / spin_recovery QTUN bits.

use ap_quadplane::quadplane_completeness::{
    leftover_qtun_assist_latch_flags, QTUN_ASSIST_FW_FORCE, QTUN_ASSIST_SPIN_RECOVERY,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn leftover_assist_latches_default_clear() {
    let qp = QuadPlane::new();
    assert!(!qp.leftover_force_fw_control_recovery());
    assert!(!qp.leftover_in_spin_recovery());
    assert_eq!(qp.leftover_qtun_assist_latch_flags(), 0);
    assert_eq!(leftover_qtun_assist_latch_flags(false, false), 0);
}

#[test]
fn leftover_assist_latches_pack_qtun_bits() {
    let mut qp = available_qp();
    qp.leftover_set_force_fw_control_recovery(true);
    assert_eq!(qp.leftover_qtun_assist_latch_flags(), QTUN_ASSIST_FW_FORCE);
    qp.leftover_set_in_spin_recovery(true);
    assert_eq!(
        qp.leftover_qtun_assist_latch_flags(),
        QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
    );
    assert_eq!(
        leftover_qtun_assist_latch_flags(true, true),
        QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
    );
}

#[test]
fn leftover_assist_latches_gate_vtol_view_and_mc_control() {
    let mut qp = available_qp();
    assert!(qp.leftover_show_vtol_view(true));
    assert!(!qp.leftover_show_vtol_view(false));
    assert!(qp.leftover_use_multicopter_control(true, false));
    assert!(!qp.leftover_use_multicopter_control(false, false));
    assert!(!qp.leftover_use_multicopter_control(true, true));

    qp.leftover_set_force_fw_control_recovery(true);
    assert!(!qp.leftover_show_vtol_view(true));
    assert!(!qp.leftover_use_multicopter_control(true, false));

    qp.leftover_set_in_spin_recovery(true);
    qp.leftover_clear_recovery_latches();
    assert!(!qp.leftover_force_fw_control_recovery());
    assert!(!qp.leftover_in_spin_recovery());
    assert!(qp.leftover_show_vtol_view(true));
    assert!(qp.leftover_use_multicopter_control(true, false));
}

#[test]
fn leftover_assist_latches_do_not_rewrite_setup_or_logging() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert_eq!(qp.logging().qtun_writes(), 0);
    assert!(!qp.leftover_force_fw_control_recovery());
    assert!(!qp.in_assisted_flight());
}
