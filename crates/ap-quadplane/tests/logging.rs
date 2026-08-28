//! QuadPlane leftover logging — QTUN / QPOS / AttRate.

use ap_quadplane::logging::{
    assemble_qtun, pack_qtun_assist, qpos_period_elapsed, qtun_motors_recent, qtun_period_elapsed,
    LogUpdateView, QPosView, QTunView, MOTB_PERIOD_MS, QPOS_PERIOD_MS, QTUN_ACTIVE_HOLD_MS,
};
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::quadplane_completeness::{
    leftover_option_is_set, qtun_assist_flags, LeftoverQOption, QTUN_ASSIST_FW_FORCE,
    QTUN_ASSIST_IN_ASSISTED_FLIGHT, QTUN_ASSIST_SPIN_RECOVERY, QTUN_PERIOD_MS,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn logging_defaults_and_periods_match_upstream() {
    let qp = QuadPlane::new();
    assert_eq!(qp.logging().qtun_writes(), 0);
    assert_eq!(qp.logging().qpos_writes(), 0);
    assert_eq!(qp.logging().att_rate_writes(), 0);
    assert_eq!(qp.logging().last_qtun_log_ms(), 0);
    assert_eq!(QPOS_PERIOD_MS, QTUN_PERIOD_MS);
    assert_eq!(QTUN_ACTIVE_HOLD_MS, 250);
    assert_eq!(MOTB_PERIOD_MS, 100);
    assert!(qtun_period_elapsed(41, 0));
    assert!(!qtun_period_elapsed(40, 0));
    assert!(qpos_period_elapsed(40, 0));
    assert!(!qpos_period_elapsed(39, 0));
    assert!(qtun_motors_recent(200, false, 0));
    assert!(!qtun_motors_recent(250, false, 0));
}

#[test]
fn log_write_qcontrol_tuning_packs_assist_and_qstabilize() {
    let mut qp = available_qp();
    qp.set_assisted_flight(true);
    let mut view = QTunView::hover();
    view.fw_force_recovery = true;
    view.spin_recovery = true;
    let pkt = qp.log_write_qcontrol_tuning(view);
    assert_eq!(pkt.assist, pack_qtun_assist(true, view));
    assert_eq!(
        pkt.assist,
        qtun_assist_flags(true, false, false, false, false, true, true)
    );
    assert_eq!(
        pkt.assist,
        QTUN_ASSIST_IN_ASSISTED_FLIGHT | QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
    );
    assert_eq!(pkt.transition_state, 2);

    let stab = qp.log_write_qcontrol_tuning(QTunView::qstabilize());
    assert_eq!(stab.desired_alt as i32, 0);
    assert_eq!(stab.target_climb_rate, 0);
    assert_eq!(assemble_qtun(false, QTunView::qstabilize()).assist, 0);
    assert_eq!(qp.logging().qtun_writes(), 2);
}

#[test]
fn log_qpos_and_att_rate_and_update_gate() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    let qpos = qp.log_qpos(QPosView {
        wp_distance: 3.0,
        target_speed_ms: 1.5,
        target_accel_mss: 0.8,
        overshoot: true,
    });
    assert_eq!(qpos.state, PositionControlState::LandDescend);
    assert!(qpos.overshoot);
    qp.log_write_att_rate();
    assert_eq!(qp.logging().att_rate_writes(), 1);

    qp.set_motors_armed(true);
    let r = qp.maybe_log_update(LogUpdateView::vtol_hover(50));
    assert!(r.wrote_qtun);
    assert!(r.wrote_ang);
    assert!(r.wrote_rate);

    let mut fw = LogUpdateView::vtol_hover(400);
    fw.in_vtol_mode = false;
    fw.last_motors_active_ms = 0;
    let quiet = qp.maybe_log_update(fw);
    assert!(!quiet.wrote_ang);
    assert!(!quiet.wrote_qtun);
}

#[test]
fn leftover_q_options_helpers_are_untouched() {
    assert!(leftover_option_is_set(
        LeftoverQOption::FsQrtl.as_i32(),
        LeftoverQOption::FsQrtl
    ));
    let qp = available_qp();
    assert!(qp.available());
    assert!(!qp.in_assisted_flight());
}
