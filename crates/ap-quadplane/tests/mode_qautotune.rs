//! ModeQAutotune + `QAutoTune::init` — upstream `mode_qautotune.cpp`
//! / `qautotune.cpp`.

use ap_quadplane::mode_q::{
    qstabilize_update, QManualUpdatePath, QManualUpdateView, MODE_QHOVER, MODE_QSTABILIZE,
};
use ap_quadplane::mode_qautotune::{
    leftover_desired_climb_rate_ms, leftover_init_z_limits, leftover_log_pids,
    leftover_pilot_desired_rp_yrate_rad, mode_qautotune_surfaces_complete, qautotune_enter,
    qautotune_exit, qautotune_run, qautotune_update, ModeQAutotune, QAutoTune, QAutotunePortStatus,
    QAutotuneRunAction, QAutotuneRunView, MODE_QAUTOTUNE, MODE_QAUTOTUNE_NAME,
    MODE_QAUTOTUNE_NAME4, MODE_QAUTOTUNE_SURFACES,
};
use ap_quadplane::mode_qland::MODE_QLOITER;
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

fn dirty_for_mode_enter(qp: &mut QuadPlane) {
    qp.set_lean_angle_max_cd(4500);
    qp.set_throttle_wait(true);
    qp.set_guided_wait_takeoff(true);
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    qp.poscontrol_mut().set_correction_ne_m(4.0, -2.0);
}

fn approx(a: f32, b: f32) -> bool {
    let d = if a > b { a - b } else { b - a };
    d < 1.0e-5
}

#[test]
fn qautotune_mode_numbers_and_flags_match_upstream() {
    assert_eq!(MODE_QAUTOTUNE, 22);
    assert_eq!(MODE_QAUTOTUNE_NAME, "QAutotune");
    assert_eq!(MODE_QAUTOTUNE_NAME4, "QATN");
    assert_eq!(ModeQAutotune.mode_number(), MODE_QAUTOTUNE);
    assert_eq!(ModeQAutotune.name(), MODE_QAUTOTUNE_NAME);
    assert_eq!(ModeQAutotune.name4(), MODE_QAUTOTUNE_NAME4);
    assert_eq!(ModeQAutotune::from_number(22), Some(ModeQAutotune));
    assert_eq!(ModeQAutotune::from_number(17), None);
    assert_eq!(ModeQAutotune::from_number(19), None);
    assert!(ModeQAutotune.is_vtol_mode());
    assert!(ModeQAutotune.is_vtol_man_mode());
    assert!(!ModeQAutotune.is_vtol_man_throttle());
}

#[test]
fn qautotune_init_refuses_when_quadplane_unavailable() {
    let qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    let mut tune = QAutoTune::new();
    assert!(!tune.init(&qp, MODE_QLOITER));
    assert!(!tune.internals_inited());
    assert!(!tune.position_hold());
}

#[test]
fn qautotune_init_position_hold_only_from_qloiter() {
    let qp = available_qp();
    let mut tune = QAutoTune::new();

    assert!(tune.init(&qp, MODE_QLOITER));
    assert!(tune.internals_inited());
    assert!(tune.position_hold());

    assert!(tune.init(&qp, MODE_QHOVER));
    assert!(tune.internals_inited());
    assert!(!tune.position_hold());

    assert!(tune.init(&qp, MODE_QSTABILIZE));
    assert!(!tune.position_hold());

    assert!(tune.init(&qp, MODE_QAUTOTUNE));
    assert!(!tune.position_hold());
}

#[test]
fn qautotune_enter_calls_mode_enter_then_init() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    let mut tune = QAutoTune::new();

    assert!(qautotune_enter(&mut qp, &mut tune, MODE_QLOITER));

    assert!(tune.internals_inited());
    assert!(tune.position_hold());
    assert!(!tune.stopped());
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
    assert!(!qp.guided_wait_takeoff());
    assert!(qp.guided_wait_takeoff_on_mode_enter());
}

#[test]
fn qautotune_enter_fails_without_setup() {
    let mut qp = QuadPlane::with_enable(1);
    dirty_for_mode_enter(&mut qp);
    let mut tune = QAutoTune::new();

    assert!(!qautotune_enter(&mut qp, &mut tune, MODE_QLOITER));
    assert!(!tune.internals_inited());
    // mode_enter still ran; unavailable skips the lean-angle write.
    assert_eq!(qp.lean_angle_max_cd(), 4500);
    assert!(qp.poscontrol().mode_enter_cleared());
    assert!(!qp.guided_wait_takeoff());
}

#[test]
fn qautotune_update_delegates_to_qstabilize() {
    let mut view = QManualUpdateView::flying();
    view.roll_input = 0.5;
    view.pitch_input = -0.25;
    let out = qautotune_update(&view);
    let stab = qstabilize_update(&view);
    assert_eq!(out, stab);
    assert_eq!(out.path, QManualUpdatePath::LimitedFw);
}

#[test]
fn qautotune_run_tunes_then_stabilizes() {
    let mut tune = QAutoTune::new();
    let out = qautotune_run(&mut tune, &QAutotuneRunView::flying());
    assert_eq!(out.action, QAutotuneRunAction::TuneThenStabilize);
    assert!(out.tune_ran);
    assert!(out.stabilize_roll);
    assert!(out.stabilize_pitch);
    assert!(out.rudder_centered);
    assert!(tune.ran());
}

#[test]
fn qautotune_run_tailsitter_fw_skips_tune() {
    let mut tune = QAutoTune::new();
    let out = qautotune_run(&mut tune, &QAutotuneRunView::tailsitter_fw_transition());
    assert_eq!(out.action, QAutotuneRunAction::FwControllers);
    assert!(!out.tune_ran);
    assert!(!out.stabilize_roll);
    assert!(!out.stabilize_pitch);
    assert!(!out.rudder_centered);
    assert!(!tune.ran());
}

#[test]
fn qautotune_exit_stops_tuner() {
    let mut tune = QAutoTune::new();
    let qp = available_qp();
    assert!(tune.init(&qp, MODE_QHOVER));
    assert!(!tune.stopped());
    qautotune_exit(&mut tune);
    assert!(tune.stopped());
}

#[test]
fn leftover_qautotune_cpp_hooks() {
    assert!(approx(leftover_desired_climb_rate_ms(250.0), 2.5));
    assert!(approx(leftover_desired_climb_rate_ms(0.0), 0.0));

    let centered = leftover_pilot_desired_rp_yrate_rad(0, 0, 1500, -800, 4500.0);
    assert!(centered.sticks_centered);
    assert!(approx(centered.des_roll_rad, 0.0));
    assert!(approx(centered.des_pitch_rad, 0.0));
    assert!(centered.des_yaw_rate_rads > 0.0);

    let leaned = leftover_pilot_desired_rp_yrate_rad(100, 0, 1500, -800, 0.0);
    assert!(!leaned.sticks_centered);
    assert!(leaned.des_roll_rad > 0.0);
    assert!(leaned.des_pitch_rad < 0.0);
    assert!(approx(leaned.des_yaw_rate_rads, 0.0));

    let z = leftover_init_z_limits();
    assert!(z.d_speed_accel_set);
    assert!(z.d_correction_set);

    let pids = leftover_log_pids();
    assert!(pids.piqr && pids.piqp && pids.piqy);
}

#[test]
fn mode_qautotune_surfaces_are_complete() {
    assert!(mode_qautotune_surfaces_complete());
    assert_eq!(MODE_QAUTOTUNE_SURFACES.len(), 9);
    let names: [&str; 9] = [
        "_enter",
        "update",
        "run",
        "_exit",
        "init",
        "get_desired_climb_rate_ms",
        "get_pilot_desired_rp_yrate_rad",
        "init_z_limits",
        "log_pids",
    ];
    for (i, row) in MODE_QAUTOTUNE_SURFACES.iter().enumerate() {
        assert_eq!(row.name, names[i]);
        assert_eq!(row.status, QAutotunePortStatus::ThisSlice);
    }
    assert_eq!(MODE_QAUTOTUNE_SURFACES[0].file, "mode_qautotune.cpp");
    assert_eq!(MODE_QAUTOTUNE_SURFACES[4].file, "qautotune.cpp");
    assert_eq!(MODE_QAUTOTUNE_SURFACES[8].file, "qautotune.cpp");
}
