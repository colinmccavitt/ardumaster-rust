//! Tailsitter motor-mask output — upstream `Tailsitter::output`
//! `Q_TAILSIT_MOTMX` / `AP_MotorsMulticopter::output_motor_mask`.

use ap_quadplane::tailsitter::{
    mask_motor_actuator, motor_in_fw_mask, OutputContext, OutputKind, Tailsitter, TailsitterConfig,
    TAILSIT_MOTMX_DEFAULT,
};

fn enabled() -> Tailsitter {
    Tailsitter::setup(TailsitterConfig::tailsitter_frame())
}

fn copter_tailsitter(mask: u16) -> Tailsitter {
    let mut cfg = TailsitterConfig::new();
    cfg.frame_class = 1; // MOTOR_FRAME_QUAD
    cfg.motor_mask = mask;
    Tailsitter::setup(cfg)
}

#[test]
fn groupinfo_default_mask_is_zero() {
    assert_eq!(TAILSIT_MOTMX_DEFAULT, 0);
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert_eq!(ts.motor_mask(), TAILSIT_MOTMX_DEFAULT);
}

#[test]
fn setup_keeps_the_configured_mask() {
    let ts = copter_tailsitter(0b1111);
    assert!(ts.enabled());
    assert_eq!(ts.motor_mask(), 0b1111);
}

#[test]
fn disabled_is_silent() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.enable = Some(0);
    cfg.motor_mask = 0b0011;
    let ts = Tailsitter::setup(cfg);
    assert!(!ts.enabled());
    assert_eq!(
        ts.output_kind(OutputContext::fw_cruise()),
        OutputKind::Silent
    );
    assert_eq!(
        ts.output_kind(OutputContext::vtol_hover()),
        OutputKind::Silent
    );
}

#[test]
fn not_initialised_is_silent() {
    let ts = enabled();
    let mut ctx = OutputContext::fw_cruise();
    ctx.initialised = false;
    assert_eq!(ts.output_kind(ctx), OutputKind::Silent);
}

#[test]
fn motor_test_is_silent() {
    let ts = enabled();
    let mut ctx = OutputContext::vtol_hover();
    ctx.motor_test = true;
    assert_eq!(ts.output_kind(ctx), OutputKind::Silent);
}

#[test]
fn fw_cruise_uses_motor_mask() {
    let ts = copter_tailsitter(0b0101);
    assert_eq!(
        ts.output_kind(OutputContext::fw_cruise()),
        OutputKind::MotorMask
    );
}

#[test]
fn duo_motor_fw_still_takes_motor_mask_with_zero_bits() {
    // Duo-motor tailsitters leave MOTMX at 0; output_motor_mask is still called.
    let ts = enabled();
    assert_eq!(ts.motor_mask(), 0);
    assert_eq!(
        ts.output_kind(OutputContext::fw_cruise()),
        OutputKind::MotorMask
    );
}

#[test]
fn vtol_hover_uses_copter_output() {
    let ts = enabled();
    assert!(ts.active(true, false));
    assert_eq!(
        ts.output_kind(OutputContext::vtol_hover()),
        OutputKind::Copter
    );
}

#[test]
fn angle_wait_fw_is_active_so_copter() {
    // active() is true in ANGLE_WAIT_FW, and that is not in_vtol_transition.
    let ts = enabled();
    let mut ctx = OutputContext::fw_cruise();
    ctx.angle_wait_fw = true;
    assert!(ts.active(false, true));
    assert_eq!(ts.output_kind(ctx), OutputKind::Copter);
}

#[test]
fn fw_to_vtol_transition_uses_motor_mask() {
    let ts = enabled();
    let mut ctx = OutputContext::fw_cruise();
    ctx.in_vtol_transition = true;
    assert_eq!(ts.output_kind(ctx), OutputKind::MotorMask);
}

#[test]
fn assisted_fw_falls_through_to_copter() {
    let ts = enabled();
    let mut ctx = OutputContext::fw_cruise();
    ctx.assisted_flight = true;
    assert_eq!(ts.output_kind(ctx), OutputKind::Copter);
}

#[test]
fn assisted_vtol_transition_uses_copter() {
    let ts = enabled();
    let mut ctx = OutputContext::fw_cruise();
    ctx.in_vtol_transition = true;
    ctx.assisted_flight = true;
    assert_eq!(ts.output_kind(ctx), OutputKind::Copter);
}

#[test]
fn output_min_first_on_disarm_or_estop_not_silent() {
    assert!(Tailsitter::output_min_first(false, false));
    assert!(Tailsitter::output_min_first(true, true));
    assert!(!Tailsitter::output_min_first(true, false));
    // Disarm does not change the path: still MotorMask in FW.
    let ts = enabled();
    assert_eq!(
        ts.output_kind(OutputContext::fw_cruise()),
        OutputKind::MotorMask
    );
}

#[test]
fn mask_bits_select_fw_motors() {
    let mask = 0b1010; // motors 1 and 3
    assert!(!motor_in_fw_mask(mask, 0));
    assert!(motor_in_fw_mask(mask, 1));
    assert!(!motor_in_fw_mask(mask, 2));
    assert!(motor_in_fw_mask(mask, 3));
    assert!(!motor_in_fw_mask(0, 0));
    assert!(!motor_in_fw_mask(mask, 16));
}

#[test]
fn mask_actuator_skips_motors_outside_the_mask() {
    assert_eq!(mask_motor_actuator(0b0001, 1, 0.5, 1.0, 0.2, true), None);
}

#[test]
fn mask_actuator_is_zero_when_disarmed() {
    let v = mask_motor_actuator(0b0001, 0, 0.5, 1.0, 0.2, false);
    assert_eq!(v, Some(0.0));
}

#[test]
fn mask_actuator_adds_rudder_differential_thrust() {
    // thrust + roll_factor * rudder_dt * 0.5
    // 0.40 + 1.0 * 0.20 * 0.5 = 0.50
    let v = mask_motor_actuator(0b0011, 0, 0.40, 1.0, 0.20, true);
    assert_eq!(v, Some(0.50));
    // opposite side: 0.40 + (-1.0) * 0.20 * 0.5 = 0.30
    let v = mask_motor_actuator(0b0011, 1, 0.40, -1.0, 0.20, true);
    assert_eq!(v, Some(0.30));
}
