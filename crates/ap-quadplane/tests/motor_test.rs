//! QuadPlane motor-test — upstream `mavlink_motor_test_start` /
//! `motor_test_output` / `motor_test_stop` (Plane-4.7.0).

use ap_quadplane::motor_test::{
    pwm_in_rc_limits, throttle_to_pwm, timeout_ms_from_sec, MavResult, MotorTestOutputView,
    MotorTestStart, MotorTestThrottleType, MotorTestTick, MOTOR_PWM_MAX_DEFAULT,
    MOTOR_PWM_MIN_DEFAULT, MOTOR_TEST_MOTOR_COUNT_MAX, MOTOR_TEST_TIMEOUT_MS_MAX, RC_MAX_LIMIT_PWM,
    RC_MIN_LIMIT_PWM,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn constants_match_upstream() {
    assert_eq!(MOTOR_TEST_TIMEOUT_MS_MAX, 30_000);
    assert_eq!(MOTOR_TEST_MOTOR_COUNT_MAX, 8);
    assert_eq!(RC_MIN_LIMIT_PWM, 800);
    assert_eq!(RC_MAX_LIMIT_PWM, 2200);
    assert_eq!(MotorTestThrottleType::Percent as u8, 0);
    assert_eq!(MotorTestThrottleType::Pwm as u8, 1);
    assert_eq!(MotorTestThrottleType::Pilot as u8, 2);
}

#[test]
fn timeout_is_capped_at_30s() {
    assert_eq!(timeout_ms_from_sec(1.0), 1000);
    assert_eq!(timeout_ms_from_sec(30.0), MOTOR_TEST_TIMEOUT_MS_MAX);
    assert_eq!(timeout_ms_from_sec(60.0), MOTOR_TEST_TIMEOUT_MS_MAX);
    assert_eq!(timeout_ms_from_sec(0.0), 0);
    assert_eq!(timeout_ms_from_sec(-1.0), 0);
}

#[test]
fn percent_pwm_lerps_motors_span() {
    let pwm = throttle_to_pwm(
        MotorTestThrottleType::Percent as u8,
        50,
        MOTOR_PWM_MIN_DEFAULT,
        MOTOR_PWM_MAX_DEFAULT,
        0.0,
    );
    assert_eq!(pwm, Some(1500));
    let zero = throttle_to_pwm(
        MotorTestThrottleType::Percent as u8,
        0,
        MOTOR_PWM_MIN_DEFAULT,
        MOTOR_PWM_MAX_DEFAULT,
        0.0,
    );
    assert_eq!(zero, Some(MOTOR_PWM_MIN_DEFAULT));
    let full = throttle_to_pwm(
        MotorTestThrottleType::Percent as u8,
        100,
        MOTOR_PWM_MIN_DEFAULT,
        MOTOR_PWM_MAX_DEFAULT,
        0.0,
    );
    assert_eq!(full, Some(MOTOR_PWM_MAX_DEFAULT));
}

#[test]
fn percent_over_100_is_pwm_zero_then_out_of_limits() {
    let pwm = throttle_to_pwm(
        MotorTestThrottleType::Percent as u8,
        101,
        MOTOR_PWM_MIN_DEFAULT,
        MOTOR_PWM_MAX_DEFAULT,
        0.0,
    );
    assert_eq!(pwm, Some(0));
    assert!(!pwm_in_rc_limits(0));
}

#[test]
fn start_fails_when_unavailable() {
    let mut qp = QuadPlane::new();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Failed
    );
    assert!(!qp.motor_test_running());
    assert!(!qp.motors_armed());
}

#[test]
fn start_fails_when_already_armed() {
    let mut qp = available_qp();
    qp.set_motors_armed(true);
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Failed
    );
    assert!(!qp.motor_test_running());
}

#[test]
fn start_fails_when_checks_reject() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.checks_ok = false;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Failed);
    assert!(!qp.motor_test_running());
}

#[test]
fn start_accepts_and_arms() {
    let mut qp = available_qp();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Accepted
    );
    assert!(qp.motor_test_running());
    assert!(qp.motors_armed());
    assert_eq!(qp.motor_test().seq(), 1);
    assert_eq!(qp.motor_test().timeout_ms(), 1000);
    assert_eq!(qp.motor_test().throttle_value(), 50);
    assert_eq!(qp.motor_test().motor_count(), 1);
}

#[test]
fn start_retargets_while_running_even_if_armed() {
    let mut qp = available_qp();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Accepted
    );
    let mut req = MotorTestStart::percent50();
    req.motor_seq = 3;
    req.throttle_value = 25;
    req.now_ms = 200;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(qp.motor_test().seq(), 3);
    assert_eq!(qp.motor_test().throttle_value(), 25);
    assert_eq!(qp.motor_test().start_ms(), 200);
    assert!(qp.motors_armed());
}

#[test]
fn start_caps_motor_count_at_eight() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.motor_count = 20;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(qp.motor_test().motor_count(), MOTOR_TEST_MOTOR_COUNT_MAX);
}

#[test]
fn output_idle_when_not_running() {
    let mut qp = available_qp();
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(0)),
        MotorTestTick::Idle
    );
}

#[test]
fn output_drives_percent_pwm() {
    let mut qp = available_qp();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Accepted
    );
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(0)),
        MotorTestTick::Drive { seq: 1, pwm: 1500 }
    );
}

#[test]
fn output_drives_absolute_pwm() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.throttle_type = MotorTestThrottleType::Pwm as u8;
    req.throttle_value = 1600;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(0)),
        MotorTestTick::Drive { seq: 1, pwm: 1600 }
    );
}

#[test]
fn output_drives_pilot_throttle() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.throttle_type = MotorTestThrottleType::Pilot as u8;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    let mut view = MotorTestOutputView::at(0);
    view.pilot_throttle = 25.0;
    assert_eq!(
        qp.motor_test_output(&view),
        MotorTestTick::Drive { seq: 1, pwm: 1250 }
    );
}

#[test]
fn output_stops_on_unknown_throttle_type() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.throttle_type = 9;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(0)),
        MotorTestTick::Stopped
    );
    assert!(!qp.motor_test_running());
    assert!(!qp.motors_armed());
}

#[test]
fn output_stops_on_pwm_outside_rc_limits() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.throttle_type = MotorTestThrottleType::Pwm as u8;
    req.throttle_value = 500;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(0)),
        MotorTestTick::Stopped
    );
    assert!(!qp.motor_test_running());
}

#[test]
fn output_stops_when_output_seq_fails() {
    let mut qp = available_qp();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Accepted
    );
    let mut view = MotorTestOutputView::at(0);
    view.output_seq_ok = false;
    assert_eq!(qp.motor_test_output(&view), MotorTestTick::Stopped);
    assert!(!qp.motor_test_running());
}

#[test]
fn single_motor_timeout_stops() {
    let mut qp = available_qp();
    assert_eq!(
        qp.mavlink_motor_test_start(&MotorTestStart::percent50()),
        MavResult::Accepted
    );
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(999)),
        MotorTestTick::Drive { seq: 1, pwm: 1500 }
    );
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(1000)),
        MotorTestTick::Stopped
    );
    assert!(!qp.motor_test_running());
    assert!(!qp.motors_armed());
    assert_eq!(qp.motor_test().start_ms(), 0);
    assert_eq!(qp.motor_test().timeout_ms(), 0);
}

#[test]
fn multi_motor_zero_then_advances() {
    let mut qp = available_qp();
    let mut req = MotorTestStart::percent50();
    req.motor_count = 2;
    req.timeout_sec = 1.0;
    assert_eq!(qp.mavlink_motor_test_start(&req), MavResult::Accepted);
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(1000)),
        MotorTestTick::OutputMin
    );
    assert_eq!(qp.motor_test().seq(), 1);
    assert_eq!(qp.motor_test().motor_count(), 2);
    // 1.5×timeout advances to the next motor.
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(1500)),
        MotorTestTick::NextMotor
    );
    assert_eq!(qp.motor_test().seq(), 2);
    assert_eq!(qp.motor_test().motor_count(), 1);
    assert_eq!(qp.motor_test().start_ms(), 1500);
    assert_eq!(
        qp.motor_test_output(&MotorTestOutputView::at(1500)),
        MotorTestTick::Drive { seq: 2, pwm: 1500 }
    );
}

#[test]
fn stop_is_idempotent_when_idle() {
    let mut qp = available_qp();
    qp.motor_test_stop();
    assert!(!qp.motor_test_running());
    assert!(!qp.motors_armed());
}

#[test]
fn set_motors_armed_is_noop_before_setup() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_motors_armed(true);
    assert!(!qp.motors_armed());
}
