//! ATRP 25 Hz log stub (upstream `log_ATRP` WriteBlock every 40 ms).

use ap_autotune::action::Action;
use ap_autotune::completeness::{completeness_has, PortStatus};
use ap_autotune::gains::AtGains;
use ap_autotune::log::{
    should_log_atrp, stamp_log_ms, AtrpLogGate, LogAtrp, ATRP_FORMAT, ATRP_LABELS, ATRP_LOG_HZ,
    ATRP_LOG_PERIOD_MS, ATRP_NAME, HEAD_BYTE1, HEAD_BYTE2,
};
use ap_autotune::state::{AtState, AtType, AutoTune};

fn close(got: f32, expect: f32) {
    assert!((got - expect).abs() < 1e-6, "{got} != {expect}");
}

#[test]
fn atrp_period_is_25_hz() {
    assert_eq!(ATRP_LOG_PERIOD_MS, 40);
    assert_eq!(ATRP_LOG_HZ, 25);
}

#[test]
fn atrp_logger_format_matches_upstream() {
    assert_eq!(ATRP_NAME, "ATRP");
    assert_eq!(ATRP_FORMAT, "QBBffffffffBff");
    assert_eq!(
        ATRP_LABELS,
        "TimeUS,Axis,State,Sur,PSlew,DSlew,FF0,FF,P,I,D,Action,RMAX,TAU"
    );
    assert_eq!(HEAD_BYTE1, 0xA3);
    assert_eq!(HEAD_BYTE2, 0x95);
}

#[test]
fn should_log_atrp_gates_on_40_ms() {
    assert!(!should_log_atrp(0, 0));
    assert!(!should_log_atrp(39, 0));
    assert!(should_log_atrp(40, 0));
    assert!(should_log_atrp(41, 0));
    assert!(!should_log_atrp(139, 100));
    assert!(should_log_atrp(140, 100));
}

#[test]
fn should_log_atrp_uses_unsigned_wrap() {
    assert!(!should_log_atrp(10, u32::MAX - 10));
    assert!(should_log_atrp(30, u32::MAX - 10));
}

#[test]
fn stamp_log_ms_is_now() {
    assert_eq!(stamp_log_ms(1234), 1234);
}

#[test]
fn packet_fields_come_from_the_session() {
    let mut tuner = AutoTune::with_gains(
        AtType::Pitch,
        AtGains {
            tau: 0.5,
            rmax_pos: 75.0,
            rmax_neg: 75.0,
            p: 0.4,
            i: 0.3,
            d: 0.02,
        },
    );
    tuner.ff = 0.15;
    tuner.start();
    tuner.state = AtState::DemandPos;

    let pkt = LogAtrp::from_session(&tuner, 1_000_000, 12.5, 3.0, 1.5, 0.11, Action::RaiseD);
    assert_eq!(pkt.head1, HEAD_BYTE1);
    assert_eq!(pkt.head2, HEAD_BYTE2);
    assert_eq!(pkt.msgid, 0);
    assert_eq!(pkt.time_us, 1_000_000);
    assert_eq!(pkt.axis, AtType::Pitch.as_u8());
    assert_eq!(pkt.state, AtState::DemandPos.as_u8());
    close(pkt.actuator, 12.5);
    close(pkt.p_slew, 3.0);
    close(pkt.d_slew, 1.5);
    close(pkt.ff_single, 0.11);
    close(pkt.ff, tuner.ff);
    close(pkt.p, 0.4);
    close(pkt.i, 0.3);
    close(pkt.d, 0.02);
    assert_eq!(pkt.action, Action::RaiseD.as_u8());
    close(pkt.rmax, 75.0);
    close(pkt.tau, 0.5);
}

#[test]
fn gate_emits_at_25_hz_and_not_faster() {
    let tuner = AutoTune::new(AtType::Roll);
    let mut gate = AtrpLogGate::new();
    assert_eq!(gate.last_log_ms, 0);
    assert!(gate
        .maybe_write(39, &tuner, 0, 0.0, 0.0, 0.0, 0.0, Action::None)
        .is_none());
    assert!(gate
        .maybe_write(40, &tuner, 40_000, 1.0, 2.0, 3.0, 0.2, Action::RaiseP)
        .is_some());
    assert_eq!(gate.last_log_ms, 40);
    assert!(gate
        .maybe_write(79, &tuner, 0, 0.0, 0.0, 0.0, 0.0, Action::None)
        .is_none());
    let again = gate
        .maybe_write(80, &tuner, 80_000, 4.0, 5.0, 6.0, 0.3, Action::LowerD)
        .expect("second 25 Hz slot");
    assert_eq!(gate.last_log_ms, 80);
    assert_eq!(again.time_us, 80_000);
    assert_eq!(again.action, Action::LowerD.as_u8());
    close(again.actuator, 4.0);
}

#[test]
fn default_gate_matches_new() {
    assert_eq!(AtrpLogGate::default(), AtrpLogGate::new());
}

#[test]
fn log_atrp_is_this_slice() {
    assert!(completeness_has("log_ATRP 25Hz", PortStatus::ThisSlice));
    assert!(!completeness_has("log_ATRP 25Hz", PortStatus::Remaining));
}
