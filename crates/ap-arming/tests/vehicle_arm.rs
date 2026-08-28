//! Vehicle arm/disarm gate: registry orchestration.

use ap_arming::rudder_arming::{RudderArming, ARMING_RUDDER_DEFAULT};
use ap_arming::vehicle_arm::{ArmOutcome, DisarmOutcome, Method};
use ap_arming::{Arming, Check, NamedCheck, Required};

fn baro(ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Baro,
        name: "BARO",
        ok,
    }
}

fn compass(ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Compass,
        name: "COMPASS",
        ok,
    }
}

#[test]
fn passing_checks_arm_and_set_armed() {
    let mut arming = Arming::new();
    assert!(!arming.is_armed());
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(true), compass(true)],
        ARMING_RUDDER_DEFAULT,
    );
    assert_eq!(outcome, ArmOutcome::Armed { method: Method::Mavlink });
    assert!(outcome.succeeded());
    assert!(arming.is_armed());
}

#[test]
fn failing_check_refuses_and_stays_disarmed() {
    let mut arming = Arming::new();
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(false), compass(true)],
        ARMING_RUDDER_DEFAULT,
    );
    assert_eq!(
        outcome,
        ArmOutcome::ChecksFailed {
            check: Check::Baro,
            name: "BARO",
        }
    );
    assert!(!outcome.succeeded());
    assert!(!arming.is_armed());
}

#[test]
fn already_armed_refuses_without_changing_state() {
    let mut arming = Arming {
        armed: true,
        ..Arming::new()
    };
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(true)],
        ARMING_RUDDER_DEFAULT,
    );
    assert_eq!(outcome, ArmOutcome::AlreadyArmed);
    assert!(arming.is_armed());
}

#[test]
fn force_arm_skips_a_failing_registry() {
    let mut arming = Arming::new();
    let outcome = arming.arm_force(Method::Scripting, ARMING_RUDDER_DEFAULT);
    assert_eq!(
        outcome,
        ArmOutcome::Armed {
            method: Method::Scripting,
        }
    );
    assert!(arming.is_armed());

    let mut again = Arming::new();
    let forced = again.arm(
        Method::AuxSwitch,
        false,
        &[baro(false)],
        ARMING_RUDDER_DEFAULT,
    );
    assert!(forced.succeeded());
    assert!(again.is_armed());
}

#[test]
fn disarm_succeeds_when_armed() {
    let mut arming = Arming {
        armed: true,
        ..Arming::new()
    };
    let outcome = arming.disarm(Method::Mavlink, ARMING_RUDDER_DEFAULT);
    assert_eq!(
        outcome,
        DisarmOutcome::Disarmed {
            method: Method::Mavlink,
        }
    );
    assert!(outcome.succeeded());
    assert!(!arming.is_armed());
}

#[test]
fn disarm_refuses_when_already_disarmed() {
    let mut arming = Arming::new();
    let outcome = arming.disarm(Method::Mavlink, ARMING_RUDDER_DEFAULT);
    assert_eq!(outcome, DisarmOutcome::AlreadyDisarmed);
    assert!(!outcome.succeeded());
    assert!(!arming.is_armed());
}

#[test]
fn rudder_method_is_gated_by_arming_rudder() {
    let mut arming = Arming::new();
    let refused = arming.arm(Method::Rudder, true, &[baro(true)], RudderArming::Disabled);
    assert_eq!(refused, ArmOutcome::RudderRefused);
    assert!(!arming.is_armed());

    let armed = arming.arm(Method::Rudder, true, &[baro(true)], RudderArming::ArmOnly);
    assert!(armed.succeeded());
    assert!(arming.is_armed());

    let no_disarm = arming.disarm(Method::Rudder, RudderArming::ArmOnly);
    assert_eq!(no_disarm, DisarmOutcome::RudderRefused);
    assert!(arming.is_armed());

    let disarmed = arming.disarm(Method::Rudder, RudderArming::ArmOrDisarm);
    assert!(disarmed.succeeded());
    assert!(!arming.is_armed());
}

#[test]
fn mavlink_ignores_a_disabled_rudder_param() {
    let mut arming = Arming::new();
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(true)],
        RudderArming::Disabled,
    );
    assert!(outcome.succeeded());
    assert!(arming.disarm(Method::Mavlink, RudderArming::Disabled).succeeded());
}

#[test]
fn require_no_allows_arm_even_when_a_named_check_fails() {
    let mut arming = Arming {
        require: Required::No,
        ..Arming::new()
    };
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(false)],
        ARMING_RUDDER_DEFAULT,
    );
    assert_eq!(outcome, ArmOutcome::Armed { method: Method::Mavlink });
    assert!(arming.is_armed());
}

#[test]
fn first_enabled_failure_is_the_named_refusal() {
    let mut arming = Arming::new();
    let outcome = arming.arm(
        Method::Mavlink,
        true,
        &[baro(true), compass(false)],
        ARMING_RUDDER_DEFAULT,
    );
    assert_eq!(
        outcome,
        ArmOutcome::ChecksFailed {
            check: Check::Compass,
            name: "COMPASS",
        }
    );
    assert!(!arming.is_armed());
}
