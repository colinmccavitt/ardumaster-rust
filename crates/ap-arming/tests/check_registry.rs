//! AP_Arming check registry: ARMING_REQUIRE / ARMING_SKIPCHK gate.

use ap_arming::{
    Arming, Check, NamedCheck, PreArmOutcome, Required, ARMING_REQUIRE_DEFAULT, ARMING_SKIPCHK_DEFAULT,
    CHECK_MASK,
};

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
fn plane_default_require_is_yes_min_pwm() {
    assert_eq!(ARMING_REQUIRE_DEFAULT, Required::YesMinPwm);
    assert_eq!(Required::from_u8(1), Some(Required::YesMinPwm));
    assert_eq!(Arming::new().arming_required(), Required::YesMinPwm);
    assert_eq!(ARMING_SKIPCHK_DEFAULT, 0);
}

#[test]
fn require_no_skips_the_registry_even_when_a_check_fails() {
    let arming = Arming {
        require: Required::No,
        ..Arming::new()
    };
    assert_eq!(arming.arming_required(), Required::No);
    assert_eq!(
        arming.pre_arm_checks(&[baro(false)]),
        PreArmOutcome::Allowed
    );
}

#[test]
fn already_armed_skips_the_registry() {
    let arming = Arming {
        armed: true,
        ..Arming::new()
    };
    assert!(arming.pre_arm_checks(&[baro(false)]).allowed());
}

#[test]
fn an_enabled_named_check_can_refuse() {
    let arming = Arming::new();
    assert!(arming.check_enabled(Check::Baro));
    assert_eq!(
        arming.pre_arm_checks(&[baro(false), compass(true)]),
        PreArmOutcome::Refused {
            check: Check::Baro,
            name: "BARO",
        }
    );
}

#[test]
fn skipping_a_named_check_lets_a_failure_through() {
    let arming = Arming {
        checks_to_skip: Check::Baro.as_u32(),
        ..Arming::new()
    };
    assert!(!arming.check_enabled(Check::Baro));
    assert!(arming.check_enabled(Check::Compass));
    assert_eq!(
        arming.pre_arm_checks(&[baro(false), compass(true)]),
        PreArmOutcome::Allowed
    );
}

#[test]
fn skipping_every_named_check_allows() {
    let arming = Arming {
        checks_to_skip: CHECK_MASK,
        ..Arming::new()
    };
    assert!(arming.should_skip_all_checks());
    assert_eq!(arming.get_enabled_checks(), 0);
    assert!(arming.pre_arm_checks(&[baro(false)]).allowed());
}

#[test]
fn the_first_enabled_failure_is_the_named_refusal() {
    let arming = Arming::new();
    assert_eq!(
        arming.pre_arm_checks(&[baro(true), compass(false)]),
        PreArmOutcome::Refused {
            check: Check::Compass,
            name: "COMPASS",
        }
    );
}

#[test]
fn all_passing_named_checks_allow() {
    let arming = Arming::new();
    assert_eq!(
        arming.pre_arm_checks(&[baro(true), compass(true)]),
        PreArmOutcome::Allowed
    );
    assert_ne!(arming.get_enabled_checks() & Check::Baro.as_u32(), 0);
}
