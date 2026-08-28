//! Plane hookup for the pre-4.7 ARMING_CHECK enable bitmask.

use ap_arming::check_bitmask::ARMING_CHECK_ALL;
use ap_arming::{Check, NamedCheck, Required};
use ap_plane::arming_check_bitmask_hookup::{
    plane_arming_from_check, plane_pre_arm_checks_arming_check,
};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

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
fn all_refuses_a_failing_named_check() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_arming_check(
            mode,
            Required::YesMinPwm,
            ARMING_CHECK_ALL,
            &[baro(false)]
        ),
        PreArmResult::Refused("BARO")
    );
}

#[test]
fn zero_disables_named_checks() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_arming_check(mode, Required::YesMinPwm, 0, &[baro(false)]),
        PreArmResult::Allowed
    );
    let arming = plane_arming_from_check(Required::YesMinPwm, 0);
    assert!(arming.should_skip_all_checks());
}

#[test]
fn a_named_bit_enables_only_that_check() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_arming_check(
            mode,
            Required::YesMinPwm,
            Check::Baro.as_u32(),
            &[baro(true), compass(false)]
        ),
        PreArmResult::Allowed
    );
    assert_eq!(
        plane_pre_arm_checks_arming_check(
            mode,
            Required::YesMinPwm,
            Check::Baro.as_u32(),
            &[baro(false), compass(false)]
        ),
        PreArmResult::Refused("BARO")
    );
}

#[test]
fn mode_refusal_is_kept() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks_arming_check(
            mode,
            Required::YesMinPwm,
            ARMING_CHECK_ALL,
            &[baro(true)]
        ),
        PreArmResult::Refused("not armable here")
    );
}
