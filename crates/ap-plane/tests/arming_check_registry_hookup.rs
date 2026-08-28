//! Plane hookup for the shared AP_Arming check registry.

use ap_arming::{Arming, Check, NamedCheck, Required};
use ap_plane::arming_check_registry_hookup::plane_pre_arm_checks_registry;
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

fn baro(ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Baro,
        name: "BARO",
        ok,
    }
}

#[test]
fn plane_registry_refuses_on_a_named_check() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_registry(mode, Arming::new(), &[baro(false)]),
        PreArmResult::Refused("BARO")
    );
}

#[test]
fn plane_registry_preserves_a_mode_refusal() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks_registry(mode, Arming::new(), &[baro(true)]),
        PreArmResult::Refused("not armable here")
    );
}

#[test]
fn plane_registry_allows_when_require_is_no() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        require: Required::No,
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_registry(mode, arming, &[baro(false)]),
        PreArmResult::Allowed
    );
}

#[test]
fn plane_registry_allows_when_the_named_check_is_skipped() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Baro.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_registry(mode, arming, &[baro(false)]),
        PreArmResult::Allowed
    );
}
