//! ARMING_RUDDER / rudder-stick arm-disarm gate.

use ap_arming::rudder_arming::{
    rudder_stick_allowed, RudderArming, RudderStickAction, ARMING_RUDDER_DEFAULT,
};

#[test]
fn plane_default_rudder_is_arm_only() {
    assert_eq!(ARMING_RUDDER_DEFAULT, RudderArming::ArmOnly);
    assert_eq!(RudderArming::from_u8(1), Some(RudderArming::ArmOnly));
    assert_eq!(ARMING_RUDDER_DEFAULT.as_u8(), 1);
    assert!(ARMING_RUDDER_DEFAULT.allows_rudder_arm());
    assert!(!ARMING_RUDDER_DEFAULT.allows_rudder_disarm());
}

#[test]
fn rudder_arming_decodes_the_three_stored_values() {
    assert_eq!(RudderArming::from_u8(0), Some(RudderArming::Disabled));
    assert_eq!(RudderArming::from_u8(1), Some(RudderArming::ArmOnly));
    assert_eq!(RudderArming::from_u8(2), Some(RudderArming::ArmOrDisarm));
    assert_eq!(RudderArming::from_u8(3), None);
    assert_eq!(RudderArming::Disabled.as_u8(), 0);
    assert_eq!(RudderArming::ArmOrDisarm.as_u8(), 2);
}

#[test]
fn disabled_refuses_rudder_stick_arm_and_disarm() {
    let rudder = RudderArming::Disabled;
    assert!(!rudder.allows_rudder_arm());
    assert!(!rudder.allows_rudder_disarm());
    assert!(!rudder_stick_allowed(rudder, RudderStickAction::Arm));
    assert!(!rudder_stick_allowed(rudder, RudderStickAction::Disarm));
}

#[test]
fn arm_only_allows_rudder_arm_but_not_disarm() {
    let rudder = RudderArming::ArmOnly;
    assert!(rudder_stick_allowed(rudder, RudderStickAction::Arm));
    assert!(!rudder_stick_allowed(rudder, RudderStickAction::Disarm));
}

#[test]
fn arm_or_disarm_allows_rudder_arm_and_disarm() {
    let rudder = RudderArming::ArmOrDisarm;
    assert!(rudder.allows_rudder_arm());
    assert!(rudder.allows_rudder_disarm());
    assert!(rudder_stick_allowed(rudder, RudderStickAction::Arm));
    assert!(rudder_stick_allowed(rudder, RudderStickAction::Disarm));
}
