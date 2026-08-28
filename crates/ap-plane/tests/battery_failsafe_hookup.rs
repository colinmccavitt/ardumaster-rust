//! Battery failsafe action, historic `FS_BATT_ENABLE` / `FS_BATT_VOLTAGE` / `FS_BATT_MAH`.
//!
//! Disabled / Land / RTL / Terminate when voltage or remaining capacity
//! crosses the configured threshold.

use ap_plane::battery_failsafe_hookup::{
    battery_failsafe_action, BatteryFailsafeEnable, BatteryFailsafeInputs, BatteryFailsafeResult,
    FS_BATT_MAH_DEFAULT, FS_BATT_VOLTAGE_DEFAULT,
};

fn voltage_at(enable: BatteryFailsafeEnable, voltage_v: f32) -> BatteryFailsafeInputs {
    BatteryFailsafeInputs {
        enable,
        voltage_v,
        remaining_mah: None,
        fs_batt_voltage: FS_BATT_VOLTAGE_DEFAULT,
        fs_batt_mah: FS_BATT_MAH_DEFAULT,
    }
}

#[test]
fn fs_batt_enable_values_match_upstream() {
    assert_eq!(
        BatteryFailsafeEnable::from_param(0),
        Some(BatteryFailsafeEnable::Disabled)
    );
    assert_eq!(
        BatteryFailsafeEnable::from_param(1),
        Some(BatteryFailsafeEnable::Land)
    );
    assert_eq!(
        BatteryFailsafeEnable::from_param(2),
        Some(BatteryFailsafeEnable::Rtl)
    );
    assert_eq!(
        BatteryFailsafeEnable::from_param(3),
        Some(BatteryFailsafeEnable::Terminate)
    );
    assert_eq!(BatteryFailsafeEnable::from_param(4), None);
    assert_eq!(
        BatteryFailsafeEnable::default_param(),
        BatteryFailsafeEnable::Disabled
    );
    assert!(!BatteryFailsafeEnable::Disabled.is_enabled());
    assert!(BatteryFailsafeEnable::Rtl.is_enabled());
    assert!(BatteryFailsafeEnable::Terminate.is_enabled());
}

#[test]
fn disabled_never_trips_on_low_voltage_or_mah() {
    let low_v = voltage_at(BatteryFailsafeEnable::Disabled, 9.0);
    assert_eq!(battery_failsafe_action(&low_v), BatteryFailsafeResult::None);

    let low_mah = BatteryFailsafeInputs {
        enable: BatteryFailsafeEnable::Disabled,
        voltage_v: 12.6,
        remaining_mah: Some(50.0),
        fs_batt_voltage: 0.0,
        fs_batt_mah: 200.0,
    };
    assert_eq!(
        battery_failsafe_action(&low_mah),
        BatteryFailsafeResult::None
    );
}

#[test]
fn voltage_below_fs_batt_voltage_selects_action() {
    let healthy = voltage_at(BatteryFailsafeEnable::Land, 10.5);
    assert_eq!(
        battery_failsafe_action(&healthy),
        BatteryFailsafeResult::None
    );
    let just_under = voltage_at(BatteryFailsafeEnable::Land, 10.499);
    assert_eq!(
        battery_failsafe_action(&just_under),
        BatteryFailsafeResult::Land
    );
    assert_eq!(
        battery_failsafe_action(&voltage_at(BatteryFailsafeEnable::Rtl, 9.0)),
        BatteryFailsafeResult::Rtl
    );
    assert_eq!(
        battery_failsafe_action(&voltage_at(BatteryFailsafeEnable::Terminate, 9.0)),
        BatteryFailsafeResult::Terminate
    );
}

#[test]
fn zero_voltage_threshold_disables_voltage_check() {
    let inp = BatteryFailsafeInputs {
        enable: BatteryFailsafeEnable::Rtl,
        voltage_v: 1.0,
        remaining_mah: None,
        fs_batt_voltage: 0.0,
        fs_batt_mah: 0.0,
    };
    assert_eq!(battery_failsafe_action(&inp), BatteryFailsafeResult::None);
}

#[test]
fn unset_voltage_reading_does_not_trip() {
    let inp = voltage_at(BatteryFailsafeEnable::Land, 0.0);
    assert_eq!(battery_failsafe_action(&inp), BatteryFailsafeResult::None);
}

#[test]
fn remaining_mah_below_fs_batt_mah_trips() {
    let healthy = BatteryFailsafeInputs {
        enable: BatteryFailsafeEnable::Rtl,
        voltage_v: 12.6,
        remaining_mah: Some(200.0),
        fs_batt_voltage: 0.0,
        fs_batt_mah: 200.0,
    };
    assert_eq!(
        battery_failsafe_action(&healthy),
        BatteryFailsafeResult::None
    );

    let low = BatteryFailsafeInputs {
        remaining_mah: Some(199.0),
        ..healthy
    };
    assert_eq!(battery_failsafe_action(&low), BatteryFailsafeResult::Rtl);

    let no_current = BatteryFailsafeInputs {
        remaining_mah: None,
        ..healthy
    };
    assert_eq!(
        battery_failsafe_action(&no_current),
        BatteryFailsafeResult::None
    );
}

#[test]
fn zero_mah_threshold_disables_capacity_check() {
    let inp = BatteryFailsafeInputs {
        enable: BatteryFailsafeEnable::Land,
        voltage_v: 12.6,
        remaining_mah: Some(0.0),
        fs_batt_voltage: 0.0,
        fs_batt_mah: FS_BATT_MAH_DEFAULT,
    };
    assert_eq!(battery_failsafe_action(&inp), BatteryFailsafeResult::None);
}
