//! Geofence breach failsafe action, Plane `FENCE_ACTION` / `AC_Fence::Action`.
//!
//! Report / RTL / Guided / GuidedThrottlePass / Terminate when a fence
//! is newly breached. Disabled, disarmed, landing-impact, and already-
//! recovering vehicles take no action.

use ap_plane::fence_failsafe_hookup::{
    fence_failsafe_action, FenceAction, FenceFailsafeInputs, FenceFailsafeResult,
};

fn breached(action: FenceAction) -> FenceFailsafeInputs {
    FenceFailsafeInputs {
        action,
        enabled: true,
        new_breach: true,
        armed: true,
        in_recovery: false,
        landing_impact: false,
    }
}

#[test]
fn fence_action_values_match_upstream_plane() {
    assert_eq!(FenceAction::from_param(0), Some(FenceAction::ReportOnly));
    assert_eq!(FenceAction::from_param(1), Some(FenceAction::Rtl));
    assert_eq!(FenceAction::from_param(6), Some(FenceAction::Guided));
    assert_eq!(
        FenceAction::from_param(7),
        Some(FenceAction::GuidedThrottlePass)
    );
    for value in [2_u8, 3, 4, 5, 8, 9] {
        assert_eq!(
            FenceAction::from_param(value),
            None,
            "{value} is not a Plane FENCE_ACTION token in this stub"
        );
    }
    assert_eq!(FenceAction::default_param(), FenceAction::Rtl);
    assert!(!FenceAction::ReportOnly.changes_vehicle());
    assert!(FenceAction::Guided.changes_vehicle());
    assert!(FenceAction::GuidedThrottlePass.changes_vehicle());
    assert!(FenceAction::Terminate.changes_vehicle());
}

#[test]
fn disabled_or_disarmed_never_acts_on_breach() {
    let disabled = FenceFailsafeInputs {
        enabled: false,
        ..breached(FenceAction::Rtl)
    };
    assert_eq!(fence_failsafe_action(&disabled), FenceFailsafeResult::None);

    let disarmed = FenceFailsafeInputs {
        armed: false,
        ..breached(FenceAction::Rtl)
    };
    assert_eq!(fence_failsafe_action(&disarmed), FenceFailsafeResult::None);
}

#[test]
fn landing_impact_and_recovery_suppress_the_action() {
    let landing = FenceFailsafeInputs {
        landing_impact: true,
        ..breached(FenceAction::Rtl)
    };
    assert_eq!(fence_failsafe_action(&landing), FenceFailsafeResult::None);

    let recovering = FenceFailsafeInputs {
        in_recovery: true,
        ..breached(FenceAction::Guided)
    };
    assert_eq!(
        fence_failsafe_action(&recovering),
        FenceFailsafeResult::None
    );
}

#[test]
fn no_new_breach_holds() {
    let quiet = FenceFailsafeInputs {
        new_breach: false,
        ..breached(FenceAction::Rtl)
    };
    assert_eq!(fence_failsafe_action(&quiet), FenceFailsafeResult::None);
    assert_eq!(
        fence_failsafe_action(&FenceFailsafeInputs::default()),
        FenceFailsafeResult::None
    );
}

#[test]
fn report_rtl_guided_throttle_pass_and_terminate_on_breach() {
    assert_eq!(
        fence_failsafe_action(&breached(FenceAction::ReportOnly)),
        FenceFailsafeResult::Report
    );
    assert_eq!(
        fence_failsafe_action(&breached(FenceAction::Rtl)),
        FenceFailsafeResult::Rtl
    );
    assert_eq!(
        fence_failsafe_action(&breached(FenceAction::Guided)),
        FenceFailsafeResult::Guided
    );
    assert_eq!(
        fence_failsafe_action(&breached(FenceAction::GuidedThrottlePass)),
        FenceFailsafeResult::GuidedThrottlePass
    );
    assert_eq!(
        fence_failsafe_action(&breached(FenceAction::Terminate)),
        FenceFailsafeResult::Terminate
    );
}
