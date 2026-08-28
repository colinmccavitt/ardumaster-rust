//! `in_vtol_mode` / `in_vtol_auto` — upstream `QuadPlane::in_vtol_mode`
//! / `QuadPlane::in_vtol_auto`.

use ap_quadplane::vtol_mode::{
    ControlKind, VtolModeView, MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_PAYLOAD_PLACE, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_VTOL_LAND,
    MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_WAYPOINT,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn unavailable_is_never_in_vtol_mode_or_auto() {
    // Upstream both methods start with `if (!available()) return false`.
    let qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    assert!(!qp.in_vtol_mode(&VtolModeView::q_mode()));
    assert!(!qp.in_vtol_auto(&VtolModeView::auto(MAV_CMD_NAV_VTOL_TAKEOFF)));
}

#[test]
fn q_mode_is_in_vtol_mode_not_auto() {
    let qp = available_qp();
    let view = VtolModeView::q_mode();
    assert_eq!(view.control, ControlKind::Vtol);
    assert!(qp.in_vtol_mode(&view));
    assert!(!qp.in_vtol_auto(&view));
}

#[test]
fn fixed_wing_mode_is_neither() {
    let qp = available_qp();
    let view = VtolModeView::new();
    assert_eq!(view.control, ControlKind::Other);
    assert!(!qp.in_vtol_mode(&view));
    assert!(!qp.in_vtol_auto(&view));
}

#[test]
fn auto_vtol_takeoff_is_in_vtol_auto_and_mode() {
    let qp = available_qp();
    let view = VtolModeView::auto(MAV_CMD_NAV_VTOL_TAKEOFF);
    assert!(qp.in_vtol_auto(&view));
    assert!(qp.in_vtol_mode(&view));
}

#[test]
fn auto_waypoint_is_neither() {
    let qp = available_qp();
    let view = VtolModeView::auto(MAV_CMD_NAV_WAYPOINT);
    assert!(!qp.in_vtol_auto(&view));
    assert!(!qp.in_vtol_mode(&view));
}

#[test]
fn auto_state_vtol_mode_flag_is_enough() {
    let qp = available_qp();
    let mut view = VtolModeView::auto(MAV_CMD_NAV_WAYPOINT);
    view.auto_vtol_mode = true;
    assert!(qp.in_vtol_auto(&view));
    assert!(qp.in_vtol_mode(&view));
}

#[test]
fn auto_loiter_follows_vtol_loiter_flag() {
    let qp = available_qp();
    let mut view = VtolModeView::auto(MAV_CMD_NAV_LOITER_UNLIM);
    assert!(!qp.in_vtol_auto(&view));
    view.auto_vtol_loiter = true;
    assert!(qp.in_vtol_auto(&view));
    // Still in approach/airbrake without poscontrol: not in_vtol_mode.
    assert!(!qp.in_vtol_mode(&view));
}

#[test]
fn auto_nav_takeoff_counts_as_vtol_when_available() {
    let qp = available_qp();
    let view = VtolModeView::auto(MAV_CMD_NAV_TAKEOFF);
    assert!(qp.is_vtol_takeoff(MAV_CMD_NAV_TAKEOFF));
    assert!(qp.in_vtol_auto(&view));
    assert!(qp.in_vtol_mode(&view));
}

#[test]
fn auto_vtol_land_and_payload_place_are_vtol() {
    let qp = available_qp();
    for id in [
        MAV_CMD_NAV_VTOL_LAND,
        MAV_CMD_NAV_LAND,
        MAV_CMD_NAV_PAYLOAD_PLACE,
    ] {
        let view = VtolModeView::auto(id);
        assert!(qp.is_vtol_land(id), "id {id}");
        assert!(qp.in_vtol_auto(&view), "id {id}");
        assert!(qp.in_vtol_mode(&view), "id {id}");
    }
}

#[test]
fn guided_takeoff_is_in_vtol_mode_not_auto() {
    let qp = available_qp();
    let view = VtolModeView::guided(true);
    assert!(qp.in_vtol_mode(&view));
    assert!(!qp.in_vtol_auto(&view));
    let fw = VtolModeView::guided(false);
    assert!(!qp.in_vtol_mode(&fw));
}

#[test]
fn mav_cmd_ids_match_mavlink() {
    assert_eq!(MAV_CMD_NAV_WAYPOINT, 16);
    assert_eq!(MAV_CMD_NAV_LOITER_UNLIM, 17);
    assert_eq!(MAV_CMD_NAV_TAKEOFF, 22);
    assert_eq!(MAV_CMD_NAV_LAND, 21);
    assert_eq!(MAV_CMD_NAV_VTOL_TAKEOFF, 84);
    assert_eq!(MAV_CMD_NAV_VTOL_LAND, 85);
    assert_eq!(MAV_CMD_NAV_PAYLOAD_PLACE, 94);
}
