//! Plane hookup for AP_Mission command/item storage.

use ap_mission::{MavFrame, MAV_CMD_NAV_WAYPOINT};
use ap_plane::mission_item_storage_hookup::{published_nav_waypoint, MissionItemStorageHookup};

#[test]
fn plane_stores_and_publishes_a_nav_waypoint_item() {
    let mut hookup = MissionItemStorageHookup::new();
    assert_eq!(hookup.num_commands(), 0);

    assert!(hookup.store_nav_waypoint(0, MavFrame::Global, -35_363_261, 149_165_237, 58_400));
    assert!(hookup.store_nav_waypoint(
        1,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        10_000,
    ));
    assert_eq!(hookup.num_commands(), 2);

    let wp = hookup.publish(1).expect("seq 1");
    assert_eq!(wp.seq, 1);
    assert_eq!(wp.command, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(wp.frame, MavFrame::GlobalRelativeAlt);
    assert_eq!(wp.lat, -35_362_000);
    assert_eq!(wp.lng, 149_166_000);
    assert_eq!(wp.alt_cm, 10_000);
    assert!(published_nav_waypoint(&wp));
}

#[test]
fn plane_publish_misses_an_unwritten_seq() {
    let hookup = MissionItemStorageHookup::new();
    assert!(hookup.publish(0).is_none());
    assert!(hookup.mission().read_cmd(0).is_none());
}
