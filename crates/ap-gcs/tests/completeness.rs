//! FW-028 completeness: GCS_MAVLink surfaces already on main vs remaining.

use ap_gcs::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items, pinned_msgid,
    pinned_msgids_unique, remaining_items, this_slice_items, PortStatus, GCS_COMPLETENESS,
    PINNED_MSGIDS,
};
use ap_gcs::{
    MSG_ID_ATTITUDE, MSG_ID_BATTERY_STATUS, MSG_ID_COMMAND_INT, MSG_ID_COMMAND_LONG,
    MSG_ID_GLOBAL_POSITION_INT, MSG_ID_HEARTBEAT, MSG_ID_MANUAL_CONTROL, MSG_ID_MISSION_ITEM_INT,
    MSG_ID_MISSION_REQUEST_INT, MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_ID_PARAM_REQUEST_LIST,
    MSG_ID_PARAM_SET, MSG_ID_PARAM_VALUE, MSG_ID_RC_CHANNELS, MSG_ID_RC_CHANNELS_OVERRIDE,
    MSG_ID_REQUEST_DATA_STREAM, MSG_ID_SERVO_OUTPUT_RAW, MSG_ID_STATUSTEXT, MSG_ID_SYS_STATUS,
    MSG_ID_VFR_HUD,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "framing",
    "heartbeat",
    "STATUSTEXT",
    "commands",
    "params",
    "pose",
    "mission",
    "health",
    "channels",
    "HUD",
    "rates",
    "override",
];

const THIS_SLICE: &[&str] = &["completeness table"];

const REMAINING: &[&str] = &[
    "full XML dialect",
    "COMMAND_ACK / PARAM_REQUEST_READ / MISSION_COUNT",
    "GCS_MAVLINK_Plane handlers",
    "sitl-diff replay",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        GCS_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the closing-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is remaining / out of scope for the SITL stub close"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}

#[test]
fn pinned_msgids_match_crate_constants_not_full_dialect() {
    assert!(pinned_msgids_unique());
    assert_eq!(PINNED_MSGIDS.len(), 20);
    assert_eq!(pinned_msgid("HEARTBEAT"), Some(MSG_ID_HEARTBEAT));
    assert_eq!(pinned_msgid("STATUSTEXT"), Some(MSG_ID_STATUSTEXT));
    assert_eq!(pinned_msgid("COMMAND_LONG"), Some(MSG_ID_COMMAND_LONG));
    assert_eq!(pinned_msgid("COMMAND_INT"), Some(MSG_ID_COMMAND_INT));
    assert_eq!(
        pinned_msgid("PARAM_REQUEST_LIST"),
        Some(MSG_ID_PARAM_REQUEST_LIST)
    );
    assert_eq!(pinned_msgid("PARAM_SET"), Some(MSG_ID_PARAM_SET));
    assert_eq!(pinned_msgid("PARAM_VALUE"), Some(MSG_ID_PARAM_VALUE));
    assert_eq!(pinned_msgid("ATTITUDE"), Some(MSG_ID_ATTITUDE));
    assert_eq!(
        pinned_msgid("GLOBAL_POSITION_INT"),
        Some(MSG_ID_GLOBAL_POSITION_INT)
    );
    assert_eq!(
        pinned_msgid("MISSION_ITEM_INT"),
        Some(MSG_ID_MISSION_ITEM_INT)
    );
    assert_eq!(
        pinned_msgid("MISSION_REQUEST_INT"),
        Some(MSG_ID_MISSION_REQUEST_INT)
    );
    assert_eq!(pinned_msgid("SYS_STATUS"), Some(MSG_ID_SYS_STATUS));
    assert_eq!(pinned_msgid("BATTERY_STATUS"), Some(MSG_ID_BATTERY_STATUS));
    assert_eq!(pinned_msgid("RC_CHANNELS"), Some(MSG_ID_RC_CHANNELS));
    assert_eq!(
        pinned_msgid("SERVO_OUTPUT_RAW"),
        Some(MSG_ID_SERVO_OUTPUT_RAW)
    );
    assert_eq!(pinned_msgid("VFR_HUD"), Some(MSG_ID_VFR_HUD));
    assert_eq!(
        pinned_msgid("NAV_CONTROLLER_OUTPUT"),
        Some(MSG_ID_NAV_CONTROLLER_OUTPUT)
    );
    assert_eq!(
        pinned_msgid("REQUEST_DATA_STREAM"),
        Some(MSG_ID_REQUEST_DATA_STREAM)
    );
    assert_eq!(pinned_msgid("MANUAL_CONTROL"), Some(MSG_ID_MANUAL_CONTROL));
    assert_eq!(
        pinned_msgid("RC_CHANNELS_OVERRIDE"),
        Some(MSG_ID_RC_CHANNELS_OVERRIDE)
    );
    assert!(
        completeness_has("full XML dialect", PortStatus::Remaining),
        "entire dialect must stay ungenerated"
    );
}
