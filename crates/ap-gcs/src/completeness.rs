//! FW-028 GCS_MAVLink completeness: surfaces already on main vs remaining.
//!
//! Catalogs the SITL-first `libraries/GCS_MAVLink` / `GCS_MAVLink_Plane`
//! port. Items marked [`PortStatus::OnMain`] landed in earlier slices
//! and must not be redone. [`PortStatus::ThisSlice`] is this table.
//! [`PortStatus::Remaining`] are documented-deferred (full XML dialect
//! generation, COMMAND_ACK / PARAM_REQUEST_READ / MISSION_COUNT,
//! `GCS_MAVLINK_Plane` vehicle handlers, sitl-diff replay) outside
//! this ticket's stub surface.
//!
//! Message ids in [`PINNED_MSGIDS`] come from the pinned
//! `modules/mavlink` common.xml definitions used by the on-main
//! stubs. This module does **not** generate the entire dialect.
//!
//! This module does not rewrite [`crate::rc_override`], [`crate::rates`],
//! or [`crate::hud`]. GCS failsafe (`FS_GCS_ENABL`) already lives in
//! `ap-plane` and is not rewritten here.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-028 closing slice (this table).
    ThisSlice,
    /// Documented-deferred: not blocking the FW-028 SITL stub close.
    Remaining,
}

/// One GCS_MAVLink surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GcsPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// One msgid taken from the pinned `modules/mavlink` common.xml.
///
/// Only the messages already stubbed on main are listed. The rest of
/// the common / ardupilotmega dialect stays ungenerated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinnedMsgid {
    /// MAVLink message name from the pinned XML.
    pub name: &'static str,
    /// `id` attribute from the pinned XML.
    pub msgid: u32,
}

/// Completeness table: ported GCS stubs vs documented-deferred gaps.
///
/// Row names match the closer catalog: framing, heartbeat, STATUSTEXT,
/// commands, params, pose, mission, health, channels, HUD, rates,
/// override.
pub const GCS_COMPLETENESS: &[GcsPortItem] = &[
    GcsPortItem {
        name: "framing",
        status: PortStatus::OnMain,
        note: "MAVLink 2 STX 0xFD encode/decode + CRC-16/MCRF4XX (framing.rs)",
    },
    GcsPortItem {
        name: "heartbeat",
        status: PortStatus::OnMain,
        note: "HEARTBEAT msgid 0 encode/decode / send_heartbeat / handle_heartbeat",
    },
    GcsPortItem {
        name: "STATUSTEXT",
        status: PortStatus::OnMain,
        note: "STATUSTEXT msgid 253 send_text severity + 50-byte text",
    },
    GcsPortItem {
        name: "commands",
        status: PortStatus::OnMain,
        note: "COMMAND_LONG/INT msgid 76/75 ARM/DISARM DO_SET_MODE NAV_TAKEOFF",
    },
    GcsPortItem {
        name: "params",
        status: PortStatus::OnMain,
        note: "PARAM_REQUEST_LIST / PARAM_SET msgid 21/23 in-memory table",
    },
    GcsPortItem {
        name: "pose",
        status: PortStatus::OnMain,
        note: "ATTITUDE / GLOBAL_POSITION_INT msgid 30/33 stream from PoseSnapshot",
    },
    GcsPortItem {
        name: "mission",
        status: PortStatus::OnMain,
        note: "MISSION_ITEM_INT / MISSION_REQUEST_INT msgid 73/51 one-waypoint table",
    },
    GcsPortItem {
        name: "health",
        status: PortStatus::OnMain,
        note: "SYS_STATUS / BATTERY_STATUS msgid 1/147 stream from HealthSnapshot",
    },
    GcsPortItem {
        name: "channels",
        status: PortStatus::OnMain,
        note: "RC_CHANNELS / SERVO_OUTPUT_RAW msgid 65/36 stream from ChannelSnapshot",
    },
    GcsPortItem {
        name: "HUD",
        status: PortStatus::OnMain,
        note: "VFR_HUD / NAV_CONTROLLER_OUTPUT msgid 74/62 stream from HudSnapshot",
    },
    GcsPortItem {
        name: "rates",
        status: PortStatus::OnMain,
        note: "REQUEST_DATA_STREAM / SET_MESSAGE_INTERVAL msgid 66 / cmd 511 RateTable",
    },
    GcsPortItem {
        name: "override",
        status: PortStatus::OnMain,
        note: "MANUAL_CONTROL / RC_CHANNELS_OVERRIDE msgid 69/70 OverrideStore",
    },
    GcsPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog + pinned-msgid subset (not the full dialect)",
    },
    GcsPortItem {
        name: "full XML dialect",
        status: PortStatus::Remaining,
        note: "ticket: generate from modules/mavlink; do not generate the entire dialect here",
    },
    GcsPortItem {
        name: "COMMAND_ACK / PARAM_REQUEST_READ / MISSION_COUNT",
        status: PortStatus::Remaining,
        note: "COMMAND_ACK, PARAM_REQUEST_READ, MISSION_COUNT / MISSION_ACK",
    },
    GcsPortItem {
        name: "GCS_MAVLINK_Plane handlers",
        status: PortStatus::Remaining,
        note: "vehicle-side GCS_MAVLINK_Plane + signing / routing / STATUSTEXT recv",
    },
    GcsPortItem {
        name: "sitl-diff replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs; not this SITL stub close",
    },
];

/// Message ids from the pinned `modules/mavlink` common.xml used by
/// the on-main stubs. Not a generated dialect.
pub const PINNED_MSGIDS: &[PinnedMsgid] = &[
    PinnedMsgid {
        name: "HEARTBEAT",
        msgid: 0,
    },
    PinnedMsgid {
        name: "SYS_STATUS",
        msgid: 1,
    },
    PinnedMsgid {
        name: "PARAM_REQUEST_LIST",
        msgid: 21,
    },
    PinnedMsgid {
        name: "PARAM_VALUE",
        msgid: 22,
    },
    PinnedMsgid {
        name: "PARAM_SET",
        msgid: 23,
    },
    PinnedMsgid {
        name: "ATTITUDE",
        msgid: 30,
    },
    PinnedMsgid {
        name: "GLOBAL_POSITION_INT",
        msgid: 33,
    },
    PinnedMsgid {
        name: "SERVO_OUTPUT_RAW",
        msgid: 36,
    },
    PinnedMsgid {
        name: "MISSION_REQUEST_INT",
        msgid: 51,
    },
    PinnedMsgid {
        name: "NAV_CONTROLLER_OUTPUT",
        msgid: 62,
    },
    PinnedMsgid {
        name: "RC_CHANNELS",
        msgid: 65,
    },
    PinnedMsgid {
        name: "REQUEST_DATA_STREAM",
        msgid: 66,
    },
    PinnedMsgid {
        name: "MANUAL_CONTROL",
        msgid: 69,
    },
    PinnedMsgid {
        name: "RC_CHANNELS_OVERRIDE",
        msgid: 70,
    },
    PinnedMsgid {
        name: "MISSION_ITEM_INT",
        msgid: 73,
    },
    PinnedMsgid {
        name: "VFR_HUD",
        msgid: 74,
    },
    PinnedMsgid {
        name: "COMMAND_INT",
        msgid: 75,
    },
    PinnedMsgid {
        name: "COMMAND_LONG",
        msgid: 76,
    },
    PinnedMsgid {
        name: "BATTERY_STATUS",
        msgid: 147,
    },
    PinnedMsgid {
        name: "STATUSTEXT",
        msgid: 253,
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static GcsPortItem> {
    GCS_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static GcsPortItem> {
    GCS_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left documented-deferred (not blocking FW-028 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static GcsPortItem> {
    GCS_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in GCS_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    GCS_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in GCS_COMPLETENESS.iter().enumerate() {
        for other in GCS_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

/// Look up a pinned msgid by XML message name.
#[must_use]
pub fn pinned_msgid(name: &str) -> Option<u32> {
    PINNED_MSGIDS
        .iter()
        .find(|item| item.name == name)
        .map(|item| item.msgid)
}

/// True when every pinned msgid name appears once.
#[must_use]
pub fn pinned_msgids_unique() -> bool {
    for (i, item) in PINNED_MSGIDS.iter().enumerate() {
        for other in PINNED_MSGIDS.iter().skip(i + 1) {
            if item.name == other.name || item.msgid == other.msgid {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_surfaces_and_this_slice() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 12);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 4);
        assert!(completeness_has("framing", PortStatus::OnMain));
        assert!(completeness_has("heartbeat", PortStatus::OnMain));
        assert!(completeness_has("STATUSTEXT", PortStatus::OnMain));
        assert!(completeness_has("commands", PortStatus::OnMain));
        assert!(completeness_has("params", PortStatus::OnMain));
        assert!(completeness_has("pose", PortStatus::OnMain));
        assert!(completeness_has("mission", PortStatus::OnMain));
        assert!(completeness_has("health", PortStatus::OnMain));
        assert!(completeness_has("channels", PortStatus::OnMain));
        assert!(completeness_has("HUD", PortStatus::OnMain));
        assert!(completeness_has("rates", PortStatus::OnMain));
        assert!(completeness_has("override", PortStatus::OnMain));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("full XML dialect", PortStatus::Remaining));
        assert!(completeness_has(
            "COMMAND_ACK / PARAM_REQUEST_READ / MISSION_COUNT",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "GCS_MAVLINK_Plane handlers",
            PortStatus::Remaining
        ));
        assert!(completeness_has("sitl-diff replay", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 12);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 4);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }

    #[test]
    fn pinned_msgids_match_common_xml_subset() {
        assert!(pinned_msgids_unique());
        assert_eq!(PINNED_MSGIDS.len(), 20);
        assert_eq!(pinned_msgid("HEARTBEAT"), Some(0));
        assert_eq!(pinned_msgid("STATUSTEXT"), Some(253));
        assert_eq!(pinned_msgid("COMMAND_LONG"), Some(76));
        assert_eq!(pinned_msgid("COMMAND_INT"), Some(75));
        assert_eq!(pinned_msgid("PARAM_REQUEST_LIST"), Some(21));
        assert_eq!(pinned_msgid("PARAM_SET"), Some(23));
        assert_eq!(pinned_msgid("ATTITUDE"), Some(30));
        assert_eq!(pinned_msgid("GLOBAL_POSITION_INT"), Some(33));
        assert_eq!(pinned_msgid("MISSION_ITEM_INT"), Some(73));
        assert_eq!(pinned_msgid("MISSION_REQUEST_INT"), Some(51));
        assert_eq!(pinned_msgid("SYS_STATUS"), Some(1));
        assert_eq!(pinned_msgid("BATTERY_STATUS"), Some(147));
        assert_eq!(pinned_msgid("RC_CHANNELS"), Some(65));
        assert_eq!(pinned_msgid("SERVO_OUTPUT_RAW"), Some(36));
        assert_eq!(pinned_msgid("VFR_HUD"), Some(74));
        assert_eq!(pinned_msgid("NAV_CONTROLLER_OUTPUT"), Some(62));
        assert_eq!(pinned_msgid("REQUEST_DATA_STREAM"), Some(66));
        assert_eq!(pinned_msgid("MANUAL_CONTROL"), Some(69));
        assert_eq!(pinned_msgid("RC_CHANNELS_OVERRIDE"), Some(70));
        assert_eq!(pinned_msgid("PARAM_VALUE"), Some(22));
        assert_eq!(pinned_msgid("common.xml entire dialect"), None);
    }
}
