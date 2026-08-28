//! Vehicle hookup for `AP_Mission` command/item storage.
//!
//! Plane owns the in-memory mission list the GCS writes. AUTO already advances
//! a waypoint index on the scheduler tick; this is the item record that index
//! will later read — `seq`, `command`, `frame`, lat/lon/alt.

use ap_mission::{MavFrame, Mission, MissionCommand, MAV_CMD_NAV_WAYPOINT};

/// Vehicle-side mission item store.
#[derive(Debug, Clone, Default)]
pub struct MissionItemStorageHookup {
    mission: Mission,
}

/// One waypoint item published for the vehicle loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionItemPublish {
    /// Mission-protocol `seq`, upstream `Mission_Command::index`.
    pub seq: u16,
    /// MAV_CMD id, upstream `Mission_Command::id`.
    pub command: u16,
    /// MAV_FRAME stored on the item.
    pub frame: MavFrame,
    /// Latitude, 1e-7 degrees.
    pub lat: i32,
    /// Longitude, 1e-7 degrees.
    pub lng: i32,
    /// Altitude in centimetres, in [`MissionItemPublish::frame`].
    pub alt_cm: i32,
}

impl MissionItemStorageHookup {
    /// Empty store; home has not been written.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mission: Mission::new(),
        }
    }

    /// The underlying mission list.
    #[must_use]
    pub const fn mission(&self) -> &Mission {
        &self.mission
    }

    /// Command count including home at seq 0.
    #[must_use]
    pub const fn num_commands(&self) -> u16 {
        self.mission.num_commands()
    }

    /// Store a `MAV_CMD_NAV_WAYPOINT` at `seq`.
    pub fn store_nav_waypoint(
        &mut self,
        seq: u16,
        frame: MavFrame,
        lat: i32,
        lng: i32,
        alt_cm: i32,
    ) -> bool {
        self.mission
            .write_cmd(MissionCommand::waypoint(seq, frame, lat, lng, alt_cm))
    }

    /// Publish the stored item at `seq`, or `None` if it is empty.
    #[must_use]
    pub fn publish(&self, seq: u16) -> Option<MissionItemPublish> {
        let cmd = self.mission.read_cmd(seq)?;
        Some(MissionItemPublish {
            seq: cmd.seq,
            command: cmd.command,
            frame: cmd.frame,
            lat: cmd.location.lat,
            lng: cmd.location.lng,
            alt_cm: cmd.location.alt,
        })
    }
}

/// Confirm a published item is the NAV_WAYPOINT the vehicle stored.
#[must_use]
pub fn published_nav_waypoint(item: &MissionItemPublish) -> bool {
    item.command == MAV_CMD_NAV_WAYPOINT
}
