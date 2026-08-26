//! ArduCopter's vehicle-level code, upstream `ArduCopter/`.
//!
//! The libraries below this — the attitude controller, the position
//! controller, the motors mixer — are ported in their own crates. What lives
//! here is the layer that decides *which* of them to call and with what: the
//! flight modes and the pilot-input conversions they share.
//!
//! # Why the pilot conversions are here and not in a library
//!
//! Upstream puts them on `Mode`, the flight-mode base class, and that is not
//! an accident of layout. Each one encodes a decision about how a *pilot*
//! should feel the aircraft — where the throttle curve bends, how yaw
//! responds near centre — rather than anything the controllers need. A library
//! that took them would be taking a position on ergonomics.

#![no_std]

pub mod alt_hold;
pub mod altitude;
pub mod auto_yaw;
pub mod ground;
pub mod land;
pub mod mode_entry;
pub mod pilot_input;
pub mod stick_nav;
