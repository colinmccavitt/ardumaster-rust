//! Geofence type bits, enable leftover, circle / alt-max / alt-min checks,
//! `check()` orchestration, pre-arm, dest-inside, and auto-enable-on-arm/
//! takeoff leftovers. Upstream `libraries/AC_Fence`. Tracked as **COP-025**.
//!
//! This is the first real `AC_Fence` leftover. Plane already has a
//! `FENCE_ACTION` table in `ap-plane::fence_failsafe_hookup`; that hookup
//! now decodes through [`Action`] here. Polygon EEPROM / `AC_PolyFence_loader`
//! stays later.
//!
//! # Enable is a change mask, not a bool
//!
//! [`Fence::enable`] returns the bits that flipped. Asking to enable a
//! type that is already on, or a type that is not in `FENCE_TYPE`, is a
//! no-op for the bitmask — but the min-alt manual-state leftover still
//! runs *before* that early return when `update_auto_enable` is set and
//! alt-min is among the configured types being asked about.
//!
//! # Circle / alt-max checks inject AHRS
//!
//! ADR-0004 forbids the AHRS singleton. [`CheckCircleContext`] is the
//! leftover of `ahrs.get_relative_position_NE_home`. [`CheckAltMaxContext`]
//! is the leftover of `get_alt_in_alt_max_frame_m`. [`CheckAltMinContext`]
//! is the leftover of `get_alt_in_alt_min_frame_m`. A missing altitude is
//! a fresh breach that does **not** call `record_breach` — [`Fence::check`]
//! reports `TYPE_ALT_MAX` / `TYPE_ALT_MIN` every cycle until the frame is
//! available again.
//!
//! # `check()` is the scheduler
//!
//! [`Fence::check`] is the leftover of `AC_Fence::check`. It clears
//! breaches from types that are no longer configured or that landing
//! asked to auto-disable, then calls the circle / alt checkers. A live
//! [`MANUAL_RECOVERY_TIME_MIN_MS`] window still records the breach but
//! returns 0 so the vehicle does not re-take control. The polygon
//! checker and poly-loader semaphore stay later.
//!
//! # What this crate does not own
//!
//! `AC_PolyFence_loader`, EEPROM / SD storage, and the polygon checker
//! stay later leftovers.

#![no_std]

pub mod fence;

pub use fence::{
    Action, AutoEnable, AutoEnableLeftover, AutoEnablePrint, CheckAltMaxContext,
    CheckAltMaxLeftover, CheckAltMinContext, CheckAltMinLeftover, CheckCircleContext,
    CheckCircleLeftover, CheckContext, CheckLeftover, DestFenceContext, DestFenceLeftover,
    EnableLeftover, Fence, MinAltState, PreArmContext, PreArmFailure, PreArmLeftover,
    ALT_MAX_BACKUP_DISTANCE_M, ALT_MAX_DEFAULT_M, ALT_MIN_BACKUP_DISTANCE_M, ALT_MIN_DEFAULT_M,
    ARMING_FENCES, AUTOENABLE_WARN_INTERVAL_MS, CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M,
    CIRCLE_RADIUS_BACKUP_DISTANCE_PLANE_M, CIRCLE_RADIUS_DEFAULT_M, FENCE_TYPE_DEFAULT_COPTER,
    FENCE_TYPE_DEFAULT_PLANE, FENCE_TYPE_DEFAULT_ROVER, GIVE_UP_DISTANCE_M,
    MANUAL_RECOVERY_TIME_MIN_MS, MARGIN_DEFAULT_M, TYPE_ALL, TYPE_ALT_MAX, TYPE_ALT_MIN,
    TYPE_CIRCLE, TYPE_POLYGON,
};
