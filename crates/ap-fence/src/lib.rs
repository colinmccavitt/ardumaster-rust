//! Geofence type bits, enable leftover, circle / alt-max / alt-min checks,
//! `check()` orchestration, pre-arm, dest-inside, auto-enable-on-arm/
//! takeoff, `check_fence_polygon`, poly-loader inclusion / exclusion
//! circles, vertex polygons, EEPROM format, scan, index, `write_fence`,
//! `load_from_storage`, and SD init. Upstream `libraries/AC_Fence`.
//! Tracked as **COP-025**.
//!
//! This is the first real `AC_Fence` leftover. Plane already has a
//! `FENCE_ACTION` table in `ap-plane::fence_failsafe_hookup`; that hookup
//! now decodes through [`Action`] here.
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
//! asked to auto-disable, then calls the circle / alt / polygon checkers.
//! A live [`MANUAL_RECOVERY_TIME_MIN_MS`] window still records the breach
//! but returns 0 so the vehicle does not re-take control. The poly-loader
//! semaphore stays later.
//!
//! # Polygon / loader leftovers
//!
//! [`Fence::check_fence_polygon`] is the leftover of
//! `AC_Fence::check_fence_polygon`. [`poly_fence::PolyFence`] is the
//! `AC_PolyFence_loader` leftover: in-memory inclusion / exclusion
//! circles, vertex inclusion / exclusion polygons, `breached(loc)`,
//! `check_inclusion_circle_margin`, `load_from_storage`, and SD init.
//! [`poly_fence_storage`] is the EEPROM leftover — magic, item types,
//! `format()`, `fence_storage_space_required`, `scan_eeprom`, the storage
//! index, `validate_fence`, `write_fence`, and the SD attach leftover.
//!
//! # What this crate does not own
//!
//! Logger / `FENCE_TOTAL` param-save and the poly-loader semaphore stay
//! later leftovers.

#![no_std]

pub mod fence;
pub mod poly_fence;
pub mod poly_fence_storage;

pub use fence::{
    Action, AutoEnable, AutoEnableLeftover, AutoEnablePrint, CheckAltMaxContext,
    CheckAltMaxLeftover, CheckAltMinContext, CheckAltMinLeftover, CheckCircleContext,
    CheckCircleLeftover, CheckContext, CheckLeftover, CheckPolygonContext, CheckPolygonLeftover,
    DestFenceContext, DestFenceLeftover, EnableLeftover, Fence, MinAltState, PreArmContext,
    PreArmFailure, PreArmLeftover, ALT_MAX_BACKUP_DISTANCE_M, ALT_MAX_DEFAULT_M,
    ALT_MIN_BACKUP_DISTANCE_M, ALT_MIN_DEFAULT_M, ARMING_FENCES, AUTOENABLE_WARN_INTERVAL_MS,
    CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M, CIRCLE_RADIUS_BACKUP_DISTANCE_PLANE_M,
    CIRCLE_RADIUS_DEFAULT_M, FENCE_TYPE_DEFAULT_COPTER, FENCE_TYPE_DEFAULT_PLANE,
    FENCE_TYPE_DEFAULT_ROVER, GIVE_UP_DISTANCE_M, MANUAL_RECOVERY_TIME_MIN_MS, MARGIN_DEFAULT_M,
    TYPE_ALL, TYPE_ALT_MAX, TYPE_ALT_MIN, TYPE_CIRCLE, TYPE_POLYGON,
};
pub use poly_fence::{
    BreachedLeftover, ExclusionCircle, InclusionCircle, LoadFromStorageContext,
    LoadFromStorageLeftover, PolyFence, Vertex, VertexPolygon, MAX_EXCLUSION_CIRCLES,
    MAX_EXCLUSION_POLYGONS, MAX_INCLUSION_CIRCLES, MAX_INCLUSION_POLYGONS, MAX_POLYGON_VERTICES,
    OPTION_INCLUSION_UNION,
};
pub use poly_fence_storage::{
    count_eeprom_fences, fence_storage_space_required, format_storage, index_eeprom,
    index_fence_count, init_sdcard_storage, max_items, read_f32_from_storage,
    read_latlon_from_storage, read_u32_from_storage, read_uint8_from_storage,
    scale_latlon_from_origin, scan_eeprom, sdcard_fence_filename, storage_formatted,
    sum_of_polygon_point_counts_and_returnpoint, validate_fence, write_eos_to_storage,
    write_f32_to_storage, write_fence, write_latlon_to_storage, write_type_to_storage,
    write_uint8_to_storage, EepromCounts, FenceIndex, IndexResult, PolyFenceItem, PolyFenceType,
    SdcardFenceContext, SdcardInitLeftover, WriteFenceResult, SDCARD_FENCE_FILENAME,
    SDCARD_FENCE_FILENAME_CHIBIOS, STORAGE_MAGIC,
};
