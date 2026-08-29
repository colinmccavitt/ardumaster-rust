//! First `AC_PolyFence_loader` leftover: inclusion-circle `breached()`
//! and `check_inclusion_circle_margin`. EEPROM / SD storage, polygon
//! vertices, and exclusion zones stay later. Upstream
//! `libraries/AC_Fence/AC_PolyFence_loader.cpp`. Tracked as **COP-025**.

use ap_math::location::Location;
use ap_math::scalar::is_equal;

/// `FENCE_OPTIONS` bit 1. Upstream `AC_Fence::OPTIONS::INCLUSION_UNION`.
pub const OPTION_INCLUSION_UNION: u16 = 1 << 1;

/// In-memory inclusion circles this leftover can hold. EEPROM stays later.
pub const MAX_INCLUSION_CIRCLES: usize = 8;

/// One circular inclusion zone. Upstream `AC_PolyFence_loader::InclusionCircle`.
///
/// `point` is absolute lat/lng (1e-7 deg). `radius` is metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InclusionCircle {
    /// Latitude, 1e-7 degrees. Upstream `InclusionCircle::point.x`.
    pub lat: i32,
    /// Longitude, 1e-7 degrees. Upstream `InclusionCircle::point.y`.
    pub lng: i32,
    /// Radius in metres.
    pub radius_m: f32,
}

impl InclusionCircle {
    /// Seat a circular inclusion zone.
    #[must_use]
    pub const fn new(lat: i32, lng: i32, radius_m: f32) -> Self {
        Self { lat, lng, radius_m }
    }

    /// Absolute centre. Upstream builds a `Location` from `point.x` / `point.y`.
    #[must_use]
    pub const fn center(self) -> Location {
        Location::new(self.lat, self.lng)
    }
}

/// `AC_PolyFence_loader::breached(loc, distance, direction)` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreachedLeftover {
    /// C++ return: the location is outside the inclusion set.
    pub breached: bool,
    /// `distance_outside_fence` out-param, metres. Positive is outside.
    pub distance_outside_m: f32,
    /// `!loaded() || total_fence_count() == 0`.
    pub skipped: bool,
    /// Inclusion circles considered. Polygon vertices stay later.
    pub num_inclusion: u16,
    /// How many of those the location was outside.
    pub num_inclusion_outside: u16,
}

/// First `AC_PolyFence_loader` leftover. Holds in-memory inclusion
/// circles only — no EEPROM index, no SD, no polygon vertices.
#[derive(Debug, Clone, PartialEq)]
pub struct PolyFence {
    inclusion: [InclusionCircle; MAX_INCLUSION_CIRCLES],
    inclusion_count: u8,
    options: u16,
    /// `_load_time_ms != 0`. EEPROM does not set this; the leftover does.
    loaded: bool,
}

impl Default for PolyFence {
    fn default() -> Self {
        Self::new()
    }
}

impl PolyFence {
    /// Empty loader. `loaded()` is false until a circle is seated or
    /// [`Self::set_loaded`] is called.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            inclusion: [InclusionCircle::new(0, 0, 0.0); MAX_INCLUSION_CIRCLES],
            inclusion_count: 0,
            options: 0,
            loaded: false,
        }
    }

    /// `loaded()` — `_load_time_ms != 0`.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.loaded
    }

    /// Seed `loaded()` without EEPROM. Used by tests and the fence leftover.
    pub fn set_loaded(&mut self, loaded: bool) {
        self.loaded = loaded;
    }

    /// `FENCE_OPTIONS` bits passed into the loader.
    #[must_use]
    pub const fn options(&self) -> u16 {
        self.options
    }

    /// Set `FENCE_OPTIONS`.
    pub fn set_options(&mut self, options: u16) {
        self.options = options;
    }

    /// Bit 1 — union of inclusion areas instead of intersection.
    #[must_use]
    pub const fn inclusion_union(&self) -> bool {
        self.options & OPTION_INCLUSION_UNION != 0
    }

    /// `get_inclusion_circle_count`.
    #[must_use]
    pub const fn inclusion_circle_count(&self) -> u8 {
        self.inclusion_count
    }

    /// `total_fence_count` for this slice — inclusion circles only.
    #[must_use]
    pub const fn total_fence_count(&self) -> u16 {
        self.inclusion_count as u16
    }

    /// Drop seated circles and mark the leftover unloaded.
    pub fn clear(&mut self) {
        self.inclusion_count = 0;
        self.loaded = false;
    }

    /// Seat one inclusion circle. Returns false when the leftover is full.
    ///
    /// Seating a circle also marks the leftover loaded — the C++ path
    /// does that only after EEPROM read; this slice has no storage.
    pub fn push_inclusion_circle(&mut self, circle: InclusionCircle) -> bool {
        let i = usize::from(self.inclusion_count);
        if i >= MAX_INCLUSION_CIRCLES {
            return false;
        }
        let Some(slot) = self.inclusion.get_mut(i) else {
            return false;
        };
        *slot = circle;
        self.inclusion_count = self.inclusion_count.saturating_add(1);
        self.loaded = true;
        true
    }

    /// `AC_PolyFence_loader::check_inclusion_circle_margin`.
    ///
    /// False when any seated inclusion radius is smaller than `margin`.
    #[must_use]
    pub fn check_inclusion_circle_margin(&self, margin: f32) -> bool {
        for i in 0..usize::from(self.inclusion_count) {
            let Some(circle) = self.inclusion.get(i) else {
                break;
            };
            if circle.radius_m < margin {
                return false;
            }
        }
        true
    }

    /// `breached(loc)` — true when the location is outside the inclusion set.
    #[must_use]
    pub fn breached(&self, loc: Location) -> bool {
        self.breached_at(loc).breached
    }

    /// `breached(loc, distance_outside_fence, fence_direction)`.
    ///
    /// Exclusion polygons / circles and vertex polygons stay later, so
    /// this leftover only walks inclusion circles. `fence_direction` is
    /// not written — the C++ circle loops leave it alone too.
    #[must_use]
    pub fn breached_at(&self, loc: Location) -> BreachedLeftover {
        if !self.loaded || self.total_fence_count() == 0 {
            return BreachedLeftover {
                breached: false,
                distance_outside_m: 0.0,
                skipped: true,
                num_inclusion: 0,
                num_inclusion_outside: 0,
            };
        }

        let num_inclusion = self.inclusion_count as u16;
        let mut num_inclusion_outside = 0_u16;
        let mut distance_outside_m = f32::MIN;

        for i in 0..usize::from(self.inclusion_count) {
            let Some(circle) = self.inclusion.get(i) else {
                break;
            };
            let center = circle.center();
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_precision_loss,
                reason = "upstream stores distance as float metres from Location::get_distance"
            )]
            let diff_m = loc.get_distance(center) as f32;
            let diff_cm = diff_m * 100.0;
            distance_outside_m = distance_outside_m.max(diff_m - circle.radius_m);
            if diff_cm > circle.radius_m * 100.0 {
                num_inclusion_outside = num_inclusion_outside.saturating_add(1);
            }
        }

        let breached = if self.inclusion_union() {
            num_inclusion > 0 && num_inclusion == num_inclusion_outside
        } else {
            num_inclusion_outside > 0
        };

        if is_equal(distance_outside_m, f32::MIN) {
            distance_outside_m = 0.0;
        }

        BreachedLeftover {
            breached,
            distance_outside_m,
            skipped: false,
            num_inclusion,
            num_inclusion_outside,
        }
    }
}
