//! OA database leftover. Upstream `AP_OADatabase`.
//!
//! This slice is [`Database::queue_push`] plus [`Database::update`]
//! (process-queue + expire). [`QueuePushContext`] is the leftover of
//! `AP::ahrs().get_relative_position_NED_home` used by the Copter
//! `ALT_MIN` near-home reject. GCS `send_adsb_vehicle` stays a later
//! leftover.
//!
//! ADR-0004 forbids the AHRS / HAL / GCS singletons. Vertical BendyRuler
//! and lean-angle avoidance in non-GPS modes stay later leftovers.

use ap_math::scalar::{is_equal, is_positive, is_zero, radians, sq, Real};
use ap_math::vector3::Vector3f;

use crate::oa_bendy_ruler::{OaDbItem, OA_DB_ITEMS_MAX};

/// Default `OA_DB_SIZE`. Upstream `AP_OADATABASE_SIZE_DEFAULT`.
pub const SIZE_DEFAULT: u16 = 100;
/// Default `OA_DB_QUEUE_SIZE`. Upstream `AP_OADATABASE_QUEUE_SIZE_DEFAULT`.
pub const QUEUE_SIZE_DEFAULT: u16 = 80;
/// Default `OA_DB_EXPIRE`, seconds. Upstream `AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT`.
pub const TIMEOUT_SECONDS_DEFAULT: i16 = 10;
/// Near-home radius for `ALT_MIN`, metres. Upstream `AP_OADATABASE_DISTANCE_FROM_HOME`.
pub const DISTANCE_FROM_HOME_M: f32 = 3.0;
/// Default `OA_DB_BEAM_WIDTH`, degrees.
pub const BEAM_WIDTH_DEFAULT_DEG: f32 = 5.0;
/// Default `OA_DB_RADIUS_MIN`, metres.
pub const RADIUS_MIN_DEFAULT_M: f32 = 0.01;
/// Default `OA_DB_DIST_MAX`, metres. Zero disables the limit.
pub const DIST_MAX_DEFAULT_M: f32 = 0.0;
/// Default Copter `OA_DB_ALT_MIN`, metres. Zero disables the check.
pub const MIN_ALT_DEFAULT_M: f32 = 0.0;
/// Default `OA_DB_OUTPUT`. Upstream `OutputLevel::HIGH`.
pub const OUTPUT_DEFAULT: i8 = 1;
/// Refresh when this much time has passed, milliseconds.
pub const REFRESH_MS: u32 = 500;
/// One `process_queue` call pops at most this many. Upstream `MIN(available, 100U)`.
pub const PROCESS_QUEUE_CAP: u16 = 100;
/// Fixed leftover queue table. Upstream default allocation is [`QUEUE_SIZE_DEFAULT`].
pub const QUEUE_MAX: usize = 80;
/// Fixed leftover database table. Upstream default allocation is [`SIZE_DEFAULT`].
pub const DATABASE_MAX: usize = 100;

/// Item importance. Upstream `AP_OADatabase::OA_DbItemImportance`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaDbImportance {
    /// Lowest GCS priority. Upstream `Low`.
    Low,
    /// Default push importance. Upstream `Normal`.
    Normal,
    /// Highest GCS priority. Upstream `High`.
    High,
}

/// Item source. Upstream `AP_OADatabase::OA_DbItem::Source`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaDbSource {
    /// Proximity / lidar. Match by overlapping radius.
    Proximity,
    /// AIS contact. Match by `id`.
    Ais,
}

/// GCS output filter. Upstream `AP_OADatabase::OutputLevel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum OaDbOutputLevel {
    /// Send nothing. Upstream `NONE`.
    None = 0,
    /// Send High only. Upstream `HIGH`.
    High = 1,
    /// Send High and Normal. Upstream `HIGH_AND_NORMAL`.
    HighAndNormal = 2,
    /// Send every item. Upstream `ALL`.
    All = 3,
}

impl OaDbOutputLevel {
    /// Leftover of the `OA_DB_OUTPUT` parameter.
    #[must_use]
    pub const fn from_param(v: i8) -> Self {
        match v {
            0 => Self::None,
            2 => Self::HighAndNormal,
            3 => Self::All,
            _ => Self::High,
        }
    }
}

/// One stored object. Upstream `AP_OADatabase::OA_DbItem`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OaDatabaseItem {
    /// Position, metres from the EKF origin (NEU). Upstream `pos`.
    pub pos_neu_m: Vector3f,
    /// Last update time. Upstream `timestamp_ms`.
    pub timestamp_ms: u32,
    /// Radius, metres. Upstream `radius`.
    pub radius_m: f32,
    /// Source-assigned id. Unused by proximity. Upstream `id`.
    pub id: u32,
    /// GCS send bitmask. Upstream `send_to_gcs`.
    pub send_to_gcs: u8,
    /// GCS priority. Upstream `importance`.
    pub importance: OaDbImportance,
    /// Who produced the item. Upstream `source`.
    pub source: OaDbSource,
}

impl OaDatabaseItem {
    /// Thin leftover [`OaDbItem`] used by BendyRuler margin.
    #[must_use]
    pub fn to_bendy_item(self) -> OaDbItem {
        OaDbItem {
            pos_neu_m: self.pos_neu_m,
            radius_m: self.radius_m,
        }
    }
}

impl Default for OaDatabaseItem {
    fn default() -> Self {
        Self {
            pos_neu_m: Vector3f::zero(),
            timestamp_ms: 0,
            radius_m: 0.0,
            id: 0,
            send_to_gcs: 0,
            importance: OaDbImportance::Normal,
            source: OaDbSource::Proximity,
        }
    }
}

/// Injected leftover of `get_relative_position_NED_home` for `ALT_MIN`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QueuePushContext {
    /// Leftover of the AHRS home-relative NED read succeeding.
    pub home_ned_valid: bool,
    /// Vehicle NED metres from home. `z` is down.
    pub pos_ned_home_m: Vector3f,
}

impl Default for QueuePushContext {
    fn default() -> Self {
        Self {
            home_ned_valid: true,
            pos_ned_home_m: Vector3f::zero(),
        }
    }
}

impl QueuePushContext {
    /// Vehicle at home, altitude `alt_m` above home.
    #[must_use]
    pub fn at_home(alt_m: f32) -> Self {
        Self {
            home_ned_valid: true,
            pos_ned_home_m: Vector3f::new(0.0, 0.0, -alt_m),
        }
    }

    /// Vehicle `ne_m` from home, altitude `alt_m` above home.
    #[must_use]
    pub fn away_from_home(north_m: f32, east_m: f32, alt_m: f32) -> Self {
        Self {
            home_ned_valid: true,
            pos_ned_home_m: Vector3f::new(north_m, east_m, -alt_m),
        }
    }
}

/// Obstacle database leftover. Upstream `AP_OADatabase`.
#[derive(Debug, Clone)]
pub struct Database {
    queue_size_param: u16,
    database_size_param: u16,
    expiry_seconds: i16,
    output_level: OaDbOutputLevel,
    beam_width_deg: f32,
    radius_min_m: f32,
    dist_max_m: f32,
    min_alt_m: f32,
    dist_to_radius_scalar: f32,
    queue: [OaDatabaseItem; QUEUE_MAX],
    queue_head: u16,
    queue_len: u16,
    queue_size: u16,
    items: [OaDatabaseItem; DATABASE_MAX],
    count: u16,
    database_size: u16,
}

impl Default for Database {
    fn default() -> Self {
        Self::new()
    }
}

impl Database {
    /// Construct and `init` with upstream parameter defaults.
    #[must_use]
    pub fn new() -> Self {
        let mut db = Self {
            queue_size_param: QUEUE_SIZE_DEFAULT,
            database_size_param: SIZE_DEFAULT,
            expiry_seconds: TIMEOUT_SECONDS_DEFAULT,
            output_level: OaDbOutputLevel::from_param(OUTPUT_DEFAULT),
            beam_width_deg: BEAM_WIDTH_DEFAULT_DEG,
            radius_min_m: RADIUS_MIN_DEFAULT_M,
            dist_max_m: DIST_MAX_DEFAULT_M,
            min_alt_m: MIN_ALT_DEFAULT_M,
            dist_to_radius_scalar: 0.0,
            queue: [OaDatabaseItem::default(); QUEUE_MAX],
            queue_head: 0,
            queue_len: 0,
            queue_size: 0,
            items: [OaDatabaseItem::default(); DATABASE_MAX],
            count: 0,
            database_size: 0,
        };
        db.init();
        db
    }

    /// Leftover of `AP_OADatabase::init` / `init_queue` / `init_database`.
    pub fn init(&mut self) {
        self.queue_size = self
            .queue_size_param
            .min(u16::try_from(QUEUE_MAX).unwrap_or(u16::MAX));
        self.database_size = self
            .database_size_param
            .min(u16::try_from(DATABASE_MAX).unwrap_or(u16::MAX));
        if self.queue_size_param == 0 {
            self.queue_size = 0;
        }
        if self.database_size_param == 0 {
            self.database_size = 0;
        }
        self.queue_head = 0;
        self.queue_len = 0;
        self.count = 0;
        self.dist_to_radius_scalar = Real::tan(radians(self.beam_width_deg.max(1.0)));
    }

    /// `true` when both tables allocated. Upstream `healthy()`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.queue_size > 0 && self.database_size > 0
    }

    /// Number of stored objects. Upstream `database_count()`.
    #[must_use]
    pub fn database_count(&self) -> u16 {
        self.count
    }

    /// Fetch item `i`. `None` when `i >= count`. Upstream `get_item`.
    #[must_use]
    pub fn get_item(&self, i: u16) -> Option<OaDatabaseItem> {
        if i >= self.count {
            return None;
        }
        self.items.get(usize::from(i)).copied()
    }

    /// First [`OA_DB_ITEMS_MAX`] items as BendyRuler leftover slots.
    #[must_use]
    pub fn fill_bendy_items(&self) -> [Option<OaDbItem>; OA_DB_ITEMS_MAX] {
        let mut out = [None; OA_DB_ITEMS_MAX];
        for (i, slot) in out.iter_mut().enumerate() {
            if let Some(item) = self.get_item(u16::try_from(i).unwrap_or(u16::MAX)) {
                *slot = Some(item.to_bendy_item());
            }
        }
        out
    }

    /// `OA_DB_SIZE`.
    #[must_use]
    pub fn database_size(&self) -> u16 {
        self.database_size
    }

    /// `OA_DB_QUEUE_SIZE`.
    #[must_use]
    pub fn queue_size(&self) -> u16 {
        self.queue_size
    }

    /// Pending queue depth.
    #[must_use]
    pub fn queue_len(&self) -> u16 {
        self.queue_len
    }

    /// `OA_DB_EXPIRE`.
    #[must_use]
    pub fn expiry_seconds(&self) -> i16 {
        self.expiry_seconds
    }

    /// `OA_DB_OUTPUT`.
    #[must_use]
    pub fn output_level(&self) -> OaDbOutputLevel {
        self.output_level
    }

    /// Beam-width scalar used when no radius is given. Upstream `dist_to_radius_scalar`.
    #[must_use]
    pub fn dist_to_radius_scalar(&self) -> f32 {
        self.dist_to_radius_scalar
    }

    /// `OA_DB_SIZE` setter. Call [`Self::init`] to apply (reboot-required upstream).
    pub fn set_database_size_param(&mut self, size: u16) {
        self.database_size_param = size;
    }

    /// `OA_DB_QUEUE_SIZE` setter. Call [`Self::init`] to apply.
    pub fn set_queue_size_param(&mut self, size: u16) {
        self.queue_size_param = size;
    }

    /// `OA_DB_EXPIRE` setter. Zero means never expire.
    pub fn set_expiry_seconds(&mut self, seconds: i16) {
        self.expiry_seconds = seconds;
    }

    /// `OA_DB_OUTPUT` setter.
    pub fn set_output_level(&mut self, level: OaDbOutputLevel) {
        self.output_level = level;
    }

    /// `OA_DB_BEAM_WIDTH` setter. Recomputes the radius scalar.
    pub fn set_beam_width_deg(&mut self, deg: f32) {
        self.beam_width_deg = deg;
        self.dist_to_radius_scalar = Real::tan(radians(self.beam_width_deg.max(1.0)));
    }

    /// `OA_DB_RADIUS_MIN` setter.
    pub fn set_radius_min_m(&mut self, radius_m: f32) {
        self.radius_min_m = radius_m;
    }

    /// `OA_DB_DIST_MAX` setter. Zero disables the limit.
    pub fn set_dist_max_m(&mut self, dist_m: f32) {
        self.dist_max_m = dist_m;
    }

    /// Copter `OA_DB_ALT_MIN` setter. Zero disables the check.
    pub fn set_min_alt_m(&mut self, alt_m: f32) {
        self.min_alt_m = alt_m;
    }

    /// Leftover of `get_send_to_gcs_flags`. `0xFF` = send, `0` = hold.
    #[must_use]
    pub fn send_to_gcs_flags(&self, importance: OaDbImportance) -> u8 {
        gcs_flags(self.output_level, importance)
    }

    /// Push with beam-width radius. Upstream `queue_push` without radius.
    pub fn queue_push(
        &mut self,
        pos_neu_m: Vector3f,
        timestamp_ms: u32,
        distance_m: f32,
        source: OaDbSource,
        id: u32,
        ctx: &QueuePushContext,
    ) {
        let radius_m = distance_m * self.dist_to_radius_scalar;
        self.queue_push_radius(
            pos_neu_m,
            timestamp_ms,
            distance_m,
            radius_m,
            source,
            id,
            ctx,
        );
    }

    /// Push with an explicit radius. Upstream `queue_push` with radius.
    pub fn queue_push_radius(
        &mut self,
        pos_neu_m: Vector3f,
        timestamp_ms: u32,
        distance_m: f32,
        mut radius_m: f32,
        source: OaDbSource,
        id: u32,
        ctx: &QueuePushContext,
    ) {
        if !self.healthy() {
            return;
        }

        if !is_zero(self.min_alt_m) {
            if !ctx.home_ned_valid {
                return;
            }
            if ctx.pos_ned_home_m.xy().length() < DISTANCE_FROM_HOME_M
                && -ctx.pos_ned_home_m.z < self.min_alt_m
            {
                return;
            }
        }

        radius_m = self.radius_min_m.max(radius_m);

        if is_positive(self.dist_max_m) {
            let closest_point = distance_m - radius_m;
            if closest_point > self.dist_max_m {
                return;
            }
        }

        let item = OaDatabaseItem {
            pos_neu_m,
            timestamp_ms,
            radius_m,
            id,
            send_to_gcs: 0,
            importance: OaDbImportance::Normal,
            source,
        };
        self.queue_enqueue(item);
    }

    /// Process the queue then expire. Upstream `update`.
    pub fn update(&mut self, now_ms: u32) {
        if !self.healthy() {
            return;
        }
        let _more = self.process_queue();
        self.database_items_remove_all_expired(now_ms);
    }

    /// Empty the queue into the database. `true` if more work remains.
    /// Upstream `process_queue`.
    pub fn process_queue(&mut self) -> bool {
        if !self.healthy() {
            return false;
        }
        let queue_available = self.queue_len.min(PROCESS_QUEUE_CAP);
        if queue_available == 0 {
            return false;
        }
        for _ in 0..queue_available {
            let Some(mut item) = self.queue_dequeue() else {
                return false;
            };
            item.send_to_gcs = gcs_flags(self.output_level, item.importance);
            let mut found = false;
            for i in 0..self.count {
                if let Some(existing) = self.items.get(usize::from(i)).copied() {
                    if item_match(&existing, &item) {
                        let flags = gcs_flags(self.output_level, existing.importance);
                        if let Some(current) = self.items.get_mut(usize::from(i)) {
                            database_item_refresh(current, &item, flags);
                        }
                        found = true;
                        break;
                    }
                }
            }
            if !found {
                self.database_item_add(item);
            }
        }
        self.queue_len > 0
    }

    fn queue_enqueue(&mut self, item: OaDatabaseItem) {
        if self.queue_size == 0 || self.queue_len >= self.queue_size {
            return;
        }
        let size = usize::from(self.queue_size);
        let idx = (usize::from(self.queue_head) + usize::from(self.queue_len)) % size;
        if let Some(slot) = self.queue.get_mut(idx) {
            *slot = item;
            self.queue_len = self.queue_len.saturating_add(1);
        }
    }

    fn queue_dequeue(&mut self) -> Option<OaDatabaseItem> {
        if self.queue_len == 0 || self.queue_size == 0 {
            return None;
        }
        let item = self.queue.get(usize::from(self.queue_head)).copied()?;
        let size = usize::from(self.queue_size);
        self.queue_head = u16::try_from((usize::from(self.queue_head) + 1) % size).unwrap_or(0);
        self.queue_len = self.queue_len.saturating_sub(1);
        Some(item)
    }

    fn database_item_add(&mut self, item: OaDatabaseItem) {
        if self.count >= self.database_size {
            return;
        }
        let flags = gcs_flags(self.output_level, item.importance);
        if let Some(slot) = self.items.get_mut(usize::from(self.count)) {
            *slot = item;
            slot.send_to_gcs = flags;
            self.count = self.count.saturating_add(1);
        }
    }

    fn database_item_remove(&mut self, index: u16) {
        if index >= self.count || self.count == 0 {
            return;
        }
        let flags = self
            .items
            .get(usize::from(index))
            .map(|slot| gcs_flags(self.output_level, slot.importance))
            .unwrap_or(0);
        if let Some(slot) = self.items.get_mut(usize::from(index)) {
            slot.radius_m = 0.0;
            slot.send_to_gcs = flags;
        }
        self.count = self.count.saturating_sub(1);
        if self.count == 0 {
            return;
        }
        if index != self.count {
            if let Some(moved) = self.items.get(usize::from(self.count)).copied() {
                let moved_flags = gcs_flags(self.output_level, moved.importance);
                if let Some(slot) = self.items.get_mut(usize::from(index)) {
                    *slot = moved;
                    slot.send_to_gcs = moved_flags;
                }
            }
        }
    }

    fn database_items_remove_all_expired(&mut self, now_ms: u32) {
        if self.expiry_seconds <= 0 {
            return;
        }
        let expiry_ms = u32::try_from(self.expiry_seconds)
            .unwrap_or(0)
            .saturating_mul(1000);
        let mut index = 0_u16;
        while index < self.count {
            let expired = self
                .items
                .get(usize::from(index))
                .is_some_and(|item| now_ms.wrapping_sub(item.timestamp_ms) > expiry_ms);
            if expired {
                self.database_item_remove(index);
            } else {
                index = index.saturating_add(1);
            }
        }
    }
}

fn gcs_flags(level: OaDbOutputLevel, importance: OaDbImportance) -> u8 {
    match importance {
        OaDbImportance::Low if level >= OaDbOutputLevel::All => 0xFF,
        OaDbImportance::Normal if level >= OaDbOutputLevel::HighAndNormal => 0xFF,
        OaDbImportance::High if level >= OaDbOutputLevel::High => 0xFF,
        _ => 0,
    }
}

/// Leftover of `AP_OADatabase::item_match`.
fn item_match(a: &OaDatabaseItem, b: &OaDatabaseItem) -> bool {
    if a.source != b.source {
        return false;
    }
    match a.source {
        OaDbSource::Ais => a.id == b.id,
        OaDbSource::Proximity => {
            let distance_sq = (a.pos_neu_m - b.pos_neu_m).length_squared();
            distance_sq < sq(a.radius_m.max(b.radius_m))
        }
    }
}

/// Leftover of `AP_OADatabase::database_item_refresh`.
fn database_item_refresh(current: &mut OaDatabaseItem, new_item: &OaDatabaseItem, send_flags: u8) {
    let age_ms = new_item.timestamp_ms.wrapping_sub(current.timestamp_ms);
    let is_different = !is_equal(current.radius_m, new_item.radius_m) || age_ms >= REFRESH_MS;
    if !is_different {
        return;
    }
    current.timestamp_ms = new_item.timestamp_ms;
    current.radius_m = new_item.radius_m;
    current.send_to_gcs = send_flags;
    if current.source == OaDbSource::Ais {
        current.pos_neu_m = new_item.pos_neu_m;
    }
}
