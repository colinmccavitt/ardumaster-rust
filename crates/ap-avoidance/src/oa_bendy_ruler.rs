//! BendyRuler leftover. Upstream `AP_OABendyRuler`.
//!
//! This slice is [`BendyRuler::update`] plus the horizontal search
//! (`search_xy_path`) and [`BendyRuler::resist_bearing_change`].
//! [`BendyMarginContext`] is the leftover of `calc_avoidance_margin` /
//! `calc_margin_from_object_database`. Fence-circle / polygon / alt-fence
//! margin arms, vertical search, and the OA database itself stay later
//! leftovers.
//!
//! ADR-0004 forbids the AHRS / fence / OA-database singletons.

use ap_math::location::Location;
use ap_math::scalar::{constrain_value, degrees, is_equal, is_positive, wrap_180, Real};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_math::Ftype;

/// C++ BendyRuler is `float`. `Location` distances are [`Ftype`].
fn metres_f32(v: Ftype) -> f32 {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream AP_OABendyRuler stores lookahead and margin as float"
    )]
    {
        v.to_f64() as f32
    }
}

/// `Location::offset_bearing` takes [`Ftype`].
fn loc_offset(loc: &mut Location, bearing_deg: f32, distance_m: f32) {
    loc.offset_bearing(Ftype::from(bearing_deg), Ftype::from(distance_m));
}

/// Default `OA_BR_LOOKAHEAD`, metres.
pub const LOOKAHEAD_M_DEFAULT: f32 = 15.0;
/// Default `OA_BR_CONT_RATIO`.
pub const RATIO_DEFAULT: f32 = 1.5;
/// Default `OA_BR_CONT_ANGLE`, degrees.
pub const ANGLE_DEFAULT: i16 = 75;
/// Default `OA_BR_TYPE` (horizontal).
pub const TYPE_DEFAULT: i8 = 1;

/// Horizontal probe increment, degrees. Upstream `OA_BENDYRULER_BEARING_INC_XY`.
pub const BEARING_INC_XY_DEG: f32 = 5.0;
/// Step-2 lookahead as a ratio of step-1. Upstream `OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO`.
pub const LOOKAHEAD_STEP2_RATIO: f32 = 1.0;
/// Step-2 looks at least this far, metres. Upstream `OA_BENDYRULER_LOOKAHEAD_STEP2_MIN`.
pub const LOOKAHEAD_STEP2_MIN_M: f32 = 2.0;
/// Lookahead is at least this far past the destination, metres.
pub const LOOKAHEAD_PAST_DEST_M: f32 = 2.0;
/// Below this ground-speed squared, use yaw. Upstream `OA_BENDYRULER_LOW_SPEED_SQUARED`.
pub const LOW_SPEED_SQUARED: f32 = 0.2 * 0.2;

/// Max injected OA-database items in one leftover call.
pub const OA_DB_ITEMS_MAX: usize = 8;

/// BendyRuler flavour. Upstream `AP_OABendyRuler::OABendyType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaBendyType {
    /// Disabled / unused. Upstream `OA_BENDY_DISABLED`.
    Disabled = 0,
    /// Horizontal search. Upstream `OA_BENDY_HORIZONTAL`.
    Horizontal = 1,
    /// Vertical search. Upstream `OA_BENDY_VERTICAL`. Later leftover.
    Vertical = 2,
}

/// One leftover OA-database item. Upstream `AP_OADatabase::OA_DbItem`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OaDbItem {
    /// Position, metres from the EKF origin. Upstream `item.pos`.
    pub pos_neu_m: Vector3f,
    /// Obstacle radius, metres. Upstream `item.radius`.
    pub radius_m: f32,
}

/// Injected leftover of `calc_avoidance_margin` (object-database arm).
///
/// Fence margin arms stay later leftovers. [`BendyMarginContext::proximity_only`]
/// is the leftover of that flag: when set, only these items count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BendyMarginContext {
    /// Leftover of `get_vector_from_origin_NEU_cm` succeeding.
    pub origin_valid: bool,
    /// Leftover of the EKF origin used to convert [`Location`] → NEU cm.
    pub origin: Location,
    /// Leftover of `AP::oadatabase()` items. Unused slots are `None`.
    pub items: [Option<OaDbItem>; OA_DB_ITEMS_MAX],
    /// Leftover of `proximity_only` in `calc_avoidance_margin`.
    pub proximity_only: bool,
}

impl Default for BendyMarginContext {
    fn default() -> Self {
        Self {
            origin_valid: true,
            origin: Location::new(0, 0),
            items: [None; OA_DB_ITEMS_MAX],
            proximity_only: false,
        }
    }
}

impl BendyMarginContext {
    /// One-item leftover of a healthy OA database.
    #[must_use]
    pub fn one_item(origin: Location, item: OaDbItem) -> Self {
        let mut ctx = Self {
            origin_valid: true,
            origin,
            items: [None; OA_DB_ITEMS_MAX],
            proximity_only: false,
        };
        if let Some(slot) = ctx.items.first_mut() {
            *slot = Some(item);
        }
        ctx
    }
}

/// Leftover of one `AP_OABendyRuler::update` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BendyUpdateLeftover {
    /// `true` when an intermediate destination was chosen (OA is required).
    pub required: bool,
    /// Intermediate origin. BendyRuler always writes `current_loc`.
    pub origin_new: Location,
    /// Intermediate destination when [`Self::required`] is set.
    pub destination_new: Location,
    /// Flavour that ran. Upstream `bendy_type` out-param.
    pub bendy_type: OaBendyType,
}

/// Horizontal BendyRuler leftover. Upstream `AP_OABendyRuler`.
#[derive(Debug, Clone)]
pub struct BendyRuler {
    /// `OA_BR_LOOKAHEAD`, metres.
    lookahead_m: f32,
    /// `OA_BR_CONT_RATIO`.
    bendy_ratio: f32,
    /// `OA_BR_CONT_ANGLE`, degrees.
    bendy_angle_deg: f32,
    /// `OA_BR_TYPE`. Vertical stays a later leftover.
    bendy_type: i8,
    /// From the path-planner frontend. Upstream `_margin_max`.
    margin_max_m: f32,
    /// Dynamic lookahead. Upstream `_current_lookahead`.
    current_lookahead_m: f32,
    /// Stored bearing. Upstream `_bearing_prev`. `f32::MAX` means unset.
    bearing_prev: f32,
    /// Previous destination, lat/lon only. Upstream `_destination_prev`.
    destination_prev: Location,
}

impl Default for BendyRuler {
    fn default() -> Self {
        Self::new()
    }
}

impl BendyRuler {
    /// Construct with upstream parameter defaults.
    #[must_use]
    pub fn new() -> Self {
        Self {
            lookahead_m: LOOKAHEAD_M_DEFAULT,
            bendy_ratio: RATIO_DEFAULT,
            bendy_angle_deg: f32::from(ANGLE_DEFAULT),
            bendy_type: TYPE_DEFAULT,
            margin_max_m: 0.0,
            current_lookahead_m: 0.0,
            bearing_prev: f32::MAX,
            destination_prev: Location::new(0, 0),
        }
    }

    /// Leftover of `AP_OABendyRuler::set_config`.
    pub fn set_config(&mut self, margin_max_m: f32) {
        self.margin_max_m = margin_max_m.max(0.0);
    }

    /// `OA_BR_LOOKAHEAD`.
    #[must_use]
    pub fn lookahead_m(&self) -> f32 {
        self.lookahead_m
    }

    /// `OA_BR_LOOKAHEAD` setter (tests / param load).
    pub fn set_lookahead_m(&mut self, lookahead_m: f32) {
        self.lookahead_m = lookahead_m;
    }

    /// `OA_BR_CONT_RATIO`.
    #[must_use]
    pub fn bendy_ratio(&self) -> f32 {
        self.bendy_ratio
    }

    /// `OA_BR_CONT_RATIO` setter.
    pub fn set_bendy_ratio(&mut self, ratio: f32) {
        self.bendy_ratio = ratio;
    }

    /// `OA_BR_CONT_ANGLE`.
    #[must_use]
    pub fn bendy_angle_deg(&self) -> f32 {
        self.bendy_angle_deg
    }

    /// `OA_BR_CONT_ANGLE` setter.
    pub fn set_bendy_angle_deg(&mut self, angle_deg: f32) {
        self.bendy_angle_deg = angle_deg;
    }

    /// `OA_BR_TYPE` raw param.
    #[must_use]
    pub fn bendy_type_param(&self) -> i8 {
        self.bendy_type
    }

    /// `OA_BR_TYPE` setter. Vertical search stays a later leftover.
    pub fn set_bendy_type_param(&mut self, bendy_type: i8) {
        self.bendy_type = bendy_type;
    }

    /// Configured wide margin, metres.
    #[must_use]
    pub fn margin_max_m(&self) -> f32 {
        self.margin_max_m
    }

    /// Leftover of `AP_OABendyRuler::get_type`.
    #[must_use]
    pub fn get_type(&self) -> OaBendyType {
        match self.bendy_type {
            2 => OaBendyType::Vertical,
            1 => OaBendyType::Horizontal,
            _ => OaBendyType::Horizontal,
        }
    }

    /// Leftover of `AP_OABendyRuler::update`.
    ///
    /// `yaw_deg` is the leftover of `AP::ahrs().get_yaw_deg` when ground
    /// speed is below [`LOW_SPEED_SQUARED`].
    #[must_use]
    pub fn update(
        &mut self,
        current_loc: Location,
        destination: Location,
        ground_speed_vec: Vector2f,
        yaw_deg: f32,
        margin: &BendyMarginContext,
    ) -> BendyUpdateLeftover {
        let origin_new = current_loc;
        let mut leftover = BendyUpdateLeftover {
            required: false,
            origin_new,
            destination_new: destination,
            bendy_type: OaBendyType::Disabled,
        };

        let bearing_to_dest = current_loc.get_bearing_to(destination) as f32 * 0.01;
        let distance_to_dest = metres_f32(current_loc.get_distance(destination));

        self.lookahead_m = self.lookahead_m.max(1.0);
        self.current_lookahead_m = constrain_value(
            self.current_lookahead_m,
            self.lookahead_m * 0.5,
            self.lookahead_m,
        );

        let lookahead_step1_dist = self
            .current_lookahead_m
            .min(distance_to_dest + LOOKAHEAD_PAST_DEST_M);
        let lookahead_step2_dist = self.current_lookahead_m * LOOKAHEAD_STEP2_RATIO;

        let ground_course_deg = if ground_speed_vec.length_squared() < LOW_SPEED_SQUARED {
            yaw_deg
        } else {
            degrees(ground_speed_vec.angle())
        };

        match self.get_type() {
            OaBendyType::Vertical => {
                // Vertical search stays a later leftover.
                leftover.bendy_type = OaBendyType::Vertical;
                leftover.required = false;
            }
            OaBendyType::Horizontal | OaBendyType::Disabled => {
                leftover.bendy_type = OaBendyType::Horizontal;
                leftover.required = self.search_xy_path(
                    current_loc,
                    destination,
                    ground_course_deg,
                    lookahead_step1_dist,
                    lookahead_step2_dist,
                    bearing_to_dest,
                    distance_to_dest,
                    margin,
                    &mut leftover.destination_new,
                );
            }
        }

        leftover
    }

    /// Leftover of `AP_OABendyRuler::search_xy_path`.
    #[allow(
        clippy::too_many_arguments,
        reason = "matches the upstream search_xy_path argument list"
    )]
    fn search_xy_path(
        &mut self,
        current_loc: Location,
        destination: Location,
        ground_course_deg: f32,
        lookahead_step1_dist: f32,
        lookahead_step2_dist: f32,
        bearing_to_dest: f32,
        distance_to_dest: f32,
        margin: &BendyMarginContext,
        destination_new: &mut Location,
    ) -> bool {
        let mut best_bearing = bearing_to_dest;
        let mut best_bearing_margin = f32::MIN;
        let mut have_best_bearing = false;
        let mut best_margin = f32::MIN;
        let mut best_margin_bearing = best_bearing;

        let probes = (170.0 / BEARING_INC_XY_DEG) as u8;
        for i in 0..=probes {
            for bdir in 0_u8..=1 {
                if i == 0 && bdir > 0 {
                    continue;
                }
                let sign = if bdir == 0 { -1.0 } else { 1.0 };
                let bearing_delta = f32::from(i) * BEARING_INC_XY_DEG * sign;
                let bearing_test = wrap_180(bearing_to_dest + bearing_delta);

                let mut test_loc = current_loc;
                loc_offset(&mut test_loc, bearing_test, lookahead_step1_dist);

                let step1_margin = self.calc_avoidance_margin(current_loc, test_loc, margin);
                if step1_margin > best_margin {
                    best_margin_bearing = bearing_test;
                    best_margin = step1_margin;
                }
                if step1_margin <= self.margin_max_m {
                    continue;
                }

                if !have_best_bearing {
                    best_bearing = bearing_test;
                    best_bearing_margin = step1_margin;
                    have_best_bearing = true;
                } else if wrap_180(ground_course_deg - bearing_test).abs()
                    < wrap_180(ground_course_deg - best_bearing).abs()
                {
                    best_bearing = bearing_test;
                    best_bearing_margin = step1_margin;
                }

                const TEST_BEARINGS: [f32; 3] = [0.0, 45.0, -45.0];
                let bearing_to_dest2 = test_loc.get_bearing_to(destination) as f32 * 0.01;
                let distance2 = constrain_value(
                    lookahead_step2_dist,
                    LOOKAHEAD_STEP2_MIN_M,
                    metres_f32(test_loc.get_distance(destination)),
                );
                for (j, &tb) in TEST_BEARINGS.iter().enumerate() {
                    let bearing_test2 = wrap_180(bearing_to_dest2 + tb);
                    let mut test_loc2 = test_loc;
                    loc_offset(&mut test_loc2, bearing_test2, distance2);

                    let margin2 = self.calc_avoidance_margin(test_loc, test_loc2, margin);
                    if margin2 <= self.margin_max_m {
                        continue;
                    }

                    let active = i != 0 || j != 0;
                    let mut final_bearing = bearing_test;
                    let mut final_margin = step1_margin;
                    let dest_prev = self.destination_prev;
                    let bearing_prev = self.bearing_prev;
                    let (resisted, dest_out, bearing_out, fb, fm) = self.resist_bearing_change(
                        destination,
                        current_loc,
                        active,
                        bearing_test,
                        lookahead_step1_dist,
                        step1_margin,
                        dest_prev,
                        bearing_prev,
                        final_bearing,
                        final_margin,
                        margin,
                    );
                    let _ = resisted;
                    self.destination_prev = dest_out;
                    self.bearing_prev = bearing_out;
                    final_bearing = fb;
                    final_margin = fm;
                    let _ = final_margin;

                    *destination_new = current_loc;
                    loc_offset(
                        destination_new,
                        final_bearing,
                        distance_to_dest.min(lookahead_step1_dist),
                    );
                    self.current_lookahead_m = self.lookahead_m.min(self.current_lookahead_m * 1.1);
                    return active;
                }
            }
        }

        let (chosen_bearing, chosen_distance) = if have_best_bearing {
            self.current_lookahead_m = self.lookahead_m.min(self.current_lookahead_m * 1.05);
            (
                best_bearing,
                (lookahead_step1_dist + best_bearing_margin.min(0.0)).max(0.0),
            )
        } else {
            self.current_lookahead_m = (self.lookahead_m * 0.5).max(self.current_lookahead_m * 0.9);
            (
                best_margin_bearing,
                (lookahead_step1_dist + best_margin.min(0.0)).max(0.0),
            )
        };

        *destination_new = current_loc;
        loc_offset(destination_new, chosen_bearing, chosen_distance);
        true
    }

    /// Leftover of `AP_OABendyRuler::resist_bearing_change`.
    ///
    /// Returns `(resisted, dest_prev, bearing_prev, final_bearing, final_margin)`.
    #[allow(
        clippy::too_many_arguments,
        reason = "matches the upstream resist_bearing_change argument list"
    )]
    #[must_use]
    pub fn resist_bearing_change(
        &self,
        destination: Location,
        current_loc: Location,
        active: bool,
        bearing_test: f32,
        lookahead_step1_dist: f32,
        margin: f32,
        mut prev_dest: Location,
        mut prev_bearing: f32,
        mut final_bearing: f32,
        mut final_margin: f32,
        margin_ctx: &BendyMarginContext,
    ) -> (bool, Location, f32, f32, f32) {
        let mut resisted_change = false;
        let mut dest_change = false;
        if !same_latlon(destination, prev_dest) {
            dest_change = true;
            prev_dest = destination;
        }

        if active && !dest_change && is_positive(self.bendy_ratio) {
            if wrap_180(prev_bearing - bearing_test).abs() > self.bendy_angle_deg
                && !is_equal(prev_bearing, f32::MAX)
            {
                let mut test_loc_previous = current_loc;
                loc_offset(
                    &mut test_loc_previous,
                    wrap_180(prev_bearing),
                    lookahead_step1_dist,
                );
                let previous_bearing_margin =
                    self.calc_avoidance_margin(current_loc, test_loc_previous, margin_ctx);
                if margin < (self.bendy_ratio * previous_bearing_margin) {
                    final_bearing = prev_bearing;
                    final_margin = previous_bearing_margin;
                    resisted_change = true;
                }
            }
        } else {
            prev_bearing = f32::MAX;
        }
        if !resisted_change {
            prev_bearing = bearing_test;
        }

        (
            resisted_change,
            prev_dest,
            prev_bearing,
            final_bearing,
            final_margin,
        )
    }

    /// Leftover of `AP_OABendyRuler::calc_avoidance_margin`.
    ///
    /// Object-database arm only. Fence-circle / polygon / alt-fence stay later.
    #[must_use]
    pub fn calc_avoidance_margin(
        &self,
        start: Location,
        end: Location,
        ctx: &BendyMarginContext,
    ) -> f32 {
        let mut margin_min = f32::MAX;
        if let Some(latest) = self.calc_margin_from_object_database(start, end, ctx) {
            margin_min = margin_min.min(latest);
        }
        if ctx.proximity_only {
            return margin_min;
        }
        margin_min
    }

    /// Leftover of `AP_OABendyRuler::calc_margin_from_object_database`.
    fn calc_margin_from_object_database(
        &self,
        start: Location,
        end: Location,
        ctx: &BendyMarginContext,
    ) -> Option<f32> {
        if !ctx.origin_valid {
            return None;
        }
        let start_neu = loc_neu_cm(ctx.origin, start);
        let end_neu = loc_neu_cm(ctx.origin, end);
        if start_neu == end_neu {
            return None;
        }

        let mut smallest = f32::MAX;
        let mut found = false;
        for slot in &ctx.items {
            let Some(item) = slot else {
                continue;
            };
            let point_cm = Vector3f::new(
                item.pos_neu_m.x * 100.0,
                item.pos_neu_m.y * 100.0,
                item.pos_neu_m.z * 100.0,
            );
            let m = Vector3f::closest_distance_between_line_and_point(start_neu, end_neu, point_cm)
                * 0.01
                - item.radius_m;
            if m < smallest {
                smallest = m;
            }
            found = true;
        }
        if found {
            Some(smallest)
        } else {
            None
        }
    }
}

/// Leftover of `Location::same_latlon_as` (lat/lon only).
#[must_use]
pub fn same_latlon(a: Location, b: Location) -> bool {
    a.lat == b.lat && a.lng == b.lng
}

/// Leftover of `Location::get_vector_from_origin_NEU_cm`.
fn loc_neu_cm(origin: Location, loc: Location) -> Vector3f {
    let ne = origin.get_distance_ne(loc);
    Vector3f::new(ne.x * 100.0, ne.y * 100.0, (loc.alt - origin.alt) as f32)
}
