//! Circle / polygon / beacon fence NE leftover.
//!
//! Upstream `AC_Avoid::adjust_velocity_fence` and the helpers it calls
//! (`adjust_velocity_circle_fence`, `adjust_velocity_inclusion_and_exclusion_polygons`,
//! `adjust_velocity_inclusion_circles`, `adjust_velocity_exclusion_circles`,
//! `adjust_velocity_beacon_fence`, `adjust_velocity_polygon`).
//!
//! ADR-0004 forbids the fence / AHRS / beacon singletons.
//! [`FenceNeContext`] injects those reads. The vertical fence tail stays
//! [`crate::avoid::Avoid::adjust_velocity_z`]. Accel-jerk limiting is
//! [`crate::avoid::Avoid::limit_accel_neu_cm`]. The OA path planner leftover is
//! [`crate::oa_path_planner`] / [`crate::oa_bendy_ruler`] / [`crate::oa_dijkstra`].

use ap_fence::{TYPE_CIRCLE, TYPE_POLYGON};
use ap_math::polygon::polygon_outside;
use ap_math::scalar::{is_negative, is_positive, is_zero, safe_sqrt, sq};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

use crate::avoid::{
    AdjustVelocityZContext, Avoid, ACCEL_CMSS_MAX, BEHAVIOR_STOP, STOP_AT_BEACON_FENCE,
    STOP_AT_FENCE,
};

/// Max vertices injected for one leftover polygon / beacon boundary.
pub const FENCE_NE_VERTICES_MAX: usize = 8;

/// One leftover polygon (inclusion, exclusion, or beacon), earth-frame cm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FencePolygon {
    /// Vertices, earth NE centimetres. Unused slots are zero.
    pub vertices_ne_cm: [Vector2f; FENCE_NE_VERTICES_MAX],
    /// Populated count. Upstream `num_points`.
    pub num_points: u8,
}

impl FencePolygon {
    /// Copy up to [`FENCE_NE_VERTICES_MAX`] vertices.
    #[must_use]
    pub fn from_slice(pts: &[Vector2f]) -> Self {
        let mut vertices_ne_cm = [Vector2f::zero(); FENCE_NE_VERTICES_MAX];
        let n = pts.len().min(FENCE_NE_VERTICES_MAX);
        vertices_ne_cm[..n].copy_from_slice(&pts[..n]);
        Self {
            vertices_ne_cm,
            num_points: n as u8,
        }
    }

    /// Populated vertices.
    #[must_use]
    pub fn as_slice(&self) -> &[Vector2f] {
        &self.vertices_ne_cm[..self.num_points as usize]
    }
}

/// One leftover polyfence circle (inclusion or exclusion).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceCircle {
    /// Center, earth-frame centimetres in the origin frame.
    pub center_ne_cm: Vector2f,
    /// Radius, metres. Upstream `radius_m`.
    pub radius_m: f32,
}

/// Injected leftovers of `AP::fence()` / AHRS / `AP::beacon()` inside
/// `adjust_velocity_fence`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceNeContext {
    /// Leftover of `AP::fence()` non-null.
    pub fence_present: bool,
    /// Leftover of `fence->get_enabled_fences()`.
    pub fence_enabled: u8,
    /// Leftover of `fence->get_breaches()`.
    pub fence_breaches: u8,
    /// Leftover of `ahrs.get_relative_position_NE_home` (metres).
    pub position_ne_home_m: Option<Vector2f>,
    /// Leftover of `ahrs.get_relative_position_NE_origin_float` (metres).
    pub position_ne_origin_m: Option<Vector2f>,
    /// Leftover of `fence->get_radius_m()` (classic circle fence).
    pub circle_radius_m: f32,
    /// Leftover of `fence->get_margin_ne_m()`.
    pub margin_ne_m: f32,
    /// One inclusion polygon. `None` / 0 points = none.
    pub inclusion_polygon: Option<FencePolygon>,
    /// One exclusion polygon.
    pub exclusion_polygon: Option<FencePolygon>,
    /// One inclusion circle (polyfence).
    pub inclusion_circle: Option<FenceCircle>,
    /// One exclusion circle (polyfence).
    pub exclusion_circle: Option<FenceCircle>,
    /// Leftover of `AP::beacon()` non-null.
    pub beacon_present: bool,
    /// Leftover of `beacon->get_boundary_points`.
    pub beacon_boundary: Option<FencePolygon>,
}

impl Default for FenceNeContext {
    fn default() -> Self {
        Self {
            fence_present: false,
            fence_enabled: 0,
            fence_breaches: 0,
            position_ne_home_m: None,
            position_ne_origin_m: None,
            circle_radius_m: 0.0,
            margin_ne_m: 0.0,
            inclusion_polygon: None,
            exclusion_polygon: None,
            inclusion_circle: None,
            exclusion_circle: None,
            beacon_present: false,
            beacon_boundary: None,
        }
    }
}

/// Leftover of one `AC_Avoid::adjust_velocity_fence` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityFenceLeftover {
    /// Desired NEU velocity after fence NE + vertical Z, cm/s.
    pub desired_vel_neu_cms: Vector3f,
    /// Combined fence backup (NE quadrants + Z), cm/s.
    pub backup_vel_neu_cms: Vector3f,
    /// Classic circle fence changed the NE velocity.
    pub circle_limited: bool,
    /// Inclusion / exclusion polygon changed the NE velocity.
    pub polygon_limited: bool,
    /// Inclusion / exclusion circle changed the NE velocity.
    pub poly_circle_limited: bool,
    /// Beacon fence changed the NE velocity.
    pub beacon_limited: bool,
    /// Vertical floor limit was armed.
    pub limit_min_alt: bool,
    /// Vertical ceiling limit was armed.
    pub limit_max_alt: bool,
}

impl Avoid {
    /// Horizontal + vertical fence leftover, upstream `adjust_velocity_fence`.
    ///
    /// Circle / polygon / polyfence circles / beacon NE, then
    /// [`Avoid::adjust_velocity_z`]. Accel-jerk limiting is `limit_accel_neu_cm`.
    #[must_use]
    pub fn adjust_velocity_fence(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_neu_cms: Vector3f,
        k_p_z: f32,
        accel_z_cmss: f32,
        dt: f32,
        fence_ne: FenceNeContext,
        vertical: AdjustVelocityZContext,
    ) -> AdjustVelocityFenceLeftover {
        let mut desired_ne = Vector2f::new(desired_vel_neu_cms.x, desired_vel_neu_cms.y);
        let accel_limited_cmss = accel_cmss.min(ACCEL_CMSS_MAX);

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();
        let mut circle_limited = false;
        let mut polygon_limited = false;
        let mut poly_circle_limited = false;
        let mut beacon_limited = false;

        if (self.enabled_bits() & STOP_AT_FENCE) > 0 && fence_ne.fence_present {
            let before = desired_ne;
            let backup = self.adjust_velocity_circle_fence(
                k_p,
                accel_limited_cmss,
                &mut desired_ne,
                dt,
                fence_ne,
            );
            circle_limited = desired_ne != before;
            Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);

            let before = desired_ne;
            let backup = self.adjust_velocity_inclusion_and_exclusion_polygons(
                k_p,
                accel_limited_cmss,
                &mut desired_ne,
                dt,
                fence_ne,
            );
            polygon_limited = desired_ne != before;
            Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);

            let before = desired_ne;
            let backup = self.adjust_velocity_inclusion_circles(
                k_p,
                accel_limited_cmss,
                &mut desired_ne,
                dt,
                fence_ne,
            );
            Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);

            let backup = self.adjust_velocity_exclusion_circles(
                k_p,
                accel_limited_cmss,
                &mut desired_ne,
                dt,
                fence_ne,
            );
            poly_circle_limited = desired_ne != before;
            Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);
        }

        if (self.enabled_bits() & STOP_AT_BEACON_FENCE) > 0 {
            let before = desired_ne;
            let backup = self.adjust_velocity_beacon_fence(
                k_p,
                accel_limited_cmss,
                &mut desired_ne,
                dt,
                fence_ne,
            );
            beacon_limited = desired_ne != before;
            Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);
        }

        let z = self.adjust_velocity_z(k_p_z, accel_z_cmss, desired_vel_neu_cms.z, dt, vertical);
        let backup_ne = q1 + q2 + q3 + q4;
        AdjustVelocityFenceLeftover {
            desired_vel_neu_cms: Vector3f::new(desired_ne.x, desired_ne.y, z.climb_rate_cms),
            backup_vel_neu_cms: Vector3f::new(backup_ne.x, backup_ne.y, z.backup_speed_cms),
            circle_limited,
            polygon_limited,
            poly_circle_limited,
            beacon_limited,
            limit_min_alt: z.limit_min_alt,
            limit_max_alt: z.limit_max_alt,
        }
    }

    /// Classic home-centered circle, upstream `adjust_velocity_circle_fence`.
    #[must_use]
    pub fn adjust_velocity_circle_fence(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        dt: f32,
        ctx: FenceNeContext,
    ) -> Vector2f {
        if !ctx.fence_present || (ctx.fence_enabled & TYPE_CIRCLE) == 0 {
            return Vector2f::zero();
        }
        if (ctx.fence_breaches & TYPE_CIRCLE) != 0 {
            return Vector2f::zero();
        }
        let desired_speed_cms = desired_vel_ne_cms.length();
        if is_zero(desired_speed_cms) {
            return Vector2f::zero();
        }
        let Some(position_ne_m) = ctx.position_ne_home_m else {
            return Vector2f::zero();
        };
        let position_ne_cm = position_ne_m * 100.0;
        let fence_radius_cm = ctx.circle_radius_m * 100.0;
        let margin_cm = ctx.margin_ne_m * 100.0;
        if margin_cm > fence_radius_cm {
            return Vector2f::zero();
        }
        let dist_from_home_cm = position_ne_cm.length();
        if dist_from_home_cm > fence_radius_cm {
            return Vector2f::zero();
        }
        let distance_to_boundary_cm = fence_radius_cm - dist_from_home_cm;

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();
        if is_negative(distance_to_boundary_cm - margin_cm) {
            calc_backup_velocity_2d(
                k_p,
                accel_cmss,
                &mut q1,
                &mut q2,
                &mut q3,
                &mut q4,
                margin_cm - distance_to_boundary_cm,
                position_ne_cm,
                dt,
            );
        }
        let backup = q1 + q2 + q3 + q4;

        if self.behavior() == BEHAVIOR_STOP {
            let stopping_point_plus_margin_ne_cm = position_ne_cm
                + *desired_vel_ne_cms
                    * ((2.0
                        + margin_cm
                        + Self::get_stopping_distance(k_p, accel_cmss, desired_speed_cms))
                        / desired_speed_cms);
            let stopping_dist = stopping_point_plus_margin_ne_cm.length();
            if dist_from_home_cm >= fence_radius_cm - margin_cm {
                if stopping_dist >= dist_from_home_cm {
                    *desired_vel_ne_cms = Vector2f::zero();
                }
            } else if let Some(intersection) = circle_segment_intersection(
                position_ne_cm,
                stopping_point_plus_margin_ne_cm,
                Vector2f::zero(),
                fence_radius_cm - margin_cm,
            ) {
                let distance_to_target_cm = (intersection - position_ne_cm).length();
                let max_speed_cms = Self::get_max_speed(k_p, accel_cmss, distance_to_target_cm, dt);
                if max_speed_cms < desired_speed_cms {
                    *desired_vel_ne_cms *= max_speed_cms.max(0.0) / desired_speed_cms;
                }
            }
        } else {
            let stopping_point_ne_cm = position_ne_cm
                + *desired_vel_ne_cms
                    * (Self::get_stopping_distance(k_p, accel_cmss, desired_speed_cms)
                        / desired_speed_cms);
            let stopping_dist = stopping_point_ne_cm.length();
            if stopping_dist <= fence_radius_cm - margin_cm {
                return backup;
            }
            let target_offset_ne_cm =
                stopping_point_ne_cm * ((fence_radius_cm - margin_cm) / stopping_dist);
            let target_direction_ne_cm = target_offset_ne_cm - position_ne_cm;
            let distance_to_target_cm = target_direction_ne_cm.length();
            if is_positive(distance_to_target_cm) {
                let max_speed_cms = Self::get_max_speed(k_p, accel_cmss, distance_to_target_cm, dt);
                *desired_vel_ne_cms = target_direction_ne_cm
                    * (desired_speed_cms.min(max_speed_cms) / distance_to_target_cm);
            }
        }
        backup
    }

    /// Inclusion then exclusion polygons, upstream
    /// `adjust_velocity_inclusion_and_exclusion_polygons`.
    #[must_use]
    pub fn adjust_velocity_inclusion_and_exclusion_polygons(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        dt: f32,
        ctx: FenceNeContext,
    ) -> Vector2f {
        if !ctx.fence_present || (ctx.fence_enabled & TYPE_POLYGON) == 0 {
            return Vector2f::zero();
        }
        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();
        if let Some(poly) = ctx.inclusion_polygon {
            if poly.num_points > 0 {
                let backup = self.adjust_velocity_polygon(
                    k_p,
                    accel_cmss,
                    desired_vel_ne_cms,
                    poly.as_slice(),
                    ctx.margin_ne_m,
                    dt,
                    true,
                    ctx.position_ne_origin_m,
                );
                Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);
            }
        }
        if let Some(poly) = ctx.exclusion_polygon {
            if poly.num_points > 0 {
                let backup = self.adjust_velocity_polygon(
                    k_p,
                    accel_cmss,
                    desired_vel_ne_cms,
                    poly.as_slice(),
                    ctx.margin_ne_m,
                    dt,
                    false,
                    ctx.position_ne_origin_m,
                );
                Self::find_max_quadrant_velocity(backup, &mut q1, &mut q2, &mut q3, &mut q4);
            }
        }
        q1 + q2 + q3 + q4
    }

    /// Inclusion circles, upstream `adjust_velocity_inclusion_circles`.
    #[must_use]
    pub fn adjust_velocity_inclusion_circles(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        dt: f32,
        ctx: FenceNeContext,
    ) -> Vector2f {
        let Some(circle) = ctx.inclusion_circle else {
            return Vector2f::zero();
        };
        if !ctx.fence_present || (ctx.fence_enabled & TYPE_POLYGON) == 0 {
            return Vector2f::zero();
        }
        let Some(position_ne_m) = ctx.position_ne_origin_m else {
            return Vector2f::zero();
        };
        let position_ne_cm = position_ne_m * 100.0;
        let margin_cm = ctx.margin_ne_m * 100.0;
        let desired_speed_cms = desired_vel_ne_cms.length();

        let mut stopping_offset_ne_cm = Vector2f::zero();
        if !is_zero(desired_speed_cms) {
            stopping_offset_ne_cm = if self.behavior() == BEHAVIOR_STOP {
                *desired_vel_ne_cms
                    * ((2.0
                        + margin_cm
                        + Self::get_stopping_distance(k_p, accel_cmss, desired_speed_cms))
                        / desired_speed_cms)
            } else {
                *desired_vel_ne_cms
                    * (Self::get_stopping_distance(k_p, accel_cmss, desired_speed_cms)
                        / desired_speed_cms)
            };
        }

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();

        let position_rel_ne_cm = position_ne_cm - circle.center_ne_cm;
        let dist_sq_cm = position_rel_ne_cm.length_squared();
        let radius_cm = circle.radius_m * 100.0;
        if dist_sq_cm > sq(radius_cm) {
            return Vector2f::zero();
        }
        let radius_with_margin_cm = radius_cm - margin_cm;
        if is_negative(radius_with_margin_cm) {
            return Vector2f::zero();
        }
        let margin_breach_cm = radius_with_margin_cm - safe_sqrt(dist_sq_cm);
        if is_negative(margin_breach_cm) {
            calc_backup_velocity_2d(
                k_p,
                accel_cmss,
                &mut q1,
                &mut q2,
                &mut q3,
                &mut q4,
                margin_breach_cm,
                position_rel_ne_cm,
                dt,
            );
        }
        if !is_zero(desired_speed_cms) {
            if self.behavior() == BEHAVIOR_STOP {
                let stopping_point = position_rel_ne_cm + stopping_offset_ne_cm;
                let dist_cm = safe_sqrt(dist_sq_cm);
                if dist_cm >= radius_cm - margin_cm {
                    if stopping_point.length() >= dist_cm {
                        *desired_vel_ne_cms = Vector2f::zero();
                        return q1 + q2 + q3 + q4;
                    }
                } else if let Some(intersection) = circle_segment_intersection(
                    position_rel_ne_cm,
                    stopping_point,
                    Vector2f::zero(),
                    radius_cm - margin_cm,
                ) {
                    let distance_to_target_cm = (intersection - position_rel_ne_cm).length();
                    let max_speed_cms =
                        Self::get_max_speed(k_p, accel_cmss, distance_to_target_cm, dt);
                    if max_speed_cms < desired_speed_cms {
                        *desired_vel_ne_cms *= max_speed_cms.max(0.0) / desired_speed_cms;
                    }
                }
            } else {
                let stopping_point = position_rel_ne_cm + stopping_offset_ne_cm;
                let stopping_dist = stopping_point.length();
                if !is_zero(stopping_dist) && stopping_dist > (radius_cm - margin_cm) {
                    let target_offset = stopping_point * ((radius_cm - margin_cm) / stopping_dist);
                    let target_direction = target_offset - position_rel_ne_cm;
                    let distance_to_target_cm = target_direction.length();
                    if is_positive(distance_to_target_cm) {
                        let max_speed_cms =
                            Self::get_max_speed(k_p, accel_cmss, distance_to_target_cm, dt);
                        *desired_vel_ne_cms = target_direction
                            * (desired_speed_cms.min(max_speed_cms) / distance_to_target_cm);
                    }
                }
            }
        }
        q1 + q2 + q3 + q4
    }

    /// Exclusion circles, upstream `adjust_velocity_exclusion_circles`.
    #[must_use]
    pub fn adjust_velocity_exclusion_circles(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        dt: f32,
        ctx: FenceNeContext,
    ) -> Vector2f {
        let Some(circle) = ctx.exclusion_circle else {
            return Vector2f::zero();
        };
        if !ctx.fence_present || (ctx.fence_enabled & TYPE_POLYGON) == 0 {
            return Vector2f::zero();
        }
        let Some(position_ne_m) = ctx.position_ne_origin_m else {
            return Vector2f::zero();
        };
        let position_ne_cm = position_ne_m * 100.0;
        let margin_cm = ctx.margin_ne_m * 100.0;
        let desired_speed_cms = desired_vel_ne_cms.length();

        let mut stopping_offset_ne_cm = Vector2f::zero();
        if !is_zero(desired_speed_cms) && self.behavior() == BEHAVIOR_STOP {
            stopping_offset_ne_cm = *desired_vel_ne_cms
                * ((2.0
                    + margin_cm
                    + Self::get_stopping_distance(k_p, accel_cmss, desired_speed_cms))
                    / desired_speed_cms);
        }

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();

        let position_rel_ne_cm = position_ne_cm - circle.center_ne_cm;
        let dist_sq_cm = position_rel_ne_cm.length_squared();
        let radius_cm = circle.radius_m * 100.0;
        if radius_cm < margin_cm {
            return Vector2f::zero();
        }
        if dist_sq_cm < sq(radius_cm) {
            return Vector2f::zero();
        }

        let vector_to_center_ne_cm = circle.center_ne_cm - position_ne_cm;
        let dist_to_boundary_cm = vector_to_center_ne_cm.length() - radius_cm;
        if is_negative(dist_to_boundary_cm - margin_cm) {
            calc_backup_velocity_2d(
                k_p,
                accel_cmss,
                &mut q1,
                &mut q2,
                &mut q3,
                &mut q4,
                margin_cm - dist_to_boundary_cm,
                vector_to_center_ne_cm,
                dt,
            );
        }
        if !is_zero(desired_speed_cms) {
            if self.behavior() == BEHAVIOR_STOP {
                let stopping_point = position_rel_ne_cm + stopping_offset_ne_cm;
                let dist_cm = safe_sqrt(dist_sq_cm);
                if dist_cm < radius_cm + margin_cm {
                    if stopping_point.length() <= dist_cm {
                        *desired_vel_ne_cms = Vector2f::zero();
                        return q1 + q2 + q3 + q4;
                    }
                } else if let Some(intersection) = circle_segment_intersection(
                    position_rel_ne_cm,
                    stopping_point,
                    Vector2f::zero(),
                    radius_cm + margin_cm,
                ) {
                    let distance_to_target_cm = (intersection - position_rel_ne_cm).length();
                    let max_speed_cms =
                        Self::get_max_speed(k_p, accel_cmss, distance_to_target_cm, dt);
                    if max_speed_cms < desired_speed_cms {
                        *desired_vel_ne_cms *= max_speed_cms.max(0.0) / desired_speed_cms;
                    }
                }
            } else {
                let mut limit_direction = vector_to_center_ne_cm;
                if !limit_direction.is_zero() {
                    let limit_distance_cm = limit_direction.length() - radius_cm;
                    if is_positive(limit_distance_cm) {
                        if let Some(dir) = limit_direction.normalized() {
                            limit_direction = dir;
                            *desired_vel_ne_cms = Self::limit_velocity_ne(
                                k_p,
                                accel_cmss,
                                *desired_vel_ne_cms,
                                limit_direction,
                                (limit_distance_cm - margin_cm).max(0.0),
                                dt,
                            );
                        }
                    }
                }
            }
        }
        q1 + q2 + q3 + q4
    }

    /// Beacon perimeter, upstream `adjust_velocity_beacon_fence`.
    #[must_use]
    pub fn adjust_velocity_beacon_fence(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        dt: f32,
        ctx: FenceNeContext,
    ) -> Vector2f {
        if !ctx.beacon_present {
            return Vector2f::zero();
        }
        let Some(boundary) = ctx.beacon_boundary else {
            return Vector2f::zero();
        };
        if boundary.num_points == 0 {
            return Vector2f::zero();
        }
        self.adjust_velocity_polygon(
            k_p,
            accel_cmss,
            desired_vel_ne_cms,
            boundary.as_slice(),
            ctx.margin_ne_m,
            dt,
            true,
            ctx.position_ne_origin_m,
        )
    }

    /// Polygon leftover, upstream `adjust_velocity_polygon`.
    ///
    /// `boundary_ne_cm` is earth-frame centimetres. `stay_inside` is true
    /// for inclusion / beacon, false for exclusion.
    #[must_use]
    pub fn adjust_velocity_polygon(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: &mut Vector2f,
        boundary_ne_cm: &[Vector2f],
        margin_m: f32,
        dt: f32,
        stay_inside: bool,
        position_ne_origin_m: Option<Vector2f>,
    ) -> Vector2f {
        if boundary_ne_cm.is_empty() {
            return Vector2f::zero();
        }
        let Some(position_ne_m) = position_ne_origin_m else {
            return Vector2f::zero();
        };
        let position_ne_cm = position_ne_m * 100.0;
        let inside = !polygon_outside(position_ne_cm, boundary_ne_cm);
        if inside != stay_inside {
            return Vector2f::zero();
        }

        let mut safe_vel = *desired_vel_ne_cms;
        let margin_cm = (margin_m * 100.0).max(0.0);
        let speed = safe_vel.length();
        let mut stopping_point_plus_margin = Vector2f::zero();
        if !desired_vel_ne_cms.is_zero() && !is_zero(speed) {
            stopping_point_plus_margin = position_ne_cm
                + safe_vel
                    * ((2.0 + margin_cm + Self::get_stopping_distance(k_p, accel_cmss, speed))
                        / speed);
        }

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();
        let n = boundary_ne_cm.len();
        for i in 0..n {
            let j = if i + 1 >= n { 0 } else { i + 1 };
            let start = boundary_ne_cm[j];
            let end = boundary_ne_cm[i];
            let vector_to_boundary =
                Vector2f::closest_point(position_ne_cm, start, end) - position_ne_cm;
            if is_negative(vector_to_boundary.length() - margin_cm) {
                calc_backup_velocity_2d(
                    k_p,
                    accel_cmss,
                    &mut q1,
                    &mut q2,
                    &mut q3,
                    &mut q4,
                    margin_cm - vector_to_boundary.length(),
                    vector_to_boundary,
                    dt,
                );
            }
            if desired_vel_ne_cms.is_zero() {
                continue;
            }
            if self.behavior() == BEHAVIOR_STOP {
                if let Some(intersection) = Vector2f::segment_intersection(
                    position_ne_cm,
                    stopping_point_plus_margin,
                    start,
                    end,
                ) {
                    let mut limit_direction = intersection - position_ne_cm;
                    let limit_distance_cm = limit_direction.length();
                    if is_zero(limit_distance_cm) {
                        return q1 + q2 + q3 + q4;
                    }
                    if limit_distance_cm <= margin_cm {
                        safe_vel = Vector2f::zero();
                    } else if let Some(dir) = limit_direction.normalized() {
                        limit_direction = dir;
                        safe_vel = Self::limit_velocity_ne(
                            k_p,
                            accel_cmss,
                            safe_vel,
                            limit_direction,
                            (limit_distance_cm - margin_cm).max(0.0),
                            dt,
                        );
                    }
                }
            } else {
                let mut limit_direction = vector_to_boundary;
                let limit_distance_cm = limit_direction.length();
                if is_zero(limit_distance_cm) {
                    return q1 + q2 + q3 + q4;
                }
                if let Some(dir) = limit_direction.normalized() {
                    limit_direction = dir;
                    safe_vel = Self::limit_velocity_ne(
                        k_p,
                        accel_cmss,
                        safe_vel,
                        limit_direction,
                        (limit_distance_cm - margin_cm).max(0.0),
                        dt,
                    );
                }
            }
        }
        *desired_vel_ne_cms = safe_vel;
        q1 + q2 + q3 + q4
    }
}

/// Upstream `AC_Avoid::calc_backup_velocity_2D`.
fn calc_backup_velocity_2d(
    k_p: f32,
    accel_cmss: f32,
    quad1: &mut Vector2f,
    quad2: &mut Vector2f,
    quad3: &mut Vector2f,
    quad4: &mut Vector2f,
    back_distance_cm: f32,
    limit_direction: Vector2f,
    dt: f32,
) {
    if limit_direction.is_zero() {
        return;
    }
    let Some(dir) = limit_direction.normalized() else {
        return;
    };
    let back_speed_cms = Avoid::get_max_speed(k_p, 0.4 * accel_cmss, back_distance_cm.abs(), dt);
    Avoid::find_max_quadrant_velocity(dir * (-back_speed_cms), quad1, quad2, quad3, quad4);
}

/// Upstream `Vector2f::circle_segment_intersection`.
fn circle_segment_intersection(
    seg_start: Vector2f,
    seg_end: Vector2f,
    circle_center: Vector2f,
    radius: f32,
) -> Option<Vector2f> {
    let seg_start_local = seg_start - circle_center;
    let d = seg_end - seg_start;
    let a = sq(d.x) + sq(d.y);
    let b = 2.0 * (d.x * seg_start_local.x + d.y * seg_start_local.y);
    let c = sq(seg_start_local.x) + sq(seg_start_local.y) - sq(radius);
    if is_zero(a) || a.is_nan() || b.is_nan() || c.is_nan() {
        return None;
    }
    let delta = sq(b) - 4.0 * a * c;
    if delta.is_nan() || delta < 0.0 {
        return None;
    }
    let delta_sqrt = safe_sqrt(delta);
    let t1 = (-b + delta_sqrt) / (2.0 * a);
    let t2 = (-b - delta_sqrt) / (2.0 * a);
    if (0.0..=1.0).contains(&t1) {
        return Some(seg_start + d * t1);
    }
    if (0.0..=1.0).contains(&t2) {
        return Some(seg_start + d * t2);
    }
    None
}
