//! GPS blending weights and blended state, upstream `AP_GPS_Blended`. FW-012.
//!
//! Implements the inverse-variance weighting from two receivers and produces a
//! virtual blended [`GpsStatus`] for SITL dual-GPS stub consumers.

use ap_math::scalar::{degrees, wrap_360, Real};
use ap_math::vector3::Vector3f;

use crate::status::GpsStatus;
use crate::FixType;

/// Maximum GPS receivers, upstream `GPS_MAX_RECEIVERS`.
pub const GPS_MAX_RECEIVERS: usize = 2;

/// Blended virtual instance index, upstream `GPS_BLENDED_INSTANCE`.
pub const GPS_BLENDED_INSTANCE: u8 = 2;

/// Use horizontal position accuracy in blend weights.
pub const BLEND_MASK_USE_HPOS_ACC: u8 = 1;
/// Use vertical position accuracy in blend weights.
pub const BLEND_MASK_USE_VPOS_ACC: u8 = 2;
/// Use speed accuracy in blend weights.
pub const BLEND_MASK_USE_SPD_ACC: u8 = 4;

/// Default blend mask from param table, upstream `GPS_BLEND_MASK` default 5.
pub const GPS_BLEND_MASK_DEFAULT: u8 = BLEND_MASK_USE_HPOS_ACC | BLEND_MASK_USE_VPOS_ACC;

const BLEND_COUNTER_FAILURE_INCREMENT: u8 = 10;

/// GPS auto-switch mode, upstream `GPSAutoSwitch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GpsAutoSwitch {
    #[default]
    UsePrimary = 0,
    UseBest = 1,
    Blend = 2,
}

impl GpsAutoSwitch {
    #[must_use]
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::UsePrimary,
            1 => Self::UseBest,
            2 => Self::Blend,
            _ => return None,
        })
    }
}

/// Reported accuracy for one receiver, upstream GPA accuracy fields.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GpsBlendAccuracy {
    pub horizontal_m: f32,
    pub vertical_m: f32,
    pub speed_mps: f32,
    pub have_horizontal: bool,
    pub have_vertical: bool,
    pub have_speed: bool,
}

impl GpsBlendAccuracy {
    /// SITL stub accuracy scaled from satellite count.
    #[must_use]
    pub fn sitl_from_sats(num_sats: u8) -> Self {
        let ns = num_sats.max(1) as f32;
        Self {
            horizontal_m: 4.0 / ns,
            vertical_m: 6.0 / ns,
            speed_mps: 0.8 / ns,
            have_horizontal: true,
            have_vertical: true,
            have_speed: true,
        }
    }
}

/// One GPS instance input to the blender.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsBlendInstance {
    pub status: GpsStatus,
    pub accuracy: GpsBlendAccuracy,
    pub rate_ms: u32,
}

impl GpsBlendInstance {
    #[must_use]
    pub fn from_status(status: GpsStatus) -> Self {
        Self {
            accuracy: GpsBlendAccuracy::sitl_from_sats(status.num_sats),
            status,
            rate_ms: crate::sitl::SITL_GPS_UPDATE_MS,
        }
    }
}

/// Blending state machine, upstream `AP_GPS_Blended`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsBlender {
    blend_mask: u8,
    blend_health_counter: u8,
    weights: [f32; GPS_MAX_RECEIVERS],
    last_gps_time_ms: u32,
}

impl Default for GpsBlender {
    fn default() -> Self {
        Self {
            blend_mask: GPS_BLEND_MASK_DEFAULT,
            blend_health_counter: 0,
            weights: [0.0; GPS_MAX_RECEIVERS],
            last_gps_time_ms: 0,
        }
    }
}

impl GpsBlender {
    #[must_use]
    pub fn new(blend_mask: u8) -> Self {
        Self {
            blend_mask,
            ..Self::default()
        }
    }

    #[must_use]
    pub const fn weights(&self) -> &[f32; GPS_MAX_RECEIVERS] {
        &self.weights
    }

    #[must_use]
    pub const fn blend_mask(&self) -> u8 {
        self.blend_mask
    }

    #[must_use]
    pub const fn output_is_blended(&self) -> bool {
        self.weights[0] > 0.0 || self.weights[1] > 0.0
    }

    fn calc_weights_inner(&mut self, instances: &[GpsBlendInstance; GPS_MAX_RECEIVERS]) -> bool {
        if !instances[0].status.have_fix || !instances[1].status.have_fix {
            return false;
        }
        if instances[0].status.fix_type < FixType::Fix3D
            || instances[1].status.fix_type < FixType::Fix3D
        {
            return false;
        }

        let mut max_ms = 0_u32;
        let mut min_ms = u32::MAX;
        let mut max_rate_ms = 0_u32;
        for inst in instances {
            let t = inst.status.last_fix_time_ms;
            if t > max_ms {
                max_ms = t;
            }
            if t > 0 && t < min_ms {
                min_ms = t;
            }
            max_rate_ms = max_rate_ms.max(inst.rate_ms);
        }
        if min_ms == u32::MAX {
            return false;
        }
        if max_ms.wrapping_sub(min_ms) < 2 * max_rate_ms {
            self.last_gps_time_ms = min_ms;
        } else {
            return false;
        }

        let mut speed_accuracy_sum_sq = 0.0_f32;
        if self.blend_mask & BLEND_MASK_USE_SPD_ACC != 0 {
            for inst in instances {
                if inst.status.fix_type >= FixType::Fix3D {
                    if inst.accuracy.have_speed && inst.accuracy.speed_mps > 0.0 {
                        speed_accuracy_sum_sq += inst.accuracy.speed_mps * inst.accuracy.speed_mps;
                    } else {
                        speed_accuracy_sum_sq = 0.0;
                        break;
                    }
                }
            }
        }

        let mut horizontal_accuracy_sum_sq = 0.0_f32;
        if self.blend_mask & BLEND_MASK_USE_HPOS_ACC != 0 {
            for inst in instances {
                if inst.status.fix_type >= FixType::Fix2D {
                    if inst.accuracy.have_horizontal && inst.accuracy.horizontal_m > 0.0 {
                        horizontal_accuracy_sum_sq +=
                            inst.accuracy.horizontal_m * inst.accuracy.horizontal_m;
                    } else {
                        horizontal_accuracy_sum_sq = 0.0;
                        break;
                    }
                }
            }
        }

        let mut vertical_accuracy_sum_sq = 0.0_f32;
        if self.blend_mask & BLEND_MASK_USE_VPOS_ACC != 0 {
            for inst in instances {
                if inst.status.fix_type >= FixType::Fix3D {
                    if inst.accuracy.have_vertical && inst.accuracy.vertical_m > 0.0 {
                        vertical_accuracy_sum_sq +=
                            inst.accuracy.vertical_m * inst.accuracy.vertical_m;
                    } else {
                        vertical_accuracy_sum_sq = 0.0;
                        break;
                    }
                }
            }
        }

        let can_do_blending = horizontal_accuracy_sum_sq > 0.0
            || vertical_accuracy_sum_sq > 0.0
            || speed_accuracy_sum_sq > 0.0;
        if !can_do_blending {
            return false;
        }

        let mut sum_of_all_weights = 0.0_f32;
        let mut hpos_blend_weights = [0.0_f32; GPS_MAX_RECEIVERS];
        if horizontal_accuracy_sum_sq > 0.0 {
            let mut sum_of_hpos_weights = 0.0_f32;
            for (i, inst) in instances.iter().enumerate() {
                if inst.status.fix_type >= FixType::Fix2D && inst.accuracy.horizontal_m >= 0.001 {
                    hpos_blend_weights[i] =
                        horizontal_accuracy_sum_sq / (inst.accuracy.horizontal_m * inst.accuracy.horizontal_m);
                    sum_of_hpos_weights += hpos_blend_weights[i];
                }
            }
            if sum_of_hpos_weights > 0.0 {
                for w in &mut hpos_blend_weights {
                    *w /= sum_of_hpos_weights;
                }
                sum_of_all_weights += 1.0;
            }
        }

        let mut vpos_blend_weights = [0.0_f32; GPS_MAX_RECEIVERS];
        if vertical_accuracy_sum_sq > 0.0 {
            let mut sum_of_vpos_weights = 0.0_f32;
            for (i, inst) in instances.iter().enumerate() {
                if inst.status.fix_type >= FixType::Fix3D && inst.accuracy.vertical_m >= 0.001 {
                    vpos_blend_weights[i] =
                        vertical_accuracy_sum_sq / (inst.accuracy.vertical_m * inst.accuracy.vertical_m);
                    sum_of_vpos_weights += vpos_blend_weights[i];
                }
            }
            if sum_of_vpos_weights > 0.0 {
                for w in &mut vpos_blend_weights {
                    *w /= sum_of_vpos_weights;
                }
                sum_of_all_weights += 1.0;
            }
        }

        let mut spd_blend_weights = [0.0_f32; GPS_MAX_RECEIVERS];
        if speed_accuracy_sum_sq > 0.0 {
            let mut sum_of_spd_weights = 0.0_f32;
            for (i, inst) in instances.iter().enumerate() {
                if inst.status.fix_type >= FixType::Fix3D && inst.accuracy.speed_mps >= 0.001 {
                    spd_blend_weights[i] =
                        speed_accuracy_sum_sq / (inst.accuracy.speed_mps * inst.accuracy.speed_mps);
                    sum_of_spd_weights += spd_blend_weights[i];
                }
            }
            if sum_of_spd_weights > 0.0 {
                for w in &mut spd_blend_weights {
                    *w /= sum_of_spd_weights;
                }
                sum_of_all_weights += 1.0;
            }
        }

        if sum_of_all_weights <= 0.0 {
            return false;
        }

        for i in 0..GPS_MAX_RECEIVERS {
            self.weights[i] =
                (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
        }
        true
    }

    /// Calculate blend weights, upstream `AP_GPS_Blended::calc_weights`.
    #[must_use]
    pub fn calc_weights(&mut self, instances: &[GpsBlendInstance; GPS_MAX_RECEIVERS]) -> bool {
        if !self.calc_weights_inner(instances) {
            self.blend_health_counter = self
                .blend_health_counter
                .saturating_add(BLEND_COUNTER_FAILURE_INCREMENT)
                .min(100);
        } else if self.blend_health_counter > 0 {
            self.blend_health_counter -= 1;
        }

        let non_zero = self.weights.iter().any(|&w| w > 0.0);
        if !non_zero {
            return false;
        }
        self.blend_health_counter < 50
    }

    /// Produce blended status from instances and current weights.
    #[must_use]
    pub fn calc_state(&self, instances: &[GpsBlendInstance; GPS_MAX_RECEIVERS]) -> GpsStatus {
        let mut fix_type = FixType::NoFix;
        let mut velocity = Vector3f::zero();
        let mut num_sats = 0_u8;
        let mut lat = 0.0_f32;
        let mut lon = 0.0_f32;
        let mut alt = 0.0_f32;
        let mut lag_sec = 0.0_f32;
        let mut best_weight = 0.0_f32;
        let mut best_index = 0_usize;

        for (i, inst) in instances.iter().enumerate() {
            if inst.status.fix_type > fix_type {
                fix_type = inst.status.fix_type;
            }
            let w = self.weights[i];
            velocity += inst.status.velocity_ned * w;
            num_sats = num_sats.max(inst.status.num_sats);
            lat += inst.status.latitude_deg * w;
            lon += inst.status.longitude_deg * w;
            alt += inst.status.altitude_m * w;
            lag_sec += inst.status.lag_sec * w;
            if w > best_weight {
                best_weight = w;
                best_index = i;
            }
        }

        let ground_speed = velocity.xy().length();
        let ground_course_deg = wrap_360(degrees(Real::atan2(velocity.y, velocity.x)));

        GpsStatus {
            fix_type,
            num_sats,
            have_fix: fix_type >= FixType::Fix2D,
            lag_sec,
            velocity_ned: velocity,
            ground_speed,
            ground_course_deg,
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            last_fix_time_ms: instances[best_index].status.last_fix_time_ms,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;

    fn sample_status(lat: f32, vel_x: f32, sats: u8, _h_acc: f32) -> GpsStatus {
        GpsStatus {
            fix_type: FixType::Fix3D,
            num_sats: sats,
            have_fix: true,
            lag_sec: 0.1,
            velocity_ned: Vector3f::new(vel_x, 0.0, 0.0),
            ground_speed: vel_x.abs(),
            ground_course_deg: 0.0,
            latitude_deg: lat,
            longitude_deg: -0.1,
            altitude_m: 100.0,
            last_fix_time_ms: 400,
        }
    }

    fn sample_instance(lat: f32, vel_x: f32, sats: u8, h_acc: f32) -> GpsBlendInstance {
        GpsBlendInstance {
            status: sample_status(lat, vel_x, sats, h_acc),
            accuracy: GpsBlendAccuracy {
                horizontal_m: h_acc,
                vertical_m: 2.0,
                speed_mps: 0.5,
                have_horizontal: true,
                have_vertical: true,
                have_speed: true,
            },
            rate_ms: 200,
        }
    }

    #[test]
    fn blend_weights_favor_tighter_horizontal_accuracy() {
        let instances = [
            sample_instance(51.0, 10.0, 12, 2.0),
            sample_instance(51.0001, 8.0, 12, 1.0),
        ];
        let mut blender = GpsBlender::default();
        assert!(blender.calc_weights(&instances));
        assert!(blender.weights()[1] > blender.weights()[0]);
    }

    #[test]
    fn blended_velocity_is_weighted_average() {
        let instances = [
            sample_instance(51.0, 10.0, 12, 1.0),
            sample_instance(51.0, 6.0, 12, 1.0),
        ];
        let mut blender = GpsBlender::default();
        assert!(blender.calc_weights(&instances));
        let blended = blender.calc_state(&instances);
        assert!((blended.velocity_ned.x - 8.0).abs() < 1e-3);
    }

    #[test]
    fn blend_fails_when_secondary_has_no_fix() {
        let mut bad = sample_instance(51.0, 10.0, 12, 1.0);
        bad.status.have_fix = false;
        bad.status.fix_type = FixType::NoFix;
        let instances = [sample_instance(51.0, 10.0, 12, 1.0), bad];
        let mut blender = GpsBlender::default();
        assert!(!blender.calc_weights(&instances));
    }
}
