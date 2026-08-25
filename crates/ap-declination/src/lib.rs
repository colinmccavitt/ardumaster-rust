//! World magnetic model lookup, upstream `libraries/AP_Declination`. FW-014.
//!
//! A compass measures the field, not north. The two differ by the magnetic
//! declination, which varies from a fraction of a degree over most of the
//! temperate world to more than ninety degrees near the poles — so a vehicle
//! that ignored it would fly a heading that was wrong by an amount depending
//! on where it was.
//!
//! The model is a 19×37 grid at ten-degree spacing, sampled from IGRF, with
//! bilinear interpolation between the corners. Ten degrees is coarse for a
//! field that varies as sharply as this one does near the poles, which is why
//! [`get_mag_field_ef`] reports whether the query was inside the table at all.
//!
//! # Three quantities, one lookup
//!
//! Declination is the angle from true north. Inclination is the dip below
//! horizontal — near vertical at the poles, which is what makes a compass
//! useless there. Intensity is the field strength, and a reading far from it
//! is how a vehicle notices interference from its own wiring.

#![no_std]

pub mod tables;

use ap_math::scalar::Real;
use tables::{
    cell, DECLINATION_TABLE, INCLINATION_TABLE, INTENSITY_TABLE, LAT_TABLE_SIZE, LON_TABLE_SIZE,
    SAMPLING_MAX_LAT, SAMPLING_MAX_LON, SAMPLING_MIN_LAT, SAMPLING_MIN_LON, SAMPLING_RES,
};

/// The magnetic field at a point on the earth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagField {
    /// Field strength, gauss.
    pub intensity_gauss: f32,
    /// Angle from true north to magnetic north, degrees east.
    pub declination_deg: f32,
    /// Angle below horizontal, degrees.
    pub inclination_deg: f32,
}

/// Whether the query fell inside the sampled table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Coverage {
    /// Inside the table; the interpolation used four real corners.
    Interpolated,
    /// On or past an edge. Upstream still returns numbers, computed from the
    /// nearest cell it can clamp to, and reports `false` — the caller decides
    /// whether an extrapolated field is good enough.
    OutsideTable,
}

/// The magnetic field at a latitude and longitude, upstream
/// `get_mag_field_ef`.
///
/// Returns the field and whether the point was inside the table. Upstream
/// returns the field through out-parameters and the coverage as its return
/// value, and callers routinely ignore the latter; making it part of the
/// answer means it has to be looked at.
#[must_use]
pub fn get_mag_field_ef(latitude_deg: f32, longitude_deg: f32) -> (MagField, Coverage) {
    let mut coverage = Coverage::Interpolated;

    // Round down to the sampling grid. Upstream casts through int32 twice and
    // says why: an implicit int32-to-float conversion is undefined on some
    // platforms, so it is made explicit.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream's explicit int32 casts, reproduced; the values are \
degrees and cannot approach the int32 range"
    )]
    let mut min_lat = ((Real::floor(latitude_deg / SAMPLING_RES) as i32) as f32) * SAMPLING_RES;
    #[allow(clippy::cast_possible_truncation, reason = "as above")]
    let mut min_lon = ((Real::floor(longitude_deg / SAMPLING_RES) as i32) as f32) * SAMPLING_RES;

    // The bounds are enforced separately, because hitting one exactly lands
    // the rounding on a cell whose north or east neighbour does not exist.
    if latitude_deg <= SAMPLING_MIN_LAT {
        min_lat = SAMPLING_MIN_LAT;
        coverage = Coverage::OutsideTable;
    }
    if latitude_deg >= SAMPLING_MAX_LAT {
        #[allow(clippy::cast_possible_truncation, reason = "upstream's cast")]
        let stepped = ((latitude_deg / SAMPLING_RES) as i32) as f32 * SAMPLING_RES - SAMPLING_RES;
        min_lat = stepped;
        coverage = Coverage::OutsideTable;
    }
    if longitude_deg <= SAMPLING_MIN_LON {
        min_lon = SAMPLING_MIN_LON;
        coverage = Coverage::OutsideTable;
    }
    if longitude_deg >= SAMPLING_MAX_LON {
        #[allow(clippy::cast_possible_truncation, reason = "upstream's cast")]
        let stepped = ((longitude_deg / SAMPLING_RES) as i32) as f32 * SAMPLING_RES - SAMPLING_RES;
        min_lon = stepped;
        coverage = Coverage::OutsideTable;
    }

    // Index of the south-west corner. Clamped two short of the end so the
    // north and east neighbours always exist.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "upstream casts to uint32 then constrains; the constrain is what \
makes the index safe and it is reproduced"
    )]
    // `clamp` rather than ap_math's constrain_value, which is float-only.
    // Upstream uses `constrain_int32`, which is the same operation.
    let min_lat_index = (((-SAMPLING_MIN_LAT + min_lat) / SAMPLING_RES) as i32)
        .clamp(0, LAT_TABLE_SIZE as i32 - 2) as usize;
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "as above"
    )]
    let min_lon_index = (((-SAMPLING_MIN_LON + min_lon) / SAMPLING_RES) as i32)
        .clamp(0, LON_TABLE_SIZE as i32 - 2) as usize;

    let lon_frac = (longitude_deg - min_lon) / SAMPLING_RES;
    let lat_frac = (latitude_deg - min_lat) / SAMPLING_RES;

    let interpolate = |table: &[[u32; LON_TABLE_SIZE]; LAT_TABLE_SIZE]| -> f32 {
        let sw = cell(table, min_lat_index, min_lon_index);
        let se = cell(table, min_lat_index, min_lon_index + 1);
        let ne = cell(table, min_lat_index + 1, min_lon_index + 1);
        let nw = cell(table, min_lat_index + 1, min_lon_index);

        let data_min = lon_frac * (se - sw) + sw;
        let data_max = lon_frac * (ne - nw) + nw;
        lat_frac * (data_max - data_min) + data_min
    };

    (
        MagField {
            intensity_gauss: interpolate(&INTENSITY_TABLE),
            declination_deg: interpolate(&DECLINATION_TABLE),
            inclination_deg: interpolate(&INCLINATION_TABLE),
        },
        coverage,
    )
}

/// Declination alone, upstream `get_declination`.
///
/// Upstream discards the coverage flag here, so a query well outside the table
/// is indistinguishable from one inside it. Reproduced: callers that need to
/// know use [`get_mag_field_ef`], which is what upstream expects of them too.
#[must_use]
pub fn get_declination(latitude_deg: f32, longitude_deg: f32) -> f32 {
    get_mag_field_ef(latitude_deg, longitude_deg)
        .0
        .declination_deg
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "at an exact grid point both interpolation fractions are zero, so \nthe answer is the table entry itself and nothing else; an epsilon would hide a table \nthat had drifted"
    )]

    use super::*;

    /// Declination is near zero along the agonic line and grows away from it.
    /// These are places, and the numbers are what the model says about them.
    #[test]
    fn known_places_have_the_declination_they_should() {
        // London: a little west of true north.
        let d = get_declination(51.5, -0.13);
        assert!((-3.0..2.0).contains(&d), "London: {d}");

        // Canberra: strongly east.
        let d = get_declination(-35.3, 149.1);
        assert!((10.0..15.0).contains(&d), "Canberra: {d}");

        // Seattle: strongly east.
        let d = get_declination(47.6, -122.3);
        assert!((13.0..18.0).contains(&d), "Seattle: {d}");
    }

    /// Inclination is near zero at the magnetic equator, steeply positive in
    /// the northern hemisphere and negative in the southern.
    #[test]
    fn inclination_follows_the_hemisphere() {
        let (north, _) = get_mag_field_ef(60.0, 0.0);
        let (south, _) = get_mag_field_ef(-60.0, 0.0);
        assert!(
            north.inclination_deg > 60.0,
            "north: {}",
            north.inclination_deg
        );
        assert!(
            south.inclination_deg < -50.0,
            "south: {}",
            south.inclination_deg
        );
    }

    /// Field strength is lowest over the South Atlantic anomaly and highest
    /// near the poles. Every sample should be a plausible terrestrial field.
    #[test]
    fn intensity_is_everywhere_plausible() {
        let mut lowest = f32::INFINITY;
        let mut highest = 0.0_f32;
        let mut lat = -85.0_f32;
        while lat <= 85.0 {
            let mut lon = -175.0_f32;
            while lon <= 175.0 {
                let (f, _) = get_mag_field_ef(lat, lon);
                lowest = lowest.min(f.intensity_gauss);
                highest = highest.max(f.intensity_gauss);
                lon += 5.0;
            }
            lat += 5.0;
        }
        assert!(lowest > 0.1, "weakest field {lowest} gauss");
        assert!(highest < 0.7, "strongest field {highest} gauss");
    }

    /// A grid point returns the table value itself, with no interpolation
    /// error — the fractions are both zero there.
    #[test]
    fn a_grid_point_returns_the_table_value() {
        // 0 N, 0 E is lat index 9, lon index 18.
        let (f, _) = get_mag_field_ef(0.0, 0.0);
        assert_eq!(f.declination_deg, cell(&DECLINATION_TABLE, 9, 18));
        assert_eq!(f.inclination_deg, cell(&INCLINATION_TABLE, 9, 18));
        assert_eq!(f.intensity_gauss, cell(&INTENSITY_TABLE, 9, 18));
    }

    /// Halfway between two grid points is halfway between their values.
    #[test]
    fn interpolation_is_linear_between_grid_points() {
        let a = get_declination(0.0, 0.0);
        let b = get_declination(0.0, 10.0);
        let mid = get_declination(0.0, 5.0);
        assert!(
            (mid - (a + b) * 0.5).abs() < 1e-3,
            "{a} and {b} should average to {mid}"
        );
    }

    /// Past the edges of the table the answer is still a number, and the
    /// coverage says it was extrapolated. Upstream's `get_declination`
    /// discards that flag, which is why it is part of the other function's
    /// return.
    #[test]
    fn queries_outside_the_table_are_reported() {
        for (lat, lon) in [
            (-90.0_f32, 0.0_f32),
            (90.0, 0.0),
            (0.0, -180.0),
            (0.0, 180.0),
        ] {
            let (f, coverage) = get_mag_field_ef(lat, lon);
            assert_eq!(coverage, Coverage::OutsideTable, "at {lat}, {lon}");
            assert!(
                f.declination_deg.is_finite(),
                "still a number at {lat}, {lon}"
            );
        }

        let (_, coverage) = get_mag_field_ef(45.0, 45.0);
        assert_eq!(coverage, Coverage::Interpolated);
    }

    /// The bounds are exactly where they claim to be: one step inside is
    /// covered, the boundary itself is not.
    #[test]
    fn the_boundary_is_where_it_says_it_is() {
        assert_eq!(get_mag_field_ef(89.9, 0.0).1, Coverage::Interpolated);
        assert_eq!(get_mag_field_ef(90.0, 0.0).1, Coverage::OutsideTable);
        assert_eq!(get_mag_field_ef(0.0, 179.9).1, Coverage::Interpolated);
        assert_eq!(get_mag_field_ef(0.0, 180.0).1, Coverage::OutsideTable);
    }

    /// Nothing anywhere on the globe produces a NaN or an infinity, including
    /// the poles and the dateline.
    #[test]
    fn the_whole_globe_is_finite() {
        let mut lat = -90.0_f32;
        while lat <= 90.0 {
            let mut lon = -180.0_f32;
            while lon <= 180.0 {
                let (f, _) = get_mag_field_ef(lat, lon);
                assert!(
                    f.declination_deg.is_finite()
                        && f.inclination_deg.is_finite()
                        && f.intensity_gauss.is_finite(),
                    "at {lat}, {lon}"
                );
                lon += 2.5;
            }
            lat += 2.5;
        }
    }
}
