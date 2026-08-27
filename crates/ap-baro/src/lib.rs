//! The 1976 US Standard Atmosphere, upstream `AP_Baro_atmosphere.cpp`.
//!
//! A barometer measures pressure. Everything the vehicle wants from it —
//! altitude, air density, the equivalent-to-true airspeed ratio — comes from
//! inverting a model of how pressure varies with height. This is that model.
//!
//! # Why a table of layers
//!
//! The atmosphere's temperature does not fall monotonically with height: it
//! drops through the troposphere, holds constant through the lower
//! stratosphere, rises again above 20 km, and so on. The 1976 model captures
//! that as a table of layers, each with a base altitude, temperature, pressure,
//! density and a temperature lapse rate. Within a layer the pressure relation
//! is a closed form — exponential where the lapse rate is zero, a power law
//! where it is not.
//!
//! # Geometric against geopotential altitude
//!
//! The model is defined in *geopotential* altitude, which folds in the fact
//! that gravity weakens with height, so that a constant `g` can be used in the
//! equations. What a vehicle wants is *geometric* altitude — actual metres.
//! The two differ by about 20 m at 11 km and the conversion is exact, so every
//! entry point here converts at its boundary and the tables stay geopotential.
//!
//! # The simple model, and why both exist
//!
//! [`altitude_difference_simple`] is a single exponential valid to about 11 km,
//! and it uses the *measured* ground temperature rather than the standard one.
//! That makes it more accurate near the ground on a day that is not standard,
//! and useless above the troposphere. Upstream keeps both and picks with a
//! build flag; SITL builds the full model.

#![no_std]

pub mod sitl;
pub mod frontend;

use ap_math::scalar::{constrain_value, is_positive, is_zero, Real};

/// Earth's radius as the 1976 model defines it, metres.
///
/// Deliberately not the value in `AP_Math/definitions.h` — upstream notes the
/// model's constants differ slightly from the ones used elsewhere, and mixing
/// them would put the layer boundaries in the wrong place.
pub const RADIUS_EARTH: f32 = 6_356.766E3;

/// Specific gas constant for air in the 1976 model, J/kg/K.
///
/// Written with upstream's digits rather than the shortest spelling of the
/// same f32, so the constant is traceable to the source it came from.
#[allow(
    clippy::excessive_precision,
    reason = "copied verbatim from AP_Baro_atmosphere.cpp; the extra digits round to the identical f32 and document the provenance"
)]
pub const R_SPECIFIC: f32 = 287.053_072;

/// Standard gravity, m/s², upstream `GRAVITY_MSS`.
pub const GRAVITY_MSS: f32 = 9.806_65;

/// Sea-level standard air density, kg/m³, upstream `SSL_AIR_DENSITY`.
pub const SSL_AIR_DENSITY: f32 = 1.225;

/// Temperature lapse rate used by the simple model, K/m, upstream
/// `ISA_LAPSE_RATE`.
pub const ISA_LAPSE_RATE: f32 = 0.0065;

/// Gas constant used by the simple model, upstream `ISA_GAS_CONSTANT`.
pub const ISA_GAS_CONSTANT: f32 = 287.26;

/// Celsius to Kelvin, upstream `C_TO_KELVIN`.
#[must_use]
pub fn c_to_kelvin(temp_c: f32) -> f32 {
    temp_c + 273.15
}

/// One layer of the standard atmosphere.
#[derive(Debug, Clone, Copy)]
pub struct Layer {
    /// Geopotential height of the layer's base above mean sea level, metres.
    pub amsl_m: f32,
    /// Temperature at the base, K.
    pub temp_k: f32,
    /// Pressure at the base, Pa.
    pub pressure_pa: f32,
    /// Density at the base, kg/m³.
    pub density: f32,
    /// Temperature gradient through the layer, K/m. Zero means iso-thermal.
    pub temp_lapse: f32,
}

/// The 1976 model's layer table, upstream `atmospheric_1976_consts`.
///
/// The first entry starts at −5000 m: the tables extend below sea level using
/// the same equations as the 0–11 km layer, so a vehicle in Death Valley or a
/// pressure setting well above standard still lands inside the table.
pub const ATMOSPHERE_1976: [Layer; 8] = [
    Layer {
        amsl_m: -5000.0,
        temp_k: 320.650,
        pressure_pa: 177_687.0,
        density: 1.930_467,
        temp_lapse: -6.5E-3,
    },
    Layer {
        amsl_m: 11000.0,
        temp_k: 216.650,
        pressure_pa: 22_632.1,
        density: 0.363_918,
        temp_lapse: 0.0,
    },
    Layer {
        amsl_m: 20000.0,
        temp_k: 216.650,
        pressure_pa: 5_474.89,
        density: 8.803_49E-2,
        temp_lapse: 1E-3,
    },
    Layer {
        amsl_m: 32000.0,
        temp_k: 228.650,
        pressure_pa: 868.019,
        density: 1.322_50E-2,
        temp_lapse: 2.8E-3,
    },
    Layer {
        amsl_m: 47000.0,
        temp_k: 270.650,
        pressure_pa: 110.906,
        density: 1.427_53E-3,
        temp_lapse: 0.0,
    },
    Layer {
        amsl_m: 51000.0,
        temp_k: 270.650,
        pressure_pa: 66.9389,
        density: 8.616_06E-4,
        temp_lapse: -2.8E-3,
    },
    Layer {
        amsl_m: 71000.0,
        temp_k: 214.650,
        pressure_pa: 3.956_42,
        density: 6.421_10E-5,
        temp_lapse: -2.0E-3,
    },
    Layer {
        amsl_m: 84852.0,
        temp_k: 186.946,
        pressure_pa: 0.373_38,
        density: 6.957_88E-6,
        temp_lapse: 0.0,
    },
];

/// The layer containing a geopotential altitude, upstream
/// `find_atmosphere_layer_by_altitude`.
///
/// Above the table's top the last layer is returned rather than an error;
/// there is nothing better to say, and a vehicle up there has bigger problems.
///
/// (Upstream's comment says this "returns at least 1". It returns `idx - 1`,
/// so the floor is actually 0. The behaviour is right and the comment is not.)
#[must_use]
pub fn find_layer_by_altitude(alt_m: f32) -> usize {
    for idx in 1..ATMOSPHERE_1976.len() {
        if let Some(layer) = ATMOSPHERE_1976.get(idx) {
            if alt_m < layer.amsl_m {
                return idx - 1;
            }
        }
    }
    ATMOSPHERE_1976.len() - 1
}

/// The layer containing a pressure, upstream
/// `find_atmosphere_layer_by_pressure`.
#[must_use]
pub fn find_layer_by_pressure(pressure: f32) -> usize {
    for idx in 1..ATMOSPHERE_1976.len() {
        if let Some(layer) = ATMOSPHERE_1976.get(idx) {
            if layer.pressure_pa < pressure {
                return idx - 1;
            }
        }
    }
    ATMOSPHERE_1976.len() - 1
}

fn layer(idx: usize) -> Layer {
    // The finders never return an out-of-range index, and a caller supplying
    // one gets the top layer rather than a panic.
    match ATMOSPHERE_1976.get(idx) {
        Some(l) => *l,
        None => Layer {
            amsl_m: 0.0,
            temp_k: 0.0,
            pressure_pa: 0.0,
            density: 0.0,
            temp_lapse: 0.0,
        },
    }
}

/// Geopotential to geometric altitude, upstream
/// `geopotential_alt_to_geometric`.
#[must_use]
pub fn geopotential_to_geometric(alt: f32) -> f32 {
    (RADIUS_EARTH * alt) / (RADIUS_EARTH - alt)
}

/// Geometric to geopotential altitude, upstream
/// `geometric_alt_to_geopotential`.
#[must_use]
pub fn geometric_to_geopotential(alt: f32) -> f32 {
    (RADIUS_EARTH * alt) / (RADIUS_EARTH + alt)
}

/// Temperature at a geopotential altitude within a known layer, upstream
/// `get_temperature_by_altitude_layer`.
#[must_use]
pub fn temperature_by_altitude_layer(alt: f32, idx: usize) -> f32 {
    let l = layer(idx);
    if is_zero(l.temp_lapse) {
        return l.temp_k;
    }
    l.temp_k + l.temp_lapse * (alt - l.amsl_m)
}

/// Standard temperature at a geometric altitude, K. Upstream
/// `get_temperature_from_altitude`.
#[must_use]
pub fn temperature_from_altitude(alt: f32) -> f32 {
    let alt = geometric_to_geopotential(alt);
    let idx = find_layer_by_altitude(alt);
    temperature_by_altitude_layer(alt, idx)
}

/// Geometric altitude for a pressure, upstream `get_altitude_from_pressure`.
///
/// `None` where upstream raises an internal error and falls back to its last
/// known altitude — a pressure of zero or below, which cannot come from a
/// working sensor. Returning the fallback here would need the barometer's
/// state, and hiding a fault inside a pure function is worse than reporting
/// it.
#[must_use]
pub fn altitude_from_pressure(pressure: f32) -> Option<f32> {
    let idx = find_layer_by_pressure(pressure);
    let l = layer(idx);
    let pressure_ratio = pressure / l.pressure_pa;

    if !is_positive(pressure_ratio) {
        return None;
    }

    let alt = if is_zero(l.temp_lapse) {
        // Iso-thermal: pressure falls exponentially with height.
        let fac = -(l.temp_k * R_SPECIFIC) / GRAVITY_MSS;
        l.amsl_m + fac * Real::log(pressure_ratio)
    } else {
        // Gradient layer: a power law in the pressure ratio.
        let fac = -(l.temp_lapse * R_SPECIFIC) / GRAVITY_MSS;
        l.amsl_m + (l.temp_k / l.temp_lapse) * (Real::powf(pressure_ratio, fac) - 1.0)
    };

    Some(geopotential_to_geometric(alt))
}

/// Standard pressure (Pa) and temperature (K) at a geometric altitude,
/// upstream `get_pressure_temperature_for_alt_amsl`.
#[must_use]
pub fn pressure_temperature_for_alt_amsl(alt_amsl: f32) -> (f32, f32) {
    let alt_amsl = geometric_to_geopotential(alt_amsl);
    let idx = find_layer_by_altitude(alt_amsl);
    let l = layer(idx);
    let temperature_k = temperature_by_altitude_layer(alt_amsl, idx);

    let pressure = if is_zero(l.temp_lapse) {
        let fac = Real::exp(-GRAVITY_MSS / (temperature_k * R_SPECIFIC) * (alt_amsl - l.amsl_m));
        l.pressure_pa * fac
    } else {
        let fac = GRAVITY_MSS / (l.temp_lapse * R_SPECIFIC);
        let temp_ratio = temperature_k / l.temp_k;
        l.pressure_pa * Real::powf(temp_ratio, -fac)
    };

    (pressure, temperature_k)
}

/// Air density at a geometric altitude, kg/m³. Upstream
/// `get_air_density_for_alt_amsl`.
#[must_use]
pub fn air_density_for_alt_amsl(alt_amsl: f32) -> f32 {
    let alt_amsl = geometric_to_geopotential(alt_amsl);
    let idx = find_layer_by_altitude(alt_amsl);
    let l = layer(idx);
    let temp = temperature_by_altitude_layer(alt_amsl, idx);

    if is_zero(l.temp_lapse) {
        let fac = Real::exp(-GRAVITY_MSS / (temp * R_SPECIFIC) * (alt_amsl - l.amsl_m));
        l.density * fac
    } else {
        let fac = GRAVITY_MSS / (l.temp_lapse * R_SPECIFIC);
        let temp_ratio = temp / l.temp_k;
        l.density * Real::powf(temp_ratio, -(fac + 1.0))
    }
}

/// Equivalent-to-true airspeed ratio at a geometric altitude, full model.
/// Upstream `get_EAS2TAS_extended`.
///
/// True airspeed exceeds equivalent airspeed as density falls, which is why a
/// vehicle climbing at constant indicated airspeed is going faster and faster
/// over the ground.
#[must_use]
pub fn eas2tas_extended(altitude: f32) -> f32 {
    let mut density = air_density_for_alt_amsl(altitude);
    if !is_positive(density) {
        // Past the top of the table. Hold the thinnest density it knows about
        // rather than divide by nothing.
        density = layer(ATMOSPHERE_1976.len() - 1).density;
    }
    Real::sqrt(SSL_AIR_DENSITY / density)
}

/// Equivalent-to-true airspeed ratio at a geometric altitude, as SITL uses it.
/// Upstream `get_EAS2TAS_for_alt_amsl`.
#[must_use]
pub fn eas2tas_for_alt_amsl(alt_amsl: f32) -> f32 {
    let density = air_density_for_alt_amsl(alt_amsl);
    let floor = 0.00001_f32;
    let d = if density > floor { density } else { floor };
    Real::sqrt(SSL_AIR_DENSITY / d)
}

/// Geometric altitude difference between two pressures, full model. Upstream
/// `get_altitude_difference`.
#[must_use]
pub fn altitude_difference(base_pressure: f32, pressure: f32) -> Option<f32> {
    let alt1 = altitude_from_pressure(base_pressure)?;
    let alt2 = altitude_from_pressure(pressure)?;
    Some(alt2 - alt1)
}

/// Altitude difference by the simple exponential model, upstream
/// `get_altitude_difference_simple`.
///
/// Good to about ±2.5 m against the standard tables through the troposphere,
/// and it uses the *measured* ground temperature, so on a hot or cold day it
/// beats the standard model near the ground. Above 11 km it is wrong.
#[must_use]
pub fn altitude_difference_simple(base_pressure: f32, pressure: f32, ground_temp_c: f32) -> f32 {
    let temp_k = c_to_kelvin(ground_temp_c);
    let scaling = pressure / base_pressure;
    153.846_2 * temp_k * (1.0 - Real::exp(0.190_259 * Real::log(scaling)))
}

/// Equivalent-to-true airspeed ratio from the simple model, upstream
/// `get_EAS2TAS_simple`.
///
/// Estimates the lapse only from the difference to the ground station rather
/// than modelling a whole atmosphere, which upstream notes gives a more
/// consistent reading.
#[must_use]
pub fn eas2tas_simple(altitude: f32, pressure: f32, ground_temp_c: f32) -> f32 {
    if is_zero(pressure) {
        return 1.0;
    }
    let temp_k = c_to_kelvin(ground_temp_c) - ISA_LAPSE_RATE * altitude;
    let eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * temp_k));
    if !is_positive(eas2tas_squared) {
        return 1.0;
    }
    Real::sqrt(eas2tas_squared)
}

/// Sea-level pressure that would put `pressure` at `altitude`, upstream
/// `get_sealevel_pressure`.
///
/// Solved numerically rather than in closed form: the layer equations are easy
/// to evaluate forward and awkward to invert across a layer boundary, so
/// upstream runs a secant iteration instead. It converges in about five steps
/// and is capped at twenty so a pathological input cannot eat the loop budget.
#[must_use]
pub fn sealevel_pressure(pressure: f32, altitude: f32) -> f32 {
    const MIN_PRESSURE: f32 = 0.01;
    const MAX_PRESSURE: f32 = 1e6;
    const DELTA: f32 = 0.1;

    let mut p0 = pressure;
    for _ in 0..20 {
        let Some(err1) = altitude_difference(p0, pressure).map(|d| d - altitude) else {
            break;
        };
        let Some(err2) = altitude_difference(p0 + DELTA, pressure).map(|d| d - altitude) else {
            break;
        };
        let dalt = err2 - err1;
        if err1.abs() < 0.01 {
            break;
        }
        p0 -= err1 * DELTA / dalt;
        p0 = constrain_value(p0, MIN_PRESSURE, MAX_PRESSURE);
    }
    p0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Sea level, standard day. The numbers everything else is calibrated
    /// against.
    #[test]
    fn sea_level_matches_the_standard_day() {
        let (p, t) = pressure_temperature_for_alt_amsl(0.0);
        assert!((p - 101_325.0).abs() < 1.0, "pressure {p}");
        assert!((t - 288.15).abs() < 0.01, "temperature {t}");
        assert!((air_density_for_alt_amsl(0.0) - 1.225).abs() < 1.0e-3);
        assert!((eas2tas_extended(0.0) - 1.0).abs() < 1.0e-3);
    }

    /// Pressure to altitude and back is the round trip the barometer actually
    /// performs.
    #[test]
    fn pressure_and_altitude_round_trip() {
        for alt in [
            -400.0_f32, 0.0, 500.0, 2000.0, 8000.0, 10_000.0, 18_000.0, 30_000.0,
        ] {
            let (p, _) = pressure_temperature_for_alt_amsl(alt);
            let back = altitude_from_pressure(p).expect("a real pressure");
            assert!(
                (back - alt).abs() < 1.0,
                "{alt} m gave {p} Pa which read back as {back} m"
            );
        }
    }

    /// The layer boundaries are where a model like this goes wrong, so cross
    /// each one.
    #[test]
    fn the_round_trip_holds_across_every_layer_boundary() {
        for l in &ATMOSPHERE_1976 {
            for offset in [-100.0_f32, 100.0] {
                let geometric = geopotential_to_geometric(l.amsl_m + offset);
                if !(-5000.0..80_000.0).contains(&geometric) {
                    continue;
                }
                let (p, _) = pressure_temperature_for_alt_amsl(geometric);
                let back = altitude_from_pressure(p).expect("a real pressure");
                assert!(
                    (back - geometric).abs() < 2.0,
                    "near the {} m boundary: {geometric} m -> {p} Pa -> {back} m",
                    l.amsl_m
                );
            }
        }
    }

    /// Temperature falls through the troposphere, holds through the lower
    /// stratosphere, and rises again above 20 km. If the layer lookup were
    /// off by one this would not hold.
    #[test]
    fn the_temperature_profile_has_the_right_shape() {
        let t0 = temperature_from_altitude(0.0);
        let t10 = temperature_from_altitude(10_000.0);
        let t15 = temperature_from_altitude(15_000.0);
        let t18 = temperature_from_altitude(18_000.0);
        let t30 = temperature_from_altitude(30_000.0);

        assert!(t10 < t0, "troposphere should cool: {t0} then {t10}");
        assert!(
            (t15 - t18).abs() < 0.5,
            "lower stratosphere should be iso-thermal: {t15} then {t18}"
        );
        assert!(
            t30 > t18,
            "above 20 km it should warm again: {t18} then {t30}"
        );
    }

    /// Density falls monotonically even where temperature does not.
    #[test]
    fn density_falls_monotonically_with_height() {
        let mut last = f32::INFINITY;
        for alt in (0..40_000).step_by(1000) {
            let d = air_density_for_alt_amsl(alt as f32);
            assert!(d < last, "density rose at {alt} m: {d} after {last}");
            last = d;
        }
    }

    /// Geopotential and geometric altitude differ by about 20 m at 11 km, and
    /// the conversion is an exact inverse.
    #[test]
    fn the_altitude_conversions_invert_each_other() {
        for alt in [0.0_f32, 1000.0, 11_000.0, 30_000.0, 80_000.0] {
            let round = geopotential_to_geometric(geometric_to_geopotential(alt));
            assert!((round - alt).abs() < 0.1, "{alt} came back as {round}");
        }
        let diff = geopotential_to_geometric(11_000.0) - 11_000.0;
        assert!(
            (15.0..25.0).contains(&diff),
            "expected about 20 m of difference at 11 km, got {diff}"
        );
    }

    /// True airspeed exceeds equivalent airspeed as the air thins — the reason
    /// this ratio exists.
    #[test]
    fn eas2tas_grows_with_altitude() {
        let sea = eas2tas_extended(0.0);
        let high = eas2tas_extended(10_000.0);
        assert!((sea - 1.0).abs() < 1.0e-3, "unity at sea level, got {sea}");
        assert!(
            high > 1.5,
            "at 10 km true airspeed should be well above equivalent, got {high}"
        );
    }

    /// The simple model agrees with the full one through the troposphere on a
    /// standard day, which is the claim upstream makes for it.
    #[test]
    fn the_simple_model_tracks_the_full_one_low_down() {
        let base = 101_325.0_f32;
        for alt in [100.0_f32, 500.0, 2000.0, 5000.0, 10_000.0] {
            let (p, _) = pressure_temperature_for_alt_amsl(alt);
            let full = altitude_difference(base, p).expect("real pressures");
            let simple = altitude_difference_simple(base, p, 15.0);
            assert!(
                (full - simple).abs() < 0.02 * alt + 3.0,
                "at {alt} m: full {full}, simple {simple}"
            );
        }
    }

    /// A pressure that cannot come from a working sensor is reported rather
    /// than turned into an altitude.
    #[test]
    fn a_nonsensical_pressure_has_no_altitude() {
        assert!(altitude_from_pressure(0.0).is_none());
        assert!(altitude_from_pressure(-100.0).is_none());
    }

    /// The sea-level pressure solver inverts the forward model.
    #[test]
    fn the_sealevel_solver_inverts_the_model() {
        for alt in [0.0_f32, 500.0, 2000.0, 8000.0] {
            let (p, _) = pressure_temperature_for_alt_amsl(alt);
            let p0 = sealevel_pressure(p, alt);
            assert!(
                (p0 - 101_325.0).abs() < 50.0,
                "at {alt} m the solver gave {p0} Pa, expected about 101325"
            );
        }
    }

    /// A pressure setting well above standard puts the vehicle below the
    /// table's zero, which is why the table starts at -5000 m.
    #[test]
    fn below_sea_level_stays_inside_the_table() {
        assert_eq!(find_layer_by_altitude(-4000.0), 0);
        let (p, _) = pressure_temperature_for_alt_amsl(-400.0);
        assert!(
            p > 101_325.0,
            "below sea level pressure should exceed standard, got {p}"
        );
        assert!(altitude_from_pressure(p).is_some());
    }
}
