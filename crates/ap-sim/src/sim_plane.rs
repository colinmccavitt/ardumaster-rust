//! Ground-truth fixed-wing flight dynamics, ported from ArduPilot SITL `SIM_Plane`
//! (Plane-4.7.0 `libraries/SITL/SIM_Plane.{h,cpp}` + `SIM_Aircraft` rigid-body
//! integrator) via the C++ port `fwcpp::sim::SimPlane` (CPP-030 / CPP-051 / CPP-082).
//!
//! STANDARD fixed-wing configuration only. This is the physics oracle SitlHarness
//! closes the loop against -- not the kinematic [`crate::AttitudeSim`].
//!
//! Upstream sources (read in full from the pinned worktree):
//!   - `libraries/SITL/SIM_Plane.h` `Coefficients` / `default_coefficients`
//!     (last_letter skywalker_2013/aerodynamics.yaml, Georacer)
//!   - `libraries/SITL/SIM_Plane.cpp`: `liftCoeff`, `dragCoeff`, `getTorque`,
//!     `getForce`, `calculate_forces`, `update`
//!   - `libraries/SITL/SIM_Aircraft.cpp`: `update_dynamics`, `update_wind`,
//!     `on_ground` / `hagl` (flat-earth simplification as in the C++ port)
//!
//! Deliberately shares no code with the DCM estimator under test. Attitude is
//! integrated with the same first-order `Matrix3::rotate` + renormalize the C++
//! port uses (`Aircraft::update_dynamics`), implemented here in f32 so the plant
//! stays inside `ap-sim` without taking an `ap-math` dependency (the kinematic
//! [`crate::AttitudeSim`] already keeps independent arithmetic).
//!
//! Exclusions matching C++ `sim_plane.hpp` (STANDARD config): elevons/vtail/
//! dspoilers/redundant mixes are available as leftover helpers but the default
//! `update()` path is four-surface; tailsitter/aerobatic alpha adjustment,
//! launcher, ship/tether/slung-payload, and the analog pitot-offset term are
//! not ported.
//!
//! `load_coeffs()` (real upstream `Plane::load_coeffs` / `AP_JSON`
//! `:model.json` frame-string suffix) IS ported and tested below -- an
//! earlier version of this comment claimed JSON model loading was not
//! ported, which was stale even when written. Coverage now includes a real,
//! byte-for-byte copy of upstream's own `Tools/autotest/models/
//! skywalker_2013.json`, the only real native-format Plane coefficient file
//! anywhere in the pinned upstream tree (confirmed by `grep -rl c_lift_a`
//! across the whole pinned tree). `Callisto.json`/`freestyle.json` under the
//! same upstream directory are multicopter frame configs (mass/battery/
//! motor-count fields, not aerodynamic coefficients) and `xplane_plane.json`/
//! `xplane_heli.json` are X-Plane DREF maps for an unrelated external-FDM
//! backend -- none of the three loads via `load_coeffs`, and none should be.
//!
//! The real `-heavy`/`-jet` frame-string mass/`thrust_scale` overrides (real
//! `SIM_Plane.cpp` lines 53-59) are also ported, as
//! [`SimPlane::with_heavy_frame`] / [`SimPlane::with_jet_frame`] rather than
//! a `frame_str` re-parse, matching this file's existing explicit-parameter
//! constructor shape (see [`SimPlane::with_config`]).
//!
//! Atmosphere is held at SSL (C++ sitl_run
//! never calls `update_position()`, so `Aircraft::update_dynamics`'s ISA
//! recompute also stays at home alt = 0). Wind estimate for the vehicle is a
//! harness concern: this plant exposes `wind_ef` truth; SitlHarness must not
//! feed it as an AHRS oracle (C++ SitlHarness leaves wind_estimate at zero).

#![allow(missing_docs)]

use core::f32::consts::PI;

/// Standard gravity, m/s². Upstream `GRAVITY_MSS`.
pub const GRAVITY_MSS: f32 = 9.806_65;
/// Sea-level air density, kg/m³. Upstream `SSL_AIR_DENSITY`.
pub const SSL_AIR_DENSITY: f32 = 1.225;
/// Upstream `AP_Airspeed_Params` default `ARSPD_RATIO` (2.0), shared with the
/// vehicle-side sensor as in C++ `kDefaultAirspeedSensorRatio`.
pub const DEFAULT_AIRSPEED_SENSOR_RATIO: f32 = 2.0;
/// Upstream SITL `ARSPD_RND` default, Pa.
pub const DEFAULT_AIRSPEED_NOISE_PA: f32 = 2.0;
/// Default RNG seed. C++ `SimPlane` uses `20260827u` so a default construct
/// stays deterministic when turbulence/noise is never exercised.
pub const DEFAULT_WIND_RNG_SEED: u32 = 20_260_827;

const FLT_EPSILON: f32 = 1.192_092_9e-7;

fn is_zero(x: f32) -> bool {
    x.abs() < FLT_EPSILON
}

fn constrain(v: f32, min: f32, max: f32) -> f32 {
    v.clamp(min, max)
}

fn radians(deg: f32) -> f32 {
    deg * PI / 180.0
}

fn degrees(rad: f32) -> f32 {
    rad * 180.0 / PI
}

/// 3-vector. Independent of `ap-math` (see module banner).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn dot(self, o: Self) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn scaled(self, k: f32) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }

    pub fn plus(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    pub fn minus(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn is_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
}

/// 3x3 matrix, rows first. Independent of `ap-math`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

impl Default for Mat3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Mat3 {
    pub const fn identity() -> Self {
        Self {
            a: Vec3::new(1.0, 0.0, 0.0),
            b: Vec3::new(0.0, 1.0, 0.0),
            c: Vec3::new(0.0, 0.0, 1.0),
        }
    }

    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        let (sr, cr) = (roll.sin(), roll.cos());
        let (sp, cp) = (pitch.sin(), pitch.cos());
        let (sy, cy) = (yaw.sin(), yaw.cos());
        Self {
            a: Vec3::new(cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy)),
            b: Vec3::new(cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy)),
            c: Vec3::new(-sp, sr * cp, cr * cp),
        }
    }

    pub fn to_euler(self) -> (f32, f32, f32) {
        let pitch = (-self.c.x).clamp(-1.0, 1.0).asin();
        let roll = self.c.y.atan2(self.c.z);
        let yaw = self.b.x.atan2(self.a.x);
        (roll, pitch, yaw)
    }

    pub fn apply(self, v: Vec3) -> Vec3 {
        Vec3::new(self.a.dot(v), self.b.dot(v), self.c.dot(v))
    }

    pub const fn transposed(self) -> Self {
        Self {
            a: Vec3::new(self.a.x, self.b.x, self.c.x),
            b: Vec3::new(self.a.y, self.b.y, self.c.y),
            c: Vec3::new(self.a.z, self.b.z, self.c.z),
        }
    }

    /// First-order row cross, upstream `Matrix3::rotate`.
    pub fn rotate(&mut self, g: Vec3) {
        let delta_a = Vec3::new(
            self.a.y * g.z - self.a.z * g.y,
            self.a.z * g.x - self.a.x * g.z,
            self.a.x * g.y - self.a.y * g.x,
        );
        let delta_b = Vec3::new(
            self.b.y * g.z - self.b.z * g.y,
            self.b.z * g.x - self.b.x * g.z,
            self.b.x * g.y - self.b.y * g.x,
        );
        let delta_c = Vec3::new(
            self.c.y * g.z - self.c.z * g.y,
            self.c.z * g.x - self.c.x * g.z,
            self.c.x * g.y - self.c.y * g.x,
        );
        self.a = self.a.plus(delta_a);
        self.b = self.b.plus(delta_b);
        self.c = self.c.plus(delta_c);
    }

    /// DCM renormalize, upstream `Matrix3::normalize`.
    pub fn normalize(&mut self) {
        let error = self.a.dot(self.b);
        let t0 = self.a.minus(self.b.scaled(0.5 * error));
        let t1 = self.b.minus(self.a.scaled(0.5 * error));
        let t2 = t0.cross(t1);
        let l0 = t0.length();
        let l1 = t1.length();
        let l2 = t2.length();
        if l0 > 0.0 {
            self.a = t0.scaled(1.0 / l0);
        }
        if l1 > 0.0 {
            self.b = t1.scaled(1.0 / l1);
        }
        if l2 > 0.0 {
            self.c = t2.scaled(1.0 / l2);
        }
    }

    pub fn det(self) -> f32 {
        self.a.dot(self.b.cross(self.c))
    }

    pub fn is_nan(self) -> bool {
        self.a.is_nan() || self.b.is_nan() || self.c.is_nan()
    }
}

/// Per-instance LCG. Replaces C++ `std::mt19937` so the plant stays no-dep;
/// sequences will not match libc mt19937, which is fine: sitl_run's FBWA
/// recipe uses zero wind, and airspeed noise is a uniform draw whose exact
/// stream is not asserted.
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u32) -> Self {
        Self {
            state: u64::from(seed) | 0x9E37_79B9_7F4A_7C15,
        }
    }

    fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (self.state >> 32) as u32
    }

    /// Uniform in [0, 1).
    fn next_f32(&mut self) -> f32 {
        (self.next_u32() as f32) / 4294967296.0
    }

    /// Upstream `rand_float()`: uniform [-1, 1].
    fn rand_float(&mut self) -> f32 {
        self.next_f32() * 2.0 - 1.0
    }

    /// Marsaglia polar Box-Muller, matching upstream `Aircraft::rand_normal`.
    fn rand_normal(&mut self) -> f64 {
        loop {
            let u = f64::from(self.next_f32()) * 2.0 - 1.0;
            let v = f64::from(self.next_f32()) * 2.0 - 1.0;
            let s = u * u + v * v;
            if s > 0.0 && s < 1.0 {
                return u * (-2.0 * s.ln() / s).sqrt();
            }
        }
    }
}

/// Upstream `SIM_Plane.h` nested `struct Coefficients` / `default_coefficients`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coefficients {
    pub s: f32,
    pub b: f32,
    pub c: f32,
    pub c_lift_0: f32,
    pub c_lift_deltae: f32,
    pub c_lift_a: f32,
    pub c_lift_q: f32,
    pub mcoeff: f32,
    pub oswald: f32,
    pub alpha_stall: f32,
    pub c_drag_q: f32,
    pub c_drag_deltae: f32,
    pub c_drag_p: f32,
    pub c_y_0: f32,
    pub c_y_b: f32,
    pub c_y_p: f32,
    pub c_y_r: f32,
    pub c_y_deltaa: f32,
    pub c_y_deltar: f32,
    pub c_l_0: f32,
    pub c_l_p: f32,
    pub c_l_b: f32,
    pub c_l_r: f32,
    pub c_l_deltaa: f32,
    pub c_l_deltar: f32,
    pub c_m_0: f32,
    pub c_m_a: f32,
    pub c_m_q: f32,
    pub c_m_deltae: f32,
    pub c_n_0: f32,
    pub c_n_b: f32,
    pub c_n_p: f32,
    pub c_n_r: f32,
    pub c_n_deltaa: f32,
    pub c_n_deltar: f32,
    pub deltaa_max: f32,
    pub deltae_max: f32,
    pub deltar_max: f32,
    /// X CoG offset comment transcribed from upstream: "-0.02 makes the plane
    /// too tail heavy in manual flight. Adjusted to -0.15".
    pub cg_offset: Vec3,
}

impl Default for Coefficients {
    fn default() -> Self {
        Self {
            s: 0.45,
            b: 1.88,
            c: 0.24,
            c_lift_0: 0.56,
            c_lift_deltae: 0.0,
            c_lift_a: 6.9,
            c_lift_q: 0.0,
            mcoeff: 50.0,
            oswald: 0.9,
            alpha_stall: 0.4712,
            c_drag_q: 0.0,
            c_drag_deltae: 0.0,
            c_drag_p: 0.1,
            c_y_0: 0.0,
            c_y_b: -0.98,
            c_y_p: 0.0,
            c_y_r: 0.0,
            c_y_deltaa: 0.0,
            c_y_deltar: -0.2,
            c_l_0: 0.0,
            c_l_p: -1.0,
            c_l_b: -0.12,
            c_l_r: 0.14,
            c_l_deltaa: 0.25,
            c_l_deltar: -0.037,
            c_m_0: 0.045,
            c_m_a: -0.7,
            c_m_q: -20.0,
            c_m_deltae: 1.0,
            c_n_0: 0.0,
            c_n_b: 0.25,
            c_n_p: 0.022,
            c_n_r: -1.0,
            c_n_deltaa: 0.0,
            c_n_deltar: 0.1,
            deltaa_max: 0.3491,
            deltae_max: 0.3491,
            deltar_max: 0.3491,
            cg_offset: Vec3::new(-0.15, 0.0, -0.05),
        }
    }
}

/// Upstream `struct sitl_input` nested `wind`. Direction is meteorological
/// FROM-heading, degrees.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WindConfig {
    pub speed: f32,
    pub direction: f32,
    pub turbulence: f32,
    pub dir_z: f32,
}

/// C++ leftover closer: airframe mix. Default is identity (STANDARD).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AirframeMix {
    #[default]
    Standard = 0,
    Elevons = 1,
    Vtail = 2,
    Dspoilers = 3,
    Redundant = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FrameConfig {
    pub mix: AirframeMix,
    pub reverse_elevator_rudder: bool,
    pub reverse_thrust: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GroundBehavior {
    #[default]
    None = 0,
    NoMovement = 1,
    FwdOnly = 2,
    Tailsitter = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceDeflections {
    pub aileron: f32,
    pub elevator: f32,
    pub rudder: f32,
    pub throttle: f32,
}

/// Ground-truth fixed-wing plant. Upstream `SITL::Plane` STANDARD config.
pub struct SimPlane {
    pub coefficient: Coefficients,
    pub hover_throttle: f32,
    pub mass: f32,
    /// Upstream `SIM_Plane.cpp` real line 47: `(mass * GRAVITY_MSS) /
    /// hover_throttle`. Stored (computed once by [`Self::with_config`] or
    /// explicitly by [`Self::with_jet_frame`]) rather than derived fresh
    /// from `mass` on every [`Self::update`] call -- load-bearing for the
    /// real `-heavy`/`-jet` asymmetry: `-heavy` changes `mass` but real
    /// upstream's own `-heavy` branch does NOT recompute `thrust_scale`,
    /// so this field must be able to go stale relative to `mass`, exactly
    /// as it does in real upstream.
    pub thrust_scale: f32,
    pub wind_config: WindConfig,
    pub frame_config: FrameConfig,
    pub ground_behavior: GroundBehavior,
    pub dcm: Mat3,
    pub gyro: Vec3,
    pub accel_body: Vec3,
    pub velocity_ef: Vec3,
    pub velocity_air_ef: Vec3,
    pub velocity_air_bf: Vec3,
    pub position: Vec3,
    pub wind_ef: Vec3,
    pub airspeed: f32,
    pub airspeed_pitot: f32,
    pub air_density: f32,
    pub eas2tas: f32,
    pub ground_level: f32,
    pub frame_height: f32,
    pub home_alt_m: f32,
    pub angle_of_attack: f32,
    pub beta: f32,
    pub turbulence_azimuth: f32,
    pub turbulence_horizontal_speed: f32,
    pub turbulence_vertical_speed: f32,
    rng: Rng,
}

impl Default for SimPlane {
    fn default() -> Self {
        Self::new()
    }
}

impl SimPlane {
    pub fn new() -> Self {
        Self::with_config(Coefficients::default(), 2.0, 0.7, DEFAULT_WIND_RNG_SEED)
    }

    /// C++ `SimPlane::load_coeffs` / upstream `Plane::load_coeffs` (AP_JSON).
    pub fn load_coeffs(&mut self, model_json: &str) -> bool {
        use crate::sim_json::{json_get_float, json_get_vector3, load_json_file};
        let obj = match load_json_file(std::path::Path::new(model_json)) {
            Ok(v) => v,
            Err(_) => return false,
        };
        json_get_float(&obj, "s", &mut self.coefficient.s);
        json_get_float(&obj, "b", &mut self.coefficient.b);
        json_get_float(&obj, "c", &mut self.coefficient.c);
        json_get_float(&obj, "c_lift_0", &mut self.coefficient.c_lift_0);
        json_get_float(&obj, "c_lift_deltae", &mut self.coefficient.c_lift_deltae);
        json_get_float(&obj, "c_lift_a", &mut self.coefficient.c_lift_a);
        json_get_float(&obj, "c_lift_q", &mut self.coefficient.c_lift_q);
        json_get_float(&obj, "mcoeff", &mut self.coefficient.mcoeff);
        json_get_float(&obj, "oswald", &mut self.coefficient.oswald);
        json_get_float(&obj, "alpha_stall", &mut self.coefficient.alpha_stall);
        json_get_float(&obj, "c_drag_q", &mut self.coefficient.c_drag_q);
        json_get_float(&obj, "c_drag_deltae", &mut self.coefficient.c_drag_deltae);
        json_get_float(&obj, "c_drag_p", &mut self.coefficient.c_drag_p);
        json_get_float(&obj, "c_y_0", &mut self.coefficient.c_y_0);
        json_get_float(&obj, "c_y_b", &mut self.coefficient.c_y_b);
        json_get_float(&obj, "c_y_p", &mut self.coefficient.c_y_p);
        json_get_float(&obj, "c_y_r", &mut self.coefficient.c_y_r);
        json_get_float(&obj, "c_y_deltaa", &mut self.coefficient.c_y_deltaa);
        json_get_float(&obj, "c_y_deltar", &mut self.coefficient.c_y_deltar);
        json_get_float(&obj, "c_l_0", &mut self.coefficient.c_l_0);
        json_get_float(&obj, "c_l_p", &mut self.coefficient.c_l_p);
        json_get_float(&obj, "c_l_b", &mut self.coefficient.c_l_b);
        json_get_float(&obj, "c_l_r", &mut self.coefficient.c_l_r);
        json_get_float(&obj, "c_l_deltaa", &mut self.coefficient.c_l_deltaa);
        json_get_float(&obj, "c_l_deltar", &mut self.coefficient.c_l_deltar);
        json_get_float(&obj, "c_m_0", &mut self.coefficient.c_m_0);
        json_get_float(&obj, "c_m_a", &mut self.coefficient.c_m_a);
        json_get_float(&obj, "c_m_q", &mut self.coefficient.c_m_q);
        json_get_float(&obj, "c_m_deltae", &mut self.coefficient.c_m_deltae);
        json_get_float(&obj, "c_n_0", &mut self.coefficient.c_n_0);
        json_get_float(&obj, "c_n_b", &mut self.coefficient.c_n_b);
        json_get_float(&obj, "c_n_p", &mut self.coefficient.c_n_p);
        json_get_float(&obj, "c_n_r", &mut self.coefficient.c_n_r);
        json_get_float(&obj, "c_n_deltaa", &mut self.coefficient.c_n_deltaa);
        json_get_float(&obj, "c_n_deltar", &mut self.coefficient.c_n_deltar);
        json_get_float(&obj, "deltaa_max", &mut self.coefficient.deltaa_max);
        json_get_float(&obj, "deltae_max", &mut self.coefficient.deltae_max);
        json_get_float(&obj, "deltar_max", &mut self.coefficient.deltar_max);
        // Real upstream key is "CGOffset" (`SIM_Plane.cpp` real line 191:
        // `{ "CGOffset", &coefficient.CGOffset, VarType::VECTOR3F }`,
        // re-verified directly against the pinned source). This function
        // previously looked up "cg" here -- a porting bug that never
        // matched any real upstream coefficient file's key name and was
        // masked by the old synthetic test fixture also (wrongly) using
        // "cg". Fixed so `load_coeffs` genuinely reproduces real upstream
        // content instead of merely parsing arbitrary JSON.
        json_get_vector3(&obj, "CGOffset", &mut self.coefficient.cg_offset);
        true
    }

    /// Real upstream `-heavy` frame-string suffix (`SIM_Plane.cpp` real
    /// lines 53-55): `if (strstr(frame_str, "-heavy")) { mass = 8; }`.
    /// `thrust_scale` is deliberately left UNCHANGED from the default (real
    /// upstream never recomputes it for `-heavy`, unlike `-jet` -- see
    /// [`Self::with_jet_frame`] and the module banner for the asymmetry,
    /// re-verified directly against the pinned upstream source before this
    /// was written).
    pub fn with_heavy_frame() -> Self {
        let mut plane = Self::new();
        plane.mass = 8.0;
        plane
    }

    /// Real upstream `-jet` frame-string suffix (`SIM_Plane.cpp` real lines
    /// 56-59): a 22kg "jet" (upstream's own comment: "level top speed is
    /// 102m/s"), with `thrust_scale` recomputed from the new mass:
    /// `(mass * GRAVITY_MSS) / hover_throttle`. Unlike `-heavy`, upstream
    /// DOES recompute `thrust_scale` here.
    pub fn with_jet_frame() -> Self {
        let mut plane = Self::new();
        plane.mass = 22.0;
        plane.thrust_scale = (plane.mass * GRAVITY_MSS) / plane.hover_throttle;
        plane
    }

    pub fn with_config(
        coeffs: Coefficients,
        mass_kg: f32,
        hover_throttle: f32,
        wind_rng_seed: u32,
    ) -> Self {
        Self {
            coefficient: coeffs,
            hover_throttle,
            mass: mass_kg,
            thrust_scale: (mass_kg * GRAVITY_MSS) / hover_throttle,
            wind_config: WindConfig::default(),
            frame_config: FrameConfig::default(),
            ground_behavior: GroundBehavior::None,
            dcm: Mat3::identity(),
            gyro: Vec3::zero(),
            accel_body: Vec3::zero(),
            velocity_ef: Vec3::zero(),
            velocity_air_ef: Vec3::zero(),
            velocity_air_bf: Vec3::zero(),
            position: Vec3::zero(),
            wind_ef: Vec3::zero(),
            airspeed: 0.0,
            airspeed_pitot: 0.0,
            air_density: SSL_AIR_DENSITY,
            eas2tas: 1.0,
            ground_level: 0.0,
            frame_height: 0.0,
            home_alt_m: 0.0,
            angle_of_attack: 0.0,
            beta: 0.0,
            turbulence_azimuth: 0.0,
            turbulence_horizontal_speed: 0.0,
            turbulence_vertical_speed: 0.0,
            rng: Rng::new(wind_rng_seed),
        }
    }

    /// Height above ground, metres. Flat earth: `hagl = -position.z + home_alt - ground_level - frame_height`.
    pub fn hagl(&self) -> f32 {
        (-self.position.z) + self.home_alt_m - self.ground_level - self.frame_height
    }

    pub fn on_ground(&self) -> bool {
        self.hagl() <= 0.001
    }

    /// Upstream `Plane::liftCoeff`. Sigmoid-blended stall; alpha clamped to
    /// `alpha_stall ± 0.8` to avoid `exp()` overflow.
    pub fn lift_coeff(&self, mut alpha: f32) -> f32 {
        let alpha0 = self.coefficient.alpha_stall;
        let m = self.coefficient.mcoeff;
        let c_lift_0 = self.coefficient.c_lift_0;
        let c_lift_a0 = self.coefficient.c_lift_a;
        let max_alpha_delta = 0.8_f32;
        if alpha - alpha0 > max_alpha_delta {
            alpha = alpha0 + max_alpha_delta;
        } else if alpha0 - alpha > max_alpha_delta {
            alpha = alpha0 - max_alpha_delta;
        }
        let a = f64::from(alpha);
        let a0 = f64::from(alpha0);
        let mm = f64::from(m);
        let sigmoid = (1.0 + (-mm * (a - a0)).exp() + (mm * (a + a0)).exp())
            / (1.0 + (-mm * (a - a0)).exp())
            / (1.0 + (mm * (a + a0)).exp());
        let linear = (1.0 - sigmoid) * (f64::from(c_lift_0) + f64::from(c_lift_a0) * a);
        let flat_plate = sigmoid
            * (2.0 * a.signum() * a.sin().powi(2) * a.cos());
        (linear + flat_plate) as f32
    }

    /// Upstream `Plane::dragCoeff`.
    pub fn drag_coeff(&self, alpha: f32) -> f32 {
        let b = f64::from(self.coefficient.b);
        let s = f64::from(self.coefficient.s);
        let c_drag_p = f64::from(self.coefficient.c_drag_p);
        let c_lift_0 = f64::from(self.coefficient.c_lift_0);
        let c_lift_a0 = f64::from(self.coefficient.c_lift_a);
        let oswald = f64::from(self.coefficient.oswald);
        let ar = b.powi(2) / s;
        let c_drag_a = c_drag_p
            + (c_lift_0 + c_lift_a0 * f64::from(alpha)).powi(2)
                / (core::f64::consts::PI * oswald * ar);
        c_drag_a as f32
    }

    /// Upstream `Plane::getForce`.
    pub fn get_force(
        &self,
        input_aileron: f32,
        input_elevator: f32,
        input_rudder: f32,
        alpha: f32,
        beta: f32,
        airspeed: f32,
        gyro: Vec3,
        air_density: f32,
    ) -> Vec3 {
        let c_drag_q = f64::from(self.coefficient.c_drag_q);
        let c_lift_q = f64::from(self.coefficient.c_lift_q);
        let s = f64::from(self.coefficient.s);
        let c = f64::from(self.coefficient.c);
        let b = f64::from(self.coefficient.b);
        let c_drag_deltae = f64::from(self.coefficient.c_drag_deltae);
        let c_lift_deltae = f64::from(self.coefficient.c_lift_deltae);
        let c_y_0 = f64::from(self.coefficient.c_y_0);
        let c_y_b = f64::from(self.coefficient.c_y_b);
        let c_y_p = f64::from(self.coefficient.c_y_p);
        let c_y_r = f64::from(self.coefficient.c_y_r);
        let c_y_deltaa = f64::from(self.coefficient.c_y_deltaa);
        let c_y_deltar = f64::from(self.coefficient.c_y_deltar);
        let rho = f64::from(air_density);
        let alpha_d = f64::from(alpha);
        let beta_d = f64::from(beta);
        let airspeed_d = f64::from(airspeed);

        let c_lift_a = f64::from(self.lift_coeff(alpha));
        let c_drag_a = f64::from(self.drag_coeff(alpha));

        let c_x_a = -c_drag_a * alpha_d.cos() + c_lift_a * alpha_d.sin();
        let c_x_q = -c_drag_q * alpha_d.cos() + c_lift_q * alpha_d.sin();
        let c_z_a = -c_drag_a * alpha_d.sin() - c_lift_a * alpha_d.cos();
        let c_z_q = -c_drag_q * alpha_d.sin() - c_lift_q * alpha_d.cos();

        let p = f64::from(gyro.x);
        let q = f64::from(gyro.y);
        let r = f64::from(gyro.z);

        let qbar = 0.5 * rho * airspeed_d.powi(2) * s;
        let (ax, ay, az) = if is_zero(airspeed) {
            (0.0, 0.0, 0.0)
        } else {
            let ax = qbar
                * (c_x_a
                    + c_x_q * c * q / (2.0 * airspeed_d)
                    - c_drag_deltae * alpha_d.cos() * f64::from(input_elevator).abs()
                    + c_lift_deltae * alpha_d.sin() * f64::from(input_elevator));
            let ay = qbar
                * (c_y_0
                    + c_y_b * beta_d
                    + c_y_p * b * p / (2.0 * airspeed_d)
                    + c_y_r * b * r / (2.0 * airspeed_d)
                    + c_y_deltaa * f64::from(input_aileron)
                    + c_y_deltar * f64::from(input_rudder));
            let az = qbar
                * (c_z_a
                    + c_z_q * c * q / (2.0 * airspeed_d)
                    - c_drag_deltae * alpha_d.sin() * f64::from(input_elevator).abs()
                    - c_lift_deltae * alpha_d.cos() * f64::from(input_elevator));
            (ax, ay, az)
        };
        Vec3::new(ax as f32, ay as f32, az as f32)
    }

    /// Upstream `Plane::getTorque`. `input_thrust` is unused in STANDARD
    /// config (tailsitter/aerobatic branch excluded), kept in the signature
    /// to match upstream.
    pub fn get_torque(
        &self,
        input_aileron: f32,
        input_elevator: f32,
        input_rudder: f32,
        _input_thrust: f32,
        force: Vec3,
        alpha: f32,
        airspeed: f32,
        beta: f32,
        gyro: Vec3,
        air_density: f32,
    ) -> Vec3 {
        let effective_airspeed = airspeed;
        let s = f64::from(self.coefficient.s);
        let c = f64::from(self.coefficient.c);
        let b = f64::from(self.coefficient.b);
        let rho = f64::from(air_density);
        let p = f64::from(gyro.x);
        let q = f64::from(gyro.y);
        let r = f64::from(gyro.z);
        let qbar = 0.5 * rho * f64::from(effective_airspeed).powi(2) * s;
        let (mut la, mut ma, mut na) = if is_zero(effective_airspeed) {
            (0.0, 0.0, 0.0)
        } else {
            let eas = f64::from(effective_airspeed);
            let la = qbar
                * b
                * (f64::from(self.coefficient.c_l_0)
                    + f64::from(self.coefficient.c_l_b) * f64::from(beta)
                    + f64::from(self.coefficient.c_l_p) * b * p / (2.0 * eas)
                    + f64::from(self.coefficient.c_l_r) * b * r / (2.0 * eas)
                    + f64::from(self.coefficient.c_l_deltaa) * f64::from(input_aileron)
                    + f64::from(self.coefficient.c_l_deltar) * f64::from(input_rudder));
            let ma = qbar
                * c
                * (f64::from(self.coefficient.c_m_0)
                    + f64::from(self.coefficient.c_m_a) * f64::from(alpha)
                    + f64::from(self.coefficient.c_m_q) * c * q / (2.0 * eas)
                    + f64::from(self.coefficient.c_m_deltae) * f64::from(input_elevator));
            let na = qbar
                * b
                * (f64::from(self.coefficient.c_n_0)
                    + f64::from(self.coefficient.c_n_b) * f64::from(beta)
                    + f64::from(self.coefficient.c_n_p) * b * p / (2.0 * eas)
                    + f64::from(self.coefficient.c_n_r) * b * r / (2.0 * eas)
                    + f64::from(self.coefficient.c_n_deltaa) * f64::from(input_aileron)
                    + f64::from(self.coefficient.c_n_deltar) * f64::from(input_rudder));
            (la, ma, na)
        };
        let cg = self.coefficient.cg_offset;
        la += f64::from(cg.y * force.z - cg.z * force.y);
        ma += f64::from(-cg.x * force.z + cg.z * force.x);
        na += f64::from(-cg.y * force.x + cg.x * force.y);
        Vec3::new(la as f32, ma as f32, na as f32)
    }

    pub fn mix_surfaces(
        &self,
        aileron: f32,
        mut elevator: f32,
        mut rudder: f32,
        throttle: f32,
    ) -> SurfaceDeflections {
        if self.frame_config.reverse_elevator_rudder {
            elevator = -elevator;
            rudder = -rudder;
        }
        let throttle = if self.frame_config.reverse_thrust {
            -throttle
        } else {
            throttle
        };
        match self.frame_config.mix {
            AirframeMix::Elevons => {
                let ch1 = aileron;
                let ch2 = elevator;
                SurfaceDeflections {
                    aileron: (ch2 - ch1) / 2.0,
                    elevator: -(ch2 + ch1) / 2.0,
                    rudder: 0.0,
                    throttle,
                }
            }
            AirframeMix::Vtail => {
                let ch1 = elevator;
                let ch2 = rudder;
                SurfaceDeflections {
                    aileron,
                    elevator: (ch2 - ch1) / 2.0,
                    rudder: (ch2 + ch1) / 2.0,
                    throttle,
                }
            }
            AirframeMix::Standard | AirframeMix::Dspoilers | AirframeMix::Redundant => {
                SurfaceDeflections {
                    aileron,
                    elevator,
                    rudder,
                    throttle,
                }
            }
        }
    }

    /// Upstream `Aircraft::update_wind`. Final `wind_ef = -wind_ef` converts
    /// meteorological FROM-heading into physical air-mass velocity so
    /// `velocity_air_ef = velocity_ef - wind_ef` is the standard identity.
    pub fn update_wind(&mut self) {
        let speed = self.wind_config.speed;
        let dir = radians(self.wind_config.direction);
        let dir_z = radians(self.wind_config.dir_z);
        self.wind_ef = Vec3::new(dir.cos() * dir_z.cos(), dir.sin() * dir_z.cos(), dir_z.sin())
            .scaled(speed);

        let wind_turb = self.wind_config.turbulence * 10.0;
        let iir_coef = 0.98_f32;
        if wind_turb > 0.0 && !self.on_ground() {
            self.turbulence_azimuth = (self.turbulence_azimuth + self.rng.next_f32() * 360.0) % 360.0;
            let n1 = self.rng.rand_normal() as f32;
            let n2 = self.rng.rand_normal() as f32;
            self.turbulence_horizontal_speed =
                self.turbulence_horizontal_speed * iir_coef + wind_turb * n1 * (1.0 - iir_coef);
            self.turbulence_vertical_speed =
                self.turbulence_vertical_speed * iir_coef + wind_turb * n2 * (1.0 - iir_coef);
            let az = radians(self.turbulence_azimuth);
            self.wind_ef = self.wind_ef.plus(Vec3::new(
                az.cos() * self.turbulence_horizontal_speed,
                az.sin() * self.turbulence_horizontal_speed,
                self.turbulence_vertical_speed,
            ));
        }
        self.wind_ef = self.wind_ef.scaled(-1.0);
    }

    fn apply_ground_behavior(&mut self) {
        if !self.on_ground() {
            return;
        }
        self.position.z = -(self.ground_level + self.frame_height - self.home_alt_m);
        match self.ground_behavior {
            GroundBehavior::None | GroundBehavior::Tailsitter => {}
            GroundBehavior::NoMovement => {
                let (_r, _p, y) = self.dcm.to_euler();
                self.dcm = Mat3::from_euler(0.0, 0.0, y);
                self.velocity_ef.x = 0.0;
                self.velocity_ef.y = 0.0;
                if self.velocity_ef.z > 0.0 {
                    self.velocity_ef.z = 0.0;
                }
                self.gyro = Vec3::zero();
            }
            GroundBehavior::FwdOnly => {
                let (_r, mut p, y) = self.dcm.to_euler();
                if self.velocity_ef.length() < 5.0 {
                    p = 0.0;
                } else {
                    p = p.max(0.0);
                }
                self.dcm = Mat3::from_euler(0.0, p, y);
                let mut v_bf = self.dcm.transposed().apply(self.velocity_ef);
                v_bf.y = 0.0;
                if v_bf.x < 0.0 {
                    v_bf.x = 0.0;
                }
                self.velocity_ef = self.dcm.apply(v_bf);
                if self.velocity_ef.z > 0.0 {
                    self.velocity_ef.z = 0.0;
                }
                self.gyro = Vec3::zero();
            }
        }
    }

    fn update_eas_airspeed(&mut self) {
        let tas = self.velocity_air_ef.length();
        self.airspeed = if self.eas2tas > 0.0 {
            tas / self.eas2tas
        } else {
            tas
        };
        self.airspeed_pitot = self.airspeed;
        let pitot_aoa = self
            .velocity_air_bf
            .y
            .hypot(self.velocity_air_bf.z)
            .atan2(self.velocity_air_bf.x);
        let max_pitot_aoa = radians(20.0);
        if pitot_aoa > radians(90.0) {
            self.airspeed_pitot = 0.0;
        } else if pitot_aoa > max_pitot_aoa {
            let gain_factor = (0.5 * PI) / (radians(90.0) - max_pitot_aoa);
            self.airspeed_pitot *= ((pitot_aoa - max_pitot_aoa) * gain_factor).cos();
        }
    }

    /// Upstream `Aircraft::update_dynamics`.
    pub fn update_dynamics(&mut self, rot_accel: Vec3, dt: f32) {
        self.gyro = self.gyro.plus(rot_accel.scaled(dt));
        let gyro_lim = radians(2000.0);
        self.gyro.x = constrain(self.gyro.x, -gyro_lim, gyro_lim);
        self.gyro.y = constrain(self.gyro.y, -gyro_lim, gyro_lim);
        self.gyro.z = constrain(self.gyro.z, -gyro_lim, gyro_lim);

        let accel_limit = 64.0 * GRAVITY_MSS;
        self.accel_body.x = constrain(self.accel_body.x, -accel_limit, accel_limit);
        self.accel_body.y = constrain(self.accel_body.y, -accel_limit, accel_limit);
        self.accel_body.z = constrain(self.accel_body.z, -accel_limit, accel_limit);

        self.dcm.rotate(self.gyro.scaled(dt));
        self.dcm.normalize();

        let mut accel_earth = self.dcm.apply(self.accel_body);
        accel_earth = accel_earth.plus(Vec3::new(0.0, 0.0, GRAVITY_MSS));

        if self.on_ground() && accel_earth.z > 0.0 {
            accel_earth.z = 0.0;
        }

        self.accel_body = self
            .dcm
            .transposed()
            .apply(accel_earth.plus(Vec3::new(0.0, 0.0, -GRAVITY_MSS)));

        self.velocity_ef = self.velocity_ef.plus(accel_earth.scaled(dt));
        let was_on_ground = self.on_ground();
        self.position = self.position.plus(self.velocity_ef.scaled(dt));

        self.velocity_air_ef = self.velocity_ef.minus(self.wind_ef);
        self.velocity_air_bf = self.dcm.transposed().apply(self.velocity_air_ef);
        self.update_eas_airspeed();

        if self.on_ground() {
            let _ = was_on_ground;
            self.apply_ground_behavior();
            if self.on_ground() && self.velocity_ef.z > 0.0 {
                self.velocity_ef.z = 0.0;
            }
        }
    }

    /// Upstream `Plane::update`: wind, mix, AoA, forces, thrust, dynamics.
    pub fn update(&mut self, aileron: f32, elevator: f32, rudder: f32, throttle: f32, dt: f32) {
        self.update_wind();
        let mixed = self.mix_surfaces(aileron, elevator, rudder, throttle);
        self.angle_of_attack = self.velocity_air_bf.z.atan2(self.velocity_air_bf.x);
        self.beta = self.velocity_air_bf.y.atan2(self.velocity_air_bf.x);

        let force = self.get_force(
            mixed.aileron,
            mixed.elevator,
            mixed.rudder,
            self.angle_of_attack,
            self.beta,
            self.airspeed,
            self.gyro,
            self.air_density,
        );
        let rot_accel = self.get_torque(
            mixed.aileron,
            mixed.elevator,
            mixed.rudder,
            mixed.throttle,
            force,
            self.angle_of_attack,
            self.airspeed,
            self.beta,
            self.gyro,
            self.air_density,
        );

        // `thrust_scale` is a stored field (see its doc comment), not
        // recomputed from `mass` here -- required for the real `-heavy`
        // asymmetry (mass changes, thrust_scale deliberately does not).
        let thrust_newtons = mixed.throttle * self.thrust_scale;
        self.accel_body = Vec3::new(thrust_newtons, 0.0, 0.0)
            .plus(force)
            .scaled(1.0 / self.mass);

        if self.on_ground() {
            let vel_body = self.dcm.transposed().apply(self.velocity_ef);
            self.accel_body.x -= vel_body.x * 0.3;
        }

        self.update_dynamics(rot_accel, dt);
    }

    /// Upstream `_update_airspeed` → `AP_Airspeed_SITL::get_differential_pressure`.
    /// `eas2tas == 1` in this port, so EAS == TAS == `self.airspeed`.
    pub fn airspeed_sensor_differential_pressure(
        &mut self,
        ratio: f32,
        noise_amplitude: f32,
    ) -> f32 {
        let eas = self.airspeed;
        let diff_pressure = (eas * eas) / ratio;
        let noisy = ratio * (diff_pressure + noise_amplitude * self.rng.rand_float());
        let eas_noisy = noisy.abs().sqrt();
        (eas_noisy * eas_noisy) / ratio
    }

    pub fn airspeed_sensor_differential_pressure_default(&mut self) -> f32 {
        self.airspeed_sensor_differential_pressure(
            DEFAULT_AIRSPEED_SENSOR_RATIO,
            DEFAULT_AIRSPEED_NOISE_PA,
        )
    }

    pub fn true_euler_deg(&self) -> (f32, f32, f32) {
        let (r, p, y) = self.dcm.to_euler();
        (degrees(r), degrees(p), degrees(y))
    }

    pub fn altitude_m(&self) -> f32 {
        -self.position.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference_lift_coeff(c: &Coefficients, alpha: f32) -> f64 {
        let alpha0 = f64::from(c.alpha_stall);
        let m = f64::from(c.mcoeff);
        let mut a = f64::from(alpha);
        let max_alpha_delta = 0.8_f64;
        if a - alpha0 > max_alpha_delta {
            a = alpha0 + max_alpha_delta;
        } else if alpha0 - a > max_alpha_delta {
            a = alpha0 - max_alpha_delta;
        }
        let sigmoid = (1.0 + (-m * (a - alpha0)).exp() + (m * (a + alpha0)).exp())
            / (1.0 + (-m * (a - alpha0)).exp())
            / (1.0 + (m * (a + alpha0)).exp());
        let linear = (1.0 - sigmoid) * (f64::from(c.c_lift_0) + f64::from(c.c_lift_a) * a);
        let flat_plate = sigmoid * (2.0 * a.signum() * a.sin().powi(2) * a.cos());
        linear + flat_plate
    }

    fn reference_drag_coeff(c: &Coefficients, alpha: f32) -> f64 {
        let ar = f64::from(c.b).powi(2) / f64::from(c.s);
        f64::from(c.c_drag_p)
            + (f64::from(c.c_lift_0) + f64::from(c.c_lift_a) * f64::from(alpha)).powi(2)
                / (core::f64::consts::PI * f64::from(c.oswald) * ar)
    }

    fn reference_steady_wind_ef(speed: f32, direction_deg: f32, dir_z_deg: f32) -> Vec3 {
        let dir = f64::from(direction_deg) * core::f64::consts::PI / 180.0;
        let dz = f64::from(dir_z_deg) * core::f64::consts::PI / 180.0;
        let raw = Vec3::new(
            (dir.cos() * dz.cos()) as f32,
            (dir.sin() * dz.cos()) as f32,
            dz.sin() as f32,
        )
        .scaled(speed);
        raw.scaled(-1.0)
    }

    fn approx(a: f32, b: f32, margin: f32) -> bool {
        (a - b).abs() <= margin
    }

    #[test]
    fn lift_coeff_matches_independent_reference() {
        let plane = SimPlane::new();
        let c = Coefficients::default();
        for alpha in [-0.6, -0.2, 0.0, 0.1, 0.3, 0.6, 1.2, 2.5] {
            let got = f64::from(plane.lift_coeff(alpha));
            let want = reference_lift_coeff(&c, alpha);
            assert!((got - want).abs() < 1e-4, "alpha={alpha}: {got} vs {want}");
        }
    }

    #[test]
    fn lift_coeff_linear_then_stall() {
        let plane = SimPlane::new();
        let c = Coefficients::default();
        let lift_0 = plane.lift_coeff(0.0);
        let lift_small = plane.lift_coeff(0.2);
        assert!(lift_small > lift_0);
        assert!(approx(lift_0, c.c_lift_0, 1e-3));
        assert!(approx(lift_small, c.c_lift_0 + c.c_lift_a * 0.2, 1e-2));
        let lift_deep = plane.lift_coeff(3.0);
        let naive = c.c_lift_0 + c.c_lift_a * 3.0;
        assert!((lift_deep - naive).abs() > 1.0);
    }

    #[test]
    fn drag_coeff_matches_independent_reference() {
        let plane = SimPlane::new();
        let c = Coefficients::default();
        for alpha in [-0.5, -0.08, 0.0, 0.3, 0.8] {
            let got = f64::from(plane.drag_coeff(alpha));
            let want = reference_drag_coeff(&c, alpha);
            assert!((got - want).abs() < 1e-5, "alpha={alpha}: {got} vs {want}");
        }
        let alpha_min = -c.c_lift_0 / c.c_lift_a;
        let drag_min = plane.drag_coeff(alpha_min);
        assert!(plane.drag_coeff(alpha_min + 0.3) > drag_min);
        assert!(plane.drag_coeff(alpha_min - 0.3) > drag_min);
    }

    #[test]
    fn get_force_zero_at_zero_airspeed() {
        let plane = SimPlane::new();
        let force = plane.get_force(
            0.1,
            0.2,
            -0.1,
            0.3,
            0.05,
            0.0,
            Vec3::new(0.1, -0.2, 0.05),
            SSL_AIR_DENSITY,
        );
        assert_eq!(force, Vec3::zero());
    }

    #[test]
    fn get_torque_at_zero_airspeed_is_cg_term() {
        let plane = SimPlane::new();
        let c = Coefficients::default();
        let force = Vec3::new(2.0, -1.0, 0.5);
        let torque = plane.get_torque(
            0.1,
            0.2,
            -0.1,
            0.5,
            force,
            0.3,
            0.0,
            0.05,
            Vec3::new(0.1, -0.2, 0.05),
            SSL_AIR_DENSITY,
        );
        let expected_la = c.cg_offset.y * force.z - c.cg_offset.z * force.y;
        let expected_ma = -c.cg_offset.x * force.z + c.cg_offset.z * force.x;
        let expected_na = -c.cg_offset.y * force.x + c.cg_offset.x * force.y;
        assert!(approx(torque.x, expected_la, 1e-5));
        assert!(approx(torque.y, expected_ma, 1e-5));
        assert!(approx(torque.z, expected_na, 1e-5));
    }

    #[test]
    fn trim_alpha_roughly_balances_weight() {
        let plane = SimPlane::new();
        let c = Coefficients::default();
        let cruise = 15.0_f32;
        let weight = plane.mass * GRAVITY_MSS;
        let qbar = 0.5 * SSL_AIR_DENSITY * cruise * cruise;
        let required_cl = weight / (qbar * c.s);
        let alpha_trim = (required_cl - c.c_lift_0) / c.c_lift_a;
        let force = plane.get_force(
            0.0,
            0.0,
            0.0,
            alpha_trim,
            0.0,
            cruise,
            Vec3::zero(),
            SSL_AIR_DENSITY,
        );
        assert!(
            approx(force.z, -weight, weight * 0.2),
            "force.z={} weight={}",
            force.z,
            weight
        );
    }

    #[test]
    fn update_from_rest_accelerates_without_nan() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::new(0.0, 0.0, -500.0);
        let dt = 0.005;
        for _ in 0..200 {
            plane.update(0.0, 0.0, 0.0, plane.hover_throttle, dt);
        }
        assert!(!plane.dcm.is_nan());
        assert!(!plane.velocity_ef.x.is_nan());
        assert!(!plane.position.z.is_nan());
        assert!(plane.velocity_ef.length() > 3.0);
        assert!(plane.airspeed > 3.0);
        assert!(plane.position.z < -490.0);
    }

    #[test]
    fn ground_contact_clamp() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::zero();
        plane.velocity_ef = Vec3::new(0.0, 0.0, 5.0);
        let dt = 0.01;
        for _ in 0..3 {
            plane.update_dynamics(Vec3::zero(), dt);
            assert!(plane.position.z < 0.1);
            assert!(plane.position.z >= 0.0);
        }
        assert!(approx(plane.velocity_ef.z, 0.0, 1e-6));
    }

    #[test]
    fn gyro_rate_limit() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::new(0.0, 0.0, -100.0);
        plane.update_dynamics(Vec3::new(1.0e8, 0.0, 0.0), 1.0e-6);
        let limit = radians(2000.0);
        assert!(approx(plane.gyro.x, limit, 1e-3));
        assert!(approx(plane.gyro.y, 0.0, 1e-6));
        assert!(approx(plane.gyro.z, 0.0, 1e-6));
        assert!(!plane.dcm.is_nan());
    }

    #[test]
    fn accel_64g_limit() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::new(0.0, 0.0, -100.0);
        plane.accel_body = Vec3::new(1.0e8, 0.0, 0.0);
        plane.update_dynamics(Vec3::zero(), 0.001);
        let limit = 64.0 * GRAVITY_MSS;
        assert!(approx(plane.accel_body.x, limit, 1e-2));
        assert!(approx(plane.accel_body.y, 0.0, 1e-6));
        assert!(approx(plane.accel_body.z, 0.0, 1e-2));
    }

    #[test]
    fn dcm_stays_orthonormal() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::new(0.0, 0.0, -100.0);
        let rot_accel = Vec3::new(0.05, -0.03, 0.02);
        for _ in 0..5000 {
            plane.update_dynamics(rot_accel, 0.001);
        }
        assert!(!plane.dcm.is_nan());
        assert!(approx(plane.dcm.a.length(), 1.0, 1e-4));
        assert!(approx(plane.dcm.b.length(), 1.0, 1e-4));
        assert!(approx(plane.dcm.c.length(), 1.0, 1e-4));
        assert!(approx(plane.dcm.a.dot(plane.dcm.b), 0.0, 1e-4));
        assert!(approx(plane.dcm.det(), 1.0, 1e-3));
    }

    #[test]
    fn update_wind_steady_sign_convention() {
        let mut plane = SimPlane::new();
        plane.wind_config.turbulence = 0.0;
        plane.wind_config.speed = 6.0;
        plane.wind_config.direction = 90.0;
        plane.wind_config.dir_z = 0.0;
        plane.update_wind();
        let expected = reference_steady_wind_ef(6.0, 90.0, 0.0);
        assert!(approx(plane.wind_ef.x, expected.x, 1e-5));
        assert!(approx(plane.wind_ef.y, expected.y, 1e-5));
        assert!(approx(plane.wind_ef.z, expected.z, 1e-5));
        assert!(plane.wind_ef.y < -5.9);
        assert!(plane.wind_ef.x.abs() < 1e-4);

        plane.wind_config.speed = 8.0;
        plane.wind_config.direction = 0.0;
        plane.update_wind();
        let expected = reference_steady_wind_ef(8.0, 0.0, 0.0);
        assert!(approx(plane.wind_ef.x, expected.x, 1e-5));
        // FROM north: physical air mass moves south, NED x (north) is negative.
        assert!(plane.wind_ef.x < -7.9);

        plane.wind_config.speed = 4.0;
        plane.wind_config.direction = 45.0;
        plane.wind_config.dir_z = -30.0;
        plane.update_wind();
        let expected = reference_steady_wind_ef(4.0, 45.0, -30.0);
        assert!(approx(plane.wind_ef.x, expected.x, 1e-4));
        assert!(approx(plane.wind_ef.y, expected.y, 1e-4));
        assert!(approx(plane.wind_ef.z, expected.z, 1e-4));
    }

    #[test]
    fn turbulence_gated_on_ground() {
        let mut on_ground = SimPlane::new();
        on_ground.wind_config.turbulence = 1.0;
        on_ground.position = Vec3::zero();
        on_ground.update_wind();
        assert_eq!(on_ground.turbulence_horizontal_speed, 0.0);
        assert_eq!(on_ground.turbulence_vertical_speed, 0.0);

        let mut no_turb = SimPlane::new();
        no_turb.position = Vec3::new(0.0, 0.0, -10.0);
        no_turb.wind_config.turbulence = 0.0;
        no_turb.update_wind();
        assert_eq!(no_turb.turbulence_horizontal_speed, 0.0);

        let mut airborne = SimPlane::new();
        airborne.position = Vec3::new(0.0, 0.0, -10.0);
        airborne.wind_config.turbulence = 1.0;
        airborne.update_wind();
        assert!(
            airborne.turbulence_horizontal_speed != 0.0
                || airborne.turbulence_vertical_speed != 0.0
        );
    }

    #[test]
    fn fwd_only_takeoff_climbs() {
        let mut plane = SimPlane::new();
        plane.ground_behavior = GroundBehavior::FwdOnly;
        let dt = 0.02;
        for _ in 0..500 {
            plane.update(0.0, 0.0, 0.0, 0.75, dt);
        }
        assert!(
            plane.altitude_m() > 1.0,
            "expected takeoff under FWD_ONLY, alt={} as={}",
            plane.altitude_m(),
            plane.airspeed
        );
        assert!(plane.airspeed > 10.0);
    }

    fn velocity_air_subtracts_wind() {
        let mut plane = SimPlane::new();
        plane.position = Vec3::new(0.0, 0.0, -100.0);
        plane.velocity_ef = Vec3::new(10.0, 0.0, 0.0);
        plane.wind_config.speed = 3.0;
        plane.wind_config.direction = 0.0;
        plane.wind_config.turbulence = 0.0;
        plane.update_wind();
        plane.update_dynamics(Vec3::zero(), 0.0);
        let expected = plane.velocity_ef.minus(plane.wind_ef);
        assert!(approx(plane.velocity_air_ef.x, expected.x, 1e-4));
        assert!(approx(plane.velocity_air_ef.y, expected.y, 1e-4));
        assert!(approx(plane.velocity_air_ef.z, expected.z, 1e-4));
    }

    #[test]
    fn load_coeffs_overrides_wing_area() {
        let dir = std::env::temp_dir();
        let path = dir.join("ardumaster_plane_coeffs.json");
        // Key is "CGOffset", matching real upstream (`SIM_Plane.cpp` real
        // line 191) -- not "cg". An earlier version of this fixture used
        // "cg", which happened to still compile and "pass" only because it
        // silently failed to update `cg_offset` at all; see
        // `load_coeffs_reproduces_real_skywalker_2013_fixture` for the
        // round-trip check against real upstream content that caught this.
        std::fs::write(
            &path,
            r#"{"s": 0.99, "c_lift_0": 0.77, "CGOffset": [-0.2, 0.0, -0.04]}"#,
        )
        .unwrap();
        let mut plane = SimPlane::new();
        assert!(plane.load_coeffs(path.to_str().unwrap()));
        assert!((plane.coefficient.s - 0.99).abs() < 1e-4);
        assert!((plane.coefficient.c_lift_0 - 0.77).abs() < 1e-4);
        assert!((plane.coefficient.cg_offset.x + 0.2).abs() < 1e-4);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn load_coeffs_reproduces_real_skywalker_2013_fixture() {
        // Real, byte-for-byte upstream `Tools/autotest/models/
        // skywalker_2013.json` (Plane-4.7.0) -- the only real,
        // native-format Plane coefficient file anywhere in the pinned
        // upstream tree (confirmed directly: `grep -rl c_lift_a` across the
        // whole pinned tree finds no second one). `Callisto.json`/
        // `freestyle.json` in the same upstream directory are multicopter
        // frame configs (mass/battery/motor-count fields, not aerodynamic
        // coefficients) and `xplane_plane.json`/`xplane_heli.json` are
        // X-Plane DREF maps for an unrelated external-FDM backend -- none
        // of the three loads via `load_coeffs`, and none should be used
        // here. This port's own `Coefficients::default()` is itself
        // disclosed as sourced from this exact file, so a correct
        // `load_coeffs` round-trip must reproduce it exactly.
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("fixtures/skywalker_2013.json"))
            .expect("workspace root");
        let mut plane = SimPlane::new();
        assert!(
            plane.load_coeffs(path.to_str().expect("utf8 path")),
            "failed to load real fixture {}",
            path.display()
        );
        assert_eq!(plane.coefficient, Coefficients::default());
    }

    #[test]
    fn with_heavy_frame_changes_mass_but_not_thrust_scale() {
        // Real upstream `SIM_Plane.cpp` real lines 53-55: `-heavy` sets
        // `mass = 8` and does NOT touch `thrust_scale` -- re-verified
        // directly against the pinned source before writing this test.
        let default_plane = SimPlane::new();
        let heavy = SimPlane::with_heavy_frame();
        assert!(approx(heavy.mass, 8.0, 1e-6));
        assert!(approx(heavy.thrust_scale, default_plane.thrust_scale, 1e-6));
        // The stale thrust_scale is NOT what a fresh recompute from the new
        // mass would give -- that gap is the whole point of the asymmetry.
        let recomputed_from_new_mass = (heavy.mass * GRAVITY_MSS) / heavy.hover_throttle;
        assert!(!approx(heavy.thrust_scale, recomputed_from_new_mass, 1e-3));
    }

    #[test]
    fn with_jet_frame_recomputes_mass_and_thrust_scale() {
        // Real upstream `SIM_Plane.cpp` real lines 56-59: `-jet` sets
        // `mass = 22` AND recomputes
        // `thrust_scale = (mass * GRAVITY_MSS) / hover_throttle`. Expected
        // value derived here from the real formula and this port's own
        // `GRAVITY_MSS`/`hover_throttle` constants, not copied from
        // anywhere else.
        let jet = SimPlane::with_jet_frame();
        assert!(approx(jet.mass, 22.0, 1e-6));
        let expected_thrust_scale = (22.0_f32 * GRAVITY_MSS) / jet.hover_throttle;
        assert!(approx(jet.thrust_scale, expected_thrust_scale, 1e-3));
        // Genuinely different from the default plane's thrust_scale.
        let default_plane = SimPlane::new();
        assert!(!approx(jet.thrust_scale, default_plane.thrust_scale, 1e-3));
    }
}
