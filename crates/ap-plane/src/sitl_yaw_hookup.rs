//! SITL compass and GPS yaw sample publish, upstream SIM → compass/GPS backends.
//!
//! The main loop calls [`publish_sitl_yaw_samples`] before building
//! [`YawUpdateInputs`](ap_ahrs::YawUpdateInputs) so SITL truth reaches the DCM
//! yaw drift path.

use ap_ahrs::{YawCompassSample, YawDriftContext, YawGpsSample};
use ap_declination::get_mag_field_ef;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{radians, Real};
use ap_math::vector3::Vector3f;

/// SITL vehicle kinematics for yaw sensor publish.
#[derive(Debug, Clone, Copy)]
pub struct SitlYawPublish {
    /// Vehicle latitude, degrees.
    pub latitude_deg: f32,
    /// Vehicle longitude, degrees.
    pub longitude_deg: f32,
    /// Ground speed, m/s. Upstream `AP_GPS::ground_speed()`.
    pub ground_speed_mps: f32,
    /// Ground course, degrees. Upstream `AP_GPS::ground_course()`.
    pub ground_course_deg: f32,
    /// Fix timestamp, ms. Upstream `AP_GPS::last_fix_time_ms()`.
    pub last_fix_time_ms: u32,
    /// Monotonic time, ms. Upstream `AP_HAL::millis()`.
    pub now_ms: u32,
    /// Upstream `AP::ahrs().get_fly_forward()`.
    pub fly_forward: bool,
    /// Upstream `compass.use_for_yaw()`.
    pub compass_use_for_yaw: bool,
    /// Horizontal wind speed, m/s.
    pub wind_speed_xy: f32,
    /// Whether a GPS fix is available.
    pub have_gps: bool,
}

impl Default for SitlYawPublish {
    fn default() -> Self {
        Self {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            ground_speed_mps: 0.0,
            ground_course_deg: 0.0,
            last_fix_time_ms: 0,
            now_ms: 0,
            fly_forward: true,
            compass_use_for_yaw: true,
            wind_speed_xy: 0.0,
            have_gps: false,
        }
    }
}

/// Yaw samples published into the main loop before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlYawSamples {
    pub compass: Option<YawCompassSample>,
    pub gps_yaw: Option<YawGpsSample>,
    pub yaw_ctx: YawDriftContext,
}

/// Earth-frame magnetic field in NED from the WMM lookup at `lat`/`lon`.
fn mag_field_ef_ned(latitude_deg: f32, longitude_deg: f32) -> (Vector3f, f32) {
    let (field, _coverage) = get_mag_field_ef(latitude_deg, longitude_deg);
    let declination_rad = radians(field.declination_deg);
    let inclination_rad = radians(field.inclination_deg);
    let intensity = field.intensity_gauss;
    let horizontal = intensity * Real::cos(inclination_rad);
    let mag_ef = Vector3f::new(
        horizontal * Real::cos(declination_rad),
        horizontal * Real::sin(declination_rad),
        intensity * Real::sin(inclination_rad),
    );
    (mag_ef, declination_rad)
}

/// Publish SITL compass and GPS samples for one AHRS cycle.
#[must_use]
pub fn publish_sitl_yaw_samples(
    source: &SitlYawPublish,
    attitude: Matrix3f,
    loop_dt: f32,
) -> SitlYawSamples {
    let (mag_ef, declination_rad) = mag_field_ef_ned(source.latitude_deg, source.longitude_deg);
    let mag_body = attitude.transposed() * mag_ef;
    let (_, _, estimated_yaw_rad) = attitude.to_euler();

    let compass = source.compass_use_for_yaw.then_some(YawCompassSample {
        mag_body,
        declination_rad,
        update_interval_s: Some(loop_dt),
        calibrating: false,
    });

    let gps_yaw = source.have_gps.then_some(YawGpsSample {
        ground_course_deg: source.ground_course_deg,
        ground_speed: source.ground_speed_mps,
        last_fix_time_ms: source.last_fix_time_ms,
    });

    let yaw_ctx = YawDriftContext {
        fly_forward: source.fly_forward,
        have_gps: source.have_gps,
        compass_use_for_yaw: source.compass_use_for_yaw,
        estimated_yaw_rad,
        wind_speed_xy: source.wind_speed_xy,
        now_ms: source.now_ms,
        gps_lat_e7: source
            .have_gps
            .then(|| (source.latitude_deg * 1e7_f32) as i32),
        gps_lng_e7: source
            .have_gps
            .then(|| (source.longitude_deg * 1e7_f32) as i32),
    };

    SitlYawSamples {
        compass,
        gps_yaw,
        yaw_ctx,
    }
}
