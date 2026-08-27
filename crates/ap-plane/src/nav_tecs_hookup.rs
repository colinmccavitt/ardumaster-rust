//! L1/TECS navigation demand publish for the stabilize path.
//!
//! Upstream `Plane::calc_nav_roll` and `Plane::calc_nav_pitch` read from
//! `nav_controller` and TECS before stabilize limits and applies them to
//! `nav_roll_cd` / `nav_pitch_cd`.

use ap_math::scalar::rad_to_cd;

use crate::stabilize_hookup::NavCommandInputs;

/// Navigation controller outputs for one stabilize tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NavTecsPublish {
    /// L1 bank demand, centidegrees. Upstream `nav_controller->nav_roll_cd()`.
    pub nav_roll_cd: i32,
    /// TECS pitch demand, radians. Upstream `TECS_controller.get_pitch_demand()`.
    pub tecs_pitch_demand_rad: f32,
}

/// Feed L1/TECS outputs into `nav_commands` before `prepare_stabilize_path`.
pub fn feed_nav_commands(out: &mut NavCommandInputs, src: &NavTecsPublish) {
    out.commanded_roll_cd = src.nav_roll_cd;
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream truncates TECS pitch demand to int32 centidegrees the same way"
    )]
    {
        out.commanded_pitch_cd = rad_to_cd(src.tecs_pitch_demand_rad) as i32;
    }
}
