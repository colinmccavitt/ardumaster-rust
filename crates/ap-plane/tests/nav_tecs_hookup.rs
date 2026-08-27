//! L1/TECS navigation feed into stabilize nav_commands.

use ap_math::scalar::{cd_to_rad, rad_to_cd};
use ap_plane::nav_tecs_hookup::{feed_nav_commands, NavTecsPublish};
use ap_plane::stabilize_hookup::NavCommandInputs;

#[test]
fn feed_nav_commands_copies_roll_and_converts_pitch() {
    let pitch_rad = cd_to_rad(1500.0);
    let mut nav = NavCommandInputs::default();
    feed_nav_commands(
        &mut nav,
        &NavTecsPublish {
            nav_roll_cd: 3200,
            tecs_pitch_demand_rad: pitch_rad,
        },
    );
    assert_eq!(nav.commanded_roll_cd, 3200);
    assert_eq!(nav.commanded_pitch_cd, rad_to_cd(pitch_rad) as i32);
}

#[test]
fn stabilize_uses_nav_tecs_publish() {
    use ap_plane::main_loop::PlaneMainLoop;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ap_plane::mode_table::ModeNumber::Stabilize.as_number();
    vehicle.update_control_mode();
    vehicle.nav_tecs = NavTecsPublish {
        nav_roll_cd: 2500,
        tecs_pitch_demand_rad: cd_to_rad(1000.0),
    };
    vehicle.stabilize_demands.roll_limit_cd = 4500;
    vehicle.stabilize_demands.pitch_limit_min_cd = -2000;
    vehicle.stabilize_demands.pitch_limit_max_cd = 2500;

    vehicle.stabilize();

    assert_eq!(vehicle.stabilize_demands.nav_roll_cd, 2500);
    assert_eq!(vehicle.stabilize_demands.nav_pitch_cd, 1000);
}
