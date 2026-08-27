use ap_math::scalar::{cd_to_rad, rad_to_cd};
use ap_plane::nav_tecs_scheduler_hookup::nav_tecs_scheduler_publish_tick;
use ap_plane::navigation_scheduler_hookup::{
    navigation_scheduler_tick, NavigationSchedulerInputs,
};

#[test]
fn nav_tecs_publish_carries_limited_demands() {
    let nav = navigation_scheduler_tick(&NavigationSchedulerInputs {
        commanded_roll_cd: 3000,
        commanded_pitch_cd: 1200,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
    });
    let pub_ = nav_tecs_scheduler_publish_tick(nav);
    assert_eq!(pub_.nav_roll_cd, 3000);
    assert_eq!(rad_to_cd(pub_.tecs_pitch_demand_rad) as i32, 1200);
}
