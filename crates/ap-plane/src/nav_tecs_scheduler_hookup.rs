//! Nav/TECS publish tick for the scheduler glue path.
//!
//! Upstream `Plane::calc_nav_*` writes demands that stabilize reads on the
//! next tick via [`feed_nav_commands`].

use crate::nav_tecs_hookup::NavTecsPublish;
use crate::navigation_scheduler_hookup::NavigationSchedulerOutput;

/// Publish L1/TECS demands for the stabilize path.
#[must_use]
pub fn nav_tecs_scheduler_publish_tick(
    nav: NavigationSchedulerOutput,
) -> NavTecsPublish {
    NavTecsPublish {
        nav_roll_cd: nav.nav_roll_cd,
        tecs_pitch_demand_rad: nav.tecs_pitch_demand_rad,
    }
}
