//! OA path-planner leftover. Upstream `AP_OAPathPlanner`.
//!
//! Frontend: type / margin / options, `init`, `pre_arm_check`,
//! `mission_avoidance`, and one tick of `avoidance_thread` as
//! [`PathPlanner::process`]. Planner arms are [`crate::oa_bendy_ruler`]
//! (horizontal) and [`crate::oa_dijkstra`].
//!
//! The OA database, vertical BendyRuler, and the background HAL thread
//! stay later leftovers. ADR-0004 forbids the AHRS / HAL / database
//! singletons.
//!
//! [`PathPlanner::process`] is the leftover of one avoidance-thread
//! iteration; [`MissionAvoidanceLeftover`] is the leftover of the
//! request / result handshake.

use ap_math::location::Location;
use ap_math::vector2::Vector2f;

use crate::oa_bendy_ruler::{
    same_latlon, BendyMarginContext, BendyRuler, OaBendyType, LOOKAHEAD_M_DEFAULT,
};
use crate::oa_dijkstra::{Dijkstra, DijkstraFenceContext, DijkstraState};

/// Default `OA_MARGIN_MAX`, metres. Upstream `OA_MARGIN_MAX_DEFAULT`.
pub const MARGIN_MAX_M_DEFAULT: f32 = 5.0;
/// Default `OA_OPTIONS`. Upstream `OA_OPTIONS_DEFAULT`.
pub const OPTIONS_DEFAULT: u16 = 1;
/// Planner ticks run at 1 Hz. Upstream `OA_UPDATE_MS`.
pub const UPDATE_MS: u32 = 1000;
/// Results older than this are ignored. Upstream `OA_TIMEOUT_MS`.
pub const TIMEOUT_MS: u32 = 3000;
/// Gap that re-arms activation. Upstream `now - _last_update_ms > 200`.
pub const REACTIVATE_GAP_MS: u32 = 200;

/// Path-planner type. Upstream `OAPathPlanTypes`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaPathPlanType {
    /// Disabled. Upstream `OA_PATHPLAN_DISABLED`.
    Disabled = 0,
    /// BendyRuler only. Upstream `OA_PATHPLAN_BENDYRULER`.
    BendyRuler = 1,
    /// Dijkstra only. Upstream `OA_PATHPLAN_DIJKSTRA`.
    Dijkstra = 2,
    /// Dijkstra with BendyRuler. Upstream `OA_PATHPLAN_DJIKSTRA_BENDYRULER`.
    DijkstraBendyRuler = 3,
}

/// Recovery-option bits. Upstream `OARecoveryOptions`.
pub const OPTION_DISABLED: u16 = 0;
/// Reset waypoint origin. Upstream `OA_OPTION_WP_RESET`.
pub const OPTION_WP_RESET: u16 = 1 << 0;
/// Log Dijkstra points. Upstream `OA_OPTION_LOG_DIJKSTRA_POINTS`.
pub const OPTION_LOG_DIJKSTRA_POINTS: u16 = 1 << 1;
/// Fast waypoints (Dijkstra only). Upstream `OA_OPTION_FAST_WAYPOINTS`.
pub const OPTION_FAST_WAYPOINTS: u16 = 1 << 2;

/// Return state. Upstream `AP_OAPathPlanner::OA_RetState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaRetState {
    /// Avoidance not required. Upstream `OA_NOT_REQUIRED`.
    NotRequired = 0,
    /// Background tick has not produced a matching result yet.
    Processing = 1,
    /// Timed out or planner error. Upstream `OA_ERROR`.
    Error = 2,
    /// Intermediate path is ready. Upstream `OA_SUCCESS`.
    Success = 3,
}

/// Which planner produced the result. Upstream `OAPathPlannerUsed`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaPathPlannerUsed {
    /// None. Upstream `None`.
    None = 0,
    /// Horizontal BendyRuler. Upstream `BendyRulerHorizontal`.
    BendyRulerHorizontal = 1,
    /// Vertical BendyRuler. Upstream `BendyRulerVertical`.
    BendyRulerVertical = 2,
    /// Dijkstra. Upstream `Dijkstras`.
    Dijkstras = 3,
}

/// Leftover of `AP_OAPathPlanner::pre_arm_check`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreArmCheckLeftover {
    /// `true` when the check passed.
    pub ok: bool,
    /// Leftover of `failure_msg`. Empty when [`Self::ok`].
    pub failure_msg: &'static str,
}

/// Leftover of one `AP_OAPathPlanner::mission_avoidance` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionAvoidanceLeftover {
    /// Return state. Upstream `OA_RetState`.
    pub ret_state: OaRetState,
    /// Intermediate origin when a result is ready.
    pub result_origin: Location,
    /// Intermediate destination when a result is ready.
    pub result_destination: Location,
    /// Intermediate next destination when a result is ready.
    pub result_next_destination: Location,
    /// Dijkstra-only clear flag. Upstream `result_dest_to_next_dest_clear`.
    pub dest_to_next_dest_clear: bool,
    /// Which planner produced the result.
    pub path_planner_used: OaPathPlannerUsed,
}

impl Default for MissionAvoidanceLeftover {
    fn default() -> Self {
        Self {
            ret_state: OaRetState::NotRequired,
            result_origin: Location::new(0, 0),
            result_destination: Location::new(0, 0),
            result_next_destination: Location::new(0, 0),
            dest_to_next_dest_clear: false,
            path_planner_used: OaPathPlannerUsed::None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct AvoidanceRequest {
    current_loc: Location,
    origin: Location,
    destination: Location,
    next_destination: Location,
    ground_speed_vec: Vector2f,
    request_time_ms: u32,
}

#[derive(Debug, Clone, Copy)]
struct AvoidanceResult {
    destination: Location,
    next_destination: Location,
    origin_new: Location,
    destination_new: Location,
    next_destination_new: Location,
    dest_to_next_dest_clear: bool,
    result_time_ms: u32,
    path_planner_used: OaPathPlannerUsed,
    ret_state: OaRetState,
}

impl Default for AvoidanceResult {
    fn default() -> Self {
        Self {
            destination: Location::new(0, 0),
            next_destination: Location::new(0, 0),
            origin_new: Location::new(0, 0),
            destination_new: Location::new(0, 0),
            next_destination_new: Location::new(0, 0),
            dest_to_next_dest_clear: false,
            result_time_ms: 0,
            path_planner_used: OaPathPlannerUsed::None,
            ret_state: OaRetState::NotRequired,
        }
    }
}

/// OA path-planner frontend leftover. Upstream `AP_OAPathPlanner`.
#[derive(Debug, Clone)]
pub struct PathPlanner {
    /// `OA_TYPE`.
    plan_type: OaPathPlanType,
    /// `OA_MARGIN_MAX`, metres.
    margin_max_m: f32,
    /// `OA_OPTIONS` bitmask.
    options: u16,
    /// Leftover of `_thread_created` after a successful `init`.
    thread_created: bool,
    /// BendyRuler instance. `None` until [`PathPlanner::init`].
    bendy: Option<BendyRuler>,
    /// Dijkstra instance. `None` until [`PathPlanner::init`].
    dijkstra: Option<Dijkstra>,
    /// Combined-type proximity-only latch. Upstream `proximity_only`.
    proximity_only: bool,
    last_update_ms: u32,
    activated_ms: u32,
    avoidance_latest_ms: u32,
    request: Option<AvoidanceRequest>,
    result: AvoidanceResult,
}

impl Default for PathPlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl PathPlanner {
    /// Construct disabled, matching upstream defaults.
    #[must_use]
    pub fn new() -> Self {
        Self {
            plan_type: OaPathPlanType::Disabled,
            margin_max_m: MARGIN_MAX_M_DEFAULT,
            options: OPTIONS_DEFAULT,
            thread_created: false,
            bendy: None,
            dijkstra: None,
            proximity_only: true,
            last_update_ms: 0,
            activated_ms: 0,
            avoidance_latest_ms: 0,
            request: None,
            result: AvoidanceResult::default(),
        }
    }

    /// `OA_TYPE`.
    #[must_use]
    pub fn plan_type(&self) -> OaPathPlanType {
        self.plan_type
    }

    /// `OA_TYPE` setter (param load).
    pub fn set_plan_type(&mut self, plan_type: OaPathPlanType) {
        self.plan_type = plan_type;
    }

    /// `OA_MARGIN_MAX`.
    #[must_use]
    pub fn margin_max_m(&self) -> f32 {
        self.margin_max_m
    }

    /// `OA_MARGIN_MAX` setter.
    pub fn set_margin_max_m(&mut self, margin_max_m: f32) {
        self.margin_max_m = margin_max_m;
    }

    /// `OA_OPTIONS`.
    #[must_use]
    pub fn options(&self) -> u16 {
        self.options
    }

    /// `OA_OPTIONS` setter.
    pub fn set_options(&mut self, options: u16) {
        self.options = options;
        if let Some(dijkstra) = self.dijkstra.as_mut() {
            dijkstra.set_options(options);
        }
    }

    /// Leftover of `_thread_created`.
    #[must_use]
    pub fn thread_created(&self) -> bool {
        self.thread_created
    }

    /// BendyRuler instance after [`PathPlanner::init`].
    #[must_use]
    pub fn bendy(&self) -> Option<&BendyRuler> {
        self.bendy.as_ref()
    }

    /// Mutable BendyRuler after [`PathPlanner::init`].
    pub fn bendy_mut(&mut self) -> Option<&mut BendyRuler> {
        self.bendy.as_mut()
    }

    /// Dijkstra instance after [`PathPlanner::init`].
    #[must_use]
    pub fn dijkstra(&self) -> Option<&Dijkstra> {
        self.dijkstra.as_ref()
    }

    /// Mutable Dijkstra after [`PathPlanner::init`].
    pub fn dijkstra_mut(&mut self) -> Option<&mut Dijkstra> {
        self.dijkstra.as_mut()
    }

    /// Leftover of `AP_OAPathPlanner::init`.
    pub fn init(&mut self) {
        match self.plan_type {
            OaPathPlanType::Disabled => {
                self.thread_created = false;
                return;
            }
            OaPathPlanType::BendyRuler => {
                self.ensure_bendy();
            }
            OaPathPlanType::Dijkstra => {
                self.ensure_dijkstra();
            }
            OaPathPlanType::DijkstraBendyRuler => {
                self.ensure_dijkstra();
                self.ensure_bendy();
            }
        }
        self.thread_created = true;
    }

    fn ensure_bendy(&mut self) {
        if self.bendy.is_none() {
            let mut bendy = BendyRuler::new();
            bendy.set_lookahead_m(LOOKAHEAD_M_DEFAULT);
            self.bendy = Some(bendy);
        }
    }

    fn ensure_dijkstra(&mut self) {
        if self.dijkstra.is_none() {
            self.dijkstra = Some(Dijkstra::new(self.options));
        }
    }

    /// Leftover of `AP_OAPathPlanner::pre_arm_check`.
    #[must_use]
    pub fn pre_arm_check(&self) -> PreArmCheckLeftover {
        match self.plan_type {
            OaPathPlanType::Disabled => PreArmCheckLeftover {
                ok: true,
                failure_msg: "",
            },
            OaPathPlanType::BendyRuler => {
                if self.bendy.is_none() {
                    PreArmCheckLeftover {
                        ok: false,
                        failure_msg: "BendyRuler OA requires reboot",
                    }
                } else {
                    PreArmCheckLeftover {
                        ok: true,
                        failure_msg: "",
                    }
                }
            }
            OaPathPlanType::Dijkstra => {
                if self.dijkstra.is_none() {
                    PreArmCheckLeftover {
                        ok: false,
                        failure_msg: "Dijkstra OA requires reboot",
                    }
                } else {
                    PreArmCheckLeftover {
                        ok: true,
                        failure_msg: "",
                    }
                }
            }
            OaPathPlanType::DijkstraBendyRuler => {
                if self.dijkstra.is_none() || self.bendy.is_none() {
                    PreArmCheckLeftover {
                        ok: false,
                        failure_msg: "OA requires reboot",
                    }
                } else {
                    PreArmCheckLeftover {
                        ok: true,
                        failure_msg: "",
                    }
                }
            }
        }
    }

    /// Leftover of `map_bendytype_to_pathplannerused`.
    #[must_use]
    pub fn map_bendytype_to_pathplannerused(bendy_type: OaBendyType) -> OaPathPlannerUsed {
        match bendy_type {
            OaBendyType::Horizontal => OaPathPlannerUsed::BendyRulerHorizontal,
            OaBendyType::Vertical => OaPathPlannerUsed::BendyRulerVertical,
            OaBendyType::Disabled => OaPathPlannerUsed::None,
        }
    }

    /// Leftover of `AP_OAPathPlanner::mission_avoidance`.
    ///
    /// Stores a request for [`PathPlanner::process`]. Returns the latest
    /// matching result, [`OaRetState::Processing`], or [`OaRetState::Error`]
    /// on timeout. `ground_speed_ne_ms` is the leftover of
    /// `AP::ahrs().groundspeed_vector()`.
    #[must_use]
    pub fn mission_avoidance(
        &mut self,
        current_loc: Location,
        origin: Location,
        destination: Location,
        next_destination: Location,
        ground_speed_ne_ms: Vector2f,
        now_ms: u32,
    ) -> MissionAvoidanceLeftover {
        if self.plan_type == OaPathPlanType::Disabled || !self.thread_created {
            return MissionAvoidanceLeftover {
                ret_state: OaRetState::NotRequired,
                ..MissionAvoidanceLeftover::default()
            };
        }

        if now_ms.wrapping_sub(self.last_update_ms) > REACTIVATE_GAP_MS {
            self.activated_ms = now_ms;
        }
        self.last_update_ms = now_ms;

        self.request = Some(AvoidanceRequest {
            current_loc,
            origin,
            destination,
            next_destination,
            ground_speed_vec: ground_speed_ne_ms,
            request_time_ms: now_ms,
        });

        let destination_matches = same_latlon(destination, self.result.destination);
        let next_destination_matches = same_latlon(next_destination, self.result.next_destination);
        let timed_out = now_ms.wrapping_sub(self.result.result_time_ms) > TIMEOUT_MS
            && now_ms.wrapping_sub(self.activated_ms) > TIMEOUT_MS;

        if destination_matches && next_destination_matches && !timed_out {
            return MissionAvoidanceLeftover {
                ret_state: self.result.ret_state,
                result_origin: self.result.origin_new,
                result_destination: self.result.destination_new,
                result_next_destination: self.result.next_destination_new,
                dest_to_next_dest_clear: self.result.dest_to_next_dest_clear,
                path_planner_used: self.result.path_planner_used,
            };
        }

        if timed_out {
            return MissionAvoidanceLeftover {
                ret_state: OaRetState::Error,
                ..MissionAvoidanceLeftover::default()
            };
        }

        MissionAvoidanceLeftover {
            ret_state: OaRetState::Processing,
            ..MissionAvoidanceLeftover::default()
        }
    }

    /// Leftover of one `AP_OAPathPlanner::avoidance_thread` iteration.
    ///
    /// `yaw_deg` is the leftover of `AP::ahrs().get_yaw_deg` inside
    /// BendyRuler. `fence` is the leftover of `AP::fence()->polyfence()`
    /// inside Dijkstra.
    ///
    /// Returns `true` when a planner tick ran and wrote the result.
    pub fn process(
        &mut self,
        now_ms: u32,
        margin: &BendyMarginContext,
        yaw_deg: f32,
        fence: &DijkstraFenceContext,
    ) -> bool {
        if self.plan_type == OaPathPlanType::Disabled || !self.thread_created {
            return false;
        }
        if now_ms.wrapping_sub(self.avoidance_latest_ms) < UPDATE_MS {
            return false;
        }
        let Some(request) = self.request else {
            return false;
        };
        if now_ms.wrapping_sub(request.request_time_ms) > TIMEOUT_MS {
            return false;
        }
        self.avoidance_latest_ms = now_ms;

        let mut origin_new = request.origin;
        let mut destination_new = request.destination;
        let mut next_destination_new = request.next_destination;
        let mut dest_to_next_dest_clear = false;
        let mut res = OaRetState::NotRequired;
        let mut path_planner_used = OaPathPlannerUsed::None;

        match self.plan_type {
            OaPathPlanType::Disabled => return false,
            OaPathPlanType::BendyRuler => {
                let Some(bendy) = self.bendy.as_mut() else {
                    return false;
                };
                bendy.set_config(self.margin_max_m);
                let leftover = bendy.update(
                    request.current_loc,
                    request.destination,
                    request.ground_speed_vec,
                    yaw_deg,
                    margin,
                );
                origin_new = leftover.origin_new;
                destination_new = leftover.destination_new;
                if leftover.required {
                    res = OaRetState::Success;
                }
                path_planner_used = Self::map_bendytype_to_pathplannerused(leftover.bendy_type);
            }
            OaPathPlanType::Dijkstra => {
                let Some(dijkstra) = self.dijkstra.as_mut() else {
                    return false;
                };
                let leftover = Self::run_dijkstra(
                    dijkstra,
                    self.margin_max_m,
                    request.current_loc,
                    request.destination,
                    request.next_destination,
                    fence,
                );
                origin_new = leftover.origin_new;
                destination_new = leftover.destination_new;
                next_destination_new = leftover.next_destination_new;
                dest_to_next_dest_clear = leftover.dest_to_next_dest_clear;
                res = leftover.ret_state;
                path_planner_used = OaPathPlannerUsed::Dijkstras;
            }
            OaPathPlanType::DijkstraBendyRuler => {
                let leftover = {
                    let Some(bendy) = self.bendy.as_mut() else {
                        return false;
                    };
                    bendy.set_config(self.margin_max_m);
                    let mut margin_br = *margin;
                    margin_br.proximity_only = self.proximity_only;
                    bendy.update(
                        request.current_loc,
                        request.destination,
                        request.ground_speed_vec,
                        yaw_deg,
                        &margin_br,
                    )
                };
                if leftover.required {
                    self.proximity_only = false;
                    origin_new = leftover.origin_new;
                    destination_new = leftover.destination_new;
                    res = OaRetState::Success;
                    path_planner_used = Self::map_bendytype_to_pathplannerused(leftover.bendy_type);
                } else {
                    if !self.proximity_only {
                        if let Some(dijkstra) = self.dijkstra.as_mut() {
                            dijkstra.recalculate_path();
                        }
                    }
                    self.proximity_only = true;
                    let Some(dijkstra) = self.dijkstra.as_mut() else {
                        return false;
                    };
                    let dleft = Self::run_dijkstra(
                        dijkstra,
                        self.margin_max_m,
                        request.current_loc,
                        request.destination,
                        request.next_destination,
                        fence,
                    );
                    origin_new = dleft.origin_new;
                    destination_new = dleft.destination_new;
                    next_destination_new = dleft.next_destination_new;
                    dest_to_next_dest_clear = dleft.dest_to_next_dest_clear;
                    res = dleft.ret_state;
                    path_planner_used = OaPathPlannerUsed::Dijkstras;
                }
            }
        }

        self.result.destination = request.destination;
        self.result.next_destination = request.next_destination;
        self.result.dest_to_next_dest_clear = dest_to_next_dest_clear;
        self.result.origin_new = if res == OaRetState::Success {
            origin_new
        } else {
            self.result.origin_new
        };
        self.result.destination_new = if res == OaRetState::Success {
            destination_new
        } else {
            self.result.destination
        };
        self.result.next_destination_new = if res == OaRetState::Success {
            next_destination_new
        } else {
            self.result.next_destination
        };
        self.result.result_time_ms = now_ms;
        self.result.path_planner_used = path_planner_used;
        self.result.ret_state = res;
        true
    }

    fn run_dijkstra(
        dijkstra: &mut Dijkstra,
        margin_max_m: f32,
        current_loc: Location,
        destination: Location,
        next_destination: Location,
        fence: &DijkstraFenceContext,
    ) -> DijkstraTick {
        dijkstra.set_fence_margin(margin_max_m);
        let leftover = dijkstra.update(current_loc, destination, next_destination, fence);
        DijkstraTick {
            origin_new: leftover.origin_new,
            destination_new: leftover.destination_new,
            next_destination_new: leftover.next_destination_new,
            dest_to_next_dest_clear: leftover.dest_to_next_dest_clear,
            ret_state: match leftover.state {
                DijkstraState::NotRequired => OaRetState::NotRequired,
                DijkstraState::Error => OaRetState::Error,
                DijkstraState::Success => OaRetState::Success,
            },
        }
    }
}

struct DijkstraTick {
    origin_new: Location,
    destination_new: Location,
    next_destination_new: Location,
    dest_to_next_dest_clear: bool,
    ret_state: OaRetState,
}
