//! `AC_PrecLand_StateMachine` leftover, upstream
//! `libraries/AC_PrecLand/AC_PrecLand_StateMachine.{h,cpp}`.
//!
//! Tracked as **COP-028**. ADR-0004 forbids `AP::ac_precland()` and
//! `AP::ahrs()`. The vehicle injects a [`StateMachineFrontend`] snapshot
//! and a [`StateMachineWorld`]. `GCS_SEND_TEXT` stays a leftover flag.

use ap_math::vector3::Vector3f;

use crate::precland::TargetState;

/// Maximum position error for retry locations. Upstream `MAX_POS_ERROR_M`.
pub const MAX_POS_ERROR_M: f32 = 0.75;
/// Timeout before failsafe measures start. Upstream `FAILSAFE_INIT_TIMEOUT_MS`.
pub const FAILSAFE_INIT_TIMEOUT_MS: u32 = 7_000;
/// Added to the retry location altitude (NED down, so subtracted from z).
/// Upstream `RETRY_OFFSET_ALT_M`.
pub const RETRY_OFFSET_ALT_M: f32 = 1.5;

/// Default `PLND_STRICT`. Upstream `var_info` `STRICT` = 1.
pub const STRICT_DEFAULT: RetryStrictness = RetryStrictness::Normal;
/// Default `PLND_RET_MAX`. Upstream `var_info` `RET_MAX` = 4.
pub const RETRY_MAX_DEFAULT: u8 = 4;
/// Default `PLND_TIMEOUT` seconds. Upstream `var_info` `TIMEOUT` = 4.
pub const RETRY_TIMEOUT_S_DEFAULT: f32 = 4.0;
/// Default `PLND_RET_BEHAVE`. Upstream `var_info` `RET_BEHAVE` = 0.
pub const RETRY_BEHAVE_DEFAULT: RetryAction = RetryAction::GoToLastLoc;

/// Current status of the precland state machine.
/// Upstream `AC_PrecLand_StateMachine::Status`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    /// Unknown error. Upstream `ERROR`.
    Error = 0,
    /// No action required, descend vertically. Upstream `DESCEND`.
    Descend = 1,
    /// Vehicle is attempting to retry landing. Upstream `RETRYING`.
    Retrying = 2,
    /// Switch to prec landing failsafe. Upstream `FAILSAFE`.
    Failsafe = 3,
}

/// Failsafe action needed. Upstream `AC_PrecLand_StateMachine::FailSafeAction`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailSafeAction {
    /// Hold the current position. Upstream `HOLD_POS`.
    HoldPos = 0,
    /// Descend vertically. Upstream `DESCEND`.
    Descend = 1,
}

/// Strictness the user wants for prec landing.
/// Upstream `AC_PrecLand_StateMachine::RetryStrictness`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RetryStrictness {
    /// Land ASAP whether the target is in sight or not. Upstream `NOT_STRICT`.
    NotStrict = 0,
    /// Retry a failed prec landing; land vertically if the target is not
    /// found. Upstream `NORMAL`.
    Normal = 1,
    /// Never land if the target is not found. Upstream `VERY_STRICT`.
    VeryStrict = 2,
}

/// Which retry action should be done.
/// Upstream `AC_PrecLand_StateMachine::RetryAction`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RetryAction {
    /// Go to the last location where the landing target was detected.
    /// Upstream `GO_TO_LAST_LOC`.
    GoToLastLoc = 0,
    /// Go towards the location of the detected landing target.
    /// Upstream `GO_TO_TARGET_LOC`.
    GoToTargetLoc = 1,
}

/// Action when the landing target is lost.
/// Upstream `AC_PrecLand_StateMachine::TargetLostAction`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum TargetLostAction {
    Init = 0,
    Descend = 1,
    LandVertically = 2,
    RetryLanding = 3,
}

/// Landing-retry submachine. Upstream `AC_PrecLand_StateMachine::RetryLanding`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum RetryLanding {
    Init = 0,
    InProgress = 1,
    Descend = 2,
    Complete = 3,
}

/// Snapshot of `AC_PrecLand` the state machine reads.
///
/// Leftover of `AP::ac_precland()` plus the getters
/// `enabled`, `get_target_state`, `get_retry_strictness`,
/// `get_last_valid_target_ms`, `get_min_retry_time_sec`,
/// `get_max_retry_allowed`, `get_retry_behaviour`,
/// `get_last_detected_landing_pos_NED_m`, and
/// `get_last_vehicle_pos_when_target_detected_NED_m`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateMachineFrontend {
    /// `enabled()`. When false, `update` returns [`Status::Error`]
    /// (same as a null singleton).
    pub enabled: bool,
    /// `get_target_state()`.
    pub target_state: TargetState,
    /// `get_retry_strictness()` / `PLND_STRICT`.
    pub retry_strictness: RetryStrictness,
    /// `get_last_valid_target_ms()`.
    pub last_valid_target_ms: u32,
    /// `get_min_retry_time_sec()` / `PLND_TIMEOUT`.
    pub min_retry_time_sec: f32,
    /// `get_max_retry_allowed()` / `PLND_RET_MAX`.
    pub max_retry_allowed: u8,
    /// `get_retry_behaviour()` / `PLND_RET_BEHAVE`.
    pub retry_behaviour: RetryAction,
    /// `get_last_detected_landing_pos_NED_m()`.
    pub last_detected_landing_pos_ned_m: Vector3f,
    /// `get_last_vehicle_pos_when_target_detected_NED_m()`.
    pub last_vehicle_pos_when_target_detected_ned_m: Vector3f,
}

impl Default for StateMachineFrontend {
    fn default() -> Self {
        Self {
            enabled: true,
            target_state: TargetState::NeverSeen,
            retry_strictness: STRICT_DEFAULT,
            last_valid_target_ms: 0,
            min_retry_time_sec: RETRY_TIMEOUT_S_DEFAULT,
            max_retry_allowed: RETRY_MAX_DEFAULT,
            retry_behaviour: RETRY_BEHAVE_DEFAULT,
            last_detected_landing_pos_ned_m: Vector3f::zero(),
            last_vehicle_pos_when_target_detected_ned_m: Vector3f::zero(),
        }
    }
}

/// AHRS / clock leftover. Upstream `AP_HAL::millis()` and
/// `AP::ahrs().get_relative_position_NED_origin`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateMachineWorld {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `get_relative_position_NED_origin`. `None` is the leftover of
    /// a failed AHRS origin (retry returns [`Status::Error`]).
    pub relative_pos_ned: Option<Vector3f>,
}

impl Default for StateMachineWorld {
    fn default() -> Self {
        Self {
            now_ms: 0,
            relative_pos_ned: None,
        }
    }
}

/// What `AC_PrecLand_StateMachine::update` decided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateMachineUpdate {
    /// Action the vehicle should take.
    pub status: Status,
    /// Filled when a retry path wrote `retry_pos_m`. `None` when this
    /// tick did not write a retry location (C++ leaves the caller's
    /// vector unchanged).
    pub retry_pos_m: Option<Vector3f>,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Retrying")`.
    pub need_gcs_retrying: bool,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Retry Completed")`.
    pub need_gcs_retry_completed: bool,
}

/// What `AC_PrecLand_StateMachine::get_failsafe_actions` decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FailSafeLeftover {
    /// Action the vehicle should take.
    pub action: FailSafeAction,
    /// Leftover of `GCS_SEND_TEXT(..., "PrecLand: Failsafe Measures")`.
    /// Set only the first time failsafe is initialised.
    pub need_gcs_failsafe: bool,
}

/// Retry / failsafe state machine, upstream `AC_PrecLand_StateMachine`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateMachine {
    landing_target_lost_action: TargetLostAction,
    retry_state: RetryLanding,
    retry_count: u8,
    failsafe_initialized: bool,
    failsafe_start_ms: u32,
}

impl Default for StateMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl StateMachine {
    /// Construct. Upstream constructor calls [`Self::init`]; without a
    /// frontend this only zeros the machines (same as a disabled
    /// singleton early-return).
    #[must_use]
    pub fn new() -> Self {
        Self {
            landing_target_lost_action: TargetLostAction::Init,
            retry_state: RetryLanding::Init,
            retry_count: 0,
            failsafe_initialized: false,
            failsafe_start_ms: 0,
        }
    }

    /// Initialize the state machine. Called every time the vehicle
    /// switches mode. Upstream `AC_PrecLand_StateMachine::init`.
    ///
    /// Early-returns when precland is not enabled, so a disabled call
    /// does not reset the retry counter.
    pub fn init(&mut self, frontend: &StateMachineFrontend) {
        if !frontend.enabled {
            return;
        }
        self.retry_count = 0;
        self.reset_failed_landing_statemachine();
    }

    /// Total retries done in this mode. Upstream `_retry_count`.
    #[must_use]
    pub fn retry_count(&self) -> u8 {
        self.retry_count
    }

    /// Run the prec land state machine. Upstream
    /// `AC_PrecLand_StateMachine::update`.
    #[must_use]
    pub fn update(
        &mut self,
        frontend: &StateMachineFrontend,
        world: &StateMachineWorld,
    ) -> StateMachineUpdate {
        if !frontend.enabled {
            return StateMachineUpdate {
                status: Status::Error,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            };
        }

        match frontend.target_state {
            TargetState::RecentlyLost => self.get_target_lost_actions(frontend, world),
            TargetState::NeverSeen => StateMachineUpdate {
                status: Status::Failsafe,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            },
            TargetState::OutOfRange | TargetState::Found => {
                self.reset_failed_landing_statemachine();
                StateMachineUpdate {
                    status: Status::Descend,
                    retry_pos_m: None,
                    need_gcs_retrying: false,
                    need_gcs_retry_completed: false,
                }
            }
        }
    }

    /// Failsafe action. Upstream `get_failsafe_actions`.
    ///
    /// Called when [`Status::Failsafe`] is current. A disabled frontend
    /// matches a null singleton and descends.
    #[must_use]
    pub fn get_failsafe_actions(
        &mut self,
        frontend: &StateMachineFrontend,
        world: &StateMachineWorld,
    ) -> FailSafeLeftover {
        if !frontend.enabled {
            return FailSafeLeftover {
                action: FailSafeAction::Descend,
                need_gcs_failsafe: false,
            };
        }

        let mut need_gcs_failsafe = false;
        if !self.failsafe_initialized {
            self.failsafe_start_ms = world.now_ms;
            self.failsafe_initialized = true;
            need_gcs_failsafe = true;
        }

        let action = match frontend.retry_strictness {
            RetryStrictness::VeryStrict => FailSafeAction::HoldPos,
            RetryStrictness::Normal => {
                if world
                    .now_ms
                    .wrapping_sub(self.failsafe_start_ms)
                    < FAILSAFE_INIT_TIMEOUT_MS
                {
                    FailSafeAction::HoldPos
                } else {
                    FailSafeAction::Descend
                }
            }
            RetryStrictness::NotStrict => FailSafeAction::Descend,
        };

        FailSafeLeftover {
            action,
            need_gcs_failsafe,
        }
    }

    fn reset_failed_landing_statemachine(&mut self) {
        self.landing_target_lost_action = TargetLostAction::Init;
        self.retry_state = RetryLanding::Init;
        self.failsafe_initialized = false;
    }

    fn get_target_lost_actions(
        &mut self,
        frontend: &StateMachineFrontend,
        world: &StateMachineWorld,
    ) -> StateMachineUpdate {
        match self.landing_target_lost_action {
            TargetLostAction::Init => {
                self.landing_target_lost_action = match frontend.retry_strictness {
                    RetryStrictness::Normal | RetryStrictness::VeryStrict => {
                        TargetLostAction::Descend
                    }
                    RetryStrictness::NotStrict => TargetLostAction::LandVertically,
                };
                StateMachineUpdate {
                    status: Status::Descend,
                    retry_pos_m: None,
                    need_gcs_retrying: false,
                    need_gcs_retry_completed: false,
                }
            }
            TargetLostAction::Descend => {
                let elapsed = world.now_ms.wrapping_sub(frontend.last_valid_target_ms) as f32;
                if elapsed >= frontend.min_retry_time_sec * 1000.0 {
                    self.landing_target_lost_action = TargetLostAction::RetryLanding;
                    self.retry_state = RetryLanding::Init;
                }
                StateMachineUpdate {
                    status: Status::Descend,
                    retry_pos_m: None,
                    need_gcs_retrying: false,
                    need_gcs_retry_completed: false,
                }
            }
            TargetLostAction::RetryLanding => self.retry_landing(frontend, world),
            TargetLostAction::LandVertically => StateMachineUpdate {
                status: Status::Descend,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            },
        }
    }

    fn retry_landing(
        &mut self,
        frontend: &StateMachineFrontend,
        world: &StateMachineWorld,
    ) -> StateMachineUpdate {
        if frontend.max_retry_allowed == 0 {
            return StateMachineUpdate {
                status: Status::Failsafe,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            };
        }

        if self.retry_count > frontend.max_retry_allowed {
            return StateMachineUpdate {
                status: Status::Failsafe,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            };
        }

        let mut go_to_pos = match frontend.retry_behaviour {
            RetryAction::GoToTargetLoc => frontend.last_detected_landing_pos_ned_m,
            RetryAction::GoToLastLoc => frontend.last_vehicle_pos_when_target_detected_ned_m,
        };
        go_to_pos.z -= RETRY_OFFSET_ALT_M;

        match self.retry_state {
            RetryLanding::Init => {
                self.retry_count = self.retry_count.wrapping_add(1);
                self.retry_state = RetryLanding::InProgress;
                let need_gcs_retrying = self.retry_count <= frontend.max_retry_allowed;
                StateMachineUpdate {
                    status: Status::Retrying,
                    retry_pos_m: Some(go_to_pos),
                    need_gcs_retrying,
                    need_gcs_retry_completed: false,
                }
            }
            RetryLanding::InProgress => {
                let Some(pos) = world.relative_pos_ned else {
                    return StateMachineUpdate {
                        status: Status::Error,
                        retry_pos_m: Some(go_to_pos),
                        need_gcs_retrying: false,
                        need_gcs_retry_completed: false,
                    };
                };
                let delta = Vector3f::new(
                    go_to_pos.x - pos.x,
                    go_to_pos.y - pos.y,
                    go_to_pos.z - pos.z,
                );
                if delta.length() < MAX_POS_ERROR_M {
                    self.retry_state = RetryLanding::Descend;
                }
                StateMachineUpdate {
                    status: Status::Retrying,
                    retry_pos_m: Some(go_to_pos),
                    need_gcs_retrying: false,
                    need_gcs_retry_completed: false,
                }
            }
            RetryLanding::Descend => {
                let Some(pos) = world.relative_pos_ned else {
                    return StateMachineUpdate {
                        status: Status::Error,
                        retry_pos_m: None,
                        need_gcs_retrying: false,
                        need_gcs_retry_completed: false,
                    };
                };
                let z_target = go_to_pos.z + RETRY_OFFSET_ALT_M;
                let retry_pos_m = Vector3f::new(pos.x, pos.y, z_target);
                let mut need_gcs_retry_completed = false;
                if libm::fabsf(pos.z - retry_pos_m.z) < MAX_POS_ERROR_M {
                    self.retry_state = RetryLanding::Complete;
                    need_gcs_retry_completed = true;
                }
                StateMachineUpdate {
                    status: Status::Retrying,
                    retry_pos_m: Some(retry_pos_m),
                    need_gcs_retrying: false,
                    need_gcs_retry_completed,
                }
            }
            RetryLanding::Complete => StateMachineUpdate {
                status: Status::Failsafe,
                retry_pos_m: None,
                need_gcs_retrying: false,
                need_gcs_retry_completed: false,
            },
        }
    }
}
