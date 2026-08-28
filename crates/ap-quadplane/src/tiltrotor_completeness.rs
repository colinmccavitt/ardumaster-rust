//! VT-008 Tiltrotor completeness: surfaces already on main vs this leftover closer.
//!
//! Catalogs the `ArduPlane/tiltrotor.cpp` / `.h` port. Items marked
//! [`PortStatus::OnMain`] landed in earlier VT-008 slices and must not
//! be redone. [`PortStatus::ThisSlice`] is leftover `tiltrotor.cpp` /
//! `.h` surfaces stubbed here (`update` / compensate / bicopter /
//! `write_log` / `get_forward_throttle` / `Tiltrotor_Transition`).
//! [`PortStatus::Remaining`] is empty — this closer covers the leftover
//! public API as parameterized stubs (no live SRV / logger / heap).
//!
//! This module does not rewrite [`crate::completeness`] (VT-007
//! tailsitter), [`crate::logging`], [`crate::mode_qautotune`],
//! [`crate::tailsitter`], or the enable / slew / vectored-yaw mix
//! already on [`crate::tiltrotor`].

use crate::tiltrotor::{
    LOG_TILT_FIELDS, LOG_TILT_NAME, TiltType, Tiltrotor, TiltrotorConfig, TILT_SERVO_MAX,
};

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this leftover closer.
    OnMain,
    /// Added by this VT-008 leftover closer (stubs + this table).
    ThisSlice,
    /// Leftover `tiltrotor.cpp` / `.h` surface, not yet stubbed.
    Remaining,
}

/// One Tiltrotor surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TiltrotorPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported tiltrotor stubs vs leftover `tiltrotor.cpp` / `.h`.
///
/// On-main rows match the VT-008 slices already landed. This-slice rows
/// are the leftover behavioral surfaces. Remaining is empty.
pub const TILTROTOR_COMPLETENESS: &[TiltrotorPortItem] = &[
    TiltrotorPortItem {
        name: "enable / type",
        status: PortStatus::OnMain,
        note: "Tiltrotor::setup / enabled / TILT_TYPE_CONTINUOUS|BINARY|VECTORED_YAW|BICOPTER",
    },
    TiltrotorPortItem {
        name: "tilt-angle / slew",
        status: PortStatus::OnMain,
        note: "current_tilt / slew / Q_TILT_RATE_UP / Q_TILT_RATE_DN / tilt_max_change",
    },
    TiltrotorPortItem {
        name: "vectored-yaw / flap mix",
        status: PortStatus::OnMain,
        note: "vectoring_hover / vectoring_fw / get_forward_flight_tilt / Q_TILT_WING_FLAP",
    },
    TiltrotorPortItem {
        name: "fully_fwd / fully_up / tilt predicates",
        status: PortStatus::OnMain,
        note: "fully_fwd / fully_up / tilt_over_max_angle / tilt_angle_achieved",
    },
    TiltrotorPortItem {
        name: "update / continuous / binary",
        status: PortStatus::ThisSlice,
        note: "update / continuous_update / binary_update / binary_slew",
    },
    TiltrotorPortItem {
        name: "tilt_compensate",
        status: PortStatus::ThisSlice,
        note: "tilt_compensate / tilt_compensate_angle",
    },
    TiltrotorPortItem {
        name: "bicopter_output",
        status: PortStatus::ThisSlice,
        note: "Tiltrotor::bicopter_output SERVO_MAX mix",
    },
    TiltrotorPortItem {
        name: "write_log",
        status: PortStatus::ThisSlice,
        note: "Tiltrotor::write_log LOG_TILT_MSG TILT TimeUS,Tilt,FL,FR",
    },
    TiltrotorPortItem {
        name: "get_forward_throttle",
        status: PortStatus::ThisSlice,
        note: "Tiltrotor::get_forward_throttle tilting-motor average",
    },
    TiltrotorPortItem {
        name: "Tiltrotor_Transition",
        status: PortStatus::ThisSlice,
        note: "use_multirotor_control_in_fwd_transition / update_yaw_target / show_vtol_view / allow_vfwd",
    },
    TiltrotorPortItem {
        name: "tilt_max_change fast-tilt / flap-range",
        status: PortStatus::ThisSlice,
        note: "tilt_max_change in_flap_range + 90 DPS fast_tilt override",
    },
    TiltrotorPortItem {
        name: "is_motor_tilting / motors_active / has_*_motor",
        status: PortStatus::ThisSlice,
        note: "is_motor_tilting / motors_active / has_fw_motor / has_vtol_motor",
    },
    TiltrotorPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this leftover-complete catalog",
    },
];

/// Iterate on-main rows.
pub fn on_main_items() -> impl Iterator<Item = &'static TiltrotorPortItem> {
    TILTROTOR_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Iterate this-slice rows.
pub fn this_slice_items() -> impl Iterator<Item = &'static TiltrotorPortItem> {
    TILTROTOR_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Iterate remaining rows.
pub fn remaining_items() -> impl Iterator<Item = &'static TiltrotorPortItem> {
    TILTROTOR_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// `(on_main, this_slice, remaining)` counts.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    let mut i = 0;
    while i < TILTROTOR_COMPLETENESS.len() {
        match TILTROTOR_COMPLETENESS[i].status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
        i += 1;
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    let mut i = 0;
    while i < TILTROTOR_COMPLETENESS.len() {
        if TILTROTOR_COMPLETENESS[i].name == name && TILTROTOR_COMPLETENESS[i].status == status {
            return true;
        }
        i += 1;
    }
    false
}

/// True when every catalog name is unique.
#[must_use]
pub fn completeness_unique_names() -> bool {
    let mut i = 0;
    while i < TILTROTOR_COMPLETENESS.len() {
        let mut j = i + 1;
        while j < TILTROTOR_COMPLETENESS.len() {
            if TILTROTOR_COMPLETENESS[i].name == TILTROTOR_COMPLETENESS[j].name {
                return false;
            }
            j += 1;
        }
        i += 1;
    }
    true
}

/// True when every listed surface is `OnMain` or `ThisSlice`.
#[must_use]
pub fn tiltrotor_surfaces_complete() -> bool {
    remaining_items().next().is_none() && !TILTROTOR_COMPLETENESS.is_empty()
}

/// Upstream `LOG_TILT_MSG` field list (`"TimeUS,Tilt,FL,FR"`).
#[must_use]
pub const fn log_tilt_fields() -> &'static str {
    LOG_TILT_FIELDS
}

/// Upstream `LOG_TILT_MSG` name (`"TILT"`).
#[must_use]
pub const fn log_tilt_name() -> &'static str {
    LOG_TILT_NAME
}

/// Upstream `SERVO_MAX` used by [`Tiltrotor::bicopter_output`].
#[must_use]
pub const fn bicopter_servo_max() -> f32 {
    TILT_SERVO_MAX
}

/// Smoke the leftover catalog against a live [`Tiltrotor`].
#[must_use]
pub fn leftover_api_contract() -> bool {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    let mut cfg = TiltrotorConfig::with_tilt_mask(0b0011);
    cfg.tilt_type = TiltType::VectoredYaw as i8;
    cfg.enable = Some(1);
    let vectored = Tiltrotor::setup(cfg);
    tr.is_motor_tilting(0)
        && !tr.is_motor_tilting(4)
        && vectored.is_vectored()
        && tr.write_log(0.0, 0.0).is_some()
        && tiltrotor_surfaces_complete()
        && completeness_unique_names()
}
