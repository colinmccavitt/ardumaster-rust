//! Completeness of FW-022 assisted/manual mode nav hookups.

use ap_plane::acro_mode_hookup::{acro_mode_nav_tick, AcroModeNavInputs};
use ap_plane::autotune_mode_hookup::{autotune_mode_nav_tick, AutotuneModeNavInputs};
use ap_plane::circle_mode_hookup::{circle_mode_nav_tick, CircleModeNavInputs};
use ap_plane::cruise_mode_hookup::{cruise_mode_nav_tick, CruiseModeNavInputs};
use ap_plane::fbwa_mode_hookup::{fbwa_mode_nav_tick, FbwaModeNavInputs};
use ap_plane::fbwb_mode_hookup::{fbwb_mode_nav_tick, FbwbModeNavInputs};
use ap_plane::manual_mode_hookup::{manual_mode_nav_tick, ManualModeNavInputs};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::mode_table_hookup::is_assisted_or_manual_mode;
use ap_plane::stabilize_mode_hookup::{stabilize_mode_nav_tick, StabilizeModeNavInputs};
use ap_plane::thermal_mode_hookup::{
    thermal_mode_nav_tick, ThermalModeNavInputs, SOAR_THML_BANK_DEFAULT_DEG,
};
use ap_plane::training_mode_hookup::{training_mode_nav_tick, TrainingModeNavInputs};

fn soaring_features() -> BuildFeatures {
    BuildFeatures {
        soaring: true,
        ..BuildFeatures::default()
    }
}

fn nav_hookup_applies(mode: ModeNumber) -> bool {
    let features = soaring_features();
    let n = mode.as_number();
    match mode {
        ModeNumber::Manual => {
            manual_mode_nav_tick(&ManualModeNavInputs {
                control_mode: n,
                features,
                roll_sensor_cd: 0,
                pitch_sensor_cd: 0,
            })
            .applied
        }
        ModeNumber::Circle => {
            circle_mode_nav_tick(&CircleModeNavInputs {
                control_mode: n,
                features,
                roll_limit_cd: 4500,
            })
            .applied
        }
        ModeNumber::Stabilize => {
            stabilize_mode_nav_tick(&StabilizeModeNavInputs {
                control_mode: n,
                features,
            })
            .applied
        }
        ModeNumber::Training => {
            training_mode_nav_tick(&TrainingModeNavInputs {
                control_mode: n,
                features,
                roll_sensor_cd: 0,
                pitch_sensor_cd: 0,
                roll_limit_cd: 4500,
                pitch_limit_min_cd: -2000,
                pitch_limit_max_cd: 2500,
            })
            .applied
        }
        ModeNumber::Acro => {
            acro_mode_nav_tick(&AcroModeNavInputs {
                control_mode: n,
                features,
                locked_roll: false,
                locked_pitch: false,
                locked_roll_err: 0.0,
                locked_pitch_cd: 0,
                roll_sensor_cd: 0,
                pitch_sensor_cd: 0,
            })
            .applied
        }
        ModeNumber::FlyByWireA => {
            fbwa_mode_nav_tick(&FbwaModeNavInputs {
                control_mode: n,
                features,
                roll_norm: 0.0,
                pitch_norm: 0.0,
                roll_limit_cd: 4500,
                pitch_limit_min_cd: -2000,
                pitch_limit_max_cd: 2500,
                roll_sensor_cd: 0,
            })
            .applied
        }
        ModeNumber::FlyByWireB => {
            fbwb_mode_nav_tick(&FbwbModeNavInputs {
                control_mode: n,
                features,
                roll_norm: 0.0,
                roll_limit_cd: 4500,
            })
            .applied
        }
        ModeNumber::Cruise => {
            cruise_mode_nav_tick(&CruiseModeNavInputs {
                control_mode: n,
                features,
                roll_norm: 0.0,
                rudder_norm: 0.0,
                locked_heading: false,
                nav_scripting_active: false,
                roll_limit_cd: 4500,
                commanded_roll_cd: 0,
            })
            .applied
        }
        ModeNumber::Autotune => {
            autotune_mode_nav_tick(&AutotuneModeNavInputs {
                control_mode: n,
                features,
                roll_norm: 0.0,
                pitch_norm: 0.0,
                roll_limit_cd: 4500,
                pitch_limit_min_cd: -2000,
                pitch_limit_max_cd: 2500,
                roll_sensor_cd: 0,
            })
            .applied
        }
        ModeNumber::Thermal => {
            thermal_mode_nav_tick(&ThermalModeNavInputs {
                control_mode: n,
                features,
                thermal_bank_deg: SOAR_THML_BANK_DEFAULT_DEG,
                roll_limit_cd: 4500,
            })
            .applied
        }
        _ => false,
    }
}

/// Every assisted/manual mode listed by the ticket, in mode-number order.
const ASSISTED_MANUAL_MODES: &[ModeNumber] = &[
    ModeNumber::Manual,
    ModeNumber::Circle,
    ModeNumber::Stabilize,
    ModeNumber::Training,
    ModeNumber::Acro,
    ModeNumber::FlyByWireA,
    ModeNumber::FlyByWireB,
    ModeNumber::Cruise,
    ModeNumber::Autotune,
    ModeNumber::Thermal,
];

#[test]
fn assisted_manual_mode_list_matches_classifier() {
    let listed: Vec<ModeNumber> = ASSISTED_MANUAL_MODES.to_vec();
    let classified: Vec<ModeNumber> = [
        ModeNumber::Manual,
        ModeNumber::Circle,
        ModeNumber::Stabilize,
        ModeNumber::Training,
        ModeNumber::Acro,
        ModeNumber::FlyByWireA,
        ModeNumber::FlyByWireB,
        ModeNumber::Cruise,
        ModeNumber::Autotune,
        ModeNumber::Auto,
        ModeNumber::Rtl,
        ModeNumber::Loiter,
        ModeNumber::Takeoff,
        ModeNumber::AvoidAdsb,
        ModeNumber::Guided,
        ModeNumber::Initialising,
        ModeNumber::QStabilize,
        ModeNumber::QHover,
        ModeNumber::QLoiter,
        ModeNumber::QLand,
        ModeNumber::QRtl,
        ModeNumber::QAutotune,
        ModeNumber::QAcro,
        ModeNumber::Thermal,
        ModeNumber::LoiterAltQLand,
        ModeNumber::Autoland,
    ]
    .into_iter()
    .filter(|m| is_assisted_or_manual_mode(*m))
    .collect();
    assert_eq!(listed, classified);
}

#[test]
fn every_assisted_or_manual_mode_has_nav_hookup() {
    for mode in ASSISTED_MANUAL_MODES {
        assert!(
            nav_hookup_applies(*mode),
            "{mode:?} is listed as assisted/manual but its nav hookup did not apply"
        );
    }
}

#[test]
fn autonomous_and_quadplane_modes_are_not_assisted() {
    for mode in [
        ModeNumber::Auto,
        ModeNumber::Rtl,
        ModeNumber::Loiter,
        ModeNumber::Takeoff,
        ModeNumber::AvoidAdsb,
        ModeNumber::Guided,
        ModeNumber::Initialising,
        ModeNumber::QStabilize,
        ModeNumber::QHover,
        ModeNumber::QLoiter,
        ModeNumber::QLand,
        ModeNumber::QRtl,
        ModeNumber::QAutotune,
        ModeNumber::QAcro,
        ModeNumber::LoiterAltQLand,
        ModeNumber::Autoland,
    ] {
        assert!(
            !is_assisted_or_manual_mode(mode),
            "{mode:?} must stay on FW-023 / quadplane, not FW-022"
        );
        assert!(
            !nav_hookup_applies(mode),
            "{mode:?} must not trip an assisted-mode nav hookup"
        );
    }
}
