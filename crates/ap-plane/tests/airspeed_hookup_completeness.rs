//! FW-010 completeness: params/hookups already on main vs remaining.

use ap_airspeed::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, this_slice_items, PortStatus, AIRSPEED_COMPLETENESS,
};

/// Hookups already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "SITL backend",
    "pre-arm",
    "health scheduler",
    "ARSPD_OFFSET",
    "ARSPD_RATIO",
    "dual-instance",
    "ARSPD_USE",
    "temperature compensation",
    "ARSPD_AUTOCAL",
    "ARSPD_PIN",
    "ARSPD_SKIP_CAL",
    "ARSPD_TYPE",
    "ARSPD_TUBE_ORDER",
    "ARSPD_BUS",
    "ARSPD_DEVID",
    "healthy-for-TECS",
    "ARSPD_OPTIONS",
    "ARSPD_WIND_MAX",
    "ARSPD_WIND_WARN",
    "ARSPD_PRIMARY",
    "ARSPD_FBW_MIN",
    "ARSPD_FBW_MAX",
    "ARSPD_PSI_RANGE",
];

const THIS_SLICE: &[&str] = &["ARSPD_OFF_PCNT", "completeness table"];

const REMAINING: &[&str] = &[
    "ARSPD_ENABLE",
    "ARSPD_WIND_GATE",
    "instances 3-6",
    "hardware backends",
    "log-replay",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(AIRSPEED_COMPLETENESS.len(), ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len());
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the closing-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is remaining / out of scope for the SITL stub close"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}
