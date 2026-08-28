//! ARSPD_OFF_PCNT stub: Plane-only offset-cal speed-error warning.

use ap_airspeed::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, PortStatus,
};
use ap_airspeed::fbw::ARSPD_FBW_MIN_DEFAULT;
use ap_airspeed::off_pcnt::ARSPD_OFF_PCNT_DEFAULT;
use ap_plane::airspeed_off_pcnt_hookup::{check_airspeed_off_pcnt, AirspeedOffPcntHookup};

#[test]
fn hookup_default_off_pcnt_disables_check() {
    let hookup = AirspeedOffPcntHookup::default();
    let published = hookup.publish(100.0, 0.0);
    assert_eq!(published.off_pcnt, ARSPD_OFF_PCNT_DEFAULT);
    assert!((published.airspeed_min - ARSPD_FBW_MIN_DEFAULT).abs() < 1e-6);
    assert!(!published.enabled);
    assert!(!published.exceeded);
}

#[test]
fn hookup_off_pcnt_flags_uncovered_pitot_offset_jump() {
    let mut hookup = AirspeedOffPcntHookup::default();
    hookup.set_off_pcnt(10);
    let fail = hookup.publish(10.0, 30.0);
    assert!(fail.enabled);
    assert!(fail.exceeded);
    let ok = hookup.publish(10.0, 12.0);
    assert!(!ok.exceeded);
    let gated = check_airspeed_off_pcnt(10.0, 30.0, 0, 9.0);
    assert!(!gated.exceeded);
}

#[test]
fn completeness_table_lists_main_versus_remaining() {
    assert!(completeness_unique_names());
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 23);
    assert_eq!(this_slice, 2);
    assert_eq!(remaining, 5);
    assert!(completeness_has("ARSPD_PSI_RANGE", PortStatus::OnMain));
    assert!(completeness_has("ARSPD_FBW_MIN", PortStatus::OnMain));
    assert!(completeness_has("ARSPD_PRIMARY", PortStatus::OnMain));
    assert!(completeness_has("ARSPD_OFF_PCNT", PortStatus::ThisSlice));
    assert!(completeness_has("ARSPD_WIND_GATE", PortStatus::Remaining));
}
