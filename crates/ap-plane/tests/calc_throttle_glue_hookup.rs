use ap_plane::calc_throttle_glue_hookup::{apply_throttle_nudge, calc_throttle_glue_tick, CalcThrottleGlueInputs};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

#[test]
fn nudge_clamps_at_ceiling() {
    assert!((apply_throttle_nudge(98.0, 5) - 100.0).abs() < 1e-6);
}

#[test]
fn fbwb_uses_tecs_demand() {
    let thr = calc_throttle_glue_tick(&CalcThrottleGlueInputs {
        control_mode: ModeNumber::FlyByWireB.as_number(),
        features: BuildFeatures::default(),
        tecs_throttle_demand: 42.0,
        throttle_nudge: 0,
        pilot_throttle: Default::default(),
    });
    assert!((thr - 42.0).abs() < 1e-6);
}
