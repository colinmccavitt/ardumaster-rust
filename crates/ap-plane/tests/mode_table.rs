//! Plane's mode-number table and mode-exit decision, against the firmware.
//!
//! The table takes one byte and all 256 inputs are recorded, so there is no
//! question of which were sampled — and no chance of a wrong table agreeing
//! with a sampled recording. That matters here: the numbers were guessed
//! wrong twice while writing this slice. `TAKEOFF` is 13 and `GUIDED` is 15,
//! which is not what the switch's ordering suggests.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::mode_table::{restores_autotune_gains, BuildFeatures, ModeNumber};

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_mode_table.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.starts_with(|c: char| c.is_alphabetic()) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// The build's feature set, as the firmware reported it rather than as the
/// port assumed it.
fn recorded_features() -> BuildFeatures {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_mode_table.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut features = BuildFeatures::default();
    let mut in_section = false;
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            in_section = tag == "features";
            continue;
        }
        if !in_section || line.starts_with("name,") {
            continue;
        }
        let Some((name, value)) = line.split_once(',') else {
            continue;
        };
        let on = value.trim() == "1";
        match name.trim() {
            "adsb" => features.adsb = on,
            "quadplane" => features.quadplane = on,
            "qautotune" => features.qautotune = on,
            "soaring" => features.soaring = on,
            "autoland" => features.autoland = on,
            other => panic!("unknown recorded feature {other}"),
        }
    }
    features
}

#[test]
fn the_mode_table_matches_upstream() {
    let rows = rows("table");
    assert_eq!(rows.len(), 256, "the table sweep should be exhaustive");

    let features = recorded_features();
    let mut valid = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 2, "malformed row");
        let number: u8 = r[0].trim().parse().expect("number");
        let want: i32 = r[1].trim().parse().expect("mode");

        let got = ModeNumber::from_number(number, &features);

        match (got, want) {
            (None, -1) => {}
            (Some(mode), w) => {
                assert_eq!(
                    i32::from(mode.as_number()),
                    w,
                    "number {number} maps to {mode:?} ({}), upstream says {w}",
                    mode.as_number()
                );
                valid += 1;
            }
            (None, w) => panic!("number {number}: the port has no mode, upstream has {w}"),
        }
    }

    assert!(valid > 20, "only {valid} numbers map to a mode");
    println!("{valid} of 256 numbers map to a mode, features {features:?}");
}

/// Every mode's number round-trips.
#[test]
fn the_mode_numbers_round_trip() {
    let all = [
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
    ];

    // Every feature on, which is the build the recording came from.
    let features = BuildFeatures {
        adsb: true,
        quadplane: true,
        qautotune: true,
        soaring: true,
        autoland: true,
    };

    let mut seen = std::collections::BTreeSet::new();
    for mode in all {
        let number = mode.as_number();
        assert!(seen.insert(number), "number {number} is used twice");
        assert_eq!(
            ModeNumber::from_number(number, &features),
            Some(mode),
            "{mode:?} did not survive a round trip through {number}"
        );
    }

    // 9 is not a mode, and never was. Its absence is upstream's, not an
    // oversight here.
    assert!(!seen.contains(&9), "9 should not be a mode number");
    assert_eq!(ModeNumber::from_number(9, &features), None);
}

/// Turning a feature off removes exactly its own modes.
///
/// The recording is from a build with everything enabled, so this is the half
/// it cannot show. Each flag is dropped in turn and the numbers that
/// disappear are checked to be that feature's and no others — a gate written
/// one case too wide would take a neighbouring mode with it.
#[test]
fn each_feature_gates_only_its_own_modes() {
    let all_on = BuildFeatures {
        adsb: true,
        quadplane: true,
        qautotune: true,
        soaring: true,
        autoland: true,
    };

    let cases: [(&str, BuildFeatures, &[u8]); 4] = [
        (
            "quadplane",
            BuildFeatures {
                quadplane: false,
                ..all_on
            },
            &[17, 18, 19, 20, 21, 22, 23, 25],
        ),
        (
            "qautotune",
            BuildFeatures {
                qautotune: false,
                ..all_on
            },
            &[22],
        ),
        (
            "soaring",
            BuildFeatures {
                soaring: false,
                ..all_on
            },
            &[24],
        ),
        (
            "autoland",
            BuildFeatures {
                autoland: false,
                ..all_on
            },
            &[26],
        ),
    ];

    for (name, features, expected_gone) in cases {
        for number in 0..=u8::MAX {
            let before = ModeNumber::from_number(number, &all_on);
            let after = ModeNumber::from_number(number, &features);
            let disappeared = before.is_some() && after.is_none();
            assert_eq!(
                disappeared,
                expected_gone.contains(&number),
                "with {name} off, number {number}: disappeared={disappeared}"
            );
        }
    }
}

/// Without ADSB, 14 means Guided rather than nothing.
///
/// Upstream's `AVOID_ADSB` case has its `break` inside the `#if`, so the case
/// falls through to `GUIDED`. It is the one number in the table whose meaning
/// depends on the build rather than merely its validity.
///
/// The recording cannot show this: the firmware was built with ADSB enabled,
/// so 14 is `AvoidAdsb` in every recorded row. Pinned here instead, and
/// labelled — a build with ADSB off would let the recording take over.
#[test]
fn without_adsb_the_avoidance_number_becomes_guided() {
    let with_adsb = BuildFeatures {
        adsb: true,
        quadplane: true,
        qautotune: true,
        soaring: true,
        autoland: true,
    };
    let without = BuildFeatures {
        adsb: false,
        ..with_adsb
    };

    assert_eq!(
        ModeNumber::from_number(14, &with_adsb),
        Some(ModeNumber::AvoidAdsb)
    );
    assert_eq!(
        ModeNumber::from_number(14, &without),
        Some(ModeNumber::Guided),
        "with ADSB off, 14 should fall through to Guided rather than refuse"
    );

    // And 15 is Guided either way, so the fallthrough adds a meaning rather
    // than moving one.
    assert_eq!(
        ModeNumber::from_number(15, &with_adsb),
        Some(ModeNumber::Guided)
    );
    assert_eq!(
        ModeNumber::from_number(15, &without),
        Some(ModeNumber::Guided)
    );
}

#[test]
fn the_autotune_restore_matches_upstream() {
    let rows = rows("exit");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut restored = 0_usize;
    let features = recorded_features();

    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let entered_num: u8 = r[1].trim().parse().expect("entered");
        let entered = ModeNumber::from_number(entered_num, &features).expect("entered mode");

        let got = restores_autotune_gains(entered);
        let want = r[3].trim() == "1";
        assert_eq!(
            got, want,
            "row {idx}: entering {entered:?}, restore={got} against upstream {want}"
        );
        if want {
            restored += 1;
        }
    }

    assert!(
        restored > 0 && restored < rows.len(),
        "the restore never varies across {} rows",
        rows.len()
    );
    println!("{} exit rows, {restored} restored the gains", rows.len());
}

/// The decision reads the mode being entered, not the one being left.
///
/// `set_mode` assigns `control_mode` before calling `old_mode.exit()`, so the
/// comparison is against the new mode. Reading it the other way would restore
/// the gains on every exit *from* autotune — discarding a tune the moment it
/// finished, which is exactly backwards.
#[test]
fn the_restore_follows_the_mode_being_entered() {
    // Leaving autotune for anything else: put the original gains back.
    assert!(restores_autotune_gains(ModeNumber::Manual));
    assert!(restores_autotune_gains(ModeNumber::FlyByWireA));

    // Entering autotune: keep the tuned gains, whatever was left behind.
    assert!(!restores_autotune_gains(ModeNumber::Autotune));
}
