//! Parity test: AP_GPS time conversion against upstream.
//!
//! Two things, reached two ways.
//!
//! `BCD_to_gps_time` is a static member, called directly out of the linked
//! `GPS_Backend.cpp` object. It carries the whole chain: BCD field extraction,
//! `ap_mktime` underneath, the leap-second and epoch offsets, and the
//! narrowing to `uint32`.
//!
//! `istate_time_to_epoch_ms` lives in `AP_GPS.cpp`, which references most of
//! the vehicle. Linking it to reach a three-term addition would be absurd, so
//! the harness recomputes that expression from upstream's **own macros**,
//! included from `AP_GPS.h` rather than retyped. The only thing that could
//! differ is the value of those constants, and the fixture carries them
//! explicitly for that reason.
//!
//! # The two-digit year
//!
//! `BCD_to_gps_time` reads the year as `100 + date % 100`, i.e. always
//! 2000-something. It cannot express a date before 2000 at all — a `date` of
//! `60180` is 2080-01-06, not the GPS epoch of 1980-01-06. That is worth
//! knowing before reading the fixture, and it is the reason there is no week-0
//! row in it.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_gps::{
    bcd_to_gps_time, istate_time_to_epoch_ms, GPS_LEAPSECONDS_MILLIS, MSEC_PER_WEEK, SEC_PER_WEEK,
    UNIX_OFFSET_MSEC,
};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/gps_time_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_gps_fixture.py",
            path.display()
        )
    })
}

#[test]
fn gps_time_conversion_matches_upstream() {
    let text = fixture();
    let mut consts_seen = false;
    let mut epochs = 0_usize;
    let mut bcds = 0_usize;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 5, "malformed row: {line}");

        match row[0] {
            "consts" => {
                // The constants upstream's macros expand to. Everything else
                // in this file rests on these four numbers.
                assert_eq!(
                    row[1].parse::<u64>().expect("offset"),
                    UNIX_OFFSET_MSEC,
                    "UNIX_OFFSET_MSEC"
                );
                assert_eq!(
                    row[2].parse::<u64>().expect("msec/week"),
                    MSEC_PER_WEEK,
                    "AP_MSEC_PER_WEEK"
                );
                assert_eq!(
                    row[3].parse::<u64>().expect("sec/week"),
                    SEC_PER_WEEK,
                    "AP_SEC_PER_WEEK"
                );
                assert_eq!(
                    row[4].parse::<u64>().expect("leap"),
                    GPS_LEAPSECONDS_MILLIS,
                    "GPS_LEAPSECONDS_MILLIS"
                );
                consts_seen = true;
            }
            "epoch_ms" => {
                let week: u16 = row[1].parse::<u32>().expect("week") as u16;
                let ms: u32 = row[2].parse().expect("ms");
                let want: u64 = row[3].parse().expect("epoch ms");
                assert_eq!(
                    istate_time_to_epoch_ms(week, ms),
                    want,
                    "week {week} ms {ms}"
                );
                epochs += 1;
            }
            "bcd" => {
                let date: u32 = row[1].parse().expect("date");
                let time_ms: u32 = row[2].parse().expect("time");
                let want_week: u16 = row[3].parse::<u32>().expect("week") as u16;
                let want_tow: u32 = row[4].parse().expect("tow");

                let (week, tow) = bcd_to_gps_time(date, time_ms)
                    .unwrap_or_else(|| panic!("{date}/{time_ms} should convert"));
                assert_eq!(week, want_week, "week for {date}/{time_ms}");
                assert_eq!(tow, want_tow, "time of week for {date}/{time_ms}");
                bcds += 1;
            }
            other => panic!("unknown fixture kind {other}"),
        }
    }

    assert!(
        consts_seen,
        "the constants row is what everything else rests on"
    );
    assert!(epochs >= 10, "got {epochs} epoch conversions");
    assert!(bcds >= 15, "got {bcds} BCD conversions");
    println!(
        "bit-exact against upstream: 4 constants, {epochs} epoch conversions, {bcds} BCD dates"
    );
}

/// The two conversions are inverses across the fixture's own dates: a BCD date
/// turned into week and time of week, then into Unix epoch milliseconds,
/// should land on the instant the date names.
///
/// This is a property neither fixture row states on its own — it is the two
/// functions composed, which is how the vehicle actually uses them.
#[test]
fn a_bcd_date_reaches_the_right_unix_instant() {
    // 2024-02-29 12:34:56.789 UTC = 1709210096789 ms
    let (week, tow) = bcd_to_gps_time(290_224, 123_456_789).expect("a real date");
    assert_eq!(istate_time_to_epoch_ms(week, tow), 1_709_210_096_789);

    // 2025-03-01 00:00:00.000 UTC = 1740787200000 ms
    let (week, tow) = bcd_to_gps_time(10_325, 0).expect("a real date");
    assert_eq!(istate_time_to_epoch_ms(week, tow), 1_740_787_200_000);
}

/// The century rule reaches this far: 2000 is a leap year, so 2000-02-29
/// exists and is exactly one day after 2000-02-28.
#[test]
fn the_leap_day_of_2000_is_a_real_day() {
    let (w1, t1) = bcd_to_gps_time(280_200, 0).expect("2000-02-28");
    let (w2, t2) = bcd_to_gps_time(290_200, 0).expect("2000-02-29");
    let a = istate_time_to_epoch_ms(w1, t1);
    let b = istate_time_to_epoch_ms(w2, t2);
    assert_eq!(b - a, 86_400_000, "one day apart");
}
