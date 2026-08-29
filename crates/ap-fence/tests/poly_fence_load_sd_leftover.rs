//! `load_from_storage` and SD-card init leftovers.
//!
//! Tracked as **COP-025**.

use ap_fence::{
    format_storage, init_sdcard_storage, scale_latlon_from_origin, sdcard_fence_filename,
    write_f32_to_storage, write_fence, write_latlon_to_storage, write_type_to_storage,
    write_uint8_to_storage, FenceIndex, LoadFromStorageContext, PolyFence, PolyFenceItem,
    PolyFenceType, SdcardFenceContext, SDCARD_FENCE_FILENAME, SDCARD_FENCE_FILENAME_CHIBIOS,
};
use ap_math::location::Location;

fn empty_index() -> [FenceIndex; 16] {
    [FenceIndex::EMPTY; 16]
}

fn origin() -> Location {
    Location::new(0, 0)
}

fn load_ctx(now_ms: u32) -> LoadFromStorageContext {
    LoadFromStorageContext {
        origin: Some(origin()),
        now_ms,
    }
}

#[test]
fn sdcard_filenames_match_upstream_defines() {
    assert_eq!(SDCARD_FENCE_FILENAME_CHIBIOS, "APM/fence.stg");
    assert_eq!(SDCARD_FENCE_FILENAME, "fence.stg");
    assert_eq!(sdcard_fence_filename(true), "APM/fence.stg");
    assert_eq!(sdcard_fence_filename(false), "fence.stg");
}

#[test]
fn sdcard_init_skips_attach_when_size_kb_is_zero() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let leftover = init_sdcard_storage(
        SdcardFenceContext {
            board_config_present: true,
            size_kb: 0,
            attach_ok: false,
        },
        12,
        &buf,
        &mut empty_index(),
    );
    assert!(!leftover.attach_attempted);
    assert!(!leftover.failed_sdcard_storage);
    assert!(!leftover.total_wiped);
    assert_eq!(leftover.old_total, 12);
    assert!(leftover.indexed);
}

#[test]
fn sdcard_init_skips_attach_when_board_config_is_missing() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let leftover = init_sdcard_storage(
        SdcardFenceContext {
            board_config_present: false,
            size_kb: 64,
            attach_ok: false,
        },
        8,
        &buf,
        &mut empty_index(),
    );
    assert!(!leftover.attach_attempted);
    assert!(!leftover.failed_sdcard_storage);
    assert!(!leftover.total_wiped);
    assert_eq!(leftover.old_total, 8);
}

#[test]
fn sdcard_attach_failure_wipes_total_without_saving() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let leftover = init_sdcard_storage(
        SdcardFenceContext {
            board_config_present: true,
            size_kb: 32,
            attach_ok: false,
        },
        20,
        &buf,
        &mut empty_index(),
    );
    assert!(leftover.attach_attempted);
    assert!(leftover.failed_sdcard_storage);
    assert!(leftover.total_wiped);
    assert_eq!(leftover.old_total, 0);
    assert!(leftover.indexed);
}

#[test]
fn sdcard_attach_ok_keeps_total_and_indexes() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let mut fence = PolyFence::new();
    let leftover = fence.init(
        SdcardFenceContext {
            board_config_present: true,
            size_kb: 16,
            attach_ok: true,
        },
        7,
        &buf,
        &mut empty_index(),
    );
    assert!(leftover.attach_attempted);
    assert!(!leftover.failed_sdcard_storage);
    assert!(!leftover.total_wiped);
    assert_eq!(leftover.old_total, 7);
    assert!(!fence.failed_sdcard_storage());
    assert!(leftover.indexed);
}

#[test]
fn scale_latlon_from_origin_is_get_distance_ne_times_100() {
    let origin = Location::new(1_000_000, 2_000_000);
    let pos = scale_latlon_from_origin(origin, 1_100_000, 2_050_000);
    let expected = origin.get_distance_ne(Location::new(1_100_000, 2_050_000)) * 100.0;
    assert_eq!(pos.x, expected.x);
    assert_eq!(pos.y, expected.y);
}

#[test]
fn load_unformatted_store_fails_without_attempting() {
    let buf = [0u8; 16];
    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(1_000));
    assert!(!leftover.ok);
    assert!(leftover.index_failed);
    assert!(!leftover.load_attempted);
    assert!(!fence.loaded());
}

#[test]
fn missing_origin_does_not_set_load_attempted() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let mut fence = PolyFence::new();
    let miss = fence.load_from_storage(
        &buf,
        &mut empty_index(),
        LoadFromStorageContext {
            origin: None,
            now_ms: 50,
        },
    );
    assert!(!miss.ok);
    assert!(miss.origin_missing);
    assert!(!miss.load_attempted);
    assert_eq!(miss.load_time_ms, 0);

    let ok = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(99));
    assert!(ok.ok);
    assert!(ok.empty);
    assert!(ok.load_attempted);
    assert_eq!(ok.load_time_ms, 99);
    assert!(fence.loaded());
}

#[test]
fn empty_formatted_store_loads_and_sets_load_time() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(1_234));
    assert!(leftover.ok);
    assert!(leftover.empty);
    assert_eq!(leftover.load_time_ms, 1_234);
    assert_eq!(fence.load_time_ms(), 1_234);
    assert_eq!(fence.total_fence_count(), 0);
}

#[test]
fn already_attempted_load_returns_previous_result_without_reread() {
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    let mut fence = PolyFence::new();
    let first = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(10));
    assert!(first.ok);

    // Corrupt the store after the first walk. A second call must not re-read.
    buf[4] = 7;
    let second = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(99));
    assert!(second.ok);
    assert!(second.already_attempted);
    assert_eq!(second.load_time_ms, 10);
}

#[test]
fn load_packs_circle_return_and_polygon() {
    let items = [
        PolyFenceItem::return_point(1_000, 2_000),
        PolyFenceItem::circle(PolyFenceType::CircleInclusion, 3_000, 4_000, 250.0),
        PolyFenceItem::circle(PolyFenceType::CircleExclusion, 5_000, 6_000, 40.0),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 10, 20, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 11, 21, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 12, 22, 3),
    ];
    let mut buf = [0u8; 80];
    write_fence(&mut buf, &items).expect("write");

    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(7));
    assert!(leftover.ok);
    assert!(!leftover.empty);
    assert_eq!(leftover.inclusion_circles, 1);
    assert_eq!(leftover.exclusion_circles, 1);
    assert_eq!(leftover.inclusion_polygons, 1);
    assert_eq!(leftover.exclusion_polygons, 0);
    assert_eq!(
        leftover.return_point.map(|v| (v.lat, v.lng)),
        Some((1_000, 2_000))
    );
    assert_eq!(
        fence.return_point().map(|v| (v.lat, v.lng)),
        Some((1_000, 2_000))
    );
    assert_eq!(fence.inclusion_circle_count(), 1);
    assert_eq!(fence.exclusion_circle_count(), 1);
    assert_eq!(fence.inclusion_polygon_count(), 1);
}

#[test]
fn load_integer_radius_circle_reads_uint32() {
    let mut buf = [0u8; 32];
    format_storage(&mut buf).expect("format");
    let mut offset = 4_u16;
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::CircleInclusionInt
    ));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 100, 200));
    // uint32 radius 75, little-endian — leftover of read_uint32.
    let radius = 75_u32.to_le_bytes();
    let at = usize::from(offset);
    for (k, byte) in radius.iter().enumerate() {
        if let Some(slot) = buf.get_mut(at + k) {
            *slot = *byte;
        }
    }
    offset = offset.saturating_add(4);
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::EndOfStorage
    ));

    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(3));
    assert!(leftover.ok);
    assert_eq!(leftover.inclusion_circles, 1);
    assert!(fence.check_inclusion_circle_margin(74.0));
    assert!(!fence.check_inclusion_circle_margin(76.0));
}

#[test]
fn non_positive_circle_radius_fails_load() {
    let mut buf = [0u8; 32];
    format_storage(&mut buf).expect("format");
    let mut offset = 4_u16;
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::CircleExclusion
    ));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 0, 0));
    assert!(write_f32_to_storage(&mut buf, &mut offset, 0.0));
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::EndOfStorage
    ));

    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(1));
    assert!(!leftover.ok);
    assert!(leftover.corrupt);
    assert!(leftover.load_attempted);
    assert!(!fence.loaded());
    assert_eq!(fence.exclusion_circle_count(), 0);

    // Second call returns the failed attempt without re-reading.
    let again = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(2));
    assert!(!again.ok);
    assert!(again.already_attempted);
}

#[test]
fn polygon_vertex_count_below_three_fails_load() {
    let mut buf = [0u8; 32];
    format_storage(&mut buf).expect("format");
    let mut offset = 4_u16;
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::PolygonExclusion
    ));
    assert!(write_uint8_to_storage(&mut buf, &mut offset, 2));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 1, 2));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 3, 4));
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::EndOfStorage
    ));

    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(1));
    assert!(!leftover.ok);
    assert!(leftover.corrupt);
    assert_eq!(fence.exclusion_polygon_count(), 0);
}

#[test]
fn multiple_return_points_fail_load() {
    // validate_fence rejects two return points, so pack by hand.
    let mut buf = [0u8; 32];
    format_storage(&mut buf).expect("format");
    let mut offset = 4_u16;
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::ReturnPoint
    ));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 1, 2));
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::ReturnPoint
    ));
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 3, 4));
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::EndOfStorage
    ));

    let mut fence = PolyFence::new();
    let leftover = fence.load_from_storage(&buf, &mut empty_index(), load_ctx(1));
    assert!(!leftover.ok);
    assert!(leftover.corrupt);
    assert!(fence.return_point().is_none());
}

#[test]
fn loaded_inclusion_circle_breaches_outside() {
    let items = [PolyFenceItem::circle(
        PolyFenceType::CircleInclusion,
        0,
        0,
        300.0,
    )];
    let mut buf = [0u8; 32];
    write_fence(&mut buf, &items).expect("write");
    let mut fence = PolyFence::new();
    assert!(
        fence
            .load_from_storage(&buf, &mut empty_index(), load_ctx(1))
            .ok
    );
    // Centre is inside. 100_000 * 1e-7 deg is ~1.1 km, outside 300 m.
    assert!(!fence.breached(Location::new(0, 0)));
    assert!(fence.breached(Location::new(100_000, 0)));
}

#[test]
fn void_index_allows_reload_after_failed_attempt() {
    let mut bad = [0u8; 32];
    format_storage(&mut bad).expect("format");
    let mut offset = 4_u16;
    assert!(write_type_to_storage(
        &mut bad,
        &mut offset,
        PolyFenceType::CircleInclusion
    ));
    assert!(write_latlon_to_storage(&mut bad, &mut offset, 0, 0));
    assert!(write_f32_to_storage(&mut bad, &mut offset, -1.0));
    assert!(write_type_to_storage(
        &mut bad,
        &mut offset,
        PolyFenceType::EndOfStorage
    ));

    let mut fence = PolyFence::new();
    assert!(
        !fence
            .load_from_storage(&bad, &mut empty_index(), load_ctx(1))
            .ok
    );

    let items = [PolyFenceItem::circle(
        PolyFenceType::CircleInclusion,
        0,
        0,
        50.0,
    )];
    let mut good = [0u8; 32];
    write_fence(&mut good, &items).expect("write");
    fence.void_index();
    let leftover = fence.load_from_storage(&good, &mut empty_index(), load_ctx(5));
    assert!(leftover.ok);
    assert_eq!(leftover.inclusion_circles, 1);
    assert_eq!(leftover.load_time_ms, 5);
}
