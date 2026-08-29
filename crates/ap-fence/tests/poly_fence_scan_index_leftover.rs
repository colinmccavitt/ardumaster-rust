//! EEPROM scan / index / `write_fence` leftover.
//!
//! Tracked as **COP-025**. `load_from_storage` / SD stay later.

use ap_fence::{
    count_eeprom_fences, fence_storage_space_required, format_storage, index_eeprom,
    index_fence_count, max_items, read_f32_from_storage, read_latlon_from_storage, scan_eeprom,
    storage_formatted, sum_of_polygon_point_counts_and_returnpoint, validate_fence, write_fence,
    FenceIndex, PolyFenceItem, PolyFenceType,
};

fn empty_index() -> [FenceIndex; 8] {
    [FenceIndex::EMPTY; 8]
}

#[test]
fn scan_unformatted_or_corrupt_store_fails() {
    assert!(scan_eeprom(&[0u8; 16], |_, _| {}).is_none());
    let mut buf = [0u8; 16];
    format_storage(&mut buf).expect("format");
    buf[4] = 7; // not a fence type
    assert!(scan_eeprom(&buf, |_, _| {}).is_none());
}

#[test]
fn scan_formatted_empty_store_visits_eos_at_four() {
    let mut buf = [0xAAu8; 16];
    format_storage(&mut buf).expect("format");
    let mut seen = [PolyFenceType::ReturnPoint; 2];
    let mut n = 0_usize;
    let eos = scan_eeprom(&buf, |kind, offset| {
        if n < seen.len() {
            seen[n] = kind;
        }
        assert_eq!(offset, 4);
        n += 1;
    })
    .expect("scan");
    assert_eq!(eos, 4);
    assert_eq!(n, 1);
    assert_eq!(seen[0], PolyFenceType::EndOfStorage);

    let counts = count_eeprom_fences(&buf).expect("count");
    assert_eq!(counts.fence_count, 0);
    assert_eq!(counts.item_count, 0);

    let indexed = index_eeprom(&buf, &mut empty_index()).expect("index empty");
    assert_eq!(indexed.num_fences, 0);
    assert_eq!(indexed.eos_offset, 4);
}

#[test]
fn write_fence_packs_circle_return_and_polygon_then_indexes() {
    let items = [
        PolyFenceItem::return_point(1_000, 2_000),
        PolyFenceItem::circle(PolyFenceType::CircleInclusion, 3_000, 4_000, 250.0),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 10, 20, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 11, 21, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 12, 22, 3),
    ];
    // header 4 + return 9 + circle 13 + poly 1+1+24 = 52; EOS is extra
    assert_eq!(fence_storage_space_required(&items), 52);

    let mut buf = [0u8; 64];
    let written = write_fence(&mut buf, &items).expect("write");
    assert!(storage_formatted(&buf));
    // 4 magic + 9 return + 13 circle + 26 poly + 0 (EOS at 52)
    assert_eq!(written.eos_offset, 52);
    // total_vertex_count = 3 (poly) + 1 (circle) = 4; new_total = 6
    assert_eq!(written.new_total, 6);

    let mut kinds = [PolyFenceType::EndOfStorage; 8];
    let mut offsets = [0_u16; 8];
    let mut n = 0_usize;
    let eos = scan_eeprom(&buf, |kind, offset| {
        if n < kinds.len() {
            kinds[n] = kind;
            offsets[n] = offset;
        }
        n += 1;
    })
    .expect("scan");
    assert_eq!(eos, 52);
    assert_eq!(n, 4);
    assert_eq!(kinds[0], PolyFenceType::ReturnPoint);
    assert_eq!(offsets[0], 4);
    assert_eq!(kinds[1], PolyFenceType::CircleInclusion);
    assert_eq!(offsets[1], 13);
    assert_eq!(kinds[2], PolyFenceType::PolygonInclusion);
    assert_eq!(offsets[2], 26);
    assert_eq!(kinds[3], PolyFenceType::EndOfStorage);
    assert_eq!(offsets[3], 52);

    let counts = count_eeprom_fences(&buf).expect("count");
    assert_eq!(counts.fence_count, 3);
    assert_eq!(counts.item_count, 1 + 1 + 3);

    let mut index = empty_index();
    let indexed = index_eeprom(&buf, &mut index).expect("index");
    assert_eq!(indexed.num_fences, 3);
    assert_eq!(indexed.eos_offset, 52);
    assert_eq!(
        index[0],
        FenceIndex {
            kind: PolyFenceType::ReturnPoint,
            count: 1,
            storage_offset: 4,
        }
    );
    assert_eq!(
        index[1],
        FenceIndex {
            kind: PolyFenceType::CircleInclusion,
            count: 1,
            storage_offset: 13,
        }
    );
    assert_eq!(
        index[2],
        FenceIndex {
            kind: PolyFenceType::PolygonInclusion,
            count: 3,
            storage_offset: 26,
        }
    );
    assert_eq!(
        index_fence_count(&index, indexed.num_fences, PolyFenceType::ReturnPoint),
        1
    );
    assert_eq!(
        index_fence_count(&index, indexed.num_fences, PolyFenceType::CircleExclusion),
        0
    );
    assert_eq!(
        sum_of_polygon_point_counts_and_returnpoint(&index, indexed.num_fences),
        4
    );

    // payload at the return-point and circle offsets
    let mut off = 5_u16; // skip return type
    assert_eq!(
        read_latlon_from_storage(&buf, &mut off),
        Some((1_000, 2_000))
    );
    off = 14; // skip circle type
    assert_eq!(
        read_latlon_from_storage(&buf, &mut off),
        Some((3_000, 4_000))
    );
    let radius = read_f32_from_storage(&buf, &mut off).expect("radius");
    assert!((radius - 250.0).abs() < f32::EPSILON);
}

#[test]
fn write_fence_empty_items_is_a_formatted_eos() {
    let mut buf = [0u8; 16];
    let written = write_fence(&mut buf, &[]).expect("clear");
    assert_eq!(written.eos_offset, 4);
    assert_eq!(written.new_total, 0);
    let counts = count_eeprom_fences(&buf).expect("count");
    assert_eq!(counts.fence_count, 0);
}

#[test]
fn write_fence_and_index_refuse_bad_inputs() {
    let mut buf = [0u8; 32];
    let two_verts = [
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 0, 0, 2),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 1, 0, 2),
    ];
    assert!(!validate_fence(&two_verts));
    assert!(write_fence(&mut buf, &two_verts).is_none());

    let zero_radius = [PolyFenceItem::circle(
        PolyFenceType::CircleExclusion,
        0,
        0,
        0.0,
    )];
    assert!(!validate_fence(&zero_radius));

    let int_circle = [PolyFenceItem::circle(
        PolyFenceType::CircleInclusionInt,
        0,
        0,
        10.0,
    )];
    assert!(!validate_fence(&int_circle));
    assert!(write_fence(&mut buf, &int_circle).is_none());

    let two_returns = [
        PolyFenceItem::return_point(0, 0),
        PolyFenceItem::return_point(1, 1),
    ];
    assert!(!validate_fence(&two_returns));

    let incomplete = [
        PolyFenceItem::polygon(PolyFenceType::PolygonExclusion, 0, 0, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonExclusion, 1, 0, 3),
    ];
    assert!(!validate_fence(&incomplete));

    let bad_lat = [PolyFenceItem::return_point(91 * 10_000_000, 0)];
    assert!(!validate_fence(&bad_lat));

    format_storage(&mut buf).expect("format");
    let mut tiny = [FenceIndex::EMPTY; 0];
    // empty formatted store does not need index slots
    assert!(index_eeprom(&buf, &mut tiny).is_some());

    let circle = [PolyFenceItem::circle(
        PolyFenceType::CircleExclusion,
        0,
        0,
        40.0,
    )];
    write_fence(&mut buf, &circle).expect("circle");
    assert!(index_eeprom(&buf, &mut tiny).is_none());
}

#[test]
fn max_items_is_storage_size_over_latlon() {
    assert_eq!(max_items(&[0u8; 0]), 0);
    assert_eq!(max_items(&[0u8; 16]), 2);
    assert_eq!(max_items(&[0u8; 672]), 84); // PixHawk leftover comment
}
