//! First EEPROM format leftover: magic, item types, `format()`, write
//! primitives, and `fence_storage_space_required`.
//!
//! Tracked as **COP-025**. Scan / index / `write_fence` / SD stay later.

use ap_fence::{
    fence_storage_space_required, format_storage, storage_formatted, write_eos_to_storage,
    write_latlon_to_storage, write_type_to_storage, PolyFenceItem, PolyFenceType, STORAGE_MAGIC,
};

#[test]
fn type_bytes_match_upstream() {
    assert_eq!(PolyFenceType::CircleInclusion.as_u8(), 92);
    assert_eq!(PolyFenceType::CircleExclusion.as_u8(), 93);
    assert_eq!(PolyFenceType::CircleInclusionInt.as_u8(), 94);
    assert_eq!(PolyFenceType::ReturnPoint.as_u8(), 95);
    assert_eq!(PolyFenceType::CircleExclusionInt.as_u8(), 96);
    assert_eq!(PolyFenceType::PolygonExclusion.as_u8(), 97);
    assert_eq!(PolyFenceType::PolygonInclusion.as_u8(), 98);
    assert_eq!(PolyFenceType::EndOfStorage.as_u8(), 99);
    assert_eq!(STORAGE_MAGIC, 235);

    assert_eq!(
        PolyFenceType::from_u8(98),
        Some(PolyFenceType::PolygonInclusion)
    );
    assert_eq!(PolyFenceType::from_u8(0), None);
    assert_eq!(PolyFenceType::from_u8(234), None);
}

#[test]
fn empty_or_short_buffer_is_not_formatted() {
    assert!(!storage_formatted(&[]));
    assert!(!storage_formatted(&[STORAGE_MAGIC, 0, 0]));
    assert!(!storage_formatted(&[0, 0, 0, 0]));
    assert!(!storage_formatted(&[STORAGE_MAGIC, 1, 0, 0]));
}

#[test]
fn format_writes_magic_and_end_of_storage() {
    let mut buf = [0xAAu8; 16];
    let eos = format_storage(&mut buf).expect("room for header + EOS");
    assert_eq!(eos, 4);
    assert!(storage_formatted(&buf));
    assert_eq!(buf[0], STORAGE_MAGIC);
    assert_eq!(&buf[1..4], &[0, 0, 0]);
    assert_eq!(buf[4], PolyFenceType::EndOfStorage.as_u8());
    // format does not wipe the rest of the store.
    assert_eq!(&buf[5..], &[0xAA; 11]);
}

#[test]
fn format_refuses_a_buffer_shorter_than_header_plus_eos() {
    let mut tiny = [0u8; 4];
    assert!(format_storage(&mut tiny).is_none());
    assert!(!storage_formatted(&tiny));
}

#[test]
fn write_primitives_pack_type_then_latlon() {
    let mut buf = [0u8; 16];
    let mut offset = 0_u16;
    assert!(write_type_to_storage(
        &mut buf,
        &mut offset,
        PolyFenceType::CircleInclusion
    ));
    assert_eq!(offset, 1);
    assert!(write_latlon_to_storage(&mut buf, &mut offset, 1_234, -5_678));
    assert_eq!(offset, 9);
    assert_eq!(buf[0], 92);
    assert_eq!(&buf[1..5], &1_234_i32.to_le_bytes());
    assert_eq!(&buf[5..9], &(-5_678_i32).to_le_bytes());

    let eos = write_eos_to_storage(&mut buf, &mut offset).expect("EOS");
    assert_eq!(eos, 9);
    assert_eq!(buf[9], 99);
}

#[test]
fn space_required_counts_header_circle_and_return_point() {
    assert_eq!(fence_storage_space_required(&[]), 4);

    let circle = PolyFenceItem::circle(PolyFenceType::CircleInclusion, 0, 0, 300.0);
    // 4 header + 1 type + 12 (lat/lng/radius)
    assert_eq!(fence_storage_space_required(&[circle]), 17);

    let ret = PolyFenceItem::return_point(10, 20);
    // 4 header + 1 type + 8 lat/lng
    assert_eq!(fence_storage_space_required(&[ret]), 13);

    let excl = PolyFenceItem::circle(PolyFenceType::CircleExclusion, 1, 2, 50.0);
    assert_eq!(fence_storage_space_required(&[circle, ret, excl]), 4 + 13 + 9 + 13);
}

#[test]
fn space_required_packs_a_polygon_as_one_record() {
    let verts = [
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 0, 0, 4),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 1, 0, 4),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 1, 1, 4),
        PolyFenceItem::polygon(PolyFenceType::PolygonInclusion, 0, 1, 4),
    ];
    // 4 header + 1 type + 1 count + 8*4 lat/lng
    assert_eq!(fence_storage_space_required(&verts), 38);

    let excl = [
        PolyFenceItem::polygon(PolyFenceType::PolygonExclusion, 2, 2, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonExclusion, 3, 2, 3),
        PolyFenceItem::polygon(PolyFenceType::PolygonExclusion, 3, 3, 3),
    ];
    // 4 + 1 + 1 + 8*3 = 30
    assert_eq!(fence_storage_space_required(&excl), 30);
}

#[test]
fn space_required_skips_integer_circle_payload() {
    let bad = PolyFenceItem::circle(PolyFenceType::CircleInclusionInt, 0, 0, 10.0);
    // header + type byte only; C++ INTERNAL_ERRORs these items.
    assert_eq!(fence_storage_space_required(&[bad]), 5);
}
