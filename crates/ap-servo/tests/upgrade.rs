//! SERVOn parameter-loading leftover — `upgrade_parameters`. COP-030.
//!
//! FUNCTION used to be Int8. The interesting cases are an old Int8 that
//! must widen, a live Int16 that must be left alone, and a missing value
//! that must stay at the default — three answers that a single "converted
//! or not" flag would collapse.

use ap_param::info::GroupInfo;
use ap_param::{
    find_old_parameter, format_storage, save, ConversionInfo, ParamHeader, ParamValue, Storage,
    VarType,
};
use ap_servo::push::REMAINING;
use ap_servo::upgrade::{
    function_configured, reversed_set_and_save_ifchanged, upgrade_parameters, CHANNEL_VAR_INFO,
    SERVO_FUNCTION_DEFAULT, SERVO_MAX_DEFAULT, SERVO_MIN_DEFAULT, SERVO_REVERSED_DEFAULT,
    SERVO_TRIM_DEFAULT,
};

struct Ram {
    bytes: [u8; 512],
}

impl Ram {
    fn formatted() -> Self {
        let mut s = Self { bytes: [0xFF; 512] };
        format_storage(&mut s).expect("format");
        s
    }
}

impl Storage for Ram {
    fn size(&self) -> u16 {
        u16::try_from(self.bytes.len()).unwrap_or(0)
    }

    fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = usize::from(offset);
        let Some(src) = self.bytes.get(start..start.saturating_add(buf.len())) else {
            return false;
        };
        buf.copy_from_slice(src);
        true
    }

    fn write(&mut self, offset: u16, data: &[u8]) -> bool {
        let start = usize::from(offset);
        let Some(dest) = self.bytes.get_mut(start..start.saturating_add(data.len())) else {
            return false;
        };
        dest.copy_from_slice(data);
        true
    }
}

fn live(key: u16, group: u32) -> ParamHeader {
    ParamHeader::new(key, VarType::Int16.as_u8(), group)
}

fn old_int8(key: u16, group: u32) -> ParamHeader {
    ParamHeader::new(key, VarType::Int8.as_u8(), group)
}

fn read_i16(storage: &Ram, key: u16, group: u32) -> Option<i16> {
    match find_old_parameter(
        storage,
        ConversionInfo {
            old_key: key,
            old_group_element: group,
            old_type: VarType::Int16,
        },
    ) {
        Some((ParamValue::Int16(v), _)) => Some(v),
        _ => None,
    }
}

#[test]
fn leftover_catalog_drops_upgrade() {
    assert!(!REMAINING.contains(&"upgrade_parameters"));
}

#[test]
fn channel_table_names_the_five_servon_parameters() {
    let names: Vec<&str> = CHANNEL_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["MIN", "MAX", "TRIM", "REVERSED", "FUNCTION"]);
    let min = CHANNEL_VAR_INFO.get(0).expect("MIN");
    let max = CHANNEL_VAR_INFO.get(1).expect("MAX");
    let trim = CHANNEL_VAR_INFO.get(2).expect("TRIM");
    let rev = CHANNEL_VAR_INFO.get(3).expect("REVERSED");
    let func = CHANNEL_VAR_INFO.get(4).expect("FUNCTION");
    assert_eq!(min.idx, 1);
    assert_eq!(max.idx, 2);
    assert_eq!(trim.idx, 3);
    assert_eq!(rev.idx, 4);
    assert_eq!(func.idx, 5);
    assert_eq!(min.ptype, VarType::Int16.as_u8());
    assert_eq!(rev.ptype, VarType::Int8.as_u8());
    assert_eq!(
        func.ptype,
        VarType::Int16.as_u8(),
        "FUNCTION is Int16 now; the leftover is widening the old Int8"
    );
    let _ = (
        SERVO_MIN_DEFAULT,
        SERVO_MAX_DEFAULT,
        SERVO_TRIM_DEFAULT,
        SERVO_REVERSED_DEFAULT,
        SERVO_FUNCTION_DEFAULT,
    );
    assert_eq!(SERVO_MIN_DEFAULT, 1100);
    assert_eq!(SERVO_MAX_DEFAULT, 1900);
    assert_eq!(SERVO_TRIM_DEFAULT, 1500);
    let _: &[GroupInfo<'static>] = CHANNEL_VAR_INFO;
}

#[test]
fn upgrade_widens_int8_function_and_leaves_the_rest() {
    let mut s = Ram::formatted();
    save(&mut s, old_int8(1, 10), ParamValue::Int8(33), None, true).expect("old ch0");
    save(&mut s, live(1, 20), ParamValue::Int16(82), None, true).expect("already-new ch1");
    // channel 2: nothing stored

    let headers = [live(1, 10), live(1, 20), live(1, 30)];
    let stats = upgrade_parameters(&mut s, &headers).expect("upgrade");
    assert_eq!(stats.saved, 1);
    assert_eq!(stats.skipped_configured, 1);
    assert_eq!(stats.not_found, 1);

    assert_eq!(read_i16(&s, 1, 10), Some(33));
    assert_eq!(read_i16(&s, 1, 20), Some(82));
    assert_eq!(read_i16(&s, 1, 30), None);
}

/// Bitmask widening would map -1 to 255. FUNCTION is a number, not a mask.
#[test]
fn negative_int8_widens_numerically_not_as_a_bitmask() {
    let mut s = Ram::formatted();
    save(&mut s, old_int8(2, 0), ParamValue::Int8(-1), None, true).expect("old");
    let stats = upgrade_parameters(&mut s, &[live(2, 0)]).expect("upgrade");
    assert_eq!(stats.saved, 1);
    assert_eq!(read_i16(&s, 2, 0), Some(-1));
}

#[test]
fn function_configured_is_the_live_int16() {
    let mut s = Ram::formatted();
    let header = live(3, 1);
    assert!(!function_configured(&s, header));
    save(&mut s, header, ParamValue::Int16(4), None, true).expect("save");
    assert!(function_configured(&s, header));
}

#[test]
fn reversed_set_and_save_ifchanged_only_fires_on_a_change() {
    let mut reversed = false;
    assert!(!reversed_set_and_save_ifchanged(&mut reversed, false));
    assert!(!reversed);
    assert!(reversed_set_and_save_ifchanged(&mut reversed, true));
    assert!(reversed);
    assert!(!reversed_set_and_save_ifchanged(&mut reversed, true));
}
