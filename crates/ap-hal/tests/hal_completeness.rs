//! FW-037 completeness: remaining AP_HAL traits already on main vs leftover.

use ap_hal::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, this_slice_items, PortStatus, HAL_COMPLETENESS,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &["GPIO", "Semaphore", "Util", "Device"];

const THIS_SLICE: &[&str] = &[
    "completeness table",
    "DeviceManager get_device factory",
    "BinarySemaphore",
];

const REMAINING: &[&str] = &["WSPIDevice", "CANIface", "DSP", "Flash", "OpticalFlow"];

#[test]
fn completeness_table_lists_gpio_semaphore_util_device_done() {
    assert!(completeness_unique_names());
    assert_eq!(
        HAL_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
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
            "{name} is remaining / out of scope for this closer"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}
