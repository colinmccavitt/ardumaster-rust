//! FW-037 remaining-traits completeness: GPIO / Semaphore / Util / Device
//! already on main vs leftover AP_HAL surfaces.
//!
//! Catalogs the FW-037 HAL leftover after FW-001. Items marked
//! [`PortStatus::OnMain`] landed in earlier slices; [`PortStatus::ThisSlice`]
//! is this table; [`PortStatus::Remaining`] are board / bus surfaces that
//! still have no fixed-wing SITL consumer.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-037 closing slice (this table).
    ThisSlice,
    /// Still deferred (no SITL consumer, or heap `get_device`).
    Remaining,
}

/// One remaining-HAL-trait surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HalPortItem {
    /// Trait or surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: FW-037 traits already on main vs leftover HAL.
pub const HAL_COMPLETENESS: &[HalPortItem] = &[
    HalPortItem {
        name: "GPIO",
        status: PortStatus::OnMain,
        note: "AP_HAL::GPIO / DigitalSource pin mode, read/write",
    },
    HalPortItem {
        name: "Semaphore",
        status: PortStatus::OnMain,
        note: "AP_HAL::Semaphore take/give, take_nonblocking",
    },
    HalPortItem {
        name: "Util",
        status: PortStatus::OnMain,
        note: "AP_HAL::Util safety_switch, system_id, persistent_data",
    },
    HalPortItem {
        name: "Device",
        status: PortStatus::OnMain,
        note: "AP_HAL::Device / I2CDevice / SPIDevice register r/w",
    },
    HalPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    HalPortItem {
        name: "WSPIDevice",
        status: PortStatus::Remaining,
        note: "AP_HAL/WSPIDevice.h; no SITL consumer yet",
    },
    HalPortItem {
        name: "CANIface",
        status: PortStatus::Remaining,
        note: "AP_HAL/CANIface.h; no SITL consumer yet",
    },
    HalPortItem {
        name: "DSP",
        status: PortStatus::Remaining,
        note: "AP_HAL/DSP.h; no SITL consumer yet",
    },
    HalPortItem {
        name: "Flash",
        status: PortStatus::Remaining,
        note: "AP_HAL/Flash.h; no SITL consumer yet",
    },
    HalPortItem {
        name: "OpticalFlow",
        status: PortStatus::Remaining,
        note: "AP_HAL/OpticalFlow.h; no SITL consumer yet",
    },
    HalPortItem {
        name: "BinarySemaphore",
        status: PortStatus::ThisSlice,
        note: "AP_HAL::BinarySemaphore wait/signal; not in the Semaphore stub",
    },
    HalPortItem {
        name: "DeviceManager get_device factory",
        status: PortStatus::ThisSlice,
        note: "table-backed I2C/SPI get_device (no heap OwnPtr)",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static HalPortItem> {
    HAL_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static HalPortItem> {
    HAL_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for later HAL work (not blocking this closer).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static HalPortItem> {
    HAL_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in HAL_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    HAL_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in HAL_COMPLETENESS.iter().enumerate() {
        for other in HAL_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_lists_gpio_semaphore_util_device_done() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 4);
        assert_eq!(this_slice, 3);
        assert_eq!(remaining, 5);
        assert!(completeness_has("GPIO", PortStatus::OnMain));
        assert!(completeness_has("Semaphore", PortStatus::OnMain));
        assert!(completeness_has("Util", PortStatus::OnMain));
        assert!(completeness_has("Device", PortStatus::OnMain));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("CANIface", PortStatus::Remaining));
        assert!(completeness_has(
            "DeviceManager get_device factory",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("BinarySemaphore", PortStatus::ThisSlice));
        assert_eq!(on_main_items().count(), 4);
        assert_eq!(this_slice_items().count(), 3);
        assert_eq!(remaining_items().count(), 5);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }
}
