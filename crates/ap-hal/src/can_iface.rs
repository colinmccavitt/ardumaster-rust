//! CAN interface, ported from `AP_HAL/CANIface.h`.
//!
//! Send/receive a classic CAN frame, set the bus bitrate, and install
//! hardware acceptance filters. Frame callbacks, `select()`, bus-off
//! recovery, and CAN-FD payloads land with a consumer that needs them.
//!
//! Upstream `send` / `receive` return `int16_t` (-1 error, 0 no space /
//! empty, 1 ok). The port uses [`Result`] / [`Option`] so absence is not
//! a sentinel in the value, matching [`crate::serial::Serial::read_byte`].
//!
//! # Acceptance filters
//!
//! `CanFilterConfig` left the Plane-4.7.0 base class, but ChibiOS still
//! exposes `NumFilters = 14` hardware slots. This stub keeps that surface:
//! a frame matches when `(id & mask) == (filter.id & filter.mask)`. An
//! empty filter list accepts every frame.

use crate::{Error, Result};

/// Classic CAN payload length. Upstream `CANFrame::MaxDataLen` when
/// `HAL_CANFD_SUPPORTED` is off.
pub const MAX_DATA_LEN: usize = 8;

/// ChibiOS `CANIface::NumFilters` hardware acceptance slots.
pub const NUM_FILTERS: usize = 14;

/// Standard 11-bit identifier mask. Upstream `CANFrame::MaskStdID`.
pub const MASK_STD_ID: u32 = 0x0000_07FF;

/// Extended 29-bit identifier mask. Upstream `CANFrame::MaskExtID`.
pub const MASK_EXT_ID: u32 = 0x1FFF_FFFF;

/// Extended frame format flag in [`CanFrame::id`]. Upstream `FlagEFF`.
pub const FLAG_EFF: u32 = 1 << 31;

/// Remote transmission request flag. Upstream `FlagRTR`.
pub const FLAG_RTR: u32 = 1 << 30;

/// Error frame flag. Upstream `FlagERR`.
pub const FLAG_ERR: u32 = 1 << 29;

/// In-memory RX depth for [`MockCanIface`]. Not an upstream constant.
const RX_QUEUE: usize = 8;

/// Raw CAN frame. Upstream `AP_HAL::CANFrame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// CAN ID with EFF / RTR / ERR flags in the high bits.
    pub id: u32,
    /// Payload bytes. Only `dlc` of them are valid.
    pub data: [u8; MAX_DATA_LEN],
    /// Data length code. Classic CAN: equal to the payload length (0..=8).
    pub dlc: u8,
}

impl Default for CanFrame {
    fn default() -> Self {
        Self {
            id: 0,
            data: [0; MAX_DATA_LEN],
            dlc: 0,
        }
    }
}

impl CanFrame {
    /// Build a classic frame. Payload longer than 8 bytes is truncated.
    ///
    /// Upstream `CANFrame(id, data, len)` also zeros unused bytes.
    #[must_use]
    pub fn new(id: u32, data: &[u8]) -> Self {
        let mut frame = Self {
            id,
            data: [0; MAX_DATA_LEN],
            dlc: 0,
        };
        let n = data.len().min(MAX_DATA_LEN);
        for (i, b) in data.iter().take(n).enumerate() {
            if let Some(slot) = frame.data.get_mut(i) {
                *slot = *b;
            }
        }
        frame.dlc = n as u8;
        frame
    }

    /// Valid payload prefix. Upstream compares `data` for `dlc` bytes.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        let n = usize::from(self.dlc).min(MAX_DATA_LEN);
        self.data.get(..n).unwrap_or(&[])
    }

    /// Extended 29-bit frame. Upstream `isExtended()`.
    #[must_use]
    pub const fn is_extended(&self) -> bool {
        self.id & FLAG_EFF != 0
    }

    /// Remote transmission request. Upstream `isRemoteTransmissionRequest()`.
    #[must_use]
    pub const fn is_remote(&self) -> bool {
        self.id & FLAG_RTR != 0
    }

    /// Error frame. Upstream `isErrorFrame()`.
    #[must_use]
    pub const fn is_error(&self) -> bool {
        self.id & FLAG_ERR != 0
    }

    /// Identifier without flags. Upstream `id_signed()` magnitude.
    #[must_use]
    pub const fn raw_id(&self) -> u32 {
        if self.is_extended() {
            self.id & MASK_EXT_ID
        } else {
            self.id & MASK_STD_ID
        }
    }
}

/// Hardware acceptance filter. Historic `CanFilterConfig` (`id` + `mask`).
///
/// A frame matches when `(frame.id & mask) == (id & mask)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFilter {
    /// Identifier bits to match after masking.
    pub id: u32,
    /// Bits of [`CanFrame::id`] that must match [`id`](Self::id).
    pub mask: u32,
}

impl CanFilter {
    /// Accept any identifier (mask 0).
    #[must_use]
    pub const fn accept_all() -> Self {
        Self { id: 0, mask: 0 }
    }

    /// Match `id` exactly (mask all 32 bits).
    #[must_use]
    pub const fn exact(id: u32) -> Self {
        Self {
            id,
            mask: 0xFFFF_FFFF,
        }
    }

    /// True when `frame_id` passes this filter.
    #[must_use]
    pub const fn matches(&self, frame_id: u32) -> bool {
        (frame_id & self.mask) == (self.id & self.mask)
    }
}

/// CAN controller. Upstream `AP_HAL::CANIface`.
///
/// `send` / `receive` / `init` / `configure_filters` are the SITL bring-up
/// surface. Heap `OwnPtr` managers and ISR callbacks are not ported.
pub trait CanIface {
    /// Bring the interface up at `bitrate` bits/s. Upstream `init(bitrate)`.
    fn init(&mut self, bitrate: u32) -> Result<()>;

    /// True after a successful [`init`](Self::init). Upstream `is_initialized()`.
    fn is_initialized(&self) -> bool;

    /// Last bitrate passed to [`init`](Self::init). Upstream `bitrate_`.
    fn bitrate(&self) -> u32;

    /// Install hardware acceptance filters. Empty list accepts every frame.
    ///
    /// More than [`NUM_FILTERS`] entries is [`Error::Unsupported`].
    fn configure_filters(&mut self, filters: &[CanFilter]) -> Result<()>;

    /// Queue `frame` for transmit. Upstream `send`.
    ///
    /// Returns [`Error::Unsupported`] before [`init`](Self::init),
    /// [`Error::BusError`] when the RX loopback queue is full.
    fn send(&mut self, frame: &CanFrame) -> Result<()>;

    /// Pop one received frame, or `None` if the RX queue is empty.
    ///
    /// Upstream `receive` returns 0 when empty.
    fn receive(&mut self) -> Option<CanFrame>;
}

/// An in-memory [`CanIface`] for tests and SITL bring-up.
///
/// [`CanIface::send`] loopbacks into a fixed RX queue so a single-threaded
/// test can exercise send/receive without a peer. Filters apply on that
/// loopback path (RX filter, not TX reject).
#[derive(Debug)]
pub struct MockCanIface {
    initialized: bool,
    bitrate: u32,
    filters: [Option<CanFilter>; NUM_FILTERS],
    filter_count: usize,
    rx: [CanFrame; RX_QUEUE],
    rx_len: usize,
    rx_pos: usize,
}

impl Default for MockCanIface {
    fn default() -> Self {
        Self {
            initialized: false,
            bitrate: 0,
            filters: [None; NUM_FILTERS],
            filter_count: 0,
            rx: [CanFrame::default(); RX_QUEUE],
            rx_len: 0,
            rx_pos: 0,
        }
    }
}

impl MockCanIface {
    /// A closed interface with an empty RX queue and no filters.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// How many frames are waiting in the RX queue.
    #[must_use]
    pub fn available(&self) -> usize {
        self.rx_len.saturating_sub(self.rx_pos)
    }

    /// Installed filters. Empty means accept-all.
    #[must_use]
    pub fn filters(&self) -> &[Option<CanFilter>] {
        self.filters.get(..self.filter_count).unwrap_or(&[])
    }

    fn accepts(&self, id: u32) -> bool {
        if self.filter_count == 0 {
            return true;
        }
        self.filters
            .iter()
            .take(self.filter_count)
            .any(|slot| slot.is_some_and(|f| f.matches(id)))
    }

    fn push_rx(&mut self, frame: CanFrame) -> Result<()> {
        if self.rx_len >= RX_QUEUE {
            return Err(Error::BusError);
        }
        if let Some(slot) = self.rx.get_mut(self.rx_len) {
            *slot = frame;
            self.rx_len += 1;
            Ok(())
        } else {
            Err(Error::BusError)
        }
    }
}

impl CanIface for MockCanIface {
    fn init(&mut self, bitrate: u32) -> Result<()> {
        if bitrate == 0 {
            return Err(Error::Unsupported);
        }
        self.bitrate = bitrate;
        self.initialized = true;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn bitrate(&self) -> u32 {
        self.bitrate
    }

    fn configure_filters(&mut self, filters: &[CanFilter]) -> Result<()> {
        if filters.len() > NUM_FILTERS {
            return Err(Error::Unsupported);
        }
        self.filters = [None; NUM_FILTERS];
        self.filter_count = 0;
        for (i, f) in filters.iter().enumerate() {
            if let Some(slot) = self.filters.get_mut(i) {
                *slot = Some(*f);
                self.filter_count += 1;
            }
        }
        Ok(())
    }

    fn send(&mut self, frame: &CanFrame) -> Result<()> {
        if !self.initialized {
            return Err(Error::Unsupported);
        }
        if !self.accepts(frame.id) {
            // TX succeeds; the hardware filter drops it on the RX path.
            return Ok(());
        }
        self.push_rx(*frame)
    }

    fn receive(&mut self) -> Option<CanFrame> {
        if self.rx_pos >= self.rx_len {
            return None;
        }
        let frame = self.rx.get(self.rx_pos).copied();
        self.rx_pos += 1;
        if self.rx_pos == self.rx_len {
            self.rx_pos = 0;
            self.rx_len = 0;
        }
        frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_sets_bitrate_and_initialized() {
        let mut can = MockCanIface::new();
        assert!(!can.is_initialized());
        assert_eq!(can.bitrate(), 0);
        assert_eq!(can.init(0), Err(Error::Unsupported));
        assert!(can.init(1_000_000).is_ok());
        assert!(can.is_initialized());
        assert_eq!(can.bitrate(), 1_000_000);
    }

    #[test]
    fn send_receive_round_trip() {
        let mut can = MockCanIface::new();
        can.init(500_000).unwrap();
        let frame = CanFrame::new(0x123, &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(can.send(&frame).is_ok());
        assert_eq!(can.available(), 1);
        let got = can.receive().expect("loopback frame");
        assert_eq!(got, frame);
        assert_eq!(got.payload(), &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(can.receive().is_none());
    }

    /// Empty filter list is accept-all, matching an unprogrammed controller.
    #[test]
    fn empty_filters_accept_all() {
        let mut can = MockCanIface::new();
        can.init(250_000).unwrap();
        assert!(can.filters().is_empty());
        assert!(can.send(&CanFrame::new(0x001, &[1])).is_ok());
        assert!(can.send(&CanFrame::new(0x7FF, &[2])).is_ok());
        assert_eq!(can.receive().map(|f| f.id), Some(0x001));
        assert_eq!(can.receive().map(|f| f.id), Some(0x7FF));
    }

    /// Hardware filter: TX still succeeds, unmatched IDs never appear on RX.
    #[test]
    fn filter_drops_unmatched_ids() {
        let mut can = MockCanIface::new();
        can.init(1_000_000).unwrap();
        can.configure_filters(&[CanFilter::exact(0x42)]).unwrap();
        assert_eq!(can.filters().len(), 1);

        assert!(can.send(&CanFrame::new(0x99, &[0x01])).is_ok());
        assert!(can.receive().is_none(), "0x99 must not pass exact 0x42");

        assert!(can.send(&CanFrame::new(0x42, &[0x02])).is_ok());
        let got = can.receive().expect("matched id");
        assert_eq!(got.id, 0x42);
        assert_eq!(got.payload(), &[0x02]);
    }

    #[test]
    fn filter_mask_matches_std_id_bits() {
        let mut can = MockCanIface::new();
        can.init(1_000_000).unwrap();
        // Match any 11-bit id whose low 4 bits are 0x5.
        can.configure_filters(&[CanFilter {
            id: 0x005,
            mask: 0x00F,
        }])
        .unwrap();
        assert!(can.send(&CanFrame::new(0x015, &[1])).is_ok());
        assert!(can.send(&CanFrame::new(0x016, &[2])).is_ok());
        let got = can.receive().expect("0x015 matches mask");
        assert_eq!(got.id, 0x015);
        assert!(can.receive().is_none());
    }

    #[test]
    fn too_many_filters_is_unsupported() {
        let mut can = MockCanIface::new();
        let extra = [CanFilter::accept_all(); NUM_FILTERS + 1];
        assert_eq!(can.configure_filters(&extra), Err(Error::Unsupported));
        assert!(can.filters().is_empty());
    }

    #[test]
    fn send_before_init_is_unsupported() {
        let mut can = MockCanIface::new();
        assert_eq!(can.send(&CanFrame::new(0x1, &[0])), Err(Error::Unsupported));
        assert!(can.receive().is_none());
    }

    #[test]
    fn full_rx_queue_is_bus_error() {
        let mut can = MockCanIface::new();
        can.init(1_000_000).unwrap();
        for i in 0..RX_QUEUE {
            assert!(can.send(&CanFrame::new(i as u32, &[i as u8])).is_ok());
        }
        assert_eq!(
            can.send(&CanFrame::new(0xFF, &[0xFF])),
            Err(Error::BusError)
        );
    }

    #[test]
    fn extended_and_rtr_flags() {
        let ext = CanFrame::new(FLAG_EFF | 0x1ABC_DEF, &[0x11]);
        assert!(ext.is_extended());
        assert_eq!(ext.raw_id(), 0x1ABC_DEF);
        let rtr = CanFrame::new(FLAG_RTR | 0x123, &[]);
        assert!(rtr.is_remote());
        assert!(!rtr.is_error());
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn can_iface_trait_is_object_safe() {
        let mut can = MockCanIface::new();
        let c: &mut dyn CanIface = &mut can;
        assert!(c.init(125_000).is_ok());
        assert_eq!(c.bitrate(), 125_000);
        assert!(c.is_initialized());
        assert!(c.configure_filters(&[CanFilter::exact(0x7)]).is_ok());
        assert!(c.send(&CanFrame::new(0x7, &[0xAA])).is_ok());
        assert_eq!(c.receive().map(|f| f.id), Some(0x7));
    }
}
