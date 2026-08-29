//! Inertial history ring, leftover of `inertial_data_frame_s` and
//! `ObjectArray<inertial_data_frame_s>`.
//!
//! Tracked as **COP-028**. Upstream
//! `libraries/AP_HAL/utility/RingBuffer.h` `ObjectArray` plus
//! `AC_PrecLand::inertial_data_frame_s`. `init` sizes the ring from
//! `PLND_LAG * update_rate_hz`. `update` `push_force`s each AHRS
//! snapshot. The estimator reads slot 0 (delayed); output prediction
//! walks `[1..available())`.

use crate::estimator::InertialSample;

/// Max inertial history slots. Covers `PLND_LAG` 0.25 s at 1 kHz.
/// Upstream heap-allocates; this port is `no_std`.
pub const INERTIAL_HISTORY_MAX: usize = 256;

/// Upstream `inertial_data_frame_s`. Same layout as [`InertialSample`].
pub type InertialDataFrame = InertialSample;

/// `ObjectArray<inertial_data_frame_s>` used as `_inertial_history`.
///
/// `[0]` is the oldest (delayed) frame. `push_force` drops the oldest
/// when full, matching upstream
/// `if (!push(t)) { pop(); push(t); }`.
#[derive(Debug, Clone)]
pub struct InertialHistory {
    buf: [InertialSample; INERTIAL_HISTORY_MAX],
    size: u16,
    count: u16,
    head: u16,
}

impl InertialHistory {
    /// Allocate a ring of `size` slots. `0` is an empty unallocated ring
    /// (upstream `NEW_NOTHROW` failure / before `init`).
    #[must_use]
    pub fn new(size: u16) -> Self {
        let size = size.min(INERTIAL_HISTORY_MAX as u16);
        Self {
            buf: [InertialSample::default(); INERTIAL_HISTORY_MAX],
            size,
            count: 0,
            head: 0,
        }
    }

    /// Upstream `ObjectArray::size`.
    #[must_use]
    pub fn size(&self) -> u16 {
        self.size
    }

    /// Upstream `ObjectArray::available`.
    #[must_use]
    pub fn available(&self) -> u16 {
        self.count
    }

    /// Upstream `ObjectArray::space`.
    #[must_use]
    pub fn space(&self) -> u16 {
        self.size.saturating_sub(self.count)
    }

    /// Upstream `ObjectArray::is_empty`.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Upstream `ObjectArray::push`. `false` when full or unallocated.
    pub fn push(&mut self, frame: InertialSample) -> bool {
        if self.size == 0 || self.space() == 0 {
            return false;
        }
        let idx = (self.head + self.count) % self.size;
        self.buf[idx as usize] = frame;
        self.count += 1;
        true
    }

    /// Upstream `ObjectArray::pop()` (discard oldest).
    pub fn pop(&mut self) -> bool {
        if self.is_empty() || self.size == 0 {
            return false;
        }
        self.head = (self.head + 1) % self.size;
        self.count -= 1;
        true
    }

    /// Upstream `ObjectArray::clear`.
    pub fn clear(&mut self) {
        self.head = 0;
        self.count = 0;
    }

    /// Upstream `ObjectArray::push_force`.
    pub fn push_force(&mut self, frame: InertialSample) -> bool {
        if self.space() == 0 {
            let _ = self.pop();
        }
        self.push(frame)
    }

    /// Upstream `operator[]`. `None` when `i >= available()`.
    #[must_use]
    pub fn get(&self, i: u16) -> Option<InertialSample> {
        if self.size == 0 || i >= self.count {
            return None;
        }
        let idx = (self.head + i) % self.size;
        Some(self.buf[idx as usize])
    }

    /// Delayed horizon. Upstream `(*_inertial_history)[0]`.
    #[must_use]
    pub fn delayed(&self) -> Option<InertialSample> {
        self.get(0)
    }

    /// Newest frame. Upstream `(*_inertial_history)[available()-1]`.
    #[must_use]
    pub fn newest(&self) -> Option<InertialSample> {
        if self.count == 0 {
            return None;
        }
        self.get(self.count - 1)
    }

    /// Frames after the delayed slot. Upstream `[1..available())`.
    #[must_use]
    pub fn later(&self) -> LaterFrames<'_> {
        LaterFrames {
            history: self,
            index: 1,
        }
    }

    /// Leftover of walking the ring for `!inertialNavVelocityValid`.
    #[must_use]
    pub fn any_inertial_nav_invalid(&self) -> bool {
        let mut i = 0;
        while i < self.count {
            if let Some(frame) = self.get(i) {
                if !frame.inertial_nav_velocity_valid {
                    return true;
                }
            }
            i += 1;
        }
        false
    }
}

impl Default for InertialHistory {
    fn default() -> Self {
        Self::new(0)
    }
}

/// Iterator over later (non-delayed) frames.
#[derive(Debug, Clone)]
pub struct LaterFrames<'a> {
    history: &'a InertialHistory,
    index: u16,
}

impl Iterator for LaterFrames<'_> {
    type Item = InertialSample;

    fn next(&mut self) -> Option<Self::Item> {
        let frame = self.history.get(self.index)?;
        self.index += 1;
        Some(frame)
    }
}
