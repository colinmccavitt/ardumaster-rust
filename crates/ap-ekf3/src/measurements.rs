//! IMU sample ring, upstream `AP_NavEKF3_Measurements.cpp` `readIMUData`.
//!
//! Gyro and accel arrive faster than the EKF prediction rate. This slice is
//! the FIFO that later prediction reads from the fusion time horizon, plus
//! the downsample that collapses those IMU frames onto [`EKF_TARGET_DT`].
//!
//! # What is stored
//!
//! Upstream `imu_elements` keeps body-frame delta angle (rad) and delta
//! velocity (m/s), the intervals they were integrated over, a millisecond
//! timestamp, and the gyro/accel instance indices. Raw rates become those
//! deltas the same way `readDeltaAngle` / `readDeltaVelocity` do: multiply
//! by the sample interval, then clamp the angle interval away from zero.
//!
//! # Downsample
//!
//! Frames accumulate into `imuDataDownSampledNew` until `delAngDT` reaches
//! the 12 ms target (or twice that if the frontend has withheld a predict).
//! The accumulated sample is then `push_youngest_element` onto `storedIMU`,
//! and `get_oldest_element` becomes the delayed IMU used at the fusion
//! horizon.
//!
//! The quaternion rotate-and-normalise that suppresses coning during
//! downsample is not here; this stub sums the deltas. Covariance prediction
//! still consumes the delayed sample, not the raw IMU frame.
//!
//! # Buffer semantics
//!
//! `EKF_IMU_buffer_t` increments the youngest index *before* the write, so
//! the first stored sample lands at index 1 and index 0 stays zero until
//! the ring wraps. [`ImuBuffer`] reproduces that, including `is_filled`
//! latching on wrap. The storage is a fixed array: no allocator, matching
//! this crate's `no_std` rule. Capacity is the 250 ms GPS-lag worst case
//! at 12 ms plus a few slots of jitter.

use ap_math::vector3::Vector3;
use ap_math::Ftype;

/// Target EKF step in milliseconds, upstream `EKF_TARGET_DT_MS`.
pub const EKF_TARGET_DT_MS: u32 = 12;

/// Target EKF step in seconds, upstream `EKF_TARGET_DT`.
pub const EKF_TARGET_DT: Ftype = 0.012;

/// Fixed ring capacity, no_std stand-in for `imu_buffer_length`.
///
/// Upstream sizes the ring as `maxTimeDelay_ms / EKF_TARGET_DT_MS + 1`, and
/// caps GPS lag at 250 ms, which is 21 slots. Twenty-six leaves room for
/// the same jitter allowance `setup_core` applies to observation buffers.
pub const IMU_BUFFER_CAPACITY: usize = 26;

/// Smallest delta-angle interval `readIMUData` will accept, 1e-4 s.
const MIN_DEL_ANG_DT: Ftype = 1.0e-4;

/// Typical INS loop delta used to seed `dtIMUavg` (400 Hz).
const DEFAULT_DT_IMU_AVG: Ftype = 0.0025;

/// One IMU frame as rates, before conversion to INS deltas.
///
/// The EKF ring stores deltas; this is the gyro/accel sample that produces
/// them. `dt` is the interval the rates were measured over.
#[derive(Debug, Clone, Copy)]
pub struct ImuRawSample {
    /// Gyro rate in body frame (rad/s).
    pub gyro: Vector3<Ftype>,
    /// Accelerometer specific force in body frame (m/s²).
    pub accel: Vector3<Ftype>,
    /// Sample interval (s).
    pub dt: Ftype,
    /// Measurement timestamp (ms), upstream `imuSampleTime_ms`.
    pub time_ms: u32,
    /// Active gyro instance, upstream `gyro_index`.
    pub gyro_index: u8,
    /// Active accel instance, upstream `accel_index`.
    pub accel_index: u8,
}

/// One stored IMU element, upstream `NavEKF3_core::imu_elements`.
#[derive(Debug, Clone, Copy)]
pub struct ImuElements {
    /// Body-frame delta angle (rad), upstream `delAng`.
    pub del_ang: Vector3<Ftype>,
    /// Body-frame delta velocity (m/s), upstream `delVel`.
    pub del_vel: Vector3<Ftype>,
    /// Interval over which `del_ang` was measured (s), upstream `delAngDT`.
    pub del_ang_dt: Ftype,
    /// Interval over which `del_vel` was measured (s), upstream `delVelDT`.
    pub del_vel_dt: Ftype,
    /// Measurement timestamp (ms), upstream `time_ms`.
    pub time_ms: u32,
    /// Gyro instance that produced `del_ang`, upstream `gyro_index`.
    pub gyro_index: u8,
    /// Accel instance that produced `del_vel`, upstream `accel_index`.
    pub accel_index: u8,
}

impl ImuElements {
    /// A zeroed element, the `calloc` state of an unused ring slot.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            del_ang: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            del_vel: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            del_ang_dt: 0.0 as Ftype,
            del_vel_dt: 0.0 as Ftype,
            time_ms: 0,
            gyro_index: 0,
            accel_index: 0,
        }
    }

    /// Convert gyro/accel rates to deltas, upstream `readDeltaAngle` /
    /// `readDeltaVelocity` into `imuDataNew`.
    #[must_use]
    pub fn from_raw(sample: ImuRawSample) -> Self {
        let dt = if sample.dt > 0.0 as Ftype {
            sample.dt
        } else {
            0.0 as Ftype
        };
        let mut del_ang_dt = dt;
        if del_ang_dt < MIN_DEL_ANG_DT {
            del_ang_dt = MIN_DEL_ANG_DT;
        }
        Self {
            del_ang: sample.gyro * dt,
            del_vel: sample.accel * dt,
            del_ang_dt,
            del_vel_dt: dt,
            time_ms: sample.time_ms,
            gyro_index: sample.gyro_index,
            accel_index: sample.accel_index,
        }
    }
}

impl Default for ImuElements {
    fn default() -> Self {
        Self::zero()
    }
}

/// IMU FIFO, upstream `EKF_IMU_buffer_t<imu_elements>` / `storedIMU`.
#[derive(Debug, Clone)]
pub struct ImuBuffer {
    slots: [ImuElements; IMU_BUFFER_CAPACITY],
    /// Live length, upstream `imu_buffer_length`, at most [`IMU_BUFFER_CAPACITY`].
    len: usize,
    /// Youngest written index, upstream `_youngest`.
    youngest: usize,
    /// Oldest readable index, upstream `_oldest`.
    oldest: usize,
    /// Whether the ring has wrapped once, upstream `_filled`.
    filled: bool,
}

impl Default for ImuBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuBuffer {
    /// A ring of [`IMU_BUFFER_CAPACITY`] zeroed slots.
    #[must_use]
    pub const fn new() -> Self {
        Self::with_len(IMU_BUFFER_CAPACITY)
    }

    /// A ring of `len` live slots (clamped to `1..=`[`IMU_BUFFER_CAPACITY`]).
    #[must_use]
    pub const fn with_len(len: usize) -> Self {
        let len = if len == 0 {
            1
        } else if len > IMU_BUFFER_CAPACITY {
            IMU_BUFFER_CAPACITY
        } else {
            len
        };
        Self {
            slots: [ImuElements::zero(); IMU_BUFFER_CAPACITY],
            len,
            youngest: 0,
            oldest: 0,
            filled: false,
        }
    }

    /// Live slot count, upstream `imu_buffer_length`.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the ring has wrapped, upstream `is_filled`.
    #[must_use]
    pub const fn is_filled(&self) -> bool {
        self.filled
    }

    /// Youngest written index, upstream `get_youngest_index`.
    #[must_use]
    pub const fn youngest_index(&self) -> usize {
        self.youngest
    }

    /// Oldest readable index, upstream `get_oldest_index`.
    #[must_use]
    pub const fn oldest_index(&self) -> usize {
        self.oldest
    }

    /// Slot at `index`, upstream `operator[]`. Out-of-range is a zero element.
    #[must_use]
    pub fn get(&self, index: usize) -> ImuElements {
        if index >= self.len {
            return ImuElements::zero();
        }
        match self.slots.get(index) {
            Some(&element) => element,
            None => ImuElements::zero(),
        }
    }

    /// Push a downsampled sample, upstream `push_youngest_element`.
    ///
    /// Increments the youngest index first, writes there, then sets oldest
    /// to the slot after youngest. The first write therefore lands at
    /// index 1; wrapping youngest back to 0 latches [`Self::is_filled`].
    pub fn push_youngest_element(&mut self, element: ImuElements) {
        self.youngest = self.youngest.saturating_add(1);
        if self.youngest == self.len {
            self.youngest = 0;
            self.filled = true;
        }
        if let Some(slot) = self.slots.get_mut(self.youngest) {
            *slot = element;
        }
        self.oldest = match self.youngest.checked_add(1) {
            Some(next) if next < self.len => next,
            _ => 0,
        };
    }

    /// Oldest stored sample, upstream `get_oldest_element`.
    ///
    /// Before the ring fills this is often a still-zero slot — the same
    /// unread `calloc` cell upstream returns.
    #[must_use]
    pub fn get_oldest_element(&self) -> ImuElements {
        match self.slots.get(self.oldest) {
            Some(&element) => element,
            None => ImuElements::zero(),
        }
    }

    /// Zero every slot and rewind the indices, upstream `reset`.
    pub fn reset(&mut self) {
        self.slots = [ImuElements::zero(); IMU_BUFFER_CAPACITY];
        self.youngest = 0;
        self.oldest = 0;
        self.filled = false;
    }
}

/// Downsample accumulator plus the IMU FIFO, upstream `readIMUData`.
///
/// Owns `imuDataDownSampledNew`, `storedIMU`, and the delayed sample that
/// prediction will read. Does not talk to `AP_InertialSensor`; the caller
/// supplies each gyro/accel frame.
#[derive(Debug, Clone)]
pub struct ImuSampleRing {
    stored: ImuBuffer,
    downsampled: ImuElements,
    delayed: ImuElements,
    run_updates: bool,
    dt_imu_avg: Ftype,
}

impl Default for ImuSampleRing {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuSampleRing {
    /// Empty ring at the default IMU buffer length.
    ///
    /// `dtIMUavg` starts at a 400 Hz INS period; [`Self::read_imu_data`]
    /// then tracks the incoming sample interval the way upstream's spike-
    /// and-lowpass does.
    #[must_use]
    pub const fn new() -> Self {
        Self::with_buffer_len(IMU_BUFFER_CAPACITY)
    }

    /// Empty ring with a caller-chosen live length.
    #[must_use]
    pub const fn with_buffer_len(len: usize) -> Self {
        Self {
            stored: ImuBuffer::with_len(len),
            downsampled: ImuElements::zero(),
            delayed: ImuElements::zero(),
            run_updates: false,
            dt_imu_avg: DEFAULT_DT_IMU_AVG,
        }
    }

    /// The FIFO, upstream `storedIMU`.
    #[must_use]
    pub const fn stored(&self) -> &ImuBuffer {
        &self.stored
    }

    /// Accumulator that has not yet been pushed, upstream `imuDataDownSampledNew`.
    #[must_use]
    pub const fn downsampled(&self) -> &ImuElements {
        &self.downsampled
    }

    /// Fusion-horizon sample, upstream `imuDataDelayed`.
    #[must_use]
    pub const fn delayed(&self) -> &ImuElements {
        &self.delayed
    }

    /// Whether the last push armed a prediction, upstream `runUpdates`.
    #[must_use]
    pub const fn run_updates(&self) -> bool {
        self.run_updates
    }

    /// Ingest one gyro/accel frame, upstream `NavEKF3_core::readIMUData`.
    ///
    /// Returns true when a downsampled sample was pushed (the core would
    /// set `runUpdates` and extract `imuDataDelayed`). `start_predict` is
    /// the frontend permission; it is ignored once the accumulator reaches
    /// twice the target step, matching the "more than twice the target time
    /// has lapsed" override.
    pub fn read_imu_data(&mut self, sample: ImuRawSample, start_predict: bool) -> bool {
        self.update_dt_imu_avg(sample.dt);
        let imu_new = ImuElements::from_raw(sample);

        self.downsampled.del_ang_dt += imu_new.del_ang_dt;
        self.downsampled.del_vel_dt += imu_new.del_vel_dt;
        self.downsampled.gyro_index = imu_new.gyro_index;
        self.downsampled.accel_index = imu_new.accel_index;
        self.downsampled.del_ang += imu_new.del_ang;
        self.downsampled.del_vel += imu_new.del_vel;

        let half_imu = self.dt_imu_avg * 0.5 as Ftype;
        let at_target = self.downsampled.del_ang_dt >= EKF_TARGET_DT - half_imu && start_predict;
        let overdue = self.downsampled.del_ang_dt >= 2.0 as Ftype * EKF_TARGET_DT;
        if !at_target && !overdue {
            return false;
        }

        self.downsampled.time_ms = imu_new.time_ms;
        self.stored.push_youngest_element(self.downsampled);
        self.delayed = self.stored.get_oldest_element();
        let min_dt = 0.1 as Ftype * EKF_TARGET_DT;
        if self.delayed.del_ang_dt < min_dt {
            self.delayed.del_ang_dt = min_dt;
        }
        if self.delayed.del_vel_dt < min_dt {
            self.delayed.del_vel_dt = min_dt;
        }
        self.downsampled = ImuElements::zero();
        self.run_updates = true;
        true
    }

    /// Spike-and-lowpass on the IMU period, upstream `dtIMUavg` update.
    fn update_dt_imu_avg(&mut self, dt: Ftype) {
        if dt <= 0.0 as Ftype {
            return;
        }
        let lo = self.dt_imu_avg * 0.5 as Ftype;
        let hi = self.dt_imu_avg * 2.0 as Ftype;
        let clipped = if dt < lo {
            lo
        } else if dt > hi {
            hi
        } else {
            dt
        };
        self.dt_imu_avg = 0.02 as Ftype * clipped + 0.98 as Ftype * self.dt_imu_avg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Ftype, b: Ftype) {
        let err = if a > b { a - b } else { b - a };
        assert!(err < 1.0e-6 as Ftype, "{a} !~= {b}");
    }

    fn rate_sample(gyro_x: Ftype, accel_x: Ftype, dt: Ftype, time_ms: u32) -> ImuRawSample {
        ImuRawSample {
            gyro: Vector3::new(gyro_x, 0.0 as Ftype, 0.0 as Ftype),
            accel: Vector3::new(accel_x, 0.0 as Ftype, 0.0 as Ftype),
            dt,
            time_ms,
            gyro_index: 0,
            accel_index: 0,
        }
    }

    #[test]
    fn raw_gyro_accel_become_delta_angle_and_velocity() {
        let sample = rate_sample(0.5 as Ftype, 10.0 as Ftype, 0.004 as Ftype, 40);
        let el = ImuElements::from_raw(sample);
        near(el.del_ang.x, 0.002 as Ftype);
        near(el.del_vel.x, 0.04 as Ftype);
        near(el.del_ang_dt, 0.004 as Ftype);
        near(el.del_vel_dt, 0.004 as Ftype);
        assert_eq!(el.time_ms, 40);
    }

    #[test]
    fn short_frame_accumulates_without_pushing() {
        let mut ring = ImuSampleRing::with_buffer_len(4);
        let pushed = ring.read_imu_data(rate_sample(1.0 as Ftype, 2.0 as Ftype, 0.004 as Ftype, 4), true);
        assert!(!pushed);
        assert!(!ring.run_updates());
        near(ring.downsampled().del_ang.x, 0.004 as Ftype);
        near(ring.downsampled().del_vel.x, 0.008 as Ftype);
        near(ring.downsampled().del_ang_dt, 0.004 as Ftype);
        assert_eq!(ring.stored().youngest_index(), 0);
    }

    #[test]
    fn target_dt_pushes_downsampled_deltas() {
        let mut ring = ImuSampleRing::with_buffer_len(4);
        // Three 4 ms frames = 12 ms, the EKF target step.
        assert!(!ring.read_imu_data(rate_sample(1.0 as Ftype, 10.0 as Ftype, 0.004 as Ftype, 4), true));
        assert!(!ring.read_imu_data(rate_sample(1.0 as Ftype, 10.0 as Ftype, 0.004 as Ftype, 8), true));
        assert!(ring.read_imu_data(rate_sample(1.0 as Ftype, 10.0 as Ftype, 0.004 as Ftype, 12), true));
        assert!(ring.run_updates());
        near(ring.downsampled().del_ang_dt, 0.0 as Ftype);

        // First write is at index 1; oldest is still the zeroed slot at 2.
        assert_eq!(ring.stored().youngest_index(), 1);
        assert_eq!(ring.stored().oldest_index(), 2);
        assert!(!ring.stored().is_filled());

        let youngest = ring.stored().get(1);
        near(youngest.del_ang.x, 0.012 as Ftype);
        near(youngest.del_vel.x, 0.12 as Ftype);
        near(youngest.del_ang_dt, 0.012 as Ftype);
        assert_eq!(youngest.time_ms, 12);
    }

    #[test]
    fn withheld_predict_waits_until_twice_target_dt() {
        let mut ring = ImuSampleRing::with_buffer_len(4);
        for i in 1_u32..=5 {
            assert!(
                !ring.read_imu_data(
                    rate_sample(0.0 as Ftype, 0.0 as Ftype, 0.004 as Ftype, i * 4),
                    false
                ),
                "frame {i} should still be accumulating"
            );
        }
        assert!(!ring.run_updates());
        assert!(ring.read_imu_data(
            rate_sample(0.0 as Ftype, 0.0 as Ftype, 0.004 as Ftype, 24),
            false
        ));
        near(ring.stored().get(1).del_ang_dt, 0.024 as Ftype);
        assert_eq!(ring.stored().get(1).time_ms, 24);
    }

    #[test]
    fn ring_wrap_latches_filled_and_oldest_is_first_write() {
        let mut buf = ImuBuffer::with_len(4);
        for i in 1_u32..=4 {
            buf.push_youngest_element(ImuElements {
                time_ms: i * 12,
                ..ImuElements::zero()
            });
        }
        assert!(buf.is_filled());
        assert_eq!(buf.youngest_index(), 0);
        assert_eq!(buf.oldest_index(), 1);
        assert_eq!(buf.get_oldest_element().time_ms, 12);

        buf.push_youngest_element(ImuElements {
            time_ms: 60,
            ..ImuElements::zero()
        });
        assert_eq!(buf.oldest_index(), 2);
        assert_eq!(buf.get_oldest_element().time_ms, 24);
    }
}
