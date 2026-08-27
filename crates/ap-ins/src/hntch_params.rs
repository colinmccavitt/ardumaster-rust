//! INS_HNTCH_* parameter binding, upstream `HarmonicNotchFilterParams` and
//! `AP_InertialSensor::update_gyro_filters`. FW-011.

use ap_filter::harmonic::{CompositeNotches, HarmonicNotchParams, TrackingMode};

use crate::ImuInstance;

/// INS_HNTCH_OPTS bit flags, upstream `HarmonicNotchFilterParams::Options`.
pub mod opts {
    pub const DOUBLE_NOTCH: u16 = 1 << 0;
    pub const DYNAMIC_HARMONIC: u16 = 1 << 1;
    pub const LOOP_RATE_UPDATE: u16 = 1 << 2;
    pub const ENABLE_ON_ALL_IMUS: u16 = 1 << 3;
    pub const TRIPLE_NOTCH: u16 = 1 << 4;
    pub const TREAT_LOW_AS_MIN: u16 = 1 << 5;
    pub const QUINTUPLE_NOTCH: u16 = 1 << 6;
}

/// Plane INS harmonic-notch parameters, upstream INS_HNTCH_*.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InsHntchParams {
    pub enable: bool,
    pub freq_hz: f32,
    pub bandwidth_hz: f32,
    pub attenuation_db: f32,
    pub harmonics: u32,
    pub reference: f32,
    pub mode: u8,
    pub options: u16,
    pub freq_min_ratio: f32,
}

impl Default for InsHntchParams {
    fn default() -> Self {
        Self {
            enable: false,
            freq_hz: 80.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 3,
            reference: 0.0,
            mode: 1,
            options: 0,
            freq_min_ratio: 1.0,
        }
    }
}

impl InsHntchParams {
    /// Whether notches should run on every IMU, upstream `EnableOnAllIMUs`.
    #[must_use]
    pub const fn enable_on_all_imus(&self) -> bool {
        self.options & opts::ENABLE_ON_ALL_IMUS != 0
    }

    /// Composite notch count from INS_HNTCH_OPTS, upstream
    /// `num_composite_notches`.
    #[must_use]
    pub fn composite_notches(&self) -> CompositeNotches {
        if self.options & opts::DOUBLE_NOTCH != 0 {
            return CompositeNotches::Double;
        }
        if self.options & opts::TRIPLE_NOTCH != 0 {
            return CompositeNotches::Triple;
        }
        if self.options & opts::QUINTUPLE_NOTCH != 0 {
            return CompositeNotches::Quintuple;
        }
        CompositeNotches::Single
    }

    /// Tracking mode from INS_HNTCH_MODE.
    #[must_use]
    pub fn tracking_mode(&self) -> TrackingMode {
        match self.mode {
            0 => TrackingMode::Fixed,
            1 => TrackingMode::UpdateThrottle,
            2 => TrackingMode::UpdateRpm,
            3 => TrackingMode::UpdateBlHeli,
            4 => TrackingMode::UpdateGyroFft,
            5 => TrackingMode::UpdateRpm2,
            _ => TrackingMode::Fixed,
        }
    }

    /// Build filter configuration for [`ImuInstance::set_gyro_notch`].
    #[must_use]
    pub fn harmonic_notch_params(&self) -> HarmonicNotchParams {
        HarmonicNotchParams {
            center_freq_hz: self.freq_hz,
            bandwidth_hz: self.bandwidth_hz,
            attenuation_db: self.attenuation_db,
            freq_min_ratio: self.freq_min_ratio,
            harmonics: self.harmonics,
            composite_notches: self.composite_notches(),
            tracking_mode: self.tracking_mode(),
            treat_low_as_min: self.options & opts::TREAT_LOW_AS_MIN != 0,
        }
    }

    /// Number of independent centre frequencies, upstream `_num_notches`.
    ///
    /// Dynamic-harmonic and FFT multi-source modes allocate one notch per
    /// motor or peak; those sources are not ported yet, so this returns 1.
    #[must_use]
    pub fn num_notches(&self) -> u8 {
        if !self.enable {
            return 0;
        }
        if self.options & opts::DYNAMIC_HARMONIC != 0 {
            return 1;
        }
        1
    }

    /// Apply gyro filters to one IMU, upstream `update_gyro_filters` for one
    /// instance.
    pub fn apply_gyro_filters_to_imu(
        &self,
        imu: &mut ImuInstance,
        sample_rate_hz: f32,
        gyro_lowpass_hz: f32,
    ) {
        if self.enable && self.num_notches() > 0 {
            imu.set_gyro_notch(
                sample_rate_hz,
                self.harmonic_notch_params(),
                self.num_notches(),
            );
        }
        imu.set_gyro_filter(sample_rate_hz, gyro_lowpass_hz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DEFAULT_GYRO_FILTER_HZ;

    fn feed(imu: &mut ImuInstance, gyro: ap_math::vector3::Vector3f, t_us: &mut u64) {
        *t_us += 125;
        imu.notify_gyro_raw_sample(gyro, *t_us, 8000, *t_us);
    }

    #[test]
    fn composite_notches_follow_opts_priority() {
        let mut p = InsHntchParams::default();
        p.options = opts::TRIPLE_NOTCH;
        assert_eq!(p.composite_notches(), CompositeNotches::Triple);

        p.options = opts::DOUBLE_NOTCH | opts::TRIPLE_NOTCH;
        assert_eq!(
            p.composite_notches(),
            CompositeNotches::Double,
            "upstream checks double before triple"
        );
    }

    #[test]
    fn disabled_notch_leaves_passthrough() {
        let params = InsHntchParams::default();
        let mut imu = ImuInstance::new();
        params.apply_gyro_filters_to_imu(&mut imu, 8000.0, DEFAULT_GYRO_FILTER_HZ);

        let mut t = 1_000_000_u64;
        feed(&mut imu, ap_math::vector3::Vector3f::new(0.5, 0.0, 0.0), &mut t);
        feed(&mut imu, ap_math::vector3::Vector3f::new(0.5, 0.0, 0.0), &mut t);
        imu.update_gyro();
        assert!((imu.gyro().x - 0.5).abs() < 1e-3);
    }

    #[test]
    fn enabled_notch_attenuates_configured_centre() {
        let params = InsHntchParams {
            enable: true,
            freq_hz: 80.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 1,
            mode: 0,
            ..InsHntchParams::default()
        };

        let mut with_notch = ImuInstance::new();
        params.apply_gyro_filters_to_imu(&mut with_notch, 8000.0, DEFAULT_GYRO_FILTER_HZ);

        let mut without_notch = ImuInstance::new();
        without_notch.set_gyro_filter(8000.0, DEFAULT_GYRO_FILTER_HZ);

        let sample_rate = 8000.0;
        let mut t = 1_000_000_u64;
        feed(&mut with_notch, ap_math::vector3::Vector3f::zero(), &mut t);
        feed(&mut without_notch, ap_math::vector3::Vector3f::zero(), &mut t);

        for i in 1_u16..=8000 {
            let phase = 2.0 * core::f32::consts::PI * 80.0 * (f32::from(i) / sample_rate);
            let gyro = ap_math::vector3::Vector3f::new(libm::sinf(phase), 0.0, 0.0);
            feed(&mut with_notch, gyro, &mut t);
            feed(&mut without_notch, gyro, &mut t);
        }
        with_notch.update_gyro();
        without_notch.update_gyro();

        let notched = with_notch.gyro().x.abs();
        let plain = without_notch.gyro().x.abs();
        assert!(
            notched < plain * 0.5,
            "80 Hz tone should be attenuated (notched={notched}, plain={plain})"
        );
    }
}
