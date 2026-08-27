//! INS_HNTCH_* parameter binding, upstream `HarmonicNotchFilterParams` and
//! `AP_InertialSensor::update_gyro_filters`. FW-011.

use ap_filter::harmonic::{CompositeNotches, HarmonicNotchParams, TrackingMode};

use crate::ImuInstance;

/// Max motors for dynamic-harmonic notch centres.
pub const INS_HNTCH_MAX_MOTORS: usize = 8;

/// Active motor count from runtime mask or non-zero RPM entries.
#[must_use]
pub fn motor_count_from_runtime(motor_mask: u32, motor_rpm: &[f32; INS_HNTCH_MAX_MOTORS]) -> u8 {
    if motor_mask != 0 {
        return (motor_mask.count_ones()).min(INS_HNTCH_MAX_MOTORS as u32) as u8;
    }
    let n = motor_rpm.iter().filter(|&&r| r > 0.0).count() as u8;
    if n > 0 { n } else { 1 }
}


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
    /// With [`opts::DYNAMIC_HARMONIC`] set, allocates one centre per motor.
    #[must_use]
    pub fn num_notches(&self, motor_count: u8) -> u8 {
        if !self.enable {
            return 0;
        }
        if self.options & opts::DYNAMIC_HARMONIC != 0 {
            return motor_count.max(1).min(INS_HNTCH_MAX_MOTORS as u8);
        }
        1
    }

    /// Whether dynamic-harmonic mode allocates per-motor notch centres.
    #[must_use]
    pub const fn dynamic_harmonic(&self) -> bool {
        self.options & opts::DYNAMIC_HARMONIC != 0
    }

    /// Per-motor notch centre frequencies for dynamic-harmonic tracking.
    #[must_use]
    pub fn calculate_notch_centers(
        &self,
        throttle: f32,
        motor_rpm: &[f32; INS_HNTCH_MAX_MOTORS],
        motor_count: u8,
    ) -> ([f32; INS_HNTCH_MAX_MOTORS], u8) {
        let mut centers = [0.0_f32; INS_HNTCH_MAX_MOTORS];
        let count = if self.dynamic_harmonic() {
            motor_count.max(1)
        } else {
            1
        };
        for i in 0..count as usize {
            let rpm = if self.dynamic_harmonic() {
                let r = motor_rpm[i];
                if r > 0.0 { Some(r) } else { None }
            } else {
                motor_rpm.first().copied().filter(|&r| r > 0.0)
            };
            centers[i] = self.calculate_center_freq_hz(throttle, rpm);
        }
        (centers, count)
    }

    /// Calculate the notch centre frequency from throttle and optional RPM,
    /// upstream `AP_Vehicle::update_dynamic_notch`.
    #[must_use]
    pub fn calculate_center_freq_hz(&self, throttle: f32, rpm: Option<f32>) -> f32 {
        if !self.enable {
            return 0.0;
        }

        let ref_freq = self.freq_hz;
        let reference = self.reference;

        if reference <= 0.0 {
            return ref_freq;
        }

        match self.tracking_mode() {
            TrackingMode::Fixed => ref_freq,
            TrackingMode::UpdateThrottle => {
                ref_freq * libm::sqrtf(throttle.max(0.0) / reference)
            }
            TrackingMode::UpdateRpm | TrackingMode::UpdateRpm2 => match rpm {
                Some(r) if r > 0.0 => r * reference * (1.0 / 60.0),
                _ => 0.0,
            },
            // BLHeli, FFT, and per-motor dynamic harmonic are later slices.
            TrackingMode::UpdateBlHeli | TrackingMode::UpdateGyroFft => ref_freq,
        }
    }

    /// Retune a configured notch bank from live throttle/RPM inputs, upstream
    /// `HarmonicNotch::update_params` frequency half.
    pub fn update_notch_centers(
        &self,
        imu: &mut ImuInstance,
        throttle: f32,
        motor_rpm: &[f32; INS_HNTCH_MAX_MOTORS],
        motor_count: u8,
    ) {
        if !self.enable || self.tracking_mode() == TrackingMode::Fixed {
            return;
        }
        let (centers, count) = self.calculate_notch_centers(throttle, motor_rpm, motor_count);
        imu.update_gyro_notch_centers(&centers[..count as usize]);
    }

    /// Apply gyro filters to one IMU, upstream `update_gyro_filters` for one
    /// instance.
    pub fn apply_gyro_filters_to_imu(
        &self,
        imu: &mut ImuInstance,
        sample_rate_hz: f32,
        gyro_lowpass_hz: f32,
        motor_count: u8,
    ) {
        let num = self.num_notches(motor_count);
        if self.enable && num > 0 {
            imu.set_gyro_notch(
                sample_rate_hz,
                self.harmonic_notch_params(),
                num,
            );
        }
        imu.set_gyro_filter(sample_rate_hz, gyro_lowpass_hz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DEFAULT_GYRO_FILTER_HZ;

    #[test]
    fn dynamic_harmonic_allocates_one_notch_per_motor() {
        let params = InsHntchParams {
            enable: true,
            options: opts::DYNAMIC_HARMONIC,
            ..InsHntchParams::default()
        };
        assert_eq!(params.num_notches(4), 4);
        assert_eq!(params.num_notches(1), 1);
    }

    #[test]
    fn dynamic_harmonic_rpm_tracking_per_motor() {
        let params = InsHntchParams {
            enable: true,
            reference: 2.0,
            mode: 2,
            options: opts::DYNAMIC_HARMONIC,
            ..InsHntchParams::default()
        };
        let motor_rpm = [3000.0_f32, 6000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (centers, count) = params.calculate_notch_centers(0.0, &motor_rpm, 2);
        assert_eq!(count, 2);
        assert!((centers[0] - 100.0).abs() < 0.01, "motor0: {}", centers[0]);
        assert!((centers[1] - 200.0).abs() < 0.01, "motor1: {}", centers[1]);

        let mut imu = ImuInstance::new();
        params.apply_gyro_filters_to_imu(&mut imu, 1000.0, DEFAULT_GYRO_FILTER_HZ, 2);
        params.update_notch_centers(&mut imu, 0.0, &motor_rpm, 2);
        let c0 = imu.gyro_notch_center(0).expect("notch0");
        let c1 = imu.gyro_notch_center(1).expect("notch1");
        assert!((c0 - 100.0).abs() < 2.0, "retuned motor0: {c0}");
        assert!((c1 - 200.0).abs() < 2.0, "retuned motor1: {c1}");
    }
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
    fn throttle_tracking_scales_with_sqrt_throttle() {
        let params = InsHntchParams {
            enable: true,
            freq_hz: 100.0,
            reference: 0.25,
            mode: 1,
            ..InsHntchParams::default()
        };
        let at_ref = params.calculate_center_freq_hz(0.25, None);
        assert!((at_ref - 100.0).abs() < 0.01, "at reference throttle -> ref freq");

        let at_quarter = params.calculate_center_freq_hz(0.0625, None);
        assert!((at_quarter - 50.0).abs() < 0.01, "quarter throttle -> half freq");
    }

    #[test]
    fn rpm_tracking_scales_with_rpm_and_reference() {
        let params = InsHntchParams {
            enable: true,
            freq_hz: 80.0,
            reference: 2.0,
            mode: 2,
            ..InsHntchParams::default()
        };
        // 3000 RPM * 2.0 ref * (1/60) = 100 Hz
        let center = params.calculate_center_freq_hz(0.0, Some(3000.0));
        assert!((center - 100.0).abs() < 0.01);

        assert_eq!(params.calculate_center_freq_hz(0.0, None), 0.0);
    }

    #[test]
    fn zero_reference_uses_configured_centre() {
        let params = InsHntchParams {
            enable: true,
            freq_hz: 80.0,
            reference: 0.0,
            mode: 1,
            ..InsHntchParams::default()
        };
        assert_eq!(params.calculate_center_freq_hz(0.5, None), 80.0);
    }

    #[test]
    fn disabled_notch_leaves_passthrough() {
        let params = InsHntchParams::default();
        let mut imu = ImuInstance::new();
        params.apply_gyro_filters_to_imu(&mut imu, 8000.0, DEFAULT_GYRO_FILTER_HZ, 1);

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
        params.apply_gyro_filters_to_imu(&mut with_notch, 8000.0, DEFAULT_GYRO_FILTER_HZ, 1);

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

    #[test]
    fn throttle_tracking_retunes_notch_centre() {
        let params = InsHntchParams {
            enable: true,
            freq_hz: 100.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 1,
            reference: 1.0,
            mode: 1,
            freq_min_ratio: 0.5,
            ..InsHntchParams::default()
        };

        let mut imu = ImuInstance::new();
        params.apply_gyro_filters_to_imu(&mut imu, 1000.0, DEFAULT_GYRO_FILTER_HZ, 1);
        params.update_notch_centers(&mut imu, 1.0, &[0.0; INS_HNTCH_MAX_MOTORS], 1);

        assert!(imu.gyro_notch_is_initialised());
        let center = imu.gyro_notch_center(0).expect("notch placed");
        assert!((center - 100.0).abs() < 1.0, "full throttle -> 100 Hz, got {center}");

        for _ in 0..32 {
            params.update_notch_centers(&mut imu, 0.25, &[0.0; INS_HNTCH_MAX_MOTORS], 1);
        }
        let center = imu.gyro_notch_center(0).expect("notch retuned");
        assert!((center - 50.0).abs() < 1.0, "quarter throttle -> 50 Hz, got {center}");
    }
}
