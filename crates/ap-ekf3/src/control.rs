//! Filter-mode control, upstream `AP_NavEKF3_Control.cpp`.
//!
//! This slice is the latch that `UpdateFilter` runs before IMU read and
//! prediction: [`FilterControl::control_filter_modes`] records arming,
//! [`detect_flight`](FilterControl::detect_flight) latches `inFlight` /
//! `onGround`, and [`set_inhibit_gps`](FilterControl::set_inhibit_gps)
//! is the historical `setInhibitGPS` handshake (removed from DAL later;
//! the aiding-mode gate is still the same). Covariance and fusion are
//! not here.
//!
//! # inFlight / onGround
//!
//! Upstream `detectFlight` uses two booleans because later algorithms
//! need different certainty. `onGround` is high certainty we are not
//! flying; `inFlight` is high certainty we are. Both may be false when
//! the status is uncertain; they cannot both be true.
//!
//! Plane (`assume_zero_sideslip`) combines arm status with ground speed,
//! airspeed, and height change. Disarmed is always on the ground.
//!
//! # Inhibit GPS
//!
//! `setInhibitGPS` returns 1 and sets `gpsInhibit` only when the core is
//! not already in `AID_ABSOLUTE` and the motors are not armed. A set
//! flag makes `readyToUseGPS` false, so `setAidingMode` will not promote
//! `AID_NONE` to `AID_ABSOLUTE`.

use ap_math::Ftype;

/// Position/velocity aiding mode, upstream `NavEKF3_core::AidingMode`.
///
/// Discriminant values match the C++ enum so a sitl-diff dump can compare
/// the integer without a translation table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AidingMode {
    /// GPS or another absolute position reference. Upstream `AID_ABSOLUTE`.
    Absolute = 0,
    /// Attitude and height only. Upstream `AID_NONE`.
    None = 1,
    /// Optical flow / body-odometry, relative position. Upstream `AID_RELATIVE`.
    Relative = 2,
}

/// Ground-speed trigger used by Plane `detectFlight`, 5 m/s.
const GND_SPD_THRESHOLD: Ftype = 5.0;
/// True-airspeed trigger used by Plane `detectFlight`, 10 m/s TAS.
const HIGH_AIRSPEED_TAS: Ftype = 10.0;
/// Height-change trigger used by Plane `detectFlight`, 10 m from origin.
const LARGE_HGT_CHANGE: Ftype = 10.0;
/// Copter `get_time_flying_ms` latch, 5 s.
const TIME_FLYING_LATCH_MS: u32 = 5000;
/// Copter height-above-takeoff latch, 1.5 m (NED down is negative up).
const COPTER_HGT_LATCH: Ftype = 1.5;

/// Filter-mode latch, the `NavEKF3_core` fields `controlFilterModes` writes.
///
/// Sensor fusion is not here: tests (and later cores) poke the flight
/// cues that `detectFlight` would have read from DAL / GPS / airspeed.
#[derive(Debug, Clone)]
pub struct FilterControl {
    motors_armed: bool,
    prev_motors_armed: bool,
    on_ground: bool,
    in_flight: bool,
    prev_on_ground: bool,
    prev_in_flight: bool,
    gps_inhibit: bool,
    aiding_mode: AidingMode,
    aiding_mode_prev: AidingMode,
    assume_zero_sideslip: bool,
    takeoff_expected: bool,
    gps_ready: bool,
    vel_north: Ftype,
    vel_east: Ftype,
    gps_spd_accuracy: Ftype,
    true_airspeed: Option<Ftype>,
    hgt_mea: Ftype,
    pos_down: Ftype,
    pos_down_at_takeoff: Ftype,
    time_flying_ms: u32,
    imu_sample_time_ms: u32,
    time_at_arming_ms: u32,
}

impl Default for FilterControl {
    fn default() -> Self {
        Self::new()
    }
}

impl FilterControl {
    /// Bootstrap defaults from `NavEKF3_core::InitialiseVariables`.
    ///
    /// `onGround = true`, `inFlight = false`, `PV_AidingMode = AID_NONE`,
    /// `gpsInhibit = false`. Plane is the default vehicle class.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            motors_armed: false,
            prev_motors_armed: false,
            on_ground: true,
            in_flight: false,
            prev_on_ground: true,
            prev_in_flight: false,
            gps_inhibit: false,
            aiding_mode: AidingMode::None,
            aiding_mode_prev: AidingMode::None,
            assume_zero_sideslip: true,
            takeoff_expected: false,
            gps_ready: false,
            vel_north: 0.0 as Ftype,
            vel_east: 0.0 as Ftype,
            gps_spd_accuracy: 0.0 as Ftype,
            true_airspeed: None,
            hgt_mea: 0.0 as Ftype,
            pos_down: 0.0 as Ftype,
            pos_down_at_takeoff: 0.0 as Ftype,
            time_flying_ms: 0,
            imu_sample_time_ms: 0,
            time_at_arming_ms: 0,
        }
    }

    /// Re-apply bootstrap defaults, upstream `InitialiseVariables`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// High certainty we are not flying, upstream `onGround`.
    #[must_use]
    pub const fn on_ground(&self) -> bool {
        self.on_ground
    }

    /// High certainty we are flying, upstream `inFlight`.
    #[must_use]
    pub const fn in_flight(&self) -> bool {
        self.in_flight
    }

    /// Previous-frame `onGround`, upstream `prevOnGround`.
    #[must_use]
    pub const fn prev_on_ground(&self) -> bool {
        self.prev_on_ground
    }

    /// Previous-frame `inFlight`, upstream `prevInFlight`.
    #[must_use]
    pub const fn prev_in_flight(&self) -> bool {
        self.prev_in_flight
    }

    /// External GPS inhibit flag, upstream `gpsInhibit`.
    #[must_use]
    pub const fn gps_inhibit(&self) -> bool {
        self.gps_inhibit
    }

    /// Current aiding mode, upstream `PV_AidingMode`.
    #[must_use]
    pub const fn aiding_mode(&self) -> AidingMode {
        self.aiding_mode
    }

    /// Whether the motors are armed, upstream `motorsArmed`.
    #[must_use]
    pub const fn motors_armed(&self) -> bool {
        self.motors_armed
    }

    /// IMU time captured on the arming edge, upstream `timeAtArming_ms`.
    #[must_use]
    pub const fn time_at_arming_ms(&self) -> u32 {
        self.time_at_arming_ms
    }

    /// Plane vs copter `detectFlight` branch, upstream `assume_zero_sideslip()`.
    pub fn set_assume_zero_sideslip(&mut self, assume: bool) {
        self.assume_zero_sideslip = assume;
    }

    /// Arm status the next [`control_filter_modes`](Self::control_filter_modes)
    /// will read, stand-in for `dal.get_armed()`.
    pub fn set_motors_armed(&mut self, armed: bool) {
        self.motors_armed = armed;
    }

    /// Takeoff-expected cue, stand-in for `dal.get_takeoff_expected()`.
    pub fn set_takeoff_expected(&mut self, expected: bool) {
        self.takeoff_expected = expected;
    }

    /// GPS NE velocity used by Plane `detectFlight`, m/s.
    pub fn set_ground_velocity(&mut self, north: Ftype, east: Ftype) {
        self.vel_north = north;
        self.vel_east = east;
    }

    /// GPS speed accuracy used in the Plane ground-speed test, m/s.
    pub fn set_gps_speed_accuracy(&mut self, accuracy: Ftype) {
        self.gps_spd_accuracy = accuracy;
    }

    /// Healthy TAS for the Plane airspeed test. `None` is "no airspeed".
    pub fn set_true_airspeed(&mut self, tas: Option<Ftype>) {
        self.true_airspeed = tas;
    }

    /// Baro / height measurement used as `|hgtMea| > 10`, metres.
    pub fn set_height_measurement(&mut self, hgt_mea: Ftype) {
        self.hgt_mea = hgt_mea;
    }

    /// NED down position used by the copter height latch, metres.
    pub fn set_position_down(&mut self, pos_down: Ftype) {
        self.pos_down = pos_down;
    }

    /// Stand-in for `dal.get_time_flying_ms()`.
    pub fn set_time_flying_ms(&mut self, time_ms: u32) {
        self.time_flying_ms = time_ms;
    }

    /// IMU clock used to stamp `timeAtArming_ms`.
    pub fn set_imu_sample_time_ms(&mut self, time_ms: u32) {
        self.imu_sample_time_ms = time_ms;
    }

    /// Alignment / GPS-data bits of `readyToUseGPS`, excluding `gpsInhibit`.
    pub fn set_gps_ready(&mut self, ready: bool) {
        self.gps_ready = ready;
    }

    /// Force an aiding mode. Tests use this to reach `AID_ABSOLUTE` so
    /// [`set_inhibit_gps`](Self::set_inhibit_gps) can reject.
    pub fn set_aiding_mode_for_test(&mut self, mode: AidingMode) {
        self.aiding_mode = mode;
        self.aiding_mode_prev = mode;
    }

    /// Historical `NavEKF3_core::setInhibitGPS`.
    ///
    /// Returns `1` and latches `gpsInhibit` when the command is accepted.
    /// Returns `0` when already in `AID_ABSOLUTE` or the motors are armed.
    pub fn set_inhibit_gps(&mut self) -> u8 {
        if self.aiding_mode == AidingMode::Absolute || self.motors_armed {
            0
        } else {
            self.gps_inhibit = true;
            1
        }
    }

    /// `readyToUseGPS` inhibit half: data/alignment ready and not inhibited.
    #[must_use]
    pub const fn ready_to_use_gps(&self) -> bool {
        self.gps_ready && !self.gps_inhibit
    }

    /// Upstream `controlFilterModes`: arm edge, flight latch, aiding mode.
    ///
    /// Wind/mag learning and tilt/yaw alignment checks are not in this slice.
    pub fn control_filter_modes(&mut self) {
        if self.motors_armed && !self.prev_motors_armed {
            self.time_at_arming_ms = self.imu_sample_time_ms;
        }
        self.prev_motors_armed = self.motors_armed;
        self.detect_flight();
        self.set_aiding_mode();
    }

    /// Upstream `detectFlight`.
    ///
    /// Public so tests can run the latch without the aiding-mode side
    /// effects of [`control_filter_modes`](Self::control_filter_modes).
    pub fn detect_flight(&mut self) {
        if self.assume_zero_sideslip {
            self.detect_flight_plane();
        } else {
            self.detect_flight_copter();
        }
        if self.on_ground {
            self.pos_down_at_takeoff = self.pos_down;
        }
        self.prev_on_ground = self.on_ground;
        self.prev_in_flight = self.in_flight;
    }

    fn detect_flight_plane(&mut self) {
        let gnd_spd_sq = self.vel_north * self.vel_north + self.vel_east * self.vel_east;
        let threshold_sq =
            GND_SPD_THRESHOLD * GND_SPD_THRESHOLD + self.gps_spd_accuracy * self.gps_spd_accuracy;
        let high_gnd_spd = gnd_spd_sq > threshold_sq;
        let high_air_spd = match self.true_airspeed {
            Some(tas) => tas > HIGH_AIRSPEED_TAS,
            None => false,
        };
        let large_hgt_change = abs_ftype(self.hgt_mea) > LARGE_HGT_CHANGE;

        if self.motors_armed {
            self.on_ground = false;
            if high_gnd_spd && (self.takeoff_expected || high_air_spd || large_hgt_change) {
                self.in_flight = true;
            }
        } else {
            self.on_ground = true;
            self.in_flight = false;
        }
    }

    fn detect_flight_copter(&mut self) {
        if self.motors_armed {
            self.on_ground = false;
        } else {
            self.in_flight = false;
            self.on_ground = true;
        }
        if !self.on_ground {
            if self.pos_down - self.pos_down_at_takeoff < -COPTER_HGT_LATCH {
                self.in_flight = true;
            }
            if self.time_flying_ms > TIME_FLYING_LATCH_MS {
                self.in_flight = true;
            }
        }
    }

    /// Upstream `setAidingMode`, GPS-inhibit half only.
    ///
    /// Flow / beacon / ext-nav promotion and absolute-mode timeouts are
    /// not here. `AID_NONE` becomes `AID_ABSOLUTE` when
    /// [`ready_to_use_gps`](Self::ready_to_use_gps) is true.
    fn set_aiding_mode(&mut self) {
        self.aiding_mode_prev = self.aiding_mode;
        match self.aiding_mode {
            AidingMode::None | AidingMode::Relative => {
                if self.ready_to_use_gps() {
                    self.aiding_mode = AidingMode::Absolute;
                }
            }
            AidingMode::Absolute => {}
        }
    }
}

fn abs_ftype(value: Ftype) -> Ftype {
    if value < 0.0 as Ftype {
        -value
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_latch_exclusive(ctrl: &FilterControl) {
        assert!(
            !(ctrl.on_ground() && ctrl.in_flight()),
            "onGround and inFlight cannot both be true"
        );
    }

    #[test]
    fn bootstrap_is_on_ground_not_in_flight() {
        let ctrl = FilterControl::new();
        assert!(ctrl.on_ground());
        assert!(!ctrl.in_flight());
        assert!(!ctrl.gps_inhibit());
        assert_eq!(ctrl.aiding_mode(), AidingMode::None);
        assert!(!ctrl.motors_armed());
        assert_latch_exclusive(&ctrl);
    }

    #[test]
    fn set_inhibit_gps_accepted_when_disarmed_and_not_absolute() {
        let mut ctrl = FilterControl::new();
        assert_eq!(ctrl.set_inhibit_gps(), 1);
        assert!(ctrl.gps_inhibit());
        assert!(!ctrl.ready_to_use_gps());
    }

    #[test]
    fn set_inhibit_gps_rejected_when_armed_or_absolute() {
        let mut armed = FilterControl::new();
        armed.set_motors_armed(true);
        assert_eq!(armed.set_inhibit_gps(), 0);
        assert!(!armed.gps_inhibit());

        let mut absolute = FilterControl::new();
        absolute.set_aiding_mode_for_test(AidingMode::Absolute);
        assert_eq!(absolute.set_inhibit_gps(), 0);
        assert!(!absolute.gps_inhibit());
    }

    #[test]
    fn inhibit_blocks_absolute_aiding_promotion() {
        let mut ctrl = FilterControl::new();
        ctrl.set_gps_ready(true);
        assert_eq!(ctrl.set_inhibit_gps(), 1);
        ctrl.control_filter_modes();
        assert_eq!(ctrl.aiding_mode(), AidingMode::None);
        assert!(!ctrl.ready_to_use_gps());

        let mut allowed = FilterControl::new();
        allowed.set_gps_ready(true);
        allowed.control_filter_modes();
        assert_eq!(allowed.aiding_mode(), AidingMode::Absolute);
        assert!(allowed.ready_to_use_gps());
    }

    #[test]
    fn plane_inflight_latch_needs_speed_and_a_cue() {
        let mut ctrl = FilterControl::new();
        ctrl.set_imu_sample_time_ms(12_000);
        ctrl.set_motors_armed(true);
        ctrl.set_ground_velocity(6.0 as Ftype, 0.0 as Ftype);
        ctrl.set_true_airspeed(Some(12.0 as Ftype));
        ctrl.control_filter_modes();

        assert!(!ctrl.on_ground());
        assert!(ctrl.in_flight());
        assert_eq!(ctrl.time_at_arming_ms(), 12_000);
        assert_latch_exclusive(&ctrl);
        // prev is stored at the end of detectFlight for the next frame.
        assert!(!ctrl.prev_on_ground());
        assert!(ctrl.prev_in_flight());
    }

    #[test]
    fn plane_armed_without_cues_is_uncertain() {
        let mut ctrl = FilterControl::new();
        ctrl.set_motors_armed(true);
        ctrl.control_filter_modes();
        assert!(!ctrl.on_ground());
        assert!(!ctrl.in_flight());
        assert_latch_exclusive(&ctrl);
    }

    #[test]
    fn plane_disarm_relatches_on_ground() {
        let mut ctrl = FilterControl::new();
        ctrl.set_motors_armed(true);
        ctrl.set_ground_velocity(8.0 as Ftype, 0.0 as Ftype);
        ctrl.set_takeoff_expected(true);
        ctrl.control_filter_modes();
        assert!(ctrl.in_flight());

        ctrl.set_motors_armed(false);
        ctrl.control_filter_modes();
        assert!(ctrl.on_ground());
        assert!(!ctrl.in_flight());
        assert_latch_exclusive(&ctrl);
    }

    #[test]
    fn plane_height_change_with_ground_speed_latches_inflight() {
        let mut ctrl = FilterControl::new();
        ctrl.set_motors_armed(true);
        ctrl.set_ground_velocity(6.0 as Ftype, 0.0 as Ftype);
        ctrl.set_height_measurement(11.0 as Ftype);
        ctrl.detect_flight();
        assert!(ctrl.in_flight());
        assert!(!ctrl.on_ground());
        assert_latch_exclusive(&ctrl);
    }

    #[test]
    fn aiding_mode_discriminants_match_upstream() {
        assert_eq!(AidingMode::Absolute as u8, 0);
        assert_eq!(AidingMode::None as u8, 1);
        assert_eq!(AidingMode::Relative as u8, 2);
    }
}
