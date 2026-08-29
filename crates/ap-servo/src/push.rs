//! Hardware send step, upstream `SRV_Channels::push`. COP-030 leftover.
//!
//! Between "the pulse is decided" and "it is on the wire" sits a fan-out:
//! `hal.rcout->push()` commits the PWM frame, then every enabled output
//! protocol gets a chance to send the same values on its own bus. Order is
//! the point — a protocol that ran first would send last iteration's pulses.
//!
//! Compile-time `#if AP_*_ENABLED` gates become [`PushFeatures`] so a test
//! can stand up the board that compiled three of them in without linking
//! the libraries (ADR-0004). The protocol bodies themselves are not here:
//! this leftover is the dispatcher.
//!
//! `cork()` is the matching hold: a wrapper around `hal.rcout->cork()`
//! with no protocol fan-out. `zero_rc_outputs` corks, writes a zero pulse
//! to every channel, then runs the same fan-out.

use ap_hal::rc::RcOutput;

use crate::NUM_SERVO_CHANNELS;

/// COP-030 leftovers still outstanding after this slice.
///
/// Empty: `push()` and `upgrade_parameters` both landed here.
pub const REMAINING: &[&str] = &[];

/// Maximum CAN driver slots `push` will walk, upstream `HAL_NUM_CAN_IFACES`.
pub const MAX_CAN_DRIVERS: usize = 3;

/// Compile-time-style enable flags for the protocols `push` can visit.
///
/// Upstream gates each with `#if`. Those become flags here so the fan-out
/// can be tested without the protocol libraries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushFeatures {
    /// `AP_VOLZ_ENABLED` — `volz.update()`.
    pub volz: bool,
    /// `AP_SBUSOUTPUT_ENABLED` — `sbus.update()`.
    pub sbus: bool,
    /// `AP_ROBOTISSERVO_ENABLED` — `robotis.update()`.
    pub robotis: bool,
    /// `HAL_SUPPORT_RCOUT_SERIAL` — `blheli.update_telemetry()`.
    pub blheli: bool,
    /// `AP_FETTEC_ONEWIRE_ENABLED` — `fetteconwire.update()`.
    pub fettec: bool,
    /// `AP_KDECAN_ENABLED`. Still needs [`Self::kdecan_present`].
    pub kdecan: bool,
    /// `AP::kdecan() != nullptr`. Even with the feature on, the singleton
    /// can be missing, and upstream skips the update.
    pub kdecan_present: bool,
    /// `HAL_ENABLE_DRONECAN_DRIVERS` — walk the CAN driver list at all.
    pub can_drivers: bool,
    /// `AP_PICCOLOCAN_ENABLED`. Off, a PiccoloCAN slot falls through to
    /// the default branch and is not visited.
    pub piccolocan: bool,
}

impl PushFeatures {
    /// Every protocol compiled out. `rcout.push()` still runs.
    pub const NONE: Self = Self {
        volz: false,
        sbus: false,
        robotis: false,
        blheli: false,
        fettec: false,
        kdecan: false,
        kdecan_present: false,
        can_drivers: false,
        piccolocan: false,
    };

    /// Every gate open, including a live kdecan singleton.
    pub const ALL: Self = Self {
        volz: true,
        sbus: true,
        robotis: true,
        blheli: true,
        fettec: true,
        kdecan: true,
        kdecan_present: true,
        can_drivers: true,
        piccolocan: true,
    };
}

impl Default for PushFeatures {
    fn default() -> Self {
        Self::NONE
    }
}

/// One CAN driver slot, upstream `AP_CAN::Protocol` plus the second lookup.
///
/// The type comes from `get_driver_type`. DroneCAN and PiccoloCAN then do
/// a second lookup (`get_dronecan` / `get_pcan`) that can return null —
/// `present` is that result. A null pointer is a skip, not a visit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanDriver {
    /// `AP_CAN::Protocol::None` — the default branch, no visit.
    None,
    /// `AP_CAN::Protocol::DroneCAN`. `present` is `get_dronecan(i) != nullptr`.
    DroneCan {
        /// Whether the driver object exists.
        present: bool,
    },
    /// `AP_CAN::Protocol::PiccoloCAN`. Also needs [`PushFeatures::piccolocan`].
    PiccoloCan {
        /// Whether the driver object exists.
        present: bool,
    },
}

/// A visit `push` makes, in the order it makes them.
///
/// `Rcout` is always first. The rest exist only when their feature is on
/// and, for CAN, when the slot's second lookup succeeded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushVisit {
    /// `hal.rcout->push()`.
    Rcout,
    /// `volz.update()`.
    Volz,
    /// `sbus.update()`.
    Sbus,
    /// `robotis.update()`.
    Robotis,
    /// `blheli.update_telemetry()`.
    BlheliTelemetry,
    /// `fetteconwire.update()`.
    FettecOnewire,
    /// `AP::kdecan()->update()`.
    KdeCan,
    /// `ap_dronecan->SRV_push_servos()`. `driver` is the slot index, not
    /// a compact count of successful visits — a leading `None` slot still
    /// occupies index 0.
    DroneCan {
        /// `get_driver_type` index.
        driver: u8,
    },
    /// `ap_pcan->update()`.
    PiccoloCan {
        /// `get_driver_type` index.
        driver: u8,
    },
}

/// Hold writes until [`push`], upstream `SRV_Channels::cork`.
///
/// A wrapper around `hal.rcout->cork()`. No protocol is visited here:
/// they send on `push`, when the PWM frame is committed.
pub fn cork<R: RcOutput>(rcout: &mut R) {
    rcout.cork();
}

/// Walk the protocol fan-out in upstream order, not including `rcout.push`.
///
/// `out` is filled from the start; the return is how many were written.
/// Extra CAN drivers beyond the remaining slots are not visited — same
/// as a firmware built with fewer CAN ifaces.
pub fn protocol_visits(
    features: PushFeatures,
    can_drivers: &[CanDriver],
    out: &mut [PushVisit],
) -> usize {
    let mut n = 0;
    let mut push_visit = |v: PushVisit| -> bool {
        let Some(slot) = out.get_mut(n) else {
            return false;
        };
        *slot = v;
        n += 1;
        true
    };

    if features.volz && !push_visit(PushVisit::Volz) {
        return n;
    }
    if features.sbus && !push_visit(PushVisit::Sbus) {
        return n;
    }
    if features.robotis && !push_visit(PushVisit::Robotis) {
        return n;
    }
    if features.blheli && !push_visit(PushVisit::BlheliTelemetry) {
        return n;
    }
    if features.fettec && !push_visit(PushVisit::FettecOnewire) {
        return n;
    }
    if features.kdecan && features.kdecan_present && !push_visit(PushVisit::KdeCan) {
        return n;
    }
    if !features.can_drivers {
        return n;
    }
    for (i, drv) in can_drivers.iter().enumerate() {
        let Ok(driver) = u8::try_from(i) else {
            break;
        };
        let visit = match *drv {
            CanDriver::None => continue,
            CanDriver::DroneCan { present: true } => PushVisit::DroneCan { driver },
            CanDriver::DroneCan { present: false } => continue,
            CanDriver::PiccoloCan { present: true } if features.piccolocan => {
                PushVisit::PiccoloCan { driver }
            }
            CanDriver::PiccoloCan { .. } => continue,
        };
        if !push_visit(visit) {
            return n;
        }
    }
    n
}

/// Commit the PWM frame and fan out to the output protocols, upstream
/// `SRV_Channels::push`.
///
/// `rcout.push()` is always first. Then each enabled protocol is visited
/// in upstream order. The visitor is how a caller actually talks to a
/// protocol library — this leftover does not own those objects.
pub fn push<R, F>(rcout: &mut R, features: PushFeatures, can_drivers: &[CanDriver], mut on_visit: F)
where
    R: RcOutput,
    F: FnMut(PushVisit),
{
    rcout.push();
    on_visit(PushVisit::Rcout);
    let mut buf = [PushVisit::Rcout; 16];
    let n = protocol_visits(features, can_drivers, &mut buf);
    if let Some(visits) = buf.get(..n) {
        for visit in visits {
            on_visit(*visit);
        }
    }
}

/// Send a zero pulse on every channel and commit, upstream
/// `SRV_Channels::zero_rc_outputs`.
///
/// The zero is the point: a 1500 µs "neutral" cut short looks like a
/// throttle command, so every channel gets an invalid pulse instead.
/// Cork, write, then the same fan-out as [`push`].
pub fn zero_rc_outputs<R, F>(
    rcout: &mut R,
    features: PushFeatures,
    can_drivers: &[CanDriver],
    on_visit: F,
) where
    R: RcOutput,
    F: FnMut(PushVisit),
{
    cork(rcout);
    for ch in 0..NUM_SERVO_CHANNELS {
        let Ok(ch) = u8::try_from(ch) else {
            break;
        };
        let _ = rcout.write(ch, 0);
    }
    push(rcout, features, can_drivers, on_visit);
}
