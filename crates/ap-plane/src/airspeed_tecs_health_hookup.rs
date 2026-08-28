//! Airspeed healthy-for-TECS publish gate, upstream `AP_Airspeed::use()`.
//!
//! TECS only consumes pitot TAS when the primary instance is healthy and
//! `ARSPD_USE` is enabled. Unhealthy sensors may still sample but do not
//! publish TAS onto the TECS path.

use ap_airspeed::sitl::{tas_for_tecs, use_airspeed_for_tecs};

/// TAS and use flag published to TECS after the health gate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AirspeedTecsHealthPublish {
    pub healthy: bool,
    pub use_for_control: bool,
    pub use_for_tecs: bool,
    pub tas_for_tecs: f32,
}

/// Gate TAS for TECS: require healthy() and ARSPD_USE.
#[must_use]
pub fn publish_airspeed_for_tecs(
    tas_mps: f32,
    healthy: bool,
    use_for_control: bool,
) -> AirspeedTecsHealthPublish {
    AirspeedTecsHealthPublish {
        healthy,
        use_for_control,
        use_for_tecs: use_airspeed_for_tecs(healthy, use_for_control),
        tas_for_tecs: tas_for_tecs(tas_mps, healthy, use_for_control),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unhealthy_or_unused_publishes_zero_tas() {
        let out = publish_airspeed_for_tecs(18.0, false, true);
        assert!(!out.use_for_tecs);
        assert_eq!(out.tas_for_tecs, 0.0);

        let out = publish_airspeed_for_tecs(18.0, true, false);
        assert!(!out.use_for_tecs);
        assert_eq!(out.tas_for_tecs, 0.0);

        let out = publish_airspeed_for_tecs(18.0, true, true);
        assert!(out.use_for_tecs);
        assert!((out.tas_for_tecs - 18.0).abs() < 1e-6);
    }
}
