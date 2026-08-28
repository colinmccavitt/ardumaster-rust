//! RC_Channel PWM scale + deadzone hookup for the vehicle loop.
//!
//! Upstream `Plane::read_radio` feeds `channel_roll->norm_input_dz()` (and
//! pitch/rudder) after the HAL has delivered PWM microseconds. This hookup
//! is the vehicle-side call into [`ap_rc`].

use ap_rc::{norm_input_dz, RcChannel};

use crate::stabilize_hookup::RcStickInputs;

/// Frontend RC PWM-scale hookup for the vehicle loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannelScaleHookup {
    /// Roll stick, upstream `channel_roll`.
    pub roll: RcChannel,
    /// Pitch stick, upstream `channel_pitch`.
    pub pitch: RcChannel,
    /// Yaw/rudder stick, upstream `channel_rudder`.
    pub yaw: RcChannel,
}

impl Default for RcChannelScaleHookup {
    fn default() -> Self {
        Self {
            roll: RcChannel::default(),
            pitch: RcChannel::default(),
            yaw: RcChannel::default(),
        }
    }
}

impl RcChannelScaleHookup {
    /// Build a hookup from per-axis channel calibration.
    #[must_use]
    pub const fn from_channels(roll: RcChannel, pitch: RcChannel, yaw: RcChannel) -> Self {
        Self { roll, pitch, yaw }
    }

    /// Scale PWM microseconds to signed sticks with deadzone.
    #[must_use]
    pub fn publish(&self, roll_pwm: u16, pitch_pwm: u16, yaw_pwm: u16) -> RcStickInputs {
        scale_rc_sticks(&self.roll, &self.pitch, &self.yaw, roll_pwm, pitch_pwm, yaw_pwm)
    }
}

/// Map three stick PWMs through [`norm_input_dz`].
#[must_use]
pub fn scale_rc_sticks(
    roll: &RcChannel,
    pitch: &RcChannel,
    yaw: &RcChannel,
    roll_pwm: u16,
    pitch_pwm: u16,
    yaw_pwm: u16,
) -> RcStickInputs {
    RcStickInputs {
        roll_norm_dz: norm_input_dz(roll_pwm, roll),
        pitch_norm_dz: norm_input_dz(pitch_pwm, pitch),
        yaw_norm_dz: norm_input_dz(yaw_pwm, yaw),
    }
}
