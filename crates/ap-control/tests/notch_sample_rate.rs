//! `AC_AttitudeControl_Multi::set_notch_sample_rate` leftover (COP-008).

#![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

use ap_control::rate_loop::RateLoop;
use ap_pid::{AcPid, Filters, NotchFilterParams, PidGains};

fn pid() -> AcPid {
    AcPid::new(PidGains::default())
}

fn valid() -> NotchFilterParams {
    NotchFilterParams {
        center_freq_hz: 80.0,
        quality: 2.0,
        attenuation_db: 30.0,
    }
}

/// Stock Multi PIDs have NTF=NEF=0. The leftover forwards and each PID
/// returns immediately.
#[test]
fn stock_indices_leave_every_axis_untouched() {
    let mut rates = RateLoop::new(pid(), pid(), pid());
    let mut filters = Filters::new();
    filters.set(1, valid());
    rates.set_notch_sample_rate(400.0, &filters);
    assert!(rates.roll.target_notch().is_none());
    assert!(rates.pitch.target_notch().is_none());
    assert!(rates.yaw.target_notch().is_none());
}

/// The leftover is the three PID calls, not a shared filter. Roll and pitch
/// can take different indices; a mix-up would configure the wrong axis.
#[test]
fn each_axis_looks_up_its_own_index() {
    let mut roll = pid();
    roll.notch_t_filter = 1;
    let mut pitch = pid();
    pitch.notch_t_filter = 2;
    let mut yaw = pid();
    yaw.notch_e_filter = 1;

    let mut filters = Filters::new();
    filters.set(
        1,
        NotchFilterParams {
            center_freq_hz: 80.0,
            ..valid()
        },
    );
    filters.set(
        2,
        NotchFilterParams {
            center_freq_hz: 120.0,
            ..valid()
        },
    );

    let mut rates = RateLoop::new(roll, pitch, yaw);
    rates.set_notch_sample_rate(400.0, &filters);

    assert_eq!(
        rates.roll.target_notch().expect("roll T").center_freq(),
        80.0
    );
    assert_eq!(
        rates.pitch.target_notch().expect("pitch T").center_freq(),
        120.0
    );
    assert!(rates.yaw.target_notch().is_none());
    assert_eq!(rates.yaw.error_notch().expect("yaw E").center_freq(), 80.0);
    assert_eq!(
        rates.roll.target_notch().expect("roll T").sample_freq(),
        400.0
    );
}
