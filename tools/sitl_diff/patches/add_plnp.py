#!/usr/bin/env python3
r"""Log the TECS-to-pitch-controller join (FW-025, longitudinal slice).

The pitch join is richer than the roll one. Roll is limit-then-subtract; pitch
adds a trim offset, a feed-forward from throttle, and a flare override that can
replace the demand outright:

    nav_pitch_cd   = constrain(TECS pitch demand, pitch_limit_min, pitch_limit_max)
    demanded_pitch = nav_pitch_cd + PTCH_TRIM_DEG*100 + throttle * KFF_THR2PTCH
    angle_err      = demanded_pitch - ahrs.pitch_sensor

PLNP records every term, so a divergence can be attributed to one of them
rather than just observed at the elevator. `dem` is the demand AFTER any flare
override, so comparing the port's reconstruction against it also says whether
the override fired.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Attitude.cpp")

LABELS = "TimeUS,tpd,pmin,pmax,nav,trm,thr,kff,dem,ps,ae"
FMT = "Qiiiiiffiii"

ANCHOR = """    return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));"""

PATCH = """    // ---- reference-build-only logging: the TECS-to-pitch join (FW-025) ----
    {
        AP_Logger *plnp_logger = AP_Logger::get_singleton();
        if (plnp_logger != nullptr) {
            plnp_logger->WriteCritical(
                "PLNP", "%s", "%s",
                AP_HAL::micros64(),
                (int32_t)TECS_controller.get_pitch_demand(),
                (int32_t)(pitch_limit_min*100),
                (int32_t)(aparm.pitch_limit_max.get()*100),
                (int32_t)nav_pitch_cd,
                (int32_t)(g.pitch_trim * 100.0),
                (float)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
                (float)g.kff_throttle_to_pitch,
                (int32_t)demanded_pitch,
                (int32_t)ahrs.pitch_sensor,
                (int32_t)(demanded_pitch - ahrs.pitch_sensor));
        }
    }
    // ---- end reference-build-only logging ----

    return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));""" % (LABELS, FMT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("PLNP not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted PLNP")
        return

    if PATCH in text:
        print("PLNP already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    n = len(LABELS.split(","))
    if n != len(FMT):
        sys.exit("PLNP: %d labels vs %d type chars" % (n, len(FMT)))
    if len(LABELS) > 64:
        sys.exit("PLNP: label string is %d chars" % len(LABELS))

    text = text.replace(ANCHOR, PATCH, 1)
    if "AP_Logger/AP_Logger.h" not in text:
        marker = '#include "Plane.h"\n'
        if marker not in text:
            sys.exit("include anchor not found")
        text = text.replace(marker, marker + "#include <AP_Logger/AP_Logger.h>\n", 1)

    TARGET.write_text(text)
    print("applied PLNP logging (%d fields)" % n)


if __name__ == "__main__":
    main()
