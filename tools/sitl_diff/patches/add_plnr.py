#!/usr/bin/env python3
r"""Log the vehicle glue between L1 and the roll controller (FW-025 slice).

The whole point of the vertical slice is to test the JOIN, so the join's own
inputs and outputs have to be observable:

    PLNR  nrc  what nav_controller->nav_roll_cd() returned
          rlc  the roll limit in force at that moment
          nav  plane.nav_roll_cd, after constraining to that limit
          rs   ahrs.roll_sensor
          ae   the angle error handed to the roll controller
          pt   pitch, because nav_roll_cd() recomputes from it every call

Logged from Plane::stabilize_roll rather than from either library, because
that is where the join happens and roll_limit_cd is a vehicle member no
library can see. It is also why this cannot be derived from ROLL_LIMIT_DEG:
Plane reduces the limit during takeoff and landing, so the parameter is not
the value in force.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Attitude.cpp")

LABELS = "TimeUS,nrc,rlc,nav,rs,ae,pt"
FMT = "Qiiiiif"

ANCHOR = """    return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
                                        ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));"""

PATCH = """    // ---- reference-build-only logging: the L1-to-roll join (FW-025) ----
    {
        AP_Logger *plnr_logger = AP_Logger::get_singleton();
        if (plnr_logger != nullptr) {
            plnr_logger->WriteCritical(
                "PLNR", "%s", "%s",
                AP_HAL::micros64(),
                (int32_t)nav_controller->nav_roll_cd(),
                (int32_t)roll_limit_cd,
                (int32_t)nav_roll_cd,
                (int32_t)ahrs.roll_sensor,
                (int32_t)(nav_roll_cd - ahrs.roll_sensor),
                (float)ahrs.get_pitch_rad());
        }
    }
    // ---- end reference-build-only logging ----

    return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
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
            print("PLNR not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted PLNR")
        return

    if PATCH in text:
        print("PLNR already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    n = len(LABELS.split(","))
    if n != len(FMT):
        sys.exit("PLNR: %d labels vs %d type chars" % (n, len(FMT)))
    if len(LABELS) > 64:
        sys.exit("PLNR: label string is %d chars" % len(LABELS))

    text = text.replace(ANCHOR, PATCH, 1)
    if "AP_Logger/AP_Logger.h" not in text:
        marker = '#include "Plane.h"\n'
        if marker not in text:
            sys.exit("include anchor not found")
        text = text.replace(marker, marker + "#include <AP_Logger/AP_Logger.h>\n", 1)

    TARGET.write_text(text)
    print("applied PLNR logging (%d fields)" % n)


if __name__ == "__main__":
    main()
