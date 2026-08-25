#!/usr/bin/env python3
"""Log the yaw sideslip damper's inputs and outputs, reference build only.

Thirteen fields including TimeUS, so one message rather than the two roll and
pitch needed.

`ms` is logged explicitly rather than derived from TimeUS: the damper measures
its own loop period from AP_HAL::millis(), and that period multiplies the
integrator directly.

`as` is the RESOLVED airspeed -- the value after the fallback to the midpoint
of the configured range -- so the replay does not have to guess which branch
upstream took.

`ig` is the integrator entering the call. It is the state, not the reported
value: this controller keeps them deliberately separate.

Note the damper is inert unless YAW2SRV_DAMP is non-zero, which it is not by
default. See enable_yaw_damper.py.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/APM_Control/AP_YawController.cpp"
)

LABELS = "TimeUS,ms,sc,di,rr,as,yr,ay,ab,ig,out,I,D"
FMT = "QIfBffffffiff"

ANCHOR = """    // Convert to centi-degrees and constrain
    return constrain_float(_last_out * 100, -4500, 4500);
}"""

PATCH = """    // Convert to centi-degrees and constrain
    const int32_t ycti_out = constrain_float(_last_out * 100, -4500, 4500);

    // ---- reference-build-only logging ----
    AP_Logger *ycti_logger = AP_Logger::get_singleton();
    if (ycti_logger != nullptr) {
        ycti_logger->WriteCritical(
            "YCTI", "%s", "%s",
            AP_HAL::micros64(),
            (uint32_t)tnow,
            (float)scaler,
            (uint8_t)disable_integrator,
            (float)AP::ahrs().get_roll_rad(),
            (float)aspeed,
            (float)omega_z,
            (float)AP::ins().get_accel().y,
            (float)abias.y,
            (float)ycti_ig,
            ycti_out,
            (float)_pid_info.I,
            (float)_pid_info.D);
    }
    // ---- end reference-build-only logging ----

    return ycti_out;
}""" % (LABELS, FMT)

ENTRY_ANCHOR = """    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;"""
ENTRY_PATCH = """    uint32_t tnow = AP_HAL::millis();
    const float ycti_ig = _integrator;   // reference-build-only logging
    uint32_t dt = tnow - _last_t;"""


def check():
    n = len(LABELS.split(","))
    if len(FMT) > 16:
        sys.exit("YCTI: %d fields -- a Write() format field holds at most 16" % len(FMT))
    if len(LABELS) > 64:
        sys.exit("YCTI: label string is %d chars, limit 64" % len(LABELS))
    if n != len(FMT):
        sys.exit("YCTI: %d labels vs %d type chars" % (n, len(FMT)))
    print("  YCTI: %d fields, %d chars of labels" % (n, len(LABELS)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("YCTI not applied")
            return
        text = text.replace(PATCH, ANCHOR).replace(ENTRY_PATCH, ENTRY_ANCHOR)
        TARGET.write_text(text)
        print("reverted YCTI")
        return

    if PATCH in text:
        print("YCTI already applied")
        return
    for name, a in (("body", ANCHOR), ("entry", ENTRY_ANCHOR)):
        if text.count(a) != 1:
            sys.exit("%s anchor matched %d times, expected 1" % (name, text.count(a)))

    check()
    text = text.replace(ANCHOR, PATCH, 1).replace(ENTRY_ANCHOR, ENTRY_PATCH, 1)
    marker = '#include "AP_YawController.h"\n'
    if marker not in text:
        sys.exit("include anchor not found")
    if "AP_Logger/AP_Logger.h" not in text:
        text = text.replace(marker, marker + "#include <AP_Logger/AP_Logger.h>\n", 1)

    TARGET.write_text(text)
    print("applied YCTI logging")


if __name__ == "__main__":
    main()
