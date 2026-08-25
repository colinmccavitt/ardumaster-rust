#!/usr/bin/env python3
"""Log the steering controller's inputs and outputs, reference build only.

One message, not two: the steering controller needs 14 fields including
TimeUS, which fits under the 16-field ceiling a dynamic Write() message has
(its format string lives in a 17-byte field). Roll and pitch needed splitting;
this does not.

The logging goes in get_steering_out_rate, which is the single point all three
entry points -- rate, lateral acceleration, and angle error -- funnel through.

`ms` is logged explicitly rather than derived from TimeUS. This controller
measures its own loop period from AP_HAL::millis(), and that period multiplies
the integrator directly, so the replay must be driven by the same millisecond
count upstream saw rather than by micros/1000.

`ig` is the integrator ENTERING the call. Note that for this controller the
integrator and the reported I are the same field, so it is read before any of
the update touches it.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/APM_Control/AP_SteerController.cpp"
)

LABELS = "TimeUS,ms,dr,gs,yr,rv,ig,out,tgt,act,P,I,D,F"
FMT = "QIfffBfiffffff"

ANCHOR = """    // Convert to centi-degrees and constrain
    return constrain_float(_last_out * 100, -derate_constraint, derate_constraint);
}"""

PATCH = """    // Convert to centi-degrees and constrain
    const int32_t stci_out = constrain_float(_last_out * 100, -derate_constraint, derate_constraint);

    // ---- reference-build-only logging ----
    AP_Logger *stci_logger = AP_Logger::get_singleton();
    if (stci_logger != nullptr) {
        stci_logger->WriteCritical(
            "STCI", "%s", "%s",
            AP_HAL::micros64(),
            (uint32_t)tnow,
            (float)desired_rate,
            (float)_ahrs.groundspeed(),
            (float)_ahrs.get_yaw_rate_earth(),
            (uint8_t)_reverse,
            (float)stci_ig,
            stci_out,
            (float)_pid_info.target,
            (float)_pid_info.actual,
            (float)_pid_info.P,
            (float)_pid_info.I,
            (float)_pid_info.D,
            (float)_pid_info.FF);
    }
    // ---- end reference-build-only logging ----

    return stci_out;
}""" % (LABELS, FMT)

# the integrator has to be captured before the update touches it
ENTRY_ANCHOR = """	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;"""
ENTRY_PATCH = """	uint32_t tnow = AP_HAL::millis();
	const float stci_ig = _pid_info.I;   // reference-build-only logging
	uint32_t dt = tnow - _last_t;"""


def check():
    n = len(LABELS.split(","))
    if len(FMT) > 16:
        sys.exit("STCI: %d fields -- a Write() format field holds at most 16" % len(FMT))
    if len(LABELS) > 64:
        sys.exit("STCI: label string is %d chars, limit 64" % len(LABELS))
    if n != len(FMT):
        sys.exit("STCI: %d labels vs %d type chars" % (n, len(FMT)))
    print("  STCI: %d fields, %d chars of labels" % (n, len(LABELS)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("STCI not applied")
            return
        text = text.replace(PATCH, ANCHOR).replace(ENTRY_PATCH, ENTRY_ANCHOR)
        TARGET.write_text(text)
        print("reverted STCI")
        return

    if PATCH in text:
        print("STCI already applied")
        return
    for name, a in (("body", ANCHOR), ("entry", ENTRY_ANCHOR)):
        if text.count(a) != 1:
            sys.exit("%s anchor matched %d times, expected 1" % (name, text.count(a)))

    check()
    text = text.replace(ANCHOR, PATCH, 1).replace(ENTRY_ANCHOR, ENTRY_PATCH, 1)
    marker = '#include "AP_SteerController.h"\n'
    if marker not in text:
        sys.exit("include anchor not found")
    if "AP_Logger/AP_Logger.h" not in text:
        text = text.replace(marker, marker + "#include <AP_Logger/AP_Logger.h>\n", 1)

    TARGET.write_text(text)
    print("applied STCI logging")


if __name__ == "__main__":
    main()
