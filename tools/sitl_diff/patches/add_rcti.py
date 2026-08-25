#!/usr/bin/env python3
"""Log the roll controller's inputs and outputs, reference build only.

AP_RollController cannot be driven by a linked harness the way AC_PID was: the
rate loop reaches into AP::ahrs() and AP::scheduler(), and standing those up
outside a vehicle means linking most of the firmware. ADR-0008's answer is to
record what upstream actually saw and replay it, as AP_TECS was verified.

TWO messages, not one, and the reason is a hard limit worth remembering:
ArduPilot's dynamically registered Write() messages store their format string
in a 17-byte field, so **a message can carry at most 16 fields including
TimeUS**. A 17-field message is not rejected cleanly -- the format overruns its
buffer and the vehicle panics at startup with "Unknown format specifier",
having reported a nonsense field count. That cost several rebuild-and-fly
cycles to find, because SITL still boots and only dies once the test drives it.

    RCTI  the inputs, plus the integrator BEFORE the update
    RCTO  the outputs, and the rescaled PID contributions

Both are WriteCritical. At loop rate WriteStreaming dropped four of every five
records, leaving 100ms gaps in a 20ms loop. WriteCritical records every call.

RCTI carries the integrator entering the call, which was originally expected to
make each record independently replayable. It does not. The controller also
carries the PID's target, error and derivative low-pass state, none of which is
logged, so a fresh controller per record takes the reset path and the D term
comes out zero every time. The replay has to run the steps in order, exactly as
the TECS replay does, and the logged integrator serves as the check that the
unlogged state evolved identically rather than as a seed for it.

The call is guarded on AP_Logger::get_singleton(): the roll controller runs
during early init, before AP_Logger is constructed, and AP::logger()
dereferences the singleton unguarded.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/APM_Control/AP_RollController.cpp"
)

IN_LABELS = "TimeUS,ae,sc,di,gm,gy,as,e2t,dt,ig"
IN_FMT = "QffBBfffff"
OUT_LABELS = "TimeUS,out,tgt,act,P,I,D,F,DF"
OUT_FMT = "Qffffffff"

ANCHOR = """    return _get_rate_out(desired_rate, scaler, disable_integrator, get_airspeed(), ground_mode);
}"""

PATCH = """    // ---- reference-build-only logging ----
    const float rcti_aspeed = get_airspeed();
    const float rcti_ig = rate_pid.get_i();
    const float rcti_gy = get_measured_rate();
    const float rcti_e2t = AP::ahrs().get_EAS2TAS();
    const float rcti_dt = AP::scheduler().get_loop_period_s();

    const float rcti_out = _get_rate_out(desired_rate, scaler, disable_integrator, rcti_aspeed, ground_mode);

    // Two messages because a dynamic Write() format string lives in a 17-byte
    // field: at most 16 fields including TimeUS. Overrunning it panics the
    // vehicle at startup rather than failing cleanly.
    // Guarded because this runs during early init, before AP_Logger exists.
    AP_Logger *rcti_logger = AP_Logger::get_singleton();
    if (rcti_logger != nullptr) {
        rcti_logger->WriteCritical(
            "RCTI", "%s", "%s",
            AP_HAL::micros64(),
            (float)angle_err,
            (float)scaler,
            (uint8_t)disable_integrator,
            (uint8_t)ground_mode,
            (float)rcti_gy,
            (float)rcti_aspeed,
            (float)rcti_e2t,
            (float)rcti_dt,
            (float)rcti_ig);
        rcti_logger->WriteCritical(
            "RCTO", "%s", "%s",
            AP_HAL::micros64(),
            (float)rcti_out,
            (float)_pid_info.target,
            (float)_pid_info.actual,
            (float)_pid_info.P,
            (float)_pid_info.I,
            (float)_pid_info.D,
            (float)_pid_info.FF,
            (float)_pid_info.DFF);
    }
    // ---- end reference-build-only logging ----

    return rcti_out;
}""" % (IN_LABELS, IN_FMT, OUT_LABELS, OUT_FMT)


def check(labels, fmt, name):
    n = len(labels.split(","))
    if len(fmt) > 16:
        sys.exit("%s: %d fields -- a Write() format field holds at most 16"
                 % (name, len(fmt)))
    if len(labels) > 64:
        sys.exit("%s: label string is %d chars, limit 64" % (name, len(labels)))
    if n != len(fmt):
        sys.exit("%s: %d labels vs %d type chars" % (name, n, len(fmt)))
    print("  %s: %d fields, %d chars of labels" % (name, n, len(labels)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("RCTI/RCTO not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted RCTI/RCTO")
        return

    if PATCH in text:
        print("RCTI/RCTO already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    check(IN_LABELS, IN_FMT, "RCTI")
    check(OUT_LABELS, OUT_FMT, "RCTO")

    text = text.replace(ANCHOR, PATCH, 1)
    for inc in ("AP_Logger/AP_Logger.h", "AP_Scheduler/AP_Scheduler.h"):
        if inc not in text:
            marker = '#include "AP_RollController.h"\n'
            assert marker in text, "include anchor not found"
            text = text.replace(marker, marker + "#include <%s>\n" % inc, 1)

    TARGET.write_text(text)
    print("applied RCTI/RCTO logging")


if __name__ == "__main__":
    main()
