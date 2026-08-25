#!/usr/bin/env python3
"""Log the pitch controller's inputs and outputs, reference build only.

The roll controller's counterpart is add_rcti.py, and the same two constraints
apply. A dynamically registered Write() message stores its format string in a
17-byte field, so a message carries at most 16 fields including TimeUS -- an
over-long one is not rejected cleanly, it panics the vehicle at startup. And
WriteCritical rather than WriteStreaming, because at loop rate WriteStreaming
dropped four of every five records.

    PCTI  the inputs, plus the integrator entering the update
    PCTO  the outputs, and the rescaled PID contributions

Pitch needs more inputs than roll because the demanded rate depends on the
current attitude twice over: the turn-coordination offset reads bank and pitch
angle, and the roll-limit taper reads the roll and pitch sensors.

The remaining inputs -- PTCH_RLL_FF, ROLL_LIMIT_DEG, AIRSPEED_MIN/MAX -- are
parameters, and come from the log's own PARM records.

Note that a record is NOT independently replayable, despite carrying the
integrator: the controller also holds unlogged low-pass state. See add_rcti.py.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/APM_Control/AP_PitchController.cpp"
)

IN_LABELS = "TimeUS,ae,sc,di,gm,gy,as,e2t,dt,ig,rr,pr,rs,ps"
IN_FMT = "QffBBfffffffii"
OUT_LABELS = "TimeUS,out,tgt,act,P,I,D,F,DF"
OUT_FMT = "Qffffffff"

ANCHOR = """    return _get_rate_out(desired_rate, scaler, disable_integrator, aspeed, ground_mode);
}"""

PATCH = """    // ---- reference-build-only logging ----
    const float pcti_ig = rate_pid.get_i();
    const float pcti_gy = get_measured_rate();
    const float pcti_e2t = AP::ahrs().get_EAS2TAS();
    const float pcti_dt = AP::scheduler().get_loop_period_s();
    const float pcti_rr = AP::ahrs().get_roll_rad();
    const float pcti_pr = AP::ahrs().get_pitch_rad();
    const int32_t pcti_rs = AP::ahrs().roll_sensor;
    const int32_t pcti_ps = AP::ahrs().pitch_sensor;

    const float pcti_out = _get_rate_out(desired_rate, scaler, disable_integrator, aspeed, ground_mode);

    // Two messages: a dynamic Write() format string lives in a 17-byte field,
    // so at most 16 fields including TimeUS. Guarded because the controller
    // runs during early init, before AP_Logger is constructed.
    AP_Logger *pcti_logger = AP_Logger::get_singleton();
    if (pcti_logger != nullptr) {
        pcti_logger->WriteCritical(
            "PCTI", "%s", "%s",
            AP_HAL::micros64(),
            (float)angle_err,
            (float)scaler,
            (uint8_t)disable_integrator,
            (uint8_t)ground_mode,
            (float)pcti_gy,
            (float)aspeed,
            (float)pcti_e2t,
            (float)pcti_dt,
            (float)pcti_ig,
            (float)pcti_rr,
            (float)pcti_pr,
            pcti_rs,
            pcti_ps);
        pcti_logger->WriteCritical(
            "PCTO", "%s", "%s",
            AP_HAL::micros64(),
            (float)pcti_out,
            (float)_pid_info.target,
            (float)_pid_info.actual,
            (float)_pid_info.P,
            (float)_pid_info.I,
            (float)_pid_info.D,
            (float)_pid_info.FF,
            (float)_pid_info.DFF);
    }
    // ---- end reference-build-only logging ----

    return pcti_out;
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
            print("PCTI/PCTO not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted PCTI/PCTO")
        return

    if PATCH in text:
        print("PCTI/PCTO already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    check(IN_LABELS, IN_FMT, "PCTI")
    check(OUT_LABELS, OUT_FMT, "PCTO")

    text = text.replace(ANCHOR, PATCH, 1)
    marker = '#include "AP_PitchController.h"\n'
    if marker not in text:
        sys.exit("include anchor not found")
    for inc in ("AP_Logger/AP_Logger.h", "AP_Scheduler/AP_Scheduler.h"):
        if inc not in text:
            text = text.replace(marker, marker + "#include <%s>\n" % inc, 1)

    TARGET.write_text(text)
    print("applied PCTI/PCTO logging")


if __name__ == "__main__":
    main()
