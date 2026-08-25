#!/usr/bin/env python3
"""Extend the reference-build logging with the 50 Hz-stage state (ADR-0008).

TECI captures the nine update_pitch_throttle arguments. The ported control
logic also consumes values produced by update_50hz, which upstream logs only
partially:

    logged already : _height, _climb_rate, _TAS_state, _vel_dot
                     (TECS.h, .dh, .sp, .dsp)
    NOT logged      : _vel_dot_lpf, _TAS_dem, _TASmin, _TASmax
                     and the AHRS values cos_roll / pitch

_vel_dot_lpf matters because the kinetic energy rate is high-passed as
`TAS_state * (vel_dot - vel_dot_lpf)`. Without it that term cannot be
reproduced.

Added as a second message TECJ rather than extending TECI, because TECI's
label string is already 49 of the 64 characters ArduPilot allows.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp")

ANCHOR = """        (float)pitch_trim_deg);
    // ---- end reference-build-only logging ----"""

PATCH = """        (float)pitch_trim_deg);
    // Second message: state produced by update_50hz that the control logic
    // consumes. _vel_dot_lpf in particular is needed because the kinetic
    // energy rate high-passes vel_dot against it, and upstream logs neither
    // that nor the airspeed demand/limits. 8 labels, 8 type chars, 46 chars
    // of labels - inside the 64-char limit.
    AP::logger().WriteStreaming(
        "TECJ", "TimeUS,vdlpf,tasd,tasmn,tasmx,tascr,cosr,ptchm",
        "Qfffffff",
        now,
        (float)_vel_dot_lpf,
        (float)_TAS_dem,
        (float)_TASmin,
        (float)_TASmax,
        (float)(aparm.airspeed_cruise * _ahrs.get_EAS2TAS()),
        (float)_ahrs.cos_roll(),
        (float)_ahrs.get_pitch_rad());
    // ---- end reference-build-only logging ----"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("TECJ not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted TECJ")
        return

    if PATCH in text:
        print("TECJ already applied")
        return
    if ANCHOR not in text:
        sys.exit("anchor not found - is the TECI patch applied?")

    TARGET.write_text(text.replace(ANCHOR, PATCH, 1))
    print("applied TECJ logging to reference build")


if __name__ == "__main__":
    main()
