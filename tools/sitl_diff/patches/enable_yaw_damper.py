#!/usr/bin/env python3
"""Turn the yaw sideslip damper on for the reference flight.

Note the file: the plane tests load default_params/plane-jsbsim.parm, NOT
models/plane.parm. The first attempt patched the latter and the parameters
never reached the vehicle. The SITL plane model leaves YAW2SRV_DAMP at zero, and AP_YawController returns
0 immediately when the damping gain is below 0.0001 -- so with stock parameters
the whole controller is dead code and a replay would verify nothing.

These are ArduPilot's own recommended starting values for a conventional plane,
not numbers picked to make the port look good: the flight still has to pass
upstream's autotest with them.

SLIP is non-zero so the lateral-acceleration path is exercised too; with it at
zero the integrator would see only the high-passed rate.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/Tools/autotest/default_params/plane-jsbsim.parm"
)

BLOCK = """
# ---- reference-build-only: exercise AP_YawController for FW-017 replay ----
YAW2SRV_DAMP 0.5
YAW2SRV_INT 0.15
YAW2SRV_SLIP 0.5
# ---- end reference-build-only ----
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found: %s" % TARGET)
    text = TARGET.read_text()

    if args.revert:
        if BLOCK not in text:
            print("yaw damper overlay not applied")
            return
        TARGET.write_text(text.replace(BLOCK, ""))
        print("reverted yaw damper overlay")
        return

    if BLOCK in text:
        print("yaw damper overlay already applied")
        return

    TARGET.write_text(text.rstrip("\n") + "\n" + BLOCK)
    print("enabled the yaw damper (DAMP 0.5, INT 0.15, SLIP 0.5)")


if __name__ == "__main__":
    main()
