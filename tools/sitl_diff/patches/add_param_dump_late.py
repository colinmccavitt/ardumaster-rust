#!/usr/bin/env python3
r"""Take the parameter dump after the vehicle has finished initialising.

Dumping from load_all() was too early in two ways that both showed up as
missing data:

  * AP_SUBGROUPVARPTR groups reach their table through a pointer that is still
    null at that point, so the sixteen MAV channel subgroups were dumped with
    no children at all -- and the eight MAVLink stream-rate parameters a real
    vehicle had saved could then not be named.
  * set_frame_type_flags() has not run, so the frame mask is zero and every
    frame-tagged parameter is filtered out.

This adds a hook to Plane::one_second_loop that, once the vehicle has been up
long enough, sets AP_PARAM_DUMP_NOW and calls AP_Param::load_all(). The dump
lives at the top of load_all() and exits the process, so nothing of load_all's
normal work runs -- calling it a second time is safe precisely because it never
gets past the first check.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Plane.cpp")

ANCHOR = """void Plane::one_second_loop()
{"""

PATCH = """void Plane::one_second_loop()
{
    // ---- reference-build-only: FW-004 parameter table dump ----
    // Deferred until the vehicle is up, so that pointer-reached group tables
    // are populated and the frame mask has been set. The dump itself is at the
    // top of AP_Param::load_all() and exits, so this never reaches load_all's
    // real work.
    if (getenv("AP_PARAM_DUMP") != nullptr && AP_HAL::millis() > 15000) {
        setenv("AP_PARAM_DUMP_NOW", "1", 1);
        AP_Param::load_all();
    }
    // ---- end reference-build-only ----
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("late param dump not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted the late param dump")
        return

    if PATCH in text:
        print("late param dump already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    text = text.replace(ANCHOR, PATCH, 1)
    if "#include <stdlib.h>" not in text:
        marker = '#include "Plane.h"\n'
        if marker not in text:
            sys.exit("include anchor not found")
        text = text.replace(marker, marker + "#include <stdlib.h>\n", 1)

    TARGET.write_text(text)
    print("applied the deferred parameter dump hook")


if __name__ == "__main__":
    main()
