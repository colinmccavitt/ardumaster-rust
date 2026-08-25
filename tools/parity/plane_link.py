"""Link a parity harness against the real ArduPlane firmware.

The counterpart to `vehicle_link`, which stages ArduCopter. Both vehicles are
already built in the pinned worktree, and the choice matters: whole libraries
exist in only one of them. `AP_Landing`, `AP_TECS` and `AP_L1_Control` are
Plane's; `AC_PosControl`, `AC_WPNav` and the multirotor attitude controller are
Copter's. A harness for a Plane library linked against Copter finds nothing to
call -- there is no `AP_Landing` singleton in that binary, because nothing in
it ever constructs one.

Everything else follows `vehicle_link` exactly, including the one
transformation: `ArduPlane/Plane.cpp` defines the vehicle globals *and*
`main`, so that single symbol is renamed and every other symbol, parameter
table and code path in the object stays the firmware's own.
"""
import glob
import os
import subprocess
from pathlib import Path

UPSTREAM = Path("/srv/ardumaster/upstream/plane-4.7.0")
BUILD = UPSTREAM / "build/sitl"
VEHICLE_DIR = BUILD / "ArduPlane"
ARCHIVE = BUILD / "lib/libArduPlane_libs.a"

# The translation unit carrying `main`, matched by exact basename for the same
# reason as in `vehicle_link`: a suffix test would also catch
# AP_Arming_Plane.cpp, RC_Channel_Plane.cpp and friends, and dropping those
# costs their vtables -- a link error that reads like a missing feature.
MAIN_OBJECT = "Plane.cpp.53.o"

LINK_FLAGS = ("-Wl,--wrap=malloc",)


def stage_main_object(stage_dir):
    """Copy the main-bearing object with its entry point renamed."""
    stage = Path(stage_dir)
    stage.mkdir(parents=True, exist_ok=True)

    src = VEHICLE_DIR / MAIN_OBJECT
    if not src.exists():
        raise SystemExit(
            "%s missing -- build the vehicle first:\n"
            "  PATH=/srv/ardumaster/venv/bin:$PATH ./waf plane" % src
        )

    dst = stage / "Plane.cpp.nomain.o"
    subprocess.run(
        ["objcopy", "--redefine-sym", "main=plane_firmware_main",
         str(src), str(dst)],
        check=True,
    )
    return dst


def objects(stage_dir="/tmp/parity_plane"):
    """Every object of the real firmware, with `main` renamed out of the way.

    Vehicle objects first, the library archive last: a linker scans an archive
    once, taking only what is undefined at that point.
    """
    if not ARCHIVE.exists():
        raise SystemExit(
            "%s missing -- build the vehicle first:\n"
            "  PATH=/srv/ardumaster/venv/bin:$PATH ./waf plane" % ARCHIVE
        )

    found = sorted(glob.glob(str(VEHICLE_DIR / "*.o")))
    if not found:
        raise SystemExit("no vehicle objects under %s" % VEHICLE_DIR)

    kept = [o for o in found if os.path.basename(o) != MAIN_OBJECT]
    if len(kept) != len(found) - 1:
        raise SystemExit(
            "expected to drop exactly one object, dropped %d"
            % (len(found) - len(kept))
        )

    return kept + [str(stage_main_object(stage_dir)), str(ARCHIVE)]


if __name__ == "__main__":
    objs = objects()
    print("%d objects (%d vehicle, 1 staged, 1 archive)"
          % (len(objs), len(objs) - 2))
