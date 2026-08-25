"""Link a parity harness against the real ArduCopter firmware.

A harness that stubs its dependencies is not measuring upstream, it is
measuring the stubs. `SRV_Channels::set_output_scaled` doing nothing and
`get_output_channel_mask` returning zero means the whole output-channel path
gets "verified" against numbers the harness made up -- which is worse than no
test, because it reads as coverage.

Everything needed is already built. waf leaves the vehicle's objects in
`build/sitl/ArduCopter` and every library it links in
`build/sitl/lib/libArduCopter_libs.a`, all compiled with the firmware's own
flags. This assembles them into an object list a harness can link.

# Why the vehicle and not just the libraries

`RC_Channels::var_info` and the `vehicle` / `copter` globals are defined by the
vehicle, not by any library -- each vehicle subclasses `RC_Channels` and
supplies its own parameter table. Linking only the libraries leaves those
undefined, and inventing them is exactly the empty-parameter-group shortcut
that makes a fixture measure the harness.

# The one transformation

`ArduCopter/Copter.cpp` defines those globals *and* `main`, which a harness
supplies itself. `objcopy --redefine-sym` renames that single symbol. It is a
rename, not a stub: every other symbol, parameter table and code path in the
object stays the firmware's own, and nothing about its behaviour changes.
"""
import glob
import os
import subprocess
from pathlib import Path

UPSTREAM = Path("/srv/ardumaster/upstream/plane-4.7.0")
BUILD = UPSTREAM / "build/sitl"
VEHICLE_DIR = BUILD / "ArduCopter"
ARCHIVE = BUILD / "lib/libArduCopter_libs.a"

# The translation unit that carries `main`. Matched by exact basename: a
# suffix test here also catches AP_Arming_Copter.cpp, RC_Channel_Copter.cpp,
# GCS_Copter.cpp and AP_ExternalControl_Copter.cpp, and dropping those four
# costs exactly their vtables -- a link error that reads like a missing
# feature rather than a typo.
MAIN_OBJECT = "Copter.cpp.52.o"

# MultiHeap routes allocation through a wrapper, so without this
# `__real_malloc` is undefined. Only malloc is wrapped -- adding calloc and
# free leaves their `__wrap_` counterparts undefined instead, since nothing
# in the firmware defines them.
LINK_FLAGS = ("-Wl,--wrap=malloc",)


def stage_main_object(stage_dir):
    """Copy the main-bearing object with its entry point renamed."""
    stage = Path(stage_dir)
    stage.mkdir(parents=True, exist_ok=True)

    src = VEHICLE_DIR / MAIN_OBJECT
    if not src.exists():
        raise SystemExit(
            "%s missing -- build the vehicle first:\n"
            "  PATH=/srv/ardumaster/venv/bin:$PATH ./waf copter" % src
        )

    dst = stage / "Copter.cpp.nomain.o"
    subprocess.run(
        ["objcopy", "--redefine-sym", "main=copter_firmware_main",
         str(src), str(dst)],
        check=True,
    )
    return dst


def objects(stage_dir="/tmp/parity_vehicle"):
    """Every object of the real firmware, with `main` renamed out of the way.

    Returned in link order: the vehicle's objects first, the library archive
    last. Archives must come last -- a linker scans one once, taking only what
    is undefined at that point.
    """
    if not ARCHIVE.exists():
        raise SystemExit(
            "%s missing -- build the vehicle first:\n"
            "  PATH=/srv/ardumaster/venv/bin:$PATH ./waf copter" % ARCHIVE
        )

    found = sorted(glob.glob(str(VEHICLE_DIR / "*.o")))
    if not found:
        raise SystemExit("no vehicle objects under %s" % VEHICLE_DIR)

    # Exact basename, not a suffix test. See MAIN_OBJECT.
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
