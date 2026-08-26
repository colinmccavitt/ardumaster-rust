#!/usr/bin/env python3
"""FW-007 slice 3: armed-flight determinism, compared properly.

The first attempt reported 349 degrees of yaw divergence and declared armed
flight non-deterministic. Both halves of that were wrong, and both are worth
recording because they are easy mistakes to make with this data.

  * 349 degrees of yaw is not a divergence, it is a wrap. Yaw is logged in
    -180..180, so 175 and -174 differ by 349 in arithmetic and by 1 in fact.
    Angles have to be compared on the circle.

  * The runs are not aligned at the start. The early boot mode changes land
    about a second apart in SIMULATED time, because the autotest driver issues
    them on the wall clock. Later mission events land at IDENTICAL simulated
    times in both runs -- 35044310, 49044541, 67704574, 121064888 -- so the
    simulator itself is keeping step; it is the pre-flight setup that is not.
    Comparing from the first mode change the two runs agree on is the honest
    window.

Everything before that window is the autotest driving the vehicle from
outside. Everything after is the vehicle flying a mission, which is what the
golden-trajectory approach actually needs to be deterministic.
"""
import sys
from pathlib import Path

from pymavlink import DFReader

REFERENCE = Path("/srv/ardumaster/reference/autotest")

# Fields that are angles in degrees and must be compared on the circle.
ANGLE_FIELDS = {"Roll", "Pitch", "Yaw", "DesRoll", "DesPitch", "DesYaw",
                "NavRoll", "NavPitch"}

SERIES = {
    "ATT": ("Roll", "Pitch", "Yaw", "DesRoll", "DesPitch", "DesYaw"),
    "POS": ("Lat", "Lng", "Alt", "RelHomeAlt"),
    "IMU": ("GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"),
    "NKF1": ("Roll", "Pitch", "Yaw", "VN", "VE", "VD"),
    "RCOU": ("C1", "C2", "C3", "C4"),
}


def angular_delta(x, y):
    """Absolute difference between two angles in degrees, on the circle."""
    d = abs(x - y) % 360.0
    return min(d, 360.0 - d)


def mode_times(path: Path):
    log = DFReader.DFReader_binary(str(path))
    out = []
    while True:
        m = log.recv_match(type="MODE")
        if m is None:
            break
        out.append(int(m.to_dict().get("TimeUS", -1)))
    return out


def load(path: Path, msg_type: str, fields):
    log = DFReader.DFReader_binary(str(path))
    out = {}
    while True:
        m = log.recv_match(type=msg_type)
        if m is None:
            break
        d = m.to_dict()
        if "TimeUS" not in d:
            continue
        try:
            out[int(d["TimeUS"])] = tuple(float(d[f]) for f in fields)
        except (KeyError, TypeError, ValueError):
            continue
    return out


def main():
    a = REFERENCE / "run1/00000002.BIN"
    b = REFERENCE / "run2/00000002.BIN"
    for p in (a, b):
        if not p.exists():
            sys.exit("missing %s" % p)

    ta, tb = set(mode_times(a)), set(mode_times(b))
    agreed = sorted(ta & tb)
    if not agreed:
        print("the two runs share no mode-change time at all; they cannot be")
        print("aligned, and any comparison would be of unrelated flight.")
        return 2

    # The first shared mode change after the boot sequence. Boot changes are
    # all mode 0 within the first few seconds; the mission starts later.
    window_start = next((t for t in agreed if t > 10_000_000), agreed[0])
    print("aligning from the first shared mode change after boot: %d us"
          % window_start)
    print("(%d of %d/%d mode-change times are shared between the runs)"
          % (len(agreed), len(ta), len(tb)))
    print()

    worst_overall = 0.0
    worst_where = None
    compared = 0

    for msg_type, fields in SERIES.items():
        left = load(a, msg_type, fields)
        right = load(b, msg_type, fields)
        shared = sorted(t for t in (set(left) & set(right)) if t >= window_start)
        if not shared:
            print("  %-5s no shared samples in the window" % msg_type)
            continue

        worst = 0.0
        worst_field = None
        for t in shared:
            for name, x, y in zip(fields, left[t], right[t]):
                d = angular_delta(x, y) if name in ANGLE_FIELDS else abs(x - y)
                if d > worst:
                    worst, worst_field = d, name
        compared += len(shared)
        if worst > worst_overall:
            worst_overall, worst_where = worst, "%s.%s" % (msg_type, worst_field)
        print("  %-5s %6d shared samples in window   max delta %.6g%s"
              % (msg_type, len(shared), worst,
                 "" if worst == 0.0 else "  (%s)" % worst_field))

    print()
    if compared == 0:
        print("VERDICT: nothing comparable in the window. Harness failure.")
        return 2
    if worst_overall == 0.0:
        print("VERDICT: armed flight IS replay-deterministic once the runs are")
        print("aligned. %d samples compared, max absolute delta ZERO in every" % compared)
        print("field. The noise floor is zero in the armed regime, so")
        print("sitl-diff tolerances are needed only where the PORT")
        print("legitimately differs.")
        return 0

    print("VERDICT: armed flight diverges even after alignment. %d samples"
          % compared)
    print("compared, worst absolute delta %.6g at %s." % (worst_overall, worst_where))
    print("Golden-trajectory comparison needs a per-field tolerance, and that")
    print("tolerance has to be justified rather than fitted to make it pass.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
