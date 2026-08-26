#!/usr/bin/env python3
"""When does the divergence start, relative to the first mistimed input?

This decides something expensive. If the two runs are identical up to the
first moment the test driver issues a command at a different simulated time,
and diverge only after it, then the simulator is deterministic and the
irreproducibility is entirely in the inputs -- which means a simulated-time
mission driver would fix it and is worth building.

If instead they diverge before any input differs, the simulator itself is not
reproducible and no amount of input discipline will help.

The data to answer it already exists; no new runs are needed.
"""
import sys
from pathlib import Path

from pymavlink import DFReader

REFERENCE = Path("/srv/ardumaster/reference/autotest")
ANGLE_FIELDS = {"Roll", "Pitch", "Yaw"}


def angular_delta(x, y):
    d = abs(x - y) % 360.0
    return min(d, 360.0 - d)


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


def mode_times(path: Path):
    log = DFReader.DFReader_binary(str(path))
    out = []
    while True:
        m = log.recv_match(type="MODE")
        if m is None:
            break
        d = m.to_dict()
        out.append((int(d.get("TimeUS", -1)), d.get("ModeNum")))
    return out


def main():
    a = REFERENCE / "run1/00000002.BIN"
    b = REFERENCE / "run2/00000002.BIN"

    ma, mb = mode_times(a), mode_times(b)
    first_input_difference = None
    for i in range(min(len(ma), len(mb))):
        if ma[i][0] != mb[i][0]:
            first_input_difference = (i, ma[i][0], mb[i][0])
            break

    fields = ("Roll", "Pitch", "Yaw")
    left = load(a, "ATT", fields)
    right = load(b, "ATT", fields)
    shared = sorted(set(left) & set(right))

    print("%d shared ATT samples" % len(shared))
    if first_input_difference:
        i, t1, t2 = first_input_difference
        print("first input timing difference: mode change #%d at %d us vs %d us"
              % (i, t1, t2))
        print("  (the earlier of the two is %d us)" % min(t1, t2))
    else:
        print("the driver issued every mode change at identical simulated times")
    print()

    # Walk forward and find the first sample where the two runs disagree at
    # all, and then the first where they disagree by something a pilot would
    # notice.
    first_any = None
    first_material = None
    for t in shared:
        worst = max(
            angular_delta(x, y) if n in ANGLE_FIELDS else abs(x - y)
            for n, x, y in zip(fields, left[t], right[t])
        )
        if worst > 0.0 and first_any is None:
            first_any = (t, worst)
        if worst > 1.0 and first_material is None:
            first_material = (t, worst)
            break

    if first_any is None:
        print("the two runs never disagree on attitude at all.")
        return 0

    print("first attitude disagreement of any size: %d us (delta %.6g deg)"
          % first_any)
    if first_material:
        print("first disagreement above 1 degree:      %d us (delta %.6g deg)"
              % first_material)
    else:
        print("never disagree by more than 1 degree.")
    print()

    if first_input_difference:
        earliest_input = min(first_input_difference[1], first_input_difference[2])
        if first_any[0] >= earliest_input:
            print("VERDICT: the runs are identical until the driver's first")
            print("mistimed command, and diverge only after it. The simulator")
            print("is keeping step; the inputs are not. A simulated-time")
            print("mission driver would fix this, and is worth building.")
            return 0
        print("VERDICT: the runs diverge at %d us, BEFORE the first mistimed"
              % first_any[0])
        print("command at %d us. The simulator itself is not reproducible,"
              % earliest_input)
        print("and input discipline alone will not fix it.")
        return 1

    print("VERDICT: inputs were identically timed, yet the runs diverge.")
    print("The simulator itself is not reproducible.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
