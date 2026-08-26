#!/usr/bin/env python3
"""FW-007: compare two SITL logs over a fixed window of SIMULATED time.

This is the oracle's comparison step, and the shape of it is the lesson from
three earlier attempts.

Bound the window by simulated time rather than trusting termination. Slice 1
warned that killing a run on the wall clock changes its sample count, and the
Lua runs then did exactly that -- the driver's disarm is refused in flight, so
the vehicle kept flying and both runs ended when a timer expired. Comparing
over a fixed simulated window makes that irrelevant: however a run ended, the
window is the same for both.

Compare angles on the circle. A first attempt reported 349 degrees of yaw
divergence, which was 175 against -174.

Report a distribution, not a maximum. A single worst-case number cannot tell
a systematic offset from one bad sample, and a tolerance set from the maximum
is a tolerance set by the noisiest instant in the run.

Usage:
    reference_compare.py LOG_A LOG_B [--from S] [--to S]
"""
import argparse
import sys
from pathlib import Path

from pymavlink import DFReader

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


def quantiles(values):
    if not values:
        return (0.0, 0.0, 0.0)
    s = sorted(values)
    return (s[len(s) // 2], s[min(len(s) - 1, int(len(s) * 0.99))], s[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a", type=Path)
    ap.add_argument("b", type=Path)
    ap.add_argument("--from", dest="start", type=float, default=0.0,
                    help="window start, seconds of simulated time")
    ap.add_argument("--to", dest="end", type=float, default=float("inf"),
                    help="window end, seconds of simulated time")
    args = ap.parse_args()

    lo = int(args.start * 1e6)
    hi = args.end * 1e6

    print("window: %.1f s .. %s simulated" %
          (args.start, "end" if args.end == float("inf") else "%.1f s" % args.end))
    print()
    print("%-5s %8s  %10s %10s %10s   %s"
          % ("msg", "samples", "median", "p99", "max", "worst field"))

    overall = {}
    for msg_type, fields in SERIES.items():
        left = load(args.a, msg_type, fields)
        right = load(args.b, msg_type, fields)
        shared = [t for t in (set(left) & set(right)) if lo <= t <= hi]
        if not shared:
            print("%-5s %8s  no shared samples in the window" % (msg_type, "-"))
            continue

        per_sample = []
        worst_field, worst = None, 0.0
        for t in shared:
            w = 0.0
            for name, x, y in zip(fields, left[t], right[t]):
                d = angular_delta(x, y) if name in ANGLE_FIELDS else abs(x - y)
                if d > w:
                    w = d
                if d > worst:
                    worst, worst_field = d, name
            per_sample.append(w)

        med, p99, mx = quantiles(per_sample)
        overall[msg_type] = (len(shared), med, p99, mx, worst_field)
        print("%-5s %8d  %10.5g %10.5g %10.5g   %s"
              % (msg_type, len(shared), med, p99, mx, worst_field or "-"))

    if not overall:
        print("\nnothing comparable in this window.")
        return 2

    print()
    print("A tolerance taken from these numbers absorbs SIMULATOR noise only:")
    print("both logs are upstream. A port difference larger than the p99 here")
    print("would be visible; one smaller would not, and claiming otherwise")
    print("would be claiming a resolution this oracle does not have.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
