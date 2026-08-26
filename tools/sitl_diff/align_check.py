#!/usr/bin/env python3
"""Are the two autotest runs even aligned before being compared?

Slice 2's notes say a run must be aligned on a simulated-time event, not on
raw TimeUS, because arming and mode changes arrive at wall-clock-variable
moments. The first comparison ignored that and reported deltas of 349 degrees
of yaw and 783 PWM -- which is what comparing a climb against a turn looks
like, not what a non-deterministic simulator looks like.

So: when did each run change mode, and do those times agree?
"""
import sys
from pathlib import Path

from pymavlink import DFReader

REFERENCE = Path("/srv/ardumaster/reference/autotest")


def mode_changes(path: Path):
    log = DFReader.DFReader_binary(str(path))
    out = []
    while True:
        m = log.recv_match(type="MODE")
        if m is None:
            break
        d = m.to_dict()
        out.append((int(d.get("TimeUS", -1)), d.get("Mode"), d.get("ModeNum")))
    return out


def span(path: Path, msg_type="ATT"):
    log = DFReader.DFReader_binary(str(path))
    first = last = None
    n = 0
    while True:
        m = log.recv_match(type=msg_type)
        if m is None:
            break
        t = int(m.to_dict().get("TimeUS", 0))
        first = t if first is None else first
        last = t
        n += 1
    return first, last, n


def main():
    a = REFERENCE / "run1/00000002.BIN"
    b = REFERENCE / "run2/00000002.BIN"

    for label, p in (("run1", a), ("run2", b)):
        first, last, n = span(p)
        print("%s ATT: %d samples, TimeUS %s .. %s (%.1f s simulated)"
              % (label, n, first, last, (last - first) / 1e6))
    print()

    ma, mb = mode_changes(a), mode_changes(b)
    print("mode changes:")
    for label, changes in (("run1", ma), ("run2", mb)):
        print("  %s: %d" % (label, len(changes)))
        for t, mode, num in changes[:10]:
            print("     %10d us  %s (%s)" % (t, mode, num))
    print()

    if ma and mb:
        n = min(len(ma), len(mb))
        aligned = all(ma[i][0] == mb[i][0] for i in range(n))
        print("first %d mode changes at identical simulated times: %s" % (n, aligned))
        if not aligned:
            for i in range(n):
                if ma[i][0] != mb[i][0]:
                    print("  first difference at index %d: %d vs %d (offset %d us)"
                          % (i, ma[i][0], mb[i][0], mb[i][0] - ma[i][0]))
                    break
    return 0


if __name__ == "__main__":
    sys.exit(main())
