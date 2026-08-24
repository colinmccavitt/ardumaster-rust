#!/usr/bin/env python3
"""Measure the SITL self-divergence noise floor.

Two runs of the SAME binary with identical inputs are not byte-identical.
The question that matters for FW-007 is how far the decoded STATE diverges,
because any sitl-diff tolerance must be wider than this floor - otherwise the
harness reports differences that are SITL's own jitter, not port defects.

Compares state series sampled on simulated time (TimeUS), which is the only
common axis the two runs share.
"""
import argparse
import sys
from pathlib import Path

from pymavlink import DFReader

# message type -> fields that describe vehicle state
SERIES = {
    "ATT": ["Roll", "Pitch", "Yaw"],
    "POS": ["Lat", "Lng", "Alt"],
    "IMU": ["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"],
}


def load(path: Path, mtype: str, fields):
    """Return {TimeUS: (values...)} for one message type."""
    log = DFReader.DFReader_binary(str(path))
    out = {}
    while True:
        m = log.recv_match(type=mtype)
        if m is None:
            break
        d = m.to_dict()
        try:
            key = int(d["TimeUS"])
            out[key] = tuple(float(d[f]) for f in fields)
        except (KeyError, TypeError, ValueError):
            continue
    return out


def compare(a, b, fields, label):
    common = sorted(set(a) & set(b))
    print("\n--- {} ---".format(label))
    print("  samples: run1={:,}  run2={:,}  shared TimeUS={:,}".format(
        len(a), len(b), len(common)))
    if not common:
        print("  NO SHARED TIMESTAMPS - the two runs do not even sample the")
        print("  same simulated instants, so pointwise comparison is impossible")
        return None

    worst = [0.0] * len(fields)
    for t in common:
        va, vb = a[t], b[t]
        for i in range(len(fields)):
            d = abs(va[i] - vb[i])
            if d > worst[i]:
                worst[i] = d

    for i, f in enumerate(fields):
        print("  max |delta| {:<6}: {:.6g}".format(f, worst[i]))
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run1", default="/srv/ardumaster/reference/determinism/run1")
    ap.add_argument("--run2", default="/srv/ardumaster/reference/determinism/run2")
    args = ap.parse_args()

    l1 = sorted(Path(args.run1).rglob("*.BIN"))
    l2 = sorted(Path(args.run2).rglob("*.BIN"))
    if not l1 or not l2:
        sys.exit("logs not found")
    p1, p2 = l1[0], l2[0]
    print("run1: {}".format(p1))
    print("run2: {}".format(p2))

    any_shared = False
    for mtype, fields in SERIES.items():
        try:
            a = load(p1, mtype, fields)
            b = load(p2, mtype, fields)
        except Exception as e:  # noqa: BLE001 - report and continue
            print("\n--- {} --- decode failed: {}".format(mtype, e))
            continue
        res = compare(a, b, fields, mtype)
        if res is not None:
            any_shared = True

    print("\n=== interpretation ===")
    if not any_shared:
        print("  The runs share no simulated timestamps at all. SITL is not")
        print("  reproducible even on its own time axis, so golden-trajectory")
        print("  comparison against recorded logs cannot work as specified in")
        print("  ADR-0005.")
    else:
        print("  Any sitl-diff tolerance must exceed the deltas above, which")
        print("  are SITL's own run-to-run jitter rather than port error.")


if __name__ == "__main__":
    main()
