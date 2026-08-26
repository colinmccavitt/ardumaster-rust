#!/usr/bin/env python3
"""FW-007: is the noise floor stable, or was one pair lucky?

The floor so far comes from a single pair of runs. A tolerance drawn from one
pair says nothing about whether the next pair would agree -- and a tolerance
that is itself noisy is worse than no tolerance, because it looks precise.

This runs N and compares every pair, then reports the spread of each field's
p99 across pairs. A field whose p99 barely moves between pairs has a
trustworthy floor; one that swings has not been measured yet, whatever the
first pair said.
"""
import argparse
import itertools
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import reference_compare as rc  # noqa: E402
import run_lua_runs as runner  # noqa: E402

WORK = Path("/srv/ardumaster/reference/tolerance")


def pair_stats(a: Path, b: Path, lo_us: int, hi_us: float):
    """p99 per message type for one pair of logs."""
    out = {}
    for msg_type, fields in rc.SERIES.items():
        left = rc.load(a, msg_type, fields)
        right = rc.load(b, msg_type, fields)
        shared = [t for t in (set(left) & set(right)) if lo_us <= t <= hi_us]
        if not shared:
            continue
        per_sample = []
        for t in shared:
            w = 0.0
            for name, x, y in zip(fields, left[t], right[t]):
                d = (rc.angular_delta(x, y) if name in rc.ANGLE_FIELDS
                     else abs(x - y))
                w = max(w, d)
            per_sample.append(w)
        _, p99, mx = rc.quantiles(per_sample)
        out[msg_type] = (len(shared), p99, mx)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--from", dest="start", type=float, default=10.0)
    ap.add_argument("--to", dest="end", type=float, default=130.0)
    ap.add_argument("--reuse", action="store_true",
                    help="compare logs already present instead of flying again")
    args = ap.parse_args()

    WORK.mkdir(parents=True, exist_ok=True)
    logs = []
    for i in range(1, args.runs + 1):
        rundir = WORK / ("run%d" % i)
        existing = sorted((rundir / "logs").glob("*.BIN")) if (rundir / "logs").exists() else []
        if args.reuse and existing:
            biggest = max(existing, key=lambda p: p.stat().st_size)
            print("run %d: reusing %s" % (i, biggest))
            logs.append(biggest)
            continue
        print("=== run %d of %d ===" % (i, args.runs))
        # run_once warns and returns None on the wall-clock backstop, which
        # this study does not care about: the comparison window is bounded by
        # simulated time, so how a run ended is irrelevant as long as it got
        # past the window.
        runner.run_once(rundir)
        produced = sorted((rundir / "logs").glob("*.BIN")) if (rundir / "logs").exists() else []
        if not produced:
            print("  no log; aborting")
            return 2
        logs.append(max(produced, key=lambda p: p.stat().st_size))

    lo, hi = int(args.start * 1e6), args.end * 1e6
    pairs = list(itertools.combinations(range(len(logs)), 2))
    print("\ncomparing %d pairs over %.0f-%.0f s simulated\n"
          % (len(pairs), args.start, args.end))

    collected = {}
    for i, j in pairs:
        stats = pair_stats(logs[i], logs[j], lo, hi)
        for msg_type, (n, p99, mx) in stats.items():
            collected.setdefault(msg_type, []).append((p99, mx))
        print("  runs %d/%d: %s" % (
            i + 1, j + 1,
            "  ".join("%s p99=%.4g" % (k, v[1]) for k, v in sorted(stats.items()))))

    print()
    print("%-6s %10s %10s %10s   %s"
          % ("msg", "min p99", "max p99", "spread", "verdict"))
    for msg_type in sorted(collected):
        p99s = [p for p, _ in collected[msg_type]]
        lo_p, hi_p = min(p99s), max(p99s)
        spread = hi_p - lo_p
        # A floor is trustworthy when the spread is small next to the value
        # itself. Ten percent is arbitrary but it has to be something, and it
        # is stated rather than hidden in a comparison.
        stable = hi_p == 0 or spread <= 0.10 * hi_p
        print("%-6s %10.5g %10.5g %10.5g   %s"
              % (msg_type, lo_p, hi_p, spread,
                 "stable" if stable else "NOT stable across pairs"))

    print()
    print("Suggested tolerance is the worst p99 seen across pairs, not the")
    print("first one measured. Anything tighter would fail on a run that")
    print("differs from the port not at all.")
    for msg_type in sorted(collected):
        worst = max(p for p, _ in collected[msg_type])
        print("    %-6s %.5g" % (msg_type, worst))
    return 0


if __name__ == "__main__":
    sys.exit(main())
