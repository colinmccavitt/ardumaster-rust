#!/usr/bin/env python3
"""FW-048: pairwise stability of the tick-injection driver, same method as
`tolerance_study.py` used for the Lua driver (FW-007).

`tolerance_study.py`'s own comparison core - `pair_stats()`, which in turn
uses `reference_compare.py`'s `SERIES`/`load`/`angular_delta`/`quantiles` -
is reused directly, unmodified, by importing it below. Only the *driver* is
swapped: `run_lua_runs.run_once(rundir)` becomes
`run_tick_injection.run_once(rundir, binary)`, because the tick-injection
runner takes an explicit `--bin` (there is no single fixed reference binary
path here - it must point at whatever isolated worktree build is under
test). Everything downstream of "given N logs, compare every pair" is
identical to the Lua study, so the two results are directly comparable.
"""
import argparse
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import tolerance_study as ts  # noqa: E402  (reuse pair_stats() unmodified)
import run_tick_injection as runner  # noqa: E402

WORK = Path("/srv/ardumaster/reference/tick_injection")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, type=Path,
                     help="tick-injection-patched arduplane binary")
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
        runner.run_once(rundir, args.bin)
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
        stats = ts.pair_stats(logs[i], logs[j], lo, hi)
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
        stable = hi_p == 0 or spread <= 0.10 * hi_p
        print("%-6s %10.5g %10.5g %10.5g   %s"
              % (msg_type, lo_p, hi_p, spread,
                 "stable" if stable else "NOT stable across pairs"))

    print()
    print("Suggested tolerance is the worst p99 seen across pairs, matching")
    print("tolerance_study.py's own precedent (FW-007).")
    for msg_type in sorted(collected):
        worst = max(p for p, _ in collected[msg_type])
        print("    %-6s %.5g" % (msg_type, worst))
    return 0


if __name__ == "__main__":
    sys.exit(main())
