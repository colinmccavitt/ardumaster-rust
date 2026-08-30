#!/usr/bin/env python3
"""Extract log-replay fixtures from an upstream dataflash log (ADR-0008).

Pairs a module's INPUTS with upstream's own OUTPUTS at the same instant. The
ported Rust module is driven with those inputs and its outputs compared with
upstream's - deterministic by construction, because the inputs are fixed data
rather than a live simulation.

# Only atomic messages are usable

A fixture is only sound when inputs and outputs are logged in the SAME message,
at the same instant. Joining two message streams by nearest timestamp does not
work: XKQ (EKF quaternion) and ATT (upstream's euler angles) are both logged at
5 Hz but are NOT synchronised, giving ~40 ms of skew. Over 40 ms an aircraft
actually rotates, so such a comparison measures the skew rather than the
port's conversion error.

TECS is the model case: every input and output it needs sits in one TECS record.

Plain CSV on purpose: the Rust side parses it dependency-free, and the fixture
stays readable and diffable in review.
"""
import argparse
import csv
from pathlib import Path

from pymavlink import DFReader

# Fixtures that come from a single atomic message.
#   name -> (message type, input fields, output fields)
ATOMIC = {
    "tecs": (
        "TECS",
        # inputs: current state, demands and limits handed to the controller
        ["h", "dh", "hin", "hdem", "dhdem", "spdem", "sp", "dsp",
         "pmin", "pmax", "dspdem", "f"],
        # outputs: what upstream's TECS produced from them
        ["th", "ph"],
    ),
    "pid_roll": (
        "PIDR",
        ["Tar", "Act", "Err", "FF", "DFF", "SRate", "Flags"],
        ["P", "I", "D", "Dmod"],
    ),
}


def read_series(path, mtype, fields):
    """[(TimeUS, (values...)), ...] for one message type."""
    log = DFReader.DFReader_binary(str(path))
    out = []
    while True:
        m = log.recv_match(type=mtype)
        if m is None:
            break
        d = m.to_dict()
        try:
            out.append((int(d["TimeUS"]), tuple(float(d[f]) for f in fields)))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def extract(log_path: Path, out_dir: Path, name: str) -> int:
    mtype, ins, outs = ATOMIC[name]
    log = DFReader.DFReader_binary(str(log_path))
    rows = []
    while True:
        m = log.recv_match(type=mtype)
        if m is None:
            break
        d = m.to_dict()
        try:
            rows.append(
                [int(d["TimeUS"])]
                + [float(d[f]) for f in ins]
                + [float(d[f]) for f in outs]
            )
        except (KeyError, TypeError, ValueError):
            continue

    path = out_dir / "{}.csv".format(name)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_us"] + ["in_" + x for x in ins] + ["out_" + x for x in outs])
        for r in rows:
            w.writerow([r[0]] + ["{:.9g}".format(x) for x in r[1:]])
    print("  {:<10} {:,} rows  ({} inputs, {} outputs)  -> {}".format(
        name, len(rows), len(ins), len(outs), path.name))
    return len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", default="/srv/ardumaster/ports/ardumaster-rust/fixtures")
    ap.add_argument("--only", help="extract just one fixture by name")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    print("source log: {}".format(args.log))

    names = [args.only] if args.only else sorted(ATOMIC)
    total = 0
    for n in names:
        total += extract(Path(args.log), out, n)
    if total == 0:
        raise SystemExit("no rows extracted")


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------------
# TECS replay fixture: TECI (inputs) joined to TECS (outputs) on EXACT TimeUS.
#
# These are two messages, but both are written from the same
# update_pitch_throttle() call using the same `now`, so an exact-timestamp
# join is atomic in the sense that matters - no interpolation, no skew. The
# join is asserted to be exact; any unmatched row is dropped and reported
# rather than silently approximated.
#
# TECI exists only because upstream logs TECS internal state rather than the
# nine arguments the controller actually received, five of which are logged
# nowhere. It is added by tools/sitl_diff/patches/apply_tecs_logging.py to the
# REFERENCE BUILD ONLY, per ADR-0008.
# ---------------------------------------------------------------------------

TECI_INPUTS = ["hdem", "easd", "stg", "dbey", "pmin", "thnu", "hafe", "ldf", "ptrm"]
TECS_OUTPUTS = ["th", "ph"]


def tecs_replay(log_path, out_dir):
    ins = dict(read_series(log_path, "TECI", TECI_INPUTS))
    outs = dict(read_series(log_path, "TECS", TECS_OUTPUTS))
    common = sorted(set(ins) & set(outs))
    dropped = len(ins) - len(common)

    path = out_dir / "tecs_replay.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_us"] + ["in_" + x for x in TECI_INPUTS]
                   + ["out_" + x for x in TECS_OUTPUTS])
        for t in common:
            w.writerow([t] + ["{:.9g}".format(v) for v in ins[t]]
                       + ["{:.9g}".format(v) for v in outs[t]])
    print("  {:<12} {:,} rows exact-joined (TECI {:,}, TECS {:,}, {} unmatched)".format(
        "tecs_replay", len(common), len(ins), len(outs), dropped))
    return len(common)
