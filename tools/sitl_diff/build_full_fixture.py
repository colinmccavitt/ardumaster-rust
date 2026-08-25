#!/usr/bin/env python3
"""Build the complete TECS replay fixture: TECI + TECJ + TECS on exact TimeUS.

All three messages are written from the same update_pitch_throttle call using
the same `now`, so the join is exact rather than interpolated. Any row missing
from any stream is dropped and reported rather than approximated.

  TECI - the nine update_pitch_throttle arguments (reference-build patch)
  TECJ - 50 Hz-stage state the control logic consumes (reference-build patch)
  TECS - upstream's own outputs, plus the filter outputs h/dh/sp/dsp
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/tecs_replay.csv")

TECI = ["hdem", "easd", "stg", "dbey", "pmin", "thnu", "hafe", "ldf", "ptrm"]
TECJ = ["vdlpf", "tasd", "tasmn", "tasmx", "tascr", "cosr", "ptchm"]
# TECK: AP_Landing-derived flags and external limits, which the control
# logic reads but upstream does not otherwise log
TECK = ["prop", "appr", "flar", "glid", "uas", "lpcd",
        "pmne", "pmxe", "tmne", "tmxe", "pfai"]
# from TECS: the 50Hz filter outputs the control logic consumes, then upstream's
# own control outputs
TECS_IN = ["h", "dh", "sp", "dsp"]
# th/ph are the controller's outputs; hdem/spdem/dhdem/dspdem are its own
# demand states, so the demand chain can be compared too
TECS_OUT = ["th", "ph", "hdem", "spdem", "dhdem", "dspdem", "pmin", "pmax"]
# TEC2: the pitch path's intermediates, so a divergence can be localised
TEC2_OUT = ["PEW", "KEW", "EBD", "EBE", "EBDD", "EBDE", "EBDDT",
            "Imin", "Imax", "I", "KI", "tmin", "tmax"]
# TECL: the height-demand stage's carried state, logged at the end of the
# update so it lines up with the port's post-call snapshot
# TEC4: the four energy terms, unweighted, so each can be compared on its
# own rather than through a weighted sum that can mask one of them
TEC4_OUT = ["P", "K", "Pdem", "Kdem"]
TECL_OUT = ["hdin", "hdip", "hrtl", "hlpf", "mcs", "mss", "crl", "srl",
            "pto", "tcs", "scs", "pdu", "dt"]

i = dict(read_series(LOG, "TECI", TECI))
j = dict(read_series(LOG, "TECJ", TECJ))
k = dict(read_series(LOG, "TECK", TECK))
s_in = dict(read_series(LOG, "TECS", TECS_IN))
s_out = dict(read_series(LOG, "TECS", TECS_OUT))
e_out = dict(read_series(LOG, "TEC2", TEC2_OUT))
l_out = dict(read_series(LOG, "TECL", TECL_OUT))
f_out = dict(read_series(LOG, "TEC4", TEC4_OUT))

common = sorted(set(i) & set(j) & set(k) & set(s_in) & set(e_out) & set(l_out)
                & set(f_out))
print("TECI {:,}  TECJ {:,}  TECK {:,}  TECS {:,}  ->  {:,} exact-joined".format(
    len(i), len(j), len(k), len(s_in), len(common)))
dropped = max(len(i), len(j), len(k), len(s_in)) - len(common)
if dropped:
    print("  {} row(s) unmatched and dropped".format(dropped))

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(
        ["time_us"]
        + ["in_" + x for x in TECI]
        + ["in_" + x for x in TECJ]
        + ["in_" + x for x in TECK]
        + ["in_" + x for x in TECS_IN]
        + ["out_" + x for x in TECS_OUT]
        + ["out_" + x for x in TEC2_OUT]
        + ["out_" + x for x in TECL_OUT]
        + ["out_" + x for x in TEC4_OUT]
    )
    for t in common:
        w.writerow(
            [t]
            + ["{:.9g}".format(v) for v in i[t]]
            + ["{:.9g}".format(v) for v in j[t]]
            + ["{:.9g}".format(v) for v in k[t]]
            + ["{:.9g}".format(v) for v in s_in[t]]
            + ["{:.9g}".format(v) for v in s_out[t]]
            + ["{:.9g}".format(v) for v in e_out[t]]
            + ["{:.9g}".format(v) for v in l_out[t]]
            + ["{:.9g}".format(v) for v in f_out[t]]
        )

print("wrote {} ({} input cols, {} output cols)".format(
    OUT.name, len(TECI) + len(TECJ) + len(TECK) + len(TECS_IN),
    len(TECS_OUT) + len(TEC2_OUT) + len(TECL_OUT) + len(TEC4_OUT)))
