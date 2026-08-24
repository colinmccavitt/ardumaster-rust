#!/bin/bash
# FW-007 slice 3: determinism through an ARMED FLIGHT, driven by upstream autotest.
#
# Slice 2 established that hand-rolling arming is the wrong layer to work at:
# upstream's autotest already solves calibration, pre-arm, EKF health and
# mission upload. This runs one upstream flying test twice and preserves the
# logs for comparison.
set -uo pipefail

SRC=/srv/ardumaster/upstream/plane-4.7.0
OUT=/srv/ardumaster/reference/autotest
TEST=${TEST:-test.Plane.ClimbBeforeTurn}
VENV=/srv/ardumaster/venv

rm -rf "$OUT"
mkdir -p "$OUT"

cd "$SRC" || exit 1

for run in 1 2; do
    echo "=== $TEST run $run ==="
    rm -rf "$SRC/logs"
    timeout 900 env PATH="$VENV/bin:$PATH" "$VENV/bin/python" \
        Tools/autotest/autotest.py --speedup=20 "$TEST" \
        > "$OUT/run${run}.log" 2>&1
    rc=$?
    echo "  exit=$rc  $(grep -c 'PASSED STEP' "$OUT/run${run}.log" 2>/dev/null) passed step(s)"
    mkdir -p "$OUT/run${run}"
    if [ -d "$SRC/logs" ]; then
        cp "$SRC"/logs/*.BIN "$OUT/run${run}/" 2>/dev/null
    fi
    n=$(ls "$OUT/run${run}"/*.BIN 2>/dev/null | wc -l)
    echo "  logs captured: $n"
    ls -l "$OUT/run${run}"/*.BIN 2>/dev/null | awk '{print "    ", $5, $9}'
done
