#!/usr/bin/env python3
"""Log the height-demand stage's internal states (TECL).

The stage comparison isolated the divergence to `_hgt_dem`, and both the port's
`update_normal` and the `_hgt_dem_in` freeze read faithfully against upstream.
What is left is the states they carry between calls — in particular the
adaptive `_max_climb_scaler`, which gates the climb rate limit and which the
port is winding down faster than upstream.

None of it is logged, so the comparison cannot go further without it.

Logged at the END of update_pitch_throttle, not the top like TECI/TECJ/TECK,
so the values are post-update and line up with the port's snapshot taken after
its own call. Logging them at the top would compare across an iteration
boundary and manufacture an off-by-one.

13 labels, 13 type chars, 58 characters of label text - inside ArduPilot's
64-char limit. Both counts are asserted below: validate_structure() rejects a
mismatch at startup, which is how two earlier patches in this series were
caught.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp")

LABELS = "TimeUS,hdin,hdip,hrtl,hlpf,mcs,mss,crl,srl,pto,tcs,scs,pdu"
FMT = "QfffffffffBBf"

ANCHOR = """                                    (double)_TAS_rate_dem,
                                    _flags_byte);
    }
#endif
}"""

PATCH = """                                    (double)_TAS_rate_dem,
                                    _flags_byte);
    }
#endif
    // ---- reference-build-only logging ----
    // Height-demand stage state, logged at the end of the update so the values
    // are post-update and can be compared against a port's own end-of-call
    // state without an off-by-one.
    AP::logger().WriteStreaming(
        "TECL", "%s",
        "%s",
        AP_HAL::micros64(),
        (float)_hgt_dem_in,
        (float)_hgt_dem_in_prev,
        (float)_hgt_dem_rate_ltd,
        (float)_hgt_dem_lpf,
        (float)_max_climb_scaler,
        (float)_max_sink_scaler,
        (float)_climb_rate_limit,
        (float)_sink_rate_limit,
        (float)_post_TO_hgt_offset,
        (uint8_t)_thr_clip_status,
        (uint8_t)_SEBdot_dem_clip,
        (float)_pitch_dem_unc);
    // ---- end reference-build-only logging ----
}""" % (LABELS, FMT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("TECL not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted TECL")
        return

    if PATCH in text:
        print("TECL already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    assert len(LABELS) <= 64, "label string is %d chars" % len(LABELS)
    assert len(LABELS.split(",")) == len(FMT), "%d labels vs %d type chars" % (
        len(LABELS.split(",")), len(FMT))

    TARGET.write_text(text.replace(ANCHOR, PATCH, 1))
    print("applied TECL logging (%d labels, %d chars)" % (
        len(LABELS.split(",")), len(LABELS)))


if __name__ == "__main__":
    main()
