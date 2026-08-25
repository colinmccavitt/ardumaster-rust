#!/usr/bin/env python3
"""Log the AP_Landing-derived flags the TECS control logic reads (TECK).

The stage-by-stage comparison against upstream's TEC2 message showed the first
divergence at the kinetic-energy weighting: upstream drops it to zero where the
port holds 1.0, on 503 of 1908 samples, and every downstream stage inherits the
error.

The cause is that `_update_pitch` reads state the replay could not supply:

    _landing.is_on_approach()   pitch limit ratcheting
    _landing.is_flaring()       flare damping and pitch limits
    _landing.get_pitch_cd()     flare pitch target
    _path_proportion            slides the speed weight to zero on approach
                                when TECS_LAND_SPDWGT < 0, which is this
                                flight's setting (-1)
    _flags.is_gliding           forces the weight to 2

None of it is in TECI/TECJ/TECS, so the replay hard-coded false/zero. With
TECS_LAND_SPDWGT = -1 the weight is `_spdWeight * (1 - path_proportion)`, so a
missing path_proportion of ~1 shows up as a weight of 1.0 instead of 0.0.

`_flags.is_doing_auto_land` is deliberately NOT logged: upstream derives it as
`flight_stage == LAND` (AP_TECS.cpp:1322) and the stage is already in TECI, so
the port must derive it the same way. Logging it would hide a mismatch.

12 labels, 12 type chars, 56 characters of label text - inside ArduPilot's
64-char limit. Both counts matter: validate_structure() rejects a mismatch at
startup, which is how two earlier versions of these patches were caught.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp")

ANCHOR = """        (float)_ahrs.get_pitch_rad());
    // ---- end reference-build-only logging ----"""

PATCH = """        (float)_ahrs.get_pitch_rad());
    // Third message: the AP_Landing-derived state and mode flags the control
    // logic reads. Without these a replay cannot reproduce the speed/height
    // weighting on approach, because TECS_LAND_SPDWGT < 0 slides the weight
    // to zero as _path_proportion goes to 1.
    // is_doing_auto_land is intentionally absent - it is derived from the
    // flight stage, which TECI already carries.
    // 12 labels, 12 type chars, 56 chars of labels.
    AP::logger().WriteStreaming(
        "TECK", "TimeUS,prop,appr,flar,glid,uas,lpcd,pmne,pmxe,tmne,tmxe,pfai",
        "QfBBBBiffffB",
        now,
        (float)_path_proportion,
        (uint8_t)_landing.is_on_approach(),
        (uint8_t)_landing.is_flaring(),
        (uint8_t)_flags.is_gliding,
        (uint8_t)use_airspeed(),
        (int32_t)_landing.get_pitch_cd(),
        (float)_PITCHminf_ext,
        (float)_PITCHmaxf_ext,
        (float)_THRminf_ext,
        (float)_THRmaxf_ext,
        (uint8_t)_flags.propulsion_failed);
    // ---- end reference-build-only logging ----"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("TECK not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted TECK")
        return

    if PATCH in text:
        print("TECK already applied")
        return
    if ANCHOR not in text:
        sys.exit("anchor not found - is the TECJ patch applied?")

    # guard the two limits that ArduPilot enforces at startup
    labels = "TimeUS,prop,appr,flar,glid,uas,lpcd,pmne,pmxe,tmne,tmxe,pfai"
    fmt = "QfBBBBiffffB"
    assert len(labels) <= 64, "label string is {} chars".format(len(labels))
    assert len(labels.split(",")) == len(fmt), "{} labels vs {} type chars".format(
        len(labels.split(",")), len(fmt))

    TARGET.write_text(text.replace(ANCHOR, PATCH, 1))
    print("applied TECK logging ({} labels, {} chars)".format(
        len(labels.split(",")), len(labels)))


if __name__ == "__main__":
    main()
