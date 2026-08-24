#!/usr/bin/env python3
"""Patch the REFERENCE BUILD to log AP_TECS's actual API arguments (ADR-0008).

WHY THIS EXISTS
---------------
`AP_TECS::update_pitch_throttle` takes nine arguments. Upstream's TECS log
message records the controller's INTERNAL state (h, dh, sp, dsp, th, ph, ...),
not those arguments, and five of them are never logged anywhere:

    distance_beyond_land_wp, ptchMinCO_cd, throttle_nudge,
    load_factor, pitch_trim_deg

Without them a replay cannot drive the ported TECS with the inputs upstream
actually received, so any output difference would be unattributable.

ADR-0008 permits patching logging into the REFERENCE BUILD ONLY, never into
the port. This adds one `WriteStreaming` call at the top of
`update_pitch_throttle` recording all nine arguments as message `TECI`
(TECS Inputs).

SCOPE AND SAFETY
----------------
- Applies to the pinned worktree used to build the reference binary.
- Adds only a log write; it does not alter control flow or any computed value.
- Idempotent: refuses to apply twice.
- Reversible: `--revert` removes it.
- The patch lives in the port repo so the reference build is reproducible and
  auditable, rather than being an undocumented local edit.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp"
)

ANCHOR = """                                    float pitch_trim_deg)
{
    uint64_t now = AP_HAL::micros64();"""

PATCH = """                                    float pitch_trim_deg)
{
    uint64_t now = AP_HAL::micros64();
    // ---- ADR-0008 REFERENCE-BUILD-ONLY LOGGING (not upstream) ----
    // Records the actual arguments to this call so a ported TECS can be
    // driven with the inputs upstream received. Five of these are logged
    // nowhere else. Log write only; no control flow or value is changed.
    AP::logger().WriteStreaming(
        // Labels must fit ArduPilot 64-char limit including commas; the
        // descriptive names came to 68 and were rejected at startup.
        "TECI", "TimeUS,hdem,easd,stg,dbey,pmin,thnu,hafe,ldf,ptrm",
        // 10 labels, 10 type chars. ArduPilot validate_structure() rejects a
        // mismatch at startup, which is how an earlier 11-char version was
        // caught: Q=uint64 i=int32 B=uint8 f=float h=int16
        "QiiBfihfff",
        now,
        (int32_t)hgt_dem_cm,
        (int32_t)EAS_dem_cm,
        (uint8_t)flight_stage,
        (float)distance_beyond_land_wp,
        (int32_t)ptchMinCO_cd,
        (int16_t)throttle_nudge,
        (float)hgt_afe,
        (float)load_factor,
        (float)pitch_trim_deg);
    // ---- end reference-build-only logging ----"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found: {}".format(TARGET))
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("not applied; nothing to revert")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted TECI logging from reference build")
        return

    if PATCH in text:
        print("already applied")
        return
    if ANCHOR not in text:
        sys.exit("anchor not found - upstream source differs from expectation")

    TARGET.write_text(text.replace(ANCHOR, PATCH, 1))
    print("applied TECI logging to reference build: {}".format(TARGET))


if __name__ == "__main__":
    main()
