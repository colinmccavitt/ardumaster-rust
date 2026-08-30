#!/usr/bin/env python3
"""Patch a REFERENCE BUILD of upstream to drive a scripted flight from the
scheduler's own tick counter (ADR-0014, FW-048).

WHY THIS EXISTS
---------------
ADR-0008/FW-007 measured two ways of driving a whole-vehicle SITL flight for
trajectory comparison, and both failed on timing jitter rather than physics
nondeterminism: MAVProxy/autotest diverged up to 349 degrees because commands
arrive over a socket at wall-clock times; onboard Lua narrowed that to a
1.72 degree / 173 centidegree floor across six run pairs, still unstable
run-to-run, because `AP_Scripting` runs the driver on its own OS thread
(`AP_Scripting.cpp:257`), independently scheduled from the 400 Hz main loop.

Both drivers share a root cause: the thing deciding "arm now" / "change mode
now" runs on a different thread (or machine) than the control loop being
steered, so its decision lands at a different simulated tick between two
runs of the identical binary. This patch removes that root cause instead of
narrowing it further: the command schedule is dispatched from
`AP_Scheduler::tick()`'s own tick counter, on the same thread, at the same
call-stack depth, immediately before that tick's tasks run. There is no
socket, no scripting VM, and no second thread anywhere in the command path.

SCHEDULE
--------
Reuses `tools/sitl_diff/mission_driver.lua`'s own arm -> TAKEOFF -> LOITER ->
RTL -> disarm shape (FW-007's ArduPlane FBWA-adjacent scripted flight), which
already gave FW-046/047's `sitl_run` port harness a proven, independent
oracle to sanity-check against. That script keyed its schedule on
`millis()` (simulated time under SITL, but still read from a separate
thread); this patch keys the identical wall-clock schedule on
`AP_Scheduler::ticks32()` instead, converted at this build's confirmed
400 Hz default main loop rate (`SCHEDULER_DEFAULT_LOOP_RATE`,
`AP_Scheduler.cpp`), i.e. 1 tick = 2.5 ms:

    lua ms   ticks (@400 Hz)   action
    -----    ---------------   ------
     5000        2000          arm_force()
     8000        3200          set_mode(TAKEOFF = 13)
    40000       16000          set_mode(LOITER  = 12)
    80000       32000          set_mode(RTL     = 11)
   120000       48000          disarm()

PATCH POINT (verified directly against this worktree's current source, not
assumed from the ADR - line numbers there had already drifted by ~2 lines
against a defined-function search, though the `loop()` line the ADR/ticket
cited, 348, was exact):

    libraries/AP_Scheduler/AP_Scheduler.cpp, AP_Scheduler::loop():
        tick();                    // line 379 in this worktree
        // <-- injected block goes here -->
        run(time_available);       // line 399 in this worktree

`tick()` (defined at line 169) only increments `_tick_counter32`; it does
not touch scheduling state, allocate, or block, so inserting a hardcoded
`if (_tick_counter32 == N)` dispatch immediately after it disturbs nothing
about which tasks `run()` goes on to execute that tick.

APIS USED (verified directly against this worktree's current headers):
    AP::arming().arm_force(AP_Arming::Method) -> bool   (AP_Arming.h:106)
    AP::arming().disarm(AP_Arming::Method, bool=true)   (AP_Arming.h:107)
    AP::vehicle()->set_mode(uint8_t, ModeReason) -> bool
        (AP_Vehicle.h:123, pure virtual; Plane::set_mode overrides it -
        AP_Scheduler.cpp already calls AP::vehicle() itself at its own
        line ~127, and already includes AP_Vehicle.h, which pulls in
        ModeReason.h transitively - so only AP_Arming.h needs adding)
`AP_Arming::Method::SCRIPTING` is used for both calls: it is the closest
existing enumerator to "driven by an embedded script", and matches the
Method the sibling Lua driver's own `arm_force()`/`disarm()` calls exercise
conceptually, without inventing a new enumerator.

SCOPE AND SAFETY
-----------------
- Applies ONLY to an isolated worktree, never the shared canonical
  `/srv/ardumaster/upstream/plane-4.7.0` tree (see FW-048's own safety
  note - that tree had live, uncommitted, unrelated modifications from a
  concurrent process at the time this was written). Unlike the sibling
  logging-only patches in this directory, this patch actively drives the
  vehicle's control flow (arms it, changes its flight mode), so stacking it
  on unrelated concurrent upstream edits - or having them reverted mid-run
  - would silently corrupt measurement, not just add noise to a log field.
  For that reason the target path is a required, explicit argument here
  (unlike the sibling patches, which hardcode the shared tree): there is no
  safe default to fall back to.
- Adds one bounded, hardcoded dispatch block and one #include. Does not
  alter any existing scheduling, arming, or mode-change logic - every branch
  it takes calls the same public APIs a MAVLink command or Lua script would
  have called.
- Idempotent: refuses to apply twice.
- Reversible: `--revert` removes it.
- REFERENCE-ONLY: never touches the port, never touches a build the port
  ships. Matches the precedent of `apply_tecs_logging.py`, `add_rcti.py`,
  `extend_logging.py` in this same directory, plus the extra care noted
  above for a patch that drives control flow rather than only logging.
"""
import argparse
import sys
from pathlib import Path

# Tick counts = mission_driver.lua's own ms schedule / 2.5 ms per tick
# (400 Hz SCHEDULER_DEFAULT_LOOP_RATE, confirmed in AP_Scheduler.cpp).
SCHEDULE = [
    (2000, "arm"),
    (3200, "mode", 13),   # TAKEOFF
    (16000, "mode", 12),  # LOITER
    (32000, "mode", 11),  # RTL
    (48000, "disarm"),
]

INCLUDE_MARKER = '#include <AP_Vehicle/AP_Vehicle.h>\n'
INCLUDE_LINE = '#include <AP_Arming/AP_Arming.h>\n'

ANCHOR = """    // tell the scheduler one tick has passed
    tick();

    // run all the tasks that are due to run. Note that we only"""

PATCH = """    // tell the scheduler one tick has passed
    tick();

    // ---- FW-048 / ADR-0014 REFERENCE-BUILD-ONLY TICK INJECTION (not upstream) ----
    // Fixed, tick-keyed command schedule for the deterministic whole-vehicle
    // SITL diff harness. Runs on this thread, at this call-stack depth,
    // immediately after tick() and before run() - no MAVLink socket, no Lua
    // VM, no second OS thread anywhere in the command path, so the driver
    // can no longer land at a different simulated tick between two runs of
    // the identical binary. Schedule mirrors
    // tools/sitl_diff/mission_driver.lua's own arm -> TAKEOFF -> LOITER ->
    // RTL -> disarm shape (FW-007), converted from that script's
    // millis()-keyed schedule to this tick counter at 400 Hz
    // (SCHEDULER_DEFAULT_LOOP_RATE): 1 tick = 2.5 ms.
#if AP_VEHICLE_ENABLED
    {
        const uint32_t fw048_tick = _tick_counter32;
        if (fw048_tick == 2000) {
            AP::arming().arm_force(AP_Arming::Method::SCRIPTING);
        } else if (fw048_tick == 3200) {
            AP::vehicle()->set_mode(13, ModeReason::UNKNOWN);   // TAKEOFF
        } else if (fw048_tick == 16000) {
            AP::vehicle()->set_mode(12, ModeReason::UNKNOWN);   // LOITER
        } else if (fw048_tick == 32000) {
            AP::vehicle()->set_mode(11, ModeReason::UNKNOWN);   // RTL
        } else if (fw048_tick == 48000) {
            AP::arming().disarm(AP_Arming::Method::SCRIPTING);
        }
    }
#endif  // AP_VEHICLE_ENABLED
    // ---- end reference-build-only tick injection ----

    // run all the tasks that are due to run. Note that we only"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True,
                     help="path to libraries/AP_Scheduler/AP_Scheduler.cpp "
                          "inside the isolated worktree to patch - there is "
                          "deliberately no default pointing at the shared tree")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    target = Path(args.target)
    if not target.exists():
        sys.exit("target not found: %s" % target)
    text = target.read_text()

    if args.revert:
        reverted = False
        if PATCH in text:
            text = text.replace(PATCH, ANCHOR)
            reverted = True
        if INCLUDE_LINE in text:
            text = text.replace(INCLUDE_LINE, "", 1)
            reverted = True
        if not reverted:
            print("not applied; nothing to revert")
            return
        target.write_text(text)
        print("reverted tick injection from %s" % target)
        return

    if PATCH in text:
        print("already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times in %s, expected 1 - "
                  "upstream source differs from expectation" % (text.count(ANCHOR), target))

    text = text.replace(ANCHOR, PATCH, 1)
    if INCLUDE_LINE not in text:
        if INCLUDE_MARKER not in text:
            sys.exit("include anchor not found - upstream source differs from expectation")
        text = text.replace(INCLUDE_MARKER, INCLUDE_MARKER + INCLUDE_LINE, 1)

    target.write_text(text)
    print("applied tick injection to %s" % target)
    print("schedule (ticks @ 400 Hz -> action):")
    for row in SCHEDULE:
        print("    %s" % (row,))


if __name__ == "__main__":
    main()
