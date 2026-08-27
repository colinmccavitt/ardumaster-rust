# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 03:30  
**Main:** \923217b\

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~38%+ |
| **Last commit** | (pending FW-008 commit) |
| **Tests** | ✅ \cargo test --workspace\ pass on loop/fw-008-ahrs |

### Parallel batch — merged slices

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-020 Main Loop](018a6e29-a5fe-4be9-9e37-456bd2c52cee) | FW-020 | \923217b\ | \PlaneMainLoop\ scheduler + mode dispatch → **15%** |
| [FW-008 AHRS](368f3ba1-4938-41ec-922f-8898b6973f27) | FW-008 | \40b695d\ | compass yaw drift in DCM loop → **30%** |
| [FW-011 INS SITL](0bb20ca5-4e59-4293-bd56-6fc253bdfb02) | FW-011 | \5747645\ | INS_HNTCH_* param binding → **68%** |

## Near-done / critical path

| Ticket | % | Next |
|--------|---|------|
| **FW-029** | 100 | Done |
| **FW-004** | 100 | Done |
| **FW-011** | 68 | INS→AHRS publish |
| **FW-008** | 45 | Vehicle wiring (PlaneMainLoop ahrs_update) |
| **FW-020** | 15 | Wire AHRS feed, stabilize/set_servos |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | (pending) | FW-008 | GPS yaw fallback in drift_correction_yaw |
| 2026-08-27 | 923217b | FW-020 | Main loop scheduler tick (cherry-pick 538f3f5) |
| 2026-08-27 | 40b695d | FW-008 | Compass yaw drift (cherry-pick 2243d7c) |
| 2026-08-27 | 5747645 | FW-011 | INS_HNTCH_* param binding |

## Loop iteration checklist

1. Read this file + \	racker.py status -v\ on server
2. One slice on highest-priority near-done or critical-path ticket
3. \cargo test --workspace\, commit, update tracker + this file
4. Post STATUS
