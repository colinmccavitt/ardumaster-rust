# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-26  
**Server:** `ssh root@172.233.192.152` → `/srv/ardumaster/ports/plane-fw-rust`  
**Goal:** Complete fw-rust fixed-wing port (currently ~28%)

## Critical path (must finish to fly)

```
FW-011 INS SITL (15%) → FW-012/13/14 sensors → FW-008 AHRS (8%) → FW-020 main loop (0%) → modes/mission
```

## Session log

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-26 | 3dd77ce | COP-020 ✅ | winch, terrain, disarm guards — **done** |
| 2026-08-26 | 3dd77ce | FW-011 | SITL INS deterministic backend slice → 15% |
| 2026-08-26 | f737425 | FW-013 | Baro SITL transforms → 25% |
| 2026-08-26 | f737425 | FW-021 ✅ | Marked done — mode.cpp scope complete |
| 2026-08-26 | f452033 | FW-029 | Go-around dispatch → 68% |
| 2026-08-26 | f452033 | FW-018 | Elevon mixer → 78% |
| 2026-08-26 | (pending) | FW-004 | BARO probe: anomaly not reproduced; slice 4 conversion primitives |
| 2026-08-26 | (pending) | FW-018 | Flaperon mixer fix + tests |
| 2026-08-26 | (pending) | FW-029 | Deepstall predict_travel_distance + verify_breakout |

## Near-done strategy (active)

Finishing highest-% tickets before critical-path sensors:
1. **FW-004** (78%→~85%) — slice 4 conversion primitives; BARO blocker cleared
2. **FW-018** (78%) — flaperon mixer; RC/HAL/params remain
3. **FW-029** (68%→~72%) — Deepstall first slice; full class remains

## FW-004 BARO investigation (2026-08-26)

`tools/sitl_diff/baro_probe.py` 2×2 matrix (cold/warm × early/late dump):
- BARO group always **36 params** in all cases — cold-EEPROM anomaly **not reproduced**
- Early dump has fewer total params (3581 vs 3634) due to init order (subgroups null before vehicle init), not BARO-specific
- **Slice 4 may proceed**; param dumps must be taken after vehicle init for full enumeration

## fw-rust status

### Done (16 tickets)
FW-001, 002, 003, 005, 006, 007, 015, 016, 017, 031, 033, 034, 035, 036, 038, 039

### In progress — priority order

| Ticket | % | Next action |
|--------|---|-------------|
| **FW-021** | ✅ | Marked done — mode.cpp scope complete |
| **FW-004** | ~85 | Slice 4 conversion primitives; BARO blocker cleared |
| **FW-018** | 78 | Flaperon mixer; RC/HAL/save_trim still blocked |
| **FW-029** | ~72 | Deepstall predict/breakout slice; full class remains |
| **FW-011** | 15 | Critical path — after near-done cleared |
| **FW-025** | 20 | Vehicle control glue |

### Not started (blocks flight)
- **FW-020** — Plane main loop (0%)
- **FW-019** — RC channels (0%)
- **FW-022/023** — Flight modes (0%)
- **FW-024** — AP_Mission (0%)
- **FW-009** — NavEKF3 (0%)

## copter-rust status (~35%)

### Done
COP-001, 003, 004, 007, 020, 029

### In progress — priority
COP-030 (95%), COP-013 (94%), COP-008 (91%), COP-005 (70%), COP-009 (36%)

## Next loop iteration should

1. Read this file and run `tracker.py status -v` on server
2. Pick highest-impact item on fw-rust critical path
3. Implement + test on server (`source /root/.cargo/env && cargo test`)
4. Commit + update tracker + this file
5. Repeat until fw-rust shows 100%

## Completion criteria

Port is **done** when:
- fw-rust tracker shows all P0/P1 tickets at 100%
- SITL trajectory diff passes (FW-007 harness vs upstream)
- Plane can complete a scripted mission in SITL
