# ArduPilot Rust Port — Loop Progress Tracker

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
| 2026-08-26 | (pending) | FW-013 | Baro SITL transforms (temp, wind, delay buffer) |
| 2026-08-26 | (pending) | FW-021 ✅ | Mark done — mode.cpp scope complete |

## fw-rust status

### Done (16 tickets)
FW-001, 002, 003, 005, 006, 007, 015, 016, 017, 031, 033, 034, 035, 036, 038, 039

### In progress — priority order

| Ticket | % | Next action |
|--------|---|-------------|
| **FW-021** | ✅ | Marked done — mode.cpp scope complete |
| **FW-011** | 15 | Add noise/vibration, multi-instance, parity fixture |
| **FW-018** | 74 | Finish SRV_Channel setters |
| **FW-004** | 78 | Resolve BARO cold-EEPROM, slice 4 |
| **FW-012** | 8 | GPS SITL backend |
| **FW-013** | 12 | Baro SITL backend (atmosphere done) |
| **FW-014** | 10 | Compass SITL backend |
| **FW-029** | 62 | Landing stage machine remainder |
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
