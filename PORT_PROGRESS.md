# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-26 09:25  
**Server:** `ssh root@172.233.192.152` → `/srv/ardumaster/ports/plane-fw-rust`  
**Loop:** `Tools/loop-ardumaster.ps1` — every **10 minutes** (PID tracked in Cursor terminals)  
**Goal:** Complete fw-rust fixed-wing port

## STATUS (latest)

| Field | Value |
|-------|-------|
| **fw-rust progress** | 29.4% (57,402 / 195,023 weighted loc) |
| **Strategy** | Near-done first: FW-004 → FW-018 → FW-029, then critical path |
| **Last commit** | (pending this iteration) |
| **Last touch** | FW-018 dspoiler mixer, FW-029 deepstall heading breakout |
| **Tests** | ap-plane + ap-landing pass on server |
| **Next target** | FW-029 deepstall steering slice, FW-004 convert_class table |

## Critical path (must finish to fly)

```
FW-011 INS SITL (15%) → FW-012/13/14 sensors → FW-008 AHRS (8%) → FW-020 main loop (0%)
```

## Session log

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-26 | 3dd77ce | COP-020 ✅ | winch, terrain, disarm guards |
| 2026-08-26 | 3dd77ce | FW-011 | SITL INS backend → 15% |
| 2026-08-26 | f737425 | FW-013 | Baro SITL transforms → 25% |
| 2026-08-26 | f737425 | FW-021 ✅ | mode.cpp scope complete |
| 2026-08-26 | f452033 | FW-029 | Go-around dispatch → 68% |
| 2026-08-26 | f452033 | FW-018 | Elevon mixer → 78% |
| 2026-08-26 | 5392290 | FW-004 | Conversion slice + baro_probe → 85% |
| 2026-08-26 | 5392290 | FW-029 | Deepstall predict + verify_breakout → 72% |
| 2026-08-26 | (pending) | FW-018 | Dspoiler mixer in servo_mix.rs |
| 2026-08-26 | (pending) | FW-029 | heading_error_deg + verify_breakout_vectors |

## Near-done tickets

| Ticket | % | Remaining |
|--------|---|-----------|
| **FW-004** | 85 | Vehicle conversion tables, convert_class |
| **FW-018** | 78→~82 | RC/HAL/save_trim blocked; dspoiler done |
| **FW-029** | 72→~74 | Deepstall steering + state machine |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One meaningful slice on highest-priority near-done ticket
3. `cargo test --workspace` on server
4. Commit + update tracker + this STATUS block
5. Post STATUS to user

## Completion criteria

- All fw-rust P0/P1 tickets at 100%
- SITL trajectory diff passes (FW-007)
- Scripted mission completes in SITL
