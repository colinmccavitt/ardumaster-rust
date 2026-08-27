# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 05:00  
**Main:** `8e4c71c`

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~54%+ |
| **Last commit** | `955e1da` (loop/fw-008-ahrs) |
| **Tests** | ✅ `cargo test --workspace` pass on loop/fw-008-ahrs |

### Parallel batch — merged slices (batch 15)

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-008 AHRS](7768972a-6367-4fd4-8a45-2f560f24f00d) | FW-008 | `955e1da` | EKF3 update hook stub wired in ahrs_update → **70%** |
| [FW-011 INS SITL](1ce6e591-d96e-44d4-9598-e7b01dc70cc6) | FW-011 | `b5b6d1b` | SIM_IMUT temp cal param binding → **88%** |
| [FW-008 AHRS](7768972a-6367-4fd4-8a45-2f560f24f00d) | FW-008 | `9d63804` | AHRS backend selection stub (DCM vs EKF3) → **65%** |
| [FW-020 Main Loop](c1647718-0a7e-4f5d-81b3-9909e4a75f93) | FW-020 | `cf26b2e` | go-around latch in set_servos → **62%** |

## Near-done / critical path

| Ticket | % | Next |
|--------|---|------|
| **FW-029** | 100 | Done |
| **FW-004** | 100 | Done |
| **FW-011** | 88 | SITL file playback hookup |
| **FW-008** | 70 | GPS lag buffer (ra_delayed) |
| **FW-020** | 62 | Mode table wiring for scheduler |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | 955e1da | FW-008 | EKF3 update hook stub in ahrs_hookup |
| 2026-08-27 | cf26b2e | FW-020 | go_around_hookup latches commanded_go_around |
| 2026-08-27 | b5b6d1b | FW-011 | SIM_IMUT temp cal param binding |
| 2026-08-27 | f4c3ac7 | FW-008 | full wind wiring (cherry-pick c16189d) |
| 2026-08-27 | bfb23d7 | FW-011 | dynamic notch tracking |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One slice on highest-priority near-done or critical-path ticket
3. `cargo test --workspace`, commit, update tracker + this file
4. Post STATUS
