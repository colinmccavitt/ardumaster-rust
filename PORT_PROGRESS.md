# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 04:45  
**Main:** `f4c3ac7`

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~41%+ |
| **Last commit** | `7c752b4` |
| **Tests** | ✅ `cargo test --workspace` pass on loop/fw-020-mainloop |

### Parallel batch — merged slices

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-020 Main Loop](046a7c38-fb96-4add-ac24-a6957906cd53) | FW-020 | `7c752b4` | go-around latch from landing_request_go_around → **62%** |
| [FW-020 Main Loop](046a7c38-fb96-4add-ac24-a6957906cd53) | FW-020 | `50d0ac3` | L1/TECS nav feed into stabilize → **58%** |
| [FW-011 INS SITL](4f32c77a-4156-44ca-9c48-cc9f0717daa3) | FW-011 | `bfb23d7` | dynamic notch throttle/RPM tracking → **85%** |
| [FW-008 AHRS](b590846f-5036-40fa-a0ed-a7100bc4780d) | FW-008 | `f4c3ac7` | WindEstimator in DCM drift path → **60%** |

## Near-done / critical path

| Ticket | % | Next |
|--------|---|------|
| **FW-029** | 100 | Done |
| **FW-004** | 100 | Done |
| **FW-011** | 85 | SITL temp cal param binding |
| **FW-008** | 60 | EKF backend selection stub |
| **FW-020** | 62 | Mode table wiring for scheduler |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | 7c752b4 | FW-020 | go_around_hookup latches commanded_go_around |
| 2026-08-27 | f4c3ac7 | FW-008 | full wind wiring (cherry-pick c16189d) |
| 2026-08-27 | 50d0ac3 | FW-020 | nav_tecs_hookup (recovered) |
| 2026-08-27 | 6c910d6 | FW-020 | landing_servo_hookup in set_servos |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One slice on highest-priority near-done or critical-path ticket
3. `cargo test --workspace`, commit, update tracker + this file
4. Post STATUS
