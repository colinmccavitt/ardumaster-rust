# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 05:00  
**Main:** `2a3dbfe`

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~54%+ |
| **Last commit** | `2a3dbfe` |
| **Tests** | ✅ `cargo test --workspace` pass on main |

### Parallel batch — merged slices (batch 14)

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-011 INS SITL](1ce6e591-d96e-44d4-9598-e7b01dc70cc6) | FW-011 | `b5b6d1b` | SIM_IMUT temp cal param binding → **88%** |
| [FW-008 AHRS](7768972a-6367-4fd4-8a45-2f560f24f00d) | FW-008 | `9d63804` | AHRS backend selection stub (DCM vs EKF3) → **65%** |
| [FW-020 Main Loop](c1647718-0a7e-4f5d-81b3-9909e4a75f93) | FW-020 | `cf26b2e` | go-around latch in set_servos → **62%** |
| [FW-011 INS SITL](4f32c77a-4156-44ca-9c48-cc9f0717daa3) | FW-011 | `bfb23d7` | dynamic notch throttle/RPM tracking → **85%** |
| [FW-008 AHRS](b590846f-5036-40fa-a0ed-a7100bc4780d) | FW-008 | `f4c3ac7` | WindEstimator in DCM drift path → **60%** |
| [FW-020 Main Loop](046a7c38-fb96-4add-ac24-a6957906cd53) | FW-020 | `50d0ac3` | L1/TECS nav feed into stabilize → **58%** |

## Near-done / critical path

| Ticket | % | Next |
|--------|---|------|
| **FW-029** | 100 | Done |
| **FW-004** | 100 | Done |
<<<<<<< HEAD
| **FW-011** | 88 | SITL file playback hookup |
| **FW-008** | 65 | EKF3 backend wiring |
| **FW-020** | 62 | Mode table wiring for scheduler |
=======
| **FW-011** | 68 | INS→AHRS publish |
| **FW-008** | 65 | GPS lag buffer (ra_delayed) |
| **FW-020** | 15 | Wire AHRS feed, stabilize/set_servos |
>>>>>>> 9ddca0a (FW-008: update progress tracker after backend selection stub)

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | cf26b2e | FW-020 | go_around_hookup latches commanded_go_around |
| 2026-08-27 | b5b6d1b | FW-011 | SIM_IMUT temp cal param binding |
| 2026-08-27 | f4c3ac7 | FW-008 | full wind wiring (cherry-pick c16189d) |
| 2026-08-27 | bfb23d7 | FW-011 | dynamic notch tracking |
| 2026-08-27 | 50d0ac3 | FW-020 | nav_tecs_hookup (recovered) |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One slice on highest-priority near-done or critical-path ticket
3. `cargo test --workspace`, commit, update tracker + this file
4. Post STATUS
