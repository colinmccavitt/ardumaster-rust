# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 05:10  
**Main:** `cbd2970`

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~55%+ |
| **Last commit** | `cbd2970` |
| **Tests** | ✅ `cargo test --workspace` pass on main |

### Parallel batch — merged slices (batch 15)

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-011 INS SITL](d85532cb-20c6-4bcf-82d8-1528ba2b8c03) | FW-011 | `3856042` | SIM file playback params → **90%** |
| [FW-008 AHRS](52216167-c198-4b20-9c48-677586d800ac) | FW-008 | `955e1da` | EKF3 update hook stub → **70%** |
| [FW-020 Main Loop](5e3d53c8-5a88-4f1c-8ce1-e6d022d111a8) | FW-020 | `c4ed79e` | mode_table_hookup in scheduler → **66%** |
| [FW-020 Main Loop](c1647718-0a7e-4f5d-81b3-9909e4a75f93) | FW-020 | `cf26b2e` | go-around latch → **62%** |
| [FW-011 INS SITL](1ce6e591-d96e-44d4-9598-e7b01dc70cc6) | FW-011 | `b5b6d1b` | SIM_IMUT temp cal → **88%** |
| [FW-008 AHRS](7768972a-6367-4fd4-8a45-2f560f24f00d) | FW-008 | `9d63804` | backend selection stub → **65%** |

## Near-done / critical path

| Ticket | % | Next |
|--------|---|------|
| **FW-029** | 100 | Done |
| **FW-004** | 100 | Done |
| **FW-011** | 90 | INS noise / deeper SITL |
| **FW-008** | 70 | GPS lag buffer (ra_delayed) |
| **FW-020** | 66 | Landing loop in scheduler tick |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | c4ed79e | FW-020 | mode_table_hookup dispatch |
| 2026-08-27 | 955e1da | FW-008 | EKF3 update hook stub |
| 2026-08-27 | 3856042 | FW-011 | SIM file playback params |
| 2026-08-27 | cf26b2e | FW-020 | go_around_hookup latch |
| 2026-08-27 | 8e4c71c | batch 14 | full batch 14 merge |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One slice on highest-priority near-done or critical-path ticket
3. `cargo test --workspace`, commit, update tracker + this file
4. Post STATUS
