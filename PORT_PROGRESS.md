# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-26 22:58  
**Server:** `ssh root@172.233.192.152` → `/srv/ardumaster/ports/plane-fw-rust`  
**Loop:** `Tools/loop-ardumaster.ps1` — event-driven (signal on completion)  
**Main:** `283da68`

## STATUS (latest)

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~30.2% |
| **Last commit** | `283da68` |
| **Tests** | ✅ workspace pass on main |

### Parallel agents — merged

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| [FW-011 INS SITL](aae79cf2-4831-4495-bbbe-71ca8b3a3dca) | FW-011 | `fb4ef6e` | Temperature model + fail masks → **20%** |
| [FW-029 Deepstall](e0d210bf-d2bd-4812-9a80-3b17aa23e88c) | FW-029 | `942d132` | `DeepstallStage` enum + query helpers → **78%** |
| [FW-004 Convert](608f9b03-6b92-44d8-ae8f-fba729de0fde) | FW-004 | `edc9f9c`+`888c3fa` | `convert_class` descriptor migration → **88%** |

## Near-done tickets

| Ticket | % | Next |
|--------|---|------|
| **FW-004** | 88 | Plane conversion tables wired to descriptors |
| **FW-029** | 82 | Wire geometry into landing controller; override_servos HAL |
| **FW-018** | 82 | RC/HAL/save_trim (blocked by FW-019) |
| **FW-011** | 20 | Wire noise into SitlImuBackend; board trim |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-26 | 283da68 | FW-029 | override_servos steering gate + travel limit |
| 2026-08-26 | 6a616c8 | FW-011 | sitl_apply_gyro_noise |
| 2026-08-26 | 54e2397 | FW-004 | migrate_parameters batch table |
| 2026-08-26 | 677218f | FW-029 | deepstall_build_approach_path |
| 2026-08-26 | da81ae0 | FW-004 | convert_class test fix merged |
| 2026-08-26 | 0f2e57c | FW-004 | convert_class descriptor slice |
| 2026-08-26 | 9b1726d | FW-029 | deepstall stage machine merged |
| 2026-08-26 | fb4ef6e | FW-011 | SITL INS temperature + fail masks |
| 2026-08-26 | 47341ca | FW-029 | deepstall L1 steering |

## Loop iteration checklist

1. Read this file + `tracker.py status -v` on server
2. One slice on highest-priority near-done or critical-path ticket
3. `cargo test --workspace`, commit, update tracker + this file
4. Post STATUS
