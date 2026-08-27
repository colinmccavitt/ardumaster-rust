# ArduPilot Rust Port — Progress Tracker

**Updated:** 2026-08-27 05:20  
**Main:** `b66f069`

| Field | Value |
|-------|-------|
| **fw-rust progress** | ~56%+ |
| **Last commit** | `b66f069` |
| **Tests** | ✅ `cargo test --workspace` pass on main |

### Loop complete — critical path targets met

| Ticket | % | Target |
|--------|---|--------|
| **FW-011** | 92 | ≥90 ✅ |
| **FW-008** | 75 | ≥75 ✅ |
| **FW-020** | 70 | ≥70 ✅ |

### Batch 16 merged

| Agent | Ticket | Commit | Result |
|-------|--------|--------|--------|
| FW-011 INS SITL | FW-011 | `19b68b2` | SIM_VIB noise param binding → **92%** |
| FW-008 AHRS | FW-008 | `fd73ed2` | GPS lag buffer ra_delayed → **75%** |
| FW-020 Main Loop | FW-020 | `c26e709` | landing_loop_hookup in scheduler → **70%** |

## Session log (recent)

| When | Commit | Ticket | What |
|------|--------|--------|------|
| 2026-08-27 | c26e709 | FW-020 | landing loop in scheduler tick |
| 2026-08-27 | fd73ed2 | FW-008 | GPS lag buffer wiring |
| 2026-08-27 | 19b68b2 | FW-011 | SIM_VIB noise params |
| 2026-08-27 | be43ba6 | batch 15 | full batch 15 merge |
