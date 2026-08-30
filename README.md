# ardumaster-rust

Rust port of ArduPilot vehicle firmware (Plane, Copter, and QuadPlane/VTOL), from
ArduPilot `Plane-4.7.0` / matching Copter tag. Tracked as the `fw-rust`, `copter-rust`,
and `vtol-rust` efforts.

## Where things are

| | |
|---|---|
| This port | `/srv/ardumaster/ports/ardumaster-rust` |
| Upstream source (pinned) | `/srv/ardumaster/upstream/plane-4.7.0` |
| Upstream full clone | `/srv/ardumaster/upstream/ardupilot` |
| Tickets, ADRs, status | `/srv/ardumaster/tracker` |
| Golden reference logs | `/srv/ardumaster/reference/plane-4.7.0` |

**Port against the pinned worktree, never against the full clone** — that one tracks
upstream `master`, which keeps moving. See ADR-0001.

## Crate layout

Crates mirror upstream library boundaries, so tickets map onto crates one-to-one.

| Crate | Upstream | Ticket |
|---|---|---|
| `ap-hal` | `libraries/AP_HAL` | FW-001 |
| `ap-math` | `libraries/AP_Math` | FW-002 |

More crates land as their tickets start. Check the tracker before adding one:

```bash
python3 /srv/ardumaster/tracker/tracker.py status -v
```

## Rules that are not negotiable

These come from the ADRs in `/srv/ardumaster/tracker/decisions/`. They exist to keep the port
verifiable, not to impose Rust taste.

1. **Behavioral equivalence beats improvement.** Where upstream returns `false` and the caller
   ignores it, reproduce that. Fixing upstream bugs is a separate ticket — never a side
   effect of porting. (ADR-0003)
2. **`done` means verified, not compiled.** Every ticket declares a `verification` method and
   must actually have run it. (ADR-0003)
3. **No exceptions, no unwinding.** `Result` + `panic = "abort"`. Upstream builds
   `-fno-exceptions`. (ADR-0004)
4. **`no_std` on the flight path.** No allocator. This is how upstream compiles for ChibiOS
   today, not speculative embedded preparation. (ADR-0004)
5. **Precision is the `ekf-double` feature, not a generic.** Mirrors upstream's global
   `HAL_WITH_EKF_DOUBLE`. A generic would allow builds upstream cannot produce, leaving
   states with nothing to compare against. (ADR-0004)
6. **No singletons.** Upstream's 114 `AP::foo()` globals are not reproduced; subsystem
   references go in an explicit context struct. This is the one place call sites diverge
   from upstream on purpose. (ADR-0004)
7. **Bit-exact float parity is not a goal.** Upstream is already not IEEE-strict. Compare
   trajectories within per-ticket tolerances, recorded in the ticket. (ADR-0004)

## Build

```bash
cargo build --workspace
```

```bash
cargo test --workspace
```

```bash
cargo build --workspace --features ap-math/ekf-double
```
