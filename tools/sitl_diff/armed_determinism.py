#!/usr/bin/env python3
"""FW-007 slice 2: is SITL deterministic through an ARMED flight?

Slice 1 showed the idle-on-ground case is exactly reproducible. That is the
easy case: nothing is happening. This exercises the hard one - armed, flying,
with the control loops closed and the EKF running.

There is a complication the idle test did not have. Commands (set mode, arm)
are sent over MAVLink at WALL-CLOCK times, so the simulated instant at which
they land varies between runs. Any trajectory difference then has two possible
causes:

  (a) SITL is not deterministic under load, or
  (b) the inputs were not identical, because the arm landed at a different
      simulated time

Those are distinguished by aligning both runs on the arm event and comparing
elapsed-since-arm rather than absolute simulated time. If the aligned
trajectories match, SITL is deterministic and the harness simply has to
guarantee identical input timing - which a pre-loaded mission can do.
"""
import argparse
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import DFReader, mavutil

ROOT = Path("/srv/ardumaster")
BIN = ROOT / "upstream/plane-4.7.0/build/sitl/bin/arduplane"
WORK = ROOT / "reference/armed"
HOME_LOC = "-35.363261,149.165230,584,353"
PORT = 5760
PLANE_MODE_TAKEOFF = 13

SERIES = {
    "ATT": ["Roll", "Pitch", "Yaw"],
    "POS": ["Lat", "Lng", "Alt"],
}


def run_once(rundir: Path, fly_secs: float, speedup: int) -> Path | None:
    rundir.mkdir(parents=True, exist_ok=True)
    params = rundir / "params.parm"
    params.write_text(
        "\n".join(
            [
                "LOG_DISARMED 1",
                "LOG_REPLAY 0",
                # remove pre-arm gating so arming happens promptly and
                # identically rather than after a variable number of checks
                "ARMING_CHECK 0",
                "SIM_SPEEDUP {}".format(speedup),
                # --wipe erases the EEPROM including accelerometer
                # calibration, and pre-arm then blocks with "3D Accel
                # calibration needed". Marking the accels calibrated is what
                # upstream autotest does; --wipe stays because a clean EEPROM
                # is what makes the two runs start from identical state.
                "INS_ACCOFFS_X 0.001",
                "INS_ACCOFFS_Y 0.001",
                "INS_ACCOFFS_Z 0.001",
                "INS_ACCSCAL_X 1.001",
                "INS_ACCSCAL_Y 1.001",
                "INS_ACCSCAL_Z 1.001",
                "INS_ACC2OFFS_X 0.001",
                "INS_ACC2OFFS_Y 0.001",
                "INS_ACC2OFFS_Z 0.001",
                "INS_ACC2SCAL_X 1.001",
                "INS_ACC2SCAL_Y 1.001",
                "INS_ACC2SCAL_Z 1.001",
                "",
            ]
        )
    )

    cmd = [
        str(BIN),
        "--model", "plane",
        "--home", HOME_LOC,
        "--speedup", str(speedup),
        "--defaults", str(params),
        "--wipe",
    ]
    with open(rundir / "sitl.out", "w") as out, open(rundir / "sitl.err", "w") as err:
        proc = subprocess.Popen(cmd, cwd=str(rundir), stdout=out, stderr=err)
        try:
            # wait for the port to accept
            for _ in range(60):
                try:
                    socket.create_connection(("127.0.0.1", PORT), timeout=1.0).close()
                    break
                except OSError:
                    time.sleep(0.25)

            m = mavutil.mavlink_connection("tcp:127.0.0.1:{}".format(PORT))
            m.wait_heartbeat(timeout=30)
            print("    heartbeat: sys {}".format(m.target_system))

            # set_mode() with a raw number silently no-ops on plane; use the
            # vehicle-reported mode mapping instead
            mapping = m.mode_mapping() or {}
            mode_id = mapping.get("TAKEOFF", PLANE_MODE_TAKEOFF)
            m.mav.set_mode_send(
                m.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            print("    mode -> TAKEOFF ({})".format(mode_id))
            time.sleep(1.0)
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
            print("    arm ack: {}".format(ack.result if ack else "none"))

            time.sleep(fly_secs)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()

    logs = sorted(rundir.rglob("*.BIN"))
    return logs[0] if logs else None


def load_series(path: Path, mtype: str, fields):
    log = DFReader.DFReader_binary(str(path))
    out = {}
    while True:
        msg = log.recv_match(type=mtype)
        if msg is None:
            break
        d = msg.to_dict()
        try:
            out[int(d["TimeUS"])] = tuple(float(d[f]) for f in fields)
        except (KeyError, TypeError, ValueError):
            continue
    return out


def find_arm_time(path: Path):
    """Simulated time the vehicle entered TAKEOFF mode.

    Used as the alignment event rather than an EV id: this log has only one
    EV record and it is not the arm event, whereas the MODE change to TAKEOFF
    is exactly the externally-issued command whose timing we are trying to
    factor out.
    """
    log = DFReader.DFReader_binary(str(path))
    while True:
        msg = log.recv_match(type="MODE")
        if msg is None:
            return None
        d = msg.to_dict()
        if int(d.get("ModeNum", -1)) == PLANE_MODE_TAKEOFF:
            return int(d["TimeUS"])


def compare_aligned(p1: Path, p2: Path):
    t1, t2 = find_arm_time(p1), find_arm_time(p2)
    print("\n=== arm event (simulated time) ===")
    print("  run1: {}".format("{:,} us".format(t1) if t1 else "NOT FOUND"))
    print("  run2: {}".format("{:,} us".format(t2) if t2 else "NOT FOUND"))
    if t1 is None or t2 is None:
        print("  cannot align without an arm event in both runs")
        return
    print("  input timing skew: {:,} us ({:.3f} s of simulated time)".format(
        abs(t1 - t2), abs(t1 - t2) / 1e6))

    for mtype, fields in SERIES.items():
        a, b = load_series(p1, mtype, fields), load_series(p2, mtype, fields)
        # re-key on elapsed-since-arm so identical inputs line up
        ra = {k - t1: v for k, v in a.items() if k >= t1}
        rb = {k - t2: v for k, v in b.items() if k >= t2}
        shared = sorted(set(ra) & set(rb))

        print("\n--- {} aligned on arm ---".format(mtype))
        print("  post-arm samples: run1={:,} run2={:,} shared={:,}".format(
            len(ra), len(rb), len(shared)))
        if not shared:
            print("  NO shared post-arm instants - the runs do not sample the")
            print("  same offsets from arm, so pointwise comparison fails")
            continue

        worst = [0.0] * len(fields)
        for t in shared:
            for i in range(len(fields)):
                d = abs(ra[t][i] - rb[t][i])
                if d > worst[i]:
                    worst[i] = d
        for i, f in enumerate(fields):
            print("  max |delta| {:<5}: {:.6g}".format(f, worst[i]))

        # also report absolute-time comparison, to show whether the skew alone
        # explains any difference
        shared_abs = sorted(set(a) & set(b))
        if shared_abs:
            worst_abs = 0.0
            for t in shared_abs:
                for i in range(len(fields)):
                    worst_abs = max(worst_abs, abs(a[t][i] - b[t][i]))
            print("  (unaligned, absolute time) max |delta| any field: {:.6g}".format(
                worst_abs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fly", type=float, default=30.0, help="wall seconds after arm")
    ap.add_argument("--speedup", type=int, default=10)
    args = ap.parse_args()

    if not BIN.exists():
        sys.exit("SITL binary not built")
    if WORK.exists():
        shutil.rmtree(WORK)

    logs = []
    for i in (1, 2):
        print("=== armed run {} ===".format(i))
        log = run_once(WORK / "run{}".format(i), args.fly, args.speedup)
        if log is None:
            print("  NO LOG")
            rd = WORK / "run{}".format(i)
            print("  " + "\n  ".join((rd / "sitl.out").read_text().splitlines()[-10:]))
            sys.exit(2)
        print("    log: {:,} bytes".format(log.stat().st_size))
        logs.append(log)

    compare_aligned(logs[0], logs[1])


if __name__ == "__main__":
    main()
