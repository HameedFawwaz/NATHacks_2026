#!/usr/bin/env python3
"""Run real ArduCopter SITL, core_main and the dashboard's live MAVLink bridge."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

os.environ["MAVLINK20"] = "1"
try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is missing. Use scripts/sitl with SITL_PYTHON pointing to a Python environment with pymavlink installed.")

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AP = ROOT.parent / "ardupilot-sitl/ardupilot"
CONTROL = "udpin:127.0.0.1:14553"


def log(text):
    print(text, flush=True)


def check_ports():
    # Do not stop or replace an existing dashboard/SITL instance.
    for port, kind in ((8080, socket.SOCK_STREAM), (5760, socket.SOCK_STREAM),
                       (14551, socket.SOCK_DGRAM), (14552, socket.SOCK_DGRAM),
                       (14553, socket.SOCK_DGRAM)):
        with socket.socket(socket.AF_INET, kind) as sock:
            if kind == socket.SOCK_STREAM:
                # A just-stopped HTTP server may leave TIME_WAIT connections.
                # Match the servers' reuse policy without allowing a second listener.
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(("127.0.0.1", port))
                if kind == socket.SOCK_STREAM:
                    sock.listen(1)
            except OSError as exc:
                raise RuntimeError(f"Port {port} is already in use; stop the existing instance first.") from exc


def snapshot():
    with urllib.request.urlopen("http://127.0.0.1:8080/api/telemetry", timeout=5) as response:
        for line in response:
            if line.startswith(b"data: "):
                return json.loads(line[6:])["Snapshot"]
    raise RuntimeError("The live bridge returned no telemetry.")


def run(args):
    ap = args.ardupilot.expanduser().resolve()
    copter = ap / "build/sitl/bin/arducopter"
    if not copter.is_file():
        raise RuntimeError(f"No built ArduCopter SITL at {copter}. Set ARDUPILOT_DIR or pass --ardupilot.")
    check_ports()
    if not args.no_build:
        subprocess.run(["cmake", "-S", str(ROOT / "core"), "-B", str(ROOT / "core/build")], check=True)
        subprocess.run(["cmake", "--build", str(ROOT / "core/build"), "--target", "core_main", "--parallel", "4"], check=True)
        subprocess.run(["make", "build"], cwd=ROOT / "dashboard", check=True)
        subprocess.run(["go", "build", "-o", str(ROOT / "core/build/dashboard-serve"), "./cmd/serve"],
                       cwd=ROOT / "dashboard", check=True)

    runtime = ROOT / "core/build/dashboard-sitl"
    runtime.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="session-", dir=runtime))
    processes = []
    logs = []
    operator = None

    def launch(name, command):
        output = open(work / f"{name}.log", "w")
        logs.append(output)
        process = subprocess.Popen(command, cwd=work, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((name, process))
        return process

    def healthy():
        for name, process in processes:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited ({process.returncode}); see {work / (name + '.log')}")

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        launch("dashboard", [str(ROOT / "core/build/dashboard-serve"), "-dir", str(ROOT / "dashboard/web"),
                             "-mavlink", "127.0.0.1:14552"])
        launch("core", [str(ROOT / "core/build/core_main"), "14551", "--allow-recovery-control", "--gps-free-backup"])
        launch("arducopter", [str(copter), "--model", "+", "--home", "-35.363262,149.165237,584,353",
                              # autonomy_failover.parm: core_main runs autonomy, whose
                              # GNSS failover needs the non-GPS EKF source set.
                              "--defaults", f"{ap / 'Tools/autotest/default_params/copter.parm'},"
                                            f"{ROOT / 'core/sitl/autonomy_failover.parm'},"
                                            f"{ROOT / 'core/sitl/dashboard.parm'}",
                              "--serial0=tcp:5760",
                              "--serial1=udpclient:127.0.0.1:14551",
                              "--serial2=udpclient:127.0.0.1:14552",
                              "--serial4=udpclient:127.0.0.1:14553"])
        log(f"Logs: {work}")
        log("Waiting for ArduPilot, GPS and core telemetry...")
        # This is the primary operator (255). Its real heartbeats let core's
        # lost-link detector monitor this local connection, not a fake score.
        deadline = time.monotonic() + 30
        while True:
            healthy()
            try:
                operator = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Timed out connecting to SITL's operator link.")
                time.sleep(0.2)

        last_heartbeat = 0
        last_check = 0
        ready = False
        deadline = time.monotonic() + 120
        while True:
            healthy()
            now = time.monotonic()
            if now - last_heartbeat >= 1:
                operator.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_heartbeat = now
            operator.recv_match(blocking=True, timeout=0.1)
            if not ready and now - last_check >= 1:
                last_check = now
                try:
                    state = snapshot()
                    ready = (state["Connected"] and state["HasPosition"] and
                             state["Nav"]["AtMs"] > 0 and state["Link"]["AtMs"] > 0)
                except (OSError, ValueError):
                    pass
                if ready:
                    log("Ready: http://127.0.0.1:8080 (LIVE ArduPilot SITL; disarmed)")
                    log("Upload a plan in the dashboard, then run: ./scripts/sitl start")
                    log("Or fly the test square: ./scripts/sitl square")
                    log("Land: ./scripts/sitl land | Status: ./scripts/sitl status | Stop: Ctrl+C")
                elif now >= deadline:
                    raise RuntimeError(f"Live telemetry did not become ready within 120s. See logs in {work}")
    finally:
        # Only our child process groups; never kill unrelated running services.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        if operator is not None:
            operator.close()
        for _, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for _, process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for output in logs:
            output.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        log("SITL, core and dashboard stopped.")


def control(action):
    # This endpoint belongs only to the local SITL launcher, not real hardware.
    master = mavutil.mavlink_connection(CONTROL, source_system=200)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                master.target_system = hb.get_srcSystem()
                master.target_component = hb.get_srcComponent()
                break
        else:
            raise RuntimeError("No SITL heartbeat. Start ./scripts/sitl in another terminal first.")
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if action == "land":
            master.set_mode_apm("LAND")
            desired_mode = 9
        else:
            # Require an uploaded mission; never manufacture one or bypass prearm checks.
            master.mav.mission_request_list_send(master.target_system, master.target_component)
            count = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
            if count is None or count.count < 2:
                raise RuntimeError("Upload a plan from the dashboard before starting a mission.")
            master.set_mode_apm("GUIDED")
            master.arducopter_arm()
            end = time.monotonic() + 30
            while time.monotonic() < end:
                msg = master.recv_match(blocking=True, timeout=1)
                if msg is not None and msg.get_type() == "STATUSTEXT":
                    log(msg.text)
                if master.motors_armed():
                    break
            else:
                raise RuntimeError("Arming was not confirmed. Wait for EKF/prearm readiness and try again.")
            master.set_mode_apm("AUTO")
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
            desired_mode = 3
        end = time.monotonic() + 10
        while time.monotonic() < end:
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.custom_mode == desired_mode:
                log(f"SITL confirmed {'AUTO mission mode' if action == 'start' else 'LAND mode'}.")
                return
        raise RuntimeError("SITL did not confirm the requested mode; check ./scripts/sitl status.")
    finally:
        master.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", default="run", choices=("run", "square", "start", "land", "status"))
    parser.add_argument("--ardupilot", type=Path, default=Path(os.environ.get("ARDUPILOT_DIR", str(DEFAULT_AP))))
    parser.add_argument("--no-build", action="store_true", help="reuse previously built core and dashboard binaries")
    args = parser.parse_args()
    try:
        if args.action == "run":
            run(args)
        elif args.action == "status":
            log(json.dumps(snapshot(), indent=2))
        elif args.action == "square":
            subprocess.run([sys.executable, str(ROOT / "core/sitl/fly_square.py"), "--connect", CONTROL], check=True)
        else:
            control(args.action)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        log(f"Error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
