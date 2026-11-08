#!/usr/bin/env python3
"""Flies ArduCopter SITL, injects an attack, and checks autonomy's response.

Usage: run_autonomy_scenario.py --ardupilot <repo> --runner <autonomy_runner> {gnss,link,multi}

The nav and link detectors aren't merged yet, so the verdict they would reach is
typed into autonomy_runner directly. The GNSS scenario also ramps a real GPS
offset inside SITL, so the failover is tested against an actually spoofed fix.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

COPTER_MODES = {"GUIDED": 4, "RTL": 6, "LAND": 9, "BRAKE": 17}
TAKEOFF_ALT_M = 20
GLITCH_DEG = 0.001  # about 110 m of false position, as a sudden jump
# A real spoofer drags the position slowly to stay inside the EKF's innovation
# gates. A sudden jump gets rejected without any help from us, and proves nothing.
SPOOF_RATE_DEG_PER_S = 0.0000135  # about 1.5 m/s
SPOOF_DURATION_S = 30  # about 45 m of false position by the end
DETECTION_DELAY_S = 5  # how long the spoof runs before the detector's verdict
MAX_TRUE_DRIFT_M = 20  # how far the real vehicle may be dragged
RUNNER_PORT = 14551


class ScenarioFailed(Exception):
    pass


class Sitl:
    def __init__(self, ardupilot, workdir):
        self.proc = subprocess.Popen(
            [
                str(ardupilot / "build/sitl/bin/arducopter"),
                "--model", "+",
                "--wipe",
                "--defaults",
                f"{ardupilot / 'Tools/autotest/default_params/copter.parm'},"
                f"{Path(__file__).parent / 'autonomy_failover.parm'}",
                f"--serial1=udpclient:127.0.0.1:{RUNNER_PORT}",
            ],
            cwd=workdir,
            stdout=open(workdir / "arducopter.log", "w"),
            stderr=subprocess.STDOUT,
        )
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", retries=60, dialect="ardupilotmega")
        self.mav.wait_heartbeat(timeout=30)
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
        )
        self.status = {"mode": None, "armed": False, "rel_alt": 0.0, "true_pos": None}

    def pump(self, seconds=0.0):
        """Reads telemetry for the given time, keeping self.status current."""
        deadline = time.monotonic() + seconds
        while True:
            msg = self.mav.recv_match(blocking=seconds > 0, timeout=0.1)
            if msg is not None:
                self._note(msg)
            elif time.monotonic() >= deadline:
                return
            if seconds > 0 and time.monotonic() >= deadline:
                return

    def _note(self, msg):
        kind = msg.get_type()
        if kind == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            self.status["mode"] = msg.custom_mode
            self.status["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "GLOBAL_POSITION_INT":
            self.status["rel_alt"] = msg.relative_alt / 1000
        elif kind == "SIMSTATE":
            self.status["true_pos"] = (msg.lat / 1e7, msg.lng / 1e7)
        elif kind == "STATUSTEXT":
            print(f"  [sitl] {msg.text}")

    def wait_for(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.2)
            if condition(self.status):
                return
        raise ScenarioFailed(f"timed out waiting for {what}; last status {self.status}")

    def command(self, command, *params):
        params = list(params) + [0] * (7 - len(params))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, command, 0, *params)

    def inject_gps_glitch(self, degrees=GLITCH_DEG):
        # Renamed between ArduPilot releases; each version ignores the other's.
        for name in ("SIM_GPS_GLITCH_X", "SIM_GPS1_GLTCH_X"):
            self.mav.mav.param_set_send(
                self.mav.target_system, self.mav.target_component, name.encode(), degrees,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

    def take_off(self):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, COPTER_MODES["GUIDED"])
        deadline = time.monotonic() + 120
        while not self.status["armed"]:  # arming is refused until the EKF has settled
            if time.monotonic() > deadline:
                raise ScenarioFailed("vehicle never armed")
            self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, COPTER_MODES["GUIDED"])
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.pump(2)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
        self.wait_for(lambda s: s["rel_alt"] > TAKEOFF_ALT_M - 1, 60, "takeoff altitude")
        self.pump(5)  # let it settle

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=10)


class Runner:
    def __init__(self, path, workdir):
        self.log_path = workdir / "transitions.csv"
        self.stderr_path = workdir / "runner.log"
        self.proc = subprocess.Popen(
            [str(path), str(RUNNER_PORT), str(self.log_path)],
            stdin=subprocess.PIPE,
            stderr=open(self.stderr_path, "w"),
            text=True,
        )

    def report(self, **scores):
        for name, value in scores.items():
            self.proc.stdin.write(f"{name} {value}\n")
        self.proc.stdin.flush()

    def transitions(self):
        lines = self.log_path.read_text().splitlines()[1:]
        return [(row[1], row[2]) for row in (line.split(",") for line in lines)]

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=10)


def distance_m(a, b):
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1]) * math.cos(math.radians(a[0]))
    return 6371000 * math.hypot(dlat, dlon)


def expect(condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'} {message}")
    if not condition:
        raise ScenarioFailed(message)


def scenario_gnss(sitl, runner):
    start = sitl.status["true_pos"]
    expect(start is not None, "SIMSTATE ground truth available")

    print("injecting slowly ramping GPS spoof")
    furthest = 0.0
    began = time.monotonic()
    reported = False
    while (elapsed := time.monotonic() - began) < SPOOF_DURATION_S:
        sitl.inject_gps_glitch(elapsed * SPOOF_RATE_DEG_PER_S)
        if not reported and elapsed >= DETECTION_DELAY_S:
            runner.report(nav=0.05)
            reported = True
        sitl.pump(0.5)
        furthest = max(furthest, distance_m(start, sitl.status["true_pos"]))
    print(f"  max true drift while spoofed: {furthest:.1f} m")

    expect(("NOMINAL", "GNSS_FAILOVER") in runner.transitions(), "switched to GNSS_FAILOVER")
    expect("EKF source set 2 active" in runner.stderr_path.read_text(), "autopilot accepted source set 2")
    expect(sitl.status["armed"] and sitl.status["mode"] == COPTER_MODES["GUIDED"], "still flying in GUIDED")
    expect(sitl.status["rel_alt"] > TAKEOFF_ALT_M / 2, f"held altitude ({sitl.status['rel_alt']:.1f} m)")
    expect(furthest < MAX_TRUE_DRIFT_M, f"true position stayed put (max drift {furthest:.1f} m)")


def scenario_link(sitl, runner):
    print("injecting link jam")
    runner.report(link=0)
    sitl.wait_for(lambda s: s["mode"] == COPTER_MODES["BRAKE"], 5, "BRAKE")
    expect(("NOMINAL", "LINK_HOLD") in runner.transitions(), "entered LINK_HOLD")

    lowest = sitl.status["rel_alt"]
    end = time.monotonic() + 35
    while sitl.status["mode"] != COPTER_MODES["RTL"]:
        if time.monotonic() > end:
            raise ScenarioFailed(f"timed out waiting for RTL; last status {sitl.status}")
        sitl.pump(0.2)
        lowest = min(lowest, sitl.status["rel_alt"])
    expect(lowest > TAKEOFF_ALT_M - 2, f"held altitude with no pilot input (lowest {lowest:.1f} m)")
    expect(("LINK_HOLD", "LINK_RETURN") in runner.transitions(), "escalated to LINK_RETURN")
    sitl.wait_for(lambda s: not s["armed"], 120, "return, landing and disarm")


def scenario_multi(sitl, runner):
    print("injecting GPS spoof and link jam")
    sitl.inject_gps_glitch()
    runner.report(nav=0.05, link=0)
    sitl.wait_for(lambda s: s["mode"] == COPTER_MODES["LAND"], 5, "LAND")
    expect(("NOMINAL", "SAFE_LAND") in runner.transitions(), "entered SAFE_LAND")
    sitl.wait_for(lambda s: not s["armed"], 120, "touchdown and disarm")


SCENARIOS = {"gnss": scenario_gnss, "link": scenario_link, "multi": scenario_multi}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ardupilot", type=Path, required=True, help="ArduPilot repo with ./waf copter built for sitl")
    parser.add_argument("--runner", type=Path, required=True, help="built autonomy_runner")
    parser.add_argument("--log-dir", type=Path, help="keep logs here instead of a temp directory")
    parser.add_argument("scenario", choices=SCENARIOS)
    args = parser.parse_args()

    workdir = args.log_dir or Path(tempfile.mkdtemp(prefix="autonomy_sitl_"))
    workdir.mkdir(parents=True, exist_ok=True)
    print(f"logs in {workdir}")

    runner = Runner(args.runner.resolve(), workdir)
    sitl = Sitl(args.ardupilot.resolve(), workdir)
    try:
        print("taking off")
        sitl.take_off()
        runner.report(nav=1, link=1)
        sitl.pump(2)
        SCENARIOS[args.scenario](sitl, runner)
        print(f"\nPASS {args.scenario}\ntransitions: {runner.transitions()}")
        return 0
    except ScenarioFailed as err:
        print(f"\nFAIL {args.scenario}: {err}\ntransitions: {runner.transitions()}")
        return 1
    finally:
        sitl.close()
        runner.close()


if __name__ == "__main__":
    sys.exit(main())
