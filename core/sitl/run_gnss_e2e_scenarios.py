#!/usr/bin/env python3
"""Flies ArduCopter SITL against core's real GNSS detector and autonomy
together, under issue #19's three spoof scenarios plus an unspoofed control
run.

Usage: run_gnss_e2e_scenarios.py --ardupilot <repo> [--core-main <binary>] [scenario ...]

Unlike run_autonomy_scenario.py, which hand-types trust scores into
autonomy_runner, this runs core_main itself - the real nav detector (#10) and
autonomy (#11), wired together since issue #19 phase 1 - against a GPS fix
that SITL actually glitches via SIM_GPS_GLITCH_X/Y. The detector sees a real
spoofed fix, and the EKF is genuinely fooled, rather than just being told
telemetry happens to be numerically "spoofed".

Assumes no other SITL instance 0 is running (same assumption
run_autonomy_scenario.py makes): it binds the default ports 5760/5501/5502.
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

EARTH_RADIUS_M = 6371000.0
COPTER_MODES = {"GUIDED": 4}
TAKEOFF_ALT_M = 20
AUTONOMY_PREFIX = "AUTONOMY "
NAV_TRUST_DETECT_THRESHOLD = 0.5  # "detected" once TRUST_NAV drops below this

# The ramp rates issue #19 asks for: the fast one is the straight-line ramp
# already used informally, about 3x the 1.5 m/s that #11's scenario (and the
# issue) treats as a realistic spoof rate.
FAST_RAMP_MS = 4.5
SLOW_RAMP_MS = 1.5

# Random-route spoof: a direct port of dashboard/internal/sim/spoof.go's
# updateSpoof/pickLeg, so the false track looks like a believable flight of
# its own rather than a straight line. CRUISE_SPEED_MS matches that file's
# model.CruiseSpeedMS.
CRUISE_SPEED_MS = 8.0
TURN_DEG_PER_S = 30.0
MIN_LEG_M = 60.0
MAX_LEG_M = 200.0
MAX_TURN_DEG = 120.0
FIRST_TURN_DEG = 40.0
MIN_APART_M = 50.0
AREA_MARGIN_M = 40.0
LEG_TRIES = 30
ARRIVE_RADIUS_M = 3.0


class ScenarioFailed(Exception):
    pass


# --- Geometry, matching dashboard/internal/model/plan.go exactly (flat,
# equirectangular approximation, accurate at the scale of a single flight). ---


def distance_m(a, b):
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1]) * math.cos(math.radians(a[0]))
    return 2 * EARTH_RADIUS_M * math.asin(min(1, math.hypot(math.sin(dlat / 2), math.sin(dlon / 2))))


def to_local(origin, p):
    """p's position in metres east and north of origin."""
    east = math.radians(p[1] - origin[1]) * EARTH_RADIUS_M * math.cos(math.radians(origin[0]))
    north = math.radians(p[0] - origin[0]) * EARTH_RADIUS_M
    return east, north


def from_local(origin, east_m, north_m):
    return (
        origin[0] + math.degrees(north_m / EARTH_RADIUS_M),
        origin[1] + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin[0])))),
    )


class RampSpoof:
    """A straight-line ramp, fixed eastward, at a constant rate."""

    def __init__(self, rate_ms):
        self.rate_ms = rate_ms
        self.offset_m = 0.0

    def step(self, dt, true_pos):
        self.offset_m += self.rate_ms * dt
        return from_local(true_pos, self.offset_m, 0.0)


class RandomRouteSpoof:
    """Ported from dashboard/internal/sim/spoof.go's updateSpoof/pickLeg: flies
    its own route of random straight legs joined by limited-rate turns at
    cruise speed, staying inside a box around home and away from the real
    vehicle, so that on its own it looks like a believable flight."""

    def __init__(self, home, true_pos, heading_deg, rng):
        self.home = home
        self.rng = rng
        self.pos = true_pos
        self.heading_deg = heading_deg
        self.target = None
        self._pick_leg(FIRST_TURN_DEG, true_pos)

    def _box(self):
        m = AREA_MARGIN_M
        return -m, m, -m, m  # min_e, max_e, min_n, max_n, around home

    def _in_box(self, p):
        e, n = to_local(self.home, p)
        min_e, max_e, min_n, max_n = self._box()
        return min_e <= e <= max_e and min_n <= n <= max_n

    def _pick_leg(self, max_turn_deg, true_pos):
        for _ in range(LEG_TRIES):
            turn = (self.rng.random() * 2 - 1) * max_turn_deg
            leg = MIN_LEG_M + self.rng.random() * (MAX_LEG_M - MIN_LEG_M)
            heading = math.radians(self.heading_deg + turn)
            candidate = from_local(self.pos, math.sin(heading) * leg, math.cos(heading) * leg)
            if self._in_box(candidate) and distance_m(candidate, true_pos) >= MIN_APART_M:
                self.target = candidate
                return

        # No forward leg fits, so turn back to anywhere in the area.
        min_e, max_e, min_n, max_n = self._box()
        for _ in range(LEG_TRIES):
            candidate = from_local(
                self.home, min_e + self.rng.random() * (max_e - min_e), min_n + self.rng.random() * (max_n - min_n)
            )
            if distance_m(self.pos, candidate) >= MIN_LEG_M and distance_m(candidate, true_pos) >= MIN_APART_M:
                self.target = candidate
                return
        self.target = self.pos  # degenerate: no leg fit at all

    def step(self, dt, true_pos):
        if distance_m(self.pos, self.target) < ARRIVE_RADIUS_M + CRUISE_SPEED_MS * dt:
            self._pick_leg(MAX_TURN_DEG, true_pos)

        east, north = to_local(self.pos, self.target)
        wanted_deg = math.degrees(math.atan2(east, north))
        diff = ((wanted_deg - self.heading_deg + 180) % 360) - 180
        max_turn = TURN_DEG_PER_S * dt
        diff = max(-max_turn, min(max_turn, diff))
        self.heading_deg = (self.heading_deg + diff) % 360

        step_m = CRUISE_SPEED_MS * dt
        rad = math.radians(self.heading_deg)
        self.pos = from_local(self.pos, math.sin(rad) * step_m, math.cos(rad) * step_m)
        return self.pos


class Sitl:
    def __init__(self, ardupilot, core_main, workdir, core_port):
        self.core_port = core_port
        self.core_log = open(workdir / "core_main.log", "w")
        self.core_proc = subprocess.Popen(
            [str(core_main), str(core_port)], cwd=workdir, stdout=self.core_log, stderr=subprocess.STDOUT
        )
        time.sleep(1)  # let core_main bind before SITL's serial1 tries to reach it

        self.proc = subprocess.Popen(
            [
                str(ardupilot / "build/sitl/bin/arducopter"),
                "--model", "+",
                "--wipe",
                "--defaults", str(ardupilot / "Tools/autotest/default_params/copter.parm"),
                f"--serial1=udpclient:127.0.0.1:{core_port}",
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
        self.status = {
            "mode": None,
            "armed": False,
            "rel_alt": 0.0,
            "true_pos": None,
            "heading_deg": 0.0,
            "nav_trust": 1.0,
            "behavior": "NOMINAL",
            "behavior_log": [],  # (monotonic time, behavior) for each change
        }

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
            self.status["heading_deg"] = msg.hdg / 100 if msg.hdg != 65535 else self.status["heading_deg"]
        elif kind == "SIMSTATE":
            self.status["true_pos"] = (msg.lat / 1e7, msg.lng / 1e7)
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "TRUST_NAV":
            self.status["nav_trust"] = msg.value
        elif kind == "STATUSTEXT":
            print(f"  [sitl] {msg.text}")
            if msg.text.startswith(AUTONOMY_PREFIX):
                behavior = msg.text[len(AUTONOMY_PREFIX):].split(":", 1)[0]
                if behavior != self.status["behavior"]:
                    self.status["behavior"] = behavior
                    self.status["behavior_log"].append((time.monotonic(), behavior))

    def wait_for(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.2)
            if condition(self.status):
                return
        raise ScenarioFailed(f"timed out waiting for {what}; last status {self.status}")

    def wait_for_soft(self, condition, timeout, what):
        """Like wait_for, but reports rather than fails when the condition never
        holds - used between scenarios, where a slow recovery is informative, not
        a reason to abandon the remaining scenarios."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.2)
            if condition(self.status):
                return True
        print(f"  note: {what} didn't happen within {timeout}s; continuing anyway")
        return False

    def command(self, command, *params):
        params = list(params) + [0] * (7 - len(params))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, command, 0, *params)

    def set_gps_glitch(self, lat_offset_deg, lon_offset_deg):
        # Renamed between ArduPilot releases; each version ignores the other's.
        for name, value in (
            ("SIM_GPS_GLITCH_X", lat_offset_deg),
            ("SIM_GPS_GLITCH_Y", lon_offset_deg),
            ("SIM_GPS1_GLTCH_X", lat_offset_deg),
            ("SIM_GPS1_GLTCH_Y", lon_offset_deg),
        ):
            self.mav.mav.param_set_send(
                self.mav.target_system, self.mav.target_component, name.encode(), value,
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
        for proc, timeout in ((self.proc, 10), (self.core_proc, 10)):
            proc.terminate()
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                proc.kill()


def expect(condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'} {message}")
    return condition


def run_window(sitl, duration_s, step_fn, label):
    """Runs the GPS at whatever offset step_fn(dt, true_pos) -> (fake_lat, fake_lon)
    says, or unspoofed if step_fn is None, and reports what happened."""
    print(f"\n{label}")
    home = sitl.status["true_pos"]
    log_start = len(sitl.status["behavior_log"])
    furthest = 0.0
    lowest_trust = sitl.status["nav_trust"]
    detect_time_s = None
    dt = 0.5
    began = time.monotonic()
    while (elapsed := time.monotonic() - began) < duration_s:
        true_pos = sitl.status["true_pos"]
        if step_fn is not None:
            fake_lat, fake_lon = step_fn(dt, true_pos)
            sitl.set_gps_glitch(fake_lat - true_pos[0], fake_lon - true_pos[1])
        sitl.pump(dt)
        furthest = max(furthest, distance_m(home, sitl.status["true_pos"]))
        lowest_trust = min(lowest_trust, sitl.status["nav_trust"])
        if detect_time_s is None and sitl.status["nav_trust"] < NAV_TRUST_DETECT_THRESHOLD:
            detect_time_s = elapsed

    if step_fn is not None:
        sitl.set_gps_glitch(0.0, 0.0)
    sitl.pump(2)

    behaviors = sitl.status["behavior_log"][log_start:]
    result = {
        "label": label,
        "duration_s": duration_s,
        "max_true_drift_m": round(furthest, 1),
        "lowest_nav_trust": round(lowest_trust, 3),
        "detect_time_s": round(detect_time_s, 1) if detect_time_s is not None else None,
        "behaviors_entered": [b for _, b in behaviors],
        "time_to_first_behavior_s": round(behaviors[0][0] - began, 1) if behaviors else None,
    }
    print(
        f"  max true drift: {result['max_true_drift_m']} m, lowest TRUST_NAV: {result['lowest_nav_trust']}, "
        f"detected at: {result['detect_time_s']}s, behaviors: {result['behaviors_entered'] or 'none'}"
    )
    return result


SCENARIOS = {
    "fast": lambda sitl, rng, duration: run_window(sitl, duration, RampSpoof(FAST_RAMP_MS).step, "fast ramp (~4.5 m/s)"),
    "slow": lambda sitl, rng, duration: run_window(sitl, duration, RampSpoof(SLOW_RAMP_MS).step, "slow ramp (~1.5 m/s)"),
    "random": lambda sitl, rng, duration: run_window(
        sitl, duration,
        RandomRouteSpoof(sitl.status["true_pos"], sitl.status["true_pos"], sitl.status["heading_deg"], rng).step,
        "random route",
    ),
    "control": lambda sitl, rng, duration: run_window(sitl, duration, None, "control (unspoofed)"),
}


def dashboard_snapshot(url):
    try:
        with urllib.request.urlopen(f"{url.rstrip('/')}/api/status", timeout=3) as resp:
            return json.loads(resp.read())
    except Exception as err:  # noqa: BLE001 - this is best-effort, not a hard dependency
        return {"error": str(err)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ardupilot", type=Path, required=True, help="ArduPilot repo with ./waf copter built for sitl")
    parser.add_argument(
        "--core-main", type=Path, default=Path(__file__).resolve().parent.parent / "build/core_main",
        help="built core_main binary (default: core/build/core_main)",
    )
    parser.add_argument("--core-port", type=int, default=14555, help="UDP port core_main listens on")
    parser.add_argument("--duration", type=float, default=60.0, help="seconds per scenario")
    parser.add_argument("--seed", type=int, default=1, help="RNG seed for the random-route scenario, for repeatability")
    parser.add_argument("--log-dir", type=Path, help="keep logs here instead of a temp directory")
    parser.add_argument("--dashboard-url", help="also record a GET /api/status snapshot from a running dashboard")
    parser.add_argument("scenarios", nargs="*", choices=[*SCENARIOS, "all"], default=["all"])
    args = parser.parse_args()

    scenarios = list(SCENARIOS) if "all" in args.scenarios else args.scenarios

    workdir = args.log_dir or Path(tempfile.mkdtemp(prefix="gnss_e2e_"))
    workdir.mkdir(parents=True, exist_ok=True)
    print(f"logs in {workdir}")

    sitl = Sitl(args.ardupilot.resolve(), args.core_main.resolve(), workdir, args.core_port)
    results = []
    try:
        print("taking off")
        sitl.take_off()

        for name in scenarios:
            rng = random.Random(args.seed)
            result = SCENARIOS[name](sitl, rng, args.duration)
            if args.dashboard_url:
                result["dashboard_status"] = dashboard_snapshot(args.dashboard_url)
            results.append(result)
            # Give trust a chance to recover to NOMINAL before the next scenario,
            # so each one starts from a clean baseline. Not recovering in time is
            # itself worth recording, not a reason to abandon the remaining runs.
            sitl.wait_for_soft(lambda s: s["nav_trust"] > 0.9, 30, "trust recovering to NOMINAL")

        print("\n=== summary ===")
        no_false_alarm = True
        for r in results:
            print(f"{r['label']}: {json.dumps(r)}")
            if r["label"].startswith("control") and (r["behaviors_entered"] or r["lowest_nav_trust"] < NAV_TRUST_DETECT_THRESHOLD):
                no_false_alarm = False
        if "control" in scenarios:
            expect(no_false_alarm, "no false alarm during the unspoofed control run")

        results_path = workdir / "results.json"
        results_path.write_text(json.dumps(results, indent=2))
        print(f"\nfull results written to {results_path}")
        return 0
    except ScenarioFailed as err:
        print(f"\nFAIL: {err}")
        return 1
    finally:
        sitl.close()


if __name__ == "__main__":
    sys.exit(main())
