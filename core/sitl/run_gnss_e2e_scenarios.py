#!/usr/bin/env python3
"""Flies ArduCopter SITL against core's real GNSS detector and autonomy
together, under issue #19's three spoof scenarios plus two unspoofed control
runs (a hover, and a square flown at cruise speed).

Usage: run_gnss_e2e_scenarios.py --ardupilot <repo> [--core-main <binary>] [scenario ...]

Unlike run_autonomy_scenario.py, which hand-types trust scores into
autonomy_runner, this runs core_main itself - the real nav detector (#10) and
autonomy (#11), wired together since issue #19 phase 1 - against a GPS fix
that SITL actually glitches via SIM_GPS_GLITCH_X/Y. The detector sees a real
spoofed fix, and the EKF is genuinely fooled, rather than just being told
telemetry happens to be numerically "spoofed".

Assumes no other SITL is running on the chosen --instance (default 0, which
binds the default ports 5760/5501/5502). Use e.g. --instance 1 --core-port
14565 to run it beside run.sh.

With --esp32 DEVICE, core is the flashed ESP32 on that serial device instead
of core_main: SITL's SERIAL1 is opened on the board, which is reset before
each scenario so every flight starts from a fresh detector and autonomy.
"""
import argparse
import json
import math
import os
import random
import signal
import socket
import statistics
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

# Side of the square the unspoofed "cruise" control flies.
CRUISE_SQUARE_M = 120.0

# Core's EST_N/EST_E (#16): the dashboard's bound, and how close the halves of
# one published pair must arrive.
MAX_ESTIMATE_M = 100_000
EST_PAIR_S = 0.15
# Simulator truth older than this isn't used to score an estimate.
TRUTH_MAX_AGE_S = 0.2
# "imu_gap": how long SITL is frozen, past core's 500 ms IMU freshness limit
# but short of the 3 s staleness checks this script and the dashboard apply.
IMU_GAP_AT_S = 10.0
IMU_GAP_S = 2.0


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
    def __init__(self, ardupilot, core_main, workdir, core_port, instance=0,
                 dashboard_serve=None, dashboard_port=14556, dashboard_http_port=8081, dashboard_url=None,
                 esp32=None, esp32_baud=921600):
        self.esp32, self.esp32_baud = esp32, esp32_baud
        self.proc = self.core_proc = self.dashboard_proc = self.mav = None
        self.logs = []
        self.dashboard_url = dashboard_url or f"http://127.0.0.1:{dashboard_http_port}"
        try:
            self._start(ardupilot, core_main, workdir, core_port, instance,
                        dashboard_serve, dashboard_port, dashboard_http_port, dashboard_url)
        except BaseException:
            self.close()
            raise

    def _launch(self, command, workdir, name):
        output = open(workdir / f"{name}.log", "w")
        self.logs.append(output)
        return subprocess.Popen(command, cwd=workdir, stdout=output, stderr=subprocess.STDOUT)

    def _start(self, ardupilot, core_main, workdir, core_port, instance,
               dashboard_serve, dashboard_port, dashboard_http_port, dashboard_url):
        # Refuse collisions instead of accidentally driving another running SITL.
        ports = [(5760 + 10 * instance, socket.SOCK_STREAM)]
        if not self.esp32:
            ports.append((core_port, socket.SOCK_DGRAM))
        if not dashboard_url:
            ports += [(dashboard_port, socket.SOCK_DGRAM), (dashboard_http_port, socket.SOCK_STREAM)]
        if len(set(ports)) != len(ports):
            raise ScenarioFailed("SITL, core and dashboard must use distinct ports")
        for port, kind in ports:
            with socket.socket(socket.AF_INET, kind) as sock:
                if kind == socket.SOCK_STREAM:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("127.0.0.1", port))
                if kind == socket.SOCK_STREAM:
                    sock.listen(1)
        if not dashboard_url:
            if dashboard_serve is None or not dashboard_serve.is_file():
                raise ScenarioFailed("Build the dashboard bridge first: cd dashboard && go build -o ../core/build/dashboard-serve ./cmd/serve")
            self.dashboard_proc = self._launch(
                [str(dashboard_serve), "-addr", f"127.0.0.1:{dashboard_http_port}",
                 "-mavlink", f"127.0.0.1:{dashboard_port}"], workdir, "dashboard")
        self.core_port = core_port
        if self.esp32:
            reset_esp32(self.esp32)
            self.core_log = None  # the board's UART carries only MAVLink
            serial1 = f"--serial1=uart:{self.esp32}:{self.esp32_baud}"
        else:
            self.core_proc = self._launch([str(core_main), str(core_port)], workdir, "core_main")
            self.core_log = workdir / "core_main.log"
            time.sleep(1)  # let core_main bind before SITL's serial1 tries to reach it
            serial1 = f"--serial1=udpclient:127.0.0.1:{core_port}"

        self.proc = self._launch(
            [
                str(ardupilot / "build/sitl/bin/arducopter"),
                "--model", "+",
                "-I", str(instance),
                "--wipe",
                "--defaults",
                f"{ardupilot / 'Tools/autotest/default_params/copter.parm'},"
                f"{Path(__file__).parent / 'autonomy_failover.parm'}",
                serial1,
                f"--serial2=udpclient:127.0.0.1:{dashboard_port}",
            ],
            workdir, "arducopter",
        )
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760 + 10 * instance}", retries=60, dialect="ardupilotmega")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.mav.target_system, self.mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        else:
            raise ScenarioFailed("no SITL autopilot heartbeat")
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
        )
        # Simulator truth with its own velocity (SIM_STATE) at 20 Hz, to score
        # core's estimate against, and home, which that estimate is relative
        # to, at 1 Hz.
        for msg_id, rate_hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 20),
                                (mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1)):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / rate_hz)
        self.status = {
            "mode": None,
            "armed": False,
            "rel_alt": 0.0,
            "true_pos": None,
            "truth_at": 0.0,
            "heading_deg": 0.0,
            "nav_trust": None,
            "nav_at": 0.0,
            "behavior": "NOMINAL",
            "behavior_log": [],  # (monotonic time, behavior) for each change
            "home": None,         # autopilot HOME_POSITION (lat, lon)
            "gps_pos": None,      # GPS_RAW_INT, spoofed or not
            "sim_truth": None,    # SIM_STATE: (monotonic time, lat, lon, vn, ve), physics state, never EKF
            "est_n": None,
            "est_n_at": 0.0,
            "est_at": 0.0,        # when the last complete EST_N/EST_E pair arrived
            "est_samples": [],    # one per pair: see _note_estimate
            "est_rejects": [],    # pairs that should never have been published
        }
        # ArduCopter's default per-serial-port stream rates are all zero (see
        # AP_MAV_DEFAULT_STREAM_RATE_* in GCS_MAVLink_Parameters.cpp): unlike the
        # primary link above, which just asked for its own streams, SERIAL1 -
        # core_main's own link - sends nothing but HEARTBEAT until something sets
        # its rates. A real companion-computer link would have these configured
        # already; here that's our job, standing in for that one-time setup.
        self._set_stream_rates(instance=1, rate_hz=10)

    def _set_stream_rates(self, instance, rate_hz):
        """Sets every stream-rate group for serial port n, over the primary
        link (these are plain vehicle parameters, not per-channel state, so
        setting them here takes effect on serial1 too). ArduPilot renamed them
        from SRn_* to MAVn_* in 4.7; each version ignores the other's."""
        for group in ("RAW_SENS", "EXT_STAT", "RC_CHAN", "RAW_CTRL", "POSITION", "EXTRA1", "EXTRA2", "EXTRA3", "PARAMS"):
            for name in (f"SR{instance}_{group}", f"MAV{instance}_{group}"):
                self.mav.mav.param_set_send(
                    self.mav.target_system, self.mav.target_component, name.encode(), rate_hz,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
        self.pump(2)  # let the param sets land before anything depends on their streams

    def pump(self, seconds=0.0):
        """Reads telemetry for the given time, keeping self.status current."""
        deadline = time.monotonic() + seconds
        while True:
            for name, proc in (("SITL", self.proc), ("core_main", self.core_proc), ("dashboard", self.dashboard_proc)):
                if proc is not None and proc.poll() is not None:
                    raise ScenarioFailed(f"{name} exited with status {proc.returncode}")
            msg = self.mav.recv_match(blocking=seconds > 0, timeout=0.1)
            if msg is not None:
                self._note(msg)
            elif time.monotonic() >= deadline:
                return
            if seconds > 0 and time.monotonic() >= deadline:
                return

    def _note(self, msg):
        if msg.get_srcSystem() != self.mav.target_system:
            return
        kind = msg.get_type()
        if kind == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            self.status["mode"] = msg.custom_mode
            self.status["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "GLOBAL_POSITION_INT":
            self.status["rel_alt"] = msg.relative_alt / 1000
            self.status["heading_deg"] = msg.hdg / 100 if msg.hdg != 65535 else self.status["heading_deg"]
        elif kind == "SIM_STATE" and (msg.lat_int or msg.lon_int):
            self.status["sim_truth"] = (time.monotonic(), msg.lat_int / 1e7, msg.lon_int / 1e7, msg.vn, msg.ve)
        elif kind == "SIMSTATE":
            self.status["true_pos"] = (msg.lat / 1e7, msg.lng / 1e7)
            self.status["truth_at"] = time.monotonic()
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "TRUST_NAV":
            self.status["nav_trust"] = msg.value
            self.status["nav_at"] = time.monotonic()
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "EST_N":
            self.status["est_n"], self.status["est_n_at"] = msg.value, time.monotonic()
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "EST_E":
            self._note_estimate(msg.value)
        elif kind == "HOME_POSITION" and (msg.latitude or msg.longitude):
            self.status["home"] = (msg.latitude / 1e7, msg.longitude / 1e7)
        elif kind == "GPS_RAW_INT" and msg.fix_type >= 2:
            self.status["gps_pos"] = (msg.lat / 1e7, msg.lon / 1e7)
        elif kind == "STATUSTEXT":
            print(f"  [sitl] {msg.text}")
            if msg.text.startswith(AUTONOMY_PREFIX):
                behavior = msg.text[len(AUTONOMY_PREFIX):].split(":", 1)[0]
                if behavior != self.status["behavior"]:
                    self.status["behavior"] = behavior
                    self.status["behavior_log"].append((time.monotonic(), behavior))

    def _note_estimate(self, est_e):
        """Scores one EST_N/EST_E pair against simulator truth and the GPS fix,
        all as metres north/east of the autopilot's home."""
        now = time.monotonic()
        s = self.status
        est_n = s["est_n"]
        if est_n is None or now - s["est_n_at"] > EST_PAIR_S:
            return  # half a pair
        s["est_n"] = None
        sample = {"t": now, "est_n": est_n, "est_e": est_e}
        if not (abs(est_n) <= MAX_ESTIMATE_M and abs(est_e) <= MAX_ESTIMATE_M) or s["home"] is None:
            s["est_rejects"].append(sample)  # NaN fails the comparison too
            return
        s["est_at"] = now
        truth = s["sim_truth"]
        if truth is not None and now - truth[0] <= TRUTH_MAX_AGE_S:
            # Truth comes only from the simulator's physics state (SIM_STATE),
            # never from the EKF, which a spoof can drag: position plus the
            # simulator's own velocity times the sample's age at arrival.
            # The estimate's own link latency is not removed.
            age = max(now - truth[0], 0.0)
            true_e, true_n = to_local(s["home"], (truth[1], truth[2]))
            true_n += truth[3] * age
            true_e += truth[4] * age
            sample["truth_age_s"] = age
            sample["error_m"] = math.hypot(est_n - true_n, est_e - true_e)
            if s["gps_pos"] is not None:
                gps_e, gps_n = to_local(s["home"], s["gps_pos"])
                sample["gps_truth_m"] = math.hypot(gps_n - true_n, gps_e - true_e)
                sample["gps_est_m"] = math.hypot(gps_n - est_n, gps_e - est_e)
        s["est_samples"].append(sample)

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

    def goto(self, lat, lon, alt_m=TAKEOFF_ALT_M):
        """GUIDED-mode position target, relative altitude."""
        self.mav.mav.set_position_target_global_int_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position only
            int(lat * 1e7), int(lon * 1e7), alt_m, 0, 0, 0, 0, 0, 0, 0, 0,
        )

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
        self.wait_for(lambda s: s["true_pos"] is not None and s["nav_trust"] is not None,
                      15, "real detector trust and simulator truth")
        if self.status["behavior"] != "NOMINAL" or self.status["nav_trust"] < NAV_TRUST_DETECT_THRESHOLD:
            raise ScenarioFailed(f"untrusted baseline before injection: {self.status}")
        if any(behavior != "NOMINAL" for _, behavior in self.status["behavior_log"]):
            raise ScenarioFailed("false alarm during unspoofed takeoff; see the core transition log")
        self.takeoff_estimate = summarize_estimate(self.status["est_samples"], self.status["est_rejects"])
        self.takeoff_estimate["core_log"] = estimate_readiness_from_log(self.core_log)

    def close(self):
        if self.mav is not None:
            self.mav.close()
        for proc in (self.proc, self.core_proc, self.dashboard_proc):
            if proc is None:
                continue
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for output in self.logs:
            output.close()


def expect(condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'} {message}")
    return condition


class SquareRoute:
    """Unspoofed: flies a square around where it started, so the control run
    covers ordinary maneuvering (accelerating, cruising, turning) and not just
    a hover. A hover never rotates accelerations through the attitude
    estimate, which is where dead-reckoning error actually comes from."""

    def __init__(self, sitl, side_m=CRUISE_SQUARE_M):
        self.sitl = sitl
        start = sitl.status["true_pos"]
        self.corners = [from_local(start, e, n) for e, n in ((side_m, 0), (side_m, side_m), (0, side_m), (0, 0))]
        self.leg = 0
        sitl.goto(*self.corners[0])

    def step(self, dt, true_pos):
        if distance_m(true_pos, self.corners[self.leg]) < ARRIVE_RADIUS_M * 2:
            self.leg = (self.leg + 1) % len(self.corners)
            self.sitl.goto(*self.corners[self.leg])
        return None  # no spoof


def run_window(sitl, duration_s, step_fn, label):
    """Runs the GPS at whatever offset step_fn(dt, true_pos) -> (fake_lat, fake_lon)
    says, or unspoofed if step_fn is None, and reports what happened."""
    print(f"\n{label}")
    home = sitl.status["true_pos"]
    log_start = len(sitl.status["behavior_log"])
    furthest = 0.0
    lowest_trust = sitl.status["nav_trust"]
    detect_time_s = None
    dashboard_samples = []
    dt = 0.5
    sitl.status["est_samples"] = []
    sitl.status["est_rejects"] = []
    began = time.monotonic()
    while (elapsed := time.monotonic() - began) < duration_s:
        true_pos = sitl.status["true_pos"]
        if step_fn is not None:
            fake = step_fn(dt, true_pos)
            if fake is not None:
                sitl.set_gps_glitch(fake[0] - true_pos[0], fake[1] - true_pos[1])
        sitl.pump(dt)
        if time.monotonic() - sitl.status["nav_at"] > 3:
            raise ScenarioFailed("real detector TRUST_NAV stopped arriving")
        if time.monotonic() - sitl.status["truth_at"] > 3:
            raise ScenarioFailed("SITL truth position stopped arriving")
        dashboard = dashboard_snapshot(sitl.dashboard_url)
        if not dashboard["Connected"] or dashboard["Nav"]["AtMs"] == 0 or dashboard["NowMs"] - dashboard["Nav"]["AtMs"] > 3000:
            raise ScenarioFailed("dashboard is not receiving live detector telemetry")
        sample = {"elapsed_s": round(time.monotonic() - began, 2),
                  "nav_trust": dashboard["Nav"]["Value"],
                  "behavior": dashboard["Behavior"],
                  "estimate_from": dashboard["EstimateFrom"]}
        sample.update(dashboard_estimate(dashboard, sitl.status["true_pos"]))
        dashboard_samples.append(sample)
        furthest = max(furthest, distance_m(home, sitl.status["true_pos"]))
        lowest_trust = min(lowest_trust, sitl.status["nav_trust"])
        if detect_time_s is None and sitl.status["nav_trust"] < NAV_TRUST_DETECT_THRESHOLD:
            detect_time_s = time.monotonic() - began

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
        "dashboard": {
            "lowest_nav_trust": round(min(s["nav_trust"] for s in dashboard_samples), 3),
            "behaviors": sorted(set(s["behavior"] for s in dashboard_samples if s["behavior"])),
            "estimate_sources": sorted(set(s["estimate_from"] for s in dashboard_samples)),
            "core_estimate": summarize_dashboard_estimate(dashboard_samples),
            "samples": dashboard_samples,
        },
        "core_estimate": summarize_estimate(sitl.status.get("est_samples", []), sitl.status.get("est_rejects", []),
                                            began),
    }
    detected = f"{result['detect_time_s']}s" if result["detect_time_s"] is not None else "never"
    print(
        f"  max true drift: {result['max_true_drift_m']} m, lowest TRUST_NAV: {result['lowest_nav_trust']}, "
        f"detected at: {detected}, behaviors: {result['behaviors_entered'] or 'none'}"
    )
    est = result["core_estimate"]
    if est["scored_pairs"] == 0:
        raise ScenarioFailed("no estimate pairs could be scored against fresh SIM_STATE truth")
    print(f"  core estimate: {est['pairs']} pairs, error vs truth median {est.get('error_median_m')} m, "
          f"p95 {est.get('error_p95_m')} m, max {est.get('error_max_m')} m; "
          f"GPS-estimate gap at end {est.get('final_gps_est_m')} m; longest silence {est.get('longest_gap_s')} s")
    return result


def _percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def summarize_estimate(samples, rejects, began=None):
    """Accuracy and continuity of core's EST_N/EST_E pairs over one window."""
    out = {"pairs": len(samples), "rejected_pairs": len(rejects)}
    errors = [s["error_m"] for s in samples if "error_m" in s]
    out["scored_pairs"] = len(errors)
    if errors:
        ages = [s["truth_age_s"] for s in samples if "truth_age_s" in s]
        out.update(error_median_m=round(statistics.median(errors), 2),
                   error_p95_m=round(_percentile(errors, 0.95), 2),
                   error_max_m=round(max(errors), 2),
                   truth_age_median_s=round(statistics.median(ages), 3),
                   truth_age_max_s=round(max(ages), 3))
    times = [s["t"] for s in samples]
    if times:
        edges = ([began] if began is not None else []) + times
        out["longest_gap_s"] = round(max(b - a for a, b in zip(edges, edges[1:])), 2) if len(edges) > 1 else None
    tail = [s for s in samples if "gps_est_m" in s][-10:]
    if tail:
        out["final_gps_truth_m"] = round(statistics.median(s["gps_truth_m"] for s in tail), 2)
        out["final_gps_est_m"] = round(statistics.median(s["gps_est_m"] for s in tail), 2)
        out["final_error_m"] = round(statistics.median(s["error_m"] for s in tail), 2)
    return out


def dashboard_estimate(snapshot, true_pos):
    """The estimate the dashboard bridge selected and delivered in its
    telemetry API: its distance from truth, and the reported-GNSS-to-estimate
    distance the drift line is drawn from. Read from the API only; no browser
    rendering is checked."""
    out = {}
    if snapshot.get("HasEstimate") and true_pos is not None:
        est = (snapshot["Estimated"]["lat"], snapshot["Estimated"]["lon"])
        out["estimate_truth_m"] = round(distance_m(est, true_pos), 2)
        if snapshot.get("HasPosition"):
            reported = (snapshot["Reported"]["lat"], snapshot["Reported"]["lon"])
            out["drift_m"] = round(distance_m(reported, est), 2)
    return out


def summarize_dashboard_estimate(samples):
    core = [s for s in samples if s["estimate_from"] == "core" and "estimate_truth_m" in s]
    out = {"core_fraction": round(len(core) / len(samples), 3) if samples else 0.0}
    if core:
        out["estimate_truth_max_m"] = max(s["estimate_truth_m"] for s in core)
        drifts = [s["drift_m"] for s in core if "drift_m" in s]
        if drifts:
            out["drift_final_m"] = drifts[-1]
            out["drift_max_m"] = max(drifts)
    return out


def reset_esp32(device):
    """Restarts the board through its USB-UART bridge's auto-reset wiring
    (RTS -> EN, DTR -> IO0 held high so it boots the app, not the ROM
    downloader), then waits for core to come up."""
    import serial  # pyserial, a pymavlink dependency
    with serial.Serial() as port:
        port.port, port.baudrate = device, 115200
        port.dtr = False
        port.rts = False
        port.open()
        port.rts = True
        time.sleep(0.1)
        port.rts = False
    time.sleep(1.5)


def estimate_readiness_from_log(path):
    """core_main's own per-second [nav_est] lines up to the first published
    estimate: what it was waiting for, and that nothing came out before."""
    if path is None:
        return {"error": "core runs on the ESP32; no core log"}
    statuses = []
    try:
        for line in Path(path).read_text(errors="replace").splitlines():
            if line.startswith("[nav_est] "):
                status = line.split()[1]
                statuses.append(status)
                if status == "ready":
                    break
    except OSError:
        return {"error": "no core log"}
    before = statuses[:-1] if statuses and statuses[-1] == "ready" else statuses
    return {"seconds_before_ready": len(before), "statuses_before_ready": sorted(set(before)),
            "reached_ready": bool(statuses) and statuses[-1] == "ready"}


class ImuGap:
    """Hovers honestly, then freezes SITL (so every stream to core, IMU
    included, stops) for IMU_GAP_S, once, to show core withholding the
    estimate across the gap and until its filter re-anchors afterward."""

    def __init__(self, sitl):
        self.sitl = sitl
        self.elapsed = 0.0
        self.done = False

    def step(self, dt, true_pos):
        self.elapsed += dt
        if not self.done and self.elapsed >= IMU_GAP_AT_S:
            self.done = True
            self.sitl.gap_started = time.monotonic()
            os.kill(self.sitl.proc.pid, signal.SIGSTOP)
            try:
                time.sleep(IMU_GAP_S)
            finally:
                os.kill(self.sitl.proc.pid, signal.SIGCONT)
        return None  # no spoof


def run_imu_gap(sitl, duration):
    gap = ImuGap(sitl)
    result = run_window(sitl, duration, gap.step, "imu gap (unspoofed hover, SITL frozen 2 s)")
    started = getattr(sitl, "gap_started", None)
    lines = []
    try:
        lines = [line.split()[1] for line in sitl.core_log.read_text(errors="replace").splitlines()
                 if line.startswith("[nav_est] ")]
    except (OSError, AttributeError):  # AttributeError: no core log on the ESP32
        pass
    # Pairs core sent in the freeze's first 0.5 s (before its IMU freshness
    # limit) sat in SITL's socket and arrive in a burst on resume, so judge the
    # withholding by the silence after that burst, and by core's own log.
    after = [s["t"] - started for s in sitl.status["est_samples"] if started is not None and s["t"] > started]
    resumed = [t for t in after if t > IMU_GAP_S + 0.3]
    result["imu_gap"] = {
        "core_statuses_seen": sorted(set(lines)),
        "pair_arrivals_after_freeze_s": [round(t, 2) for t in after[:12]],
        "first_pair_after_resume_burst_s": round(min(resumed), 2) if resumed else None,
    }
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
    "cruise": lambda sitl, rng, duration: run_window(
        sitl, duration, SquareRoute(sitl).step, "control cruise (unspoofed, flying a square)"
    ),
    "imu_gap": lambda sitl, rng, duration: run_imu_gap(sitl, duration),
}


def dashboard_snapshot(url):
    try:
        with urllib.request.urlopen(f"{url.rstrip('/')}/api/telemetry", timeout=3) as resp:
            for line in resp:
                if line.startswith(b"data: "):
                    return json.loads(line[6:])["Snapshot"]
        raise ScenarioFailed("dashboard returned no telemetry")
    except (OSError, ValueError, KeyError) as err:
        raise ScenarioFailed(f"dashboard telemetry unavailable: {err}") from err


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ardupilot", type=Path, required=True, help="ArduPilot repo with ./waf copter built for sitl")
    parser.add_argument(
        "--core-main", type=Path, default=Path(__file__).resolve().parent.parent / "build/core_main",
        help="built core_main binary (default: core/build/core_main)",
    )
    parser.add_argument("--core-port", type=int, default=14555, help="UDP port core_main listens on")
    parser.add_argument("--dashboard-serve", type=Path,
                        default=Path(__file__).resolve().parent.parent / "build/dashboard-serve",
                        help="built dashboard serve binary (started for each scenario)")
    parser.add_argument("--dashboard-port", type=int, default=14556, help="dashboard MAVLink UDP port")
    parser.add_argument("--dashboard-http-port", type=int, default=8081, help="dashboard HTTP port")
    parser.add_argument(
        "--instance", type=int, default=0,
        help="SITL instance; each one uses its own ports (5760 + 10*N), so a non-zero one can run beside run.sh",
    )
    parser.add_argument("--duration", type=float, default=60.0, help="seconds per scenario")
    parser.add_argument("--seed", type=int, default=1, help="RNG seed for the random-route scenario, for repeatability")
    parser.add_argument("--log-dir", type=Path, help="keep logs here instead of a temp directory")
    parser.add_argument("--dashboard-url", help="use an existing live bridge instead of starting one; it must listen on --dashboard-port")
    parser.add_argument("--esp32", metavar="DEVICE",
                        help="core is a flashed ESP32 on this serial device (e.g. /dev/cu.usbserial-0001), not core_main")
    parser.add_argument("--esp32-baud", type=int, default=921600, help="the firmware's CONFIG_CORE_UART_BAUD")
    parser.add_argument("scenarios", nargs="*", choices=[*SCENARIOS, "all"], default=["all"])
    args = parser.parse_args()
    if args.duration <= 0 or args.instance < 0:
        parser.error("duration must be positive and instance non-negative")

    scenarios = list(SCENARIOS) if "all" in args.scenarios else args.scenarios

    workdir = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="gnss_e2e_"))
    workdir.mkdir(parents=True, exist_ok=True)
    print(f"logs in {workdir}")

    results = []
    try:
        for name in scenarios:
            # Each scenario gets its own fresh flight: once autonomy fails
            # over away from GPS it deliberately doesn't rush back (flapping
            # back onto a source that might still be spoofed would defeat the
            # point), so a scenario chained right after one that triggered
            # failover would start from a contaminated, non-NOMINAL baseline
            # rather than the clean one its own result implies.
            scenario_dir = workdir / name
            scenario_dir.mkdir(parents=True, exist_ok=True)
            sitl = Sitl(args.ardupilot.resolve(), args.core_main.resolve(), scenario_dir, args.core_port, args.instance,
                        args.dashboard_serve.resolve(), args.dashboard_port, args.dashboard_http_port, args.dashboard_url,
                        args.esp32, args.esp32_baud)
            try:
                print(f"\n[{name}] taking off")
                sitl.take_off()
                rng = random.Random(args.seed)
                result = SCENARIOS[name](sitl, rng, args.duration)
                takeoff_estimate = getattr(sitl, "takeoff_estimate", None)
                if isinstance(result, dict) and isinstance(takeoff_estimate, dict):
                    result["takeoff_estimate"] = takeoff_estimate
            finally:
                sitl.close()
            results.append(result)

        print("\n=== summary ===")
        no_false_alarm = True
        for r in results:
            summary = dict(r)
            if "dashboard" in summary:
                summary["dashboard"] = {k: v for k, v in r["dashboard"].items() if k != "samples"}
            print(f"{r['label']}: {json.dumps(summary)}")
            if r["label"].startswith("control") and (r["behaviors_entered"] or r["lowest_nav_trust"] < NAV_TRUST_DETECT_THRESHOLD):
                no_false_alarm = False
        if any(r["label"].startswith("control") for r in results):
            expect(no_false_alarm, "no false alarm during the unspoofed control runs")

        results_path = workdir / "results.json"
        results_path.write_text(json.dumps(results, indent=2))
        print(f"\nfull results written to {results_path}")
        return 0 if no_false_alarm else 1
    except (ScenarioFailed, OSError) as err:
        print(f"\nFAIL: {err}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
