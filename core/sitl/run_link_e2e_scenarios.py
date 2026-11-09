#!/usr/bin/env python3
"""Flies ArduCopter SITL, jams or cuts the operator's link, and checks that
core_main's real link monitor and autonomy hold and then return the vehicle.

Usage: run_link_e2e_scenarios.py --ardupilot <repo> [--core-main <binary>] [scenario ...]

    operator GCS (this script, sysid 255) -- link_jam_proxy -- SITL serial2
                         core_main (real link monitor + nav + autonomy) -- SITL serial1
                         dashboard bridge (live mode) -- SITL serial4
              flight script and observer (this script, sysid 200) -- SITL tcp 5760+10N

Nothing types a trust score: the proxy drops the operator's real heartbeats
and reports the radio's RSSI and noise as a SiK radio would, core_main judges
the link from what ArduPilot forwards, and its autonomy commands the vehicle.
ArduPilot's own GCS failsafe is off (link_e2e.parm), so any hold or return
comes from core. The flight commands come from a separate link (sysid 200), as
a mission would; the operator's link is the one under attack.

Scenarios, each on a fresh SITL, flown out 60 m from home at 20 m:

    jam        jamming (noise up, RSSI high, packets gone): JAMMED, BRAKE, then
               RTL after autonomy's 30 s hold; lands at home
    lost       clean loss (signal gone): LOST, BRAKE, then RTL; lands at home
    dropouts   4 x 2 s full dropouts: no autonomy action, mode unchanged
    recover    jamming until autonomy holds, then the link returns: back to
               NOMINAL without a return, and the mode is left to the operator
    companion_outage
               the companion computer's own link to the autopilot is cut for
               8 s while the operator's is fine: no hold, no return, no BRAKE
               once it's back (core_main sits behind a pausable relay here)

Assumes nothing else uses the chosen instance and ports; it refuses to start if
they're taken. Needs pymavlink and a built dashboard bridge (or --no-dashboard).
"""
import argparse
import csv
import json
import os
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from link_jam_proxy import LinkJamProxy  # noqa: E402
from run_gnss_e2e_scenarios import dashboard_snapshot, distance_m, from_local  # noqa: E402
from run_link_scenario import Operator  # noqa: E402

GUIDED, RTL, LAND, BRAKE = 4, 6, 9, 17
MODE_NAMES = {GUIDED: "GUIDED", RTL: "RTL", LAND: "LAND", BRAKE: "BRAKE"}
LINK_STATES = {0: "UNKNOWN", 1: "NOMINAL", 2: "DEGRADED", 3: "LOST", 4: "JAMMED"}
SEVERITY = {"UNKNOWN": -1, "NOMINAL": 0, "DEGRADED": 1, "LOST": 2, "JAMMED": 3}
TAKEOFF_ALT_M = 20
OUTBOUND_M = 60
LINK_HOLD_MS = 30000  # core_autonomy_default_config().link_hold_ms
UNTRUSTED_BELOW = 0.4  # core_autonomy_default_config().untrusted_below


class ScenarioFailed(Exception):
    pass


def check_ports(ports):
    if len(set(ports)) != len(ports):
        raise ScenarioFailed("each component needs a distinct port")
    for port, kind in ports:
        with socket.socket(socket.AF_INET, kind) as sock:
            if kind == socket.SOCK_STREAM:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(("127.0.0.1", port))
            except OSError as err:
                raise ScenarioFailed(f"port {port} is in use; pick another --instance or port") from err


class CompanionRelay:
    """A pausable UDP relay between SITL's SERIAL1 and core_main, standing in
    for the companion computer's cable to the flight controller."""

    def __init__(self, listen_port, core_port):
        self.fc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fc_sock.bind(("127.0.0.1", listen_port))
        self.core_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.core_sock.bind(("127.0.0.1", 0))
        self.core_addr = ("127.0.0.1", core_port)
        self.fc_addr = None
        self.paused = False
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            ready, _, _ = select.select([self.fc_sock, self.core_sock], [], [], 0.2)
            for sock in ready:
                try:
                    data, addr = sock.recvfrom(65535)
                except OSError:
                    continue
                if sock is self.fc_sock:
                    self.fc_addr = addr
                    if not self.paused:
                        self.core_sock.sendto(data, self.core_addr)
                elif self.fc_addr is not None and not self.paused:
                    self.fc_sock.sendto(data, self.fc_addr)

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        self.fc_sock.close()
        self.core_sock.close()


class Flight:
    """One SITL vehicle with core_main, the link proxy, the operator and the
    dashboard, plus a timeline of everything observed."""

    def __init__(self, args, workdir):
        self.workdir = workdir
        self.procs, self.logs = [], []
        self.proxy = self.operator = self.mav = self.relay = None
        self.dashboard_url = None
        self.t0 = time.monotonic()
        self.status = {"mode": None, "armed": False, "rel_alt": 0.0, "pos": None, "home": None,
                       "groundspeed": 0.0, "link_state": "UNKNOWN", "link_trust": None, "link_at": 0.0,
                       "nav_trust": None, "nav_at": 0.0, "behavior": "NOMINAL"}
        self.events = []  # (seconds since start, kind, detail)
        self.texts = []  # (seconds, text)
        self.dash = {"link_values": [], "behaviors": set(), "link_at_ms": 0}
        try:
            self._start(args)
        except BaseException:
            self.close()
            raise

    def _launch(self, name, command):
        output = open(self.workdir / f"{name}.log", "w")
        self.logs.append(output)
        proc = subprocess.Popen(command, cwd=self.workdir, stdout=output, stderr=subprocess.STDOUT)
        self.procs.append((name, proc))
        return proc

    def _start(self, args):
        tcp = 5760 + 10 * args.instance
        ports = [(tcp, socket.SOCK_STREAM), (args.core_port, socket.SOCK_DGRAM),
                 (args.proxy_fc_port, socket.SOCK_DGRAM), (args.proxy_gcs_port, socket.SOCK_DGRAM)]
        if args.relay:
            ports.append((args.relay_port, socket.SOCK_DGRAM))
        if not args.no_dashboard:
            ports += [(args.dashboard_port, socket.SOCK_DGRAM), (args.dashboard_http_port, socket.SOCK_STREAM)]
        check_ports(ports)

        if not args.no_dashboard:
            if not args.dashboard_serve.is_file():
                raise ScenarioFailed("build the dashboard bridge first: "
                                     "cd dashboard && go build -o ../core/build/dashboard-serve ./cmd/serve")
            self._launch("dashboard", [str(args.dashboard_serve), "-addr", f"127.0.0.1:{args.dashboard_http_port}",
                                       "-mavlink", f"127.0.0.1:{args.dashboard_port}"])
            self.dashboard_url = f"http://127.0.0.1:{args.dashboard_http_port}"
        # core_main runs in workdir, so its autonomy_transitions.csv lands there.
        self._launch("core_main", [str(args.core_main), str(args.core_port)])
        self.proxy = LinkJamProxy(args.proxy_fc_port, args.proxy_gcs_port, seed=args.seed).start()
        serial1_port = args.core_port
        if args.relay:
            self.relay = CompanionRelay(args.relay_port, args.core_port)
            serial1_port = args.relay_port
        time.sleep(1)

        serials = [f"--serial1=udpclient:127.0.0.1:{serial1_port}",
                   f"--serial2=udpclient:127.0.0.1:{args.proxy_fc_port}"]
        if not args.no_dashboard:
            serials.append(f"--serial4=udpclient:127.0.0.1:{args.dashboard_port}")
        defaults = ",".join(str(p) for p in (
            args.ardupilot / "Tools/autotest/default_params/copter.parm", HERE / "autonomy_failover.parm",
            HERE / "link_jam.parm", HERE / "link_e2e.parm"))
        self._launch("arducopter", [str(args.ardupilot / "build/sitl/bin/arducopter"), "--model", "+",
                                    "-I", str(args.instance), "--wipe", "--defaults", defaults, *serials])

        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{tcp}", retries=60, source_system=200,
                                              dialect="ardupilotmega")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.mav.target_system, self.mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        else:
            raise ScenarioFailed("no SITL autopilot heartbeat")
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        self.operator = Operator(args.proxy_gcs_port)
        self._last_hb = 0.0

    def now(self):
        return time.monotonic() - self.t0

    def event(self, kind, detail):
        self.events.append((round(self.now(), 1), kind, detail))
        print(f"  {self.now():6.1f}s {kind}: {detail}", flush=True)

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while True:
            for name, proc in self.procs:
                if proc.poll() is not None:
                    raise ScenarioFailed(f"{name} exited with status {proc.returncode}")
            if time.monotonic() - self._last_hb >= 1.0:
                # A GCS, but not the operator: ArduPilot only forwards to links it has heard from.
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                            0, 0, 0)
                self._last_hb = time.monotonic()
            msg = self.mav.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self._note(msg)
            if time.monotonic() >= deadline:
                return

    def _note(self, msg):
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            self.texts.append((round(self.now(), 1), msg.text))
            if msg.text.startswith(("LINK ", "AUTONOMY ")) or "ailsafe" in msg.text:
                self.event("text", msg.text)
            if msg.text.startswith("AUTONOMY "):
                self.status["behavior"] = msg.text[len("AUTONOMY "):].split(":", 1)[0]
            return
        if msg.get_srcSystem() != self.mav.target_system:
            return
        if kind == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            mode = msg.custom_mode
            if mode != self.status["mode"]:
                self.event("mode", MODE_NAMES.get(mode, str(mode)))
            self.status["mode"] = mode
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != self.status["armed"]:
                self.event("armed", armed)
            self.status["armed"] = armed
        elif kind == "GLOBAL_POSITION_INT":
            self.status["rel_alt"] = msg.relative_alt / 1000
            self.status["pos"] = (msg.lat / 1e7, msg.lon / 1e7)
        elif kind == "HOME_POSITION":
            self.status["home"] = (msg.latitude / 1e7, msg.longitude / 1e7)
        elif kind == "VFR_HUD":
            self.status["groundspeed"] = msg.groundspeed
        elif kind == "NAMED_VALUE_FLOAT":
            if msg.name == "TRUST_LNK":
                self.status["link_trust"], self.status["link_at"] = msg.value, time.monotonic()
            elif msg.name == "LNK_STATE":
                state = LINK_STATES.get(int(round(msg.value)), "UNKNOWN")
                if state != self.status["link_state"]:
                    self.event("link", state)
                self.status["link_state"] = state
            elif msg.name == "TRUST_NAV":
                self.status["nav_trust"], self.status["nav_at"] = msg.value, time.monotonic()

    def sample_dashboard(self):
        if self.dashboard_url is None:
            return
        snap = dashboard_snapshot(self.dashboard_url)
        link = snap["Link"]
        if link["AtMs"]:
            self.dash["link_values"].append(round(link["Value"], 3))
            self.dash["link_at_ms"] = link["AtMs"]
        if snap.get("Behavior"):
            self.dash["behaviors"].add(snap["Behavior"])

    def wait_for(self, condition, timeout, what, sample=None):
        deadline = time.monotonic() + timeout
        last_dash = 0.0
        while time.monotonic() < deadline:
            self.pump(0.2)
            if sample is not None:
                sample(self.status)
            if time.monotonic() - last_dash >= 1.0:
                self.sample_dashboard()
                last_dash = time.monotonic()
            if condition(self.status):
                return self.now()
        raise ScenarioFailed(f"timed out after {timeout:.0f} s waiting for {what}; status {self.status}")

    def command(self, command, *params):
        params = list(params) + [0] * (7 - len(params))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, command, 0, *params)

    def take_off_and_fly_out(self):
        self.wait_for(lambda s: s["link_state"] == "NOMINAL", 120, "core to judge the operator link NOMINAL")
        self.wait_for(lambda s: s["nav_trust"] is not None and s["pos"] is not None, 60, "nav trust and position")
        deadline = time.monotonic() + 120
        while not self.status["armed"]:  # refused until the EKF has settled
            if time.monotonic() > deadline:
                raise ScenarioFailed("vehicle never armed")
            self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, GUIDED)
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.pump(2)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
        self.wait_for(lambda s: s["rel_alt"] > TAKEOFF_ALT_M - 1, 60, "takeoff altitude")
        if self.status["home"] is None:
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
        self.wait_for(lambda s: s["home"] is not None, 10, "home position")
        target = from_local(self.status["home"], 0.0, OUTBOUND_M)
        self.mav.mav.set_position_target_global_int_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
            int(target[0] * 1e7), int(target[1] * 1e7), TAKEOFF_ALT_M, 0, 0, 0, 0, 0, 0, 0, 0)
        self.wait_for(lambda s: distance_m(s["pos"], target) < 3 and s["groundspeed"] < 0.5, 60,
                      f"arrival {OUTBOUND_M} m out")
        self.pump(3)
        if self.status["behavior"] != "NOMINAL" or self.status["link_state"] != "NOMINAL":
            raise ScenarioFailed(f"not a clean baseline before the attack: {self.status}")
        self.event("ready", f"{self.home_distance():.0f} m from home at {self.status['rel_alt']:.0f} m, "
                            f"mode {MODE_NAMES.get(self.status['mode'])}")

    def home_distance(self):
        return distance_m(self.status["home"], self.status["pos"])

    def transitions(self):
        """autonomy_transitions.csv, as core_main wrote it."""
        path = self.workdir / "autonomy_transitions.csv"
        if not path.is_file():
            return []
        with open(path) as f:
            return list(csv.DictReader(f))

    def close(self):
        if self.operator is not None:
            self.operator.running = False
        if self.mav is not None:
            self.mav.close()
        if self.proxy is not None:
            self.proxy.stop()
        if self.relay is not None:
            self.relay.stop()
        for _, proc in reversed(self.procs):  # only our own children
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for output in self.logs:
            output.close()


def expect(results, condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'} {message}", flush=True)
    results["checks"].append({"ok": bool(condition), "check": message})
    if not condition:
        raise ScenarioFailed(message)


def common_checks(f, r):
    behaviors = [row["to"] for row in f.transitions()]
    expect(r, not ({"GNSS_FAILOVER", "SAFE_LAND"} & set(behaviors)),
           f"navigation stayed trusted: no GNSS_FAILOVER or SAFE_LAND ({' -> '.join(behaviors) or 'none'})")
    expect(r, not any("ailsafe" in text for _, text in f.texts), "ArduPilot's own failsafes never fired")


def attack_hold_return(f, r, profile, ramp_s, verdict, other):
    """Attack until the link is judged, autonomy holds, escalates to RTL, and
    the vehicle lands at home."""
    start = f.now()
    f.proxy.profile(profile, ramp_s=ramp_s)
    f.event("inject", f"{profile} (ramp {ramp_s:.0f} s)")
    states = []
    track = lambda s: states.append(s["link_state"]) if not states or states[-1] != s["link_state"] else None

    t_verdict = f.wait_for(lambda s: s["link_state"] == verdict, 60, f"link {verdict}", track)
    t_hold = f.wait_for(lambda s: s["behavior"] == "LINK_HOLD", 10, "autonomy LINK_HOLD", track)
    t_brake = f.wait_for(lambda s: s["mode"] == BRAKE, 10, "BRAKE confirmed by the vehicle", track)
    expect(r, f.status["link_trust"] is not None and f.status["link_trust"] < UNTRUSTED_BELOW,
           f"measured TRUST_LNK {f.status['link_trust']:.2f} is below autonomy's {UNTRUSTED_BELOW}")
    f.pump(5)
    held_at = f.status["pos"]
    held_speed = f.status["groundspeed"]
    expect(r, held_speed < 1.0, f"the vehicle holds position in BRAKE ({held_speed:.2f} m/s)")

    t_return = f.wait_for(lambda s: s["behavior"] == "LINK_RETURN", LINK_HOLD_MS / 1000 + 10,
                          "autonomy LINK_RETURN", track)
    t_rtl = f.wait_for(lambda s: s["mode"] == RTL, 10, "RTL confirmed by the vehicle", track)
    hold_s = t_return - t_hold
    expect(r, LINK_HOLD_MS / 1000 - 1 <= hold_s <= LINK_HOLD_MS / 1000 + 3,
           f"held {hold_s:.1f} s before returning (autonomy's link_hold_ms is {LINK_HOLD_MS / 1000:.0f} s)")
    drift_in_hold = distance_m(held_at, f.status["pos"])
    expect(r, drift_in_hold < 5, f"stayed within {drift_in_hold:.1f} m while holding")
    return_from_m = f.home_distance()
    expect(r, return_from_m > OUTBOUND_M / 2, f"RTL started {return_from_m:.0f} m from home")
    t_landed = f.wait_for(lambda s: not s["armed"], 180, "RTL to land and disarm", track)
    home_m = f.home_distance()
    expect(r, home_m < 5, f"returned and landed {home_m:.1f} m from home")
    expect(r, other not in states, f"never called {other} ({' -> '.join(states)})")

    rows = f.transitions()
    pairs = [(row["from"], row["to"]) for row in rows]
    expect(r, ("NOMINAL", "LINK_HOLD") in pairs and ("LINK_HOLD", "LINK_RETURN") in pairs,
           f"transitions logged in autonomy_transitions.csv ({pairs})")
    hold_row = next(row for row in rows if row["to"] == "LINK_HOLD")
    expect(r, float(hold_row["link_trust"]) < UNTRUSTED_BELOW and float(hold_row["nav_trust"]) >= 0.7,
           f"logged with link_trust {hold_row['link_trust']} and nav_trust {hold_row['nav_trust']}")
    common_checks(f, r)
    if f.dashboard_url:
        expect(r, f.dash["link_values"] and min(f.dash["link_values"]) < UNTRUSTED_BELOW,
               f"dashboard showed the measured link trust (min {min(f.dash['link_values'] or [1]):.2f})")
        expect(r, {"LINK_HOLD", "LINK_RETURN"} <= f.dash["behaviors"],
               f"dashboard showed the behaviors ({sorted(f.dash['behaviors'])})")
    r["timeline_s"] = {
        "link_verdict": round(t_verdict - start, 1), "link_hold": round(t_hold - start, 1),
        "brake_confirmed": round(t_brake - start, 1), "link_return": round(t_return - start, 1),
        "rtl_confirmed": round(t_rtl - start, 1), "landed_disarmed": round(t_landed - start, 1),
    }
    r["link_states"] = states
    r["return_from_home_m"] = round(return_from_m, 1)
    r["landed_from_home_m"] = round(home_m, 1)


def scenario_jam(f, r):
    attack_hold_return(f, r, "jam", 8, "JAMMED", "LOST")


def scenario_lost(f, r):
    attack_hold_return(f, r, "lost", 2, "LOST", "JAMMED")


def scenario_dropouts(f, r):
    states, min_trust = [], [1.0]

    def track(s):
        if not states or states[-1] != s["link_state"]:
            states.append(s["link_state"])
        if s["link_trust"] is not None:
            min_trust[0] = min(min_trust[0], s["link_trust"])

    for i in range(4):
        f.proxy.set(drop=1.0)
        f.event("inject", f"dropout {i + 1}/4, 2 s")
        end = time.monotonic() + 2
        while time.monotonic() < end:
            f.pump(0.2)
            track(f.status)
        f.proxy.set(drop=0.0)
        end = time.monotonic() + 8
        while time.monotonic() < end:
            f.pump(0.2)
            track(f.status)
            f.sample_dashboard()
    end = time.monotonic() + 15
    while time.monotonic() < end:
        f.pump(0.2)
        track(f.status)
    worst = max(states, key=lambda s: SEVERITY[s])
    expect(r, SEVERITY[worst] <= SEVERITY["DEGRADED"], f"brief dropouts never LOST or JAMMED ({' -> '.join(states)})")
    expect(r, min_trust[0] >= UNTRUSTED_BELOW, f"TRUST_LNK stayed trusted (min {min_trust[0]:.2f})")
    expect(r, not f.transitions(), "autonomy never acted")
    expect(r, f.status["mode"] == GUIDED and f.status["armed"], "still flying the operator's GUIDED mission")
    common_checks(f, r)
    r["link_states"], r["min_link_trust"] = states, round(min_trust[0], 3)


def scenario_recover(f, r):
    start = f.now()
    f.proxy.profile("jam", ramp_s=8)
    f.event("inject", "jam (ramp 8 s)")
    t_hold = f.wait_for(lambda s: s["behavior"] == "LINK_HOLD", 60, "autonomy LINK_HOLD")
    f.wait_for(lambda s: s["mode"] == BRAKE, 10, "BRAKE confirmed by the vehicle")
    f.proxy.profile("nominal")
    t_restored = f.now()
    f.event("restore", "link nominal")
    t_nominal = f.wait_for(lambda s: s["behavior"] == "NOMINAL", LINK_HOLD_MS / 1000, "autonomy back to NOMINAL")
    f.pump(10)
    expect(r, t_nominal - t_hold < LINK_HOLD_MS / 1000, f"recovered {t_nominal - t_restored:.1f} s after the link "
                                                        f"returned, before the {LINK_HOLD_MS / 1000:.0f} s hold ran out")
    pairs = [(row["from"], row["to"]) for row in f.transitions()]
    expect(r, pairs == [("NOMINAL", "LINK_HOLD"), ("LINK_HOLD", "NOMINAL")], f"no return was triggered ({pairs})")
    expect(r, f.status["mode"] == BRAKE and f.status["armed"],
           f"the mode is left to the operator (still {MODE_NAMES.get(f.status['mode'])})")
    common_checks(f, r)
    r["timeline_s"] = {"link_hold": round(t_hold - start, 1), "link_restored": round(t_restored - start, 1),
                       "nominal": round(t_nominal - start, 1)}


def scenario_companion_outage(f, r):
    states = []
    track = lambda s: states.append(s["link_state"]) if not states or states[-1] != s["link_state"] else None
    f.relay.paused = True
    f.event("inject", "companion link to the autopilot cut for 8 s; operator link untouched")
    end = time.monotonic() + 8
    while time.monotonic() < end:
        f.pump(0.2)
        track(f.status)
    f.relay.paused = False
    f.event("restore", "companion link back")
    end = time.monotonic() + 40  # past a 30 s hold, so a latched LINK_RETURN would show too
    while time.monotonic() < end:
        f.pump(0.2)
        track(f.status)
        f.sample_dashboard()
    expect(r, f.status["link_state"] == "NOMINAL", f"the operator link is judged NOMINAL again ({' -> '.join(states)})")
    expect(r, not f.transitions(), f"autonomy never acted ({[(x['from'], x['to']) for x in f.transitions()]})")
    expect(r, f.status["mode"] == GUIDED and f.status["armed"],
           f"still flying the operator's GUIDED mission (mode {MODE_NAMES.get(f.status['mode'], f.status['mode'])})")
    common_checks(f, r)
    r["link_states"] = states


SCENARIOS = {"jam": scenario_jam, "lost": scenario_lost, "dropouts": scenario_dropouts, "recover": scenario_recover,
             "companion_outage": scenario_companion_outage}
RELAY_SCENARIOS = {"companion_outage"}


def main():
    build = HERE.parent / "build"
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ardupilot", type=Path, required=True, help="ArduPilot repo with ArduCopter SITL built")
    parser.add_argument("--core-main", type=Path, default=build / "core_main")
    parser.add_argument("--dashboard-serve", type=Path, default=build / "dashboard-serve")
    parser.add_argument("--no-dashboard", action="store_true", help="don't start or check the dashboard bridge")
    parser.add_argument("--instance", type=int, default=0, help="SITL instance; its TCP port is 5760 + 10*N")
    parser.add_argument("--core-port", type=int, default=14551)
    parser.add_argument("--proxy-fc-port", type=int, default=14560)
    parser.add_argument("--proxy-gcs-port", type=int, default=14562)
    parser.add_argument("--dashboard-port", type=int, default=14556)
    parser.add_argument("--dashboard-http-port", type=int, default=8081)
    parser.add_argument("--relay-port", type=int, default=14557,
                        help="companion_outage only: SERIAL1 goes to this relay, which forwards to --core-port")
    parser.add_argument("--seed", type=int, default=2, help="the proxy's packet-drop RNG seed")
    parser.add_argument("--log-dir", type=Path, help="keep logs here instead of a temp directory")
    parser.add_argument("scenarios", nargs="*", choices=[*SCENARIOS, "all"], default=["all"])
    args = parser.parse_args()
    args.ardupilot = args.ardupilot.resolve()
    args.core_main = args.core_main.resolve()
    args.dashboard_serve = args.dashboard_serve.resolve()

    build.mkdir(parents=True, exist_ok=True)
    workdir = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="link_e2e_", dir=build))
    workdir.mkdir(parents=True, exist_ok=True)
    print(f"logs in {workdir}")
    names = list(SCENARIOS) if "all" in args.scenarios else args.scenarios
    results, ok = [], True
    for name in names:
        print(f"\n[{name}]", flush=True)
        scenario_dir = workdir / name
        scenario_dir.mkdir(parents=True, exist_ok=True)
        r = {"scenario": name, "checks": [], "passed": False}
        flight = None
        args.relay = name in RELAY_SCENARIOS
        try:
            flight = Flight(args, scenario_dir)
            flight.take_off_and_fly_out()
            SCENARIOS[name](flight, r)
            r["passed"] = True
        except (ScenarioFailed, OSError) as err:
            print(f"  FAILED: {err}", flush=True)
            r["error"] = str(err)
            ok = False
        finally:
            if flight is not None:
                r["events"] = flight.events
                r["transitions"] = flight.transitions()
                r["proxy"] = dict(flight.proxy.stats) if flight.proxy else None
                r["dashboard"] = {"min_link_trust": min(flight.dash["link_values"]) if flight.dash["link_values"]
                                  else None, "behaviors": sorted(flight.dash["behaviors"])}
                flight.close()
        results.append(r)

    print("\n=== summary ===")
    for r in results:
        print(f"{r['scenario']:<9} {'PASS' if r['passed'] else 'FAIL'}  "
              f"{json.dumps(r.get('timeline_s') or {'link_states': r.get('link_states')})}")
    (workdir / "results.json").write_text(json.dumps(results, indent=2))
    print(f"full results in {workdir / 'results.json'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
