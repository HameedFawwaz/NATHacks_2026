#!/usr/bin/env python3
"""Jams and drops the operator link in ArduCopter SITL and checks core_main's verdict.

Usage: run_link_scenario.py --ardupilot <repo> --core-main <build/core_main>

    operator GCS (this script, sysid 255) -- link_jam_proxy -- SITL serial2
                                       core_main (udp 14551) -- SITL serial1
                         observer (this script, sysid 200) -- SITL tcp 5760

The observer reads what core_main publishes, as any ground station would:
TRUST_LNK and LNK_STATE as NAMED_VALUE_FLOAT, and "LINK ..." STATUSTEXTs.
The vehicle stays on the ground; detection doesn't depend on flight.

Each phase checks one acceptance criterion from issue #2. Detection times
include the radio's reporting lag, so they're slower than the unit tests.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from link_jam_proxy import LinkJamProxy  # noqa: E402

CORE_PORT, PROXY_FC_PORT, PROXY_GCS_PORT = 14551, 14560, 14562
STATES = {0: "UNKNOWN", 1: "NOMINAL", 2: "DEGRADED", 3: "LOST", 4: "JAMMED"}
SEVERITY = {"UNKNOWN": -1, "NOMINAL": 0, "DEGRADED": 1, "LOST": 2, "JAMMED": 3}


class Failed(Exception):
    pass


class Observer:
    """Follows core_main's published link verdict through the autopilot."""

    def __init__(self):
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", retries=60, source_system=200)
        self.mav.wait_heartbeat(timeout=60)
        self.lock = threading.Lock()
        self.state, self.trust, self.updated = "UNKNOWN", None, 0.0
        self.history = []  # (time, state)
        self.texts = []
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_hb = 0.0
        while self.running:
            now = time.monotonic()
            if now - last_hb >= 1.0:  # without it, the autopilot forwards nothing to this link
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            kind = msg.get_type()
            with self.lock:
                if kind == "NAMED_VALUE_FLOAT" and msg.name == "TRUST_LNK":
                    self.trust, self.updated = msg.value, now
                elif kind == "NAMED_VALUE_FLOAT" and msg.name == "LNK_STATE":
                    state = STATES.get(int(round(msg.value)), "UNKNOWN")
                    if state != self.state:
                        self.history.append((now, state))
                    self.state, self.updated = state, now
                elif kind == "STATUSTEXT" and msg.text.startswith("LINK "):
                    self.texts.append((now, msg.text))
                    print(f"  [core] {msg.text}", flush=True)

    def snapshot(self):
        with self.lock:
            return self.state, self.trust, self.updated


class Operator:
    """The operator's ground station: a heartbeat a second, through the proxy."""

    def __init__(self):
        self.mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{PROXY_GCS_PORT}", source_system=255)
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last = 0.0
        while self.running:
            if time.monotonic() - last >= 1.0:
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last = time.monotonic()
            self.mav.recv_match(blocking=True, timeout=0.1)


class Phase:
    """Samples the verdict for a stretch of time and records what it saw."""

    def __init__(self, observer, name):
        self.observer, self.name = observer, name
        self.start = time.monotonic()
        self.states, self.min_trust, self.first_seen = [], None, {}

    def watch(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            state, trust, updated = self.observer.snapshot()
            if updated and time.monotonic() - updated > 3:
                raise Failed(f"{self.name}: core_main stopped publishing its link state")
            if not self.states or self.states[-1] != state:
                self.states.append(state)
            self.first_seen.setdefault(state, time.monotonic() - self.start)
            if trust is not None and state != "UNKNOWN":
                self.min_trust = trust if self.min_trust is None else min(self.min_trust, trust)
            if until is not None and state == until:
                return True
            time.sleep(0.2)
        return until is None

    @property
    def worst(self):
        return max(self.states, key=lambda s: SEVERITY[s])

    def summary(self):
        trust = "n/a" if self.min_trust is None else f"{self.min_trust:.2f}"
        return f"{self.name:<16} states {' -> '.join(self.states):<40} min trust {trust}"


def expect(condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'} {message}", flush=True)
    if not condition:
        raise Failed(message)


def run(proxy, observer):
    results = []

    print("waiting for core_main to judge the link")
    warmup = Phase(observer, "warmup")
    expect(warmup.watch(120, until="NOMINAL"), "link reaches NOMINAL")

    print("phase: nominal")
    p = Phase(observer, "nominal")
    p.watch(20)
    results.append(p)
    expect(p.states == ["NOMINAL"], "a healthy link stays NOMINAL")
    expect(p.min_trust >= 0.9, f"trust stays high (min {p.min_trust:.2f})")

    print("phase: brief dropouts, 4 x 2 s")
    p = Phase(observer, "brief dropouts")
    for _ in range(4):
        proxy.set(drop=1.0)
        p.watch(2)
        proxy.set(drop=0.0)
        p.watch(8)
    results.append(p)
    expect(SEVERITY[p.worst] <= SEVERITY["DEGRADED"], f"brief dropouts are never LOST or JAMMED (worst {p.worst})")
    expect(p.min_trust >= 0.4, f"and never cross autonomy's untrusted threshold (min {p.min_trust:.2f})")

    print("phase: degraded, 30% loss and a weak signal")
    proxy.set(drop=0.3, rssi=60, noise=45)
    p = Phase(observer, "degraded")
    p.watch(30)
    results.append(p)
    expect("DEGRADED" in p.states, "sustained loss is DEGRADED")
    expect(p.states.count("NOMINAL") <= 1 and p.states[-1] != "NOMINAL",
           f"and stays DEGRADED rather than bouncing ({' -> '.join(p.states)})")
    expect("JAMMED" not in p.states, "a weak signal is not called jamming")
    proxy.profile("nominal")
    p = Phase(observer, "recovery 1")
    expect(p.watch(60, until="NOMINAL"), "recovers to NOMINAL")
    expect("JAMMED" not in p.states[1:], f"without being called jamming on the way back ({' -> '.join(p.states)})")
    p.watch(10)
    results.append(p)
    expect(p.states[-1] == "NOMINAL", "and stays there")

    print("phase: jamming, noise up and packets gone over 8 s")
    proxy.profile("jam", ramp_s=8)
    p = Phase(observer, "jam")
    detected = p.watch(40, until="JAMMED")
    p.watch(5)
    results.append(p)
    expect(detected, f"jamming detected as JAMMED (after {p.first_seen.get('JAMMED', float('nan')):.1f} s)")
    expect("LOST" not in p.states, "and not mistaken for a clean loss")
    expect(p.min_trust == 0.0, "trust drops to 0")
    proxy.profile("nominal")
    p = Phase(observer, "recovery 2")
    expect(p.watch(60, until="NOMINAL"), "recovers to NOMINAL")
    expect("JAMMED" not in p.states[1:], f"without being called jamming on the way back ({' -> '.join(p.states)})")
    p.watch(10)
    results.append(p)
    expect(len(p.states) <= 3, f"without bouncing ({' -> '.join(p.states)})")

    print("phase: clean loss, operator's signal gone")
    proxy.profile("lost", ramp_s=2)
    p = Phase(observer, "clean loss")
    detected = p.watch(40, until="LOST")
    p.watch(5)
    results.append(p)
    expect(detected, f"loss detected as LOST (after {p.first_seen.get('LOST', float('nan')):.1f} s)")
    expect("JAMMED" not in p.states, "and not mistaken for jamming")
    proxy.profile("nominal")
    p = Phase(observer, "recovery 3")
    expect(p.watch(60, until="NOMINAL"), "recovers to NOMINAL")
    expect("JAMMED" not in p.states[1:], f"without being called jamming on the way back ({' -> '.join(p.states)})")
    results.append(p)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--core-main", type=Path, required=True)
    args = parser.parse_args()

    work = Path(tempfile.mkdtemp(prefix="link_scenario_"))
    here = Path(__file__).parent
    procs = []
    proxy = LinkJamProxy(PROXY_FC_PORT, PROXY_GCS_PORT, seed=2).start()
    results = []
    observer = operator = None
    try:
        procs.append(subprocess.Popen([str(args.core_main), str(CORE_PORT)], stdout=open(work / "core_main.log", "w"),
                                      stderr=subprocess.STDOUT))
        procs.append(subprocess.Popen(
            [str(args.ardupilot / "build/sitl/bin/arducopter"), "--model", "+", "--wipe", "--defaults",
             f"{args.ardupilot / 'Tools/autotest/default_params/copter.parm'},{here / 'link_jam.parm'}",
             f"--serial1=udpclient:127.0.0.1:{CORE_PORT}", f"--serial2=udpclient:127.0.0.1:{PROXY_FC_PORT}"],
            cwd=work, stdout=open(work / "arducopter.log", "w"), stderr=subprocess.STDOUT))
        observer = Observer()
        operator = Operator()
        results = run(proxy, observer)
        ok = True
    except Failed as e:
        print(f"FAILED: {e}")
        ok = False
    finally:
        for helper in (observer, operator):
            if helper is not None:
                helper.running = False
        time.sleep(0.3)
        proxy.stop()
        for p in procs:
            p.terminate()
            p.wait(timeout=10)
        print("\nsummary")
        for r in results:
            print("  " + r.summary())
        print(f"proxy: {proxy.stats['relayed']} relayed, {proxy.stats['dropped']} dropped")
        print(f"logs in {work}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
