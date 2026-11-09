#!/usr/bin/env python3
"""Local ArduCopter acceptance: real operator heartbeats, estimator and browser.

Requires built core_main/dashboard-serve/WASM, pymavlink, Node 22+ and Chrome.
Flies only a newly launched SITL instance; never connects to physical hardware.
Browser requests are read-only. All processes and profiles are isolated.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time

import run_gnss_e2e_scenarios as e2e

ROOT = Path(__file__).resolve().parents[2]


class OperatorSitl(e2e.Sitl):
    def __init__(self, *args, **kwargs):
        self.last_operator_heartbeat = 0
        super().__init__(*args, **kwargs)

    def _launch(self, command, workdir, name):
        if name == "dashboard":
            command = [*command, "-dir", str(ROOT / "dashboard/web")]
        return super()._launch(command, workdir, name)

    def pump(self, seconds=0.0):
        end = time.monotonic() + seconds
        while True:
            now = time.monotonic()
            if now - self.last_operator_heartbeat >= 1:
                self.mav.mav.heartbeat_send(e2e.mavutil.mavlink.MAV_TYPE_GCS,
                                           e2e.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.last_operator_heartbeat = now
            if seconds <= 0:
                return super().pump(0)
            remaining = end - now
            if remaining <= 0:
                return
            super().pump(min(0.1, remaining))


def capture(sitl, work, name, advance_spoof=None):
    out = work / name
    with open(work / f"{name}-browser.log", "w") as log:
        child = subprocess.Popen(["node", str(ROOT / "dashboard/web/dashboard-smoke.cjs"),
                                  "--mode", "live", "--url", sitl.dashboard_url, "--out", str(out)],
                                 cwd=ROOT / "dashboard", stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 40
        last_injection = time.monotonic()
        try:
            while child.poll() is None:
                if time.monotonic() >= deadline:
                    raise e2e.ScenarioFailed("browser capture timed out")
                sitl.pump(0.1)
                now = time.monotonic()
                if advance_spoof is not None and now - last_injection >= 0.5:
                    advance_spoof(now - last_injection, sitl.status["true_pos"])
                    last_injection = now
            if child.returncode:
                raise e2e.ScenarioFailed((work / f"{name}-browser.log").read_text())
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
    state = json.loads((out / "live-state.json").read_text())
    assert (out / "live-readonly.png").is_file()
    assert state["source"] == "Live" and state["plannedPixels"] > 100, state
    assert state["map"] == "Offline grid · map tiles unavailable", state
    assert any(c["name"] == "Link" and c["score"] == "100%" for c in state["cards"]), state
    if name == "slow-spoof-live":
        assert state["driftPixels"] > 50 and float(state["drift"].split()[0]) >= 10, state
        assert any(c["name"] == "GNSS" and ("Caution" in c["level"] or "Critical" in c["level"])
                   for c in state["cards"]), state
    if name == "critical-live":
        assert any(c["name"] == "GNSS" and "Critical" in c["level"] for c in state["cards"]), state
    print(f"PASS real SITL browser capture: {out / 'live-readonly.png'}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=18)
    parser.add_argument("--core-port", type=int, default=15465)
    parser.add_argument("--dashboard-port", type=int, default=15466)
    parser.add_argument("--dashboard-http-port", type=int, default=18188)
    parser.add_argument("--log-dir", type=Path)
    args = parser.parse_args()
    build = ROOT / "core/build"
    build.mkdir(parents=True, exist_ok=True)
    work = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="dashboard-e2e-", dir=build))
    work.mkdir(parents=True, exist_ok=True)
    print(f"Logs and screenshots: {work}", flush=True)
    sitl = None
    try:
        sitl = OperatorSitl(args.ardupilot.resolve(), build / "core_main", work, args.core_port,
                            args.instance, build / "dashboard-serve", args.dashboard_port, args.dashboard_http_port)
        sitl.take_off()
        normal = e2e.run_window(sitl, 15, None, "control integrated hover")
        assert normal["lowest_nav_trust"] > 0.99 and not normal["behaviors_entered"], normal
        assert normal["dashboard"]["core_estimate"]["core_fraction"] >= 0.8, normal
        assert normal["core_estimate"]["error_p95_m"] < 5, normal
        capture(sitl, work, "normal-live")

        ramp = e2e.RampSpoof(e2e.SLOW_RAMP_MS)
        captured = False

        def slow_step(dt, truth):
            nonlocal captured
            fake = ramp.step(dt, truth)
            snapshot = e2e.dashboard_snapshot(sitl.dashboard_url)
            measure = e2e.dashboard_estimate(snapshot, truth)
            if (not captured and snapshot["EstimateFrom"] == "core" and measure.get("drift_m", 0) >= 15
                    and snapshot["Nav"]["Value"] < 0.65):
                # A drift threshold isn't a trust threshold. Capture only once
                # a real warning exists, and keep both the attack and genuine
                # operator heartbeats advancing while Chrome starts.
                sitl.set_gps_glitch(fake[0] - truth[0], fake[1] - truth[1])

                def advance(seconds, current_truth):
                    nonlocal fake
                    fake = ramp.step(seconds, current_truth)
                    sitl.set_gps_glitch(fake[0] - current_truth[0], fake[1] - current_truth[1])

                capture(sitl, work, "slow-spoof-live", advance)
                captured = True
            return fake

        slow = e2e.run_window(sitl, 45, slow_step, "integrated slow ramp (~1.5 m/s)")
        assert captured, "no real core drift screenshot was possible"
        assert slow["lowest_nav_trust"] < 0.4 and "GNSS_FAILOVER" in slow["behaviors_entered"], slow
        capture(sitl, work, "critical-live")
        (work / "results.json").write_text(json.dumps({"normal": normal, "slow": slow}, indent=2))
        print(f"PASS combined estimator/autonomy/live UI acceptance: {work / 'results.json'}", flush=True)
    finally:
        if sitl is not None:
            sitl.close()


if __name__ == "__main__":
    main()
