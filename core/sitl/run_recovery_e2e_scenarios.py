#!/usr/bin/env python3
"""Verify real recovery toggles and fault-clear reacquisition on isolated SITL."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import time
import urllib.request

import run_gnss_e2e_scenarios as e2e
from run_dashboard_e2e_scenario import OperatorSitl, ROOT


class RecoverySitl(OperatorSitl):
    def _launch(self, command, workdir, name):
        if name == "core_main":
            command = [*command, "--allow-recovery-control", "--gps-free-backup"]
        return super()._launch(command, workdir, name)


def post(sitl, endpoint, body):
    def request():
        req = urllib.request.Request(sitl.dashboard_url + endpoint,
                                     data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=12) as response:
            return json.load(response)
    # Operator heartbeats must not stop while an HTTP handler awaits core/FC.
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(request)
        while not future.done():
            sitl.pump(0.1)
        return future.result()


def wait_snapshot(sitl, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sitl.pump(0.2)
        snap = e2e.dashboard_snapshot(sitl.dashboard_url)
        if condition(snap):
            print(f"PASS {label}: nav={snap['Nav']['Value']:.3f}, behavior={snap['Behavior']}, "
                  f"enabled={snap['RecoveryEnabled']}", flush=True)
            return snap
    raise AssertionError(f"Timed out: {label}; last snapshot {snap}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=22)
    parser.add_argument("--core-port", type=int, default=15865)
    parser.add_argument("--dashboard-port", type=int, default=15866)
    parser.add_argument("--dashboard-http-port", type=int, default=18192)
    parser.add_argument("--log-dir", type=Path)
    args = parser.parse_args()
    build = ROOT / "core/build"
    work = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="recovery-e2e-", dir=build))
    work.mkdir(parents=True, exist_ok=True)
    print(f"Logs: {work}", flush=True)
    sitl = None
    results = {}
    try:
        sitl = RecoverySitl(args.ardupilot.resolve(), build / "core_main", work, args.core_port,
                            args.instance, build / "recovery-dashboard-serve", args.dashboard_port,
                            args.dashboard_http_port)
        sitl.take_off()
        wait_snapshot(sitl, lambda s: s.get("RecoveryControllable") and s.get("RecoveryEnabled"), 5, "core reports enabled")
        post(sitl, "/api/recovery", {"enabled": False})
        wait_snapshot(sitl, lambda s: not s["RecoveryEnabled"], 3, "disabled acknowledged by core")
        post(sitl, "/api/fault", {"fault": "gps_jump"})
        wait_snapshot(sitl, lambda s: s["Nav"]["Value"] < 0.4, 10, "detection remains active while disabled")
        sitl.pump(8)
        results["disabled_attack"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        assert results["disabled_attack"]["Behavior"] == "NOMINAL", results
        assert "EKF source set 2 active" not in sitl.core_log.read_text(), "disabled system switched navigation"
        post(sitl, "/api/recovery", {"enabled": True})
        wait_snapshot(sitl, lambda s: s["RecoveryEnabled"] and s["Behavior"] == "GNSS_FAILOVER", 10,
                      "enabling causes actual failover")
        sitl.pump(10)
        assert "EKF source set 2 active" in sitl.core_log.read_text(), "FC never acknowledged backup"
        assert "GNSS recovery: backup agreement verified" not in sitl.core_log.read_text(), "ongoing attack was forgiven"
        results["enabled_attack"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        post(sitl, "/api/fault", {"fault": "none"})
        results["just_cleared"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        assert results["just_cleared"]["Nav"]["Value"] < 0.7, "All clear forced trust healthy"
        results["jump_recovered"] = wait_snapshot(sitl, lambda s: s["Nav"]["Value"] > 0.99 and s["Behavior"] == "NOMINAL",
                                                   60, "honest GPS reacquired after jump removed")
        assert "EKF source set 1 active" in sitl.core_log.read_text(), "primary GPS not acknowledged again"

        post(sitl, "/api/fault", {"fault": "gps_drift"})
        wait_snapshot(sitl, lambda s: s["Nav"]["Value"] < 0.4 and s["Behavior"] == "GNSS_FAILOVER", 35,
                      "second drift attack triggers failover")
        sitl.pump(10)
        results["drift_attack"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        assert results["drift_attack"]["Nav"]["Value"] < 0.4
        post(sitl, "/api/fault", {"fault": "none"})
        results["drift_recovered"] = wait_snapshot(sitl, lambda s: s["Nav"]["Value"] > 0.99 and s["Behavior"] == "NOMINAL",
                                                    60, "honest GPS reacquired after drift removed")
        print("PASS core-controlled local SITL recovery control/clear sequence", flush=True)
    finally:
        (work / "results.json").write_text(json.dumps(results, indent=2))
        if sitl is not None:
            sitl.close()


if __name__ == "__main__":
    main()
