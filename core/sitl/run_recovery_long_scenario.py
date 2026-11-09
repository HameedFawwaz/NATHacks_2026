#!/usr/bin/env python3
"""Long moving-route recovery, landing, ground reset and re-arm regression."""
import argparse
import json
from pathlib import Path
import tempfile
import time

import run_gnss_e2e_scenarios as e2e
from run_recovery_e2e_scenarios import RecoverySitl, ROOT, post, wait_snapshot


def fly_window(sitl, route, seconds, samples):
    end = time.monotonic() + seconds
    last = 0
    while time.monotonic() < end:
        route.step(0.2, sitl.status["true_pos"])
        sitl.pump(0.2)
        if time.monotonic() - last >= 1:
            samples.append(e2e.dashboard_snapshot(sitl.dashboard_url))
            last = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--ground-reset-case", action="store_true",
                        help="leave automatic responses disabled after the attack, then recover via ground reset")
    parser.add_argument("--instance", type=int, default=23)
    parser.add_argument("--core-port", type=int, default=15965)
    parser.add_argument("--dashboard-port", type=int, default=15966)
    parser.add_argument("--dashboard-http-port", type=int, default=18193)
    args = parser.parse_args()
    build = ROOT / "core/build"
    work = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="recovery-long-", dir=build))
    work.mkdir(parents=True, exist_ok=True)
    print(f"Logs: {work}", flush=True)
    sitl = None
    result = {"attack_samples": [], "clear_samples": []}
    try:
        sitl = RecoverySitl(args.ardupilot.resolve(), build / "core_main", work, args.core_port,
                            args.instance, build / "recovery-dashboard-serve", args.dashboard_port, args.dashboard_http_port)
        wait_snapshot(sitl, lambda s: s.get("RecoveryControllable"), 10, "core controls ready")
        post(sitl, "/api/recovery", {"enabled": False})
        sitl.take_off()
        route = e2e.SquareRoute(sitl, side_m=100)
        fly_window(sitl, route, 15, [])
        post(sitl, "/api/fault", {"fault": "gps_drift"})
        fly_window(sitl, route, 8, result["attack_samples"])
        post(sitl, "/api/recovery", {"enabled": True})
        wait_snapshot(sitl, lambda s: s["Behavior"] == "GNSS_FAILOVER", 10, "mid-attack backup response")
        fly_window(sitl, route, 82, result["attack_samples"])
        assert result["attack_samples"][-1]["Nav"]["Value"] < 0.4, "ongoing long attack was forgiven"
        if args.ground_reset_case:
            post(sitl, "/api/recovery", {"enabled": False})
        post(sitl, "/api/fault", {"fault": "none"})
        began = time.monotonic()
        recovered = False
        for _ in range(15 if args.ground_reset_case else 60):
            fly_window(sitl, route, 1, result["clear_samples"])
            s = result["clear_samples"][-1]
            if s["Nav"]["Value"] > 0.99 and s["Behavior"] == "NOMINAL":
                recovered = True
                result["recovery_seconds"] = round(time.monotonic() - began, 2)
                break
        result["inflight_recovered"] = recovered
        print(f"Long-route in-flight recovery: {recovered}", flush=True)
        sitl.command(e2e.mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 9)
        sitl.wait_for(lambda s: not s["armed"], 100, "land and disarm")
        sitl.pump(5)
        result["before_reset"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        if args.ground_reset_case:
            assert result["before_reset"]["Nav"]["Value"] < .4, "ground-reset case did not reproduce stale untrusted state"
        home = sitl.status["home"]
        waypoints = []
        for east, north in ((30, 0), (30, 30)):
            lat, lon = e2e.from_local(home, east, north)
            waypoints.append({"lat": lat, "lon": lon, "alt_m": 20})
        post(sitl, "/api/mission", {"home": {"lat": home[0], "lon": home[1]}, "waypoints": waypoints})
        post(sitl, "/api/recovery", {"enabled": False})
        post(sitl, "/api/reset", {"confirm": True})
        result["after_reset"] = wait_snapshot(sitl, lambda s: not s["Armed"] and s["Nav"]["Value"] > .99,
                                              30, "ground reset revalidates NAV without arming")
        assert result["after_reset"]["MissionReady"], "reset discarded the uploaded mission"
        assert not result["after_reset"]["RecoveryEnabled"], "reset silently enabled automatic responses"
        assert result["after_reset"]["Behavior"] == "NOMINAL", "reset retained a previous-flight decision latch"
        # Re-arm using normal prearm checks, never force-arm or disable checks.
        deadline = time.monotonic() + 60
        while not sitl.status["armed"] and time.monotonic() < deadline:
            sitl.command(e2e.mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4)
            sitl.command(e2e.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            sitl.pump(2)
        assert sitl.status["armed"], "EKF/prearm state never recovered after landing/reset"
        result["rearmed_with_normal_checks"] = True
        sitl.command(e2e.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        sitl.wait_for(lambda s: not s["armed"], 10, "disarm on the ground")
        if not args.ground_reset_case:
            assert recovered, "ground reset works, but long-route automatic recovery still needs investigation"
        result["passed"] = True
        print("PASS long moving attack -> clear -> recover -> land -> reset -> normal re-arm", flush=True)
    finally:
        (work / "results.json").write_text(json.dumps(result, indent=2))
        if sitl is not None:
            sitl.close()


if __name__ == "__main__":
    main()
