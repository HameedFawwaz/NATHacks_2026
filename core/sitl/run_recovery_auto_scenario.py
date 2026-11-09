#!/usr/bin/env python3
"""Two dashboard AUTO flights without reset, with second-flight GNSS drift.

Runs only a newly launched isolated SITL. Normal prearm checks remain enabled.
"""
import argparse
import json
from pathlib import Path
import tempfile
import time
import urllib.error

import run_gnss_e2e_scenarios as e2e
from run_recovery_e2e_scenarios import RecoverySitl, ROOT, post, wait_snapshot


class DashboardDefaultsSitl(RecoverySitl):
    def _launch(self, command, workdir, name):
        if name == "arducopter":
            command = list(command)
            index = command.index("--defaults") + 1
            command[index] += "," + str(ROOT / "core/sitl/dashboard.parm")
            command += ["--home", "-35.363262,149.165237,584,353"]
        return super()._launch(command, workdir, name)

    def _set_stream_rates(self, instance, rate_hz):
        # Preserve the actual launcher's dashboard.parm, notably SR1_RAW_SENS=5.
        self.pump(2)


def upload(sitl, offsets=None, waypoints=None):
    home = sitl.status["home"]
    if waypoints is None:
        waypoints = []
        for east, north in offsets:
            lat, lon = e2e.from_local(home, east, north)
            waypoints.append({"lat": lat, "lon": lon, "alt_m": 20})
    return post(sitl, "/api/mission", {"home": {"lat": home[0], "lon": home[1]},
                                        "waypoints": waypoints})


def start(sitl):
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        try:
            response = post(sitl, "/api/start", {})
            assert response.get("started"), response
            sitl.wait_for(lambda s: s["armed"] and s["mode"] == 3, 15, "armed AUTO")
            return time.monotonic()
        except urllib.error.HTTPError as exc:
            print(f"start rejected ({exc.code}): {exc.read().decode()}", flush=True)
            if sitl.status["armed"]:
                raise
            sitl.pump(3)
    raise AssertionError("normal dashboard start never armed")


def samples(sitl, seconds, out):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        sitl.pump(min(0.5, max(0, end - time.monotonic())))
        snap = e2e.dashboard_snapshot(sitl.dashboard_url)
        if not snap["Connected"] or snap["NowMs"] - snap["Nav"]["AtMs"] > 3000:
            raise AssertionError("dashboard detector telemetry stale")
        out.append({"elapsed": round(time.monotonic(), 2), "nav": snap["Nav"]["Value"],
                    "behavior": snap["Behavior"], "mode": sitl.status["mode"],
                    "armed": sitl.status["armed"], "alt": sitl.status["rel_alt"],
                    "recovery_gps_status": snap.get("RecoveryGPSStatus"),
                    "recovery_gps_seconds": snap.get("RecoveryGPSSeconds"),
                    "recovery_gps_at_ms": snap.get("RecoveryGPSAtMs"),
                    "recovery_position_error_m": snap.get("RecoveryGPSPositionErrorM"),
                    "recovery_position_at_ms": snap.get("RecoveryGPSPositionAtMs")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=26)
    parser.add_argument("--core-port", type=int, default=16265)
    parser.add_argument("--dashboard-port", type=int, default=16266)
    parser.add_argument("--dashboard-http-port", type=int, default=18206)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--mission-file", type=Path, help="downloaded MISSION_ITEM_INT JSON, replay waypoints unchanged")
    parser.add_argument("--dashboard-serve", type=Path, help="override dashboard bridge executable")
    args = parser.parse_args()
    if args.instance < 1 or {args.core_port, args.dashboard_port, args.dashboard_http_port} & {5760, 14551, 14552, 14553, 8080}:
        parser.error("live instance ports are forbidden")
    build = ROOT / "core/build"
    work = args.log_dir.resolve() if args.log_dir else Path(tempfile.mkdtemp(prefix="recovery-auto-", dir=build))
    work.mkdir(parents=True, exist_ok=True)
    print(f"Logs: {work}", flush=True)
    mission = None
    if args.mission_file:
        items = json.loads(args.mission_file.read_text())
        mission = [{"lat": item["x"] / 1e7, "lon": item["y"] / 1e7, "alt_m": item["z"]}
                   for item in sorted(items, key=lambda x: x["seq"])
                   if item.get("mavpackettype") == "MISSION_ITEM_INT" and item["seq"] != 0
                   and item["command"] == 16 and item["x"] and item["y"]]
        if not mission:
            parser.error("mission file has no nonzero waypoint items")
    result = {"scenario": "mission-file" if mission else "600m-triangle-stress",
              "mission_file": str(args.mission_file.resolve()) if args.mission_file else None,
              "waypoints": mission, "attack_samples": [], "clear_samples": [], "rtl_samples": [], "passed": False}
    sitl = None
    try:
        sitl = DashboardDefaultsSitl(args.ardupilot.resolve(), build / "core_main", work, args.core_port,
                                     args.instance, args.dashboard_serve.resolve() if args.dashboard_serve else build / "recovery-dashboard-serve", args.dashboard_port,
                                     args.dashboard_http_port)
        sitl.wait_for(lambda s: s["home"] is not None, 90, "home position")
        wait_snapshot(sitl, lambda s: s.get("RecoveryControllable") and s.get("RecoveryEnabled"), 30, "recovery ready")
        upload(sitl, [(30, 0), (30, 30)], mission)
        start(sitl)
        sitl.wait_for(lambda s: s["rel_alt"] > 5, 60, "first AUTO takeoff")
        sitl.wait_for(lambda s: not s["armed"], 600, "first AUTO mission RTL and landing")
        result["first_landed"] = True
        sitl.pump(5)
        upload(sitl, [(600, 0), (600, 600), (0, 600)], mission)
        started = start(sitl)
        result["second_rearmed"] = True
        sitl.pump(max(0, 23 - (time.monotonic() - started)))
        assert sitl.status["armed"] and sitl.status["mode"] == 3, "second flight not in AUTO at injection"
        result["pre_injection"] = e2e.dashboard_snapshot(sitl.dashboard_url)
        assert (result["pre_injection"]["Nav"]["Value"] > .99 and
                result["pre_injection"]["Behavior"] == "NOMINAL"), "second flight not healthy before injection"
        post(sitl, "/api/fault", {"fault": "gps_drift"})
        attack_start = time.monotonic()
        samples(sitl, 57, result["attack_samples"])
        result["attack_duration"] = round(time.monotonic() - attack_start, 2)
        assert any(s["nav"] < .4 and s["behavior"] == "GNSS_FAILOVER" for s in result["attack_samples"]), "no drift failover"
        assert "EKF source set 2 active" in sitl.core_log.read_text(), "backup source not acknowledged"
        log_offset = sitl.core_log.stat().st_size
        post(sitl, "/api/fault", {"fault": "none"})
        samples(sitl, 60, result["clear_samples"])
        result["recovered"] = any(s["nav"] > .99 and s["behavior"] == "NOMINAL" for s in result["clear_samples"])
        result["primary_acknowledged"] = "EKF source set 1 active" in sitl.core_log.read_text()[log_offset:]
        print(f"Recovery after clear: nav/behavior={result['recovered']}, primary={result['primary_acknowledged']}", flush=True)
        # Mission-file case flies the uploaded route through its natural RTL.
        # For the long stress triangle, request RTL to keep the run bounded.
        if not mission:
            sitl.command(e2e.mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)
            sitl.wait_for(lambda s: s["mode"] == 6 or not s["armed"], 15, "RTL mode")
        deadline = time.monotonic() + (600 if mission else 180)
        while sitl.status["armed"] and time.monotonic() < deadline:
            samples(sitl, 1, result["rtl_samples"])
        assert not sitl.status["armed"], "second RTL landing timed out"
        result["second_landed"] = True
        result["recovered_during_rtl"] = any(s["nav"] > .99 and s["behavior"] == "NOMINAL"
                                             for s in result["rtl_samples"])
        result["primary_acknowledged_by_landing"] = "EKF source set 1 active" in sitl.core_log.read_text()[log_offset:]
        assert result["recovered"], "NAV did not recover after clear"
        assert result["primary_acknowledged"], "primary source not acknowledged after clear"
        upload(sitl, [(30, 0), (30, 30)], mission)
        start(sitl)
        result["third_rearmed"] = True
        sitl.command(e2e.mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 9)
        sitl.wait_for(lambda s: not s["armed"], 120, "third flight LAND and disarm")
        result["third_landed"] = True
        result["passed"] = True
        print("PASS two AUTO flights, drift recovery, RTL and normal rearm", flush=True)
    finally:
        (work / "results.json").write_text(json.dumps(result, indent=2))
        if sitl is not None:
            sitl.close()


if __name__ == "__main__":
    main()
