#!/usr/bin/env python3
"""
Flies ArduCopter SITL through an autonomous square pattern via GUIDED-mode
SET_POSITION_TARGET_LOCAL_NED commands, so the GNSS spoofing detector has a
genuinely moving vehicle to test against (a stationary SITL instance doesn't
exercise the IMU-based Kalman filter or the KF-divergence checks - see
core/README.md's note on the "looped-live-timestamp" replay scenario).

This talks directly to the vehicle (arm, set mode, takeoff, fly), separately
from core_main/gnss_spoof_proxy, which only observe the telemetry stream. It
needs its own MAVProxy --out endpoint to both receive telemetry and send
commands through:

    sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553
    python3 fly_square.py                      # defaults to udpin:0.0.0.0:14553

Run this BEFORE or independently of gnss_spoof_proxy/core_main - they listen
on a different port (14551/14552) and only observe, never command, so there's
no conflict with this script controlling the vehicle directly.

Requires pymavlink (already a dependency of mavproxy.py, so it's present in
any environment that can already run ArduPilot SITL + MAVProxy).

Usage:
    fly_square.py [--connect ADDR] [--altitude M] [--side-length M] [--loops N]
"""
import argparse
import math
import time

from pymavlink import mavutil

# POSITION_TARGET_TYPEMASK: use x/y/z position only, ignore velocity,
# acceleration, yaw and yaw rate. Standard value used throughout
# ArduPilot/pymavlink GUIDED-mode examples.
TYPE_MASK_POSITION_ONLY = 0b0000111111111000

POSITION_TARGET_RESEND_INTERVAL_S = 0.5  # ArduCopter expects a refreshed
                                          # target at least every ~3s in
                                          # GUIDED mode, or it may hold/stop.
ARRIVAL_TOLERANCE_M = 1.5
LEG_TIMEOUT_S = 30.0


def send_position_target(master, x, y, z):
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POSITION_ONLY,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def fly_to(master, x, y, z, tolerance=ARRIVAL_TOLERANCE_M, timeout=LEG_TIMEOUT_S):
    """Streams a position target until the vehicle arrives (or times out)."""
    start = time.time()
    last_sent = 0.0
    while time.time() - start < timeout:
        now = time.time()
        if now - last_sent >= POSITION_TARGET_RESEND_INTERVAL_S:
            send_position_target(master, x, y, z)
            last_sent = now

        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.2)
        if msg is None:
            continue

        dist = math.sqrt((msg.x - x) ** 2 + (msg.y - y) ** 2 + (msg.z - z) ** 2)
        if dist < tolerance:
            return True

    print(f"  warning: did not reach ({x:.1f}, {y:.1f}, {z:.1f}) within {timeout}s - continuing anyway")
    return False


def wait_for_gps_fix(master, min_fix_type=3, timeout=60):
    print("waiting for GPS fix...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
        if msg and msg.fix_type >= min_fix_type:
            print(f"  got fix_type={msg.fix_type}, satellites={msg.satellites_visible}")
            return True
    raise RuntimeError("timed out waiting for GPS fix")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udpin:0.0.0.0:14553",
                         help="pymavlink connection string (default: %(default)s)")
    parser.add_argument("--altitude", type=float, default=10.0, help="takeoff altitude in meters")
    parser.add_argument("--side-length", type=float, default=20.0, help="square side length in meters")
    parser.add_argument("--loops", type=int, default=1, help="number of times to fly the square")
    args = parser.parse_args()

    print(f"connecting to {args.connect} ...")
    master = mavutil.mavlink_connection(args.connect)
    master.wait_heartbeat()
    print(f"heartbeat received from system {master.target_system} component {master.target_component}")

    wait_for_gps_fix(master)

    print("setting mode GUIDED")
    master.set_mode_apm("GUIDED")
    time.sleep(1)

    print("arming")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("armed")

    print(f"taking off to {args.altitude}m")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, args.altitude,
    )

    # Wait until close to the takeoff altitude before commanding horizontal
    # moves - LOCAL_POSITION_NED's z is NED-down, so altitude = -z.
    start = time.time()
    while time.time() - start < 30:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg and -msg.z >= args.altitude * 0.9:
            break
    else:
        print("warning: did not confirm takeoff altitude - continuing anyway")

    home_msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    home_x = home_msg.x if home_msg else 0.0
    home_y = home_msg.y if home_msg else 0.0
    target_z = -args.altitude

    side = args.side_length
    waypoints = [
        (home_x, home_y, target_z),
        (home_x + side, home_y, target_z),
        (home_x + side, home_y + side, target_z),
        (home_x, home_y + side, target_z),
        (home_x, home_y, target_z),
    ]

    for loop in range(args.loops):
        print(f"flying square, loop {loop + 1}/{args.loops}")
        for i, (x, y, z) in enumerate(waypoints):
            print(f"  leg {i + 1}/{len(waypoints)}: -> ({x:.1f}, {y:.1f}, {z:.1f})")
            fly_to(master, x, y, z)

    print("square complete, landing")
    master.set_mode_apm("LAND")
    master.motors_disarmed_wait()
    print("landed and disarmed")


if __name__ == "__main__":
    main()
