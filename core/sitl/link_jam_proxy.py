#!/usr/bin/env python3
"""Stands in for the operator's telemetry radio, so the link can be jammed or lost in SITL.

Sits between ArduPilot SITL and the operator's ground station. It relays traffic
both ways, dropping a chosen fraction of packets in both directions, and, like
the aircraft-side radio of a SiK pair, reports RADIO_STATUS to the autopilot
once a second. Those reports are injected on the autopilot side and are never
dropped, because that radio is wired to the flight controller.

    SITL --serial2=udpclient:127.0.0.1:14560   (proxy's autopilot side)
    mavproxy.py --master=udpout:127.0.0.1:14562 (proxy's operator side)
    link_jam_proxy.py

Set RSSI_TYPE=5 on the vehicle (see link_jam.parm) so ArduPilot passes the
radio's RSSI on to the companion computer in RC_CHANNELS.

Type a profile name and Enter to switch: nominal, degraded, jam, lost. Or
"drop 0.3", "rssi 120", "noise 90" to set one value.

What each profile models is an assumption about radio behaviour, not a
measurement: jamming raises the noise floor while RSSI stays high, a clean loss
leaves RSSI at the floor with a normal noise level.
"""
import argparse
import os
import random
import select
import socket
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402

# (drop fraction, rssi, noise), in SiK units
PROFILES = {
    "nominal": (0.0, 180, 40),
    "degraded": (0.35, 90, 45),
    "jam": (1.0, 200, 160),
    "lost": (1.0, 8, 40),
}

SIK_SYSID, SIK_COMPID = 51, 68  # what a SiK radio identifies as


class LinkJamProxy:
    def __init__(self, fc_port=14560, gcs_port=14562, profile="nominal", seed=None):
        self.fc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fc_sock.bind(("127.0.0.1", fc_port))
        self.gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gcs_sock.bind(("127.0.0.1", gcs_port))
        self.fc_addr = None
        self.gcs_addr = None
        self.rng = random.Random(seed)
        self.lock = threading.Lock()
        self.drop, self.rssi, self.noise = PROFILES[profile]
        self.ramp = None  # (start_time, seconds, from, to)
        self.rxerrors = 0
        self.mav = mavlink.MAVLink(None, srcSystem=SIK_SYSID, srcComponent=SIK_COMPID)
        self.stats = {"relayed": 0, "dropped": 0}
        self.running = True
        self.threads = [threading.Thread(target=self._relay, daemon=True),
                        threading.Thread(target=self._report, daemon=True)]

    def start(self):
        for t in self.threads:
            t.start()
        return self

    def stop(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=2)
        self.fc_sock.close()
        self.gcs_sock.close()

    def set(self, drop=None, rssi=None, noise=None, ramp_s=0.0):
        """Changes the link, at once or linearly over ramp_s seconds."""
        with self.lock:
            now = self._values()
            target = (now[0] if drop is None else drop,
                      now[1] if rssi is None else rssi,
                      now[2] if noise is None else noise)
            if ramp_s > 0:
                self.ramp = (time.monotonic(), ramp_s, now, target)
            else:
                self.ramp = None
                self.drop, self.rssi, self.noise = target

    def profile(self, name, ramp_s=0.0):
        drop, rssi, noise = PROFILES[name]
        self.set(drop, rssi, noise, ramp_s)

    def _values(self):
        if self.ramp is None:
            return (self.drop, self.rssi, self.noise)
        start, seconds, a, b = self.ramp
        f = min(1.0, (time.monotonic() - start) / seconds)
        values = tuple(x + (y - x) * f for x, y in zip(a, b))
        if f >= 1.0:
            self.ramp = None
            self.drop, self.rssi, self.noise = values
        return values

    def values(self):
        with self.lock:
            return self._values()

    def _relay(self):
        while self.running:
            ready, _, _ = select.select([self.fc_sock, self.gcs_sock], [], [], 0.2)
            for sock in ready:
                try:
                    data, addr = sock.recvfrom(65535)
                except OSError:
                    continue
                if sock is self.fc_sock:
                    self.fc_addr, out, dest = addr, self.gcs_sock, self.gcs_addr
                else:
                    self.gcs_addr, out, dest = addr, self.fc_sock, self.fc_addr
                if dest is None:
                    continue
                drop = self.values()[0]
                if self.rng.random() < drop:
                    self.stats["dropped"] += 1
                    self.rxerrors = (self.rxerrors + 1) & 0xFFFF
                    continue
                self.stats["relayed"] += 1
                out.sendto(data, dest)

    def _report(self):
        while self.running:
            time.sleep(1.0)
            if self.fc_addr is None:
                continue
            _, rssi, noise = self.values()
            msg = mavlink.MAVLink_radio_status_message(
                rssi=int(rssi), remrssi=int(rssi), txbuf=100, noise=int(noise),
                remnoise=int(noise), rxerrors=self.rxerrors, fixed=0)
            try:
                self.fc_sock.sendto(msg.pack(self.mav), self.fc_addr)
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fc-port", type=int, default=14560, help="port SITL's operator serial sends to")
    parser.add_argument("--gcs-port", type=int, default=14562, help="port the ground station connects to")
    parser.add_argument("--profile", choices=PROFILES, default="nominal")
    parser.add_argument("--ramp", type=float, default=5.0, help="seconds to ramp between profiles")
    args = parser.parse_args()

    proxy = LinkJamProxy(args.fc_port, args.gcs_port, args.profile).start()
    print(f"relaying udp {args.fc_port} (autopilot) <-> {args.gcs_port} (operator), profile {args.profile}")
    try:
        for line in sys.stdin:
            words = line.split()
            if not words:
                continue
            if words[0] in PROFILES:
                proxy.profile(words[0], args.ramp)
            elif len(words) == 2 and words[0] in ("drop", "rssi", "noise"):
                proxy.set(**{words[0]: float(words[1])}, ramp_s=args.ramp)
            else:
                print("profiles:", ", ".join(PROFILES), "| or: drop <0..1>, rssi <0..254>, noise <n>")
                continue
            drop, rssi, noise = proxy.values()
            print(f"-> drop {drop:.2f} rssi {rssi:.0f} noise {noise:.0f} (ramping {args.ramp:.0f} s)")
    except KeyboardInterrupt:
        pass
    proxy.stop()


if __name__ == "__main__":
    main()
