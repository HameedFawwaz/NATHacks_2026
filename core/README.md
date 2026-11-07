# core

C implementation of the drone survivability layer: GNSS spoofing detection, lost-link
mitigation, autonomous failover decisions, and operator-link encryption. Runs on the
companion computer and talks to the flight controller over MAVLink.

## Layout

```
core/
  include/core/      public headers, one subdir per module
    mavlink/          MAVLink helper functions / connection handling (shared by all modules)
    nav/              GNSS spoofing detection
    link/             lost-link detection and failover
    autonomy/         decision layer consuming trust scores from nav/link/hardware
    crypto/           operator<->drone link encryption/signing
    common/           shared types (e.g. trust score struct) and utilities
  src/                implementation, mirrors include/core/
  third_party/        vendored deps (MAVLink C headers, see third_party/README.md)
  tests/              automated tests (no external dependencies, e.g. loopback UDP)
  tools/              manual dev executables that need something external running,
                      e.g. mavlink_monitor against a live ArduPilot SITL instance
  sitl/               SITL spoof/jam injection scripts and scenario configs
```

## Build

```
git submodule update --init --recursive   # first time, pulls in MAVLink headers
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`mavlink/` and `common/` are implemented (connection handling, telemetry decode, send
helpers, shared trust-score struct — see `include/core/mavlink/mavlink.h` for the public
API). `nav/` (GNSS spoofing detection) is implemented — see below. `link/`, `autonomy/`,
`crypto/` are still empty, scoped to their own issues.

## GNSS spoofing detection (`nav/`)

Maintains an independent position estimate (a Kalman filter fusing accelerometer, gyro,
and barometer — never GPS) and cross-checks incoming GPS against it. Six checks (jump,
short- and long-horizon KF divergence, raw-GPS-vs-EKF divergence, timestamp/replay, HDOP
abnormality) each produce a sub-score in `[0,1]`, fused via `min()` so one unambiguous red
flag can't be averaged away, then published as `nav_trust` with asymmetric slew limiting
(drops instantly, recovers slowly). See `include/core/nav/nav.h` for the public API,
`include/core/nav/gnss_anomaly.h` for the six checks, and `core/tests/test_gnss_anomaly.c`
/ `test_nav_kf.c` for fully synthetic (no SITL needed) scenario and filter-math tests.

`core_main` is the real integration loop: connect → detect → publish, against a live
flight controller.

**Known limitation**: the long-horizon KF divergence check was calibrated against real
flight (`core/sitl/fly_square.py`, not just synthetic/stationary data) and can still
occasionally false-positive for a few seconds during normal maneuvering. Root cause:
this filter has no accel/gyro bias calibration, and in testing only received ~4Hz IMU
samples (SITL/MAVProxy didn't honor our `MAV_CMD_SET_MESSAGE_INTERVAL` request to speed
`SCALED_IMU2` up to 50Hz — see `core_mavlink_request_message_interval` in
`core/include/core/mavlink/mavlink_send.h`), so integration error compounds faster than a
properly-sampled filter would see. Thresholds were loosened and the long horizon shortened
(15s → 8s) based on measured real-flight divergence, which shortened the false-positive
window substantially but didn't eliminate it. Follow-up: figure out why the rate request
isn't taking effect (possibly a MAVProxy pass-through quirk, worth checking against a
direct SITL connection without MAVProxy in between), or add basic bias estimation.

## Testing against ArduPilot SITL

`tools/mavlink_monitor` is a standalone connectivity check: it opens a MAVLink
connection and prints the heartbeat plus whatever other telemetry has been decoded
(GPS, position, attitude, radio status) each time a new heartbeat arrives.

1. Start ArduPilot SITL with an extra MAVLink output pointed at this tool
   (14550 is conventionally already used by MAVProxy's own GCS link, so add a
   second `--out` for the companion computer):
   ```
   sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
   ```
2. Run the monitor against that same port (14551 is the default if omitted):
   ```
   ./build/tools/mavlink_monitor 14551
   ```
   You should see a `[HEARTBEAT]` line (plus GPS/position/attitude once SITL has a
   fix) roughly once per second. Ctrl+C to stop.

### Running the real detector, and injecting spoofing scenarios

`core_main [port]` (default 14551) is the real GNSS spoofing detector: point it at SITL
the same way as `mavlink_monitor` above, and it prints `nav_trust` plus each sub-score
once a second. With no attack running, `nav_trust` should stay at/near 1.0.

To inject a scripted attack on top of a live SITL instance, run
`core/sitl/gnss_spoof_proxy` between SITL and `core_main` — it relays everything
transparently except `GPS_RAW_INT`, which it mutates per `--scenario`:

```
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
./build/sitl/gnss_spoof_proxy --scenario=jump --out-port=14552      # or drift / replay
./build/core_main 14552
```

- `--scenario=jump` (`--jump-at-sec`, `--jump-offset-m`, `--jump-heading-deg`): one
  fixed-offset teleport, held afterward. `nav_trust` should drop to 0 immediately.
- `--scenario=drift` (`--drift-rate-mps`, `--drift-heading-deg`, `--drift-start-sec`):
  self-consistent position+velocity drift. `nav_trust` decays smoothly as divergence
  accumulates.
- `--scenario=replay` (`--replay-at-sec`, `--replay-buffer-sec`, `--replay-mode`):
  loops a buffered window of real past fixes. `looped-live-timestamp` re-stamps time to
  look fresh (caught by KF divergence - needs the vehicle actually moving to show a
  signal); `stale-timestamp` leaves the original timestamps in place (caught by the
  replay/monotonicity check once the loop wraps - a smaller `--replay-buffer-sec` makes
  this wrap, and the detection, more frequent/visible in a live demo).

In every case, watch for the EKF-divergence sub-score (`ekf=`) - since the proxy only
mutates `GPS_RAW_INT`, the flight controller's own `GLOBAL_POSITION_INT` keeps reflecting
ground truth, making that check the most durable signal for a sustained attack.

### Flying the vehicle for a real (non-stationary) test

A stationary SITL instance doesn't exercise the IMU-based filter or the KF-divergence
checks (nothing is moving for them to track) — some scenarios above only show a real
signal with the vehicle actually flying. `core/sitl/fly_square.py` (Python + pymavlink)
arms, takes off, and flies an autonomous square via GUIDED-mode
`SET_POSITION_TARGET_LOCAL_NED` commands:

```
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553
python3 core/sitl/fly_square.py                     # defaults to udpin:0.0.0.0:14553
```

It talks directly to the vehicle (arm/mode/takeoff/position commands), separately from
`core_main`/`gnss_spoof_proxy`, which only observe telemetry — run it independently,
before or during a `core_main` session on a different port, with no conflict.

## Dashboard

The operator dashboard is a separate project/framework and lives outside `core/`
(see `../dashboard`). It consumes trust scores and telemetry published by this
codebase over MAVLink; it does not share a build system with it.
