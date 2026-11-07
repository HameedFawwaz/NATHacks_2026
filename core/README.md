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
API), and so is `autonomy/` (see below). `nav/`, `link/`, `crypto/` are still empty,
scoped to their own issues.

## Autonomy

`autonomy/` turns trust scores into a response. The table of behaviors is at the top of
`include/core/autonomy/autonomy.h`. Each transition goes to stderr, to a STATUSTEXT, and
to a CSV log.

The nav and link detectors aren't merged yet, so `tools/autonomy_runner` stands in for
them: it reads scores from stdin (`nav 0.1`, `link 0`, `hw 1`, `crypto 1`) and otherwise
runs the real decision loop. Once the detectors exist they should write through
`core_trust_score_set_component()` and call `core_autonomy_tick()` from the main loop.

`sitl/run_autonomy_scenario.py` runs this end to end against ArduPilot SITL. It flies a
copter, injects an attack, and checks the vehicle's response:

```
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner gnss
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner link
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner multi
```

`--ardupilot` is a checkout with `./waf configure --board sitl && ./waf copter` already
run. The script needs `pymavlink`. GNSS failover needs a non-GPS source set on the vehicle; see
`sitl/autonomy_failover.parm` for the optical-flow setup the scenario uses.

Building Copter-4.5 SITL on Apple Silicon needed three workarounds: `setuptools<81` in
the venv (`dronecan` imports `pkg_resources`, and without it the build hangs instead of
failing), `CC=/usr/bin/clang CXX=/usr/bin/clang++`, and `LDFLAGS=-Wl,-no_fixup_chains`
at `./waf configure`.

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

## Dashboard

The operator dashboard is a separate project/framework and lives outside `core/`
(see `../dashboard`). It consumes trust scores and telemetry published by this
codebase over MAVLink; it does not share a build system with it.
