# Trust Issues

A drone survivability layer and mission-control dashboard. GPS spoofing, jammed or
lost control links, and attacks on the operator connection don't have to end a flight:
`core` detects them in real time and autonomy decides how to respond, while the
dashboard shows the operator exactly what's trusted, what isn't, and why.

`core` is a small C library built to run anywhere a companion computer could go,
including [an ESP32 wired straight to a flight controller's telemetry port](core/README.md#running-core-on-an-esp32)
— not just a laptop. Everything here runs against real ArduPilot SITL, not a scripted
demo: the detectors see the simulator's actual GPS, IMU and link telemetry, and
autonomy sends its EKF source-set, hold, return and land commands back over real
MAVLink, the same way it would to a real flight controller.

## Structure

- `core/` — C implementation of the drone survivability layer: GNSS spoofing
  detection, lost-link detection, autonomy (the decision layer that acts on both),
  and operator-link encryption. Runs as `core_main` on a computer or as firmware on
  an ESP32. See [`core/README.md`](core/README.md).
- `dashboard/` — operator dashboard, written in Go and compiled to WebAssembly; a
  separate project/framework from `core`. See [`dashboard/README.md`](dashboard/README.md).
- `run.sh` / `scripts/sitl` — the two ways to run the whole stack locally, below.

## Running it

Both of the following start real ArduCopter SITL, `core`'s real GNSS and lost-link
detectors with autonomy, and the dashboard in **live mode** (not its browser demo).
The vehicle always starts disarmed; nothing here points at real hardware.

Prerequisites either way: CMake, a C compiler, Make, Go 1.24+, a built ArduCopter
SITL checkout (`./waf configure --board sitl && ./waf copter`), and a Python
environment with `pymavlink` (`scripts/sitl` also wants MAVProxy and `psutil`).

### `scripts/sitl` — scripted flight, recommended for a first run

```sh
./scripts/sitl
```

Builds core and the dashboard, starts everything, and waits for GPS lock and first
telemetry before printing `Ready: http://127.0.0.1:8080`. Open that, upload a plan
on the **Plan** tab, then in a second terminal:

```sh
./scripts/sitl square   # take off, fly a 20 m square at 10 m, then land
./scripts/sitl start    # or: arm and start the plan you uploaded
./scripts/sitl land     # request landing
./scripts/sitl status   # print live telemetry as JSON
```

Ctrl+C in the first terminal stops the whole stack; each run starts from fresh SITL
state, and logs land in `core/build/dashboard-sitl/session-*/`.

It defaults to a sibling `../ardupilot-sitl/ardupilot` checkout and
`../ardupilot-sitl/venv/bin/python`. Elsewhere, set `ARDUPILOT_DIR` to your SITL
checkout and `SITL_PYTHON` to a Python with `pymavlink` (or pass `--ardupilot`).
Use `--no-build` after the first run to skip rebuilding. Repeatable spoof/control
and link-loss/jamming scenario scripts, and what they measured on real SITL runs,
are documented in `core/README.md`'s
[End-to-end GNSS scenarios](core/README.md#end-to-end-gnss-scenarios) and
[End-to-end link scenarios](core/README.md#end-to-end-link-scenarios).

**Core on a flashed ESP32 instead of this computer:** pass its serial device,
e.g. `./scripts/sitl --esp32 /dev/ttyUSB0` (Linux) or `--esp32 /dev/cu.usbserial-0001`
(macOS). The board becomes the real companion computer, reading SITL's simulated
telemetry over its own USB-UART and commanding the recovery itself. See
[`core/README.md`](core/README.md#running-core-on-an-esp32) to build and flash it
first — the "Results" there compare the board's detection times and drift against
`core_main`'s on the same scenarios.

### `run.sh` — manual flying with a MAVProxy console

```sh
ARDUPILOT_DIR=/path/to/ardupilot ARDUPILOT_VENV=/path/to/venv ./run.sh
```

Same stack, but drops you at an interactive MAVProxy console instead of scripting
the flight — useful for setting SITL parameters by hand (`param set
SIM_GPS_GLITCH_X 0.001` for an instant ~110 m GPS jump) or flying manually. MAVProxy
is also the operator whose heartbeats the lost-link detector watches. Exit MAVProxy
or press Ctrl+C to stop; logs stay in the printed run directory. `./run.sh --help`
lists the port/instance flags for running more than one copy at once. This doesn't
support the ESP32 — use `scripts/sitl --esp32` for that.

The dashboard's own threat simulator (Fly tab, live mode) injects the same kinds of
faults into SITL directly — GPS jump/drift, link jamming, a motor fault — without
needing a MAVProxy console at all; see `dashboard/README.md`'s
[Threat simulator](dashboard/README.md#threat-simulator-sitl-only).

## What's implemented

- **GNSS spoofing detection** (`core/nav`): six independent checks (position jump,
  two dead-reckoning horizons, GPS-vs-EKF divergence, replay/timestamp, HDOP) fused
  so one clear red flag can't be averaged away, published as a slew-limited
  `nav_trust`, with gated, carefully-bounded reacquisition once GPS looks honest
  again. `core` publishes its own independent position estimate alongside it.
- **Lost-link detection** (`core/link`): judges the operator's control link from
  heartbeat loss and radio RSSI, distinguishing jamming from a clean loss, since
  the companion computer isn't on that link itself.
- **Autonomy** (`core/autonomy`): turns both trust scores into a response — EKF
  source failover, a hold, a return home, or landing — and commands it over MAVLink.
- **Operator-link encryption** (`core/crypto`): MAVLink 2 message signing between
  dashboard and vehicle.
- **Dashboard**: live map, per-subsystem health, automatic-recovery on/off and a
  ground-only reset, a plan editor, and (SITL only) a threat simulator for every
  fault above.
- **Real hardware**: the same detection-and-autonomy loop also runs as ESP32
  firmware, standing in for a real companion computer wired to a flight
  controller's telemetry port.

Everything above runs against real ArduPilot SITL in CI-less, repeatable scripts —
see the two `core/README.md` sections linked above for exact commands and measured
results, including what each detector still can't catch.
