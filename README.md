# Trust Issues

A drone survivability layer and mission-control dashboard. Detect spoofing, monitor
subsystem trust, and keep flying when the world stops being trustworthy.

## Structure

- `core/` — C implementation of the drone survivability layer (GNSS spoofing detection, lost-link mitigation, autonomy, link encryption). See `core/README.md`.
- `dashboard/` — operator dashboard, separate project/framework. See `dashboard/README.md`.

## Run the local simulator

With a built ArduCopter SITL checkout and a Python environment containing
`pymavlink`, MAVProxy and `psutil`:

```sh
ARDUPILOT_DIR=/path/to/ardupilot ARDUPILOT_VENV=/path/to/venv ./run.sh
```

This builds core and the dashboard, starts real SITL with the GNSS and lost-link
detectors and autonomy, and opens a MAVProxy console (MAVProxy is the operator
whose heartbeats the lost-link detector watches). Open **http://127.0.0.1:8080**, upload a
mission, then use **Start flight**. The live threat controls inject faults into
SITL, not browser-generated trust scores. The simulator is not real hardware.
Exit MAVProxy or press Ctrl+C to stop this run's processes; logs stay in the
printed run directory. See `./run.sh --help` for alternate ports/instances.

For repeatable spoof and unspoofed-control scenarios, see
[`core/README.md`](core/README.md#end-to-end-gnss-scenarios); for operator-link
loss and jamming driving autonomy's hold and return, see
[`core/README.md`](core/README.md#end-to-end-link-scenarios).

## Real ArduPilot simulation (SITL)

```sh
./scripts/sitl
```

Open **http://127.0.0.1:8080**. This builds and starts ArduCopter SITL, the real C
GNSS/link detectors, and the dashboard in **live mode**, not its browser demo.
The vehicle starts disarmed. In a second terminal:

```sh
./scripts/sitl square   # take off, fly a 20 m square at 10 m, then land
# Or upload your route from the dashboard's Plan tab, then:
./scripts/sitl start    # arm and start the uploaded mission
./scripts/sitl land     # request landing
./scripts/sitl status   # print live telemetry
```

Ctrl+C in the launcher terminal stops the whole local stack. Each run uses fresh
SITL state; logs are kept in `core/build/dashboard-sitl/session-*/`.

The launcher reuses `../ardupilot-sitl/ardupilot` (already built locally) and
`../ardupilot-sitl/venv/bin/python` (with `pymavlink`). Elsewhere, set
`ARDUPILOT_DIR` to a built ArduPilot checkout and `SITL_PYTHON` to a Python with
`pymavlink`. Also requires CMake, a C compiler, Make and Go 1.24+.
Use `./scripts/sitl --no-build` after the first build for faster startup.

To run core on a flashed ESP32 plugged in over USB instead of as `core_main` on
this computer, pass its serial device: `./scripts/sitl --esp32
/dev/cu.usbserial-0001`. See
[`core/README.md`](core/README.md#running-core-on-an-esp32) to build and flash it.

Ports: dashboard HTTP `8080`, core UDP `14551`, live bridge UDP `14552`, flight
scripts UDP `14553`, primary operator TCP `5760`. These are local simulation
connections; do not point the flight commands at hardware.

`core_main` runs the GNSS and lost-link detectors and autonomy together; the
launcher itself is the operator (sysid 255) whose heartbeats the lost-link
detector watches. Missing motor/hardware readings remain missing rather than
being filled with demo scores. The map is a telemetry view, not a 3D simulator.
