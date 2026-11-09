# Trust Issues

A drone survivability layer and mission-control dashboard. Detect spoofing, monitor
subsystem trust, and keep flying when the world stops being trustworthy.

## Structure

- `core/` — C implementation of the drone survivability layer (GNSS spoofing detection, lost-link mitigation, autonomy, link encryption). See `core/README.md`.
- `dashboard/` — operator dashboard, separate project/framework. See `dashboard/README.md`.

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

Ports: dashboard HTTP `8080`, core UDP `14551`, live bridge UDP `14552`, flight
scripts UDP `14553`, primary operator TCP `5760`. These are local simulation
connections; do not point the flight commands at hardware.

This only wires up existing functionality: `core_main` does not yet run the
autonomy decision loop, and missing motor/hardware/autonomy readings remain
missing rather than being filled with demo scores. The map is a telemetry view,
not a 3D simulator.
