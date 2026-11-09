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

This builds core and the dashboard, starts real SITL with the GNSS detector and
autonomy, and opens a MAVProxy console. Open **http://127.0.0.1:8080**, upload a
mission, then use **Start flight**. The live threat controls inject faults into
SITL, not browser-generated trust scores. The simulator is not real hardware.
Exit MAVProxy or press Ctrl+C to stop this run's processes; logs stay in the
printed run directory. See `./run.sh --help` for alternate ports/instances.

For repeatable spoof and unspoofed-control scenarios, see
[`core/README.md`](core/README.md#end-to-end-gnss-scenarios).
