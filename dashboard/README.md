# dashboard

Operator dashboard, written in Go and compiled to WebAssembly. It runs entirely in the
browser and doesn't share a build system with `../core`.

- **Fly**: live map of the planned route, GPS position and independent position
  estimate, with a drift line when those two disagree by 10 m or more. A panel shows
  the health of each subsystem in plain language.
- **Plan**: click the map to add waypoints, drag to move them, and set altitudes. The
  plan is saved in the browser's local storage.

## Run

Needs Go 1.24 or newer.

```
make run    # simulation: serves http://127.0.0.1:8080
make live   # live: also listens for MAVLink on udp 127.0.0.1:14552
make test
```

With no bridge behind it, the page runs a simulated flight. The top bar always says
which one you're looking at.

## Live mode

Point a MAVLink output at the bridge, either from MAVProxy or directly from SITL:

```
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14552
arducopter ... --serial2=udpclient:127.0.0.1:14552
```

The Plan tab's button becomes "Upload to drone". It refuses while the drone is armed,
and a plan with a leg over 5 km, which usually means it was made for somewhere else.
The dashboard doesn't arm or fly the vehicle: start the mission from your ground
control, e.g. in MAVProxy: `mode guided`, `arm throttle`, `mode auto`,
`long MISSION_START`. The Fly tab shows the last plan uploaded from the page, or the
plan being edited if there hasn't been one.

The API has no authentication, and neither does MAVLink itself until #4 is done. It
only answers to the dashboard's own address and localhost, which stops other websites
reaching it through the browser, but keep both on localhost or a trusted network.

| Dashboard field | MAVLink source |
|---|---|
| Armed, mode | `HEARTBEAT` from the autopilot |
| GPS position, satellites | `GPS_RAW_INT` |
| Position estimate, altitude, speed, heading | `GLOBAL_POSITION_INT` (see below) |
| Battery, IMU | `SYS_STATUS`; IMU health is yes or no, so that score is 0% or 100% |
| Motors | Nothing yet, so it reads "No data". ArduPilot always sets the `MOTOR_OUTPUTS` health bit, so `SYS_STATUS` can't show a failed motor; this needs ESC telemetry |
| GNSS, link and hardware scores | `TRUST_NAV`, `TRUST_LNK`, `TRUST_HW` from core |
| Autonomy behavior | core's `AUTONOMY <behavior>: <reason>` `STATUSTEXT` |
| Events | every `STATUSTEXT` |

Things to know when reading the live view:

- **The drift line compares GPS with the autopilot's EKF**, because core doesn't publish
  its independent estimate yet. The EKF fuses GPS, so a slow spoof that it accepts won't
  show as drift. Once core publishes its estimate, this should switch to it.
- **The summary line is what autonomy has decided**, not proof the aircraft has done it.
  The mode in the top bar shows what the autopilot is actually doing.
- **The behavior only updates on a transition.** A dashboard that connects after one has
  happened shows "hasn't reported an autonomy state yet" until the next.
- **GNSS and link stay "No data" until core is running** and publishing those scores.

## Simulation

`internal/sim` generates telemetry in the browser, and the buttons under "Demo: inject
a problem" trigger each state. It imitates core's thresholds and behaviors so that the
interface can be demonstrated without a vehicle, but it doesn't run core's detectors.

## Layout

```
cmd/app/           browser code (syscall/js): map canvas, Fly and Plan panels
cmd/serve/         serves the page and, with -mavlink, runs the bridge
internal/model/    plan, geometry and health rules; no browser dependencies
internal/sim/      simulated flight and injected problems
internal/mavlink/  MAVLink frames and the few messages used; no dependencies
internal/bridge/   MAVLink over UDP to telemetry stream and mission upload over HTTP
web/               page and styles; `make build` adds app.wasm and wasm_exec.js
```

The map has no internet tiles, only a metric grid, so it works offline.
