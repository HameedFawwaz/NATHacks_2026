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
make run    # builds, then serves http://127.0.0.1:8080
make test
```

## Data source

**Everything on screen is simulated for now.** `internal/sim` generates telemetry in
the browser, and the buttons under "Demo: inject a problem" trigger each state. It
imitates core's thresholds and behaviors so that the interface can be built and
demonstrated, but it doesn't run core's detectors. The top bar shows a "Simulated
data" badge.

Connecting a vehicle means filling `model.Snapshot` from a MAVLink bridge instead of
from the simulator. Plan upload needs that bridge too.

| Dashboard field | Source | Available from core? |
|---|---|---|
| Position, altitude, speed, heading, satellites, mode, armed | `GLOBAL_POSITION_INT`, `GPS_RAW_INT`, `HEARTBEAT` | Yes |
| GNSS score | `TRUST_NAV` | Yes |
| Link score | `TRUST_LNK` | Yes |
| Battery | `SYS_STATUS` | From the autopilot; core doesn't decode it yet |
| Behavior | `STATUSTEXT` carries transitions | Only as text; there is no structured field |
| IMU and motor scores | ? | No. Core only publishes a combined `TRUST_HW` |
| Independent position estimate | ? | No. PR #10 computes one but doesn't publish it |

## Layout

```
cmd/app/         browser code (syscall/js): map canvas, Fly and Plan panels
cmd/serve/       static file server for local use
internal/model/  plan, geometry and health rules; no browser dependencies
internal/sim/    simulated flight and injected problems
web/             page and styles; `make build` adds app.wasm and wasm_exec.js
```

The map has no internet tiles, only a metric grid, so it works offline.
