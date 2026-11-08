# Trust Issues — Mission Control

Operator dashboard, written in Go and compiled to WebAssembly. It runs entirely in the
browser and doesn't share a build system with `../core`.

- **Fly**: live map of the planned route, GPS position and independent position
  estimate, with a drift line when those two disagree by 10 m or more. A panel shows
  the health of each subsystem in plain language.
- **Plan**: click the map to add waypoints, drag to move them, and set altitudes. The
  plan is saved in the browser's local storage. Shift-click a waypoint to remove it;
  Shift-clicking empty space or home does nothing.

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
and a plan with a leg over 5 km, including the return home, which usually means it was
made for somewhere else.
The dashboard doesn't arm or fly the vehicle: start the mission from your ground
control, e.g. in MAVProxy: `mode guided`, `arm throttle`, `mode auto`,
`long MISSION_START`. The Fly tab shows the last plan uploaded from the page, or the
plan being edited if there hasn't been one.

The API has no authentication, and neither does MAVLink itself until #4 is done. It
only answers to the dashboard's own address and localhost, which stops other websites
reaching it through the browser, but keep both on localhost or a trusted network. To
open the dashboard from another machine, use `-addr 0.0.0.0:8080 -public-host <its IP>`.
Remote clients can view telemetry, but mission uploads are localhost-only by default
(based on the connection's client address, not proxy headers). To permit uploads
from other machines, also pass `-allow-remote-upload`. This adds no authentication:
anyone on that network can replace the disarmed vehicle's mission. Enable it only
on a trusted network. Host and browser Origin checks still apply with this flag.

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

## Map and critical alerts

The cockpit-inspired interface has a navigation rail, a large map, a telemetry dock,
and a live heading dial (not a flight control). Choose **Satellite** (Esri World Imagery)
or **Street** (OpenStreetMap); no API key is needed for these endpoints. Satellite
imagery is explicitly labeled as a basemap, not a live camera feed. Tile requests go
directly to the selected provider and reveal the viewed area to that provider.
Only visible tiles are requested, with a bounded in-memory cache.
If the network or provider is unavailable, the metric grid remains usable for planning.
Map attribution stays visible in both Fly and Plan.

Critical subsystem assessments turn the console red and show a persistent warning
banner, even while planning. Yellow/caution adds mild RGB separation and fine digital
line jitter; red/critical turns up the channel splitting, static, and HUD ghosting.
A single WebGL pass is capped at 20 fps and 1024×720, rather than using expensive SVG
turbulence or displacement. No liquid warping, large tear blocks or rapid strobes.
The effect stops entirely when healthy, hidden, reduced-motion, or switched off;
unsupported graphics/CORS falls back to the ordinary map and cheap CSS warnings.
Base map redraws are capped at 30 fps rather than tracking 60/120 Hz displays.
This is a labeled cosmetic alert,
not corrupted telemetry. **Glitch FX on** toggles these effects
without muting the alarm or hiding the critical banner. Reduced-motion preferences
disable them automatically. Canvas layout, telemetry, and pointer coordinates are
unchanged; a pointer-transparent GPU canvas supplies only the cosmetic effect.
After the first click or keypress, a sourced emergency-siren recording loops during critical
states. Use **Alarm sound on** to mute it; visual warnings stay active. Browser autoplay
restrictions may require **Enable alarm**. Sound stops on recovery, stale/no-contact
assessments, reset, mute, or when the page is hidden. No thresholds or autonomy behavior
are changed: trust below 40% and battery below 15% use the existing critical rules.
The recording is bundled locally as `web/audio/critical-siren.mp3`, with attribution
and license details in `web/audio/README.md` and the console's **Sound credits** link.
There are no synthesized oscillator beeps. If the file cannot load, visual warnings
continue and **Retry alarm audio** allows another attempt.

Visual styling (`web/mission.css`, `web/glitch.css`), GPU effects (`web/glitch.js`), alerts (`web/mission.js`), the basemap
(`web/map-tiles.js`), and read-only instrument mirrors (`web/cockpit.js`) are isolated
from the telemetry and planning code. Canvas resizing follows panel/banner layout changes.

Optional presentation tests (Node.js, no dependencies):

```
node --test web/mission.test.cjs
```
