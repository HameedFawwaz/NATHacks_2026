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
API). `nav/` (GNSS spoofing detection), `link/` (lost-link detection), `autonomy/` (decision
layer), and `crypto/` (link signing) are also implemented — see below.

## GNSS spoofing detection (`nav/`)

Maintains dead-reckoned position estimates (Kalman filters propagating accelerometer,
gyro and barometer measurements between GPS anchors) and cross-checks incoming GPS
against them. The default IMU rotation uses the flight controller's attitude, so the
estimate is not wholly GPS-independent; see the limitations below. Six checks (jump,
short- and long-horizon KF divergence, raw-GPS-vs-EKF divergence, timestamp/replay, HDOP
abnormality) each produce a sub-score in `[0,1]`, fused via `min()` so one unambiguous red
flag can't be averaged away, then published as `nav_trust` with asymmetric slew limiting
(drops instantly, recovers slowly). See `include/core/nav/nav.h` for the public API,
`include/core/nav/gnss_anomaly.h` for the six checks, and `core/tests/test_gnss_anomaly.c`
/ `test_nav_kf.c` for fully synthetic (no SITL needed) scenario and filter-math tests.

`core_main` is the real integration loop: connect → detect → autonomy → publish,
against a live flight controller. **It now sends autonomy commands**, including GNSS
source-set changes and, if failover fails, landing commands. It is not a passive
monitor; use `tools/mavlink_monitor` for read-only observation. Configure and validate
the fallback source set in SITL before connecting this loop to hardware.

## Published position estimate (`EST_N` / `EST_E`)

`core_main` publishes core's dead-reckoned horizontal position with the trust scores
(every 200 ms) as two `NAMED_VALUE_FLOAT`s from component 191:

| Name | Value |
|---|---|
| `EST_N` | metres north of the autopilot's `HOME_POSITION` |
| `EST_E` | metres east of the autopilot's `HOME_POSITION` |

The pair is always sent together, `EST_N` first. A float can't hold latitude and
longitude precisely, but it can hold local metres. To get back to latitude and
longitude, use the equirectangular projection on a 6,371,000 m sphere about home:
`lat = home_lat + deg(N / R)`, `lon = home_lon + deg(E / (R cos(home_lat)))`. That is
exactly the dashboard's `model.FromLocal`, and core projects with its inverse
(`core_nav_local_from_geo`).

**Validity is signalled by absence.** Nothing is sent unless every one of these holds,
so a consumer must treat a pair it hasn't received recently (the dashboard uses 3 s)
as no estimate, never reuse an old one, and never assume zero:

- the autopilot's `HOME_POSITION` is known (core requests it at 1 Hz; ArduCopter
  moves home when it arms, and core re-reads it every tick);
- the filter has been anchored to a trusted GPS fix;
- an IMU sample arrived within the last 500 ms;
- if the IMU stream went stale, the filter has re-anchored since it came back (the
  first sample back is integrated across the whole gap);
- an attitude source is available (fresh FC `ATTITUDE`, or core's filter with yaw
  seeded);
- no more than 15 s of pure dead reckoning since the last anchor;
- both values are finite and within ±100 km.

`core_main` prints the reason once a second (`[nav_est] no_home (not published)`,
`imu_stale`, `await_reanchor`, `dr_expired` and so on).

**What it is.** The source is the long-horizon KF set. Its origin is its last trusted
GPS fix, not home: the published value is that anchor projected relative to home,
plus the KF's metre offset from it. In normal flight the anchor is refreshed every
8 s. While the long-horizon drift check sees GPS diverging, re-anchoring stops, so the
estimate keeps dead-reckoning from the last fix it trusted rather than following the
spoof. After 15 s of that it is withdrawn, because without IMU bias tracking the error
grows quickly (see below). The short-horizon set re-anchors every 1.5 s and absorbs a
slow spoof, so it isn't used.

**What it is not.** It is not GPS-independent. Every anchor is a GPS fix (a spoof
slower than the drift check's ~1.25 m/s limit is absorbed at each re-anchor), and by
default the IMU is rotated by the flight controller's `ATTITUDE`, which EKF3 slowly
corrects with GPS velocity. Altitude is not published.

The SITL harness requests `SIM_STATE` at 20 Hz and scores received estimate pairs
against its simulator physics position and velocity, never the GPS-influenced EKF.
It carries the latest simulator position forward using simulator velocity only when
its arrival age is at most 200 ms, reports the median and maximum arrival age, and
does not score older samples. Arrival age does not measure source-to-receiver
latency; constant-velocity extrapolation and separate telemetry links also introduce
uncertainty, particularly during maneuvers. Earlier reported position-error numbers
used EKF velocity and should not be treated as independent truth.

In a fresh 60 s per scenario run on ArduCopter SITL `b720755` (Apple Silicon),
all received pairs were scored with fresh simulator truth and none was rejected:

| Scenario | Scored pairs | Error median / p95 / max | Truth arrival age median / max |
|---|---:|---|---|
| Honest hover | 295 | 0.24 / 0.82 / 1.54 m | 25 / 55 ms |
| Honest 10 m/s cruise square | 296 | 1.49 / 2.15 / 2.49 m | 23 / 55 ms |
| ~1.5 m/s spoof ramp | 97 | 4.24 / 7.79 / 7.80 m | 25 / 53 ms |

Both controls stayed at nav trust 1.0 with no autonomy transitions. During the
spoof, trust fell below 0.5 at 16.2 s and `GNSS_FAILOVER` followed; the bridge
selected core until 22.93 s and then the labelled EKF fallback. Its largest
reported-GNSS-to-core estimate separation was 36.61 m, calculated from API samples,
not a browser observation. Core's log first reported `dr_expired` at an anchor age
of 15.25 s. A 15 s cutoff is a heuristic, not a guaranteed error bound.

### Combined live dashboard check

`sitl/run_dashboard_e2e_scenario.py` launches a fresh local SITL, maintains real
primary-GCS heartbeats, flies an honest hover, then injects a slow GPS ramp. It
checks the real estimator/autonomy and read-only rendered dashboard together,
with map tiles blocked. Build core, the bridge and WASM first:

```sh
cmake -S core -B core/build && cmake --build core/build --parallel 4
(cd dashboard && make build && go build -o ../core/build/dashboard-serve ./cmd/serve)
/path/to/venv/bin/python core/sitl/run_dashboard_e2e_scenario.py \
  --ardupilot /path/to/ardupilot --log-dir core/build/dashboard-e2e
```

Requires `pymavlink`, Node 22+ and installed Chrome/Chromium. Defaults use instance
18, core UDP 15465, dashboard UDP 15466 and HTTP 18188; flags can move them, and
occupied ports are refused. Only this run's child processes are stopped. Each
browser uses a fresh profile and rejects mutating requests. Screenshots, rendered
state JSON and measured results stay in the log directory, not in source control.
The displayed planned route is the dashboard's default local plan; the vehicle
hovers in GUIDED, rather than executing that drawn route. The attack and operator
heartbeats continue during browser capture.

In the final combined run, honest-hover error was median 0.27 m, p95 1.22 m,
maximum 1.48 m, with nav/link trust 1.0. Inspected live screenshots showed the
offline planned route, a 21 m reported/core disagreement with a GNSS warning,
then a critical GNSS panel and explicitly labelled EKF fallback. The slow spoof
caused actual optical-flow fusion and `GNSS_FAILOVER`. During that attack the
core estimate's maximum measured error reached 10.71 m before withdrawal
(10.94 m in an earlier combined run):
the expiry heuristic is not an accuracy guarantee. This is SITL/browser evidence,
not physical flight, firmware or radio validation.

The GNSS-only harness's dashboard observations are the estimate selected and
delivered by the bridge's telemetry API, not evidence of browser rendering.
In an earlier `imu_gap`
scenario (SITL frozen for 2 s mid-hover), core logged
`imu_stale` then `await_reanchor` and published nothing new until its filter
re-anchored, 1.25 s after the freeze ended. The two pairs sent in the first 0.5 s of
the freeze arrived in a burst when SITL resumed. `tests/test_nav_estimate.c` covers
the same gates synthetically.

## End-to-end GNSS scenarios

From the repository root, using a Python environment with `pymavlink` and a built
ArduCopter SITL checkout:

```sh
cmake -S core -B core/build
cmake --build core/build --parallel 4
(cd dashboard && make build && go build -o ../core/build/dashboard-serve ./cmd/serve)
/path/to/venv/bin/python core/sitl/run_gnss_e2e_scenarios.py \
  --ardupilot /path/to/ardupilot --duration 90 --log-dir core/build/gnss-e2e all
/path/to/venv/bin/python core/sitl/test_gnss_e2e_scenarios.py
```

Every scenario starts a fresh SITL, real detector/autonomy, and live dashboard
bridge. `fast`, `slow`, and `random` inject GPS glitches inside the autopilot;
`control` hovers honestly and `cruise` flies an honest square at 10 m/s. The script
records observed minimum trust, detection time (below 0.5), autonomy transitions,
maximum true displacement from the starting point, and actual dashboard telemetry
samples in `results.json`. A missed spoof is recorded as a useful result; a control
false alarm, missing detector/dashboard telemetry, or failed setup exits nonzero.

Default test ports are core UDP 14555, dashboard UDP 14556 and dashboard HTTP 8081.
Use `--instance`, `--core-port`, `--dashboard-port`, and `--dashboard-http-port` to
avoid an existing run. The script refuses occupied ports and only stops its own
children. An existing dashboard may be used with `--dashboard-url` if it listens on
`--dashboard-port`. The test reads its telemetry API; it does not validate browser
rendering. Each result also scores core's `EST_N`/`EST_E` against simulator truth
(`core_estimate`, plus `takeoff_estimate` for the climb), and records the dashboard's
estimate source and drift line. The `imu_gap` scenario freezes SITL for 2 s to check
the estimate is withheld across a stream outage (see above).

### Review results (2026-11-09)

Two complete runs passed on local ArduCopter SITL (checkout `b720755`, Apple Silicon
macOS), each using fresh state per scenario, seed 1 and 90-second observation windows.
The second run used the reviewed nav calibration/input guards. Its observed results:

| Scenario | Detect below 0.5 | Lowest nav trust | Behavior entered | Maximum true displacement |
|---|---:|---:|---|---:|
| Fast ramp (~4.5 m/s) | ~4.8 s | 0.000 | GNSS_FAILOVER | 4.1 m |
| Slow ramp (~1.5 m/s) | ~16.0 s | 0.000 | GNSS_FAILOVER | 23.3 m |
| Random route | ~1.6 s | 0.000 | GNSS_FAILOVER | 4.5 m |
| Honest hover | never | 1.000 | none | <0.1 m |
| Honest cruise square | never | 1.000 | none | 168.9 m (intended travel) |

Detection times are polled telemetry observations, not exact detector transition
times; an autonomy STATUSTEXT can arrive before the next periodic trust sample.
Both control runs had no false alarms. The live dashboard API received trust dropping
to zero and `GNSS_FAILOVER` for all three attacks, and trust 1.0 throughout both
controls. Its estimate source was `ekf` throughout: core did not publish an estimate
until #16 (see "Published position estimate" for results since). On the controls no autonomy transition was published, so the behavior field
remained unreported rather than inventing a NOMINAL message.

A separate fresh SITL run verified that the dashboard refuses start before a
confirmed upload, starts the uploaded mission through the normal prearm checks and
actually takes off, and confirms GPS jump/drift/jam, motor fault, and All clear.
Core's six test suites passed normally and under ASan/UBSan. Dashboard native/race,
WASM vet/build, and all 13 JavaScript tests passed; flight-control regressions were
also repeated 30 times under the race detector. The interactive MAVProxy launcher
was syntax/help-checked, not exercised end-to-end in this review. No real hardware
or browser-rendering validation is claimed.

**Known limitations**:

- **The KFs rotate IMU data by the flight controller's ATTITUDE**, not core's own
  attitude filter (`kf_attitude_from_fc`, on by default; core's filter is the fallback
  whenever ATTITUDE is stale). Core's filter has no magnetometer and no in-flight gyro
  bias tracking, and drifted a few degrees in tilt and heading over a minute of
  aggressive flight; at 30 deg of tilt each degree is ~0.17 m/s^2 of phantom
  acceleration, which on its own false-alarmed the long-horizon check on an unspoofed
  square route (`sitl/run_gnss_e2e_scenarios.py cruise`). The trade: attitude is the
  part of the flight controller's state a GNSS spoof barely moves (tilt and heading are
  held by gyro, accel and compass), but it isn't fully GPS-independent, since EKF3 does
  use GPS velocity to correct tilt slowly. Position and velocity are still compared only
  against core's own dead reckoning.
- **GPS staleness uses real time as well as the IMU-paced clock.** The detector's
  clock follows IMU timestamps, so if the IMU and GPS streams both stopped while the
  link stayed up, GPS age used to freeze and `nav_trust` stayed healthy. A stale IMU
  alone still only sets `IMU_LOST` and neutralises the KF checks; it does not lower
  `nav_trust` by itself.
- Earlier false positives during maneuvering, once blamed on a ~4 Hz IMU, were core's
  attitude filter reading tilt with the wrong sign, then lagging through every
  acceleration (see `attitude_filter.c`). Wired straight to SITL's SERIAL1 (no MAVProxy
  in between), `SCALED_IMU2` arrives at the requested 50 Hz.
- **Minimum detectable sustained-drift rate**: re-anchoring to GPS is now gated on the
  filter's own divergence score staying at ceiling (see `gnss_anomaly.c`), so a slow spoof
  no longer gets silently absorbed every re-anchor interval the way it used to — but the
  detectable rate is still bounded by `warn_m / horizon_ms`. With the current long-horizon
  defaults (10m / 8s) that's **~1.25 m/s**: a spoof patient enough to drift slower than
  that, held indefinitely, remains undetectable by the KF-based checks alone. (The
  raw-GPS-vs-EKF check can still catch it — see the caveat on that check above — but only
  if the attacker fooled our telemetry relay without also fooling the flight controller's
  own GPS reception.) Closing this gap for good needs real IMU bias estimation, not just
  threshold tuning.

## Autonomy

`autonomy/` turns trust scores into a response. The table of behaviors is at the top of
`include/core/autonomy/autonomy.h`. Each transition goes to stderr, to a STATUSTEXT, and
to a CSV log.

`core_main` now calls the real decision loop using the nav detector's and the link
monitor's measured trust (see [End-to-end link scenarios](#end-to-end-link-scenarios)).
The hardware producer is still separate work. While the vehicle's own heartbeat is more
than 3 s old, autonomy holds its decisions: armed state and mode are unknown then, every
producer fed by the autopilot goes quiet with it, and nothing commanded could arrive. After
the heartbeat returns, autonomy waits another 3 s for producers to catch up; a persistent
attack is acted on after that delay. For
isolated scenarios, `tools/autonomy_runner` reads scores from stdin (`nav 0.1`, `link 0`,
`hw 1`, `crypto 1`) and runs the same decision loop.

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

## Lost-link detection (`link/`)

`link/link_monitor` judges the operator's control link and publishes `link_trust`, which
autonomy turns into a hold and then a return home. `core_main` runs it every tick. See
`include/core/link/link_monitor.h` for the states and the full reasoning.

The companion computer isn't on the operator's link, so it measures it indirectly:

- **Operator heartbeats.** ArduPilot forwards the ground station's heartbeats to every
  other link. The monitor counts the fraction missing over 10 s and how long the link has
  been silent. The operator is `SYSID_MYGCS`'s default, sysid 255; change
  `operator_gcs_sysid` in the connection config if yours differs.
- **Radio data, to tell jamming from a clean loss.** ArduPilot never forwards
  `RADIO_STATUS`, but with `RSSI_TYPE=5` it puts the telemetry radio's RSSI in
  `RC_CHANNELS`, which `core_main` requests. Silent while the radio still hears strong RF
  is read as jamming; silent with the signal gone is a loss. If the radio is wired to the
  companion computer instead, its `RADIO_STATUS` noise floor is used, which is the
  stronger signal.

| State | When | `link_trust` |
|---|---|---|
| `UNKNOWN` | no operator heard yet, or the autopilot itself is silent | not reported |
| `NOMINAL` | heartbeats arriving as expected | about 1 |
| `DEGRADED` | heartbeats missing, weak signal, or interference while connected | below 0.7 |
| `LOST` | silent for 5 s, no sign of jamming | 0 |
| `JAMMED` | silent for 5 s while the radio shows interference lasting 2 s | 0 |

Each state change goes out as a `LINK <STATE>: <reason>` STATUSTEXT, and the state and
loss are published as `LNK_STATE` and `LNK_LOSS` alongside `TRUST_LNK`.

**When autonomy acts.** Autonomy acts when `link_trust` falls below 0.4, which happens
when the operator can't control the vehicle for more than 5 s: either no heartbeat for 5 s,
or more than 30% of heartbeats lost over the last 30 s, continuously for 5 s. Short of that,
trust stays at 0.45 or above, so the link can be `DEGRADED` without autonomy acting.

The 30% rule is judged over 30 s because at 1 Hz ten heartbeats can't measure 30%: 10%
random loss read as over 30% often enough to act. Even 30 samples can't fully separate a
link near the threshold from one over it. In simulation, over five minutes of random loss:

| Average loss | Autonomy acts, by the 30% rule (20 runs) |
|---|---|
| 10% | never |
| 20% | 7 of 20 |
| 25% | 16 of 20 |
| 35% or more | every run, after about 35 to 50 s |

Random loss also sometimes drops four heartbeats in a row, which is 5 s without contact
and acts by the silence rule. `loss_act` and `loss_act_after_ms` in
`core_link_config_default()` are the knobs.

Brief dropouts don't trip it: a full dropout of up to 3 s never acts, the first 3 s of
silence cost nothing, trust drops at once but climbs back at 0.1/s, and the state only steps back down
after holding for 3 s. Steady loss doesn't make it bounce either: the link becomes
`DEGRADED` below 0.7 trust but is only `NOMINAL` again at 0.85 with a clean 30 s record. So
after an outage, trust (which autonomy acts on) recovers in about 10 s while the label
stays `DEGRADED` for about 30 s.

Limits:

- **Jamming versus loss is a heuristic.** With RSSI only, the assumption is that a jammed
  radio still reports strong RF energy. That holds in the SITL scenario because the
  scenario is built on it; check it against the real radio. Interference must last 2 s
  to count, because a returning signal shows strong RF a moment before its next heartbeat.
- **The GCS heartbeat rate is configured, not learned**: `heartbeat_interval_ms`, 1 s by
  default, which is what MAVProxy, QGroundControl and Mission Planner send. From the
  companion computer a GCS that slows down looks exactly like one losing heartbeats, and
  learning the rate kept turning that into wrong verdicts. So a GCS slower than configured
  reads as loss and must be configured; a faster one still has total loss caught by
  silence, but partial loss shows less. Partial loss is measured once 10 intervals have
  passed; silence covers the first 10 s. After an outage long enough to be `LOST`, loss is
  counted again from the operator's return, so the outage isn't acted on twice.
- **The noise floor is calibrated, then follows the link slowly.** It starts as the median
  of 10 samples while heartbeats are arriving, then follows with a time constant of about a
  minute, so a jammer that ramps up over tens of seconds is caught but one that creeps up
  over many minutes may not be. Raised noise that costs at most 5% of heartbeats over 30 s,
  for 60 s, is taken as the new normal and calibrated again. A sample is only learned once
  a heartbeat has arrived after it, so a link going silent, perhaps jammed, can't feed the
  baseline.
- **`RC_CHANNELS.rssi` of 0 is read as "no radio"**, which is what ArduPilot sends with
  `RSSI_TYPE=5` and no radio reporting. Other `RSSI_TYPE`s report the RC receiver's RSSI
  instead, which the monitor can't tell apart; use 5.
- **Forwarding needs an ordinary link.** ArduPilot only forwards heartbeats to active links
  that aren't marked private (`MAV_OPTIONS`/`SERIALn_OPTIONS`).
- **No operator ever heard means no verdict.** A drone that takes off without its ground
  station connected isn't monitored, the same as autonomy's rule for producers that have
  never reported.
- **No backup link.** The issue allows switching to one, but the hardware has none, so the
  response is left to autonomy.
- **An autopilot outage isn't blamed on the operator.** While the autopilot is silent the
  link is `UNKNOWN`. When it's back, an operator heard before is judged at once, with its
  silence timed from the autopilot's return: one still there stays trusted, and one gone
  is `LOST` 5 s later. Operator heartbeats missed in the 2 s before the autopilot is
  treated as late still count as loss, so trust dips to about 0.8, still trusted.

### Scripted jamming in SITL

`sitl/link_jam_proxy.py` stands in for the telemetry radio: it relays the operator's link,
drops packets, and reports `RADIO_STATUS` to the autopilot with chosen RSSI and noise.
`sitl/run_link_scenario.py` runs it against SITL and `core_main` and checks each case:

```
python3 sitl/run_link_scenario.py --ardupilot /path/to/ardupilot --core-main build/core_main
```

It checks that a steady link stays `NOMINAL`, that 2 s dropouts never reach `LOST` or
untrusted, that 30% loss is `DEGRADED`, that jamming is `JAMMED` and not `LOST`, that a
clean loss is `LOST` and not `JAMMED`, and that each recovers without bouncing. The vehicle
stays on the ground. Needs `pymavlink`. `--instance`, `--core-port`, `--proxy-fc-port` and
`--proxy-gcs-port` move it off instance 0's ports, so it can run beside another simulator.

To try it by hand with MAVProxy as the operator, start the proxy and type a profile
(`nominal`, `degraded`, `jam`, `lost`) to switch:

```
python3 sitl/link_jam_proxy.py
arducopter ... --defaults <copter.parm>,sitl/link_jam.parm \
  --serial1=udpclient:127.0.0.1:14551 --serial2=udpclient:127.0.0.1:14560
./build/core_main 14551
mavproxy.py --master=udpout:127.0.0.1:14562
```

## End-to-end link scenarios

`sitl/run_link_e2e_scenarios.py` flies ArduCopter SITL with `core_main` (the real link
monitor, nav detector and autonomy together), the live dashboard bridge, and the operator's
link through `link_jam_proxy`. Nothing types a trust score: the proxy drops the operator's
real heartbeats and reports RSSI and noise as a SiK radio would, `core_main` judges the
link from what ArduPilot forwards, and autonomy commands the vehicle. ArduPilot's own GCS
failsafe is off (`sitl/link_e2e.parm`), so any hold or return comes from core. Each
scenario takes off on a fresh SITL, flies 60 m out at 20 m from a separate link (sysid
200, like a mission), then attacks the operator's link:

| Scenario | Injected | Checked |
|---|---|---|
| `jam` | noise up, RSSI high, packets gone over 8 s | settles as `JAMMED`, `BRAKE`, `RTL` after the 30 s hold, lands at home |
| `lost` | signal gone over 2 s | `LOST` (never `JAMMED`), then the same hold and return |
| `dropouts` | 4 × 2 s full dropouts | never `LOST`/`JAMMED`, trust never below 0.4, no autonomy action, still `GUIDED` |
| `recover` | jamming until autonomy holds, then the link returns | back to `NOMINAL` before the hold runs out, no return, mode left in `BRAKE` for the operator |
| `companion_outage` | core's own link to the autopilot cut for 8 s, operator fine | no autonomy action at all, still `GUIDED` |

Every scenario also checks that navigation stayed trusted (no `GNSS_FAILOVER` or
`SAFE_LAND`) and that no ArduPilot failsafe fired; attack scenarios check the
`autonomy_transitions.csv` rows and that the dashboard showed the measured `TRUST_LNK` and
the behaviors. `companion_outage` puts `core_main` behind a pausable UDP relay.

```sh
cmake -S core -B core/build && cmake --build core/build --parallel 4
(cd dashboard && go build -o ../core/build/dashboard-serve ./cmd/serve)
/path/to/venv/bin/python core/sitl/run_link_e2e_scenarios.py --ardupilot /path/to/ardupilot \
  --instance 14 --core-port 15065 --proxy-fc-port 15066 --proxy-gcs-port 15067 \
  --dashboard-port 15068 --dashboard-http-port 18065 --relay-port 15069 \
  --log-dir core/build/link-e2e all
```

It refuses to start on ports already in use. Results, events and transition rows go to
`results.json`, logs to one directory per scenario.

### Results (2026-11-09, ArduCopter 4.5 SITL, macOS arm64)

Seconds after injection; each line is one run.

| Scenario | Link verdict | `LINK_HOLD` | `BRAKE` confirmed | `LINK_RETURN` | `RTL` confirmed | Landed, from home |
|---|---|---|---|---|---|---|
| `jam` | `JAMMED` 10.5 | 10.7 | 10.9 | 40.5 | 40.7 | 91.1 s, 0.0 m |
| `lost` | `LOST` 5.2 | 5.4 | 5.6 | 35.1 | 35.3 | 85.1 s, 0.0 m |

- In `BRAKE` the vehicle held position before the return; both holds lasted about 30 s,
  then `RTL` brought it back 60 m and landed it at home.
- `dropouts`: `NOMINAL -> DEGRADED -> NOMINAL`, minimum `TRUST_LNK` 0.45, no autonomy
  transition, still `GUIDED`.
- `recover`: hold at 10.7 s, link restored at 10.9 s, autonomy `NOMINAL` 13.0 s later
  (trust climbs at 0.1/s to 0.7, then autonomy's 5 s recovery hold); no return, mode left
  in `BRAKE`. This run briefly reported `LOST: interference unconfirmed` before `JAMMED`:
  the interference confirmation and silence thresholds can coincide. Both verdicts
  correctly triggered the same safe hold.
- `companion_outage`: after an 8 s cut, no autonomy transition, still `GUIDED`. Before
  the reconnect and settling fixes, the same test incorrectly entered `GNSS_FAILOVER`.
- Ground-link scenario passed; the 30% random-loss phase briefly reached `LOST` after
  consecutive dropped heartbeats, then recovered. A 90 s fast GNSS ramp triggered
  `GNSS_FAILOVER` with EKF source set 2 active; a 90 s unspoofed control had no false
  alarm. Those GNSS runs had no operator heartbeat, so the link was `UNKNOWN` and they
  do not demonstrate concurrent active-link monitoring during GNSS spoofing.

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
ground truth, so the EKF-divergence check reliably fires in this specific harness. **This
is a property of the harness, not a durability guarantee**: a real spoof that fools the
GPS receiver itself would make `GLOBAL_POSITION_INT` agree with the spoofed
`GPS_RAW_INT` too, and the EKF-divergence check would show nothing — in that case
detection rests entirely on the IMU-based KF checks (`dr=`/`drift=`), which never ingest
GPS at all.

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

It talks directly to the vehicle (arm/mode/takeoff/position commands) on a separate
link from `core_main`/`gnss_spoof_proxy`. Unlike the proxy, `core_main` now also sends
autonomy commands: an anomaly during this flight can change sources or request landing.

## Link signing (operator <-> drone authentication)

The link is protected with **MAVLink 2 message signing**: every frame carries a
SHA-256 signature over the key, header, payload, link id and a timestamp. With
signing on, `core_mavlink_conn_*` drops any inbound frame that is unsigned,
signed with the wrong key, tampered with, or replayed (old/repeated timestamp),
and signs everything it sends. Latency cost is a few microseconds per message
(`test_link_signing` prints the measurement).

Signing authenticates but does **not encrypt**: telemetry is still readable by
an eavesdropper, but commands and alerts cannot be forged or replayed.

### Provisioning a key

For MAVProxy/ArduPilot, derive the key from the **same nonempty ASCII
passphrase** used in MAVProxy's `signing setup <same passphrase>`. Only printable
ASCII bytes `0x21..0x7e` are accepted: no whitespace or control characters.
MAVProxy splits its command line on whitespace and uses only the first argument,
so a multiword passphrase would otherwise produce mismatched keys.

```sh
./build/tools/signing_keygen signing.key --from-passphrase
# Enter the passphrase on stdin (one line); do not pass it as a command argument.
```

This writes SHA-256 of the passphrase bytes, matching MAVProxy exactly. The
trailing line ending is stripped. Use a strong, unique passphrase and a private
terminal: stdin is not hidden when entered interactively.

Random keys are for links where **both ends are our own code**:

```sh
./build/tools/signing_keygen signing.key              # random key, hex file, mode 0600
./build/tools/signing_keygen other.key --print-hex     # also expose raw key on stdout
```

`--print-hex` is not a MAVProxy provisioning command: MAVProxy hashes its
passphrase argument and cannot take a raw hex key. It can be combined with
`--from-passphrase`, but avoid exposing key material unnecessarily.

`signing_keygen` refuses to overwrite an existing key. `*.key` is gitignored -
never commit a key. Copy the same file to the ground station and the companion
computer over a trusted channel (physical media or `scp`; not over the
unsigned link itself). Both ends must hold the same key.

### Enabling it in code

```c
core_mavlink_conn_config_t cfg = { /* ... as before ... */ };
/* CORE_SIGNING_KEY_FILE=signing.key; a bad configured key must fail closed. */
core_signing_env_result_t result = core_signing_key_load_from_env(cfg.signing_key);
if (result == CORE_SIGNING_ENV_ERROR) {
    return EXIT_FAILURE;  /* in main; include <stdlib.h> */
}
cfg.signing_enabled = (result == CORE_SIGNING_ENV_LOADED); /* UNSET leaves it off */
cfg.signing_link_id = 0;   /* unique per link if you run several */
```

or load a path directly with `core_signing_key_load_file()`. If signing is not
enabled (default for a zero-initialised config), behaviour is unchanged.

Rejections are counted in `core_telemetry_t.link_security`
(`rejected_unsigned`, `rejected_bad_sig`, `rejected_replay`, `last_reject_ms`)
so the dashboard and autonomy layer can treat a rising count as an attack
indicator.

When signing is enabled, core also publishes the rejection counts as MAVLink
`NAMED_VALUE_FLOAT` messages on the trust-score publication loop (5Hz):
`SIG_UNSGN` (unsigned frames), `SIG_BADSG` (invalid signatures), and `SIG_REPLY`
(replayed/stale signed frames). These names fit MAVLink's 10-byte name field.
The values are floats, so all integer counts are exact only through 2^24
(16,777,216); above that, adjacent counts may round to the same float. No
signing counters are published when signing is disabled.

### Rotation

1. Generate a new key with `signing_keygen` (use `--from-passphrase` for MAVProxy).
2. Stop the drone link, copy the new key to both ends, restart both.
3. Delete the old key file.

There is no live rekeying: both ends must switch together. Rotate after any
suspected key exposure and before each event/flight day.

### Flight-controller side

The autopilot must be given the same key so it accepts our signed commands
(and signs its telemetry). ArduPilot stores it via the `SETUP_SIGNING` MAVLink
message; MAVProxy's `signing setup <passphrase>` command does this, deriving
the 32-byte key as the SHA-256 of the ASCII passphrase. Use the passphrase route
above, and provision over a trusted local link, not an exposed unsigned link.
Verified against local ArduCopter SITL during the 2026-11-09 issue #4 audit:
on a protected SERIAL1 MAVLink 2 channel, unsigned and wrong-key mode commands
received no ACK and did not change the disarmed vehicle's mode; the correctly
signed command was accepted and its mode change confirmed. Provisioning used
`SETUP_SIGNING`, the same mechanism as MAVProxy's command. This is simulation
evidence, not a physical-hardware security test. ArduPilot deliberately permits
unsigned traffic on its trusted primary/USB channel and certain radio-status
messages, so do not expose that trusted channel as an authenticated network link.
Validate your hardware's channel configuration before flight, and never silently
fall back to an unsigned autopilot link. Signing still does not provide encryption.

## Dashboard

The operator dashboard is a separate project/framework and lives outside `core/`
(see `../dashboard`). It consumes trust scores and telemetry published by this
codebase over MAVLink; it does not share a build system with it.
