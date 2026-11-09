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
API). `nav/` (GNSS spoofing detection), `autonomy/` (decision layer), and `crypto/`
(link signing) are also implemented — see below. `link/` (lost-link detection/failover)
is still empty, scoped to its own issue.

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
rendering. Until #16 publishes `EST_N`/`EST_E`, drift remains the labelled EKF fallback.

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
controls. Its estimate source was `ekf` throughout, not core's still-unpublished
estimate. On the controls no autonomy transition was published, so the behavior field
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

`core_main` now calls the real decision loop using the nav detector's measured trust.
The link and hardware producers are still separate work. For isolated scenarios,
`tools/autonomy_runner` reads scores from stdin (`nav 0.1`, `link 0`, `hw 1`,
`crypto 1`) and runs the same decision loop.

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
**Not yet verified against our SITL setup**: confirm the handshake and signature
verification before flight; do not silently fall back to an unsigned autopilot link.

## Dashboard

The operator dashboard is a separate project/framework and lives outside `core/`
(see `../dashboard`). It consumes trust scores and telemetry published by this
codebase over MAVLink; it does not share a build system with it.
